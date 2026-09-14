#!/usr/bin/env python3
"""Relay between `robosim --jsonl` streams and ROS 2 via rclpy.

Two modes:
  * spawn:  python3 robosim_relay.py            (spawns `robosim run --jsonl - --cmd-stdin`)
  * stdin:  robosim run --jsonl - | python3 robosim_relay.py --stdin

Each JSONL line from the sim is a rosbridge-style
{"op": "advertise"|"publish", "topic": ..., "type": ..., "msg": ...}.
/cmd_vel subscriptions are forwarded to the sim's stdin as {"v":..,"omega":..}.
"""

import argparse
import json
import subprocess
import sys
import threading

try:
    import rclpy
    from rclpy.node import Node
    from rosidl_runtime_py.set_message import set_message_fields
except ImportError:
    print(
        "robosim_relay: could not import rclpy/rosidl_runtime_py.\n"
        "Source your ROS 2 environment first, e.g.:\n"
        "  source /opt/ros/<distro>/setup.bash",
        file=sys.stderr,
    )
    sys.exit(1)

MSG_TYPES = {
    "rosgraph_msgs/msg/Clock": ("rosgraph_msgs.msg", "Clock"),
    "tf2_msgs/msg/TFMessage": ("tf2_msgs.msg", "TFMessage"),
    "sensor_msgs/msg/JointState": ("sensor_msgs.msg", "JointState"),
    "sensor_msgs/msg/LaserScan": ("sensor_msgs.msg", "LaserScan"),
    "sensor_msgs/msg/Image": ("sensor_msgs.msg", "Image"),
    "nav_msgs/msg/Odometry": ("nav_msgs.msg", "Odometry"),
}


def load_msg_class(type_str):
    import importlib

    module_name, class_name = MSG_TYPES[type_str]
    return getattr(importlib.import_module(module_name), class_name)


class Relay(Node):
    def __init__(self, sim_stdin):
        super().__init__("robosim_relay")
        self.sim_stdin = sim_stdin
        self.pubs = {}
        self.cmd_sub = self.create_subscription(
            self._twist_type(), "/cmd_vel", self._on_cmd_vel, 10
        )
        self._stdin_lock = threading.Lock()

    @staticmethod
    def _twist_type():
        from geometry_msgs.msg import Twist

        return Twist

    def publisher_for(self, topic, type_str):
        if topic not in self.pubs:
            self.pubs[topic] = self.create_publisher(load_msg_class(type_str), topic, 10)
        return self.pubs[topic]

    def handle_line(self, line):
        op = json.loads(line)
        if op.get("op") != "publish":
            return
        msg_cls = load_msg_class(op["type"])
        msg = msg_cls()
        set_message_fields(msg, op["msg"])
        self.publisher_for(op["topic"], op["type"]).publish(msg)

    def _on_cmd_vel(self, twist):
        if self.sim_stdin is None:
            return
        cmd = {"v": twist.linear.x, "omega": twist.angular.z}
        try:
            with self._stdin_lock:
                self.sim_stdin.write(json.dumps(cmd) + "\n")
                self.sim_stdin.flush()
        except (BrokenPipeError, ValueError):
            pass


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--stdin", action="store_true", help="read JSONL from stdin instead of spawning robosim")
    ap.add_argument("--steps", type=int, default=0, help="pass --steps N to spawned robosim")
    ap.add_argument("--twist", default=None, help="pass --twist to spawned robosim")
    ap.add_argument("--robosim", default="robosim", help="path to the robosim binary")
    args = ap.parse_args()

    proc = None
    if args.stdin:
        sim_stdout = sys.stdin
        sim_stdin = None
    else:
        cmd = [args.robosim, "run", "--jsonl", "-", "--cmd-stdin"]
        if args.steps:
            cmd += ["--steps", str(args.steps)]
        if args.twist:
            cmd += ["--twist", args.twist]
        proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stdin=subprocess.PIPE, text=True
        )
        sim_stdout = proc.stdout
        sim_stdin = proc.stdin

    rclpy.init()
    relay = Relay(sim_stdin)
    spin_thread = threading.Thread(target=rclpy.spin, args=(relay,), daemon=True)
    spin_thread.start()

    try:
        for line in sim_stdout:
            line = line.strip()
            if not line:
                continue
            try:
                relay.handle_line(line)
            except (json.JSONDecodeError, KeyError) as e:
                print(f"relay: skipping bad line: {e}", file=sys.stderr)
    except KeyboardInterrupt:
        pass
    finally:
        relay.destroy_node()
        rclpy.shutdown()
        if proc is not None:
            proc.terminate()

    return 0


if __name__ == "__main__":
    sys.exit(main())
