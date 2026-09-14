# ROS 2 integration

robo-sim publishes `/clock`, `/tf`, `/joint_states`, `/scan`,
`/camera/image_raw` and `/odom`, and subscribes to `/cmd_vel`.
Three ways to connect, in increasing order of fidelity:

## (a) rosbridge_suite (websocket)

Start rosbridge, then run the sim against it:

```sh
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
robosim run --rosbridge ws://localhost:9090
```

The sim advertises and publishes over the rosbridge protocol and listens for
`/cmd_vel` (geometry_msgs/msg/Twist). If the socket is unreachable the sim
prints a warning and keeps running without ROS 2.

## (b) rclpy relay (JSONL)

`robosim run --jsonl -` emits one rosbridge-style JSON object per line;
`ros2/robosim_relay.py` converts them to real rclpy publishers and forwards
`/cmd_vel` back to the sim's stdin.

```sh
source /opt/ros/<distro>/setup.bash
python3 ros2/robosim_relay.py --steps 1000 --twist 0.4,0.2
# or pipe an existing sim:
robosim run --jsonl - | python3 ros2/robosim_relay.py --stdin
```

## (c) Zenoh / native DDS (future)

A native path via `zenoh-bridge-ros2dds` or an rmw-compatible Rust crate is
planned but not implemented. See `docs/roadmap.md`.
