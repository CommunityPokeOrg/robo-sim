use robosim_core::{Simulator, World};
use robosim_ros2::msgs::{self, Twist};
use robosim_ros2::{
    connect_or_fallback, frame_to_messages, BridgeConfig, JsonlTransport, Transport,
};

fn demo_frame() -> robosim_core::Frame {
    let mut sim = Simulator::new(World::demo_scene());
    sim.camera_every_n_steps = 1;
    sim.set_twist(0, 0.3, 0.0);
    sim.step_n(3);
    sim.snapshot()
}

#[test]
fn converter_emits_all_topics() {
    let f = demo_frame();
    let pubs = frame_to_messages(&f, &BridgeConfig::default());
    let expected = [
        ("/clock", "rosgraph_msgs/msg/Clock"),
        ("/tf", "tf2_msgs/msg/TFMessage"),
        ("/joint_states", "sensor_msgs/msg/JointState"),
        ("/scan", "sensor_msgs/msg/LaserScan"),
        ("/camera/image_raw", "sensor_msgs/msg/Image"),
        ("/odom", "nav_msgs/msg/Odometry"),
    ];
    assert_eq!(pubs.len(), expected.len());
    for (p, (topic, ty)) in pubs.iter().zip(expected) {
        assert_eq!(p.topic, topic);
        assert_eq!(p.msg_type, ty);
    }
}

#[test]
fn jsonl_writes_valid_lines() {
    let f = demo_frame();
    let pubs = frame_to_messages(&f, &BridgeConfig::default());
    let mut buf = Vec::new();
    {
        let mut t = JsonlTransport::new(&mut buf);
        t.advertise("/scan", "sensor_msgs/msg/LaserScan").unwrap();
        for p in &pubs {
            t.publish(p).unwrap();
        }
    }
    let text = String::from_utf8(buf).unwrap();
    let lines: Vec<&str> = text.lines().collect();
    assert_eq!(lines.len(), pubs.len() + 1);
    let first: serde_json::Value = serde_json::from_str(lines[0]).unwrap();
    assert_eq!(first["op"], "advertise");
    assert_eq!(first["topic"], "/scan");
    let pub0: serde_json::Value = serde_json::from_str(lines[1]).unwrap();
    assert_eq!(pub0["op"], "publish");
    assert_eq!(pub0["topic"], "/clock");
    assert!(pub0["msg"]["clock"]["sec"].is_number());
}

#[test]
fn quat_for_yaw_pi_over_2() {
    let q = msgs::quat_from_yaw(std::f32::consts::FRAC_PI_2);
    assert!((q.x - 0.0).abs() < 1e-6);
    assert!((q.y - 0.0).abs() < 1e-6);
    let s2 = std::f64::consts::FRAC_PI_4.sin();
    assert!((q.z - s2).abs() < 1e-6);
    assert!((q.w - s2).abs() < 1e-6);
}

#[test]
fn unreachable_rosbridge_falls_back() {
    // Port 1 on loopback should refuse immediately.
    let t = connect_or_fallback(Some("ws://127.0.0.1:1"));
    let mut t = t;
    // NullTransport still works.
    t.advertise("/scan", "sensor_msgs/msg/LaserScan").unwrap();
    assert_eq!(t.published_count(), 0);
}

#[test]
fn twist_deserializes_rosbridge_shape() {
    let v: Twist = serde_json::from_str(
        r#"{"linear":{"x":0.5,"y":0.0,"z":0.0},"angular":{"x":0.0,"y":0.0,"z":0.3}}"#,
    )
    .unwrap();
    assert!((v.linear.x - 0.5).abs() < 1e-9);
    assert!((v.angular.z - 0.3).abs() < 1e-9);
}
