//! `Frame` → ROS message conversion.

use crate::msgs;
use crate::Publication;
use robosim_core::Frame;

/// Frame/topic naming for the bridge.
#[derive(Debug, Clone)]
pub struct BridgeConfig {
    /// Fixed world frame.
    pub map_frame: String,
    /// Odometry frame.
    pub odom_frame: String,
    /// Robot base frame.
    pub base_frame: String,
    /// Lidar frame.
    pub laser_frame: String,
    /// Camera frame.
    pub camera_frame: String,
}

impl Default for BridgeConfig {
    fn default() -> Self {
        Self {
            map_frame: "odom".to_string(),
            odom_frame: "odom".to_string(),
            base_frame: "base_link".to_string(),
            laser_frame: "laser_frame".to_string(),
            camera_frame: "camera_link".to_string(),
        }
    }
}

fn stamp(frame: &Frame) -> msgs::Time {
    let sec = frame.time.floor() as i32;
    msgs::Time {
        sec,
        nanosec: ((frame.time - sec as f64) * 1e9).round() as u32,
    }
}

fn header(frame: &Frame, frame_id: &str) -> msgs::Header {
    msgs::Header {
        stamp: stamp(frame),
        frame_id: frame_id.to_string(),
    }
}

fn tf(
    parent: &str,
    child: &str,
    x: f64,
    y: f64,
    yaw: f32,
    frame: &Frame,
) -> msgs::TransformStamped {
    msgs::TransformStamped {
        header: header(frame, parent),
        child_frame_id: child.to_string(),
        transform: msgs::Transform {
            translation: msgs::Vector3 { x, y, z: 0.0 },
            rotation: msgs::quat_from_yaw(yaw),
        },
    }
}

/// Topics advertised by the bridge, in emission order.
pub const TOPICS: [(&str, &str); 6] = [
    ("/clock", "rosgraph_msgs/msg/Clock"),
    ("/tf", "tf2_msgs/msg/TFMessage"),
    ("/joint_states", "sensor_msgs/msg/JointState"),
    ("/scan", "sensor_msgs/msg/LaserScan"),
    ("/camera/image_raw", "sensor_msgs/msg/Image"),
    ("/odom", "nav_msgs/msg/Odometry"),
];

/// Convert a frame into rosbridge publications.
pub fn frame_to_messages(frame: &Frame, cfg: &BridgeConfig) -> Vec<Publication> {
    let mut out = Vec::with_capacity(TOPICS.len());

    out.push(Publication {
        topic: "/clock".into(),
        msg_type: "rosgraph_msgs/msg/Clock".into(),
        payload: serde_json::to_value(msgs::Clock {
            clock: stamp(frame),
        })
        .unwrap_or_default(),
    });

    // TF tree: odom→base_link (robot pose), plus static sensor mounts.
    let mut transforms = Vec::new();
    if let Some(r) = frame.robots.first() {
        transforms.push(tf(
            &cfg.odom_frame,
            &cfg.base_frame,
            r.pose.position.x as f64,
            r.pose.position.y as f64,
            r.pose.heading,
            frame,
        ));
        transforms.push(tf(&cfg.base_frame, &cfg.laser_frame, 0.0, 0.0, 0.0, frame));
        transforms.push(tf(
            &cfg.base_frame,
            &cfg.camera_frame,
            0.05,
            0.0,
            0.0,
            frame,
        ));
    }
    out.push(Publication {
        topic: "/tf".into(),
        msg_type: "tf2_msgs/msg/TFMessage".into(),
        payload: serde_json::to_value(msgs::TFMessage { transforms }).unwrap_or_default(),
    });

    let js = frame
        .robots
        .first()
        .map(|r| msgs::JointState {
            header: header(frame, ""),
            name: r.joint_names.clone(),
            position: r.joint_positions.iter().map(|v| *v as f64).collect(),
            velocity: r.joint_velocities.iter().map(|v| *v as f64).collect(),
            effort: vec![0.0; r.joint_names.len()],
        })
        .unwrap_or(msgs::JointState {
            header: header(frame, ""),
            name: vec![],
            position: vec![],
            velocity: vec![],
            effort: vec![],
        });
    out.push(Publication {
        topic: "/joint_states".into(),
        msg_type: "sensor_msgs/msg/JointState".into(),
        payload: serde_json::to_value(js).unwrap_or_default(),
    });

    let scan = &frame.scan;
    out.push(Publication {
        topic: "/scan".into(),
        msg_type: "sensor_msgs/msg/LaserScan".into(),
        payload: serde_json::to_value(msgs::LaserScan {
            header: header(frame, &cfg.laser_frame),
            angle_min: scan.angle_min,
            angle_max: scan.angle_max,
            angle_increment: scan.angle_increment,
            time_increment: 0.0,
            scan_time: 0.0,
            range_min: scan.range_min,
            range_max: scan.range_max,
            ranges: scan.ranges.clone(),
            intensities: vec![],
        })
        .unwrap_or_default(),
    });

    let img = frame
        .image
        .as_ref()
        .map(|i| msgs::Image {
            header: header(frame, &cfg.camera_frame),
            height: i.height,
            width: i.width,
            encoding: i.encoding.clone(),
            is_bigendian: 0,
            step: i.width * 3,
            data: i.data.clone(),
        })
        .unwrap_or(msgs::Image {
            header: header(frame, &cfg.camera_frame),
            height: 0,
            width: 0,
            encoding: "rgb8".into(),
            is_bigendian: 0,
            step: 0,
            data: vec![],
        });
    out.push(Publication {
        topic: "/camera/image_raw".into(),
        msg_type: "sensor_msgs/msg/Image".into(),
        payload: serde_json::to_value(img).unwrap_or_default(),
    });

    let odom = frame
        .robots
        .first()
        .map(|r| msgs::Odometry {
            header: header(frame, &cfg.odom_frame),
            child_frame_id: cfg.base_frame.clone(),
            pose: msgs::PoseWithCovariance {
                pose: msgs::Pose {
                    position: msgs::Point {
                        x: r.pose.position.x as f64,
                        y: r.pose.position.y as f64,
                        z: 0.0,
                    },
                    orientation: msgs::quat_from_yaw(r.pose.heading),
                },
                covariance: vec![0.0; 36],
            },
            twist: msgs::TwistWithCovariance {
                twist: msgs::Twist {
                    linear: msgs::Vector3 {
                        x: r.v as f64,
                        y: 0.0,
                        z: 0.0,
                    },
                    angular: msgs::Vector3 {
                        x: 0.0,
                        y: 0.0,
                        z: r.omega as f64,
                    },
                },
                covariance: vec![0.0; 36],
            },
        })
        .unwrap_or(msgs::Odometry {
            header: header(frame, &cfg.odom_frame),
            child_frame_id: cfg.base_frame.clone(),
            pose: Default::default(),
            twist: Default::default(),
        });
    out.push(Publication {
        topic: "/odom".into(),
        msg_type: "nav_msgs/msg/Odometry".into(),
        payload: serde_json::to_value(odom).unwrap_or_default(),
    });

    out
}
