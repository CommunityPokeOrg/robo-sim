//! ROS 2 message structs with field names matching rosbridge JSON exactly.

use serde::{Deserialize, Serialize};

/// builtin_interfaces/Time
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct Time {
    /// Seconds.
    pub sec: i32,
    /// Nanoseconds.
    pub nanosec: u32,
}

/// std_msgs/Header
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Header {
    /// Stamp.
    pub stamp: Time,
    /// Frame id.
    pub frame_id: String,
}

/// rosgraph_msgs/Clock
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Clock {
    /// Sim time.
    pub clock: Time,
}

/// geometry_msgs/Vector3
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct Vector3 {
    /// x
    pub x: f64,
    /// y
    pub y: f64,
    /// z
    pub z: f64,
}

/// geometry_msgs/Quaternion
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct Quaternion {
    /// x
    pub x: f64,
    /// y
    pub y: f64,
    /// z
    pub z: f64,
    /// w
    pub w: f64,
}

/// Quaternion from yaw (rad).
pub fn quat_from_yaw(yaw: f32) -> Quaternion {
    let half = yaw as f64 * 0.5;
    Quaternion {
        x: 0.0,
        y: 0.0,
        z: half.sin(),
        w: half.cos(),
    }
}

/// geometry_msgs/Transform
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct Transform {
    /// Translation.
    pub translation: Vector3,
    /// Rotation.
    pub rotation: Quaternion,
}

/// geometry_msgs/TransformStamped
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransformStamped {
    /// Header.
    pub header: Header,
    /// Child frame.
    pub child_frame_id: String,
    /// Transform.
    pub transform: Transform,
}

/// tf2_msgs/TFMessage
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TFMessage {
    /// Transforms.
    pub transforms: Vec<TransformStamped>,
}

/// sensor_msgs/JointState
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JointState {
    /// Header.
    pub header: Header,
    /// Joint names.
    pub name: Vec<String>,
    /// Positions (rad).
    pub position: Vec<f64>,
    /// Velocities (rad/s).
    pub velocity: Vec<f64>,
    /// Efforts.
    pub effort: Vec<f64>,
}

/// sensor_msgs/LaserScan
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LaserScan {
    /// Header.
    pub header: Header,
    /// First beam angle.
    pub angle_min: f32,
    /// Last beam angle.
    pub angle_max: f32,
    /// Beam spacing.
    pub angle_increment: f32,
    /// Time between measurements.
    pub time_increment: f32,
    /// Time between scans.
    pub scan_time: f32,
    /// Min range.
    pub range_min: f32,
    /// Max range.
    pub range_max: f32,
    /// Ranges.
    pub ranges: Vec<f32>,
    /// Intensities.
    pub intensities: Vec<f32>,
}

/// sensor_msgs/Image
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Image {
    /// Header.
    pub header: Header,
    /// Rows.
    pub height: u32,
    /// Cols.
    pub width: u32,
    /// Pixel encoding.
    pub encoding: String,
    /// 0 = little endian.
    pub is_bigendian: u8,
    /// Row stride in bytes.
    pub step: u32,
    /// Pixel bytes (JSON array of u8 for rosbridge).
    pub data: Vec<u8>,
}

/// geometry_msgs/Point
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct Point {
    /// x
    pub x: f64,
    /// y
    pub y: f64,
    /// z
    pub z: f64,
}

/// geometry_msgs/Pose
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct Pose {
    /// Position.
    pub position: Point,
    /// Orientation.
    pub orientation: Quaternion,
}

/// geometry_msgs/Twist
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct Twist {
    /// Linear velocity.
    pub linear: Vector3,
    /// Angular velocity.
    pub angular: Vector3,
}

/// geometry_msgs/PoseWithCovariance
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PoseWithCovariance {
    /// Pose.
    pub pose: Pose,
    /// 6x6 covariance, row-major (36 elements).
    pub covariance: Vec<f64>,
}

/// geometry_msgs/TwistWithCovariance
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TwistWithCovariance {
    /// Twist.
    pub twist: Twist,
    /// 6x6 covariance (36 elements).
    pub covariance: Vec<f64>,
}

/// nav_msgs/Odometry
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Odometry {
    /// Header.
    pub header: Header,
    /// Child frame.
    pub child_frame_id: String,
    /// Pose.
    pub pose: PoseWithCovariance,
    /// Twist.
    pub twist: TwistWithCovariance,
}
