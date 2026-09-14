//! Core simulation library for robo-sim: clock, world, differential-drive
//! robots, pluggable physics, and analytic sensors.
#![forbid(unsafe_code)]

pub mod geometry;
pub mod physics;
pub mod robot;
pub mod sensors;
pub mod sim;
pub mod time;
pub mod world;

pub use geometry::{Aabb, Pose2};
pub use physics::{PhysicsBackend, SimplePhysics};
pub use robot::{DiffDriveParams, DiffDriveRobot, DiffDriveState, WheelCommand};
pub use sensors::{Image, LaserScan, Lidar2D, MockCamera, OdomSample, Sensor};
pub use sim::{Frame, RobotSnapshot, Simulator};
pub use time::SimClock;
pub use world::{Obstacle, World};
