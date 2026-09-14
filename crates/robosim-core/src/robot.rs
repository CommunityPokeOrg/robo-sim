//! Differential-drive robot model.

use crate::geometry::Pose2;
use glam::Vec2;

/// Physical parameters of a differential-drive robot.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct DiffDriveParams {
    /// Wheel radius (m).
    pub wheel_radius: f32,
    /// Distance between the wheels (m).
    pub wheel_base: f32,
    /// Maximum wheel angular speed (rad/s); commands are clamped.
    pub max_wheel_speed: f32,
    /// Collision-body radius (m).
    pub body_radius: f32,
}

impl Default for DiffDriveParams {
    fn default() -> Self {
        Self {
            wheel_radius: 0.033,
            wheel_base: 0.16,
            max_wheel_speed: 20.0,
            body_radius: 0.12,
        }
    }
}

/// Wheel-speed command (rad/s): left, right.
#[derive(Debug, Clone, Copy, Default, serde::Serialize, serde::Deserialize)]
pub struct WheelCommand {
    /// Left wheel angular velocity (rad/s).
    pub left_rad_s: f32,
    /// Right wheel angular velocity (rad/s).
    pub right_rad_s: f32,
}

impl WheelCommand {
    /// Convert a body twist (linear v m/s, angular omega rad/s) to wheel speeds.
    /// v = r/2 (wl + wr); omega = r/L (wr - wl).
    pub fn from_twist(v: f32, omega: f32, p: &DiffDriveParams) -> Self {
        let wl = (v - omega * p.wheel_base * 0.5) / p.wheel_radius;
        let wr = (v + omega * p.wheel_base * 0.5) / p.wheel_radius;
        Self {
            left_rad_s: wl,
            right_rad_s: wr,
        }
    }

    /// Clamp both wheel speeds to `max_wheel_speed` (preserving ratio).
    pub fn clamped(&self, max: f32) -> Self {
        let scale = (self.left_rad_s.abs().max(self.right_rad_s.abs()) / max).max(1.0);
        Self {
            left_rad_s: self.left_rad_s / scale,
            right_rad_s: self.right_rad_s / scale,
        }
    }
}

/// Dynamic state of a differential-drive robot.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct DiffDriveState {
    /// World pose.
    pub pose: Pose2,
    /// Body-frame linear velocity (m/s).
    pub v: f32,
    /// Yaw rate (rad/s).
    pub omega: f32,
    /// Accumulated wheel rotation, [left, right] (rad) for /joint_states.
    pub wheel_angles: [f32; 2],
    /// Current wheel speeds, [left, right] (rad/s).
    pub wheel_velocities: [f32; 2],
}

impl Default for DiffDriveState {
    fn default() -> Self {
        Self {
            pose: Pose2::default(),
            v: 0.0,
            omega: 0.0,
            wheel_angles: [0.0; 2],
            wheel_velocities: [0.0; 2],
        }
    }
}

/// A differential-drive robot with a pending wheel command.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct DiffDriveRobot {
    /// Robot name (used as tf prefix base: `<name>/base_link` is `base_link` for now).
    pub name: String,
    /// Physical parameters.
    pub params: DiffDriveParams,
    /// Dynamic state.
    pub state: DiffDriveState,
    /// Latest wheel command.
    pub cmd: WheelCommand,
}

/// Joint names published in /joint_states, matching the wheel order.
pub const WHEEL_JOINT_NAMES: [&str; 2] = ["left_wheel_joint", "right_wheel_joint"];

impl DiffDriveRobot {
    /// Create a robot with default parameters at a pose.
    pub fn new(name: impl Into<String>, pose: Pose2) -> Self {
        Self {
            name: name.into(),
            params: DiffDriveParams::default(),
            state: DiffDriveState {
                pose,
                ..Default::default()
            },
            cmd: WheelCommand::default(),
        }
    }

    /// Command a body twist (v m/s, omega rad/s).
    pub fn set_twist(&mut self, v: f32, omega: f32) {
        self.cmd =
            WheelCommand::from_twist(v, omega, &self.params).clamped(self.params.max_wheel_speed);
    }

    /// Integrate kinematics one step: exact arc when |omega| > eps.
    pub fn integrate(&mut self, dt: f32) {
        let cmd = self.cmd.clamped(self.params.max_wheel_speed);
        let r = self.params.wheel_radius;
        let l = self.params.wheel_base;
        let v = r * 0.5 * (cmd.left_rad_s + cmd.right_rad_s);
        let omega = r / l * (cmd.right_rad_s - cmd.left_rad_s);

        let p = self.state.pose;
        let new_pose = if omega.abs() > 1e-6 {
            // Exact arc: heading rotates by omega*dt; position follows the ICC arc.
            let dtheta = omega * dt;
            let radius = v / omega;
            let rel = Vec2::new(radius * dtheta.sin(), radius * (1.0 - dtheta.cos()));
            Pose2::new(p.position + p.rotate(rel), p.heading + dtheta)
        } else {
            Pose2::new(p.position + p.rotate(Vec2::new(v * dt, 0.0)), p.heading)
        };

        self.state.pose = new_pose;
        self.state.v = v;
        self.state.omega = omega;
        self.state.wheel_velocities = [cmd.left_rad_s, cmd.right_rad_s];
        self.state.wheel_angles[0] += cmd.left_rad_s * dt;
        self.state.wheel_angles[1] += cmd.right_rad_s * dt;
    }
}
