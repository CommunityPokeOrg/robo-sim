//! The `Simulator` driver: clock + world + physics + sensors → `Frame`s.

use crate::physics::{PhysicsBackend, SimplePhysics};
use crate::robot::WHEEL_JOINT_NAMES;
use crate::sensors::{Image, LaserScan, Lidar2D, MockCamera, Sensor};
use crate::time::SimClock;
use crate::world::World;
use crate::Pose2;

/// Per-robot state in a snapshot.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct RobotSnapshot {
    /// Robot name.
    pub name: String,
    /// World pose.
    pub pose: Pose2,
    /// Linear velocity (m/s).
    pub v: f32,
    /// Yaw rate (rad/s).
    pub omega: f32,
    /// Joint names, matching `WHEEL_JOINT_NAMES`.
    pub joint_names: Vec<String>,
    /// Wheel joint angles (rad).
    pub joint_positions: Vec<f32>,
    /// Wheel joint velocities (rad/s).
    pub joint_velocities: Vec<f32>,
}

/// A complete observable snapshot of the simulation at one instant.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Frame {
    /// Sim time (s).
    pub time: f64,
    /// Step index.
    pub step: u64,
    /// Per-robot state.
    pub robots: Vec<RobotSnapshot>,
    /// Lidar scan for robot 0.
    pub scan: LaserScan,
    /// Camera image, present only on `camera_every_n_steps` boundaries.
    pub image: Option<Image>,
}

/// Top-level simulator.
pub struct Simulator {
    /// Simulation clock.
    pub clock: SimClock,
    /// World state.
    pub world: World,
    /// Physics backend.
    pub physics: Box<dyn PhysicsBackend>,
    /// Lidar mounted on robot 0.
    pub lidar: Lidar2D,
    /// Mock camera mounted on robot 0.
    pub camera: MockCamera,
    /// Render a camera frame every N steps; 0 disables camera output.
    pub camera_every_n_steps: u64,
}

impl Simulator {
    /// Create a simulator with default physics, lidar and camera.
    pub fn new(world: World) -> Self {
        Self {
            clock: SimClock::fixed_timestep(),
            world,
            physics: Box::new(SimplePhysics),
            lidar: Lidar2D::default(),
            camera: MockCamera::default(),
            camera_every_n_steps: 0,
        }
    }

    /// Command a twist on robot `idx` (v m/s, omega rad/s).
    pub fn set_twist(&mut self, idx: usize, v: f32, omega: f32) {
        if let Some(r) = self.world.robots.get_mut(idx) {
            r.set_twist(v, omega);
        }
    }

    /// Advance one fixed step.
    pub fn step(&mut self) {
        self.physics.step(&mut self.world, self.clock.dt);
        self.clock.advance();
    }

    /// Advance `n` steps.
    pub fn step_n(&mut self, n: u64) {
        for _ in 0..n {
            self.step();
        }
    }

    /// Capture a `Frame` of the current state.
    pub fn snapshot(&self) -> Frame {
        let robots = self
            .world
            .robots
            .iter()
            .map(|r| RobotSnapshot {
                name: r.name.clone(),
                pose: r.state.pose,
                v: r.state.v,
                omega: r.state.omega,
                joint_names: WHEEL_JOINT_NAMES.iter().map(|s| s.to_string()).collect(),
                joint_positions: r.state.wheel_angles.to_vec(),
                joint_velocities: r.state.wheel_velocities.to_vec(),
            })
            .collect();
        let scan = self
            .world
            .robots
            .first()
            .map(|r| self.lidar.sample(&self.world, r, &self.clock))
            .unwrap_or(LaserScan {
                stamp: self.clock.to_sec_nanosec(),
                angle_min: 0.0,
                angle_max: 0.0,
                angle_increment: 0.0,
                range_min: 0.0,
                range_max: 0.0,
                ranges: Vec::new(),
            });
        let image = if self.camera_every_n_steps > 0
            && self.clock.step.is_multiple_of(self.camera_every_n_steps)
        {
            self.world
                .robots
                .first()
                .map(|r| self.camera.sample(&self.world, r, &self.clock))
        } else {
            None
        };
        Frame {
            time: self.clock.time_s,
            step: self.clock.step,
            robots,
            scan,
            image,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::World;

    #[test]
    fn straight_drive_distance() {
        let mut sim = Simulator::new(World::demo_scene());
        sim.set_twist(0, 0.5, 0.0);
        sim.step_n(100); // 1 s
        let p = sim.world.robots[0].state.pose.position;
        assert!((p.x - 0.5).abs() < 0.02, "x={}", p.x);
        assert!(p.y.abs() < 1e-4);
    }

    #[test]
    fn pure_rotation() {
        let mut sim = Simulator::new(World::demo_scene());
        sim.set_twist(0, 0.0, 1.0);
        sim.step_n(100);
        let h = sim.world.robots[0].state.pose.heading;
        assert!((h - 1.0).abs() < 0.01, "heading={h}");
    }

    #[test]
    fn wheel_speed_clamped() {
        let mut sim = Simulator::new(World::demo_scene());
        sim.set_twist(0, 100.0, 0.0);
        sim.step();
        let wv = sim.world.robots[0].state.wheel_velocities;
        let max = sim.world.robots[0].params.max_wheel_speed;
        assert!(wv.iter().all(|w| w.abs() <= max + 1e-4));
    }

    #[test]
    fn determinism() {
        let mut a = Simulator::new(World::demo_scene());
        let mut b = Simulator::new(World::demo_scene());
        for _ in 0..50 {
            a.set_twist(0, 0.4, 0.2);
            b.set_twist(0, 0.4, 0.2);
            a.step();
            b.step();
        }
        let fa = serde_json::to_string(&a.snapshot()).unwrap();
        let fb = serde_json::to_string(&b.snapshot()).unwrap();
        assert_eq!(fa, fb);
    }

    #[test]
    fn frame_serde_roundtrip() {
        let mut sim = Simulator::new(World::demo_scene());
        sim.camera_every_n_steps = 1;
        sim.step_n(5);
        let f = sim.snapshot();
        let s = serde_json::to_string(&f).unwrap();
        let f2: Frame = serde_json::from_str(&s).unwrap();
        assert_eq!(f2.step, f.step);
        assert_eq!(f2.scan.ranges.len(), f.scan.ranges.len());
        assert_eq!(f2.image.unwrap().data.len(), 160 * 120 * 3);
    }
}
