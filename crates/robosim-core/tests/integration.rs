//! End-to-end tests: physics behaviour and sensor correctness in a real world.

use glam::Vec2;
use robosim_core::{Aabb, MockCamera, Obstacle, Pose2, Sensor, SimClock, Simulator, World};

fn empty_world(pose: Pose2) -> World {
    World {
        obstacles: Vec::new(),
        bounds: Aabb::from_center(Vec2::ZERO, Vec2::splat(50.0)),
        robots: vec![robosim_core::DiffDriveRobot::new("r", pose)],
    }
}

#[test]
fn lidar_hits_known_box() {
    let mut world = empty_world(Pose2::default());
    world.obstacles.push(Obstacle::Box {
        center: Vec2::new(3.0, 0.0),
        half_extents: Vec2::splat(0.5),
    });
    let clock = SimClock::fixed_timestep();
    let lidar = robosim_core::Lidar2D {
        num_beams: 1,
        fov_rad: 0.0,
        ..Default::default()
    };
    let scan = lidar.sample(&world, &world.robots[0], &clock);
    // Beam points along +x; box face at x = 2.5.
    assert!((scan.ranges[0] - 2.5).abs() < 0.01, "r={}", scan.ranges[0]);
}

#[test]
fn lidar_miss_is_infinite() {
    let world = empty_world(Pose2::default());
    let clock = SimClock::fixed_timestep();
    let lidar = robosim_core::Lidar2D {
        num_beams: 1,
        fov_rad: 0.0,
        ..Default::default()
    };
    let scan = lidar.sample(&world, &world.robots[0], &clock);
    assert_eq!(scan.ranges[0], f32::INFINITY);
}

#[test]
fn robot_stops_at_wall() {
    let mut world = empty_world(Pose2::default());
    // Wall across x = 1.0 (box face at 0.95).
    world.obstacles.push(Obstacle::Box {
        center: Vec2::new(1.0, 0.0),
        half_extents: Vec2::new(0.05, 5.0),
    });
    let mut sim = Simulator::new(world);
    sim.set_twist(0, 0.5, 0.0);
    for _ in 0..500 {
        sim.step();
        let pos = sim.world.robots[0].state.pose.position;
        let radius = sim.world.robots[0].params.body_radius;
        // Robot body must never enter the box (face at 0.95).
        assert!(pos.x + radius <= 0.95 + 1e-4, "x={}", pos.x);
    }
    // And it should have reached the wall (stuck just outside).
    let pos = sim.world.robots[0].state.pose.position;
    assert!(pos.x > 0.7, "x={}", pos.x);
}

#[test]
fn camera_image_size() {
    let world = World::demo_scene();
    let cam = MockCamera {
        width: 64,
        height: 48,
        ..Default::default()
    };
    let clock = SimClock::fixed_timestep();
    let img = cam.sample(&world, &world.robots[0], &clock);
    assert_eq!(img.data.len(), 64 * 48 * 3);
    assert_eq!(img.encoding, "rgb8");
}

#[test]
fn wheel_command_roundtrip() {
    let p = robosim_core::DiffDriveParams::default();
    let cmd = robosim_core::WheelCommand::from_twist(0.5, 0.25, &p);
    let v = p.wheel_radius * 0.5 * (cmd.left_rad_s + cmd.right_rad_s);
    let omega = p.wheel_radius / p.wheel_base * (cmd.right_rad_s - cmd.left_rad_s);
    assert!((v - 0.5).abs() < 1e-5);
    assert!((omega - 0.25).abs() < 1e-5);
}
