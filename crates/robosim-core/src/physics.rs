//! Pluggable physics. `SimplePhysics` integrates diff-drive kinematics and
//! resolves robot-vs-obstacle collisions by push-out along the contact normal.

use crate::geometry::{closest_point_aabb, Aabb};
use crate::robot::DiffDriveRobot;
use crate::world::{Obstacle, World};
use glam::Vec2;

/// A physics step over the world.
pub trait PhysicsBackend {
    /// Advance the world by `dt` seconds.
    fn step(&mut self, world: &mut World, dt: f64);
}

/// Default kinematic backend with circle-vs-shape push-out collision.
/// Deterministic: no randomness, fixed iteration order.
#[derive(Debug, Default)]
pub struct SimplePhysics;

impl PhysicsBackend for SimplePhysics {
    fn step(&mut self, world: &mut World, dt: f64) {
        let bounds = world.bounds;
        for robot in &mut world.robots {
            robot.integrate(dt as f32);
            let radius = robot.params.body_radius;
            for obstacle in &world.obstacles {
                resolve_robot_obstacle(robot, radius, *obstacle);
            }
            resolve_robot_bounds(robot, radius, &bounds);
        }
    }
}

/// Kill the velocity component along `normal` (normal points away from the
/// contact, toward free space).
fn kill_normal_velocity(robot: &mut DiffDriveRobot, normal: Vec2) {
    let vel = robot.state.pose.rotate(Vec2::new(robot.state.v, 0.0));
    let vn = vel.dot(normal);
    if vn < 0.0 {
        let corrected = vel - normal * vn;
        let heading_vec = robot.state.pose.rotate(Vec2::X);
        robot.state.v = corrected.dot(heading_vec);
    }
}

/// Push the robot out of an obstacle and kill velocity into the contact.
fn resolve_robot_obstacle(robot: &mut DiffDriveRobot, radius: f32, ob: Obstacle) {
    let pos = robot.state.pose.position;
    match ob {
        Obstacle::Box {
            center,
            half_extents,
        } => {
            let b = Aabb::from_center(center, half_extents);
            let closest = closest_point_aabb(&b, pos);
            let d = pos - closest;
            let dist = d.length();
            if dist >= radius {
                return;
            }
            let normal = if dist > f32::EPSILON {
                d / dist
            } else {
                (pos - center).normalize_or(Vec2::X)
            };
            robot.state.pose.position = pos + normal * (radius - dist);
            kill_normal_velocity(robot, normal);
        }
        Obstacle::Circle { center, radius: r } => {
            let d = pos - center;
            let dist = d.length();
            let overlap = r + radius - dist;
            if overlap <= 0.0 {
                return;
            }
            let normal = d.normalize_or(Vec2::X);
            robot.state.pose.position = pos + normal * overlap;
            kill_normal_velocity(robot, normal);
        }
    }
}

/// Keep the robot body inside the arena bounds.
fn resolve_robot_bounds(robot: &mut DiffDriveRobot, radius: f32, bounds: &Aabb) {
    let pos = robot.state.pose.position;
    let clamped = pos.clamp(
        bounds.min + Vec2::splat(radius),
        bounds.max - Vec2::splat(radius),
    );
    if clamped != pos {
        let push = clamped - pos;
        robot.state.pose.position = clamped;
        kill_normal_velocity(robot, push.normalize_or_zero());
    }
}
