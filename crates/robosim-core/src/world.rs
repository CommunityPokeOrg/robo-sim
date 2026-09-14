//! The simulated world: arena bounds, obstacles, and robots.

use crate::geometry::{Aabb, Pose2};
use crate::robot::DiffDriveRobot;
use glam::Vec2;

/// A static obstacle in the world.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub enum Obstacle {
    /// Axis-aligned box.
    Box {
        /// Center position.
        center: Vec2,
        /// Half extents.
        half_extents: Vec2,
    },
    /// Circle pillar.
    Circle {
        /// Center position.
        center: Vec2,
        /// Radius.
        radius: f32,
    },
}

impl Obstacle {
    /// Center of the obstacle.
    pub fn center(&self) -> Vec2 {
        match self {
            Obstacle::Box { center, .. } | Obstacle::Circle { center, .. } => *center,
        }
    }
}

/// Everything the simulator steps: robots plus static geometry.
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct World {
    /// Static obstacles.
    pub obstacles: Vec<Obstacle>,
    /// Arena bounds; robots are kept inside.
    pub bounds: Aabb,
    /// Simulated robots.
    pub robots: Vec<DiffDriveRobot>,
}

impl World {
    /// Demo scene: 10x10 m arena, perimeter walls, a few boxes and pillars,
    /// one robot at the origin.
    pub fn demo_scene() -> Self {
        let half = Vec2::splat(5.0);
        let bounds = Aabb {
            min: -half,
            max: half,
        };
        let wall = 0.05;
        let mut obstacles = vec![
            // Perimeter walls as thin boxes just inside the bounds.
            Obstacle::Box {
                center: Vec2::new(0.0, 4.9),
                half_extents: Vec2::new(5.0, wall),
            },
            Obstacle::Box {
                center: Vec2::new(0.0, -4.9),
                half_extents: Vec2::new(5.0, wall),
            },
            Obstacle::Box {
                center: Vec2::new(4.9, 0.0),
                half_extents: Vec2::new(wall, 5.0),
            },
            Obstacle::Box {
                center: Vec2::new(-4.9, 0.0),
                half_extents: Vec2::new(wall, 5.0),
            },
            // Interior obstacles.
            Obstacle::Box {
                center: Vec2::new(2.0, 1.5),
                half_extents: Vec2::new(0.4, 0.4),
            },
            Obstacle::Box {
                center: Vec2::new(-1.5, -2.0),
                half_extents: Vec2::new(0.6, 0.3),
            },
            Obstacle::Circle {
                center: Vec2::new(-2.0, 2.0),
                radius: 0.35,
            },
            Obstacle::Circle {
                center: Vec2::new(2.5, -2.5),
                radius: 0.5,
            },
        ];
        let mut robots = vec![DiffDriveRobot::new("robo1", Pose2::default())];
        robots[0].state.pose.heading = 0.0;
        obstacles.shrink_to_fit();
        Self {
            obstacles,
            bounds,
            robots,
        }
    }
}
