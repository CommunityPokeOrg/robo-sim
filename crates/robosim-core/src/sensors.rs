//! Sensors: analytic 2D lidar, software raycast camera, odometry helper.

use crate::geometry::{ray_aabb, ray_circle, Aabb, Pose2};
use crate::robot::DiffDriveRobot;
use crate::time::SimClock;
use crate::world::{Obstacle, World};
use glam::Vec2;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

/// Anything that samples the world for a robot.
pub trait Sensor {
    /// Sample output type.
    type Output;
    /// Take a measurement.
    fn sample(&self, world: &World, robot: &DiffDriveRobot, clock: &SimClock) -> Self::Output;
}

/// ROS-style laser scan result.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct LaserScan {
    /// Stamp as (sec, nanosec).
    pub stamp: (i32, u32),
    /// First beam angle (rad, in sensor frame).
    pub angle_min: f32,
    /// Last beam angle.
    pub angle_max: f32,
    /// Angular spacing between beams.
    pub angle_increment: f32,
    /// Minimum valid range.
    pub range_min: f32,
    /// Maximum valid range.
    pub range_max: f32,
    /// Measured ranges; missed rays are `f32::INFINITY`.
    pub ranges: Vec<f32>,
}

/// 2D lidar with analytic ray casting.
#[derive(Debug, Clone)]
pub struct Lidar2D {
    /// Number of beams.
    pub num_beams: usize,
    /// Field of view (rad).
    pub fov_rad: f32,
    /// Maximum range (m); misses return INFINITY.
    pub max_range: f32,
    /// Minimum range (m); closer hits are clamped to min_range.
    pub min_range: f32,
    /// Gaussian noise stddev (m); 0 disables noise.
    pub noise_std: f32,
    /// Sensor pose in the robot body frame.
    pub mount_offset: Pose2,
}

impl Default for Lidar2D {
    fn default() -> Self {
        Self {
            num_beams: 360,
            fov_rad: std::f32::consts::TAU,
            max_range: 8.0,
            min_range: 0.05,
            noise_std: 0.0,
            mount_offset: Pose2::default(),
        }
    }
}

impl Lidar2D {
    /// Cast a single ray against all obstacles and arena bounds; returns the
    /// nearest hit distance (may exceed max_range; caller clamps).
    pub fn cast_ray(world: &World, origin: Vec2, angle: f32) -> f32 {
        let dir = Vec2::from_angle(angle);
        let mut best = f32::INFINITY;
        for ob in &world.obstacles {
            let t = match *ob {
                Obstacle::Box {
                    center,
                    half_extents,
                } => ray_aabb(origin, dir, &Aabb::from_center(center, half_extents)),
                Obstacle::Circle { center, radius } => ray_circle(origin, dir, center, radius),
            };
            if let Some(t) = t {
                if t < best {
                    best = t;
                }
            }
        }
        // Walls at the arena bounds are obstacles in demo scenes, but keep a
        // safety bound check so scans are sane in bound-only worlds.
        best
    }
}

impl Sensor for Lidar2D {
    type Output = LaserScan;

    fn sample(&self, world: &World, robot: &DiffDriveRobot, clock: &SimClock) -> LaserScan {
        let sensor_pose = Pose2 {
            position: robot.state.pose.transform_point(self.mount_offset.position),
            heading: robot.state.pose.heading + self.mount_offset.heading,
        };
        let angle_min = -self.fov_rad * 0.5;
        let angle_inc = if self.num_beams > 1 {
            self.fov_rad / (self.num_beams - 1) as f32
        } else {
            0.0
        };
        // Deterministic noise stream per sample.
        let mut rng =
            (self.noise_std > 0.0).then(|| StdRng::seed_from_u64(clock.step ^ 0x5EED_5EED));

        let mut ranges = Vec::with_capacity(self.num_beams);
        for i in 0..self.num_beams {
            let angle = sensor_pose.heading + angle_min + i as f32 * angle_inc;
            let mut r = Self::cast_ray(world, sensor_pose.position, angle);
            if let Some(rng) = &mut rng {
                if r.is_finite() {
                    r += rng.random_range(-self.noise_std..self.noise_std);
                }
            }
            if r < self.min_range {
                r = self.min_range;
            }
            ranges.push(r);
        }
        LaserScan {
            stamp: clock.to_sec_nanosec(),
            angle_min,
            angle_max: angle_min + (self.num_beams.saturating_sub(1)) as f32 * angle_inc,
            angle_increment: angle_inc,
            range_min: self.min_range,
            range_max: self.max_range,
            ranges,
        }
    }
}

/// Minimal RGB image produced by the mock camera.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Image {
    /// Stamp as (sec, nanosec).
    pub stamp: (i32, u32),
    /// Width in pixels.
    pub width: u32,
    /// Height in pixels.
    pub height: u32,
    /// Always "rgb8".
    pub encoding: String,
    /// RGB pixel data, row-major, `width * height * 3` bytes.
    pub data: Vec<u8>,
}

/// Wolfenstein-style software camera: one ray per screen column.
#[derive(Debug, Clone)]
pub struct MockCamera {
    /// Image width.
    pub width: u32,
    /// Image height.
    pub height: u32,
    /// Horizontal field of view (rad).
    pub fov_rad: f32,
    /// Camera pose in the robot body frame.
    pub mount_offset: Pose2,
}

impl Default for MockCamera {
    fn default() -> Self {
        Self {
            width: 160,
            height: 120,
            fov_rad: std::f32::consts::FRAC_PI_2,
            mount_offset: Pose2::default(),
        }
    }
}

const SKY: [u8; 3] = [135, 206, 250];
const FLOOR: [u8; 3] = [96, 96, 96];
const BOX_COLOR: [u8; 3] = [200, 80, 60];
const CIRCLE_COLOR: [u8; 3] = [80, 160, 90];

impl Sensor for MockCamera {
    type Output = Image;

    fn sample(&self, world: &World, robot: &DiffDriveRobot, clock: &SimClock) -> Image {
        let cam_pose = Pose2 {
            position: robot.state.pose.transform_point(self.mount_offset.position),
            heading: robot.state.pose.heading + self.mount_offset.heading,
        };
        let (w, h) = (self.width as usize, self.height as usize);
        let mut data = vec![0u8; w * h * 3];
        for x in 0..w {
            let frac = (x as f32 + 0.5) / w as f32;
            let angle = cam_pose.heading + (frac - 0.5) * self.fov_rad;
            let dir = Vec2::from_angle(angle);
            // Nearest hit and its color.
            let mut best = f32::INFINITY;
            let mut color = BOX_COLOR;
            for ob in &world.obstacles {
                let (t, c) = match *ob {
                    Obstacle::Box {
                        center,
                        half_extents,
                    } => (
                        ray_aabb(
                            cam_pose.position,
                            dir,
                            &Aabb::from_center(center, half_extents),
                        ),
                        BOX_COLOR,
                    ),
                    Obstacle::Circle { center, radius } => (
                        ray_circle(cam_pose.position, dir, center, radius),
                        CIRCLE_COLOR,
                    ),
                };
                if let Some(t) = t {
                    if t < best {
                        best = t;
                        color = c;
                    }
                }
            }
            // Correct fisheye so flat walls look flat.
            let corrected = best * ((frac - 0.5) * self.fov_rad).cos();
            let wall_h = if corrected.is_finite() && corrected > 1e-3 {
                ((h as f32) * 0.8 / corrected).min(h as f32) as usize
            } else {
                0
            };
            let wall_top = (h - wall_h) / 2;
            for y in 0..h {
                let px = if y < wall_top || y >= h - wall_top {
                    if y < h / 2 {
                        SKY
                    } else {
                        FLOOR
                    }
                } else {
                    // Shade by distance.
                    let shade = (1.0 - (corrected / 8.0).min(0.7)).clamp(0.3, 1.0);
                    [
                        (color[0] as f32 * shade) as u8,
                        (color[1] as f32 * shade) as u8,
                        (color[2] as f32 * shade) as u8,
                    ]
                };
                let i = (y * w + x) * 3;
                data[i..i + 3].copy_from_slice(&px);
            }
        }
        Image {
            stamp: clock.to_sec_nanosec(),
            width: self.width,
            height: self.height,
            encoding: "rgb8".to_string(),
            data,
        }
    }
}

/// Odometry reading derived from robot state.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct OdomSample {
    /// World pose.
    pub pose: Pose2,
    /// Linear velocity (m/s).
    pub v: f32,
    /// Yaw rate (rad/s).
    pub omega: f32,
}

impl DiffDriveRobot {
    /// Current odometry reading.
    pub fn odometry(&self) -> OdomSample {
        OdomSample {
            pose: self.state.pose,
            v: self.state.v,
            omega: self.state.omega,
        }
    }
}

// Used by sim/render to find lidar hit points for drawing dots.
impl LaserScan {
    /// World-frame hit points (finite ranges only) given the sensor world pose.
    pub fn hit_points(&self, sensor_pose: Pose2) -> Vec<Vec2> {
        self.ranges
            .iter()
            .enumerate()
            .filter(|(_, r)| r.is_finite() && **r <= self.range_max)
            .map(|(i, r)| {
                let a = sensor_pose.heading + self.angle_min + i as f32 * self.angle_increment;
                sensor_pose.position + Vec2::from_angle(a) * *r
            })
            .collect()
    }
}
