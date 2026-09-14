//! Top-down software rasterizer; writes RGB frames and PPM files.

use crate::{RenderBackend, RenderError, RenderOutput};
use glam::Vec2;
use robosim_core::{Frame, Obstacle, World};
use std::path::Path;

const BG: [u8; 3] = [24, 24, 28];
const BOUNDS_COLOR: [u8; 3] = [70, 70, 80];
const BOX_COLOR: [u8; 3] = [200, 80, 60];
const CIRCLE_COLOR: [u8; 3] = [80, 160, 90];
const ROBOT_COLOR: [u8; 3] = [240, 220, 60];
const HEADING_COLOR: [u8; 3] = [255, 255, 255];
const LIDAR_COLOR: [u8; 3] = [80, 200, 240];

/// Software top-down renderer.
pub struct HeadlessBackend {
    w: u32,
    h: u32,
    buf: Vec<u8>,
    view_min: Vec2,
    view_max: Vec2,
}

impl Default for HeadlessBackend {
    fn default() -> Self {
        Self {
            w: 0,
            h: 0,
            buf: Vec::new(),
            view_min: Vec2::new(-5.5, -5.5),
            view_max: Vec2::new(5.5, 5.5),
        }
    }
}

impl HeadlessBackend {
    /// Set the world rect shown in the image.
    pub fn with_view(mut self, min: Vec2, max: Vec2) -> Self {
        self.view_min = min;
        self.view_max = max;
        self
    }

    /// Save the last frame as a binary PPM (P6). Call after `end_frame`.
    pub fn save_ppm(&self, path: impl AsRef<Path>) -> Result<(), RenderError> {
        use std::io::Write;
        let mut f = std::io::BufWriter::new(std::fs::File::create(path)?);
        write!(f, "P6\n{} {}\n255\n", self.w, self.h)?;
        f.write_all(&self.buf)?;
        Ok(())
    }

    fn to_px(&self, p: Vec2) -> (i32, i32) {
        let span = self.view_max - self.view_min;
        let x = (p.x - self.view_min.x) / span.x * self.w as f32;
        // Flip y so +y is up on the image.
        let y = (self.view_max.y - p.y) / span.y * self.h as f32;
        (x as i32, y as i32)
    }

    fn put(&mut self, x: i32, y: i32, c: [u8; 3]) {
        if x >= 0 && y >= 0 && x < self.w as i32 && y < self.h as i32 {
            let i = (y as usize * self.w as usize + x as usize) * 3;
            self.buf[i..i + 3].copy_from_slice(&c);
        }
    }

    fn line(&mut self, a: Vec2, b: Vec2, c: [u8; 3]) {
        let (x0, y0) = self.to_px(a);
        let (x1, y1) = self.to_px(b);
        let n = (x1 - x0).abs().max((y1 - y0).abs()).max(1);
        for i in 0..=n {
            let t = i as f32 / n as f32;
            self.put(
                x0 + ((x1 - x0) as f32 * t) as i32,
                y0 + ((y1 - y0) as f32 * t) as i32,
                c,
            );
        }
    }

    fn fill_circle(&mut self, c: Vec2, r_m: f32, col: [u8; 3]) {
        let (cx, cy) = self.to_px(c);
        let pr = (r_m / (self.view_max.x - self.view_min.x) * self.w as f32).max(1.0) as i32;
        for dy in -pr..=pr {
            for dx in -pr..=pr {
                if dx * dx + dy * dy <= pr * pr {
                    self.put(cx + dx, cy + dy, col);
                }
            }
        }
    }
}

impl RenderBackend for HeadlessBackend {
    fn name(&self) -> &str {
        "headless"
    }

    fn begin_frame(&mut self, w: u32, h: u32) {
        self.w = w;
        self.h = h;
        self.buf = vec![0; w as usize * h as usize * 3];
        for px in self.buf.chunks_exact_mut(3) {
            px.copy_from_slice(&BG);
        }
    }

    fn draw_scene(&mut self, frame: &Frame, world: &World) {
        // Arena bounds rectangle.
        let b = world.bounds;
        for p in [
            (b.min, Vec2::new(b.max.x, b.min.y)),
            (Vec2::new(b.max.x, b.min.y), b.max),
            (b.max, Vec2::new(b.min.x, b.max.y)),
            (Vec2::new(b.min.x, b.max.y), b.min),
        ] {
            self.line(p.0, p.1, BOUNDS_COLOR);
        }
        for ob in &world.obstacles {
            match *ob {
                Obstacle::Box {
                    center,
                    half_extents,
                } => {
                    let bb = robosim_core::Aabb::from_center(center, half_extents);
                    for e in [
                        (bb.min, Vec2::new(bb.max.x, bb.min.y)),
                        (Vec2::new(bb.max.x, bb.min.y), bb.max),
                        (bb.max, Vec2::new(bb.min.x, bb.max.y)),
                        (Vec2::new(bb.min.x, bb.max.y), bb.min),
                    ] {
                        self.line(e.0, e.1, BOX_COLOR);
                    }
                }
                Obstacle::Circle { center, radius } => {
                    // Approximate with a polygon.
                    let n = 24;
                    for i in 0..n {
                        let a0 = i as f32 / n as f32 * std::f32::consts::TAU;
                        let a1 = (i + 1) as f32 / n as f32 * std::f32::consts::TAU;
                        self.line(
                            center + Vec2::from_angle(a0) * radius,
                            center + Vec2::from_angle(a1) * radius,
                            CIRCLE_COLOR,
                        );
                    }
                }
            }
        }
        // Lidar hits (sensor at robot pose).
        for r in &frame.robots {
            self.fill_circle(r.pose.position, 0.12, ROBOT_COLOR);
            let tip = r.pose.position + r.pose.rotate(Vec2::X) * 0.3;
            self.line(r.pose.position, tip, HEADING_COLOR);
            let sensor_pose = r.pose;
            for p in frame.scan.hit_points(sensor_pose) {
                self.fill_circle(p, 0.02, LIDAR_COLOR);
            }
        }
    }

    fn end_frame(&mut self) -> RenderOutput {
        RenderOutput::Pixels {
            width: self.w,
            height: self.h,
            rgb: self.buf.clone(),
        }
    }
}
