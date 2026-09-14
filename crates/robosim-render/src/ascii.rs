//! ASCII top-down map renderer for the teleop TUI.

use crate::{RenderBackend, RenderOutput};
use glam::Vec2;
use robosim_core::{Frame, Obstacle, World};

/// Renders the world to a fixed-size character grid.
pub struct AsciiBackend {
    /// Grid columns.
    pub cols: usize,
    /// Grid rows.
    pub rows: usize,
    grid: Vec<char>,
    view_min: Vec2,
    view_max: Vec2,
}

impl AsciiBackend {
    /// Create a grid renderer of `cols` x `rows`.
    pub fn new(cols: usize, rows: usize) -> Self {
        Self {
            cols,
            rows,
            grid: vec![' '; cols * rows],
            view_min: Vec2::new(-5.5, -5.5),
            view_max: Vec2::new(5.5, 5.5),
        }
    }

    /// World rect shown on the grid.
    pub fn with_view(mut self, min: Vec2, max: Vec2) -> Self {
        self.view_min = min;
        self.view_max = max;
        self
    }

    fn put(&mut self, p: Vec2, c: char) {
        let span = self.view_max - self.view_min;
        let x = ((p.x - self.view_min.x) / span.x * self.cols as f32) as i32;
        let y = ((self.view_max.y - p.y) / span.y * self.rows as f32) as i32;
        if x >= 0 && y >= 0 && x < self.cols as i32 && y < self.rows as i32 {
            self.grid[y as usize * self.cols + x as usize] = c;
        }
    }
}

impl RenderBackend for AsciiBackend {
    fn name(&self) -> &str {
        "ascii"
    }

    fn begin_frame(&mut self, w: u32, h: u32) {
        self.cols = w as usize;
        self.rows = h as usize;
        self.grid = vec![' '; self.cols * self.rows];
    }

    fn draw_scene(&mut self, frame: &Frame, world: &World) {
        let b = world.bounds;
        for x in (0..=100).map(|i| b.min.x + (b.max.x - b.min.x) * i as f32 / 100.0) {
            self.put(Vec2::new(x, b.min.y), '.');
            self.put(Vec2::new(x, b.max.y), '.');
        }
        for y in (0..=50).map(|i| b.min.y + (b.max.y - b.min.y) * i as f32 / 50.0) {
            self.put(Vec2::new(b.min.x, y), '.');
            self.put(Vec2::new(b.max.x, y), '.');
        }
        for ob in &world.obstacles {
            match *ob {
                Obstacle::Box {
                    center,
                    half_extents,
                } => {
                    let bb = robosim_core::Aabb::from_center(center, half_extents);
                    for i in 0..=20 {
                        let t = i as f32 / 20.0;
                        self.put(bb.min.lerp(Vec2::new(bb.max.x, bb.min.y), t), '#');
                        self.put(Vec2::new(bb.min.x, bb.max.y).lerp(bb.max, t), '#');
                        self.put(bb.min.lerp(Vec2::new(bb.min.x, bb.max.y), t), '#');
                        self.put(Vec2::new(bb.max.x, bb.min.y).lerp(bb.max, t), '#');
                    }
                }
                Obstacle::Circle { center, radius } => {
                    for i in 0..16 {
                        let a = i as f32 / 16.0 * std::f32::consts::TAU;
                        self.put(center + Vec2::from_angle(a) * radius, 'o');
                    }
                }
            }
        }
        for r in &frame.robots {
            for p in frame.scan.hit_points(r.pose) {
                self.put(p, '*');
            }
            self.put(r.pose.position, '@');
            self.put(r.pose.position + r.pose.rotate(Vec2::X) * 0.3, '>');
        }
    }

    fn end_frame(&mut self) -> RenderOutput {
        let mut s = String::with_capacity(self.grid.len() + self.rows);
        for row in self.grid.chunks(self.cols) {
            s.extend(row.iter());
            s.push('\n');
        }
        RenderOutput::Text(s)
    }
}
