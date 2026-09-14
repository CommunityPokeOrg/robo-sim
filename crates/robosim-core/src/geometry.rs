//! 2D geometry primitives and ray-casting helpers.

use glam::Vec2;

/// 2D pose: position plus heading (radians, counter-clockwise from +x).
#[derive(Debug, Clone, Copy, Default, serde::Serialize, serde::Deserialize)]
pub struct Pose2 {
    /// Position in world frame.
    pub position: Vec2,
    /// Heading angle in radians.
    pub heading: f32,
}

impl Pose2 {
    /// Create a pose.
    pub fn new(position: Vec2, heading: f32) -> Self {
        Self { position, heading }
    }

    /// Transform a point from this pose's local frame into the world frame.
    pub fn transform_point(&self, local: Vec2) -> Vec2 {
        let (s, c) = self.heading.sin_cos();
        self.position + Vec2::new(c * local.x - s * local.y, s * local.x + c * local.y)
    }

    /// Rotate a direction vector by the heading.
    pub fn rotate(&self, v: Vec2) -> Vec2 {
        let (s, c) = self.heading.sin_cos();
        Vec2::new(c * v.x - s * v.y, s * v.x + c * v.y)
    }
}

/// Axis-aligned bounding box.
#[derive(Debug, Clone, Copy, Default, serde::Serialize, serde::Deserialize)]
pub struct Aabb {
    /// Minimum corner.
    pub min: Vec2,
    /// Maximum corner.
    pub max: Vec2,
}

impl Aabb {
    /// Create an AABB from a center and half extents.
    pub fn from_center(center: Vec2, half_extents: Vec2) -> Self {
        Self {
            min: center - half_extents,
            max: center + half_extents,
        }
    }

    /// Does the box contain the point (inclusive)?
    pub fn contains(&self, p: Vec2) -> bool {
        p.x >= self.min.x && p.x <= self.max.x && p.y >= self.min.y && p.y <= self.max.y
    }

    /// Closest point on or inside the box to `p`.
    pub fn closest_point(&self, p: Vec2) -> Vec2 {
        p.clamp(self.min, self.max)
    }
}

/// Closest point on/in an AABB to `p`.
pub fn closest_point_aabb(b: &Aabb, p: Vec2) -> Vec2 {
    b.closest_point(p)
}

/// Ray-vs-circle intersection. Returns the smallest positive hit distance, if any.
/// `origin`/`dir`: ray parametrization `origin + t*dir`; `dir` need not be normalized
/// but returned `t` is in units where `|dir| == 1`.
pub fn ray_circle(origin: Vec2, dir: Vec2, center: Vec2, radius: f32) -> Option<f32> {
    let oc = origin - center;
    let a = dir.dot(dir);
    let b = oc.dot(dir);
    let c = oc.dot(oc) - radius * radius;
    let disc = b * b - a * c;
    if disc < 0.0 || a <= f32::EPSILON {
        return None;
    }
    let sq = disc.sqrt();
    let t = (-b - sq) / a;
    if t >= 0.0 {
        Some(t)
    } else {
        let t2 = (-b + sq) / a;
        (t2 >= 0.0).then_some(t2)
    }
}

/// Ray-vs-AABB via the slab method. Returns entry distance if hit, `Some(0.0)` if
/// the origin is inside.
pub fn ray_aabb(origin: Vec2, dir: Vec2, b: &Aabb) -> Option<f32> {
    let mut tmin = f32::NEG_INFINITY;
    let mut tmax = f32::INFINITY;
    for axis in 0..2 {
        let o = origin[axis];
        let d = dir[axis];
        let (lo, hi) = (b.min[axis], b.max[axis]);
        if d.abs() < f32::EPSILON {
            if o < lo || o > hi {
                return None;
            }
        } else {
            let inv = 1.0 / d;
            let (t1, t2) = {
                let a = (lo - o) * inv;
                let c = (hi - o) * inv;
                (a.min(c), a.max(c))
            };
            tmin = tmin.max(t1);
            tmax = tmax.min(t2);
            if tmin > tmax {
                return None;
            }
        }
    }
    if tmax < 0.0 {
        return None;
    }
    Some(tmin.max(0.0))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ray_hits_circle() {
        let t = ray_circle(Vec2::ZERO, Vec2::X, Vec2::new(5.0, 0.0), 1.0).unwrap();
        assert!((t - 4.0).abs() < 1e-5);
    }

    #[test]
    fn ray_hits_aabb() {
        let b = Aabb::from_center(Vec2::new(5.0, 0.0), Vec2::splat(0.5));
        let t = ray_aabb(Vec2::ZERO, Vec2::X, &b).unwrap();
        assert!((t - 4.5).abs() < 1e-5);
        assert!(ray_aabb(Vec2::ZERO, Vec2::Y, &b).is_none());
    }
}
