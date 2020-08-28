use crate::math::{cross, dot_v3, normalized, sub_v3};
use crate::EPSILON;

#[derive(Copy, Clone)]
pub struct Plane {
    normal: [f32; 3],
    dist: f32,
}

impl Plane {
    #[inline(always)]
    pub fn new(normal: [f32; 3], dist: f32) -> Self {
        Plane { normal, dist }
    }

    #[inline(always)]
    pub fn from_pos_normal(pos: [f32; 3], normal: [f32; 3]) -> Self {
        Plane {
            normal,
            dist: dot_v3(normal, pos),
        }
    }

    #[inline(always)]
    pub fn from_spanning_vectors(a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> Self {
        let normal = normalized(cross(sub_v3(b, a), sub_v3(c, a)));

        Plane {
            normal,
            dist: -dot_v3(normal, a),
        }
    }

    pub(crate) fn classify_side(&self, point: [f32; 3]) -> Side {
        let res = dot_v3(self.normal, point) - self.dist;
        if res < -EPSILON {
            Side::Below
        } else if res > EPSILON {
            Side::Above
        } else {
            Side::On
        }
    }

    #[inline(always)]
    pub fn dist(&self) -> f32 {
        self.dist
    }

    #[inline(always)]
    pub fn normal(&self) -> [f32; 3] {
        self.normal
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub(crate) enum Side {
    // On the plane
    On,
    // above the plane(aka the side where the normal points)
    Above,
    // below the plane(against the normal)
    Below,
}
