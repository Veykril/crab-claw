use crate::math::dot_v3;
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

    pub fn classify_side(&self, point: [f32; 3]) -> Side {
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
pub enum Side {
    On,
    Above,
    Below,
}
