use crate::math::Vec3;
use crate::EPSILON;

#[derive(Copy, Clone)]
pub struct Plane {
    normal: Vec3,
    dist: f32,
}

impl Plane {
    pub fn new(normal: Vec3, dist: f32) -> Self {
        Plane { normal, dist }
    }

    pub fn from_pos_normal(pos: Vec3, normal: Vec3) -> Self {
        Plane {
            normal,
            dist: normal.dot(pos),
        }
    }

    pub fn classify_side(&self, point: Vec3) -> Side {
        let res = self.normal.dot(point) - self.dist;
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
    pub fn normal(&self) -> Vec3 {
        self.normal
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Side {
    Above,
    Below,
    On,
}
