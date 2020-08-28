use crate::{
    math::{dot_v3, sub_v3},
    plane::Side,
    Plane, Vertex, EPSILON,
};
use arrayvec::ArrayVec;

/// Result of an triangle intersection
pub(crate) struct TriangleSplit<V> {
    /// split points
    pub points: [V; 2],
    /// new triangles, can be either 2 or 3
    pub hull: ArrayVec<[Triangle<V>; 3]>, // this seems weird, is there a better way?
    /// index into hull seperating lower from upper hull [lower; higher]
    pub lower_split: usize,
}

#[derive(Clone)]
pub struct Triangle<V> {
    pub a: V,
    pub b: V,
    pub c: V,
}

impl<V: Vertex> Triangle<V> {
    pub fn new(a: V, b: V, c: V) -> Self {
        Triangle { a, b, c }
    }

    /// Reverses the winding order of this triangle
    pub(crate) fn reverse_winding(&mut self) {
        std::mem::swap(&mut self.b, &mut self.c);
    }

    pub(crate) fn flip_normals(&mut self) {
        self.a.flip_normal();
        self.b.flip_normal();
        self.c.flip_normal();
    }
}

// clean this up
pub(crate) fn intersect_triangle<V: Vertex>(
    plane: Plane,
    triangle: Triangle<V>,
) -> Option<TriangleSplit<V>> {
    // TODO: could return a Result<TriangleSplit<V>, Side> instead, returning the side the triangle is on
    // would allow to remove the repeated checks in slice_convex
    // also optimize this in general, lots of branching
    let side_a = plane.classify_side(triangle.a.pos());
    let side_b = plane.classify_side(triangle.b.pos());
    let side_c = plane.classify_side(triangle.c.pos());

    // triangle is either fully on the plane or not touching the plan
    if side_a == side_b && side_b == side_c {
        return None;
    }

    // triangle is adjacent to plane with one side
    if (side_a == Side::On && side_b == Side::On)
        || (side_b == Side::On && side_c == Side::On)
        || (side_c == Side::On && side_a == Side::On)
    {
        return None;
    }

    // one point is on the plane, rest on one side
    if (side_a == Side::On && side_b != Side::On && side_b == side_c)
        || (side_b == Side::On && side_a != Side::On && side_a == side_c)
        || (side_c == Side::On && side_a != Side::On && side_a == side_b)
    {
        return None;
    }

    // cases in which we will gen 2 triangles due to one point lying on the plane
    if side_a == Side::On {
        if let Some(ip) = intersect_line(plane, &triangle.b, &triangle.c) {
            let a = Triangle::new(triangle.a.clone(), triangle.b.clone(), ip.clone());
            let b = Triangle::new(triangle.a.clone(), ip.clone(), triangle.c.clone());
            let mut res = TriangleSplit {
                points: [ip, triangle.a],
                lower_split: 1,
                hull: ArrayVec::new(),
            };
            match side_b {
                Side::Above => {
                    res.hull.push(b);
                    res.hull.push(a);
                }
                Side::Below => {
                    res.hull.push(a);
                    res.hull.push(b);
                }
                Side::On => unreachable!(),
            };
            return Some(res);
        }
    } else if side_b == Side::On {
        if let Some(ip) = intersect_line(plane, &triangle.a, &triangle.c) {
            let a = Triangle::new(triangle.a.clone(), triangle.b.clone(), ip.clone());
            let b = Triangle::new(ip.clone(), triangle.b.clone(), triangle.c.clone());
            let mut res = TriangleSplit {
                points: [ip, triangle.b],
                lower_split: 1,
                hull: ArrayVec::new(),
            };
            match side_a {
                Side::Above => {
                    res.hull.push(b);
                    res.hull.push(a);
                }
                Side::Below => {
                    res.hull.push(a);
                    res.hull.push(b);
                }
                Side::On => unreachable!(),
            };
            return Some(res);
        }
    } else if side_c == Side::On {
        if let Some(ip) = intersect_line(plane, &triangle.a, &triangle.b) {
            let a = Triangle::new(triangle.a.clone(), ip.clone(), triangle.c.clone());
            let b = Triangle::new(ip.clone(), triangle.b.clone(), triangle.c.clone());
            let mut res = TriangleSplit {
                points: [ip, triangle.c],
                lower_split: 1,
                hull: ArrayVec::new(),
            };
            match side_a {
                Side::Above => {
                    res.hull.push(b);
                    res.hull.push(a);
                }
                Side::Below => {
                    res.hull.push(a);
                    res.hull.push(b);
                }
                Side::On => unreachable!(),
            };
            return Some(res);
        }
    // 3 triangles, we cut through two lines in these cases, so one side of the split will be a polygon with 4 edges which has to be split
    } else {
        if side_a != side_b {
            if let Some(ip) = intersect_line(plane, &triangle.a, &triangle.b) {
                if side_a == side_c {
                    if let Some(ip2) = intersect_line(plane, &triangle.b, &triangle.c) {
                        let a = Triangle::new(ip.clone(), triangle.b.clone(), ip2.clone());
                        let b = Triangle::new(triangle.a.clone(), ip.clone(), ip2.clone());
                        let c = Triangle::new(triangle.a.clone(), ip2.clone(), triangle.c.clone());
                        let mut hull = ArrayVec::new();
                        let lower_split = match side_a {
                            Side::Above => {
                                hull.push(a);
                                hull.push(b);
                                hull.push(c);
                                1
                            }
                            Side::Below => {
                                hull.push(b);
                                hull.push(c);
                                hull.push(a);
                                2
                            }
                            Side::On => unreachable!(),
                        };
                        return Some(TriangleSplit {
                            points: [ip, ip2],
                            lower_split,
                            hull,
                        });
                    }
                } else if let Some(ip2) = intersect_line(plane, &triangle.a, &triangle.c) {
                    let a = Triangle::new(triangle.a.clone(), ip.clone(), ip2.clone());
                    let b = Triangle::new(ip.clone(), triangle.b.clone(), triangle.c.clone());
                    let c = Triangle::new(ip2.clone(), ip.clone(), triangle.c.clone());
                    let mut hull = ArrayVec::new();
                    let lower_split = match side_a {
                        Side::Above => {
                            hull.push(b);
                            hull.push(c);
                            hull.push(a);
                            2
                        }
                        Side::Below => {
                            hull.push(a);
                            hull.push(b);
                            hull.push(c);
                            1
                        }
                        Side::On => unreachable!(),
                    };
                    return Some(TriangleSplit {
                        points: [ip, ip2],
                        lower_split,
                        hull,
                    });
                } else {
                    return None;
                }
            }
        }
        // no match to have lazy logical and-ing
        if let Some(ip) = intersect_line(plane, &triangle.c, &triangle.a) {
            if let Some(ip2) = intersect_line(plane, &triangle.c, &triangle.b) {
                let a = Triangle::new(ip.clone(), ip2.clone(), triangle.c.clone());
                let b = Triangle::new(triangle.a.clone(), ip2.clone(), ip.clone());
                let c = Triangle::new(triangle.a.clone(), triangle.b.clone(), ip2.clone());

                let mut hull = ArrayVec::new();
                let lower_split = match side_a {
                    Side::Above => {
                        hull.push(a);
                        hull.push(b);
                        hull.push(c);
                        1
                    }
                    Side::Below => {
                        hull.push(b);
                        hull.push(c);
                        hull.push(a);
                        2
                    }
                    Side::On => unreachable!(),
                };
                return Some(TriangleSplit {
                    points: [ip, ip2],
                    lower_split,
                    hull,
                });
            }
        }
    }
    None
}

fn intersect_line<V: Vertex>(plane: Plane, a: &V, b: &V) -> Option<V> {
    let line = sub_v3(b.pos(), a.pos());

    let ln = dot_v3(plane.normal(), line);
    if ln == 0.0 {
        None
    } else {
        let t = (plane.dist() - dot_v3(plane.normal(), a.pos())) / ln;
        // clamp between ~0.0 and ~1.0 since we only want the segment
        if t >= -EPSILON && t <= (1.0 + EPSILON) {
            Some(V::new_interpolated(a, b, t))
        } else {
            None
        }
    }
}
