use crate::math::{dot_v3, sub_v3};
use crate::plane::Side;
use crate::{Plane, Vertex, EPSILON};

pub enum TriangleSplit<V> {
    UpperLower {
        upper: Triangle<V>,
        lower: Triangle<V>,
    },
    UpperTwoLower {
        upper: Triangle<V>,
        lower: [Triangle<V>; 2],
    },
    LowerTwoUpper {
        upper: [Triangle<V>; 2],
        lower: Triangle<V>,
    },
}

impl<V> TriangleSplit<V> {
    #[inline]
    pub fn append_to(self, lower_hull: &mut Vec<Triangle<V>>, upper_hull: &mut Vec<Triangle<V>>) {
        match self {
            TriangleSplit::UpperLower { upper, lower } => {
                lower_hull.push(lower);
                upper_hull.push(upper);
            }
            TriangleSplit::UpperTwoLower {
                upper,
                lower: [lower0, lower1],
            } => {
                lower_hull.push(lower0);
                lower_hull.push(lower1);
                upper_hull.push(upper);
            }
            TriangleSplit::LowerTwoUpper {
                upper: [upper0, upper1],
                lower,
            } => {
                lower_hull.push(lower);
                upper_hull.push(upper0);
                upper_hull.push(upper1);
            }
        }
    }
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
}

// clean this up
pub(crate) fn intersect_triangle<V: Vertex + Clone>(
    plane: Plane,
    triangle: Triangle<V>,
) -> Option<([V; 2], TriangleSplit<V>)> {
    // TODO: could return a Result<TriangleSplit<V>, Side> instead, returning the side the triangle is on
    // would allow to remove the repeated checks in slice_convex
    // also optimize this in general, lots of branching
    let (ta, tb, tc) = (triangle.a, triangle.b, triangle.c);
    let side_a = plane.classify_side(ta.pos());
    let side_b = plane.classify_side(tb.pos());
    let side_c = plane.classify_side(tc.pos());

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
        if let Some(ip) = intersect_line(plane, &tb, &tc) {
            let a = Triangle::new(ta.clone(), tb, ip.clone());
            let b = Triangle::new(ta.clone(), ip.clone(), tc);
            let (lower, upper) = match side_b {
                Side::Above => (b, a),
                Side::Below => (a, b),
                Side::On => unreachable!(),
            };
            return Some(([ip, ta], TriangleSplit::UpperLower { upper, lower }));
        }
    } else if side_b == Side::On {
        if let Some(ip) = intersect_line(plane, &ta, &tc) {
            let a = Triangle::new(ta, tb.clone(), ip.clone());
            let b = Triangle::new(ip.clone(), tb.clone(), tc);
            let (lower, upper) = match side_a {
                Side::Above => (b, a),
                Side::Below => (a, b),
                Side::On => unreachable!(),
            };
            return Some(([ip, tb], TriangleSplit::UpperLower { upper, lower }));
        }
    } else if side_c == Side::On {
        if let Some(ip) = intersect_line(plane, &ta, &tb) {
            let a = Triangle::new(ta, ip.clone(), tc.clone());
            let b = Triangle::new(ip.clone(), tb, tc.clone());
            let (lower, upper) = match side_a {
                Side::Above => (b, a),
                Side::Below => (a, b),
                Side::On => unreachable!(),
            };
            return Some(([ip, tc], TriangleSplit::UpperLower { upper, lower }));
        }
    // 3 triangles, we cut through two lines in these cases, so one side of the split will be a polygon with 4 edges which has to be split
    } else {
        if side_a != side_b {
            if let Some(ip) = intersect_line(plane, &ta, &tb) {
                if side_a == side_c {
                    if let Some(ip2) = intersect_line(plane, &tb, &tc) {
                        let a = Triangle::new(ip.clone(), tb, ip2.clone());
                        let b = Triangle::new(ta.clone(), ip.clone(), ip2.clone());
                        let c = Triangle::new(ta, ip2.clone(), tc);
                        let split = match side_a {
                            Side::Above => TriangleSplit::LowerTwoUpper {
                                lower: a,
                                upper: [b, c],
                            },
                            Side::Below => TriangleSplit::UpperTwoLower {
                                lower: [b, c],
                                upper: a,
                            },
                            Side::On => unreachable!(),
                        };
                        return Some(([ip, ip2], split));
                    }
                } else if let Some(ip2) = intersect_line(plane, &ta, &tc) {
                    let a = Triangle::new(ta, ip.clone(), ip2.clone());
                    let b = Triangle::new(ip.clone(), tb, tc.clone());
                    let c = Triangle::new(ip2.clone(), ip.clone(), tc);
                    let split = match side_a {
                        Side::Above => TriangleSplit::UpperTwoLower {
                            lower: [b, c],
                            upper: a,
                        },
                        Side::Below => TriangleSplit::LowerTwoUpper {
                            lower: a,
                            upper: [b, c],
                        },
                        Side::On => unreachable!(),
                    };
                    return Some(([ip, ip2], split));
                } else {
                    return None;
                }
            }
        }
        // no match to have lazy logical and-ing
        if let Some(ip) = intersect_line(plane, &tc, &ta) {
            if let Some(ip2) = intersect_line(plane, &tc, &tb) {
                let a = Triangle::new(ip.clone(), ip2.clone(), tc.clone());
                let b = Triangle::new(ta.clone(), ip2.clone(), ip.clone());
                let c = Triangle::new(ta, tb, ip2.clone());
                let split = match side_a {
                    Side::Above => TriangleSplit::LowerTwoUpper {
                        lower: a,
                        upper: [b, c],
                    },
                    Side::Below => TriangleSplit::UpperTwoLower {
                        lower: [b, c],
                        upper: a,
                    },
                    Side::On => unreachable!(),
                };
                return Some(([ip, ip2], split));
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
