mod triangle;
pub use self::triangle::Triangle;

mod math;
use self::math::{Vec2, Vec3};
mod plane;
pub use self::plane::Plane;
use self::plane::Side;
use triangle::intersect_triangle;

const EPSILON: f32 = 1e-7;

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum HullKind {
    Upper,
    Lower,
}

pub fn vertex_to_triangle<'a, V: Vertex, I: IntoIterator<Item = V> + 'a>(
    i: I,
) -> impl Iterator<Item = Triangle<V>> + 'a {
    let mut iter = i.into_iter();
    std::iter::from_fn(move || {
        let a = iter.next()?;
        let b = iter.next().unwrap();
        let c = iter.next().unwrap();
        Some(Triangle { a, b, c })
    })
}

pub fn triangle_to_vertex<'a, V: Vertex + 'a, I: IntoIterator<Item = Triangle<V>> + 'a>(
    i: I,
) -> impl Iterator<Item = V> + 'a {
    i.into_iter()
        .flat_map(|triangle| vec![triangle.a, triangle.b, triangle.c])
}

pub struct TextureBounds {
    x_min: f32,
    y_min: f32,
    x_max: f32,
    y_max: f32,
}

impl TextureBounds {
    fn map(&self, uv: Vec2) -> Vec2 {
        Vec2::new(
            uv.x * (self.x_max - self.x_min) + self.x_min,
            uv.y * (self.y_max - self.y_min) + self.y_min,
        )
    }
}

impl Default for TextureBounds {
    fn default() -> Self {
        TextureBounds {
            x_min: 0.0,
            y_min: 0.0,
            x_max: 1.0,
            y_max: 1.0,
        }
    }
}

pub struct SubMesh<V> {
    pub hull: Vec<Triangle<V>>,
    pub cross_section: Vec<Triangle<V>>,
}

pub fn slice<V: Vertex>(
    triangles: impl IntoIterator<Item = Triangle<V>>,
    plane: Plane,
    texture_bounds: TextureBounds,
) -> Option<(SubMesh<V>, SubMesh<V>)> {
    let mut upper = vec![];
    let mut lower = vec![];
    let mut cross = vec![];

    for triangle in triangles {
        if let Some(mut split) = intersect_triangle(plane, triangle.clone()) {
            upper.extend(split.hull.drain(split.lower_split..));
            lower.extend(split.hull);
            // FIXME: IntoIter for Arrays when
            cross.extend(split.points.iter().cloned());
        } else {
            let side_a = plane.classify_side(triangle.a.pos());
            let side_b = plane.classify_side(triangle.b.pos());
            let side_c = plane.classify_side(triangle.c.pos());
            // what is this????
            let side = if side_a != Side::On { side_a } else { Side::On };
            let side = if side_b != Side::On {
                debug_assert!(side == Side::On || side == side_b);
                side_b
            } else {
                side
            };
            let side = if side_c != Side::On {
                debug_assert!(side == Side::On || side_c == side_b);
                side_c
            } else {
                side
            };
            if let Side::Above | Side::On = side {
                upper.push(triangle);
            } else {
                lower.push(triangle);
            }
        }
    }

    if !(upper.is_empty() || lower.is_empty()) {
        let cross = gen_cross_hull(cross, plane, &texture_bounds).unwrap_or_default();
        let (upper, upper_cross) = create_hull(upper, &cross, HullKind::Upper);
        let (lower, lower_cross) = create_hull(lower, &cross, HullKind::Lower);
        Some((
            SubMesh {
                hull: upper,
                cross_section: upper_cross,
            },
            SubMesh {
                hull: lower,
                cross_section: lower_cross,
            },
        ))
    } else {
        // no slicing occured
        None
    }
}

fn create_hull<V: Vertex>(
    hull: Vec<Triangle<V>>,
    cross: &[Triangle<V>],
    hull_kind: HullKind,
) -> (Vec<Triangle<V>>, Vec<Triangle<V>>) {
    let mut cross_hull = cross.to_vec();
    if hull_kind == HullKind::Lower {
        cross_hull.iter_mut().for_each(|trig| {
            trig.flip_normals(); // TODO: check normals
            trig.reverse_winding();
        });
    }
    (hull, cross_hull)
}

/// montone chain algorithm, assumes vertices are ordered by the second tuple element
fn monotone_chain<V: Clone>(vertices: Vec<(V, Vec2)>) -> Vec<(V, Vec2)> {
    fn cross_2d(a: Vec2, b: Vec2, c: Vec2) -> f32 {
        (a.x - b.x) * (b.y - c.y) - (b.x - c.x) * (a.y - b.y)
    }

    let mut hull: Vec<(_, _)> = Vec::with_capacity(vertices.len() / 2);
    // lower
    for mapped in vertices.iter().cloned() {
        while {
            let len = hull.len();
            len >= 2 && cross_2d(hull[len - 2].1, hull[len - 1].1, mapped.1) <= 0.0
        } {
            hull.pop();
        }
        hull.push(mapped)
    }

    let offset = hull.len() + 2;
    for mapped in vertices.into_iter().rev() {
        while {
            let len = hull.len();
            len >= offset && cross_2d(hull[len - 2].1, hull[len - 1].1, mapped.1) <= 0.0
        } {
            hull.pop();
        }
        hull.push(mapped)
    }
    hull.pop(); // repeated elements
    hull.remove(offset - 2); // repeated elements
    debug_assert!(hull.len() >= 3);
    hull
}

struct BoundingBox {
    x: f32,
    y: f32,
    width: f32,
    height: f32,
}

// Map the vertices onto the cutting plane, calculating the bounding box
fn map_to_2d_with_bb<V: Vertex>(plane: Plane, vertices: Vec<V>) -> (BoundingBox, Vec<(V, Vec2)>) {
    // generate the plane from the normal
    let normal = plane.normal();
    let mut plane_u = normal.cross(Vec3::new(1.0, 1.0, 0.0)).normalized();
    // our chosen vector for the cross product might be linearly dependent on the plane normal
    // so choose a different vector that is linear independent to our former chosen one if the cross product didnt work out
    if !plane_u.as_array().iter().copied().sum::<f32>().is_normal() {
        plane_u = normal.cross(Vec3::new(0.0, 1.0, 1.0));
    }
    let plane_v = plane_u.cross(normal);

    let mut minx = f32::MAX;
    let mut miny = f32::MAX;
    let mut maxx = f32::MIN;
    let mut maxy = f32::MIN;

    let mapped = vertices
        .into_iter()
        .map(|vertex| {
            let v2 = Vec2::new(vertex.pos().dot(plane_u), vertex.pos().dot(plane_v));
            minx = minx.min(v2.x);
            miny = miny.min(v2.y);
            maxx = maxx.max(v2.x);
            maxy = maxy.max(v2.y);
            (vertex, v2)
        })
        .collect::<Vec<_>>();
    (
        BoundingBox {
            x: minx,
            y: miny,
            width: maxx - minx,
            height: maxy - miny,
        },
        mapped,
    )
}

/// generate the cross section mesh from the intersection points
fn gen_cross_hull<V: Vertex>(
    vertices: Vec<V>,
    plane: Plane,
    tb: &TextureBounds,
) -> Option<Vec<Triangle<V>>> {
    if vertices.len() < 3 {
        return None;
    }

    let plane_normal = plane.normal();
    let (bounding_box, mut mapped) = map_to_2d_with_bb(plane, vertices);

    // sort by 2d projection x coord, and y coord if equal
    mapped.sort_by(|(_, a), (_, b)| {
        a.x.partial_cmp(&b.x)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| a.y.partial_cmp(&b.y).unwrap_or(std::cmp::Ordering::Equal))
    });

    let hull = monotone_chain(mapped);

    let tri_count = (hull.len() - 2) * 3;
    let mut triangles = Vec::with_capacity(tri_count);

    let BoundingBox {
        x,
        y,
        width,
        height,
    } = bounding_box;
    // FIXME: const generic slice functions should be able to help here
    for vertices in hull.windows(3) {
        // const generics would clean up this mess as well
        let [(a, uva), (b, uvb), (c, uvc)] =
            if let [(ref a, uva), (ref b, uvb), (ref c, uvc), ..] = *vertices {
                [(a.pos(), uva), (b.pos(), uvb), (c.pos(), uvc)]
            } else {
                unreachable!()
            };
        let uva = (uva - Vec2::new(x, y)) / Vec2::new(width, height);
        let uvb = (uvb - Vec2::new(x, y)) / Vec2::new(width, height);
        let uvc = (uvc - Vec2::new(x, y)) / Vec2::new(width, height);
        let triangle = Triangle::new(
            V::new(a, tb.map(uva), plane_normal),
            V::new(b, tb.map(uvb), plane_normal),
            V::new(c, tb.map(uvc), plane_normal),
        );
        triangles.push(triangle);
    }

    Some(triangles)
}

pub trait Vertex: Clone + Sized {
    fn new_interpolated(a: &Self, b: &Self, t: f32) -> Self;

    fn new(pos: Vec3, uv: Vec2, normal: Vec3) -> Self;

    fn pos(&self) -> Vec3;

    fn flip_normal(&mut self);
}
