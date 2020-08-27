mod triangle;
pub use self::triangle::Triangle;

mod math;
mod triangulate;
use self::math::{Vec2, Vec3};
mod plane;
pub use self::plane::Plane;
use self::plane::Side;
use triangle::intersect_triangle;
use triangulate::triangulate;

const EPSILON: f32 = 1e-7;

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

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct TextureBounds {
    x_min: f32,
    y_min: f32,
    x_max: f32,
    y_max: f32,
}

impl TextureBounds {
    pub fn new(x_min: f32, y_min: f32, x_max: f32, y_max: f32) -> Self {
        TextureBounds {
            x_min,
            y_min,
            x_max,
            y_max,
        }
    }

    /// returns a function that maps a vec2 in range [0;1] to the bounds of this texture
    fn mapper(&self) -> impl Fn(Vec2) -> Vec2 {
        let diffx = self.x_max - self.x_min;
        let diffy = self.y_max - self.y_min;
        let x_min = self.x_min;
        let y_min = self.y_min;
        move |uv| Vec2::new(uv.x * diffx + x_min, uv.y * diffy + y_min)
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

pub fn slice_convex<V: Vertex>(
    triangles: impl IntoIterator<Item = Triangle<V>>,
    plane: Plane,
    texture_bounds: TextureBounds,
) -> Option<(SubMesh<V>, SubMesh<V>)> {
    let triangles = triangles.into_iter();
    let mut upper = Vec::with_capacity(triangles.size_hint().0);
    let mut lower = Vec::with_capacity(triangles.size_hint().0);
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

            // the plane didnt intersect this triangle, figure out into what hull to put it
            let side = if side_a != Side::On {
                side_a
            } else if side_b != Side::On {
                side_b
            } else if side_c != Side::On {
                side_c
            } else {
                Side::On
            };
            if let Side::Above | Side::On = side {
                upper.push(triangle);
            } else {
                lower.push(triangle);
            }
        }
    }

    if !(upper.is_empty() || lower.is_empty()) {
        let cross = triangulate(cross, plane, &texture_bounds)
            // only happens if we didnt gather enough vertices to form a triangle
            .unwrap_or_default();
        let mut cross_lower = cross.clone();
        cross_lower.iter_mut().for_each(|trig| {
            trig.flip_normals(); // TODO: check whether flipping here is correct
            trig.reverse_winding();
        });
        Some((
            SubMesh {
                hull: upper,
                cross_section: cross,
            },
            SubMesh {
                hull: lower,
                cross_section: cross_lower,
            },
        ))
    } else {
        // no slicing occured
        None
    }
}

/// Trait to be implemented by vertices for slicing
pub trait Vertex: Clone + Sized {
    fn new_interpolated(a: &Self, b: &Self, t: f32) -> Self;

    fn new(pos: Vec3, uv: Vec2, normal: Vec3) -> Self;

    fn pos(&self) -> Vec3;

    fn flip_normal(&mut self);
}
