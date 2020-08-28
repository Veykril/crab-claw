mod triangle;
use self::triangle::intersect_triangle;
pub use self::triangle::Triangle;

mod math;
pub use self::math::{lerp2, lerp3};

mod plane;
pub use self::plane::Plane;
use self::plane::Side;

mod triangulate;
use self::triangulate::triangulate;

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
        // FIXME: IntoIter for Arrays when
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
    #[inline]
    pub fn new(x_min: f32, y_min: f32, x_max: f32, y_max: f32) -> Self {
        TextureBounds {
            x_min,
            y_min,
            x_max,
            y_max,
        }
    }

    /// returns a function that maps a vec2 in range [0;1] to the bounds of this texture
    #[inline]
    fn mapper(&self) -> impl Fn([f32; 2]) -> [f32; 2] {
        let diffx = self.x_max - self.x_min;
        let diffy = self.y_max - self.y_min;
        let x_min = self.x_min;
        let y_min = self.y_min;
        move |[x, y]| [x * diffx + x_min, y * diffy + y_min]
    }
}

impl Default for TextureBounds {
    #[inline]
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

pub fn slice_convex<V: Vertex + Clone>(
    triangles: impl IntoIterator<Item = Triangle<V>>,
    plane: Plane,
    texture_bounds: TextureBounds,
) -> Option<(SubMesh<V>, SubMesh<V>)> {
    let triangles = triangles.into_iter();
    let mut upper = Vec::with_capacity(triangles.size_hint().0);
    let mut lower = Vec::with_capacity(triangles.size_hint().0);
    let mut cross = vec![];

    for triangle in triangles {
        if let Some((points, split)) = intersect_triangle(plane, triangle.clone()) {
            split.append_to(&mut lower, &mut upper);
            // FIXME: IntoIter for Arrays when
            cross.extend(points.iter().cloned());
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
        let (lower_cross, upper_cross) = triangulate(cross, plane, &texture_bounds)
            // only happens if we didnt gather enough vertices to form a triangle
            .unwrap_or_default();

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

/// Trait to be implemented by vertices for slicing
pub trait Vertex: Sized {
    /// Creates a new vertex that will lie between the two given ones where t is a value between 0.0 and 1.0.
    /// This will be called to create the vertices that lie on the cutting plane where the mesh intersects with the plane.
    fn new_interpolated(a: &Self, b: &Self, t: f32) -> Self;

    /// Create a new vertex from the given position, uv and normal.
    /// This is solely used when constructing the cross section and will therefor be called twice per vertex
    /// with opposing normals.
    fn new(pos: [f32; 3], uv: [f32; 2], normal: [f32; 3]) -> Self;

    /// Retrieves the position of this vertex.
    fn pos(&self) -> [f32; 3];
}

/*
pub trait VertexConstructor<V> {
    fn vertex(&mut self, pos: [f32; 3], uv: [f32; 2], normal: [f32; 3]) -> V;
    fn interpolated(&mut self, a: &V, b: &V, t: f32) -> V;
}

impl<V, VB: VertexConstructor<V>> VertexConstructor<V> for &mut VB {
    fn vertex(&mut self, pos: [f32; 3], uv: [f32; 2], normal: [f32; 3]) -> V {
        VB::vertex(self, pos, uv, normal)
    }

    fn interpolated(&mut self, a: &V, b: &V, t: f32) -> V {
        VB::interpolated(self, a, b, t)
    }
}
*/
