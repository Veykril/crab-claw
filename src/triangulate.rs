use crate::math::{Vec2, Vec3};
use crate::{Plane, TextureBounds, Triangle, Vertex};

/// monotone chain algorithm to calculate the convex hull of the vertices
fn monotone_chain<V: Clone>(mut vertices: Vec<(V, Vec2)>) -> Vec<(V, Vec2)> {
    fn cross_2d(a: Vec2, b: Vec2, c: Vec2) -> f32 {
        (a.x - b.x) * (b.y - c.y) - (b.x - c.x) * (a.y - b.y)
    }

    // sort by 2d projection x coord, and y coord if equal
    vertices.sort_by(|(_, a), (_, b)| {
        a.x.partial_cmp(&b.x)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| a.y.partial_cmp(&b.y).unwrap_or(std::cmp::Ordering::Equal))
    });

    let mut hull: Vec<(_, _)> = Vec::with_capacity(vertices.len() / 2);

    // lower hull
    for mapped in vertices.iter().cloned() {
        while {
            let len = hull.len();
            len >= 2 && cross_2d(hull[len - 2].1, hull[len - 1].1, mapped.1) <= 0.0
        } {
            hull.pop();
        }
        hull.push(mapped);
    }

    // upper hull
    let offset = hull.len() + 2;
    for mapped in vertices.into_iter().rev() {
        while {
            let len = hull.len();
            len >= offset && cross_2d(hull[len - 2].1, hull[len - 1].1, mapped.1) <= 0.0
        } {
            hull.pop();
        }
        hull.push(mapped);
    }
    hull.pop(); // duplicate of hull[0]
    hull.remove(offset - 2); // duplicate of hull[offset - 1]
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
pub fn triangulate<V: Vertex>(
    vertices: Vec<V>,
    plane: Plane,
    tb: &TextureBounds,
) -> Option<Vec<Triangle<V>>> {
    if vertices.len() < 3 {
        return None;
    }

    let plane_normal = plane.normal();
    let (bounding_box, mapped) = map_to_2d_with_bb(plane, vertices);

    let mut hull = monotone_chain(mapped);

    let mut triangles = Vec::with_capacity(hull.len() - 2);

    let BoundingBox {
        x,
        y,
        width,
        height,
    } = bounding_box;
    let max = Vec2::new(width, height);
    let min = Vec2::new(x, y);
    let tb_map = tb.mapper();
    let p = {
        let (v, uv) = hull.pop().unwrap();
        V::new(v.pos(), tb_map((uv - min) / max), plane_normal)
    };
    // FIXME: const generic slice functions
    for vertices in hull.windows(2) {
        let &(ref a, uva) = &vertices[0];
        let &(ref b, uvb) = &vertices[1];
        triangles.push(Triangle::new(
            V::new(a.pos(), tb_map((uva - min) / max), plane_normal),
            V::new(b.pos(), tb_map((uvb - min) / max), plane_normal),
            p.clone(),
        ));
    }

    Some(triangles)
}
