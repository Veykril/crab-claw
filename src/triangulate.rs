use crate::math::{cross, div_v2, dot_v3, negate_v3, normalized, sub_v2};
use crate::{Plane, TextureBounds, Triangle, Vertex};

/// monotone chain algorithm to calculate the convex hull of the vertices
fn monotone_chain<V: Clone>(mut vertices: Vec<(V, [f32; 2])>) -> Vec<(V, [f32; 2])> {
    fn cross_2d(a: [f32; 2], b: [f32; 2], c: [f32; 2]) -> f32 {
        (a[0] - b[0]) * (b[1] - c[1]) - (b[0] - c[0]) * (a[1] - b[1])
    }

    // sort by 2d projection x coord, and y coord if equal
    vertices.sort_by(|(_, a), (_, b)| {
        a[0].partial_cmp(&b[0])
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| a[1].partial_cmp(&b[1]).unwrap_or(std::cmp::Ordering::Equal))
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
fn map_to_2d_with_bb<V: Vertex>(
    plane: Plane,
    vertices: Vec<V>,
) -> (BoundingBox, Vec<(V, [f32; 2])>) {
    // generate the plane from the normal
    let normal = plane.normal();
    let mut plane_u = normalized(cross(normal, [1.0, 1.0, 0.0]));
    // our chosen vector for the cross product might be linearly dependent on the plane normal
    // so choose a different vector that is linear independent to our former chosen one if the cross product didnt work out
    if !plane_u.iter().copied().sum::<f32>().is_normal() {
        plane_u = cross(normal, [0.0, 1.0, 1.0]);
    }
    let plane_v = cross(plane_u, normal);

    let mut minx = f32::MAX;
    let mut miny = f32::MAX;
    let mut maxx = f32::MIN;
    let mut maxy = f32::MIN;

    let mapped = vertices
        .into_iter()
        .map(|vertex| {
            let v2 = [dot_v3(vertex.pos(), plane_u), dot_v3(vertex.pos(), plane_v)];
            minx = minx.min(v2[0]);
            miny = miny.min(v2[1]);
            maxx = maxx.max(v2[0]);
            maxy = maxy.max(v2[1]);
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

/// generate the cross section mesh from the intersection points twice, for each side
#[allow(clippy::type_complexity)]
pub fn triangulate<V: Vertex + Clone>(
    vertices: Vec<V>,
    plane: Plane,
    tb: &TextureBounds,
) -> Option<(Vec<Triangle<V>>, Vec<Triangle<V>>)> {
    if vertices.len() < 3 {
        return None;
    }

    let plane_normal = plane.normal();
    let neg_plane_normal = negate_v3(plane.normal());
    let (bounding_box, mapped) = map_to_2d_with_bb(plane, vertices);

    let mut hull = monotone_chain(mapped);

    let mut lower_cross = Vec::with_capacity(hull.len() - 2);
    let mut upper_cross = Vec::with_capacity(hull.len() - 2);

    let BoundingBox {
        x,
        y,
        width,
        height,
    } = bounding_box;
    let max = [width, height];
    let min = [x, y];
    let tb_map = tb.mapper();
    let (c, uvc) = {
        let (v, uv) = hull.pop().unwrap();
        (v.pos(), tb_map(div_v2(sub_v2(uv, min), max)))
    };
    // FIXME: const generic slice functions
    for vertices in hull.windows(2) {
        let &(ref a, uva) = &vertices[0];
        let &(ref b, uvb) = &vertices[1];
        let (a, uva) = (a.pos(), tb_map(div_v2(sub_v2(uva, min), max)));
        let (b, uvb) = (b.pos(), tb_map(div_v2(sub_v2(uvb, min), max)));
        upper_cross.push(Triangle::new(
            V::new(a, uva, plane_normal),
            V::new(b, uvb, plane_normal),
            V::new(c, uvc, plane_normal),
        ));
        // reversed winding order and normal
        lower_cross.push(Triangle::new(
            V::new(a, uva, neg_plane_normal),
            V::new(c, uvc, neg_plane_normal),
            V::new(b, uvb, neg_plane_normal),
        ));
    }

    Some((lower_cross, upper_cross))
}
