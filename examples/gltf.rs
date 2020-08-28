use crab_claw::{triangle_to_vertex, vertex_to_triangle, Triangle};
use genmesh::Triangulate;
use genmesh::Vertices;
use std::{fs, io::Write, mem};

fn main() {
    let cone = genmesh::generators::IcoSphere::subdivide(3);
    let f: Vec<Triangle<Vertex>> =
        vertex_to_triangle(cone.triangulate().vertices().map(Into::into)).collect();
    let s = crab_claw::slice_convex(
        f,
        crab_claw::Plane::from_pos_normal([0.0; 3], [1.0, 0.2, 0.0]),
        Default::default(),
    )
    .unwrap();

    export("upper", triangle_to_vertex(s.0.hull));
    export("upperc", triangle_to_vertex(s.0.cross_section));
    export("lower", triangle_to_vertex(s.1.hull));
    export("lowerc", triangle_to_vertex(s.1.cross_section));
}

#[derive(Clone)]
struct Vertex {
    position: [f32; 3],
    normal: [f32; 3],
}

impl crab_claw::Vertex for Vertex {
    fn new_interpolated(a: &Self, b: &Self, t: f32) -> Self {
        let [ax, ay, az] = a.pos();
        let [bx, by, bz] = b.pos();
        let [anx, any, anz] = a.normal;
        let [bnx, bny, bnz] = b.normal;
        Vertex {
            position: [ax + t * (bx - ax), ay + t * (by - ay), az + t * (bz - az)],
            normal: [
                anx + t * (bnx - anx),
                any + t * (bny - any),
                anz + t * (bnz - anz),
            ],
        }
    }

    fn new(position: [f32; 3], _: [f32; 2], normal: [f32; 3]) -> Self {
        Vertex { position, normal }
    }

    fn pos(&self) -> [f32; 3] {
        self.position
    }

    fn flip_normal(&mut self) {
        self.normal = [-self.normal[0], -self.normal[1], -self.normal[2]];
    }
}

impl From<genmesh::Vertex> for Vertex {
    fn from(g: genmesh::Vertex) -> Self {
        Vertex {
            position: g.pos.into(),
            normal: g.normal.into(),
        }
    }
}

use gltf::json::{self, validation::Checked::Valid};
fn export(name: &str, iter: impl Iterator<Item = Vertex>) {
    let buf_uri = format!("{}-buffer0.bin", name);
    let vertices = iter.collect::<Vec<_>>();
    let buffer_length = (vertices.len() * mem::size_of::<Vertex>()) as u32;
    let buffer = json::Buffer {
        byte_length: buffer_length,
        extensions: Default::default(),
        extras: Default::default(),
        name: None,
        uri: Some(buf_uri.clone()),
    };
    let buffer_view = json::buffer::View {
        buffer: json::Index::new(0),
        byte_length: buffer.byte_length,
        byte_offset: None,
        byte_stride: Some(mem::size_of::<Vertex>() as u32),
        extensions: Default::default(),
        extras: Default::default(),
        name: None,
        target: Some(Valid(json::buffer::Target::ArrayBuffer)),
    };
    let positions = json::Accessor {
        buffer_view: Some(json::Index::new(0)),
        byte_offset: 0,
        count: vertices.len() as u32,
        component_type: Valid(json::accessor::GenericComponentType(
            json::accessor::ComponentType::F32,
        )),
        extensions: Default::default(),
        extras: Default::default(),
        type_: Valid(json::accessor::Type::Vec3),
        min: None,
        max: None,
        name: None,
        normalized: false,
        sparse: None,
    };
    let normals = json::Accessor {
        buffer_view: Some(json::Index::new(0)),
        byte_offset: (3 * mem::size_of::<f32>()) as u32,
        count: vertices.len() as u32,
        component_type: Valid(json::accessor::GenericComponentType(
            json::accessor::ComponentType::F32,
        )),
        extensions: Default::default(),
        extras: Default::default(),
        type_: Valid(json::accessor::Type::Vec3),
        min: None,
        max: None,
        name: None,
        normalized: false,
        sparse: None,
    };

    let primitive = json::mesh::Primitive {
        attributes: {
            let mut map = std::collections::HashMap::new();
            map.insert(Valid(json::mesh::Semantic::Positions), json::Index::new(0));
            map.insert(Valid(json::mesh::Semantic::Normals), json::Index::new(1));
            map
        },
        extensions: Default::default(),
        extras: Default::default(),
        indices: None,
        material: None,
        mode: Valid(json::mesh::Mode::Triangles),
        targets: None,
    };

    let mesh = json::Mesh {
        extensions: Default::default(),
        extras: Default::default(),
        name: None,
        primitives: vec![primitive],
        weights: None,
    };

    let node = json::Node {
        camera: None,
        children: None,
        extensions: Default::default(),
        extras: Default::default(),
        matrix: None,
        mesh: Some(json::Index::new(0)),
        name: None,
        rotation: None,
        scale: None,
        translation: None,
        skin: None,
        weights: None,
    };

    let root = json::Root {
        accessors: vec![positions, normals],
        buffers: vec![buffer],
        buffer_views: vec![buffer_view],
        meshes: vec![mesh],
        nodes: vec![node],
        scenes: vec![json::Scene {
            extensions: Default::default(),
            extras: Default::default(),
            name: None,
            nodes: vec![json::Index::new(0)],
        }],
        ..Default::default()
    };

    let _ = fs::create_dir("test");

    let writer = fs::File::create(format!("test/{}.gltf", name)).expect("I/O error");
    json::serialize::to_writer_pretty(writer, &root).expect("Serialization error");

    let bin = to_padded_byte_vector(vertices);
    let mut writer = fs::File::create(format!("test/{}", buf_uri)).expect("I/O error");
    writer.write_all(&bin).expect("I/O error");
}

fn to_padded_byte_vector<T>(vec: Vec<T>) -> Vec<u8> {
    let byte_length = vec.len() * mem::size_of::<T>();
    let byte_capacity = vec.capacity() * mem::size_of::<T>();
    let alloc = vec.into_boxed_slice();
    let ptr = Box::<[T]>::into_raw(alloc) as *mut u8;
    let mut new_vec = unsafe { Vec::from_raw_parts(ptr, byte_length, byte_capacity) };
    while new_vec.len() % 4 != 0 {
        new_vec.push(0); // pad to multiple of four bytes
    }
    new_vec
}
