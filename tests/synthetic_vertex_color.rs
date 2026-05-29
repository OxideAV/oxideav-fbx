//! Hand-authored binary FBX fixture exercising the full vertex-colour
//! decode path through `FbxDecoder`.
//!
//! Builds a 2-triangle quad with one `LayerElementColor` sub-record
//! (`ByPolygonVertex` / `Direct`, RGBA quadruples) and verifies the
//! per-corner colour buffer reaches `Primitive::colors[0]` after fan
//! triangulation.
//!
//! The colour layer follows the same `MappingInformationType` /
//! `ReferenceInformationType` / `Colors` shape ufbx documents for
//! every `LayerElement*` record in
//! `docs/3d/fbx/ufbx/elements-meshes.md` §"Attributes". The on-disk
//! record name follows the same ufbx-field → FBX-7.x-PascalCase
//! derivation rounds 1–5 used (`vertex_uv` → `LayerElementUV`,
//! `vertex_normal` → `LayerElementNormal`, so `vertex_color` →
//! `LayerElementColor`).

use oxideav_fbx::{FbxDecoder, FBX_MAGIC};
use oxideav_mesh3d::{Mesh3DDecoder, Topology};

const NULL_RECORD_BYTES_32: usize = 13;

struct Rec {
    name: String,
    props: Vec<u8>,
    num_props: u32,
    children: Vec<Rec>,
}

impl Rec {
    fn new(name: &str) -> Self {
        Self {
            name: name.to_owned(),
            props: Vec::new(),
            num_props: 0,
            children: Vec::new(),
        }
    }

    fn with_prop_string(mut self, s: &[u8]) -> Self {
        prop_string(&mut self.props, s);
        self.num_props += 1;
        self
    }

    fn with_prop_i64(mut self, v: i64) -> Self {
        prop_i64(&mut self.props, v);
        self.num_props += 1;
        self
    }

    fn with_prop_f64_array(mut self, arr: &[f64]) -> Self {
        prop_f64_array(&mut self.props, arr);
        self.num_props += 1;
        self
    }

    fn with_prop_i32_array(mut self, arr: &[i32]) -> Self {
        prop_i32_array(&mut self.props, arr);
        self.num_props += 1;
        self
    }

    fn with_child(mut self, child: Rec) -> Self {
        self.children.push(child);
        self
    }
}

fn serialize_node(rec: &Rec, out: &mut Vec<u8>, base_offset: usize) {
    let header_off = out.len();
    out.extend_from_slice(&0u32.to_le_bytes()); // EndOffset placeholder
    out.extend_from_slice(&rec.num_props.to_le_bytes());
    out.extend_from_slice(&(rec.props.len() as u32).to_le_bytes());
    out.push(rec.name.len() as u8);
    out.extend_from_slice(rec.name.as_bytes());
    out.extend_from_slice(&rec.props);
    if !rec.children.is_empty() {
        for child in &rec.children {
            serialize_node(child, out, base_offset);
        }
        out.extend_from_slice(&[0u8; NULL_RECORD_BYTES_32]);
    }
    let end_offset_abs = (base_offset + out.len()) as u32;
    out[header_off..header_off + 4].copy_from_slice(&end_offset_abs.to_le_bytes());
}

fn prop_string(out: &mut Vec<u8>, s: &[u8]) {
    out.push(b'S');
    out.extend_from_slice(&(s.len() as u32).to_le_bytes());
    out.extend_from_slice(s);
}

fn prop_i64(out: &mut Vec<u8>, v: i64) {
    out.push(b'L');
    out.extend_from_slice(&v.to_le_bytes());
}

fn prop_f64_array(out: &mut Vec<u8>, arr: &[f64]) {
    out.push(b'd');
    out.extend_from_slice(&(arr.len() as u32).to_le_bytes());
    out.extend_from_slice(&0u32.to_le_bytes()); // Encoding == 0
    out.extend_from_slice(&0u32.to_le_bytes()); // CompressedLength
    for &v in arr {
        out.extend_from_slice(&v.to_le_bytes());
    }
}

fn prop_i32_array(out: &mut Vec<u8>, arr: &[i32]) {
    out.push(b'i');
    out.extend_from_slice(&(arr.len() as u32).to_le_bytes());
    out.extend_from_slice(&0u32.to_le_bytes());
    out.extend_from_slice(&0u32.to_le_bytes());
    for &v in arr {
        out.extend_from_slice(&v.to_le_bytes());
    }
}

/// One quad with vertex-colour data: red at v0, green at v1, blue at
/// v2, white at v3. The four per-polygon-vertex slots are stored in
/// `Colors` directly (`Direct` reference mode, no `ColorIndex`).
fn build_geometry_rec() -> Rec {
    let vertices = Rec::new("Vertices").with_prop_f64_array(&[
        0.0, 0.0, 0.0, // v0
        1.0, 0.0, 0.0, // v1
        1.0, 1.0, 0.0, // v2
        0.0, 1.0, 0.0, // v3
    ]);
    let pvi = Rec::new("PolygonVertexIndex").with_prop_i32_array(&[0, 1, 2, -4]);
    let color_layer = Rec::new("LayerElementColor")
        .with_prop_i64(0)
        .with_child(Rec::new("MappingInformationType").with_prop_string(b"ByPolygonVertex"))
        .with_child(Rec::new("ReferenceInformationType").with_prop_string(b"Direct"))
        .with_child(Rec::new("Colors").with_prop_f64_array(&[
            1.0, 0.0, 0.0, 1.0, // corner 0 = red (v0)
            0.0, 1.0, 0.0, 1.0, // corner 1 = green (v1)
            0.0, 0.0, 1.0, 1.0, // corner 2 = blue (v2)
            1.0, 1.0, 1.0, 1.0, // corner 3 = white (v3)
        ]));
    Rec::new("Geometry")
        .with_prop_i64(100)
        .with_prop_string(b"Quad\x00\x01Geometry")
        .with_prop_string(b"Mesh")
        .with_child(vertices)
        .with_child(pvi)
        .with_child(color_layer)
}

fn build_model_rec() -> Rec {
    Rec::new("Model")
        .with_prop_i64(200)
        .with_prop_string(b"QuadModel\x00\x01Model")
        .with_prop_string(b"Mesh")
}

fn build_objects_rec() -> Rec {
    Rec::new("Objects")
        .with_child(build_geometry_rec())
        .with_child(build_model_rec())
}

fn build_connection_rec(kind: &[u8], child_id: i64, parent_id: i64) -> Rec {
    Rec::new("C")
        .with_prop_string(kind)
        .with_prop_i64(child_id)
        .with_prop_i64(parent_id)
}

fn build_connections_rec() -> Rec {
    Rec::new("Connections")
        .with_child(build_connection_rec(b"OO", 100, 200))
        .with_child(build_connection_rec(b"OO", 200, 0))
}

fn build_synthetic_fbx() -> Vec<u8> {
    let mut buf = Vec::new();
    buf.extend_from_slice(FBX_MAGIC);
    buf.extend_from_slice(&[0x1A, 0x00]);
    buf.extend_from_slice(&7400u32.to_le_bytes()); // pre-7500 (32-bit headers)
    let base_offset = buf.len();
    let mut body = Vec::new();
    serialize_node(&build_objects_rec(), &mut body, base_offset);
    serialize_node(&build_connections_rec(), &mut body, base_offset);
    body.extend_from_slice(&[0u8; NULL_RECORD_BYTES_32]);
    buf.extend_from_slice(&body);
    buf
}

#[test]
fn synthetic_quad_with_vertex_colors_surfaces_per_corner_rgba() {
    let bytes = build_synthetic_fbx();
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(&bytes).expect("synthetic coloured quad decodes");

    assert_eq!(scene.meshes.len(), 1, "exactly one Mesh");
    let prim = &scene.meshes[0].primitives[0];
    assert_eq!(prim.topology, Topology::Triangles);
    // Quad fan-triangulated -> 2 triangles -> 6 corners.
    assert_eq!(prim.positions.len(), 6);

    // One colour set on the primitive (mirrors `vertex_color` first
    // slot per ufbx §`ufbx_mesh.vertex_color`).
    assert_eq!(prim.colors.len(), 1, "one colour set surfaced");
    let cset = &prim.colors[0];
    assert_eq!(cset.len(), 6, "one RGBA quad per triangle corner");

    // Fan triangulation: (v0,v1,v2) + (v0,v2,v3). With Direct mapping
    // by polygon-vertex, the per-corner ColorIndex picks up the
    // per-PVI slot — so the colour at corner k of triangle t is the
    // Colors entry at PVI[polygon_corner].
    // Triangle 0: PVI[0,1,2] => red, green, blue.
    // Triangle 1: PVI[0,2,3] => red, blue, white.
    assert_eq!(cset[0], [1.0, 0.0, 0.0, 1.0]); // v0 = red
    assert_eq!(cset[1], [0.0, 1.0, 0.0, 1.0]); // v1 = green
    assert_eq!(cset[2], [0.0, 0.0, 1.0, 1.0]); // v2 = blue
    assert_eq!(cset[3], [1.0, 0.0, 0.0, 1.0]); // v0 = red
    assert_eq!(cset[4], [0.0, 0.0, 1.0, 1.0]); // v2 = blue
    assert_eq!(cset[5], [1.0, 1.0, 1.0, 1.0]); // v3 = white
}
