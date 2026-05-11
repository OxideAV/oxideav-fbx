//! Hand-authored binary FBX fixture exercising the full decode path.
//!
//! Constructs the exact byte stream documented in
//! `docs/3d/fbx/blender-fbx-binary-format.html` (header + recursive
//! Node Records) carrying:
//!
//! - One `Objects` top-level record with one `Geometry` element
//!   (FBX id 100, `"Quad::Mesh"` Name+SubType pair, subtype `"Mesh"`).
//!   The Geometry has 4 shared vertices arranged as a unit quad in
//!   the XY plane and one polygon spanning all four.
//! - One `Model` element (FBX id 200, subtype `"Mesh"`).
//! - One `Connections` record with two `C` "OO" children:
//!   - Geometry (100) → Model (200) — attribute attachment.
//!   - Model (200) → 0 — root-of-document attachment.
//!
//! The test decodes via `FbxDecoder` and asserts:
//!
//! - The header parses (version we wrote round-trips).
//! - The object graph has exactly one Geometry and one Mesh.
//! - The fan-triangulated polygon yields 6 corner positions
//!   (one quad → two triangles).
//! - The shared-positions extras key carries the original 12-float
//!   FBX vertex buffer.
//! - The Model node has its mesh attribute set to the Mesh.
//! - The scene's roots vec contains the Model node.

use oxideav_fbx::{FbxDecoder, FBX_MAGIC};
use oxideav_mesh3d::{Mesh3DDecoder, Topology};

/// Pre-7500 layout: u32 EndOffset / u32 NumProps / u32 PropListLen.
const NULL_RECORD_BYTES_32: usize = 13;

/// In-memory tree node — gets serialised by `serialize_node` once the
/// whole document is built so absolute file offsets can be computed
/// after all sizes are known.
struct Rec {
    name: String,
    /// Property-list payload bytes (already encoded with `prop_*`).
    props: Vec<u8>,
    /// Number of properties (caller tracks; the property encoders
    /// don't naturally count themselves).
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

/// Serialise a [`Rec`] into `out` at the position implied by
/// `out.len() + base_offset` (i.e. `base_offset` is the offset of
/// `out[0]` in the final file). Recursive — children get their
/// EndOffset values computed relative to the same `base_offset`.
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
        // Terminating NULL-record sentinel for the nested list.
        out.extend_from_slice(&[0u8; NULL_RECORD_BYTES_32]);
    }
    let end_offset_abs = (base_offset + out.len()) as u32;
    out[header_off..header_off + 4].copy_from_slice(&end_offset_abs.to_le_bytes());
}

/// Encode an `S` (string) property: `'S' | u32 length | bytes`.
fn prop_string(out: &mut Vec<u8>, s: &[u8]) {
    out.push(b'S');
    out.extend_from_slice(&(s.len() as u32).to_le_bytes());
    out.extend_from_slice(s);
}

/// Encode an `L` (i64) property: `'L' | i64`.
fn prop_i64(out: &mut Vec<u8>, v: i64) {
    out.push(b'L');
    out.extend_from_slice(&v.to_le_bytes());
}

/// Encode a `d` (f64 array) property uncompressed:
/// `'d' | u32 array_len | u32 encoding=0 | u32 comp_len=0 | f64×N`.
fn prop_f64_array(out: &mut Vec<u8>, arr: &[f64]) {
    out.push(b'd');
    out.extend_from_slice(&(arr.len() as u32).to_le_bytes());
    out.extend_from_slice(&0u32.to_le_bytes()); // Encoding == 0 (uncompressed)
    out.extend_from_slice(&0u32.to_le_bytes()); // CompressedLength (unused when raw)
    for &v in arr {
        out.extend_from_slice(&v.to_le_bytes());
    }
}

/// Encode an `i` (i32 array) property uncompressed.
fn prop_i32_array(out: &mut Vec<u8>, arr: &[i32]) {
    out.push(b'i');
    out.extend_from_slice(&(arr.len() as u32).to_le_bytes());
    out.extend_from_slice(&0u32.to_le_bytes());
    out.extend_from_slice(&0u32.to_le_bytes());
    for &v in arr {
        out.extend_from_slice(&v.to_le_bytes());
    }
}

/// Build one Geometry element record.
fn build_geometry_rec() -> Rec {
    let vertices = Rec::new("Vertices").with_prop_f64_array(&[
        0.0, 0.0, 0.0, // v0
        1.0, 0.0, 0.0, // v1
        1.0, 1.0, 0.0, // v2
        0.0, 1.0, 0.0, // v3
    ]);
    // PolygonVertexIndex: [0, 1, 2, ~3 = -4].
    let pvi = Rec::new("PolygonVertexIndex").with_prop_i32_array(&[0, 1, 2, -4]);
    Rec::new("Geometry")
        .with_prop_i64(100)
        .with_prop_string(b"Quad\x00\x01Geometry")
        .with_prop_string(b"Mesh")
        .with_child(vertices)
        .with_child(pvi)
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

fn build_synthetic_fbx(version: u32) -> Vec<u8> {
    let mut buf = Vec::new();
    // 27-byte header: 20-byte magic + 0x1A 0x00 + version (LE u32).
    buf.extend_from_slice(FBX_MAGIC);
    buf.extend_from_slice(&[0x1A, 0x00]);
    buf.extend_from_slice(&version.to_le_bytes());
    let base_offset = buf.len();

    // Top-level records: Objects, Connections.
    let mut body = Vec::new();
    serialize_node(&build_objects_rec(), &mut body, base_offset);
    serialize_node(&build_connections_rec(), &mut body, base_offset);
    // Final NULL-record sentinel for the top-level list.
    body.extend_from_slice(&[0u8; NULL_RECORD_BYTES_32]);

    buf.extend_from_slice(&body);
    buf
}

#[test]
fn synthetic_quad_decodes_to_one_mesh() {
    let bytes = build_synthetic_fbx(7400); // pre-7500 layout
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(&bytes).expect("synthetic quad decodes");

    // Underlying document captured.
    let doc = dec.last_document.as_ref().expect("document captured");
    assert_eq!(doc.version, 7400);
    assert!(doc.root.child("Objects").is_some());
    assert!(doc.root.child("Connections").is_some());

    // Mesh + Node populated.
    assert_eq!(scene.meshes.len(), 1, "exactly one Mesh");
    assert_eq!(scene.nodes.len(), 1, "exactly one Node");
    assert_eq!(scene.roots.len(), 1, "Model attached to scene root");

    // The Model node carries the Mesh attribute via OO connection.
    let node = &scene.nodes[0];
    assert_eq!(node.name.as_deref(), Some("QuadModel"));
    assert_eq!(node.mesh.map(|m| m.0), Some(0), "Geometry -> Model wired");

    // Mesh has one Triangles primitive with 6 corner positions
    // (quad fan-triangulated -> 2 triangles x 3 corners).
    let mesh = &scene.meshes[0];
    assert_eq!(mesh.name.as_deref(), Some("Quad"));
    assert_eq!(mesh.primitives.len(), 1);
    let prim = &mesh.primitives[0];
    assert_eq!(prim.topology, Topology::Triangles);
    assert_eq!(prim.positions.len(), 6, "quad -> 2 triangles -> 6 corners");

    // Triangles should be (v0, v1, v2) and (v0, v2, v3) per fan.
    assert_eq!(prim.positions[0], [0.0, 0.0, 0.0]); // v0
    assert_eq!(prim.positions[1], [1.0, 0.0, 0.0]); // v1
    assert_eq!(prim.positions[2], [1.0, 1.0, 0.0]); // v2
    assert_eq!(prim.positions[3], [0.0, 0.0, 0.0]); // v0
    assert_eq!(prim.positions[4], [1.0, 1.0, 0.0]); // v2
    assert_eq!(prim.positions[5], [0.0, 1.0, 0.0]); // v3

    // Shared-positions extras key preserves the original 12-float
    // buffer (lives on Primitive::extras since Mesh has no extras
    // slot in the round-1 oxideav-mesh3d API).
    let extras = prim
        .extras
        .get("fbx:shared_positions")
        .expect("shared_positions extras populated");
    let arr = extras.as_array().unwrap();
    assert_eq!(arr.len(), 12, "4 verts x 3 coords");
}

#[test]
fn synthetic_quad_post_7500_layout_decodes() {
    // The synthetic-FBX writer ships only the 32-bit (pre-7500)
    // layout; this test feeds a bare 7700 header followed by EOF
    // and asserts the version sniff fires + the parser gracefully
    // surfaces an empty top-level list.
    let mut buf = Vec::new();
    buf.extend_from_slice(FBX_MAGIC);
    buf.extend_from_slice(&[0x1A, 0x00]);
    buf.extend_from_slice(&7700u32.to_le_bytes());
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(&buf).expect("empty 7700 decodes");
    assert!(scene.meshes.is_empty());
    let doc = dec.last_document.as_ref().unwrap();
    assert_eq!(doc.version, 7700);
}

#[test]
fn ascii_input_returns_unsupported() {
    let ascii = b"; FBX 7.4.0 project file\nFBXHeaderExtension:  {\n";
    let mut dec = FbxDecoder::new();
    let err = dec.decode(ascii).expect_err("ASCII rejected");
    let s = err.to_string();
    assert!(s.contains("ASCII"), "got error: {s}");
}
