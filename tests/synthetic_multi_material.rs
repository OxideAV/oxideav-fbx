//! End-to-end integration test for round-178 multi-material wiring.
//!
//! Builds a minimal synthetic binary-FBX byte stream with:
//!
//! - Two triangle polygons in `PolygonVertexIndex`.
//! - A `LayerElementMaterial` with `MappingInformationType=ByPolygon`
//!   and `Materials = [0, 1]` — polygon 0 uses slot 0, polygon 1 uses
//!   slot 1.
//! - Two `Material` elements: "Steel" (id 300) and "Wood" (id 301).
//! - Two `Material -> Model` OO connections in slot order
//!   (Steel first, Wood second).
//!
//! After `FbxDecoder::decode` the asserts confirm:
//!
//! - `Primitive::material == Some(MaterialId(0))` (Steel, slot 0)
//!   for back-compat with single-binding consumers.
//! - `Primitive::extras["fbx:material_slots"]` is a 2-element JSON
//!   array `[0, 1]` listing both connected MaterialIds in connection
//!   order.
//! - `Primitive::extras["fbx:face_material_slots"]` is a 6-element
//!   JSON array `[0,0,0,1,1,1]` — three corners per polygon, slot
//!   index broadcast over each triangle.
//! - `Primitive::extras["fbx:material_mapping"]` is `"ByPolygon"`.

use oxideav_fbx::{FbxDecoder, FBX_MAGIC};
use oxideav_mesh3d::Mesh3DDecoder;

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
    out.extend_from_slice(&0u32.to_le_bytes());
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
    out.extend_from_slice(&0u32.to_le_bytes());
    out.extend_from_slice(&0u32.to_le_bytes());
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

fn build_geometry_rec() -> Rec {
    // Two triangles: poly 0 = [0,1,2], poly 1 = [3,4,5].
    let vertices = Rec::new("Vertices").with_prop_f64_array(&[
        0.0, 0.0, 0.0, // v0
        1.0, 0.0, 0.0, // v1
        0.5, 1.0, 0.0, // v2
        2.0, 0.0, 0.0, // v3
        3.0, 0.0, 0.0, // v4
        2.5, 1.0, 0.0, // v5
    ]);
    // Two triangles → end-of-polygon markers at corner indices 2, 5.
    let pvi = Rec::new("PolygonVertexIndex").with_prop_i32_array(&[0, 1, -3, 3, 4, -6]);
    // LayerElementMaterial: ByPolygon, Materials = [0, 1].
    let lem = Rec::new("LayerElementMaterial")
        .with_prop_i64(0)
        .with_child(Rec::new("MappingInformationType").with_prop_string(b"ByPolygon"))
        .with_child(Rec::new("ReferenceInformationType").with_prop_string(b"IndexToDirect"))
        .with_child(Rec::new("Materials").with_prop_i32_array(&[0, 1]));
    Rec::new("Geometry")
        .with_prop_i64(100)
        .with_prop_string(b"TwoMat\x00\x01Geometry")
        .with_prop_string(b"Mesh")
        .with_child(vertices)
        .with_child(pvi)
        .with_child(lem)
}

fn build_model_rec() -> Rec {
    Rec::new("Model")
        .with_prop_i64(200)
        .with_prop_string(b"TwoMatModel\x00\x01Model")
        .with_prop_string(b"Mesh")
}

fn build_material_rec(id: i64, name: &[u8]) -> Rec {
    Rec::new("Material")
        .with_prop_i64(id)
        .with_prop_string(name)
        .with_prop_string(b"")
}

fn build_objects_rec() -> Rec {
    Rec::new("Objects")
        .with_child(build_geometry_rec())
        .with_child(build_model_rec())
        .with_child(build_material_rec(300, b"Steel\x00\x01Material"))
        .with_child(build_material_rec(301, b"Wood\x00\x01Material"))
}

fn build_connection_oo(child_id: i64, parent_id: i64) -> Rec {
    Rec::new("C")
        .with_prop_string(b"OO")
        .with_prop_i64(child_id)
        .with_prop_i64(parent_id)
}

fn build_connections_rec() -> Rec {
    // Material -> Model order = slot order in the per-corner
    // `Materials` array (Steel slot 0, Wood slot 1).
    Rec::new("Connections")
        .with_child(build_connection_oo(100, 200)) // Geometry -> Model
        .with_child(build_connection_oo(200, 0)) // Model -> root
        .with_child(build_connection_oo(300, 200)) // Steel -> Model (slot 0)
        .with_child(build_connection_oo(301, 200)) // Wood -> Model (slot 1)
}

fn build_synthetic_fbx() -> Vec<u8> {
    let mut buf = Vec::new();
    buf.extend_from_slice(FBX_MAGIC);
    buf.extend_from_slice(&[0x1A, 0x00]);
    buf.extend_from_slice(&7400u32.to_le_bytes());
    let base_offset = buf.len();
    let mut body = Vec::new();
    serialize_node(&build_objects_rec(), &mut body, base_offset);
    serialize_node(&build_connections_rec(), &mut body, base_offset);
    body.extend_from_slice(&[0u8; NULL_RECORD_BYTES_32]);
    buf.extend_from_slice(&body);
    buf
}

#[test]
fn multi_material_by_polygon_surfaces_slot_table_and_per_face_indices() {
    let bytes = build_synthetic_fbx();

    let mut dec = FbxDecoder::new();
    let scene = dec
        .decode(&bytes)
        .expect("synthetic-multi-material FBX decodes cleanly");

    assert_eq!(scene.meshes.len(), 1, "exactly one Mesh");
    assert_eq!(scene.materials.len(), 2, "two Material elements surfaced");

    // Steel was connected first, so it lives at slot 0 / MaterialId(0).
    assert_eq!(scene.materials[0].name.as_deref(), Some("Steel"));
    assert_eq!(scene.materials[1].name.as_deref(), Some("Wood"));

    let prim = &scene.meshes[0].primitives[0];

    // Back-compat: single-binding renderers see slot 0.
    assert_eq!(
        prim.material.map(|m| m.0),
        Some(0),
        "single-binding fallback points at slot 0 (Steel)"
    );

    // Slot table on `Primitive::extras` lists both connected MaterialIds.
    let slots = prim
        .extras
        .get("fbx:material_slots")
        .expect("multi-material slot table surfaced on Primitive::extras");
    let slots_arr = slots.as_array().expect("material slots is a JSON array");
    let slot_ids: Vec<u64> = slots_arr
        .iter()
        .map(|v| v.as_u64().expect("slot is an integer"))
        .collect();
    assert_eq!(slot_ids, vec![0, 1], "slot order: Steel then Wood");

    // Per-corner slot indices come from LayerElementMaterial.
    let face_slots = prim
        .extras
        .get("fbx:face_material_slots")
        .expect("per-corner material slot indices surfaced");
    let face_arr = face_slots.as_array().expect("face slots is a JSON array");
    let face_ids: Vec<u64> = face_arr
        .iter()
        .map(|v| v.as_u64().expect("face slot is an integer"))
        .collect();
    // Two triangles, three corners each, polygon 0 -> slot 0, poly 1 -> slot 1.
    assert_eq!(face_ids, vec![0, 0, 0, 1, 1, 1]);

    // Mapping mode preserved as a diagnostic crumb.
    let mapping = prim
        .extras
        .get("fbx:material_mapping")
        .expect("material mapping mode surfaced");
    assert_eq!(mapping.as_str(), Some("ByPolygon"));
}
