//! Hand-authored binary FBX fixtures exercising `LayerElementTangent`
//! / `LayerElementBinormal` decode (round 301).
//!
//! `docs/3d/fbx/fbx-binary-properties70.md` §6 point 4 enumerates
//! `LayerElementTangent` and `LayerElementBinormal` as `Geometry`
//! LayerElement sub-discriminators alongside Normal / UV / Color /
//! Material, each carrying the same `MappingInformationType` /
//! `ReferenceInformationType` string leaves; the
//! `docs/3d/fbx/fbx-ascii-grammar.md` §7c worked example shows the
//! on-disk shape: a `Tangents` (`d`-array, 3-component) triple array
//! plus a companion `TangentsW` (`d`-array, 1-component, per-corner
//! handedness sign), and likewise `Binormals` / `BinormalsW`.
//!
//! `oxideav_mesh3d` stores tangents glTF-style (`[x,y,z,w]` — unit
//! tangent xyz + bitangent-sign w), so the first `LayerElementTangent`
//! populates the canonical `Primitive::tangents`; additional tangent
//! layers ride on `Primitive::extras["fbx:extra_tangents"]`. There is
//! no first-class binormal slot, so every `LayerElementBinormal`
//! surfaces on `Primitive::extras["fbx:binormals"]`.
//!
//! Test 1 — single tangent layer with a mix of `+1.0` / `-1.0`
//! handedness signs populates `Primitive::tangents` with the W applied.
//! Test 2 — a tangent layer with no `TangentsW` defaults every sign to
//! `+1.0`.
//! Test 3 — two tangent layers: first canonical, second in extras.
//! Test 4 — a binormal layer surfaces on `fbx:binormals` with W.

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

// A unit quad in the XY plane: 4 shared vertices, one polygon spanning
// all four. Fan-triangulates to 2 triangles / 6 corners. PVI order is
// v0,v1,v2,v3 → fan = (v0,v1,v2),(v0,v2,v3).
const QUAD_VERTICES: &[f64] = &[
    0.0, 0.0, 0.0, // v0
    1.0, 0.0, 0.0, // v1
    1.0, 1.0, 0.0, // v2
    0.0, 1.0, 0.0, // v3
];
const QUAD_PVI: &[i32] = &[0, 1, 2, !3];

// ByPolygonVertex / Direct: one tangent per polygon-vertex corner.
// Four corners → 12 floats. Distinct per-corner xyz so we can verify
// the per-corner ordering.
const TANGENTS_BY_PV: &[f64] = &[
    1.0, 0.0, 0.0, // corner 0
    1.0, 0.0, 0.0, // corner 1
    1.0, 0.0, 0.0, // corner 2
    1.0, 0.0, 0.0, // corner 3
];
// Per-corner handedness signs: corner 2 is left-handed (-1).
const TANGENTS_W: &[f64] = &[1.0, 1.0, -1.0, 1.0];

const BINORMALS_BY_PV: &[f64] = &[
    0.0, 1.0, 0.0, // corner 0
    0.0, 1.0, 0.0, // corner 1
    0.0, 1.0, 0.0, // corner 2
    0.0, 1.0, 0.0, // corner 3
];
const BINORMALS_W: &[f64] = &[1.0, 1.0, 1.0, 1.0];

fn tangent_layer(typed_index: i64, tangents: &[f64], w: Option<&[f64]>) -> Rec {
    let mut layer = Rec::new("LayerElementTangent")
        .with_prop_i64(typed_index)
        .with_child(Rec::new("MappingInformationType").with_prop_string(b"ByPolygonVertex"))
        .with_child(Rec::new("ReferenceInformationType").with_prop_string(b"Direct"))
        .with_child(Rec::new("Tangents").with_prop_f64_array(tangents));
    if let Some(w) = w {
        layer = layer.with_child(Rec::new("TangentsW").with_prop_f64_array(w));
    }
    layer
}

fn binormal_layer(typed_index: i64, binormals: &[f64], w: Option<&[f64]>) -> Rec {
    let mut layer = Rec::new("LayerElementBinormal")
        .with_prop_i64(typed_index)
        .with_child(Rec::new("MappingInformationType").with_prop_string(b"ByPolygonVertex"))
        .with_child(Rec::new("ReferenceInformationType").with_prop_string(b"Direct"))
        .with_child(Rec::new("Binormals").with_prop_f64_array(binormals));
    if let Some(w) = w {
        layer = layer.with_child(Rec::new("BinormalsW").with_prop_f64_array(w));
    }
    layer
}

fn build_objects(layers: Vec<Rec>) -> Rec {
    let mut geom = Rec::new("Geometry")
        .with_prop_i64(100)
        .with_prop_string(b"Quad\x00\x01Geometry")
        .with_prop_string(b"Mesh")
        .with_child(Rec::new("Vertices").with_prop_f64_array(QUAD_VERTICES))
        .with_child(Rec::new("PolygonVertexIndex").with_prop_i32_array(QUAD_PVI));
    for layer in layers {
        geom = geom.with_child(layer);
    }
    Rec::new("Objects").with_child(geom).with_child(
        Rec::new("Model")
            .with_prop_i64(200)
            .with_prop_string(b"QuadModel\x00\x01Model")
            .with_prop_string(b"Mesh"),
    )
}

fn build_connections() -> Rec {
    Rec::new("Connections")
        .with_child(
            Rec::new("C")
                .with_prop_string(b"OO")
                .with_prop_i64(100)
                .with_prop_i64(200),
        )
        .with_child(
            Rec::new("C")
                .with_prop_string(b"OO")
                .with_prop_i64(200)
                .with_prop_i64(0),
        )
}

fn build_synthetic_fbx(layers: Vec<Rec>) -> Vec<u8> {
    let mut buf = Vec::new();
    buf.extend_from_slice(FBX_MAGIC);
    buf.extend_from_slice(&[0x1A, 0x00]);
    buf.extend_from_slice(&7400u32.to_le_bytes());
    let base_offset = buf.len();
    let mut body = Vec::new();
    serialize_node(&build_objects(layers), &mut body, base_offset);
    serialize_node(&build_connections(), &mut body, base_offset);
    body.extend_from_slice(&[0u8; NULL_RECORD_BYTES_32]);
    buf.extend_from_slice(&body);
    buf
}

#[test]
fn tangent_layer_populates_prim_tangents_with_w_sign() {
    let bytes = build_synthetic_fbx(vec![tangent_layer(0, TANGENTS_BY_PV, Some(TANGENTS_W))]);
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(&bytes).expect("tangent quad decodes");
    let prim = &scene.meshes[0].primitives[0];

    // One quad fan-triangulates to 2 triangles → 6 corners.
    assert_eq!(prim.positions.len(), 6);
    let tangents = prim.tangents.as_ref().expect("tangents surfaced");
    assert_eq!(tangents.len(), 6, "one tangent per triangle corner");

    // Quad pvi = [v0,v1,v2,v3]; fan = (v0,v1,v2),(v0,v2,v3).
    // corner_pvi_index per corner: 0,1,2, 0,2,3.
    // TANGENTS_W indexed by pvi: [1, 1, -1, 1, -1, 1].
    let expected_w = [1.0_f32, 1.0, -1.0, 1.0, -1.0, 1.0];
    for (t, &ew) in tangents.iter().zip(expected_w.iter()) {
        assert_eq!([t[0], t[1], t[2]], [1.0, 0.0, 0.0]);
        assert!((t[3] - ew).abs() < 1e-6, "expected w {ew}, got {}", t[3]);
    }
    assert!(!prim.extras.contains_key("fbx:extra_tangents"));
}

#[test]
fn tangent_layer_without_w_defaults_sign_to_plus_one() {
    let bytes = build_synthetic_fbx(vec![tangent_layer(0, TANGENTS_BY_PV, None)]);
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(&bytes).expect("tangent quad decodes");
    let prim = &scene.meshes[0].primitives[0];

    let tangents = prim.tangents.as_ref().expect("tangents surfaced");
    for t in tangents {
        assert!((t[3] - 1.0).abs() < 1e-6, "default +1.0 sign");
    }
}

#[test]
fn two_tangent_layers_first_canonical_rest_in_extras() {
    let bytes = build_synthetic_fbx(vec![
        tangent_layer(0, TANGENTS_BY_PV, Some(TANGENTS_W)),
        tangent_layer(7, TANGENTS_BY_PV, None),
    ]);
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(&bytes).expect("two-tangent quad decodes");
    let prim = &scene.meshes[0].primitives[0];

    // Canonical slot keeps layer 0 with its real signs.
    assert!(prim.tangents.is_some());

    let extra = prim
        .extras
        .get("fbx:extra_tangents")
        .and_then(|v| v.as_array())
        .expect("fbx:extra_tangents present");
    assert_eq!(extra.len(), 1, "exactly one extra tangent layer");
    // 6 corners * 4 components.
    assert_eq!(extra[0].as_array().unwrap().len(), 24);

    let ti = prim
        .extras
        .get("fbx:extra_tangents_typed_index")
        .and_then(|v| v.as_array())
        .and_then(|a| a.first())
        .and_then(|v| v.as_i64());
    assert_eq!(ti, Some(7));

    let mapping = prim
        .extras
        .get("fbx:extra_tangents_mapping")
        .and_then(|v| v.as_array())
        .and_then(|a| a.first())
        .and_then(|v| v.as_str());
    assert_eq!(mapping, Some("ByPolygonVertex"));
}

#[test]
fn binormal_layer_surfaces_on_extras_with_w() {
    let bytes = build_synthetic_fbx(vec![binormal_layer(0, BINORMALS_BY_PV, Some(BINORMALS_W))]);
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(&bytes).expect("binormal quad decodes");
    let prim = &scene.meshes[0].primitives[0];

    // No first-class binormal slot — must NOT touch tangents.
    assert!(prim.tangents.is_none());

    let binormals = prim
        .extras
        .get("fbx:binormals")
        .and_then(|v| v.as_array())
        .expect("fbx:binormals present");
    assert_eq!(binormals.len(), 1);
    let buf = binormals[0].as_array().unwrap();
    assert_eq!(buf.len(), 24, "6 corners * 4 components");
    // First corner = (0,1,0,1).
    assert_eq!(buf[0].as_f64().unwrap(), 0.0);
    assert_eq!(buf[1].as_f64().unwrap(), 1.0);
    assert_eq!(buf[2].as_f64().unwrap(), 0.0);
    assert_eq!(buf[3].as_f64().unwrap(), 1.0);

    let mapping = prim
        .extras
        .get("fbx:binormals_mapping")
        .and_then(|v| v.as_array())
        .and_then(|a| a.first())
        .and_then(|v| v.as_str());
    assert_eq!(mapping, Some("ByPolygonVertex"));
}
