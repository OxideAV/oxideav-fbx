//! Hand-authored binary FBX fixtures exercising multi-layer
//! `LayerElementNormal` decode.
//!
//! A `Geometry` element may carry more than one `LayerElementNormal`
//! record, each distinguished by its `Layer`/`TypedIndex` integer per
//! `docs/3d/fbx/fbx-binary-properties70.md` §6.4 "LayerElement*
//! sub-discriminator (within Geometry)": every layer carries its own
//! `MappingInformationType` / `ReferenceInformationType` string
//! leaves that sub-classify its indexing, and the leading integer
//! property on the node is the `TypedIndex` the parent `Layer` node
//! references.
//!
//! `oxideav_mesh3d::Primitive` exposes a single `normals: Option<…>`
//! slot, so the FIRST normal layer becomes the canonical
//! `vertex_normal`; any further normal layers ride on
//! `Primitive::extras["fbx:extra_normals"]` (one flattened per-corner
//! `[x,y,z,…]` buffer each), with `fbx:extra_normals_typed_index`
//! and `fbx:extra_normals_mapping` recording each extra layer's
//! `TypedIndex` and source mapping mode.
//!
//! Test 1 (`single_normal_layer_populates_prim_normals`) is the
//! baseline: a quad with one `LayerElementNormal` surfaces on
//! `prim.normals` and leaves `extras` untouched.
//!
//! Test 2 (`two_normal_layers_first_canonical_rest_in_extras`) adds
//! a second `LayerElementNormal` (a different `TypedIndex` and a
//! different mapping mode — `ByVertex` instead of `ByPolygonVertex`)
//! and proves the second layer is flattened into `extras` with its
//! metadata, while channel 0 is unchanged.
//!
//! The `MappingInformationType` / `ReferenceInformationType` /
//! `Normals` / `NormalsIndex` shape follows
//! `docs/3d/fbx/fbx-binary-properties70.md` §6.4.

use oxideav_fbx::{FbxDecoder, FbxEncoder, FBX_MAGIC};
use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DEncoder};

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

// ---------------------------------------------------------------------------
// A unit quad in the XY plane: 4 shared vertices, one polygon spanning
// all four. Fan-triangulates to 2 triangles / 6 corners.
// ---------------------------------------------------------------------------

const QUAD_VERTICES: &[f64] = &[
    0.0, 0.0, 0.0, // v0
    1.0, 0.0, 0.0, // v1
    1.0, 1.0, 0.0, // v2
    0.0, 1.0, 0.0, // v3
];

// One quad (v0,v1,v2,v3); last corner bitwise-NOT'd to mark polygon end.
const QUAD_PVI: &[i32] = &[0, 1, 2, !3];

/// `ByPolygonVertex` / `Direct`: one normal per polygon-vertex corner.
/// Four corners -> 12 floats. Distinctive values so we can identify
/// channel 0 at assertion time.
const NORMALS_BY_PV: &[f64] = &[
    0.0, 0.0, 1.0, // corner 0 (+Z)
    0.0, 0.0, 1.0, // corner 1
    0.0, 0.0, 1.0, // corner 2
    0.0, 0.0, 1.0, // corner 3
];

/// `ByVertex` / `Direct`: one normal per *shared* vertex. Four shared
/// vertices -> 12 floats. Per-vertex distinct so we can verify the
/// extra layer used the `ByVertex` lookup, not the polygon-vertex one.
const NORMALS_BY_VERTEX: &[f64] = &[
    1.0, 0.0, 0.0, // shared vertex 0 (+X)
    0.0, 1.0, 0.0, // shared vertex 1 (+Y)
    -1.0, 0.0, 0.0, // shared vertex 2 (-X)
    0.0, -1.0, 0.0, // shared vertex 3 (-Y)
];

fn normal_layer(typed_index: i64, mapping: &[u8], normals: &[f64]) -> Rec {
    Rec::new("LayerElementNormal")
        .with_prop_i64(typed_index)
        .with_child(Rec::new("MappingInformationType").with_prop_string(mapping))
        .with_child(Rec::new("ReferenceInformationType").with_prop_string(b"Direct"))
        .with_child(Rec::new("Normals").with_prop_f64_array(normals))
}

fn build_geometry(extra_normal_layer: Option<Rec>) -> Rec {
    let mut geom = Rec::new("Geometry")
        .with_prop_i64(100)
        .with_prop_string(b"Quad\x00\x01Geometry")
        .with_prop_string(b"Mesh")
        .with_child(Rec::new("Vertices").with_prop_f64_array(QUAD_VERTICES))
        .with_child(Rec::new("PolygonVertexIndex").with_prop_i32_array(QUAD_PVI))
        .with_child(normal_layer(0, b"ByPolygonVertex", NORMALS_BY_PV));
    if let Some(extra) = extra_normal_layer {
        geom = geom.with_child(extra);
    }
    geom
}

fn build_objects(extra_normal_layer: Option<Rec>) -> Rec {
    Rec::new("Objects")
        .with_child(build_geometry(extra_normal_layer))
        .with_child(
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

fn build_synthetic_fbx(extra_normal_layer: Option<Rec>) -> Vec<u8> {
    let mut buf = Vec::new();
    buf.extend_from_slice(FBX_MAGIC);
    buf.extend_from_slice(&[0x1A, 0x00]);
    buf.extend_from_slice(&7400u32.to_le_bytes()); // pre-7500 (32-bit headers)
    let base_offset = buf.len();
    let mut body = Vec::new();
    serialize_node(&build_objects(extra_normal_layer), &mut body, base_offset);
    serialize_node(&build_connections(), &mut body, base_offset);
    body.extend_from_slice(&[0u8; NULL_RECORD_BYTES_32]);
    buf.extend_from_slice(&body);
    buf
}

#[test]
fn single_normal_layer_populates_prim_normals() {
    let bytes = build_synthetic_fbx(None);
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(&bytes).expect("single-normal quad decodes");

    assert_eq!(scene.meshes.len(), 1, "exactly one Mesh");
    let prim = &scene.meshes[0].primitives[0];

    // One quad fan-triangulates to 2 triangles -> 6 corners.
    assert_eq!(prim.positions.len(), 6);

    let normals = prim.normals.as_ref().expect("normals surfaced");
    assert_eq!(normals.len(), 6, "one normal per triangle corner");
    // ByPolygonVertex/Direct with all-(+Z) values.
    for n in normals {
        assert_eq!(*n, [0.0, 0.0, 1.0]);
    }

    // No extra normal layer -> no extras key.
    assert!(
        !prim.extras.contains_key("fbx:extra_normals"),
        "single layer must not populate extra-normals extras"
    );
}

#[test]
fn two_normal_layers_first_canonical_rest_in_extras() {
    // Second normal layer: TypedIndex 1, ByVertex/Direct, per-shared-
    // vertex values distinct from channel 0.
    let extra = normal_layer(1, b"ByVertex", NORMALS_BY_VERTEX);

    let bytes = build_synthetic_fbx(Some(extra));
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(&bytes).expect("two-normal quad decodes");
    let prim = &scene.meshes[0].primitives[0];

    // Channel 0 (the canonical vertex_normal) is unchanged: all +Z.
    let normals = prim.normals.as_ref().expect("canonical normals surfaced");
    assert_eq!(normals.len(), 6);
    for n in normals {
        assert_eq!(*n, [0.0, 0.0, 1.0]);
    }

    // The second layer rode into extras.
    let extra_normals = prim
        .extras
        .get("fbx:extra_normals")
        .and_then(|v| v.as_array())
        .expect("fbx:extra_normals present");
    assert_eq!(extra_normals.len(), 1, "exactly one extra normal layer");

    let layer0 = extra_normals[0]
        .as_array()
        .expect("extra normal layer 0 is an array");
    // 6 corners * 3 components = 18 floats.
    assert_eq!(
        layer0.len(),
        18,
        "flattened per-corner buffer for 6 corners"
    );

    // Reconstruct the per-corner [x,y,z] and verify the ByVertex
    // lookup picked NORMALS_BY_VERTEX by *shared* vertex, fanned
    // over the quad's two triangles.
    //
    // Quad pvi = [v0,v1,v2,v3]; fan around corner 0 ->
    //   triangle 0 = (v0,v1,v2), triangle 1 = (v0,v2,v3).
    // ByVertex picks NORMALS_BY_VERTEX[shared_vertex]:
    //   v0=+X, v1=+Y, v2=-X, v3=-Y.
    let got: Vec<[f64; 3]> = layer0
        .chunks_exact(3)
        .map(|c| {
            [
                c[0].as_f64().unwrap(),
                c[1].as_f64().unwrap(),
                c[2].as_f64().unwrap(),
            ]
        })
        .collect();
    let expected: Vec<[f64; 3]> = vec![
        [1.0, 0.0, 0.0],  // tri0 c0 = v0 (+X)
        [0.0, 1.0, 0.0],  // tri0 c1 = v1 (+Y)
        [-1.0, 0.0, 0.0], // tri0 c2 = v2 (-X)
        [1.0, 0.0, 0.0],  // tri1 c0 = v0 (+X)
        [-1.0, 0.0, 0.0], // tri1 c1 = v2 (-X)
        [0.0, -1.0, 0.0], // tri1 c2 = v3 (-Y)
    ];
    assert_eq!(got, expected, "ByVertex extra layer flattened correctly");

    // TypedIndex metadata: the extra layer's leading integer (1).
    let typed = prim
        .extras
        .get("fbx:extra_normals_typed_index")
        .and_then(|v| v.as_array())
        .expect("typed-index metadata present");
    assert_eq!(typed.len(), 1);
    assert_eq!(typed[0].as_i64(), Some(1));

    // Mapping-mode metadata: the extra layer's source mapping.
    let mapping = prim
        .extras
        .get("fbx:extra_normals_mapping")
        .and_then(|v| v.as_array())
        .expect("mapping metadata present");
    assert_eq!(mapping.len(), 1);
    assert_eq!(mapping[0].as_str(), Some("ByVertex"));
}

// ---------------------------------------------------------------------------
// `ByPolygon` and `AllSame` mapping modes (canonical channel-0 normals).
//
// A two-triangle-polygon mesh: 6 vertices, PVI = [0,1,~2, 3,4,~5], i.e.
// polygon 0 = (v0,v1,v2), polygon 1 = (v3,v4,v5). Fan-triangulates 1:1
// (each polygon is already a triangle) -> 2 triangles / 6 corners with
// `tri_polygon_index = [0, 1]`.
// ---------------------------------------------------------------------------

const TWO_TRI_VERTICES: &[f64] = &[
    0.0, 0.0, 0.0, // v0
    1.0, 0.0, 0.0, // v1
    0.0, 1.0, 0.0, // v2
    2.0, 0.0, 0.0, // v3
    3.0, 0.0, 0.0, // v4
    2.0, 1.0, 0.0, // v5
];
const TWO_TRI_PVI: &[i32] = &[0, 1, !2, 3, 4, !5];

fn build_fbx_custom_geometry(normal0: Rec, vertices: &[f64], pvi: &[i32]) -> Vec<u8> {
    let geom = Rec::new("Geometry")
        .with_prop_i64(100)
        .with_prop_string(b"Quad\x00\x01Geometry")
        .with_prop_string(b"Mesh")
        .with_child(Rec::new("Vertices").with_prop_f64_array(vertices))
        .with_child(Rec::new("PolygonVertexIndex").with_prop_i32_array(pvi))
        .with_child(normal0);
    let objects = Rec::new("Objects").with_child(geom).with_child(
        Rec::new("Model")
            .with_prop_i64(200)
            .with_prop_string(b"QuadModel\x00\x01Model")
            .with_prop_string(b"Mesh"),
    );
    let mut buf = Vec::new();
    buf.extend_from_slice(FBX_MAGIC);
    buf.extend_from_slice(&[0x1A, 0x00]);
    buf.extend_from_slice(&7400u32.to_le_bytes());
    let base_offset = buf.len();
    let mut body = Vec::new();
    serialize_node(&objects, &mut body, base_offset);
    serialize_node(&build_connections(), &mut body, base_offset);
    body.extend_from_slice(&[0u8; NULL_RECORD_BYTES_32]);
    buf.extend_from_slice(&body);
    buf
}

#[test]
fn by_polygon_normals_flatten_per_polygon() {
    // Two polygons, one normal each: poly0 = +Z, poly1 = +X.
    let normals = &[
        0.0, 0.0, 1.0, // polygon 0 (+Z)
        1.0, 0.0, 0.0, // polygon 1 (+X)
    ];
    let layer0 = normal_layer(0, b"ByPolygon", normals);
    let bytes = build_fbx_custom_geometry(layer0, TWO_TRI_VERTICES, TWO_TRI_PVI);
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(&bytes).expect("ByPolygon normals decode");
    let prim = &scene.meshes[0].primitives[0];

    let got = prim.normals.as_ref().expect("ByPolygon normals surfaced");
    assert_eq!(got.len(), 6, "one normal per triangle corner");
    // Corners 0,1,2 belong to polygon 0 (+Z); 3,4,5 to polygon 1 (+X).
    assert_eq!(got[0], [0.0, 0.0, 1.0]);
    assert_eq!(got[1], [0.0, 0.0, 1.0]);
    assert_eq!(got[2], [0.0, 0.0, 1.0]);
    assert_eq!(got[3], [1.0, 0.0, 0.0]);
    assert_eq!(got[4], [1.0, 0.0, 0.0]);
    assert_eq!(got[5], [1.0, 0.0, 0.0]);
}

#[test]
fn all_same_normals_broadcast_to_every_corner() {
    // AllSame: a single normal applies to the whole mesh.
    let normals = &[0.0, 1.0, 0.0]; // +Y for all corners
    let layer0 = normal_layer(0, b"AllSame", normals);
    let bytes = build_fbx_custom_geometry(layer0, TWO_TRI_VERTICES, TWO_TRI_PVI);
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(&bytes).expect("AllSame normals decode");
    let prim = &scene.meshes[0].primitives[0];

    let got = prim.normals.as_ref().expect("AllSame normals surfaced");
    assert_eq!(got.len(), 6);
    for n in got {
        assert_eq!(*n, [0.0, 1.0, 0.0]);
    }
}

#[test]
fn by_polygon_normals_index_to_direct() {
    // ByPolygon + IndexToDirect: NormalsIndex keys per polygon into the
    // Normals data pool.
    let normals = &[
        0.0, 0.0, 1.0, // pool[0] = +Z
        1.0, 0.0, 0.0, // pool[1] = +X
    ];
    let layer0 = Rec::new("LayerElementNormal")
        .with_prop_i64(0)
        .with_child(Rec::new("MappingInformationType").with_prop_string(b"ByPolygon"))
        .with_child(Rec::new("ReferenceInformationType").with_prop_string(b"IndexToDirect"))
        .with_child(Rec::new("Normals").with_prop_f64_array(normals))
        // polygon 0 -> pool[1] (+X); polygon 1 -> pool[0] (+Z).
        .with_child(Rec::new("NormalsIndex").with_prop_i32_array(&[1, 0]));
    let bytes = build_fbx_custom_geometry(layer0, TWO_TRI_VERTICES, TWO_TRI_PVI);
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(&bytes).expect("ByPolygon/IndexToDirect decode");
    let prim = &scene.meshes[0].primitives[0];

    let got = prim.normals.as_ref().expect("normals surfaced");
    assert_eq!(got.len(), 6);
    assert_eq!(got[0], [1.0, 0.0, 0.0], "poly0 -> index 1 (+X)");
    assert_eq!(got[3], [0.0, 0.0, 1.0], "poly1 -> index 0 (+Z)");
}

#[test]
fn by_polygon_normals_survive_encode_decode_round_trip() {
    // Decode a ByPolygon-normal fixture, then re-encode via the public
    // FbxEncoder and decode again. The encoder emits per-corner
    // (ByPolygonVertex) normals, so the flattened per-corner values must
    // survive the full decode -> encode -> decode cycle unchanged.
    let normals = &[
        0.0, 0.0, 1.0, // polygon 0 (+Z)
        1.0, 0.0, 0.0, // polygon 1 (+X)
    ];
    let layer0 = normal_layer(0, b"ByPolygon", normals);
    let bytes = build_fbx_custom_geometry(layer0, TWO_TRI_VERTICES, TWO_TRI_PVI);

    let scene1 = FbxDecoder::new().decode(&bytes).expect("first decode");
    let want: Vec<[f32; 3]> = scene1.meshes[0].primitives[0]
        .normals
        .clone()
        .expect("normals after first decode");

    let reencoded = FbxEncoder::new().encode(&scene1).expect("re-encode");
    let scene2 = FbxDecoder::new().decode(&reencoded).expect("second decode");
    let got = scene2.meshes[0].primitives[0]
        .normals
        .as_ref()
        .expect("normals after round-trip");

    assert_eq!(got.len(), 6);
    assert_eq!(*got, want, "ByPolygon normals survive the round-trip");
    // Concrete per-corner expectation: poly0 corners +Z, poly1 corners +X.
    assert_eq!(got[0], [0.0, 0.0, 1.0]);
    assert_eq!(got[3], [1.0, 0.0, 0.0]);
}
