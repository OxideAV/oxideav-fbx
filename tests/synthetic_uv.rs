//! Hand-authored binary FBX fixtures exercising `LayerElementUV`
//! decode for one and two UV channels.
//!
//! Test 1 (`single_uv_set_matches_cubes_ascii_fixture`) reconstructs
//! a cube whose `Vertices` / `PolygonVertexIndex` / `LayerElementUV`
//! arrays are the same byte-values as the first mesh in the staged
//! `docs/3d/fbx/fixtures/cubes-ascii-v7500.fbx` ASCII fixture. The
//! ASCII fixture is the externally-authored ground truth; the
//! synthetic binary is the same data routed through this crate's
//! binary writer/decoder so we can prove the UV decode path
//! round-trips a known UV/UVIndex pair without writing an ASCII
//! parser (which is out of scope per `CHANGELOG.md` round 1).
//!
//! Test 2 (`two_uv_sets_surface_in_document_order`) builds a quad
//! with two `LayerElementUV` records to prove the `Primitive::uvs`
//! `Vec<Vec<[f32; 2]>>` is populated for every UV channel, mirroring
//! the existing `LayerElementColor` multi-channel behaviour. Per
//! `docs/3d/fbx/ufbx/reference.html` §`ufbx_mesh.uv_sets` /
//! §`ufbx_uv_set`, an FBX mesh may carry several UV channels
//! (commonly: diffuse + lightmap); the first one is also surfaced at
//! `ufbx_mesh.vertex_uv`.
//!
//! The `MappingInformationType` / `ReferenceInformationType` /
//! `UV` / `UVIndex` shape follows
//! `docs/3d/fbx/ufbx/elements-meshes.md` §"Attributes" and
//! `docs/3d/fbx/fbx-binary-properties70.md` §"LayerElement*
//! sub-discriminator (within Geometry)".

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

// ---------------------------------------------------------------------------
// Cube ground truth from `docs/3d/fbx/fixtures/cubes-ascii-v7500.fbx`
// (the first `Geometry` block, lines 251-311 of the fixture).
// ---------------------------------------------------------------------------

/// 8 unique cube corner positions (24 floats). Verbatim from the
/// fixture's `Vertices: *24 { a: -0.5,-0.5,0.5,…,0.5,-0.5,-0.5 }`.
const CUBE_VERTICES: &[f64] = &[
    -0.5, -0.5, 0.5, 0.5, -0.5, 0.5, -0.5, 0.5, 0.5, 0.5, 0.5, 0.5, -0.5, 0.5, -0.5, 0.5, 0.5,
    -0.5, -0.5, -0.5, -0.5, 0.5, -0.5, -0.5,
];

/// 24 polygon-vertex indices (6 quads × 4 corners; last per polygon
/// is bitwise-NOT'd to mark quad end per
/// `docs/3d/fbx/fbx-binary-properties70.md` §"PolygonVertexIndex").
/// Verbatim from the fixture's `PolygonVertexIndex: *24 { a:
/// 0,1,3,-3,2,3,5,-5,…,6,0,2,-5 }`.
const CUBE_PVI: &[i32] = &[
    0, 1, 3, -3, 2, 3, 5, -5, 4, 5, 7, -7, 6, 7, 1, -1, 1, 7, 5, -4, 6, 0, 2, -5,
];

/// 14 unique UV pairs (28 floats). Verbatim from the fixture's
/// `UV: *28 { a: 0.375,0,0.625,0,…,0.125,0,0.125,0.25 }`.
const CUBE_UV_RAW: &[f64] = &[
    0.375, 0.0, 0.625, 0.0, 0.375, 0.25, 0.625, 0.25, 0.375, 0.5, 0.625, 0.5, 0.375, 0.75, 0.625,
    0.75, 0.375, 1.0, 0.625, 1.0, 0.875, 0.0, 0.875, 0.25, 0.125, 0.0, 0.125, 0.25,
];

/// 24 indices into `CUBE_UV_RAW` (one per polygon-vertex corner).
/// Verbatim from the fixture's `UVIndex: *24 { a:
/// 0,1,3,2,2,3,5,4,4,5,7,6,6,7,9,8,1,10,11,3,12,0,2,13 }`.
const CUBE_UV_INDEX: &[i32] = &[
    0, 1, 3, 2, 2, 3, 5, 4, 4, 5, 7, 6, 6, 7, 9, 8, 1, 10, 11, 3, 12, 0, 2, 13,
];

fn build_cube_geometry_rec(extra_uv_layer: Option<Rec>) -> Rec {
    let vertices = Rec::new("Vertices").with_prop_f64_array(CUBE_VERTICES);
    let pvi = Rec::new("PolygonVertexIndex").with_prop_i32_array(CUBE_PVI);
    let uv_layer = Rec::new("LayerElementUV")
        .with_prop_i64(0)
        .with_child(Rec::new("MappingInformationType").with_prop_string(b"ByPolygonVertex"))
        .with_child(Rec::new("ReferenceInformationType").with_prop_string(b"IndexToDirect"))
        .with_child(Rec::new("UV").with_prop_f64_array(CUBE_UV_RAW))
        .with_child(Rec::new("UVIndex").with_prop_i32_array(CUBE_UV_INDEX));
    let mut geom = Rec::new("Geometry")
        .with_prop_i64(100)
        .with_prop_string(b"Cube\x00\x01Geometry")
        .with_prop_string(b"Mesh")
        .with_child(vertices)
        .with_child(pvi)
        .with_child(uv_layer);
    if let Some(extra) = extra_uv_layer {
        geom = geom.with_child(extra);
    }
    geom
}

fn build_model_rec() -> Rec {
    Rec::new("Model")
        .with_prop_i64(200)
        .with_prop_string(b"CubeModel\x00\x01Model")
        .with_prop_string(b"Mesh")
}

fn build_objects_rec(extra_uv_layer: Option<Rec>) -> Rec {
    Rec::new("Objects")
        .with_child(build_cube_geometry_rec(extra_uv_layer))
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

fn build_synthetic_fbx(extra_uv_layer: Option<Rec>) -> Vec<u8> {
    let mut buf = Vec::new();
    buf.extend_from_slice(FBX_MAGIC);
    buf.extend_from_slice(&[0x1A, 0x00]);
    buf.extend_from_slice(&7400u32.to_le_bytes()); // pre-7500 (32-bit headers)
    let base_offset = buf.len();
    let mut body = Vec::new();
    serialize_node(&build_objects_rec(extra_uv_layer), &mut body, base_offset);
    serialize_node(&build_connections_rec(), &mut body, base_offset);
    body.extend_from_slice(&[0u8; NULL_RECORD_BYTES_32]);
    buf.extend_from_slice(&body);
    buf
}

#[test]
fn single_uv_set_matches_cubes_ascii_fixture() {
    let bytes = build_synthetic_fbx(None);
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(&bytes).expect("synthetic cube decodes");

    assert_eq!(scene.meshes.len(), 1, "exactly one Mesh");
    let prim = &scene.meshes[0].primitives[0];
    assert_eq!(prim.topology, Topology::Triangles);

    // 6 quads, fan-triangulated -> 12 triangles -> 36 corners.
    assert_eq!(prim.positions.len(), 36);

    // One UV set (mirrors the fixture's single LayerElementUV).
    assert_eq!(prim.uvs.len(), 1, "exactly one UV set surfaced");
    let uv = &prim.uvs[0];
    assert_eq!(uv.len(), 36, "one UV pair per triangle corner");

    // Spot-check: corner 0 of polygon 0 should pick UV_RAW[UVIndex[0]]
    // = UV_RAW[0] = (0.375, 0.0). Corner 1 of polygon 0 -> UVIndex[1]
    // = 1 -> UV_RAW[1] = (0.625, 0.0). Corner 2 of polygon 0 ->
    // UVIndex[2] = 3 -> UV_RAW[3] = (0.625, 0.25). These are the
    // first three triangle corners (the fan triangulation of the
    // first quad reuses polygon-vertex 0 as the fan apex).
    assert_eq!(uv[0], [0.375, 0.0]);
    assert_eq!(uv[1], [0.625, 0.0]);
    assert_eq!(uv[2], [0.625, 0.25]);

    // Second triangle of polygon 0: corners (0, 2, 3) of polygon ->
    // UVIndex[0,2,3] -> UV_RAW[0,3,2] -> (0.375,0), (0.625,0.25),
    // (0.375,0.25).
    assert_eq!(uv[3], [0.375, 0.0]);
    assert_eq!(uv[4], [0.625, 0.25]);
    assert_eq!(uv[5], [0.375, 0.25]);

    // Final corner of the last polygon (polygon 5, fan triangle 1,
    // corner 2) -> per-polygon-vertex index 23 -> UVIndex[23] = 13
    // -> UV_RAW[13] = (0.125, 0.25). The last triangle of the cube
    // is fan-second-triangle of polygon 5 ((corners 0, 2, 3)), so
    // the *11th* triangle's third corner == polygon-corner 23.
    let last = uv.last().copied().expect("at least one UV pair");
    assert_eq!(last, [0.125, 0.25]);

    // Sanity: every emitted UV pair is one of the 14 unique entries
    // in CUBE_UV_RAW (no out-of-range remap).
    let unique_uvs: std::collections::HashSet<(u32, u32)> = CUBE_UV_RAW
        .chunks_exact(2)
        .map(|c| (c[0].to_bits() as u32, c[1].to_bits() as u32))
        .collect();
    for &[u, v] in uv {
        let key = ((u as f64).to_bits() as u32, (v as f64).to_bits() as u32);
        assert!(
            unique_uvs.contains(&key),
            "decoded UV ({u}, {v}) is not one of the 14 ground-truth pairs"
        );
    }
}

#[test]
fn two_uv_sets_surface_in_document_order() {
    // Second UV layer: same arity as the first but with all-zero
    // UV.x and arithmetic-progression UV.y so we can distinguish
    // the two channels at assertion time. IndexToDirect with a
    // deliberately different remap (reverse).
    let mut second_uv_raw = Vec::with_capacity(28);
    for i in 0..14 {
        second_uv_raw.push(0.0); // U is zero across the channel
        second_uv_raw.push(i as f64 / 13.0); // V is a 0..1 ramp
    }
    let mut reversed_index = CUBE_UV_INDEX.to_vec();
    reversed_index.reverse();

    let extra = Rec::new("LayerElementUV")
        .with_prop_i64(1) // layer index 1 (the second UV channel)
        .with_child(Rec::new("MappingInformationType").with_prop_string(b"ByPolygonVertex"))
        .with_child(Rec::new("ReferenceInformationType").with_prop_string(b"IndexToDirect"))
        .with_child(Rec::new("UV").with_prop_f64_array(&second_uv_raw))
        .with_child(Rec::new("UVIndex").with_prop_i32_array(&reversed_index));

    let bytes = build_synthetic_fbx(Some(extra));
    let mut dec = FbxDecoder::new();
    let scene = dec
        .decode(&bytes)
        .expect("synthetic cube with two UV sets decodes");
    let prim = &scene.meshes[0].primitives[0];

    // Both UV channels populated, in document order.
    assert_eq!(prim.uvs.len(), 2, "two UV sets surfaced");
    let first = &prim.uvs[0];
    let second = &prim.uvs[1];
    assert_eq!(first.len(), 36);
    assert_eq!(second.len(), 36);

    // Channel 0 is unchanged from the single-layer test.
    assert_eq!(first[0], [0.375, 0.0]);
    assert_eq!(first[1], [0.625, 0.0]);
    assert_eq!(first[2], [0.625, 0.25]);

    // Channel 1 has U == 0 throughout and V drawn from the
    // arithmetic-progression ramp, picked via the reversed index.
    // Reversed CUBE_UV_INDEX[0] = 13, so corner 0 of polygon 0 ->
    // second_uv_raw[13] = (0.0, 13/13 = 1.0).
    assert_eq!(second[0], [0.0, 1.0]);
    // Every channel-1 sample has U == 0.0 (sanity).
    for &[u, _v] in second {
        assert_eq!(u, 0.0);
    }
    // Every channel-1 V is a multiple of 1/13 ∈ [0, 1].
    for &[_u, v] in second {
        assert!((0.0..=1.0).contains(&v));
    }
}
