//! End-to-end integration test for round-5 Material / Texture / Video
//! surfacing.
//!
//! Builds a minimal synthetic binary-FBX byte stream — the same
//! synthetic-quad scaffold as [`synthetic_quad`] plus three extra
//! `Objects` records (one `Material`, one `Texture`, one `Video`)
//! and three extra `Connections C` records:
//!
//! 1. Material 300 -> Model 200 (OO; surface assignment)
//! 2. Texture 400 -> Material 300 (OP "DiffuseColor"; base-colour binding)
//! 3. Video 500 -> Texture 400 (OO; embedded media)
//!
//! The Video record carries an 8-byte PNG-magic blob inside a
//! `Content { R<bytes> }` sub-record so the decoder takes the
//! embedded-media path (favoured over RelativeFilename — the embedded
//! `Content` R-blob, per `docs/3d/fbx/fbx-binary-properties70.md` §3c).
//!
//! After `FbxDecoder::decode` the asserts confirm:
//!
//! - `Scene3D::materials.len() == 1` with the FBX element-name "Wood".
//! - `Scene3D::textures.len() == 1` with `ImageData::Source` carrying
//!   the embedded PNG bytes + `image/png` MIME hint.
//! - The material's `base_color_texture` slot points at that texture.
//! - The Model node's mesh primitive carries `material = Some(0)`.

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
    fn with_prop_raw(mut self, bytes: &[u8]) -> Self {
        prop_raw(&mut self.props, bytes);
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
fn prop_raw(out: &mut Vec<u8>, s: &[u8]) {
    out.push(b'R');
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
    let vertices = Rec::new("Vertices").with_prop_f64_array(&[
        0.0, 0.0, 0.0, //
        1.0, 0.0, 0.0, //
        1.0, 1.0, 0.0, //
        0.0, 1.0, 0.0, //
    ]);
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

fn build_material_rec() -> Rec {
    Rec::new("Material")
        .with_prop_i64(300)
        .with_prop_string(b"Wood\x00\x01Material")
        .with_prop_string(b"")
}

fn build_texture_rec() -> Rec {
    // Texture with no RelativeFilename — the decoder must fall back
    // to the connected Video's `Content` blob.
    Rec::new("Texture")
        .with_prop_i64(400)
        .with_prop_string(b"WoodTex\x00\x01Texture")
        .with_prop_string(b"")
}

fn build_video_rec(content: &[u8]) -> Rec {
    let filename = Rec::new("Filename").with_prop_string(b"wood.png");
    let content_rec = Rec::new("Content").with_prop_raw(content);
    Rec::new("Video")
        .with_prop_i64(500)
        .with_prop_string(b"WoodVideo\x00\x01Video")
        .with_prop_string(b"Clip")
        .with_child(filename)
        .with_child(content_rec)
}

fn build_objects_rec(png: &[u8]) -> Rec {
    Rec::new("Objects")
        .with_child(build_geometry_rec())
        .with_child(build_model_rec())
        .with_child(build_material_rec())
        .with_child(build_texture_rec())
        .with_child(build_video_rec(png))
}

fn build_connection_oo(child_id: i64, parent_id: i64) -> Rec {
    Rec::new("C")
        .with_prop_string(b"OO")
        .with_prop_i64(child_id)
        .with_prop_i64(parent_id)
}

fn build_connection_op(child_id: i64, parent_id: i64, prop: &[u8]) -> Rec {
    Rec::new("C")
        .with_prop_string(b"OP")
        .with_prop_i64(child_id)
        .with_prop_i64(parent_id)
        .with_prop_string(prop)
}

fn build_connections_rec() -> Rec {
    Rec::new("Connections")
        .with_child(build_connection_oo(100, 200)) // Geometry -> Model
        .with_child(build_connection_oo(200, 0)) // Model -> root
        .with_child(build_connection_oo(300, 200)) // Material -> Model
        .with_child(build_connection_op(400, 300, b"DiffuseColor")) // Texture -> Material
        .with_child(build_connection_oo(500, 400)) // Video -> Texture
}

fn build_synthetic_fbx(png: &[u8]) -> Vec<u8> {
    let mut buf = Vec::new();
    buf.extend_from_slice(FBX_MAGIC);
    buf.extend_from_slice(&[0x1A, 0x00]);
    buf.extend_from_slice(&7400u32.to_le_bytes());
    let base_offset = buf.len();
    let mut body = Vec::new();
    serialize_node(&build_objects_rec(png), &mut body, base_offset);
    serialize_node(&build_connections_rec(), &mut body, base_offset);
    body.extend_from_slice(&[0u8; NULL_RECORD_BYTES_32]);
    buf.extend_from_slice(&body);
    buf
}

#[test]
fn synthetic_material_round_trips_through_full_decoder() {
    // 8-byte PNG signature — the embedded-media payload the decoder
    // should surface verbatim through `Texture::from_encoded`.
    let png = [0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A];
    let bytes = build_synthetic_fbx(&png);

    let mut dec = FbxDecoder::new();
    let scene = dec
        .decode(&bytes)
        .expect("synthetic-material FBX decodes cleanly");

    // Geometry / Model pipeline still works.
    assert_eq!(scene.meshes.len(), 1, "exactly one Mesh");
    assert_eq!(scene.nodes.len(), 1, "exactly one Model node");

    // Material surfaced + bound.
    assert_eq!(scene.materials.len(), 1, "Material element surfaced");
    let mat = &scene.materials[0];
    assert_eq!(mat.name.as_deref(), Some("Wood"));

    // Texture surfaced.
    assert_eq!(scene.textures.len(), 1, "Texture element surfaced");
    let tex = &scene.textures[0];
    assert_eq!(tex.name.as_deref(), Some("WoodTex"));
    match &tex.image {
        oxideav_mesh3d::ImageData::Source(src) => {
            // Embedded PNG path picked up the Video.Content blob.
            assert_eq!(
                src.mime(),
                Some("image/png"),
                "MIME inferred from Video.Filename"
            );
            assert_eq!(src.size_hint(), Some(png.len() as u64));
        }
        other => panic!("expected Source image, got {other:?}"),
    }

    // DiffuseColor OP binding wired into base_color_texture.
    let texref = mat
        .base_color_texture
        .expect("DiffuseColor OP record bound texture into base_color_texture");
    assert_eq!(texref.texture.0, 0);

    // Material -> Model OO connection landed on the primitive.
    let prim = &scene.meshes[0].primitives[0];
    assert_eq!(
        prim.material.map(|m| m.0),
        Some(0),
        "Material attached to mesh primitive via Material -> Model OO"
    );
}
