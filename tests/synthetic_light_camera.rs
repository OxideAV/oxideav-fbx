//! Hand-authored binary FBX fixture exercising round-207 Light /
//! Camera `NodeAttribute` surfacing.
//!
//! Builds a binary FBX containing two `NodeAttribute` records (one
//! subtype `"Light"`, one subtype `"Camera"`), each with the
//! `Properties70` `P`-record block the well-known fields require, and
//! two `Model` records (one per attribute) so the `OO` connections
//! `NodeAttribute -> Model` can resolve onto scene-graph nodes.
//!
//! Per `docs/3d/fbx/fbx-binary-properties70.md` §6:
//!
//! - `NodeAttribute` is a top-level object whose **third property** is
//!   the subtype string (`"Light"` / `"Camera"` / `"LimbNode"` / …).
//!   The same triple shape `Geometry` / `Model` / `Material` use.
//! - The attribute's parameters live in a `Properties70` child whose
//!   children are the §4 `P` records (4 leading strings + typed
//!   values).
//!
//! Property names + value typing are taken from
//! `docs/3d/fbx/ufbx/reference.html` §`ufbx_light` / §`ufbx_camera`.

use std::collections::HashMap;

use oxideav_fbx::{FbxDecoder, FBX_MAGIC};
use oxideav_mesh3d::{Camera, Light, Mesh3DDecoder};

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
        self.props.push(b'S');
        self.props
            .extend_from_slice(&(s.len() as u32).to_le_bytes());
        self.props.extend_from_slice(s);
        self.num_props += 1;
        self
    }
    fn with_prop_i64(mut self, v: i64) -> Self {
        self.props.push(b'L');
        self.props.extend_from_slice(&v.to_le_bytes());
        self.num_props += 1;
        self
    }
    fn with_prop_i32(mut self, v: i32) -> Self {
        self.props.push(b'I');
        self.props.extend_from_slice(&v.to_le_bytes());
        self.num_props += 1;
        self
    }
    fn with_prop_f64(mut self, v: f64) -> Self {
        self.props.push(b'D');
        self.props.extend_from_slice(&v.to_le_bytes());
        self.num_props += 1;
        self
    }
    fn with_child(mut self, child: Rec) -> Self {
        self.children.push(child);
        self
    }
}

fn serialize_node(rec: &Rec, out: &mut Vec<u8>, base: usize) {
    let header_off = out.len();
    out.extend_from_slice(&0u32.to_le_bytes());
    out.extend_from_slice(&rec.num_props.to_le_bytes());
    out.extend_from_slice(&(rec.props.len() as u32).to_le_bytes());
    out.push(rec.name.len() as u8);
    out.extend_from_slice(rec.name.as_bytes());
    out.extend_from_slice(&rec.props);
    if !rec.children.is_empty() {
        for child in &rec.children {
            serialize_node(child, out, base);
        }
        out.extend_from_slice(&[0u8; NULL_RECORD_BYTES_32]);
    }
    let end_offset_abs = (base + out.len()) as u32;
    out[header_off..header_off + 4].copy_from_slice(&end_offset_abs.to_le_bytes());
}

fn p_double(name: &str, ty: &str, label: &str, flags: &str, v: f64) -> Rec {
    Rec::new("P")
        .with_prop_string(name.as_bytes())
        .with_prop_string(ty.as_bytes())
        .with_prop_string(label.as_bytes())
        .with_prop_string(flags.as_bytes())
        .with_prop_f64(v)
}

fn p_int(name: &str, ty: &str, label: &str, flags: &str, v: i32) -> Rec {
    Rec::new("P")
        .with_prop_string(name.as_bytes())
        .with_prop_string(ty.as_bytes())
        .with_prop_string(label.as_bytes())
        .with_prop_string(flags.as_bytes())
        .with_prop_i32(v)
}

fn p_color(name: &str, label: &str, flags: &str, rgb: [f64; 3]) -> Rec {
    Rec::new("P")
        .with_prop_string(name.as_bytes())
        .with_prop_string(b"ColorRGB")
        .with_prop_string(label.as_bytes())
        .with_prop_string(flags.as_bytes())
        .with_prop_f64(rgb[0])
        .with_prop_f64(rgb[1])
        .with_prop_f64(rgb[2])
}

fn light_attribute(id: i64) -> Rec {
    // Point light, white, 200 raw → 2.0 final after the 0.01x scale
    // (per §ufbx_light.intensity).
    let props70 = Rec::new("Properties70")
        .with_child(p_color("Color", "Color", "A", [1.0, 0.8, 0.6]))
        .with_child(p_double("Intensity", "Number", "Intensity", "A", 200.0))
        .with_child(p_int("LightType", "enum", "", "", 0))
        .with_child(p_int("DecayType", "enum", "", "", 2));
    Rec::new("NodeAttribute")
        .with_prop_i64(id)
        .with_prop_string(b"PointLight\x00\x01NodeAttribute")
        .with_prop_string(b"Light")
        .with_child(props70)
}

fn camera_attribute(id: i64) -> Rec {
    // Perspective camera, FieldOfViewY directly carries vertical
    // angle in degrees → mesh3d yfov in radians without further math.
    let props70 = Rec::new("Properties70")
        .with_child(p_int("CameraProjectionType", "enum", "", "", 0))
        .with_child(p_double("FieldOfViewY", "Number", "FieldOfView", "A", 36.0))
        .with_child(p_double("NearPlane", "Number", "", "", 0.5))
        .with_child(p_double("FarPlane", "Number", "", "", 800.0))
        .with_child(p_double("AspectWidth", "Number", "", "", 1920.0))
        .with_child(p_double("AspectHeight", "Number", "", "", 1080.0));
    Rec::new("NodeAttribute")
        .with_prop_i64(id)
        .with_prop_string(b"MainCamera\x00\x01NodeAttribute")
        .with_prop_string(b"Camera")
        .with_child(props70)
}

fn model(id: i64, name: &[u8], subtype: &[u8]) -> Rec {
    let mut display = name.to_vec();
    display.extend_from_slice(b"\x00\x01Model");
    Rec::new("Model")
        .with_prop_i64(id)
        .with_prop_string(&display)
        .with_prop_string(subtype)
}

fn connection(kind: &[u8], child: i64, parent: i64) -> Rec {
    Rec::new("C")
        .with_prop_string(kind)
        .with_prop_i64(child)
        .with_prop_i64(parent)
}

fn assemble(version: u32, top_levels: Vec<Rec>) -> Vec<u8> {
    let mut buf = Vec::new();
    buf.extend_from_slice(FBX_MAGIC);
    buf.extend_from_slice(&[0x1A, 0x00]);
    buf.extend_from_slice(&version.to_le_bytes());
    let base = buf.len();
    let mut body = Vec::new();
    for r in &top_levels {
        serialize_node(r, &mut body, base);
    }
    body.extend_from_slice(&[0u8; NULL_RECORD_BYTES_32]);
    buf.extend_from_slice(&body);
    buf
}

#[test]
fn binary_fbx_light_and_camera_surface_through_full_decoder() {
    // IDs: 100 Light attr, 101 Camera attr, 200 Light model, 201 Camera model.
    let objects = Rec::new("Objects")
        .with_child(light_attribute(100))
        .with_child(camera_attribute(101))
        .with_child(model(200, b"PointLamp", b"Light"))
        .with_child(model(201, b"MainCam", b"Camera"));
    let connections = Rec::new("Connections")
        .with_child(connection(b"OO", 100, 200))
        .with_child(connection(b"OO", 101, 201))
        // Anchor both models to the scene root so they show up in
        // Scene3D::roots.
        .with_child(connection(b"OO", 200, 0))
        .with_child(connection(b"OO", 201, 0));

    let bytes = assemble(7400, vec![objects, connections]);
    let scene = FbxDecoder::new().decode(&bytes).expect("decode");

    assert_eq!(scene.lights.len(), 1, "exactly one surfaced light");
    assert_eq!(scene.cameras.len(), 1, "exactly one surfaced camera");

    // Light: Point, color (1, 0.8, 0.6), intensity = 200 * 0.01 = 2.0.
    match scene.lights[0] {
        Light::Point {
            color, intensity, ..
        } => {
            assert!((color[0] - 1.0).abs() < 1e-4);
            assert!((color[1] - 0.8).abs() < 1e-4);
            assert!((color[2] - 0.6).abs() < 1e-4);
            assert!((intensity - 2.0).abs() < 1e-4);
        }
        other => panic!("expected Point light, got {other:?}"),
    }

    // Camera: Perspective, yfov = 36° in radians, znear=0.5,
    // zfar=Some(800), aspect=16/9.
    match scene.cameras[0] {
        Camera::Perspective {
            aspect_ratio,
            yfov,
            znear,
            zfar,
        } => {
            let expected_yfov = 36.0_f32.to_radians();
            assert!((yfov - expected_yfov).abs() < 1e-4);
            assert!((znear - 0.5).abs() < 1e-5);
            assert_eq!(zfar, Some(800.0));
            let ar = aspect_ratio.expect("AspectWidth+Height → ratio");
            assert!((ar - 1920.0 / 1080.0).abs() < 1e-4);
        }
        other => panic!("expected Perspective camera, got {other:?}"),
    }

    // Owning nodes have light/camera attached.
    let mut node_light_count = 0usize;
    let mut node_camera_count = 0usize;
    let mut light_name = None;
    let mut camera_name = None;
    let mut node_extra_keys: HashMap<&str, usize> = HashMap::new();
    for n in &scene.nodes {
        if n.light.is_some() {
            node_light_count += 1;
            light_name = n.name.clone();
        }
        if n.camera.is_some() {
            node_camera_count += 1;
            camera_name = n.name.clone();
        }
        for k in n.extras.keys() {
            *node_extra_keys.entry(k.as_str()).or_default() += 1;
        }
    }
    assert_eq!(node_light_count, 1);
    assert_eq!(node_camera_count, 1);
    assert_eq!(light_name.as_deref(), Some("PointLamp"));
    assert_eq!(camera_name.as_deref(), Some("MainCam"));
    assert!(node_extra_keys.contains_key("fbx:camera_resolution"));
}
