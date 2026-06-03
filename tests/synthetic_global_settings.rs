//! Round 219 — `GlobalSettings` end-to-end through `FbxDecoder`.
//!
//! Hand-builds a binary FBX whose top-level node tree is:
//!
//! ```text
//! GlobalSettings (Version=1000)
//!   Properties70
//!     P UpAxis             int       Integer "" 1
//!     P UpAxisSign         int       Integer "" 1
//!     P FrontAxis          int       Integer "" 2
//!     P FrontAxisSign      int       Integer "" 1
//!     P CoordAxis          int       Integer "" 0
//!     P CoordAxisSign      int       Integer "" 1
//!     P OriginalUpAxis     int       Integer "" 1
//!     P OriginalUpAxisSign int       Integer "" 1
//!     P UnitScaleFactor    double    Number  "" 100.0
//!     P OriginalUnitScaleFactor double Number "" 100.0
//!     P AmbientColor       ColorRGB  Color   "" 0,0,0
//!     P DefaultCamera      KString   ""      "" "Producer Perspective"
//!     P TimeMode           enum      ""      "" 11
//!     P TimeProtocol       enum      ""      "" 2
//!     P SnapOnFrameMode    enum      ""      "" 0
//!     P TimeSpanStart      KTime     Time    "" 1924423250
//!     P TimeSpanStop       KTime     Time    "" 384884650000
//!     P CustomFrameRate    double    Number  "" -1.0
//!     P CurrentTimeMarker  int       Integer "" -1
//! Objects     (empty — no geometry needed for this round)
//! Connections (empty)
//! ```
//!
//! The byte layout mirrors the §4 grammar staged in
//! `docs/3d/fbx/fbx-binary-properties70.md` (each `P` record's
//! property list is `S name, S typeName, S label, S flags, ...values`)
//! and the GlobalSettings P-record set from the cubes-ascii-v7500.fbx
//! fixture.
//!
//! Verifies the public `FbxDecoder::decode` path lifts the
//! `UnitScaleFactor=100` value to `Scene3D::unit = Centimetres` and
//! surfaces every well-known P-record onto `Scene3D::extras` under the
//! `"fbx:*"` snake_case key convention.

use oxideav_fbx::{FbxDecoder, FBX_MAGIC};
use oxideav_mesh3d::{Mesh3DDecoder, Unit};

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

    fn with_prop_i32(mut self, v: i32) -> Self {
        prop_i32(&mut self.props, v);
        self.num_props += 1;
        self
    }

    fn with_prop_i64(mut self, v: i64) -> Self {
        prop_i64(&mut self.props, v);
        self.num_props += 1;
        self
    }

    fn with_prop_f64(mut self, v: f64) -> Self {
        prop_f64(&mut self.props, v);
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

fn prop_i32(out: &mut Vec<u8>, v: i32) {
    out.push(b'I');
    out.extend_from_slice(&v.to_le_bytes());
}

fn prop_i64(out: &mut Vec<u8>, v: i64) {
    out.push(b'L');
    out.extend_from_slice(&v.to_le_bytes());
}

fn prop_f64(out: &mut Vec<u8>, v: f64) {
    out.push(b'D');
    out.extend_from_slice(&v.to_le_bytes());
}

/// Build a single `P` record. Mirrors the fixture-grounded shape from
/// `docs/3d/fbx/fbx-binary-properties70.md` §4: `S name, S typeName,
/// S label, S flags, ...values`.
fn p_int(name: &str, type_name: &str, label: &str, val: i32) -> Rec {
    Rec::new("P")
        .with_prop_string(name.as_bytes())
        .with_prop_string(type_name.as_bytes())
        .with_prop_string(label.as_bytes())
        .with_prop_string(b"")
        .with_prop_i32(val)
}

fn p_long(name: &str, type_name: &str, label: &str, val: i64) -> Rec {
    Rec::new("P")
        .with_prop_string(name.as_bytes())
        .with_prop_string(type_name.as_bytes())
        .with_prop_string(label.as_bytes())
        .with_prop_string(b"")
        .with_prop_i64(val)
}

fn p_double(name: &str, type_name: &str, label: &str, val: f64) -> Rec {
    Rec::new("P")
        .with_prop_string(name.as_bytes())
        .with_prop_string(type_name.as_bytes())
        .with_prop_string(label.as_bytes())
        .with_prop_string(b"")
        .with_prop_f64(val)
}

fn p_string(name: &str, type_name: &str, label: &str, val: &str) -> Rec {
    Rec::new("P")
        .with_prop_string(name.as_bytes())
        .with_prop_string(type_name.as_bytes())
        .with_prop_string(label.as_bytes())
        .with_prop_string(b"")
        .with_prop_string(val.as_bytes())
}

fn p_vec3(name: &str, type_name: &str, label: &str, val: [f64; 3]) -> Rec {
    Rec::new("P")
        .with_prop_string(name.as_bytes())
        .with_prop_string(type_name.as_bytes())
        .with_prop_string(label.as_bytes())
        .with_prop_string(b"")
        .with_prop_f64(val[0])
        .with_prop_f64(val[1])
        .with_prop_f64(val[2])
}

fn build_global_settings() -> Rec {
    let props70 = Rec::new("Properties70")
        .with_child(p_int("UpAxis", "int", "Integer", 1))
        .with_child(p_int("UpAxisSign", "int", "Integer", 1))
        .with_child(p_int("FrontAxis", "int", "Integer", 2))
        .with_child(p_int("FrontAxisSign", "int", "Integer", 1))
        .with_child(p_int("CoordAxis", "int", "Integer", 0))
        .with_child(p_int("CoordAxisSign", "int", "Integer", 1))
        .with_child(p_int("OriginalUpAxis", "int", "Integer", 1))
        .with_child(p_int("OriginalUpAxisSign", "int", "Integer", 1))
        .with_child(p_double("UnitScaleFactor", "double", "Number", 100.0))
        .with_child(p_double(
            "OriginalUnitScaleFactor",
            "double",
            "Number",
            100.0,
        ))
        .with_child(p_vec3("AmbientColor", "ColorRGB", "Color", [0.1, 0.2, 0.3]))
        .with_child(p_string(
            "DefaultCamera",
            "KString",
            "",
            "Producer Perspective",
        ))
        .with_child(p_int("TimeMode", "enum", "", 11))
        .with_child(p_int("TimeProtocol", "enum", "", 2))
        .with_child(p_int("SnapOnFrameMode", "enum", "", 0))
        .with_child(p_long("TimeSpanStart", "KTime", "Time", 1_924_423_250))
        .with_child(p_long("TimeSpanStop", "KTime", "Time", 384_884_650_000))
        .with_child(p_double("CustomFrameRate", "double", "Number", -1.0))
        .with_child(p_int("CurrentTimeMarker", "int", "Integer", -1));
    Rec::new("GlobalSettings").with_child(props70)
}

fn build_synthetic_fbx() -> Vec<u8> {
    let mut buf = Vec::new();
    buf.extend_from_slice(FBX_MAGIC);
    buf.extend_from_slice(&[0x1A, 0x00]);
    buf.extend_from_slice(&7400u32.to_le_bytes());
    let base_offset = buf.len();
    let mut body = Vec::new();
    serialize_node(&build_global_settings(), &mut body, base_offset);
    serialize_node(&Rec::new("Objects"), &mut body, base_offset);
    serialize_node(&Rec::new("Connections"), &mut body, base_offset);
    body.extend_from_slice(&[0u8; NULL_RECORD_BYTES_32]);
    buf.extend_from_slice(&body);
    buf
}

#[test]
fn global_settings_surfaces_axis_unit_time_extras() {
    let bytes = build_synthetic_fbx();
    let mut dec = FbxDecoder::new();
    let scene = dec
        .decode(&bytes)
        .expect("synthetic global settings decodes");

    // Underlying document has the GlobalSettings + Properties70 tree.
    let doc = dec.last_document.as_ref().expect("document captured");
    let gs = doc
        .root
        .child("GlobalSettings")
        .expect("GlobalSettings present");
    assert!(gs.child("Properties70").is_some());

    // UnitScaleFactor = 100.0 → Scene3D::unit = Centimetres.
    assert_eq!(scene.unit, Unit::Centimetres);

    // Spot-check the documented bucket types reached extras.
    let up = scene
        .extras
        .get("fbx:up_axis")
        .expect("UpAxis surfaced")
        .as_i64()
        .unwrap();
    assert_eq!(up, 1);
    let front = scene
        .extras
        .get("fbx:front_axis")
        .expect("FrontAxis surfaced")
        .as_i64()
        .unwrap();
    assert_eq!(front, 2);
    let coord = scene
        .extras
        .get("fbx:coord_axis")
        .expect("CoordAxis surfaced")
        .as_i64()
        .unwrap();
    assert_eq!(coord, 0);
    let original_up = scene
        .extras
        .get("fbx:original_up_axis")
        .expect("OriginalUpAxis surfaced")
        .as_i64()
        .unwrap();
    assert_eq!(original_up, 1);
    let usf = scene
        .extras
        .get("fbx:unit_scale_factor")
        .expect("UnitScaleFactor surfaced")
        .as_f64()
        .unwrap();
    assert!((usf - 100.0).abs() < 1e-9);
    let ousf = scene
        .extras
        .get("fbx:original_unit_scale_factor")
        .expect("OriginalUnitScaleFactor surfaced")
        .as_f64()
        .unwrap();
    assert!((ousf - 100.0).abs() < 1e-9);
    let amb = scene
        .extras
        .get("fbx:ambient_color")
        .expect("AmbientColor surfaced");
    let comps = amb.as_array().unwrap();
    assert_eq!(comps.len(), 3);
    assert!((comps[0].as_f64().unwrap() - 0.1).abs() < 1e-9);
    let cam = scene
        .extras
        .get("fbx:default_camera")
        .expect("DefaultCamera surfaced")
        .as_str()
        .unwrap();
    assert_eq!(cam, "Producer Perspective");
    let stop_ticks = scene
        .extras
        .get("fbx:time_span_stop")
        .expect("TimeSpanStop surfaced")
        .as_i64()
        .unwrap();
    assert_eq!(stop_ticks, 384_884_650_000);
    let fps = scene
        .extras
        .get("fbx:custom_frame_rate")
        .expect("CustomFrameRate surfaced")
        .as_f64()
        .unwrap();
    assert!((fps - -1.0).abs() < 1e-9);
}
