//! End-to-end ASCII-FBX tests against the staged
//! `docs/3d/fbx/fixtures/cubes-ascii-v7500.fbx` fixture (round 200).
//!
//! The ASCII front-end produces the same typed [`oxideav_fbx::FbxDocument`]
//! tree as the binary reader, so this exercises the whole decoder
//! pipeline (sniff → parse → scene::build_scene) end-to-end on a real
//! exporter-produced file.

use oxideav_fbx::{is_ascii_fbx, FbxDecoder};
use oxideav_mesh3d::Mesh3DDecoder;

const FIXTURE: &[u8] = include_bytes!("fixtures/cubes-ascii-v7500.fbx");

#[test]
fn sniffer_recognises_ascii_fixture() {
    assert!(is_ascii_fbx(FIXTURE));
}

#[test]
fn ascii_fixture_decodes_to_scene_with_meshes() {
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(FIXTURE).expect("ASCII fixture decodes");
    // The fixture has 4 Geometry elements; each produces at least
    // one mesh in the scene builder's pass.
    assert!(
        !scene.meshes.is_empty(),
        "expected at least one mesh, got {}",
        scene.meshes.len()
    );
    // Document captured.
    let doc = dec.last_document.as_ref().unwrap();
    assert_eq!(doc.version, 7500);
    // Every top-level section the §7 grammar listing claims.
    for s in &[
        "FBXHeaderExtension",
        "GlobalSettings",
        "Documents",
        "References",
        "Definitions",
        "Objects",
        "Connections",
        "Takes",
    ] {
        assert!(
            doc.root.child(s).is_some(),
            "missing top-level section: {s}"
        );
    }
}

#[test]
fn ascii_fixture_global_settings_surface_to_scene_extras() {
    // Round 219 — the fixture's `GlobalSettings { Properties70 { ... } }`
    // block (UnitScaleFactor=1, AmbientColor=(0,0,0), TimeMode=11,
    // ...) should reach `Scene3D::extras` via the ASCII front-end
    // round-trip. UnitScaleFactor=1 maps to `Unit::Metres` per the
    // documented mapping in `docs/3d/fbx/ufbx/elements-nodes.md`.
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(FIXTURE).expect("ASCII fixture decodes");
    assert_eq!(scene.unit, oxideav_mesh3d::Unit::Metres);
    let usf = scene
        .extras
        .get("fbx:unit_scale_factor")
        .expect("UnitScaleFactor surfaced from fixture")
        .as_f64()
        .unwrap();
    assert!((usf - 1.0).abs() < 1e-9);
    let cam = scene
        .extras
        .get("fbx:default_camera")
        .expect("DefaultCamera surfaced from fixture")
        .as_str()
        .unwrap();
    assert_eq!(cam, "Producer Perspective");
    let tm = scene
        .extras
        .get("fbx:time_mode")
        .expect("TimeMode surfaced from fixture")
        .as_i64()
        .unwrap();
    assert_eq!(tm, 11);
}

#[test]
fn ascii_fixture_first_mesh_has_24_vertices() {
    // First Geometry in the fixture is `*24` Vertices (an 8-corner
    // cube emitted as 8 xyz triples) per the grammar §6 / §7c worked
    // example. Walk through the parsed doc to verify the ASCII
    // front-end's typed-array decode reaches the geometry walker
    // intact.
    let mut dec = FbxDecoder::new();
    let _ = dec.decode(FIXTURE).expect("ASCII fixture decodes");
    let doc = dec.last_document.as_ref().unwrap();
    let objs = doc.root.child("Objects").unwrap();
    let g0 = objs
        .children_named("Geometry")
        .next()
        .expect("at least one Geometry");
    let verts = &g0
        .child("Vertices")
        .expect("Vertices sub-record")
        .properties[0];
    match verts {
        oxideav_fbx::FbxProperty::F64Array(v) => assert_eq!(v.len(), 24),
        other => panic!("expected F64Array(24), got {other:?}"),
    }
}
