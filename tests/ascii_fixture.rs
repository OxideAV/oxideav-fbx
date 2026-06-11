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

#[test]
fn ascii_fixture_definitions_decode_with_material_template() {
    // Round 280 — the fixture's `Definitions` section is the grammar
    // §7b worked example: Version 100, total Count 13, six
    // `ObjectType` blocks (GlobalSettings without a template; the
    // other five classes with one).
    use oxideav_fbx::definitions::Definitions;

    let mut dec = FbxDecoder::new();
    let scene = dec.decode(FIXTURE).expect("ASCII fixture decodes");
    let doc = dec.last_document.as_ref().unwrap();
    let defs = Definitions::from_document(doc);
    assert_eq!(defs.version, Some(100));
    assert_eq!(defs.total_count, Some(13));
    assert_eq!(
        defs.object_types(),
        vec![
            "AnimationLayer",
            "AnimationStack",
            "Geometry",
            "GlobalSettings",
            "Material",
            "Model",
        ]
    );

    // GlobalSettings: count-only block, no PropertyTemplate.
    let gs = defs.get("GlobalSettings").unwrap();
    assert_eq!(gs.count, Some(1));
    assert!(defs.template_for("GlobalSettings").is_none());

    // Geometry / Model template names per the fixture.
    assert_eq!(
        defs.get("Geometry").unwrap().template_name.as_deref(),
        Some("FbxMesh")
    );
    assert_eq!(
        defs.get("Model").unwrap().template_name.as_deref(),
        Some("FbxNode")
    );

    // Material: 2 instances, FbxSurfaceLambert template with its full
    // 17-record default property set.
    let mat = defs.get("Material").unwrap();
    assert_eq!(mat.count, Some(2));
    assert_eq!(mat.template_name.as_deref(), Some("FbxSurfaceLambert"));
    let tpl = defs.template_for("Material").expect("Material template");
    assert_eq!(tpl.len(), 17);
    assert_eq!(tpl.as_vec3("DiffuseColor"), Some([0.8, 0.8, 0.8]));
    assert_eq!(tpl.as_f64("DiffuseFactor"), Some(1.0));
    assert_eq!(tpl.as_str("ShadingModel"), Some("Lambert"));

    // Template-default resolution must NOT override instance data:
    // Mat_Green re-states DiffuseColor (0,1,0) x DiffuseFactor
    // 0.800000011920929, and writes the direct-child leaf
    // `ShadingModel: "lambert"` that beats the template's "Lambert"
    // P-record.
    let green = scene
        .materials
        .iter()
        .find(|m| m.name.as_deref() == Some("Material::Mat_Green"))
        .expect("Mat_Green decoded");
    assert!((green.base_color[0] - 0.0).abs() < 1e-6);
    assert!((green.base_color[1] - 0.8).abs() < 1e-6);
    assert!((green.base_color[2] - 0.0).abs() < 1e-6);
    assert_eq!(
        green
            .extras
            .get("fbx:shading_model")
            .and_then(|v| v.as_str()),
        Some("lambert")
    );
}
