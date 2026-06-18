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
fn ascii_fixture_takes_surface_to_scene_extras() {
    // The fixture's `Takes` section (§7e) carries
    // `Current: "Take 001"` + one `Take: "Take 001" { FileName,
    // LocalTime: 1924423250,230930790000, ReferenceTime: ... }`.
    // Those should reach `Scene3D::extras` via the ASCII front-end
    // round-trip, joinable to the `AnimStack::Take 001` animation by
    // the matching display name.
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(FIXTURE).expect("ASCII fixture decodes");

    let current = scene
        .extras
        .get("fbx:current_take")
        .expect("Current take surfaced from fixture")
        .as_str()
        .unwrap();
    assert_eq!(current, "Take 001");

    let takes = match scene.extras.get("fbx:takes") {
        Some(serde_json::Value::Array(v)) => v,
        other => panic!("expected fbx:takes array, got {other:?}"),
    };
    assert_eq!(takes.len(), 1);
    let t = &takes[0];
    assert_eq!(t["name"].as_str(), Some("Take 001"));
    assert_eq!(t["file_name"].as_str(), Some("Take_001.tak"));
    assert_eq!(
        t["local_time"].as_array().unwrap()[0].as_i64(),
        Some(1924423250)
    );
    assert_eq!(
        t["local_time"].as_array().unwrap()[1].as_i64(),
        Some(230930790000)
    );
    assert_eq!(
        t["reference_time"].as_array().unwrap()[1].as_i64(),
        Some(230930790000)
    );

    // The take name is the join key back to the `Objects` section:
    // the fixture's `AnimationStack: "AnimStack::Take 001"` shares the
    // "Take 001" display name with this take. (The fixture's stack
    // carries no AnimationCurve records, so `extract_animations` emits
    // no channels for it and no Animation materialises — the take
    // catalogue still surfaces independently; the name is the join.)
    let stack_name = dec
        .last_document
        .as_ref()
        .unwrap()
        .root
        .child("Objects")
        .unwrap()
        .children_named("AnimationStack")
        .next()
        .unwrap()
        .properties
        .get(1)
        .and_then(|p| p.as_str())
        .unwrap();
    assert!(
        stack_name.ends_with("Take 001"),
        "AnimationStack display name should match the take name, got {stack_name:?}"
    );
}

#[test]
fn ascii_fixture_header_info_surfaces_to_scene_extras() {
    // Round 335 — the fixture's `FBXHeaderExtension` (§7a) carries
    // `Creator: "FBX SDK/FBX Plugins version 2018.1.1"`, a
    // `CreationTimeStamp` of 2019-01-07 16:17:31.730, and a
    // `SceneInfo { Properties70 { Original|ApplicationName: "Maya",
    // Original|ApplicationVendor: "Autodesk",
    // Original|ApplicationVersion: "201800", DocumentUrl: "U:\..." } }`.
    // Those should reach `Scene3D::extras` via the ASCII front-end
    // round-trip. (The fixture's `MetaData` Title/Subject/Author/...
    // are all the SDK-default empty strings, so no `fbx:meta_*` keys
    // surface — empty fields are skipped per the §7a-grounded rule.)
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(FIXTURE).expect("ASCII fixture decodes");

    assert_eq!(
        scene.extras.get("fbx:creator").and_then(|v| v.as_str()),
        Some("FBX SDK/FBX Plugins version 2018.1.1")
    );
    assert_eq!(
        scene
            .extras
            .get("fbx:header_version")
            .and_then(|v| v.as_i64()),
        Some(1003)
    );
    assert_eq!(
        scene
            .extras
            .get("fbx:creation_time")
            .and_then(|v| v.as_str()),
        Some("2019-01-07T16:17:31.730")
    );
    assert_eq!(
        scene
            .extras
            .get("fbx:application_name")
            .and_then(|v| v.as_str()),
        Some("Maya")
    );
    assert_eq!(
        scene
            .extras
            .get("fbx:application_vendor")
            .and_then(|v| v.as_str()),
        Some("Autodesk")
    );
    assert_eq!(
        scene
            .extras
            .get("fbx:application_version")
            .and_then(|v| v.as_str()),
        Some("201800")
    );
    assert_eq!(
        scene
            .extras
            .get("fbx:document_url")
            .and_then(|v| v.as_str()),
        Some(r"U:\Some\Absolute\Path\cubes_with_names.fbx")
    );
    // MetaData fields are all empty in the fixture → skipped.
    assert!(!scene.extras.contains_key("fbx:meta_title"));
    assert!(!scene.extras.contains_key("fbx:meta_author"));
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
fn ascii_fixture_first_mesh_surfaces_tangents_and_binormals() {
    // Round 301 — the fixture's first Geometry carries a
    // `LayerElementTangent` (`Tangents: *72` triple + `TangentsW: *24`
    // per-corner sign, `ByPolygonVertex` / `Direct`) and a
    // `LayerElementBinormal` (`Binormals: *72` + `BinormalsW: *24`),
    // both enumerated as Geometry LayerElement sub-discriminators in
    // `docs/3d/fbx/fbx-binary-properties70.md` §6 point 4 and shown in
    // the `docs/3d/fbx/fbx-ascii-grammar.md` §7c worked example. The
    // tangent layer must populate the canonical `Primitive::tangents`
    // slot (glTF-style `[x,y,z,w]`); the binormal layer must surface on
    // `Primitive::extras["fbx:binormals"]`.
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(FIXTURE).expect("ASCII fixture decodes");

    // Find a mesh that actually carries a tangent layer (the cube
    // Geometry; iterate so the test is robust to scene ordering).
    let prim = scene
        .meshes
        .iter()
        .flat_map(|m| m.primitives.iter())
        .find(|p| p.tangents.is_some())
        .expect("at least one primitive surfaces tangents");

    let tangents = prim.tangents.as_ref().unwrap();
    // One tangent per triangle corner (24 PolygonVertexIndex entries →
    // 6 quads → 12 triangles → 36 corners).
    assert_eq!(
        tangents.len(),
        prim.positions.len(),
        "tangents are per-corner, same length as positions"
    );
    // Every handedness sign in the fixture is +1.0 (TangentsW: all 1s).
    for t in tangents {
        assert!(
            (t[3] - 1.0).abs() < 1e-6,
            "expected +1.0 handedness sign, got {}",
            t[3]
        );
        // Tangent xyz is a unit axis vector in the fixture (1,0,0)
        // family — finite and non-degenerate.
        let len = (t[0] * t[0] + t[1] * t[1] + t[2] * t[2]).sqrt();
        assert!(len > 0.5, "tangent xyz should be ~unit length, got {len}");
    }

    // Binormals surfaced on extras as one flattened per-corner buffer.
    let binormals = prim
        .extras
        .get("fbx:binormals")
        .expect("fbx:binormals surfaced")
        .as_array()
        .expect("fbx:binormals is a JSON array");
    assert_eq!(binormals.len(), 1, "one LayerElementBinormal layer");
    let buf = binormals[0].as_array().expect("buffer is an array");
    // 4 components (x,y,z,w) per corner.
    assert_eq!(buf.len(), prim.positions.len() * 4);
    let mapping = prim
        .extras
        .get("fbx:binormals_mapping")
        .and_then(|v| v.as_array())
        .and_then(|a| a.first())
        .and_then(|v| v.as_str());
    assert_eq!(mapping, Some("ByPolygonVertex"));
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
