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
    // round-trip. UnitScaleFactor=1 maps to `Unit::Metres` (the
    // canonical metre-unit factor; factor 100 = centimetres).
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
fn ascii_fixture_model_local_transforms_reach_scene_nodes() {
    // Round 367 — each `Model`'s `Lcl Translation` / `Lcl Scaling`
    // P-records should land on the owning scene-graph node's local
    // `Transform::Trs`. The fixture's Models carry no pivots /
    // pre-post-rotation and `RotationOrder` defaults to 0 (XYZ), so the
    // node-transform chain reduces cleanly to `T * R * S`. `Cube3` is
    // the uniquely-named Model: translation
    // (-1.0671…, 0.998…, 9.3902…), scale (10, 10, 10), no rotation.
    use oxideav_mesh3d::Transform;

    let mut dec = FbxDecoder::new();
    let scene = dec.decode(FIXTURE).expect("ASCII fixture decodes");

    // ASCII object names arrive as the `Class::Name` joined identifier
    // (the §7c object-opening line), so match on the `Cube3` suffix.
    let cube3 = scene
        .nodes
        .iter()
        .find(|n| n.name.as_deref().is_some_and(|s| s.ends_with("Cube3")))
        .expect("Cube3 node present");

    match cube3.transform {
        Transform::Trs {
            translation,
            rotation,
            scale,
        } => {
            assert!((translation[0] - (-1.067_117_6)).abs() < 1e-4);
            assert!((translation[1] - 0.998_288_8).abs() < 1e-4);
            assert!((translation[2] - 9.390_235).abs() < 1e-4);
            assert!((scale[0] - 10.0).abs() < 1e-4);
            assert!((scale[1] - 10.0).abs() < 1e-4);
            assert!((scale[2] - 10.0).abs() < 1e-4);
            // No Lcl Rotation record → identity quaternion.
            assert!((rotation[3] - 1.0).abs() < 1e-5);
            assert!(rotation[0].abs() < 1e-5);
        }
        Transform::Matrix(_) => panic!("expected decomposed Trs, got Matrix"),
    }

    // The fixture's reduced chain means no node should carry the
    // lossy-reduction marker.
    assert!(
        scene
            .nodes
            .iter()
            .all(|n| !n.extras.contains_key("fbx:transform_incomplete")),
        "fixture transforms reduce to TRS; none should be marked incomplete"
    );
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

#[test]
fn ascii_fixture_cube_edges_decode_to_the_documented_pairs() {
    // Round 407 — `docs/3d/fbx/fbx-edges-smoothing-layer.md` §2 works
    // the fixture's first Geometry by hand: `Edges: *12` (values
    // 0,1,2,3,5,6,7,9,10,11,13,15) against `PolygonVertexIndex: *24`
    // decodes to exactly the 12 undirected edges of the cube. The
    // decoder must surface those pairs (indices into the shared
    // vertex table) on `Primitive::extras["fbx:edges"]`.
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(FIXTURE).expect("ASCII fixture decodes");

    // The three plain-cube Geometries carry 12-edge tables; find one
    // (robust to scene ordering).
    let prim = scene
        .meshes
        .iter()
        .flat_map(|m| m.primitives.iter())
        .find(|p| {
            p.extras
                .get("fbx:edges")
                .and_then(|v| v.as_array())
                .is_some_and(|a| a.len() == 24)
        })
        .expect("a cube primitive surfaces 12 decoded edges");

    let flat: Vec<i64> = prim.extras["fbx:edges"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_i64().unwrap())
        .collect();
    // The §2 worked-decode table, in Edges-array order.
    assert_eq!(
        flat,
        vec![0, 1, 1, 3, 3, 2, 2, 0, 3, 5, 5, 4, 4, 2, 5, 7, 7, 6, 6, 4, 7, 1, 0, 6]
    );
}

#[test]
fn ascii_fixture_cube_smoothing_is_by_edge_all_hard() {
    // Round 407 — the fixture's cube Geometry carries a
    // `LayerElementSmoothing` mapped `ByEdge` / `Direct` with
    // `Smoothing: *12` all zeros (§4a of the staged doc: 0 = hard
    // edge → a fully faceted cube, consistent with its
    // `ByPolygonVertex` per-face normals).
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(FIXTURE).expect("ASCII fixture decodes");

    let prim = scene
        .meshes
        .iter()
        .flat_map(|m| m.primitives.iter())
        .find(|p| {
            p.extras
                .get("fbx:edges")
                .and_then(|v| v.as_array())
                .is_some_and(|a| a.len() == 24)
        })
        .expect("a cube primitive surfaces 12 decoded edges");

    assert_eq!(
        prim.extras["fbx:smoothing_mapping"].as_str(),
        Some("ByEdge")
    );
    // Per-edge flags: 12 zeros, aligned with fbx:edges.
    let per_edge: Vec<i64> = prim.extras["fbx:edge_smoothing"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_i64().unwrap())
        .collect();
    assert_eq!(per_edge, vec![0i64; 12]);
    // Per-corner resolution: every corner's starting edge is hard.
    let per_corner: Vec<i64> = prim.extras["fbx:smoothing"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_i64().unwrap())
        .collect();
    assert_eq!(per_corner, vec![0i64; prim.positions.len()]);
    assert_eq!(prim.positions.len(), 36, "6 quads → 12 tris → 36 corners");
}

#[test]
fn ascii_fixture_subdivided_cube_edges_all_decode_distinct() {
    // Round 407 — the fixture's fourth Geometry (the smooth-mesh
    // preview cube) carries `Edges: *384` over a 768-entry
    // `PolygonVertexIndex` (192 quads). Every entry must decode
    // in-range, and — per §1 ("Edges is the deduplicated edge set") —
    // the 384 undirected pairs must all be distinct, with no
    // degenerate self-edges.
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(FIXTURE).expect("ASCII fixture decodes");

    let prim = scene
        .meshes
        .iter()
        .flat_map(|m| m.primitives.iter())
        .find(|p| {
            p.extras
                .get("fbx:edges")
                .and_then(|v| v.as_array())
                .is_some_and(|a| a.len() == 384 * 2)
        })
        .expect("the subdivided cube surfaces 384 decoded edges");

    let flat: Vec<i64> = prim.extras["fbx:edges"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_i64().unwrap())
        .collect();
    let mut set: Vec<(i64, i64)> = flat
        .chunks_exact(2)
        .map(|p| (p[0].min(p[1]), p[0].max(p[1])))
        .collect();
    for &(a, b) in &set {
        assert_ne!(a, b, "degenerate self-edge ({a},{b})");
    }
    set.sort_unstable();
    set.dedup();
    assert_eq!(set.len(), 384, "all 384 undirected edges distinct");

    // Its smoothing layer is ByEdge/Direct over the same 384-edge
    // domain.
    assert_eq!(
        prim.extras["fbx:smoothing_mapping"].as_str(),
        Some("ByEdge")
    );
    assert_eq!(
        prim.extras["fbx:edge_smoothing"].as_array().unwrap().len(),
        384
    );
    assert_eq!(
        prim.extras["fbx:smoothing"].as_array().unwrap().len(),
        prim.positions.len()
    );
}

#[test]
fn ascii_fixture_edges_arrays_are_the_complete_deduplicated_edge_sets() {
    // Round 407 — §1 of fbx-edges-smoothing-layer.md states `Edges`
    // enumerates the mesh's unique edges, duplicates shared by two
    // polygons listed once. Double-entry check: for every Geometry in
    // the fixture, independently derive the unique undirected edge
    // set from the raw `PolygonVertexIndex` (each corner starts one
    // polygon edge, running to the next corner in the same polygon
    // with wrap at the negative closing corner) and assert the
    // decoder-surfaced `fbx:edges` pairs are exactly that set — same
    // members, no duplicates, nothing missing.
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(FIXTURE).expect("ASCII fixture decodes");
    let doc = dec.last_document.as_ref().unwrap();

    let geometries: Vec<&oxideav_fbx::FbxNode> = doc
        .root
        .child("Objects")
        .expect("Objects section")
        .children_named("Geometry")
        .collect();
    assert_eq!(geometries.len(), 4, "fixture carries four Geometry nodes");

    // Independent derivation from each raw PolygonVertexIndex.
    let derived_sets: Vec<Vec<(i64, i64)>> = geometries
        .iter()
        .map(|g| {
            let pvi = match &g.child("PolygonVertexIndex").unwrap().properties[0] {
                oxideav_fbx::FbxProperty::I32Array(a) => a.clone(),
                other => panic!("PolygonVertexIndex not i32: {other:?}"),
            };
            let decode = |raw: i32| -> i64 {
                if raw < 0 {
                    (!raw) as i64
                } else {
                    raw as i64
                }
            };
            let mut set = Vec::new();
            let mut start = 0usize;
            for (k, &raw) in pvi.iter().enumerate() {
                let next = if raw < 0 { start } else { k + 1 };
                if raw < 0 {
                    start = k + 1;
                }
                let a = decode(raw);
                let b = decode(pvi[next]);
                set.push((a.min(b), a.max(b)));
            }
            set.sort_unstable();
            set.dedup();
            set
        })
        .collect();

    // Decoder-surfaced fbx:edges, one per mesh, matched by edge count
    // (three 12-edge cubes + one 384-edge subdivided cube).
    let mut surfaced: Vec<Vec<(i64, i64)>> = scene
        .meshes
        .iter()
        .flat_map(|m| m.primitives.iter())
        .filter_map(|p| p.extras.get("fbx:edges"))
        .map(|v| {
            let flat: Vec<i64> = v
                .as_array()
                .unwrap()
                .iter()
                .map(|x| x.as_i64().unwrap())
                .collect();
            let mut set: Vec<(i64, i64)> = flat
                .chunks_exact(2)
                .map(|p| (p[0].min(p[1]), p[0].max(p[1])))
                .collect();
            let n = set.len();
            set.sort_unstable();
            set.dedup();
            assert_eq!(set.len(), n, "fbx:edges lists each edge once");
            set
        })
        .collect();
    assert_eq!(surfaced.len(), 4, "every Geometry surfaced an edge set");

    let mut derived = derived_sets;
    derived.sort();
    surfaced.sort();
    assert_eq!(
        derived, surfaced,
        "each Edges array is exactly the mesh's deduplicated edge set"
    );
}

#[test]
fn ascii_fixture_documents_surface_to_scene_extras() {
    // Round 413 — the fixture's `Documents` section (per the §7
    // top-level section list) carries `Count: 1` + one
    // `Document: <uid>, "", "Scene"` whose Properties70 holds
    // `P: "ActiveAnimStackName", "KString", "", "", "Take 001"`.
    // The catalogue surfaces on `Scene3D::extras`, and the active
    // stack name joins to the `Takes` `Current` leaf / the
    // `AnimStack::Take 001` display name.
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(FIXTURE).expect("ASCII fixture decodes");

    assert_eq!(
        oxideav_fbx::documents::active_anim_stack_from_extras(&scene),
        Some("Take 001")
    );

    let docs =
        oxideav_fbx::documents::documents_from_extras(&scene).expect("fbx:documents surfaced");
    assert_eq!(docs.len(), 1);
    let d = &docs[0];
    assert_eq!(d["name"].as_str(), Some(""));
    assert_eq!(d["subtype"].as_str(), Some("Scene"));
    assert_eq!(d["active_anim_stack"].as_str(), Some("Take 001"));

    // Join keys agree: Documents' ActiveAnimStackName == Takes'
    // Current take name.
    assert_eq!(
        scene
            .extras
            .get("fbx:current_take")
            .and_then(|v| v.as_str()),
        Some("Take 001")
    );
}
