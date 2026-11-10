//! The 2026-09 export-sweep fixtures (`docs/3d/fbx/fixtures/blender-*/`
//! and `nurbs-curves-ascii-v7200/`, described in
//! `docs/3d/fbx/blender-export-sweeps-2026-09.md`): every fixture
//! decodes to the scene its `notes.md` describes, and our re-encode
//! carries the same wire facts back.
//!
//! - `keyattr-sweep-binary-v7400.fbx` (sweep §2): nine curves, each
//!   with the one-entry sampled-curve attribute block; the authored
//!   interpolation survives only in the per-frame samples.
//! - `texture-placement-binary-v7400.fbx` (§3): `WrapModeU/V` 0 / 1,
//!   placement `Translation` / `Rotation` (radians) / `Scaling`.
//! - `blendshape-*-binary-v7400.fbx` (§4): five one-Shape channels,
//!   `DeformPercent` leaves, `FullWeights` `d[len(Indexes)]` percent.
//! - `nurbs-as-mesh-binary-v7400.fbx` (§5.1): curves and surfaces as
//!   point-only / tessellated `Mesh` geometry, no NURBS records.
//! - `nurbs-curves-ascii-v7200.fbx` (§5.2): five `NurbsCurve`
//!   geometries (Open / Closed / Periodic, rational and not, order 3
//!   and 4).
//!
//! The corpus lives in the workspace docs checkout; every test skips
//! cleanly when it is absent.

use std::path::PathBuf;

use oxideav_fbx::binary::{FbxDocument, FbxNode, FbxProperty};
use oxideav_fbx::nurbs_geometry::NURBS_CURVE_KEY;
use oxideav_fbx::{FbxDecoder, FbxEncoder, FbxNurbsCurve, FbxOutputForm, NurbsForm};
use oxideav_mesh3d::{
    Animation, AnimationChannel, AnimationProperty, AnimationSampler, AnimationValues,
    Interpolation, Mesh3DDecoder, Mesh3DEncoder, Node, Scene3D, Topology, WrapMode,
};
use serde_json::Value;

fn fixtures_dir() -> Option<PathBuf> {
    if let Ok(dir) = std::env::var("OXIDEAV_FBX_DOCS_FIXTURES") {
        let p = PathBuf::from(dir);
        return p.is_dir().then_some(p);
    }
    let p = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../docs/3d/fbx/fixtures");
    p.is_dir().then_some(p)
}

fn fixture(name: &str) -> Option<Vec<u8>> {
    let dir = fixtures_dir()?;
    match std::fs::read(dir.join(name)) {
        Ok(b) => Some(b),
        Err(_) => {
            eprintln!("skipping: {name} not present under {}", dir.display());
            None
        }
    }
}

const KEYATTR: &str = "blender-keyattr-sweep/keyattr-sweep-binary-v7400.fbx";
const TEXTURE: &str = "blender-texture-placement/texture-placement-binary-v7400.fbx";
const SHAPES: &str = "blender-blendshape/blendshape-channels-binary-v7400.fbx";
const SHAPES_ANIM: &str = "blender-blendshape/blendshape-anim-binary-v7400.fbx";
const NURBS_AS_MESH: &str = "blender-nurbs-export/nurbs-as-mesh-binary-v7400.fbx";
const NURBS_CURVES: &str = "nurbs-curves-ascii-v7200/nurbs-curves-ascii-v7200.fbx";

fn decode(bytes: &[u8]) -> Scene3D {
    FbxDecoder::new().decode(bytes).expect("decode")
}

fn encode(scene: &Scene3D, form: FbxOutputForm) -> Vec<u8> {
    FbxEncoder::new().form(form).encode(scene).expect("encode")
}

fn parse_any(bytes: &[u8]) -> FbxDocument {
    if oxideav_fbx::is_ascii_fbx(bytes) {
        oxideav_fbx::ascii::parse(bytes).expect("ascii parse")
    } else {
        oxideav_fbx::binary::parse(bytes).expect("binary parse")
    }
}

fn objects(doc: &FbxDocument) -> &FbxNode {
    doc.root.child("Objects").expect("Objects")
}

/// Display name of an element (`Name\x00\x01Class`).
fn display(n: &FbxNode) -> String {
    let raw = n
        .properties
        .get(1)
        .and_then(FbxProperty::as_str)
        .unwrap_or("");
    raw.split('\u{0}').next().unwrap_or("").to_owned()
}

fn node_named<'a>(scene: &'a Scene3D, name: &str) -> (usize, &'a Node) {
    scene
        .nodes
        .iter()
        .enumerate()
        .find(|(_, n)| n.name.as_deref() == Some(name))
        .unwrap_or_else(|| panic!("node {name}"))
}

// ---------------------------------------------------------------------
// #336 (1) — KeyAttrFlags sweep
// ---------------------------------------------------------------------

fn key_attrs(scene: &Scene3D) -> &Vec<Value> {
    scene.extras["fbx:key_attrs"].as_array().expect("key_attrs")
}

#[test]
fn keyattr_sweep_curves_carry_the_sampled_block() {
    let Some(bytes) = fixture(KEYATTR) else {
        return;
    };
    let scene = decode(&bytes);
    let entries = key_attrs(&scene);
    assert_eq!(entries.len(), 9, "one animated curve per Empty");
    for e in entries {
        let target = e["target"].as_str().unwrap();
        let keys = if target == "C_CONSTANT" { 5 } else { 21 };
        assert_eq!(e["key_count"].as_u64(), Some(keys), "{target}");
        assert_eq!(e["flags"], serde_json::json!([24836]), "{target}");
        assert_eq!(
            e["data_bits"],
            serde_json::json!([0, 0, 0x0F3F_0F3F_u32, 0]),
            "{target}"
        );
        assert_eq!(e["ref_count"], serde_json::json!([keys]), "{target}");
        assert_eq!(e["sampled_block"], Value::Bool(true), "{target}");
        assert_eq!(e["axis"].as_str(), Some("d|X"));
        assert_eq!(e["key_ver"].as_i64(), Some(4008));
    }
}

/// The per-frame samples carry the authored curve shape (sweep §2.2,
/// values in cm; frame f at f × 1 924 423 250 ticks = f / 24 s).
#[test]
fn keyattr_sweep_samples_decode_per_frame() {
    let Some(bytes) = fixture(KEYATTR) else {
        return;
    };
    let scene = decode(&bytes);
    let anim = &scene.animations[0];
    let x_of = |name: &str| -> (Vec<f32>, Vec<f32>) {
        let (ni, _) = node_named(&scene, name);
        let ch = anim
            .channels
            .iter()
            .find(|c| {
                c.target.node.0 as usize == ni
                    && c.target.property == AnimationProperty::Translation
            })
            .unwrap_or_else(|| panic!("{name} translation channel"));
        let AnimationValues::Vec3(v) = &ch.sampler.values else {
            panic!("vec3 values")
        };
        (
            ch.sampler.keyframes.clone(),
            v.iter().map(|p| p[0]).collect(),
        )
    };
    let close = |a: f32, b: f32| (a - b).abs() < 1e-3;

    let (t, x) = x_of("C_CONSTANT");
    let frames: Vec<f32> = t.iter().map(|s| s * 24.0).collect();
    for (f, want) in frames.iter().zip([1.0, 10.0, 11.0, 20.0, 21.0]) {
        assert!((f - want).abs() < 1e-3, "frames {frames:?}");
    }
    for (v, want) in x.iter().zip([0.0, 0.0, 500.0, 500.0, 200.0]) {
        assert!(close(*v, want), "C_CONSTANT {x:?}");
    }
    // f1, f6, f11, f16, f21 per curve.
    for (name, want) in [
        ("C_BEZ_ALIGNED", [0.0, 250.0, 500.0, 350.0, 200.0]),
        ("C_BEZ_AUTO", [0.0, 231.25, 500.0, 368.75, 200.0]),
        ("C_BEZ_EXTRAP", [0.0, 343.75, 500.0, 406.25, 200.0]),
        ("C_BEZ_FREE_BROKEN", [0.0, 387.117, 500.0, 381.081, 200.0]),
        ("C_LINEAR", [0.0, 250.0, 500.0, 350.0, 200.0]),
    ] {
        let (t, x) = x_of(name);
        assert_eq!(t.len(), 21, "{name}");
        for (k, w) in [0usize, 5, 10, 15, 20].iter().zip(want) {
            assert!(close(x[*k], w), "{name}[{k}] = {} want {w}", x[*k]);
        }
    }
}

/// Re-encode keeps every source curve verbatim — same keys, same
/// attribute block — in both output forms.
#[test]
fn keyattr_sweep_block_survives_re_encode() {
    let Some(bytes) = fixture(KEYATTR) else {
        return;
    };
    let scene = decode(&bytes);
    for form in [FbxOutputForm::Binary, FbxOutputForm::Ascii] {
        let again = decode(&encode(&scene, form));
        assert_eq!(key_attrs(&again), key_attrs(&scene), "{form:?}");
    }
}

/// A scene built through the API (no attribute catalogue) gets the
/// sampled-curve block on every curve the writer emits.
#[test]
fn api_authored_curves_get_the_sampled_block() {
    let mut scene = Scene3D::new();
    let n = scene.add_node(Node::new().with_name("Mover"));
    scene.roots.push(n);
    let mut anim = Animation::new(Some("Take".to_string()));
    anim.channels.push(AnimationChannel::new(
        n,
        AnimationProperty::Translation,
        AnimationSampler {
            keyframes: vec![0.0, 0.5, 1.0],
            values: AnimationValues::Vec3(vec![[0.0; 3], [1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]),
            interpolation: Interpolation::Linear,
        },
    ));
    scene.animations.push(anim);
    for form in [FbxOutputForm::Binary, FbxOutputForm::Ascii] {
        let bytes = encode(&scene, form);
        let doc = parse_any(&bytes);
        let curves: Vec<&FbxNode> = objects(&doc).children_named("AnimationCurve").collect();
        assert_eq!(curves.len(), 3, "{form:?}");
        for c in curves {
            let flags = c.child("KeyAttrFlags").expect("flags");
            assert!(matches!(&flags.properties[0], FbxProperty::I32Array(a) if a == &[24836]));
            let rc = c.child("KeyAttrRefCount").expect("refcount");
            assert!(matches!(&rc.properties[0], FbxProperty::I32Array(a) if a == &[3]));
        }
        let again = decode(&bytes);
        for e in key_attrs(&again) {
            assert_eq!(e["sampled_block"], Value::Bool(true), "{form:?}");
        }
    }
}

// ---------------------------------------------------------------------
// #371 — texture placement
// ---------------------------------------------------------------------

#[test]
fn texture_placement_decodes_wrap_and_transform() {
    let Some(bytes) = fixture(TEXTURE) else {
        return;
    };
    let scene = decode(&bytes);
    let tref = |obj: &str| {
        let mat = scene
            .materials
            .iter()
            .find(|m| m.name.as_deref() == Some(&format!("{obj}_mat")))
            .unwrap_or_else(|| panic!("{obj}_mat"));
        mat.base_color_texture.expect("base colour texture")
    };
    let rot30 = -std::f32::consts::FRAC_PI_6;
    let third = 0.333_333_34_f32;
    for (obj, clamp, offset, rotation, scale) in [
        ("T00_DEFAULT", false, None, None, None),
        ("T01_EXT_EXTEND", true, None, None, None),
        ("T02_EXT_CLIP", true, None, None, None),
        ("T03_EXT_MIRROR", true, None, None, None),
        (
            "T04_POINT_LOC",
            false,
            Some([0.25, 0.5]),
            Some(0.0),
            Some([1.0, 1.0]),
        ),
        (
            "T05_POINT_ROTZ30",
            false,
            Some([0.0, 0.0]),
            Some(rot30),
            Some([1.0, 1.0]),
        ),
        (
            "T06_POINT_SCALE",
            false,
            Some([0.0, 0.0]),
            Some(0.0),
            Some([0.5, third]),
        ),
        (
            "T07_POINT_TRS",
            false,
            Some([0.25, 0.5]),
            Some(rot30),
            Some([0.5, third]),
        ),
        (
            "T08_TEXTURE_TRS",
            false,
            Some([0.25, 0.5]),
            Some(rot30),
            Some([0.5, third]),
        ),
        (
            "T10_VECTOR_TRS",
            false,
            Some([0.0, 0.0]),
            Some(rot30),
            Some([0.5, third]),
        ),
        (
            "T11_POINT_NEGSCALE",
            false,
            Some([0.0, 0.0]),
            Some(0.0),
            Some([-1.0, 1.0]),
        ),
        ("T12_UVSET2", false, None, None, None),
        (
            "T13_EXT_EXTEND_TRS",
            true,
            Some([0.25, 0.5]),
            Some(rot30),
            Some([0.5, third]),
        ),
    ] {
        let r = tref(obj);
        let sampler = scene.textures[r.texture.0 as usize].sampler;
        let wrap = if clamp {
            WrapMode::ClampToEdge
        } else {
            WrapMode::Repeat
        };
        assert_eq!((sampler.wrap_s, sampler.wrap_t), (wrap, wrap), "{obj}");
        assert_eq!(r.uv_set, 0, "{obj}: no UVSet is ever written");
        match offset {
            None => assert_eq!(r.transform, None, "{obj}"),
            Some(o) => {
                let t = r.transform.unwrap_or_else(|| panic!("{obj} transform"));
                assert_eq!(t.offset, o, "{obj}");
                assert!((t.rotation - rotation.unwrap()).abs() < 1e-6, "{obj}");
                assert_eq!(t.scale, scale.unwrap(), "{obj}");
            }
        }
    }
    // T09 rotates about X: no planar home, the record rides raw.
    let r = tref("T09_POINT_ROTX45");
    assert_eq!(r.transform, None);
    let raw = &scene.extras["fbx:texture_records"][r.texture.0.to_string()];
    let rot: Vec<f64> = raw["rotation"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_f64().unwrap())
        .collect();
    assert_eq!(rot[0], -0.7853981852531433);
}

/// Every `Texture` element's `Properties70` re-encodes value-exact
/// (the f64 placement triples, the `WrapMode` integers, nothing
/// invented — no `UVSet`, no pivots).
#[test]
fn texture_placement_records_re_encode_exactly() {
    let Some(bytes) = fixture(TEXTURE) else {
        return;
    };
    let records = |doc: &FbxDocument| -> Vec<String> {
        let mut out: Vec<String> = objects(doc)
            .children_named("Texture")
            .flat_map(|t| {
                t.child("Properties70")
                    .map(|p| p.children.clone())
                    .unwrap_or_default()
            })
            .map(|p| format!("{:?}", p.properties))
            .collect();
        out.sort();
        out
    };
    let source = records(&parse_any(&bytes));
    let scene = decode(&bytes);
    for form in [FbxOutputForm::Binary, FbxOutputForm::Ascii] {
        let mut ours = records(&parse_any(&encode(&scene, form)));
        if form == FbxOutputForm::Ascii {
            // The ASCII grammar has one integer token: scalar ints
            // read back as `L` (the writer module's documented form
            // limit), so compare the integer value only.
            for r in &mut ours {
                *r = r.replace("I64(", "I32(");
            }
        }
        assert_eq!(ours, source, "{form:?}");
    }
}

// ---------------------------------------------------------------------
// #398 — blend-shape channels and FullWeights
// ---------------------------------------------------------------------

const KEYS: [&str; 5] = [
    "K1_UpHalf",
    "K2_UpFull_relK1",
    "K3_Corner",
    "K4_Tip_relK2",
    "K5_VGroup",
];

#[test]
fn blendshape_channels_decode_names_weights_and_full_weights() {
    let Some(bytes) = fixture(SHAPES) else {
        return;
    };
    let scene = decode(&bytes);
    assert_eq!(scene.meshes.len(), 1);
    let mesh = &scene.meshes[0];
    assert_eq!(mesh.target_names, KEYS);
    // `DeformPercent` leaves ÷ 100.
    assert_eq!(mesh.weights, [0.5, 0.0, 0.25, 0.0, 1.0]);
    let prim = &mesh.primitives[0];
    assert_eq!(prim.targets.len(), 5, "one Shape per channel");
    let fw = prim.extras["fbx:blend_full_weights"].as_array().unwrap();
    assert_eq!(fw.len(), 5);
    for (slot, entry) in fw.iter().enumerate() {
        let ix: Vec<i64> = entry["indexes"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_i64().unwrap())
            .collect();
        let w: Vec<f64> = entry["weights"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_f64().unwrap())
            .collect();
        match slot {
            2 => assert_eq!((ix, w), (vec![0], vec![100.0])),
            4 => assert_eq!((ix, w), (vec![0, 2, 8], vec![25.0, 0.0, 75.0])),
            _ => assert_eq!((ix, w), ((0..9).collect(), vec![100.0; 9])),
        }
    }
    // Deltas relative to each key's own reference (sweep §4.1).
    let max_dz = |slot: usize| -> f32 {
        prim.targets[slot]
            .position
            .as_ref()
            .unwrap()
            .iter()
            .map(|d| d[2])
            .fold(f32::MIN, f32::max)
    };
    assert_eq!(max_dz(0), 0.5);
    assert_eq!(max_dz(1), 0.5, "K2 stored relative to K1");
    assert_eq!(max_dz(3), 0.5, "K4 stored relative to K2");
    let k3 = prim.targets[2].position.as_ref().unwrap();
    assert!(k3.iter().any(|d| *d == [-0.5, 0.0, 0.0]));
}

/// Locate a 32-bit-header node record by name in a v7400 file and
/// return `(record start, end offset, property-list length,
/// property bytes)`.
fn find_record<'a>(bytes: &'a [u8], name: &str) -> Vec<(usize, usize, usize, &'a [u8])> {
    let mut needle = vec![name.len() as u8];
    needle.extend_from_slice(name.as_bytes());
    let mut out = Vec::new();
    for at in 0..bytes.len().saturating_sub(needle.len()) {
        if at < 12 || bytes[at..at + needle.len()] != needle[..] {
            continue;
        }
        let start = at - 12;
        let u32_at = |o: usize| u32::from_le_bytes(bytes[o..o + 4].try_into().unwrap()) as usize;
        let end = u32_at(start);
        let num = u32_at(start + 4);
        let len = u32_at(start + 8);
        if num != 1 || end <= start || end > bytes.len() {
            continue;
        }
        let props = at + needle.len();
        out.push((start, end, len, &bytes[props..props + len]));
    }
    out
}

/// The sweep §4.1 byte pins hold on our binary re-encode: `FullWeights`
/// is one `d` array, encoding 0, `len(Indexes)` elements in percent;
/// `DeformPercent` is one `D`; neither record carries a null-record
/// terminator (`EndOffset` = end of the property list).
#[test]
fn blendshape_full_weights_byte_pins_on_re_encode() {
    let Some(bytes) = fixture(SHAPES) else {
        return;
    };
    let ours = encode(&decode(&bytes), FbxOutputForm::Binary);
    for (label, file) in [("source", &bytes), ("ours", &ours)] {
        let fws = find_record(file, "FullWeights");
        assert_eq!(fws.len(), 5, "{label}");
        let mut lens: Vec<usize> = Vec::new();
        for (start, end, len, props) in &fws {
            assert_eq!(*end, start + 13 + 11 + len, "{label}: no terminator");
            assert_eq!(props[0], b'd', "{label}");
            let n = u32::from_le_bytes(props[1..5].try_into().unwrap()) as usize;
            let enc = u32::from_le_bytes(props[5..9].try_into().unwrap());
            let clen = u32::from_le_bytes(props[9..13].try_into().unwrap()) as usize;
            assert_eq!((enc, clen), (0, 8 * n), "{label}");
            lens.push(n);
        }
        lens.sort();
        assert_eq!(lens, [1, 3, 9, 9, 9], "{label}");
        // K3's record, value 100.0 — the pinned bytes.
        assert!(
            fws.iter()
                .any(|r| r.3 == b"d\x01\0\0\0\0\0\0\0\x08\0\0\0\0\0\0\0\0\0\x59\x40"),
            "{label}: K3 FullWeights bytes"
        );
        let dps = find_record(file, "DeformPercent");
        let leaves: Vec<_> = dps.iter().filter(|r| r.3.first() == Some(&b'D')).collect();
        assert_eq!(leaves.len(), 5, "{label}");
        for (start, end, len, _) in leaves {
            assert_eq!(*end, start + 13 + 13 + len, "{label}");
        }
    }
}

/// Channel / Shape / BlendShape bodies re-encode with the staged leaf
/// set, and the decoded scene is unchanged by the trip.
#[test]
fn blendshape_bodies_re_encode_with_the_staged_leaf_set() {
    let Some(bytes) = fixture(SHAPES) else {
        return;
    };
    let scene = decode(&bytes);
    for form in [FbxOutputForm::Binary, FbxOutputForm::Ascii] {
        let out = encode(&scene, form);
        let doc = parse_any(&out);
        let leaf_names = |sub: &str| -> Vec<Vec<String>> {
            objects(&doc)
                .children
                .iter()
                .filter(|n| n.properties.get(2).and_then(FbxProperty::as_str) == Some(sub))
                .map(|n| n.children.iter().map(|c| c.name.clone()).collect())
                .collect()
        };
        assert_eq!(leaf_names("BlendShape"), [["Version"]], "{form:?}");
        for c in leaf_names("BlendShapeChannel") {
            assert_eq!(c, ["Version", "DeformPercent", "FullWeights"], "{form:?}");
        }
        for s in leaf_names("Shape") {
            assert_eq!(
                s,
                ["Properties70", "Version", "Indexes", "Vertices", "Normals"],
                "{form:?}"
            );
        }
        let again = decode(&out);
        assert_eq!(again.meshes[0].target_names, KEYS);
        assert_eq!(again.meshes[0].weights, scene.meshes[0].weights);
        assert_eq!(
            again.meshes[0].primitives[0].extras["fbx:blend_full_weights"],
            scene.meshes[0].primitives[0].extras["fbx:blend_full_weights"],
            "{form:?}"
        );
    }
}

/// The animated file: K1 / K2 weight curves drive the merged
/// `MorphWeights` channel; each animated key's second curve node (on
/// the base geometry's per-key `Number` property) re-wires onto the
/// re-encoded geometry; every source curve keeps its attribute block.
#[test]
fn blendshape_anim_curves_round_trip() {
    let Some(bytes) = fixture(SHAPES_ANIM) else {
        return;
    };
    let scene = decode(&bytes);
    let anim = &scene.animations[0];
    let ch = anim
        .channels
        .iter()
        .find(|c| c.target.property == AnimationProperty::MorphWeights)
        .expect("morph channel");
    let frames = ch.sampler.morph_weight_frames().expect("frames");
    assert_eq!(ch.sampler.morph_weight_stride(), Some(5));
    assert!((frames[0][0] - 0.0).abs() < 1e-6);
    assert!((frames.last().unwrap()[0] - 1.0).abs() < 1e-6);
    assert!((frames.last().unwrap()[1] - 1.0).abs() < 1e-6);
    let aux = scene.extras["fbx:aux_curve_nodes"].as_array().unwrap();
    let props: Vec<&str> = aux.iter().filter_map(|a| a["property"].as_str()).collect();
    assert_eq!(props, ["K1_UpHalf", "K2_UpFull_relK1"]);
    for a in aux {
        assert!(a["target"].get("geometry_of").is_some(), "{a}");
    }
    for form in [FbxOutputForm::Binary, FbxOutputForm::Ascii] {
        let again = decode(&encode(&scene, form));
        assert_eq!(
            again.extras["fbx:aux_curve_nodes"], scene.extras["fbx:aux_curve_nodes"],
            "{form:?}"
        );
        // Same entries (the catalogue follows document order, which
        // the writer may permute).
        let sorted = |s: &Scene3D| {
            let mut v: Vec<String> = key_attrs(s).iter().map(|e| e.to_string()).collect();
            v.sort();
            v
        };
        assert_eq!(sorted(&again), sorted(&scene), "{form:?}");
        assert_eq!(key_attrs(&again).len(), 4);
    }
}

/// In-betweens: the staged producer writes exactly one `Shape` per
/// channel (chained keys become separate channels), so every decoded
/// target has no in-between stations.
#[test]
fn blendshape_fixtures_carry_no_in_betweens() {
    for name in [SHAPES, SHAPES_ANIM] {
        let Some(bytes) = fixture(name) else {
            continue;
        };
        let doc = parse_any(&bytes);
        let mut shapes_per_channel: std::collections::HashMap<i64, usize> = Default::default();
        let channels: std::collections::HashSet<i64> = objects(&doc)
            .children
            .iter()
            .filter(|n| {
                n.properties.get(2).and_then(FbxProperty::as_str) == Some("BlendShapeChannel")
            })
            .filter_map(|n| n.properties[0].as_i64())
            .collect();
        for c in doc.root.child("Connections").unwrap().children_named("C") {
            if c.properties[0].as_str() == Some("OO")
                && let Some(parent) = c.properties.get(2).and_then(FbxProperty::as_i64)
                && channels.contains(&parent)
            {
                *shapes_per_channel.entry(parent).or_default() += 1;
            }
        }
        assert_eq!(shapes_per_channel.len(), 5);
        assert!(shapes_per_channel.values().all(|n| *n == 1), "{name}");
        let scene = decode(&bytes);
        for t in &scene.meshes[0].primitives[0].targets {
            assert!(t.inbetweens.is_empty());
        }
    }
}

// ---------------------------------------------------------------------
// #381 — NURBS
// ---------------------------------------------------------------------

#[test]
fn nurbs_as_mesh_fixture_decodes_as_plain_meshes() {
    let Some(bytes) = fixture(NURBS_AS_MESH) else {
        return;
    };
    let scene = decode(&bytes);
    let mut got: Vec<(String, usize, usize)> = scene
        .meshes
        .iter()
        .map(|m| {
            let p = &m.primitives[0];
            let verts = p.extras["fbx:shared_positions"].as_array().unwrap().len() / 3;
            let pvi = p.extras["fbx:polygon_vertex_index"]
                .as_array()
                .map_or(0, Vec::len);
            assert!(!p.extras.contains_key(NURBS_CURVE_KEY));
            (m.name.clone().unwrap_or_default(), verts, pvi)
        })
        .collect();
    got.sort();
    assert_eq!(
        got,
        [
            ("Mesh".to_string(), 13, 0),
            ("Mesh.001".to_string(), 25, 0),
            ("NurbsCurveBevelData".to_string(), 576, 2256),
            ("NurbsSurfData".to_string(), 256, 900),
        ]
    );
    for n in &scene.nodes {
        assert!(
            !n.extras.contains_key("fbx:geometry_kind"),
            "no NURBS subtype"
        );
    }
}

/// `(model, order, form, rational, points, knots)` per the fixture's
/// notes table.
const CURVES: [(&str, u32, NurbsForm, bool, usize, usize); 5] = [
    ("Object_1", 3, NurbsForm::Open, true, 8, 11),
    ("Object_2", 3, NurbsForm::Closed, true, 8, 12),
    ("Object_4", 3, NurbsForm::Open, false, 7, 10),
    ("Object_5", 3, NurbsForm::Periodic, false, 7, 12),
    ("Object_6", 4, NurbsForm::Periodic, false, 8, 15),
];

fn curve_of<'a>(scene: &'a Scene3D, model: &str) -> (FbxNurbsCurve, &'a oxideav_mesh3d::Primitive) {
    let (_, node) = node_named(scene, model);
    let mesh = &scene.meshes[node.mesh.expect("curve mesh").0 as usize];
    let prim = &mesh.primitives[0];
    (
        FbxNurbsCurve::from_primitive(prim).expect("typed curve"),
        prim,
    )
}

#[test]
fn nurbs_curves_decode_to_typed_records_and_sampled_lines() {
    let Some(bytes) = fixture(NURBS_CURVES) else {
        return;
    };
    let scene = decode(&bytes);
    let dist = |a: [f64; 3], b: [f64; 3]| {
        ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
    };
    for (model, order, form, rational, n, k) in CURVES {
        let (_, node) = node_named(&scene, model);
        assert_eq!(node.extras["fbx:model_subtype"], "NurbsCurve");
        assert_eq!(node.extras["fbx:geometry_kind"], "NurbsCurve");
        let (c, prim) = curve_of(&scene, model);
        assert_eq!(
            (
                c.order,
                c.form,
                c.rational,
                c.points.len(),
                c.knots.len(),
                c.dimension
            ),
            (order, form, rational, n, k, 3),
            "{model}"
        );
        assert_eq!(
            c.expected_knot_count(),
            k,
            "{model}: staged knot-count relation"
        );
        let curve = c.to_curve().unwrap_or_else(|e| panic!("{model}: {e:?}"));
        let (lo, hi) = curve.domain();
        let first = [c.points[0][0], c.points[0][1], c.points[0][2]];
        match form {
            NurbsForm::Open => {
                let last = c.points[n - 1];
                assert!(dist(curve.evaluate(lo), first) < 1e-9, "{model}");
                assert!(dist(curve.evaluate(hi), [last[0], last[1], last[2]]) < 1e-9);
                assert_eq!(prim.topology, Topology::LineStrip);
            }
            NurbsForm::Closed => {
                assert!(dist(curve.evaluate(lo), first) < 1e-9, "{model}");
                assert!(dist(curve.evaluate(hi), first) < 1e-9, "{model} closes");
                assert_eq!(prim.topology, Topology::LineStrip);
            }
            NurbsForm::Periodic => {
                assert!(
                    dist(curve.evaluate(lo), curve.evaluate(hi)) < 1e-9,
                    "{model} wraps"
                );
                assert_eq!(prim.topology, Topology::LineLoop);
            }
        }
        assert!(prim.positions.len() >= 16, "{model} sampled");
    }
    // The Model transforms ride the ordinary node path.
    let (_, n5) = node_named(&scene, "Object_5");
    let (_, layer) = node_named(&scene, "Layer 01");
    assert_eq!(layer.children.len(), 5);
    assert!(matches!(
        n5.transform,
        oxideav_mesh3d::Transform::Trs { translation, .. } if (translation[1] - 3.383_683_7).abs() < 1e-5
    ));
}

/// The `NurbsCurve` records re-encode value-exact through both output
/// forms (and the binary form reads back through the ASCII-free path).
#[test]
fn nurbs_curves_re_encode_value_exact() {
    let Some(bytes) = fixture(NURBS_CURVES) else {
        return;
    };
    let source: Vec<FbxNurbsCurve> = objects(&parse_any(&bytes))
        .children_named("Geometry")
        .map(|g| FbxNurbsCurve::from_element(g).expect("typed"))
        .collect();
    assert_eq!(source.len(), 5);
    let scene = decode(&bytes);
    for form in [FbxOutputForm::Binary, FbxOutputForm::Ascii] {
        let out = encode(&scene, form);
        let doc = parse_any(&out);
        let geoms: Vec<&FbxNode> = objects(&doc).children_named("Geometry").collect();
        let ours: Vec<FbxNurbsCurve> = geoms
            .iter()
            .map(|g| FbxNurbsCurve::from_element(g).expect("typed"))
            .collect();
        assert_eq!(ours, source, "{form:?}");
        for g in &geoms {
            assert_eq!(g.properties[2].as_str(), Some("NurbsCurve"));
            let names: Vec<&str> = g.children.iter().map(|c| c.name.as_str()).collect();
            assert_eq!(
                names,
                [
                    "GeometryVersion",
                    "Type",
                    "NurbsCurveVersion",
                    "Order",
                    "Dimension",
                    "Form",
                    "Rational",
                    "Points",
                    "KnotVector"
                ]
            );
        }
        // Model display names keep no `Model::` class prefix.
        let names: Vec<String> = objects(&doc).children_named("Model").map(display).collect();
        assert!(names.contains(&"Object_1".to_string()), "{names:?}");
        let again = decode(&out);
        for (model, ..) in CURVES {
            assert_eq!(curve_of(&again, model).0, curve_of(&scene, model).0);
        }
    }
}

/// An API-authored curve reaches the wire as a `NurbsCurve` geometry
/// under a `NurbsCurve` Model and reads back identically.
#[test]
fn api_authored_nurbs_curve_round_trips() {
    let curve = FbxNurbsCurve::new(
        4,
        NurbsForm::Open,
        vec![
            [0.0, 0.0, 0.0, 1.0],
            [1.0, 1.0, 0.0, 0.5],
            [2.0, -1.0, 0.0, 2.0],
            [3.0, 1.0, 0.0, 1.0],
            [4.0, 0.0, 0.0, 1.0],
        ],
        vec![0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0],
    );
    assert!(curve.rational);
    let mut scene = Scene3D::new();
    let mid = scene.add_mesh(curve.to_mesh(Some("Wire".to_string())));
    let mut node = Node::new().with_name("Wire").with_mesh(mid);
    node.extras
        .insert("fbx:model_subtype".to_string(), Value::from("NurbsCurve"));
    let nid = scene.add_node(node);
    scene.roots.push(nid);
    for form in [FbxOutputForm::Binary, FbxOutputForm::Ascii] {
        let out = encode(&scene, form);
        let doc = parse_any(&out);
        let g = objects(&doc).children_named("Geometry").next().unwrap();
        assert_eq!(
            FbxNurbsCurve::from_element(g),
            Some(curve.clone()),
            "{form:?}"
        );
        let again = decode(&out);
        assert_eq!(curve_of(&again, "Wire").0, curve);
    }
}

// ---------------------------------------------------------------------
// Black-box re-open of our re-encodes in the `blender` binary
// ---------------------------------------------------------------------

fn blender_binary() -> Option<PathBuf> {
    if let Ok(p) = std::env::var("OXIDEAV_FBX_BLENDER") {
        let p = PathBuf::from(p);
        return p.is_file().then_some(p);
    }
    for p in [
        "/opt/homebrew/bin/blender",
        "/usr/local/bin/blender",
        "/usr/bin/blender",
        "/Applications/Blender.app/Contents/MacOS/Blender",
    ] {
        let p = PathBuf::from(p);
        if p.is_file() {
            return Some(p);
        }
    }
    None
}

const REOPEN_PY: &str = r#"
import bpy, sys, json
path = sys.argv[sys.argv.index("--") + 1]
bpy.ops.wm.read_factory_settings(use_empty=True)
res = {"ok": True}
try:
    bpy.ops.import_scene.fbx(filepath=path)
except Exception as e:
    res["ok"] = False
objs = []
for o in sorted(bpy.data.objects, key=lambda o: o.name):
    d = {"name": o.name, "type": o.type, "anim": bool(o.animation_data and o.animation_data.action)}
    if o.type == "MESH":
        d["verts"] = len(o.data.vertices)
        d["polys"] = len(o.data.polygons)
        if o.data.shape_keys:
            d["keys"] = [(k.name, round(k.value, 4)) for k in o.data.shape_keys.key_blocks]
    objs.append(d)
res["objects"] = objs
tex = []
for m in sorted(bpy.data.materials, key=lambda m: m.name):
    if m.node_tree:
        for n in m.node_tree.nodes:
            if n.type == "TEX_IMAGE":
                tex.append([m.name, n.extension])
            if n.type == "MAPPING":
                tex.append([m.name, n.vector_type] + [[round(v, 4) for v in n.inputs[s].default_value] for s in ("Location", "Rotation", "Scale")])
res["tex"] = sorted(tex)
print("REOPEN " + json.dumps(res, sort_keys=True))
"#;

fn reopen(blender: &PathBuf, script: &PathBuf, file: &PathBuf) -> Value {
    let out = std::process::Command::new(blender)
        .args(["-b", "--factory-startup", "--python"])
        .arg(script)
        .arg("--")
        .arg(file)
        .output()
        .expect("run blender");
    let text = String::from_utf8_lossy(&out.stdout);
    let line = text
        .lines()
        .find_map(|l| l.strip_prefix("REOPEN "))
        .unwrap_or_else(|| panic!("no result line for {}", file.display()));
    serde_json::from_str(line).expect("json")
}

/// Each binary sweep fixture and our binary re-encode of it import
/// to the same objects, shape keys, animation presence and texture
/// node settings; the `NurbsCurve` sample (ASCII, which that importer
/// does not read) imports from our binary re-encode with all six
/// Models.
#[test]
fn blender_reopens_our_re_encodes_like_the_originals() {
    let (Some(blender), Some(dir)) = (blender_binary(), fixtures_dir()) else {
        eprintln!("skipping: no blender binary or docs corpus");
        return;
    };
    let work = std::env::temp_dir().join(format!("oxideav-fbx-reopen-{}", std::process::id()));
    std::fs::create_dir_all(&work).unwrap();
    let script = work.join("reopen.py");
    std::fs::write(&script, REOPEN_PY).unwrap();
    for name in [
        KEYATTR,
        TEXTURE,
        SHAPES,
        SHAPES_ANIM,
        NURBS_AS_MESH,
        NURBS_CURVES,
    ] {
        let src = dir.join(name);
        let bytes = std::fs::read(&src).unwrap();
        let ours = work.join(name.replace('/', "_"));
        std::fs::write(&ours, encode(&decode(&bytes), FbxOutputForm::Binary)).unwrap();
        let got = reopen(&blender, &script, &ours);
        assert_eq!(got["ok"], Value::Bool(true), "{name}: {got}");
        if name == NURBS_CURVES {
            assert_eq!(got["objects"].as_array().unwrap().len(), 6, "{got}");
        } else {
            let want = reopen(&blender, &script, &src);
            assert_eq!(got, want, "{name}");
        }
    }
    let _ = std::fs::remove_dir_all(&work);
}
