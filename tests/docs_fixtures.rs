//! Validation over the staged clean-room corpus at
//! `docs/3d/fbx/fixtures/` (round 439) — the fixtures added by the
//! 2026-08-10 docs extension (#287 / #317) plus decode-parity sweeps
//! over the whole set.
//!
//! The corpus lives in the workspace docs repository, not in this
//! crate, so every test resolves it via the
//! `OXIDEAV_FBX_DOCS_FIXTURES` env var or the umbrella-checkout
//! relative path and **skips cleanly when absent** (standalone-crate
//! CI has no docs checkout; nothing is copied into this repo).
//!
//! What the staged docs pin and these tests verify:
//!
//! - `box-binary-v7500.fbx` — the first staged **binary** v7500
//!   fixture: confirms the ≥ 7500 64-bit node-record widening
//!   directly, the v7400-unchanged footer layout
//!   (`fbx-binary-properties70.md` §2a), and the `c`-type byte
//!   array the fixture was staged to exercise.
//! - `cubes-pivots-ascii-v7500.fbx` + `cubes-ascii-v7500.fbx` — the
//!   same scene exported with and without authored pivots
//!   (`fbx-node-transform-chain.md` §1.1): the pivot block must
//!   compose to the identity on the doc's named cube pair. This
//!   validates the `Soff`/`Sp` grouping of the §1 product against
//!   real bytes.
//! - Axis integers (`fbx-node-transform-chain.md` §4a): every staged
//!   fixture is a Maya-lineage Y-up export, so each must decode to
//!   the typed `up_axis = PosY` / `front_axis = PosZ`.
//! - Every fixture: typed-tree re-encode closure
//!   (`parse(write(parse(x))) == parse(x)`) through its own
//!   front-end, and a full `Scene3D` decode that produces content.

use std::path::PathBuf;

use oxideav_fbx::binary::{FbxDocument, FbxNode};
use oxideav_fbx::{parse_footer, write_document_with_options, FbxDecoder, WriterOptions};
use oxideav_mesh3d::{Axis, Mesh3DDecoder, Scene3D, Transform};

/// Locate `docs/3d/fbx/fixtures/`. Env override first, then the
/// umbrella-checkout layout (`crates/oxideav-fbx/../../docs/…`).
fn fixtures_dir() -> Option<PathBuf> {
    if let Ok(dir) = std::env::var("OXIDEAV_FBX_DOCS_FIXTURES") {
        let p = PathBuf::from(dir);
        return p.is_dir().then_some(p);
    }
    let p = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../docs/3d/fbx/fixtures");
    p.is_dir().then_some(p)
}

/// Read one fixture, or `None` (with a skip note) when the corpus
/// isn't on this machine.
fn fixture(name: &str) -> Option<Vec<u8>> {
    let dir = fixtures_dir()?;
    match std::fs::read(dir.join(name)) {
        Ok(bytes) => Some(bytes),
        Err(_) => {
            eprintln!("skipping: {name} not present under {}", dir.display());
            None
        }
    }
}

/// Structural equality over the typed node tree.
fn nodes_equal(a: &FbxNode, b: &FbxNode) -> bool {
    a.name == b.name
        && a.properties == b.properties
        && a.children.len() == b.children.len()
        && a.children
            .iter()
            .zip(b.children.iter())
            .all(|(x, y)| nodes_equal(x, y))
}

fn parse_any(bytes: &[u8]) -> FbxDocument {
    if oxideav_fbx::is_ascii_fbx(bytes) {
        oxideav_fbx::ascii::parse(bytes).expect("ascii parse")
    } else {
        oxideav_fbx::binary::parse(bytes).expect("binary parse")
    }
}

fn decode(bytes: &[u8]) -> Scene3D {
    FbxDecoder::new().decode(bytes).expect("scene decode")
}

const ALL_FIXTURES: &[&str] = &[
    "box-binary-v7400.fbx",
    "box-binary-v7500.fbx",
    "camera-attr-binary-v7400.fbx",
    "skin-anim-binary-v7400.fbx",
    "cubes-ascii-v7500.fbx",
    "cubes-pivots-ascii-v7500.fbx",
    "texture-video-ascii-v7500.fbx",
];

/// `box-binary-v7500.fbx`: the 64-bit node-record layout parses; the
/// footer decodes under the v7400-unchanged layout to the exact id
/// the docs record (`fbx-binary-properties70.md` §2a footer table);
/// the `c`-type byte array (the very payload the fixture was staged
/// to exercise) decodes with its pinned 1-byte element width; and
/// the typed tree survives re-encode through the 64-bit writer, both
/// uncompressed and re-deflated.
///
/// Byte-for-byte output equality is deliberately *not* asserted for
/// this fixture (unlike `box-binary-v7400.fbx`): its arrays are
/// zlib-compressed (`Encoding = 1`), and a re-deflate is only
/// guaranteed to be stream-equivalent, not byte-identical to the
/// producer's compressor output.
#[test]
fn box_binary_v7500_parses_and_closes_the_round_trip() {
    let Some(bytes) = fixture("box-binary-v7500.fbx") else {
        return;
    };
    let doc = oxideav_fbx::binary::parse(&bytes).expect("v7500 parses");
    assert_eq!(doc.version, 7500);

    // Footer layout unchanged from v7400; the id matches the docs'
    // recorded value for this fixture.
    let footer = parse_footer(&bytes).expect("footer decodes");
    assert_eq!(footer.id_hex(), "fabcaa01d9cbd66abc71fa8913fa287b");

    // The staged `c`-array: the header thumbnail's `ImageData`
    // (`FBXHeaderExtension/SceneInfo/Thumbnail/ImageData`), 12288
    // one-byte elements (ArrayLength == decompressed byte count pins
    // the element width from these very bytes).
    fn find_named<'a>(
        n: &'a oxideav_fbx::binary::FbxNode,
        name: &str,
    ) -> Option<&'a oxideav_fbx::binary::FbxNode> {
        if n.name == name {
            return Some(n);
        }
        n.children.iter().find_map(|c| find_named(c, name))
    }
    let image_data = find_named(&doc.root, "ImageData").expect("ImageData record");
    match image_data.properties.first() {
        Some(oxideav_fbx::binary::FbxProperty::ByteArray(b)) => {
            assert_eq!(b.len(), 12288);
            assert!(b.contains(&0xff), "pixel payload bytes");
        }
        other => panic!("expected ByteArray, got {other:?}"),
    }

    // Typed-tree closure through the 64-bit writer — uncompressed
    // and re-deflated forms both re-parse to the identical tree.
    let footer_opts = WriterOptions::default().footer_id(footer.id);
    for opts in [
        footer_opts.clone(),
        footer_opts.clone().compress_arrays_at(256),
    ] {
        let out = write_document_with_options(&doc, &opts).expect("re-encode");
        let re = oxideav_fbx::binary::parse(&out).expect("re-parse");
        assert_eq!(re.version, 7500);
        assert!(
            nodes_equal(&doc.root, &re.root),
            "typed tree diverged through re-encode"
        );
        let refooter = parse_footer(&out).expect("footer survives");
        assert_eq!(refooter.id, footer.id);
    }
}

/// The §1.1 fixture pair (`fbx-node-transform-chain.md`): the
/// exporter authored `RotationPivot` / `ScalingPivot` /
/// `ScalingOffset` on the pivots export **without moving the
/// geometry**, so the pivot block must compose to the identity —
/// the composed translation equals the authored `Lcl Translation`
/// (`Soff + (I − S)·Sp = 0` on all three axes), which in turn equals
/// the pivot-free export's counterpart cube to float precision.
///
/// The doc names the pair explicitly: plain `Model::Куб1` with
/// `Lcl Translation = (1.04023893373156, −0.998288783259251,
/// 1.1806740271636)` and uniform `Lcl Scaling = 0.77384837213491`,
/// vs the pivots export's `Model::Cube1` carrying the same
/// translation (Δ ≈ 4.9 × 10⁻⁸) plus the three authored pivot
/// records. (The two exports differ in `UnitScaleFactor` — 1 vs
/// 100 — and in per-cube mirroring, so a whole-scene transform
/// comparison is *not* what the doc pins; the named pair is.)
#[test]
fn pivot_pair_composes_to_matching_transforms() {
    let (Some(plain), Some(pivoted)) = (
        fixture("cubes-ascii-v7500.fbx"),
        fixture("cubes-pivots-ascii-v7500.fbx"),
    ) else {
        return;
    };
    let scene_plain = decode(&plain);
    let scene_piv = decode(&pivoted);
    assert_eq!(scene_plain.nodes.len(), scene_piv.nodes.len());

    // The doc's pivoted cube: the node carrying the authored
    // ScalingPivot extras (the fixture has two nodes named "Cube1";
    // the chain extras disambiguate).
    let piv_node = scene_piv
        .nodes
        .iter()
        .find(|n| n.extras.contains_key("fbx:scaling_pivot"))
        .expect("pivots export surfaces the authored ScalingPivot");
    let ex_vec3 = |key: &str| -> [f64; 3] {
        let a = piv_node.extras[key].as_array().expect(key);
        [
            a[0].as_f64().unwrap(),
            a[1].as_f64().unwrap(),
            a[2].as_f64().unwrap(),
        ]
    };
    // The authored records, verbatim per the doc §1.1 listing.
    let sp = ex_vec3("fbx:scaling_pivot");
    let soff = ex_vec3("fbx:scaling_offset");
    let rp = ex_vec3("fbx:rotation_pivot");
    assert!((sp[0] - -0.747048924260949).abs() < 1e-12);
    assert!((soff[0] - 0.168946330316482).abs() < 1e-12);
    assert!((rp[0] - -0.578102593944473).abs() < 1e-12);
    let lcl_t = ex_vec3("fbx:lcl_translation");
    let lcl_s = ex_vec3("fbx:lcl_scaling");

    // §1.1 core: `Soff + (I − S)·Sp = 0` per axis, at
    // double-rounding level.
    for i in 0..3 {
        let residual = soff[i] + (1.0 - lcl_s[i]) * sp[i];
        assert!(
            residual.abs() < 1e-12,
            "axis {i}: Soff + (I−S)·Sp = {residual}"
        );
    }

    // Therefore the composed transform equals the authored Lcl
    // triple (pivot block collapses to identity)...
    let Transform::Trs {
        translation, scale, ..
    } = piv_node.transform
    else {
        panic!("expected composed Trs");
    };
    for i in 0..3 {
        assert!(
            (f64::from(translation[i]) - lcl_t[i]).abs() < 1e-5,
            "composed t[{i}] {} != authored {}",
            translation[i],
            lcl_t[i]
        );
        assert!((f64::from(scale[i]) - lcl_s[i]).abs() < 1e-6);
    }

    // ...and matches the pivot-free export's counterpart cube (the
    // doc's `Model::Куб1` with the same 0.7738… uniform scale) to
    // float precision.
    let plain_node = scene_plain
        .nodes
        .iter()
        .filter(|n| {
            // ASCII object names keep the `Class::Name` display
            // form, so the node is `Model::Куб1`.
            n.name.as_deref().is_some_and(|s| s.ends_with("Куб1"))
        })
        .find(|n| match n.transform {
            Transform::Trs { scale, .. } => (scale[0] - 0.773_848_4).abs() < 1e-5,
            Transform::Matrix(_) => false,
        })
        .expect("plain export's scaled Куб1");
    let Transform::Trs {
        translation: t_plain,
        scale: s_plain,
        ..
    } = plain_node.transform
    else {
        unreachable!()
    };
    for i in 0..3 {
        assert!(
            (t_plain[i] - translation[i]).abs() < 1e-5,
            "pair translation mismatch on axis {i}: {t_plain:?} vs {translation:?}"
        );
        assert!((s_plain[i] - scale[i]).abs() < 1e-6);
    }
}

/// Every staged fixture is a Maya-lineage Y-up / Z-front export
/// (`fbx-node-transform-chain.md` §4a evidence table), so each must
/// decode the pinned axis integers to the typed scene fields.
#[test]
fn all_fixtures_decode_the_pinned_axis_convention() {
    let Some(_) = fixtures_dir() else { return };
    for name in ALL_FIXTURES {
        let Some(bytes) = fixture(name) else { continue };
        let scene = decode(&bytes);
        assert_eq!(scene.up_axis, Axis::PosY, "{name}: up axis");
        assert_eq!(scene.front_axis, Axis::PosZ, "{name}: front axis");
        // §4a structural fact on real bytes: the triple is mutually
        // distinct and exhausts {0,1,2} — CoordAxis is the remaining
        // (right) axis, X = 0 on every Maya-lineage fixture — so the
        // coherence guard must stay silent.
        assert_eq!(
            scene.extras.get("fbx:coord_axis").and_then(|v| v.as_i64()),
            Some(0),
            "{name}: coord axis is the remaining index"
        );
        assert!(
            !scene
                .extras
                .contains_key("fbx:axis_convention_inconsistent"),
            "{name}: coherent triple raises no marker"
        );
    }
}

/// Typed-tree re-encode closure over the whole corpus: parsing the
/// re-encoded document reproduces the parsed original exactly, for
/// each fixture through its own front-end (binary fixtures through
/// the binary writer, ASCII through the ASCII writer).
#[test]
fn all_fixtures_close_the_typed_tree_round_trip() {
    let Some(_) = fixtures_dir() else { return };
    for name in ALL_FIXTURES {
        let Some(bytes) = fixture(name) else { continue };
        let doc = parse_any(&bytes);
        let re_bytes = if oxideav_fbx::is_ascii_fbx(&bytes) {
            oxideav_fbx::write_ascii_document(&doc).expect("ascii write")
        } else {
            oxideav_fbx::write_document(&doc).expect("binary write")
        };
        let re_doc = parse_any(&re_bytes);
        assert_eq!(doc.version, re_doc.version, "{name}: version");
        assert!(
            nodes_equal(&doc.root, &re_doc.root),
            "{name}: typed tree diverged through re-encode"
        );
    }
}

/// Full-decoder smoke over the corpus: every fixture produces a
/// populated scene.
#[test]
fn all_fixtures_decode_to_populated_scenes() {
    let Some(_) = fixtures_dir() else { return };
    for name in ALL_FIXTURES {
        let Some(bytes) = fixture(name) else { continue };
        let scene = decode(&bytes);
        assert!(
            !scene.nodes.is_empty() || !scene.meshes.is_empty() || !scene.extras.is_empty(),
            "{name}: empty scene"
        );
    }
}

/// The skin-anim fixture's 90 `AnimationCurve`s each carry the three
/// key-attribute sub-records: the raw catalogue surfaces all of them
/// with resolved join keys and non-empty verbatim arrays. (No bit is
/// interpreted — the value assignment is the GAP-TRACKER's open
/// item.)
#[test]
fn skin_anim_key_attr_catalogue_surfaces_from_real_bytes() {
    let Some(bytes) = fixture("skin-anim-binary-v7400.fbx") else {
        return;
    };
    let scene = decode(&bytes);
    let catalogue = scene
        .extras
        .get("fbx:key_attrs")
        .and_then(|v| v.as_array())
        .expect("fbx:key_attrs present");
    assert_eq!(catalogue.len(), 90, "one entry per attributed curve");
    for entry in catalogue {
        let e = entry.as_object().expect("object entry");
        assert!(e.contains_key("stack"), "stack join key resolves");
        assert!(e.contains_key("property"));
        assert!(e.contains_key("axis"));
        assert!(e["key_count"].as_u64().unwrap_or(0) > 0);
        for k in ["flags", "data_bits", "ref_count"] {
            assert!(
                !e[k].as_array().expect(k).is_empty(),
                "{k} array is non-empty"
            );
        }
    }
    // And the animation itself still decodes.
    assert!(!scene.animations.is_empty());
}

/// `texture-video-ascii-v7500.fbx`: the `Texture` element's authored
/// `UVSet = "UVChannel_1"` KString joins against the geometry's
/// `LayerElementUV` `Name` leaves (`"UVChannel_1"` at channel 0,
/// `"UVChannel_3"` at channel 1) onto the typed
/// `TextureRef::uv_set = 0`, the channel labels surface on
/// `Primitive::extras["fbx:uv_set_names"]`, and — the texture
/// authoring no placement records — the typed transform stays `None`.
#[test]
fn texture_video_fixture_uvset_joins_to_channel_zero() {
    let Some(bytes) = fixture("texture-video-ascii-v7500.fbx") else {
        return;
    };
    let scene = decode(&bytes);

    // Channel labels in document order.
    let prim = scene
        .meshes
        .iter()
        .flat_map(|m| m.primitives.iter())
        .find(|p| p.uvs.len() == 2)
        .expect("the box mesh carries two UV channels");
    let names: Vec<&str> = prim.extras["fbx:uv_set_names"]
        .as_array()
        .expect("fbx:uv_set_names")
        .iter()
        .map(|v| v.as_str().unwrap())
        .collect();
    assert_eq!(names, ["UVChannel_1", "UVChannel_3"]);

    // The DiffuseColor-bound texture samples channel 0 by label.
    let texref = scene
        .materials
        .iter()
        .find_map(|m| m.base_color_texture)
        .expect("DiffuseColor binding");
    assert_eq!(texref.uv_set, 0, "UVSet = UVChannel_1 -> channel 0");
    assert_eq!(
        texref.transform, None,
        "no placement records authored on the fixture texture"
    );

    // The fixture texture's remaining authored §3.1 records surface
    // raw for lossless re-encode: `CurrentTextureBlendMode = 0`
    // (differing from the template default `1`) and
    // `UseMaterial = 1`.
    let rec = scene.extras["fbx:texture_records"]
        .get(texref.texture.0.to_string())
        .expect("raw records for the bound texture");
    assert_eq!(rec["current_texture_blend_mode"].as_i64(), Some(0));
    assert_eq!(rec["use_material"].as_bool(), Some(true));
}

/// The fixture's `UVSet` join + channel labels survive a full
/// `decode → encode → decode` cycle in both output forms: the
/// encoder re-emits the authored `LayerElementUV` `Name` leaves from
/// `fbx:uv_set_names` and a matching `UVSet` KString on the
/// `Texture` element, and the second decode re-joins them.
#[test]
fn texture_video_fixture_uvset_survives_re_encode() {
    use oxideav_fbx::{FbxEncoder, FbxOutputForm};
    use oxideav_mesh3d::Mesh3DEncoder;

    let Some(bytes) = fixture("texture-video-ascii-v7500.fbx") else {
        return;
    };
    let scene = decode(&bytes);

    for form in [FbxOutputForm::Binary, FbxOutputForm::Ascii] {
        let out = FbxEncoder::new()
            .form(form)
            .encode(&scene)
            .expect("re-encode");
        let scene2 = decode(&out);
        let prim = scene2
            .meshes
            .iter()
            .flat_map(|m| m.primitives.iter())
            .find(|p| p.uvs.len() == 2)
            .expect("both UV channels survive re-encode");
        let names: Vec<&str> = prim.extras["fbx:uv_set_names"]
            .as_array()
            .expect("labels re-emitted")
            .iter()
            .map(|v| v.as_str().unwrap())
            .collect();
        assert_eq!(names, ["UVChannel_1", "UVChannel_3"]);
        let texref = scene2
            .materials
            .iter()
            .find_map(|m| m.base_color_texture)
            .expect("binding survives");
        assert_eq!(texref.uv_set, 0, "UVSet label re-joined after re-encode");
    }
}

/// `skin-anim-binary-v7400.fbx` wires every bone `Model` as the
/// *child* of its `Cluster` (`C: "OO", <model>, <cluster>`), the
/// mirror image of the Cluster-as-child form. The decoder accepts
/// both, so the fixture's one `Skin` + nine `Cluster` deformers
/// materialise a 9-joint skeleton named after the Skin element with
/// per-corner joint / weight buffers on the mesh, and the whole skin
/// survives `decode → encode → decode` in both output forms
/// (skeleton name, joint order, inverse-bind matrices, top-4
/// weights).
#[test]
fn skin_anim_fixture_decodes_and_round_trips_its_skin() {
    use oxideav_fbx::{FbxEncoder, FbxOutputForm};
    use oxideav_mesh3d::Mesh3DEncoder;

    let Some(bytes) = fixture("skin-anim-binary-v7400.fbx") else {
        return;
    };
    let scene = decode(&bytes);
    assert_eq!(scene.skeletons.len(), 1, "one Skin deformer");
    assert_eq!(scene.skins.len(), 1);
    let skel = &scene.skeletons[0];
    assert_eq!(skel.name.as_deref(), Some("Armature"));
    assert_eq!(skel.joints.len(), 9, "nine Cluster deformers");
    assert_eq!(skel.inverse_bind_matrices.len(), 9);
    let joint_names: Vec<&str> = skel
        .joints
        .iter()
        .map(|j| scene.nodes[j.0 as usize].name.as_deref().unwrap())
        .collect();
    assert_eq!(joint_names[0], "Bone");
    assert_eq!(joint_names[8], "Bone.008");
    let skinned = scene
        .nodes
        .iter()
        .find(|n| n.skin.is_some())
        .expect("the mesh node carries the skin");
    assert_eq!(skinned.name.as_deref(), Some("Cylinder"));
    let prim = &scene.meshes[skinned.mesh.unwrap().0 as usize].primitives[0];
    let joints = prim.joints.as_ref().expect("per-corner joints");
    let weights = prim.weights.as_ref().expect("per-corner weights");
    assert_eq!(joints.len(), prim.positions.len());
    assert!(
        weights.iter().any(|w| w[0] > 0.0),
        "the clusters carry non-zero weights"
    );

    for form in [FbxOutputForm::Binary, FbxOutputForm::Ascii] {
        let out = FbxEncoder::new()
            .form(form)
            .encode(&scene)
            .expect("re-encode");
        let scene2 = decode(&out);
        assert_eq!(scene2.skeletons.len(), 1);
        let skel2 = &scene2.skeletons[0];
        assert_eq!(skel2.name.as_deref(), Some("Armature"));
        assert_eq!(skel2.joints, skel.joints, "joint order survives");
        for (a, b) in skel2
            .inverse_bind_matrices
            .iter()
            .zip(&skel.inverse_bind_matrices)
        {
            for (ra, rb) in a.iter().zip(b) {
                for (x, y) in ra.iter().zip(rb) {
                    assert!((x - y).abs() < 1e-5, "inverse-bind survives");
                }
            }
        }
        let prim2 = &scene2.meshes[0].primitives[0];
        assert_eq!(prim2.joints.as_ref(), Some(joints));
        for (a, b) in prim2.weights.as_ref().unwrap().iter().zip(weights) {
            for (x, y) in a.iter().zip(b) {
                assert!((x - y).abs() < 1e-6);
            }
        }
    }
}

/// `fbx-property-templates.md` §5: template bodies are producer
/// renditions, so a round trip must re-emit the file's own bodies
/// rather than this crate's built-ins. Pinned on the two producers
/// in the corpus: the Blender-written 24-record `FbxSurfacePhong`
/// and the `FbxCamera` body that `camera-attr-binary-v7400.fbx`
/// carries despite its light + camera attribute mixture (the
/// built-in rule would emit none), and the SDK-written 22-record
/// `FbxSurfacePhong` of `texture-video-ascii-v7500.fbx`.
#[test]
fn definitions_templates_re_emit_the_files_own_bodies() {
    use oxideav_fbx::definitions::Definitions;
    use oxideav_fbx::{FbxEncoder, FbxOutputForm};
    use oxideav_mesh3d::Mesh3DEncoder;

    for (name, class, template, n_records) in [
        (
            "camera-attr-binary-v7400.fbx",
            "Material",
            "FbxSurfacePhong",
            24,
        ),
        (
            "camera-attr-binary-v7400.fbx",
            "NodeAttribute",
            "FbxCamera",
            106,
        ),
        (
            "texture-video-ascii-v7500.fbx",
            "Material",
            "FbxSurfacePhong",
            22,
        ),
        (
            "texture-video-ascii-v7500.fbx",
            "Texture",
            "FbxFileTexture",
            16,
        ),
    ] {
        let Some(bytes) = fixture(name) else {
            return;
        };
        let scene = decode(&bytes);
        let src = Definitions::from_document(&parse_any(&bytes));
        let src_tpl = src.template_for(class).expect("source template");
        assert_eq!(src_tpl.len(), n_records, "{name}/{class} source body");
        for form in [FbxOutputForm::Binary, FbxOutputForm::Ascii] {
            let out = FbxEncoder::new()
                .form(form)
                .encode(&scene)
                .expect("re-encode");
            let defs = Definitions::from_document(&parse_any(&out));
            let def = defs.get(class).expect("class re-emitted");
            assert_eq!(
                def.template_name.as_deref(),
                Some(template),
                "{name}/{class}"
            );
            let tpl = def.template.as_ref().unwrap();
            assert_eq!(tpl.len(), n_records, "{name}/{class} body survives");
            assert_eq!(
                template_record_names(&parse_any(&bytes), class),
                template_record_names(&parse_any(&out), class),
                "{name}/{class} record order"
            );
        }
    }
}

/// The `P` record names of one class's first `PropertyTemplate`, in
/// wire order (a `PropertyMap` is unordered).
fn template_record_names(doc: &FbxDocument, class: &str) -> Vec<String> {
    doc.root
        .child("Definitions")
        .into_iter()
        .flat_map(|d| d.children_named("ObjectType"))
        .find(|ot| {
            ot.properties
                .first()
                .and_then(oxideav_fbx::binary::FbxProperty::as_str)
                == Some(class)
        })
        .and_then(|ot| ot.child("PropertyTemplate"))
        .and_then(|t| t.child("Properties70"))
        .map(|p| {
            p.children
                .iter()
                .filter_map(|c| {
                    c.properties
                        .first()
                        .and_then(oxideav_fbx::binary::FbxProperty::as_str)
                        .map(str::to_owned)
                })
                .collect()
        })
        .unwrap_or_default()
}

/// Node-level verbatim passthrough on `camera-attr-binary-v7400.fbx`:
/// the camera `NodeAttribute`'s records outside the typed camera
/// mapping (`FocalLength`, `FilmWidth`, `GateFit`) and its scalar
/// body leaves (`Position` / `Up` / `LookAt` / `TypeFlags`), and the
/// `Model`'s untyped records (`DefaultAttributeIndex`) and leaves
/// (`MultiLayer` / `MultiTake`), all reappear on the re-encoded
/// elements — the typed camera itself unchanged.
#[test]
fn camera_attr_fixture_keeps_untyped_attribute_and_model_records() {
    use oxideav_fbx::binary::FbxProperty;
    use oxideav_fbx::{FbxEncoder, FbxOutputForm};
    use oxideav_mesh3d::Mesh3DEncoder;

    let Some(bytes) = fixture("camera-attr-binary-v7400.fbx") else {
        return;
    };
    let scene = decode(&bytes);
    let cam_node = scene
        .nodes
        .iter()
        .find(|n| n.camera.is_some())
        .expect("a camera node");
    let names = |v: &serde_json::Value| -> Vec<String> {
        v.as_array()
            .unwrap()
            .iter()
            .map(|r| r["name"].as_str().unwrap().to_owned())
            .collect()
    };
    let rec = names(&cam_node.extras["fbx:node_attribute_records"]);
    assert!(rec.iter().any(|n| n == "FocalLength"), "{rec:?}");
    let leaves = names(&cam_node.extras["fbx:node_attribute_leaves"]);
    assert!(leaves.iter().any(|n| n == "Position"), "{leaves:?}");
    assert!(leaves.iter().any(|n| n == "TypeFlags"), "{leaves:?}");
    let model_rec = names(&cam_node.extras["fbx:model_records"]);
    assert!(
        model_rec.iter().any(|n| n == "DefaultAttributeIndex"),
        "{model_rec:?}"
    );

    fn p_names(element: &FbxNode) -> Vec<String> {
        element
            .child("Properties70")
            .map(|p| {
                p.children
                    .iter()
                    .filter_map(|c| c.properties.first().and_then(FbxProperty::as_str))
                    .map(str::to_owned)
                    .collect()
            })
            .unwrap_or_default()
    }
    for form in [FbxOutputForm::Binary, FbxOutputForm::Ascii] {
        let out = FbxEncoder::new()
            .form(form)
            .encode(&scene)
            .expect("re-encode");
        let doc = parse_any(&out);
        let objects = doc.root.child("Objects").unwrap();
        let cam_attr = objects
            .children
            .iter()
            .find(|o| {
                o.name == "NodeAttribute"
                    && o.properties.get(2).and_then(FbxProperty::as_str) == Some("Camera")
            })
            .expect("camera attribute re-emitted");
        let ps = p_names(cam_attr);
        for want in ["FocalLength", "FilmWidth", "GateFit", "FieldOfView"] {
            assert!(ps.iter().any(|n| n == want), "{form:?}: {want} in {ps:?}");
        }
        for leaf in ["Position", "Up", "LookAt", "TypeFlags", "GeometryVersion"] {
            assert!(cam_attr.child(leaf).is_some(), "{form:?}: {leaf} leaf");
        }
        let model = objects
            .children
            .iter()
            .find(|o| {
                o.name == "Model"
                    && o.properties.get(1).and_then(FbxProperty::as_str)
                        == cam_attr
                            .properties
                            .get(1)
                            .and_then(FbxProperty::as_str)
                            .map(|_| "Camera\u{0}\u{1}Model")
            })
            .or_else(|| objects.children.iter().find(|o| o.name == "Model"))
            .expect("a Model");
        assert!(model.child("Version").is_some());
        assert!(
            model.child("MultiTake").is_some(),
            "{form:?}: MultiTake leaf"
        );
        assert!(p_names(model).iter().any(|n| n == "DefaultAttributeIndex"));

        // The typed camera is unchanged by the passthrough.
        let scene2 = decode(&out);
        let c1 = scene.cameras[cam_node.camera.unwrap().0 as usize];
        let c2 = scene2.cameras[cam_node.camera.unwrap().0 as usize];
        assert_eq!(format!("{c1:?}"), format!("{c2:?}"));
    }
}
