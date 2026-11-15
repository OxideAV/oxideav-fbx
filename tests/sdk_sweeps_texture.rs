//! The vendor-exporter texture-placement sweep
//! (`docs/3d/fbx/fbx-sdk-export-sweeps-2026-11.md` §2,
//! `fixtures/fbxsdk-texture-placement/notes.md`): 46 file textures on
//! one material, each with one setting changed.

mod common;

use common::{display, flavours, loose, parse_any, read, render};
use oxideav_fbx::binary::{FbxDocument, FbxProperty};
use oxideav_fbx::{FbxDecoder, FbxEncoder, FbxOutputForm};
use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DEncoder, Scene3D, Texture, WrapMode};

const DIR: &str = "fbxsdk-texture-placement";

fn decode(bytes: &[u8]) -> Scene3D {
    FbxDecoder::new().decode(bytes).expect("decode")
}

fn encode(scene: &Scene3D, form: FbxOutputForm) -> Vec<u8> {
    FbxEncoder::new().form(form).encode(scene).expect("encode")
}

fn texture<'a>(scene: &'a Scene3D, name: &str) -> &'a Texture {
    scene
        .textures
        .iter()
        .find(|t| t.name.as_deref() == Some(name))
        .unwrap_or_else(|| panic!("texture {name}"))
}

/// `(name, body leaves)` per texture and `(texture, kind, parent,
/// property)` per connection, file order.
type Census = (
    Vec<(String, Vec<String>)>,
    Vec<(String, String, String, String)>,
);

fn census(doc: &FbxDocument) -> Census {
    let objects = doc.root.child("Objects").expect("Objects");
    let mut names = std::collections::HashMap::new();
    for o in &objects.children {
        if let Some(id) = o.properties.first().and_then(FbxProperty::as_i64) {
            names.insert(id, format!("{}:{}", o.name, display(o)));
        }
    }
    let textures = objects
        .children_named("Texture")
        .map(|t| {
            (
                display(t),
                t.children.iter().map(|c| render(loose, c)).collect(),
            )
        })
        .collect();
    let conns = doc
        .root
        .child("Connections")
        .expect("Connections")
        .children_named("C")
        .filter_map(|c| {
            let child = names.get(&c.properties.get(1)?.as_i64()?)?;
            (child.starts_with("Texture:") || child.starts_with("Video:")).then(|| {
                (
                    child.clone(),
                    c.properties[0].as_str().unwrap_or("").to_owned(),
                    names
                        .get(&c.properties[2].as_i64().unwrap_or(0))
                        .cloned()
                        .unwrap_or_default(),
                    c.properties
                        .get(3)
                        .and_then(FbxProperty::as_str)
                        .unwrap_or("")
                        .to_owned(),
                )
            })
        })
        .collect();
    (textures, conns)
}

/// §2.1: `WrapModeU` / `WrapModeV` — 0 repeat (the default, not
/// written), 1 clamp; the two axes are independent.
#[test]
fn wrap_modes_type_onto_the_sampler() {
    for name in flavours("texture-placement-sweep") {
        let scene = decode(&read(DIR, &name));
        assert_eq!(scene.textures.len(), 46, "{name}");
        assert_eq!(scene.materials.len(), 1, "{name}");
        let wraps = |t: &str| {
            let s = texture(&scene, t).sampler;
            (s.wrap_s, s.wrap_t)
        };
        use WrapMode::{ClampToEdge as Clamp, Repeat};
        assert_eq!(wraps("T00_default"), (Repeat, Repeat), "{name}");
        assert_eq!(
            wraps("T01_wrap_u_clamp_v_repeat"),
            (Clamp, Repeat),
            "{name}"
        );
        assert_eq!(
            wraps("T02_wrap_u_repeat_v_clamp"),
            (Repeat, Clamp),
            "{name}"
        );
        assert_eq!(wraps("T03_wrap_u_clamp_v_clamp"), (Clamp, Clamp), "{name}");
        assert_eq!(
            wraps("T13_all_trs_pivots_swap_clamp"),
            (Clamp, Clamp),
            "{name}"
        );
        // Forced integers 2 and 3 are outside the vocabulary: the typed
        // sampler stays at the default, the raw values ride along.
        assert_eq!(
            wraps("T04_wrap_property_forced_int_2_and_3"),
            (Repeat, Repeat)
        );
    }
}

/// Scene → file → document: all 46 textures come back with every
/// property record and child record (`Texture_Alpha_Source`,
/// `Cropping`, `ModelUVTranslation`, `ModelUVScaling`, …) and their
/// connections, in both output forms.
#[test]
fn scene_re_encode_keeps_every_texture_record() {
    for name in flavours("texture-placement-sweep") {
        let bytes = read(DIR, &name);
        let (src_textures, src_conns) = census(&parse_any(&bytes));
        assert_eq!(src_textures.len(), 46);
        let scene = decode(&bytes);
        for form in [FbxOutputForm::Binary, FbxOutputForm::Ascii] {
            let out = encode(&scene, form);
            let (textures, conns) = census(&parse_any(&out));
            assert_eq!(textures.len(), 46, "{name} {form:?}");
            for (a, b) in src_textures.iter().zip(&textures) {
                assert_eq!(a, b, "{name} {form:?}");
            }
            // The 46 `Texture → Material` connections in file order
            // (all on `DiffuseColor`), and the one `Video → Texture`.
            let of = |c: &[(String, String, String, String)], kind: &str| -> Vec<_> {
                c.iter().filter(|x| x.1 == kind).cloned().collect()
            };
            assert_eq!(of(&conns, "OP"), of(&src_conns, "OP"), "{name} {form:?}");
            assert_eq!(of(&conns, "OP").len(), 46);
            assert_eq!(of(&conns, "OO"), of(&src_conns, "OO"), "{name} {form:?}");
            assert_eq!(
                of(&conns, "OO"),
                vec![(
                    "Video:T00_default".to_owned(),
                    "OO".to_owned(),
                    "Texture:T00_default".to_owned(),
                    String::new()
                )]
            );
        }
    }
}

/// §2.2 / §2.3: the placement and enumeration properties as stored,
/// read through the scene's per-texture records.
#[test]
fn placement_records_are_stored_as_authored() {
    use oxideav_fbx::properties70::{PropertyMap, json_to_p_record};
    for name in flavours("texture-placement-sweep") {
        let scene = decode(&read(DIR, &name));
        let props = |t: &str| -> PropertyMap {
            let idx = scene
                .textures
                .iter()
                .position(|x| x.name.as_deref() == Some(t))
                .unwrap_or_else(|| panic!("texture {t}"));
            let records = scene.extras["fbx:texture_records"][idx.to_string()]["records"]
                .as_array()
                .cloned()
                .unwrap_or_default();
            PropertyMap::from_properties70(&oxideav_fbx::FbxNode {
                name: "Properties70".to_owned(),
                properties: Vec::new(),
                children: records.iter().filter_map(json_to_p_record).collect(),
            })
        };
        // No record differs from the template on the default texture.
        assert!(props("T00_default").is_empty(), "{name}");
        assert_eq!(props("T05_swap_uv").as_i64("UVSwap"), Some(1));
        assert_eq!(
            props("T06_translation_u0p25_v0p5").as_vec3("Translation"),
            Some([0.25, 0.5, 0.0])
        );
        // The in-plane turn is the third component, stored as authored.
        assert_eq!(
            props("T07_rotation_u10_v20_w30").as_vec3("Rotation"),
            Some([10.0, 20.0, 30.0])
        );
        assert_eq!(
            props("T15_rotation_w30_only").as_vec3("Rotation"),
            Some([0.0, 0.0, 30.0])
        );
        assert_eq!(
            props("T08_scale_u2_v3").as_vec3("Scaling"),
            Some([2.0, 3.0, 1.0])
        );
        assert_eq!(
            props("T10_scaling_property_xyz").as_vec3("Scaling"),
            Some([2.0, 3.0, 4.0])
        );
        assert_eq!(
            props("T11_rotation_pivot").as_vec3("TextureRotationPivot"),
            Some([0.5, 0.5, 0.0])
        );
        assert_eq!(
            props("T12_scaling_pivot").as_vec3("TextureScalingPivot"),
            Some([0.25, 0.75, 0.0])
        );
        let all = props("T13_all_trs_pivots_swap_clamp");
        assert_eq!(all.len(), 8, "{name}");
        assert_eq!(all.as_i64("WrapModeU"), Some(1));
        assert_eq!(all.as_i64("WrapModeV"), Some(1));
        let forced = props("T04_wrap_property_forced_int_2_and_3");
        assert_eq!(
            (forced.as_i64("WrapModeU"), forced.as_i64("WrapModeV")),
            (Some(2), Some(3))
        );
        assert_eq!(props("T16_uvset_named_map2").as_str("UVSet"), Some("map2"));
        let alpha = props("T17_alpha_0p5_premultiply_off");
        assert_eq!(alpha.as_f64("Texture alpha"), Some(0.5));
        assert_eq!(alpha.as_i64("PremultiplyAlpha"), Some(0));
        assert_eq!(
            props("T20_material_use_model_material").as_i64("UseMaterial"),
            Some(1)
        );
        // Mapping type and planar normal are one stored integer.
        for (t, v) in [
            ("M0_mapping_null", 10),
            ("M1_mapping_planar", 2),
            ("M2_mapping_spherical", 4),
            ("M3_mapping_cylindrical", 5),
            ("M4_mapping_box", 8),
            ("M5_mapping_face", 9),
            ("M7_mapping_environment", 6),
            ("P0_planar_normal_x", 2),
            ("P1_planar_normal_y", 3),
            ("P2_planar_normal_z", 1),
        ] {
            assert_eq!(props(t).as_i64("CurrentMappingType"), Some(v), "{name} {t}");
        }
        assert!(props("M6_mapping_uv").is_empty());
        for (t, v) in [
            ("B0_blend_translucent", 0),
            ("B2_blend_modulate", 2),
            ("B3_blend_modulate2", 3),
            ("B4_blend_over", 4),
        ] {
            assert_eq!(
                props(t).as_i64("CurrentTextureBlendMode"),
                Some(v),
                "{name} {t}"
            );
        }
        for (t, v) in [
            ("U1_use_shadow_map", 3),
            ("U2_use_light_map", 4),
            ("U3_use_spherical_reflection_map", 1),
            ("U4_use_sphere_reflection_map", 2),
            ("U5_use_bump_normal_map", 5),
        ] {
            assert_eq!(props(t).as_i64("TextureTypeUse"), Some(v), "{name} {t}");
        }
    }
}

/// The scene model's view: all 46 connections are listed, the
/// material's typed base-colour slot holds one of them, and a
/// pivot-free planar placement types onto the reference.
#[test]
fn bindings_and_typed_placement() {
    for name in flavours("texture-placement-sweep") {
        let scene = decode(&read(DIR, &name));
        let bindings = scene.extras["fbx:texture_bindings"]
            .as_array()
            .expect("bindings");
        assert_eq!(bindings.len(), 46, "{name}");
        for (i, b) in bindings.iter().enumerate() {
            assert_eq!(b["texture"], i as u64, "{name}");
            assert_eq!(b["material"], 0);
            assert_eq!(b["property"], "DiffuseColor");
        }
        let bound = scene.materials[0]
            .base_color_texture
            .expect("base colour texture");
        assert_eq!(
            scene.textures[bound.texture.0 as usize].name.as_deref(),
            Some("A2_alpha_source_black"),
            "{name}: the last connection holds the typed slot"
        );
    }
    // Re-binding the typed slot to the translated texture, with the
    // placement the texture authors: the round trip lists the new
    // connection in place of the one the slot held before, and the
    // placement types onto the reference.
    let mut scene = decode(&read(DIR, "texture-placement-sweep-binary-v7400.fbx"));
    let t06 = scene
        .textures
        .iter()
        .position(|t| t.name.as_deref() == Some("T06_translation_u0p25_v0p5"))
        .unwrap();
    let mut placement = oxideav_mesh3d::TextureTransform::IDENTITY;
    placement.offset = [0.25, 0.5];
    scene.materials[0].base_color_texture = Some(
        oxideav_mesh3d::TextureRef::new(oxideav_mesh3d::TextureId(t06 as u32))
            .with_transform(placement),
    );
    let back = decode(&encode(&scene, FbxOutputForm::Binary));
    let bindings = back.extras["fbx:texture_bindings"].as_array().unwrap();
    assert_eq!(bindings.len(), 46);
    assert_eq!(bindings[45]["texture"], t06 as u64);
    let bound = back.materials[0].base_color_texture.expect("bound");
    assert_eq!(bound.texture.0 as usize, t06);
    let transform = bound.transform.expect("typed placement");
    assert_eq!(transform.offset, [0.25, 0.5]);
    assert_eq!(transform.scale, [1.0, 1.0]);
    assert_eq!(transform.rotation, 0.0);
}
