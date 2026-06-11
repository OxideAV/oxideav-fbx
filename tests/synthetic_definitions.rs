//! End-to-end integration test for round-280 `Definitions` /
//! `PropertyTemplate` default resolution.
//!
//! Builds a synthetic ASCII-FBX document (the ASCII front-end produces
//! the same typed `FbxDocument` tree as the binary reader, per
//! `docs/3d/fbx/fbx-binary-properties70.md` §4's isomorphism note)
//! whose `Definitions` section carries an `ObjectType: "Material"`
//! `PropertyTemplate` per `docs/3d/fbx/fbx-ascii-grammar.md` §7b, plus
//! two `Material` objects:
//!
//! 1. `TemplateBacked` — re-states only `DiffuseColor`; its
//!    `DiffuseFactor` / `EmissiveColor` / `ShadingModel` must resolve
//!    from the class template's *"default property set"*.
//! 2. `AllDefaults` — writes no `Properties70` at all; every channel
//!    must resolve from the template.

use oxideav_fbx::FbxDecoder;
use oxideav_mesh3d::Mesh3DDecoder;

const SRC: &[u8] = b"; FBX 7.5.0 project file\n\
; ----------------------------------------------------\n\
FBXHeaderExtension:  {\n\
\tFBXVersion: 7500\n\
}\n\
Definitions:  {\n\
\tVersion: 100\n\
\tCount: 2\n\
\tObjectType: \"Material\" {\n\
\t\tCount: 2\n\
\t\tPropertyTemplate: \"FbxSurfaceLambert\" {\n\
\t\t\tProperties70:  {\n\
\t\t\t\tP: \"ShadingModel\", \"KString\", \"\", \"\", \"Lambert\"\n\
\t\t\t\tP: \"EmissiveColor\", \"Color\", \"\", \"A\",0.5,0,0\n\
\t\t\t\tP: \"EmissiveFactor\", \"Number\", \"\", \"A\",0.5\n\
\t\t\t\tP: \"DiffuseColor\", \"Color\", \"\", \"A\",0.8,0.8,0.8\n\
\t\t\t\tP: \"DiffuseFactor\", \"Number\", \"\", \"A\",0.5\n\
\t\t\t}\n\
\t\t}\n\
\t}\n\
}\n\
Objects:  {\n\
\tMaterial: 300, \"Material::TemplateBacked\", \"\" {\n\
\t\tVersion: 102\n\
\t\tProperties70:  {\n\
\t\t\tP: \"DiffuseColor\", \"Color\", \"\", \"A\",0,1,0\n\
\t\t}\n\
\t}\n\
\tMaterial: 301, \"Material::AllDefaults\", \"\" {\n\
\t\tVersion: 102\n\
\t}\n\
}\n\
Connections:  {\n\
}\n";

#[test]
fn material_template_defaults_resolve_into_scene_materials() {
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(SRC).expect("synthetic ASCII doc decodes");
    assert_eq!(scene.materials.len(), 2);

    // Materials materialise in sorted-FBX-id order: 300 then 301.
    let backed = &scene.materials[0];
    assert_eq!(backed.name.as_deref(), Some("Material::TemplateBacked"));
    // Own DiffuseColor (0,1,0) wins over the template's (0.8,0.8,0.8);
    // the template's DiffuseFactor 0.5 (not re-stated by the object)
    // multiplies in.
    assert!((backed.base_color[0] - 0.0).abs() < 1e-6);
    assert!((backed.base_color[1] - 0.5).abs() < 1e-6);
    assert!((backed.base_color[2] - 0.0).abs() < 1e-6);
    // EmissiveColor (0.5,0,0) x EmissiveFactor 0.5 — both pure
    // template defaults.
    assert!((backed.emissive_factor[0] - 0.25).abs() < 1e-6);
    assert!((backed.emissive_factor[1] - 0.0).abs() < 1e-6);
    // ShadingModel resolves from the template (no own record, no
    // direct-child leaf).
    assert_eq!(
        backed
            .extras
            .get("fbx:shading_model")
            .and_then(|v| v.as_str()),
        Some("Lambert")
    );

    let defaults = &scene.materials[1];
    assert_eq!(defaults.name.as_deref(), Some("Material::AllDefaults"));
    // No Properties70 at all: every channel is the §7b class default.
    // DiffuseColor (0.8,0.8,0.8) x DiffuseFactor 0.5 = 0.4.
    for c in &defaults.base_color[..3] {
        assert!((*c - 0.4).abs() < 1e-6, "expected 0.4, got {c}");
    }
    assert!((defaults.emissive_factor[0] - 0.25).abs() < 1e-6);
    assert_eq!(
        defaults
            .extras
            .get("fbx:shading_model")
            .and_then(|v| v.as_str()),
        Some("Lambert")
    );
}

#[test]
fn definitions_surface_decodes_from_the_same_document() {
    use oxideav_fbx::definitions::Definitions;

    let mut dec = FbxDecoder::new();
    let _ = dec.decode(SRC).expect("synthetic ASCII doc decodes");
    let doc = dec.last_document.as_ref().unwrap();
    let defs = Definitions::from_document(doc);
    assert_eq!(defs.version, Some(100));
    assert_eq!(defs.total_count, Some(2));
    assert_eq!(defs.object_types(), vec!["Material"]);
    let mat = defs.get("Material").expect("Material class decoded");
    assert_eq!(mat.count, Some(2));
    assert_eq!(mat.template_name.as_deref(), Some("FbxSurfaceLambert"));
    let tpl = defs.template_for("Material").expect("Material template");
    assert_eq!(tpl.len(), 5);
    assert_eq!(tpl.as_vec3("DiffuseColor"), Some([0.8, 0.8, 0.8]));
    assert_eq!(tpl.as_f64("DiffuseFactor"), Some(0.5));
    assert_eq!(tpl.as_str("ShadingModel"), Some("Lambert"));
}
