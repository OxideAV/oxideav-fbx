//! `Texture` element `Properties70` surfacing — the `FbxFileTexture`
//! record set staged by `docs/3d/fbx/fbx-property-templates.md` §3.1.
//!
//! What these tests pin (decode side, ASCII front-end — the binary
//! form renders the identical node tree per
//! `docs/3d/fbx/fbx-binary-properties70.md` §4):
//!
//! - Every resolved `LayerElementUV`'s `Name` leaf lands on
//!   `Primitive::extras["fbx:uv_set_names"]` in channel order, and a
//!   `Texture` element's `UVSet` KString joins against those labels
//!   to select the typed `TextureRef::uv_set` channel index (the
//!   staged texture-video fixture shape: `UVSet = "UVChannel_1"`
//!   naming the geometry's first UV channel).
//! - Authored `Translation` / `Rotation` / `Scaling` placement
//!   records decode onto the typed
//!   [`oxideav_mesh3d::TextureTransform`] when the placement is
//!   purely 2D and pivot-free (rotation about the plane axis only,
//!   degrees → radians; offset / scale literal). Template defaults
//!   are the identity, so authored-vs-absent equals own-record
//!   presence — mirroring the mesh3d `None` = "no transform declared"
//!   contract, and an *authored identity* stays `Some(IDENTITY)`.
//! - Untypable records — non-zero `TextureRotationPivot` /
//!   `TextureScalingPivot` (composition order unpinned by any staged
//!   doc), `UVSwap`, and the enum-typed `WrapModeU` / `WrapModeV`
//!   (whose value table beyond the observed default `0` is a
//!   staged-docs gap) plus `UseMipMap` — surface raw on
//!   `Scene3D::extras["fbx:texture_records"]` keyed by scene texture
//!   index, and re-emit verbatim on encode.

use oxideav_fbx::{FbxDecoder, FbxEncoder};
use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DEncoder, Sampler, TextureTransform};

/// Minimal ASCII document: a quad with two named UV channels, one
/// material, one texture bound to `DiffuseColor` with the given
/// `Properties70` record lines spliced into the `Texture` element.
fn ascii_doc(texture_props: &str) -> Vec<u8> {
    format!(
        r#"; FBX 7.5.0 project file
; ----------------------------------------------------
Objects:  {{
	Geometry: 100, "Geometry::Quad", "Mesh" {{
		Vertices: *12 {{ a: 0.0,0.0,0.0,1.0,0.0,0.0,1.0,1.0,0.0,0.0,1.0,0.0 }}
		PolygonVertexIndex: *4 {{ a: 0,1,2,-4 }}
		LayerElementUV: 0 {{
			Version: 101
			Name: "diffuseUV"
			MappingInformationType: "ByPolygonVertex"
			ReferenceInformationType: "Direct"
			UV: *12 {{ a: 0.0,0.0,1.0,0.0,1.0,1.0,0.0,0.0,1.0,1.0,0.0,1.0 }}
		}}
		LayerElementUV: 1 {{
			Version: 101
			Name: "lightmapUV"
			MappingInformationType: "ByPolygonVertex"
			ReferenceInformationType: "Direct"
			UV: *12 {{ a: 0.0,0.0,0.5,0.0,0.5,0.5,0.0,0.0,0.5,0.5,0.0,0.5 }}
		}}
	}}
	Model: 200, "Model::QuadModel", "Mesh" {{
	}}
	Material: 300, "Material::Wood", "" {{
	}}
	Texture: 400, "Texture::WoodTex", "" {{
		Version: 202
		Properties70:  {{
{texture_props}
		}}
		FileName: "wood.png"
	}}
}}
Connections:  {{
	C: "OO",100,200
	C: "OO",200,0
	C: "OO",300,200
	C: "OP",400,300, "DiffuseColor"
}}
"#
    )
    .into_bytes()
}

fn decode(bytes: &[u8]) -> oxideav_mesh3d::Scene3D {
    FbxDecoder::new().decode(bytes).expect("decode")
}

#[test]
fn uv_set_names_surface_and_uvset_label_selects_channel() {
    let scene = decode(&ascii_doc(
        r#"			P: "UVSet", "KString", "", "", "lightmapUV""#,
    ));

    // Channel labels surfaced in document order.
    let prim = &scene.meshes[0].primitives[0];
    assert_eq!(prim.uvs.len(), 2, "two UV channels");
    let names: Vec<&str> = prim.extras["fbx:uv_set_names"]
        .as_array()
        .expect("fbx:uv_set_names array")
        .iter()
        .map(|v| v.as_str().unwrap())
        .collect();
    assert_eq!(names, ["diffuseUV", "lightmapUV"]);

    // The UVSet label picked the SECOND channel.
    let texref = scene.materials[0]
        .base_color_texture
        .expect("DiffuseColor binding");
    assert_eq!(texref.uv_set, 1, "UVSet joined to channel index 1");
    assert_eq!(texref.effective_uv_set(), 1);
    // No placement records authored → no transform declared.
    assert_eq!(texref.transform, None);
}

#[test]
fn unknown_uvset_label_keeps_default_channel() {
    let scene = decode(&ascii_doc(
        r#"			P: "UVSet", "KString", "", "", "noSuchChannel""#,
    ));
    let texref = scene.materials[0].base_color_texture.expect("binding");
    assert_eq!(texref.uv_set, 0, "unmatched label falls back to 0");
}

#[test]
fn planar_placement_decodes_to_typed_transform() {
    let scene = decode(&ascii_doc(
        r#"			P: "Translation", "Vector", "", "A",0.25,0.5,0
			P: "Rotation", "Vector", "", "A",0,0,90
			P: "Scaling", "Vector", "", "A",2,3,1"#,
    ));
    let texref = scene.materials[0].base_color_texture.expect("binding");
    let t = texref.transform.expect("typed transform attached");
    assert_eq!(t.offset, [0.25, 0.5]);
    assert!(
        (t.rotation - std::f32::consts::FRAC_PI_2).abs() < 1e-6,
        "90 deg -> pi/2 rad, got {}",
        t.rotation
    );
    assert_eq!(t.scale, [2.0, 3.0]);
    assert_eq!(t.uv_set, None);
    // Fully representable → nothing left to surface raw.
    assert!(!scene.extras.contains_key("fbx:texture_records"));
}

#[test]
fn authored_identity_stays_explicit_identity() {
    let scene = decode(&ascii_doc(
        r#"			P: "Translation", "Vector", "", "A",0,0,0"#,
    ));
    let texref = scene.materials[0].base_color_texture.expect("binding");
    assert_eq!(
        texref.transform,
        Some(TextureTransform::IDENTITY),
        "authored identity is Some(IDENTITY), distinguishable from absent (None)"
    );
}

#[test]
fn pivot_blocks_typed_transform_and_surfaces_raw() {
    let scene = decode(&ascii_doc(
        r#"			P: "Translation", "Vector", "", "A",0.25,0.5,0
			P: "TextureRotationPivot", "Vector3D", "Vector", "",0.5,0.5,0"#,
    ));
    let texref = scene.materials[0].base_color_texture.expect("binding");
    assert_eq!(
        texref.transform, None,
        "non-zero pivot: composition order unpinned, no typed claim"
    );
    let rec = scene.extras["fbx:texture_records"]
        .get("0")
        .expect("raw records for scene texture 0");
    assert_eq!(
        rec["translation"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_f64().unwrap())
            .collect::<Vec<_>>(),
        [0.25, 0.5, 0.0]
    );
    assert_eq!(
        rec["rotation_pivot"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_f64().unwrap())
            .collect::<Vec<_>>(),
        [0.5, 0.5, 0.0]
    );
}

#[test]
fn out_of_plane_rotation_blocks_typed_transform() {
    let scene = decode(&ascii_doc(r#"			P: "Rotation", "Vector", "", "A",45,0,90"#));
    let texref = scene.materials[0].base_color_texture.expect("binding");
    assert_eq!(texref.transform, None, "x-axis rotation has no 2D home");
    let rec = &scene.extras["fbx:texture_records"]["0"];
    assert!(rec.get("rotation").is_some(), "raw rotation surfaced");
}

#[test]
fn wrap_swap_mip_records_surface_raw_and_sampler_stays_default() {
    let scene = decode(&ascii_doc(
        r#"			P: "WrapModeU", "enum", "", "",1
			P: "WrapModeV", "enum", "", "",0
			P: "UVSwap", "bool", "", "",1
			P: "UseMipMap", "bool", "", "",1"#,
    ));
    let rec = &scene.extras["fbx:texture_records"]["0"];
    assert_eq!(rec["wrap_mode_u"].as_i64(), Some(1));
    assert_eq!(rec["wrap_mode_v"].as_i64(), Some(0));
    assert_eq!(rec["uv_swap"].as_bool(), Some(true));
    assert_eq!(rec["use_mip_map"].as_bool(), Some(true));
    // The wrap-enum value table beyond the observed default 0 is a
    // staged-docs gap — the typed sampler keeps the default state
    // (repeat wrapping, filters undefined) rather than guessing.
    assert_eq!(scene.textures[0].sampler, Sampler::default_sampler());
}

#[test]
fn uv_swap_blocks_typed_transform() {
    let scene = decode(&ascii_doc(
        r#"			P: "UVSwap", "bool", "", "",1
			P: "Scaling", "Vector", "", "A",2,2,1"#,
    ));
    let texref = scene.materials[0].base_color_texture.expect("binding");
    assert_eq!(
        texref.transform, None,
        "UVSwap interaction with the placement TRS is unpinned"
    );
    let rec = &scene.extras["fbx:texture_records"]["0"];
    assert!(rec.get("scaling").is_some());
    assert_eq!(rec["uv_swap"].as_bool(), Some(true));
}

/// Untypable records survive `decode → encode → decode` verbatim in
/// both output forms (the encoder re-emits `fbx:texture_records`
/// onto the `Texture` element's `Properties70`).
#[test]
fn raw_texture_records_round_trip() {
    let src = ascii_doc(
        r#"			P: "UVSet", "KString", "", "", "lightmapUV"
			P: "WrapModeU", "enum", "", "",1
			P: "UVSwap", "bool", "", "",1
			P: "Translation", "Vector", "", "A",0.25,0.5,0
			P: "TextureScalingPivot", "Vector3D", "Vector", "",0.1,0.2,0"#,
    );
    let scene = decode(&src);
    let records = scene.extras["fbx:texture_records"].clone();
    assert!(records.get("0").is_some());

    for (form, bytes) in [
        ("binary", FbxEncoder::new().encode(&scene).expect("binary")),
        (
            "ascii",
            FbxEncoder::new()
                .form(oxideav_fbx::FbxOutputForm::Ascii)
                .encode(&scene)
                .expect("ascii"),
        ),
    ] {
        let scene2 = decode(&bytes);
        assert_eq!(
            scene2.extras.get("fbx:texture_records"),
            Some(&records),
            "{form}: raw records survive verbatim"
        );
        let texref = scene2.materials[0].base_color_texture.expect("binding");
        assert_eq!(texref.transform, None, "{form}: still no typed claim");
        assert_eq!(texref.uv_set, 1, "{form}: UVSet label re-joined");
    }
}
