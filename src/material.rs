//! Material / Texture / Video surfacing onto [`Scene3D`].
//!
//! Per `docs/3d/fbx/fbx-binary-properties70.md` §5–§7, an FBX file
//! describes surface appearance via three element types in the
//! `Objects` block (`Material`, `Texture`, `Video`):
//!
//! ```text
//! Objects {
//!   Material : i64 id, "Name\x00\x01Material", "subtype" { ... }
//!   Texture  : i64 id, "Name\x00\x01Texture",  "subtype" {
//!       RelativeFilename : "<path>"
//!       FileName         : "<path>"   // alternative — sometimes both
//!   }
//!   Video    : i64 id, "Name\x00\x01Video",    "subtype" {
//!       RelativeFilename : "<path>"
//!       Content          : R<bytes>   // optional embedded media blob
//!   }
//! }
//! ```
//!
//! `RelativeFilename` and `FileName` are the two well-known direct
//! sub-records that carry a string-typed image-path payload. The
//! `Content` blob on `Video` records may carry the embedded image
//! bytes for self-contained FBX exports.
//!
//! The wiring between elements uses the `Connections` `C`-record graph
//! (`docs/3d/fbx/fbx-binary-properties70.md` §7):
//!
//! ```text
//! Material  --(OO)--> Model                — surface assignment
//! Texture   --(OP, "DiffuseColor"
//!                  / "Maya|TEX_color_map"
//!                  / "NormalMap"
//!                  / ...)
//!                                 --> Material
//! Video     --(OO)--> Texture              — backing media
//! ```
//!
//! # What this round surfaces
//!
//! - One [`oxideav_mesh3d::Material`] per FBX `Material` element, with
//!   its `name` field populated from the FBX element-name.
//!   PBR factors / colours are decoded from the element's
//!   `Properties70` `P`-record block via
//!   [`crate::properties70::PropertyMap`]:
//!   `DiffuseColor` × `DiffuseFactor` → `base_color` rgb,
//!   `Opacity` → `base_color[3]` + `AlphaMode::Blend` when < 1,
//!   `EmissiveColor` × `EmissiveFactor` → `emissive_factor`,
//!   `Shininess` (Phong exponent) → `roughness` via
//!   `sqrt(2/(n+2))`, `ReflectionFactor` → `metallic`, and
//!   `ShadingModel` → `Material::extras["fbx:shading_model"]`. See
//!   `docs/3d/fbx/fbx-binary-properties70.md` §4 for the `P`-record
//!   grammar this consumes.
//! - One [`oxideav_mesh3d::Texture`] per FBX `Texture` element, built
//!   from the `RelativeFilename` sub-record (fallback: `FileName`) via
//!   [`oxideav_mesh3d::Texture::from_uri`]. When the texture is
//!   connected to a `Video` element that carries a `Content` blob, we
//!   prefer the embedded bytes via [`oxideav_mesh3d::Texture::from_encoded`]
//!   so self-contained FBX files don't need an external file resolve.
//! - `Connections OP Texture -> Material(prop_name)` wires the texture
//!   into the matching [`oxideav_mesh3d::Material`] channel:
//!   `DiffuseColor` / `Maya|TEX_color_map` map to `base_color_texture`,
//!   `NormalMap` / `Maya|TEX_normal_map` map to `normal_texture`,
//!   `EmissiveColor` / `Maya|TEX_emissive_map` map to `emissive_texture`.
//!   Other channels round-trip through the [`crate::FbxDocument`] but
//!   don't surface a typed binding (the PBR map list on
//!   [`oxideav_mesh3d::Material`] is glTF-style metallic/roughness
//!   only; unmapped channels would round-trip via `Material::extras`,
//!   which is left for the encoder side of a future round).
//! - `Connections OO Material -> Model` sets the first connected
//!   material on every [`oxideav_mesh3d::Primitive`] of the model's
//!   mesh (`Primitive::material`). When more than one `Material` is
//!   OO-connected to the same `Model`, the full slot table lands on
//!   `Primitive::extras["fbx:material_slots"]` (round 178) — the
//!   per-corner indices into this table that
//!   [`crate::geometry::pull_layer_material_slots`] stashes on
//!   `Primitive::extras["fbx:face_material_slots"]` give a downstream
//!   consumer everything it needs to split the primitive into one
//!   per-material primitive without re-walking the FBX document.

use std::collections::HashMap;

use oxideav_mesh3d::{
    AlphaMode, Material, MaterialId, MeshId, NodeId, Scene3D, Texture, TextureId,
};

use crate::binary::{FbxDocument, FbxNode, FbxProperty};
use crate::definitions::Definitions;
use crate::properties70::PropertyMap;

/// Walk the top-level `Objects` + `Connections` records to populate
/// `Scene3D::materials` + `Scene3D::textures` and wire them into the
/// already-built `Primitive::material` slots.
///
/// `model_to_mesh` is the per-model `MeshId` lookup produced by the
/// scene builder (one entry per FBX `Model` element that received a
/// `Geometry` OO connection). Materials connected to a model with no
/// mesh entry are still created in the scene's material arena, but no
/// primitive binding is performed.
pub fn extract_materials(
    doc: &FbxDocument,
    scene: &mut Scene3D,
    model_to_mesh: &HashMap<i64, MeshId>,
    _model_nodes: &HashMap<i64, NodeId>,
) {
    // 1) Index every Material / Texture / Video element in Objects.
    let mut fbx_materials: HashMap<i64, &FbxNode> = HashMap::new();
    let mut fbx_textures: HashMap<i64, &FbxNode> = HashMap::new();
    let mut fbx_videos: HashMap<i64, &FbxNode> = HashMap::new();

    if let Some(objects) = doc.root.child("Objects") {
        for child in &objects.children {
            let id = match child.properties.first().and_then(FbxProperty::as_i64) {
                Some(i) => i,
                None => continue,
            };
            match child.name.as_str() {
                "Material" => {
                    fbx_materials.insert(id, child);
                }
                "Texture" => {
                    fbx_textures.insert(id, child);
                }
                "Video" => {
                    fbx_videos.insert(id, child);
                }
                _ => {}
            }
        }
    }

    if fbx_materials.is_empty() && fbx_textures.is_empty() {
        return;
    }

    // 2) Walk Connections so we know which Video backs which Texture
    //    (`Video -> Texture` OO), which Texture binds which Material
    //    slot (`Texture -> Material(prop_name)` OP), and which Material
    //    is assigned to which Model (`Material -> Model` OO).
    let mut video_of_texture: HashMap<i64, i64> = HashMap::new();
    let mut texture_bindings: Vec<(i64, i64, String)> = Vec::new(); // (texture_id, material_id, prop)
    let mut model_to_materials: HashMap<i64, Vec<i64>> = HashMap::new();

    if let Some(conns) = doc.root.child("Connections") {
        for c in conns.children_named("C") {
            let kind = c.properties.first().and_then(FbxProperty::as_str);
            let child_id = c.properties.get(1).and_then(FbxProperty::as_i64);
            let parent_id = c.properties.get(2).and_then(FbxProperty::as_i64);
            let (Some(kind), Some(child_id), Some(parent_id)) = (kind, child_id, parent_id) else {
                continue;
            };
            match kind {
                "OO" => {
                    if fbx_videos.contains_key(&child_id) && fbx_textures.contains_key(&parent_id) {
                        video_of_texture.insert(parent_id, child_id);
                    } else if fbx_materials.contains_key(&child_id)
                        && model_to_mesh.contains_key(&parent_id)
                    {
                        model_to_materials
                            .entry(parent_id)
                            .or_default()
                            .push(child_id);
                    }
                }
                "OP" if fbx_textures.contains_key(&child_id)
                    && fbx_materials.contains_key(&parent_id) =>
                {
                    // Texture -> Material OP records carry the
                    // channel name in property[3] (e.g. "DiffuseColor",
                    // "NormalMap"). Other OP shapes (Texture ->
                    // AnimationCurveNode for animated UV transforms,
                    // etc.) are deferred — they round-trip through
                    // FbxDocument but don't surface a typed binding.
                    if let Some(prop) = c.properties.get(3).and_then(FbxProperty::as_str) {
                        texture_bindings.push((child_id, parent_id, prop.to_owned()));
                    }
                }
                _ => {}
            }
        }
    }

    // 3) Materialise Texture elements onto the scene. Prefer embedded
    //    `Video.Content` over `RelativeFilename` / `FileName` so a
    //    self-contained FBX file decodes without an external file
    //    resolver. Keep an `fbx_id -> TextureId` map for the OP-binding
    //    walk in step 5.
    let mut texture_lookup: HashMap<i64, TextureId> = HashMap::new();
    // Sort by FBX id so the materialisation order is deterministic
    // across HashMap-iteration-order variations between Rust releases.
    let mut texture_ids: Vec<i64> = fbx_textures.keys().copied().collect();
    texture_ids.sort_unstable();
    for tid in texture_ids {
        let tex_node = match fbx_textures.get(&tid) {
            Some(n) => *n,
            None => continue,
        };
        let name = element_name(tex_node);
        let video_id = video_of_texture.get(&tid).copied();
        let video_node = video_id.and_then(|v| fbx_videos.get(&v).copied());

        // Prefer the embedded Video.Content blob if present — that's
        // the self-contained-FBX case (the `Content` R-blob on a
        // `Video` record).
        let embedded = video_node.and_then(read_content_blob);
        let tex = if let Some(bytes) = embedded {
            let mime = video_node
                .and_then(|v| read_string_child(v, "Filename"))
                .or_else(|| video_node.and_then(|v| read_string_child(v, "RelativeFilename")))
                .as_deref()
                .and_then(guess_mime_from_path)
                .unwrap_or_else(|| "application/octet-stream".to_string());
            let mut t = Texture::from_encoded(mime, bytes);
            t.name = name;
            t
        } else {
            let uri = read_string_child(tex_node, "RelativeFilename")
                .or_else(|| read_string_child(tex_node, "FileName"))
                .or_else(|| read_string_child(tex_node, "Filename"))
                .or_else(|| video_node.and_then(|v| read_string_child(v, "RelativeFilename")))
                .or_else(|| video_node.and_then(|v| read_string_child(v, "FileName")))
                .unwrap_or_default();
            let mut t = Texture::from_uri(uri);
            t.name = name;
            t
        };
        let tex_id = scene.add_texture(tex);
        texture_lookup.insert(tid, tex_id);
    }

    // 4) Materialise Material elements. The `Properties70` `P`-record
    //    grammar (now staged in `docs/3d/fbx/fbx-binary-properties70.md`
    //    §4) lets us decode the real PBR factors: `DiffuseColor` /
    //    `Diffuse` (vec3) → `base_color` rgb; `DiffuseFactor` (scalar)
    //    multiplies into that rgb; `Opacity` (scalar, 1.0 = opaque) →
    //    `base_color[3]` + `AlphaMode::Blend` when < 1; `EmissiveColor`
    //    (vec3) + `EmissiveFactor` (scalar) → `emissive_factor`;
    //    `Shininess` / `ShininessExponent` (Phong specular exponent)
    //    → `roughness` via a 1 − tanh(N/96) mapping;
    //    `ReflectionFactor` → `metallic` (FBX Phong/Lambert has no
    //    proper metallic channel, but ReflectionFactor is the closest
    //    legacy-shader analogue). `ShadingModel` is captured into
    //    `Material::extras` for downstream consumers that need the
    //    raw FBX shader-kind hint.
    //    Round 280: each material's own `Properties70` block is
    //    resolved against the `Definitions` `ObjectType: "Material"`
    //    `PropertyTemplate` (the docs ascii-grammar §7b "default
    //    property set" for the class), so exporter-omitted defaults
    //    (e.g. the FbxSurfaceLambert template's `DiffuseFactor = 1`)
    //    decode the same as explicitly-written records.
    let definitions = Definitions::from_root(&doc.root);
    let material_template = definitions.template_for("Material");
    let mut material_lookup: HashMap<i64, MaterialId> = HashMap::new();
    let mut material_ids: Vec<i64> = fbx_materials.keys().copied().collect();
    material_ids.sort_unstable();
    for mid in material_ids {
        let mat_node = match fbx_materials.get(&mid) {
            Some(n) => *n,
            None => continue,
        };
        let mut mat = Material::new();
        mat.name = element_name(mat_node);
        apply_properties70(mat_node, &mut mat, material_template);
        let mat_id = scene.add_material(mat);
        material_lookup.insert(mid, mat_id);
    }

    // 5) Wire texture -> material bindings via the OP prop_name slot
    //    map. Unrecognised property names round-trip through the
    //    FbxDocument but don't surface as typed bindings.
    for (texture_fid, material_fid, prop) in &texture_bindings {
        let tex_id = match texture_lookup.get(texture_fid) {
            Some(&t) => t,
            None => continue,
        };
        let mat_id = match material_lookup.get(material_fid) {
            Some(&m) => m,
            None => continue,
        };
        let mat = match scene.materials.get_mut(mat_id.0 as usize) {
            Some(m) => m,
            None => continue,
        };
        let texref = oxideav_mesh3d::TextureRef::new(tex_id);
        match prop.as_str() {
            // Base-colour aliases. Maya / 3ds-Max / standard FBX-2014
            // exporters each carry one of these OP-connection property
            // names (observed exporter conventions).
            "DiffuseColor"
            | "Maya|TEX_color_map"
            | "Maya|baseColor"
            | "3dsMax|main|base_color_map" => {
                mat.base_color_texture = Some(texref);
            }
            // Normal-map aliases.
            "NormalMap" | "Maya|TEX_normal_map" | "3dsMax|main|norm_map" => {
                mat.normal_texture = Some(texref);
            }
            // Emission-map aliases.
            "EmissiveColor"
            | "Maya|TEX_emissive_map"
            | "Maya|emissionColor"
            | "3dsMax|main|emit_color_map" => {
                mat.emissive_texture = Some(texref);
            }
            // Metallic-roughness packed map (3ds Max Physical / FBX
            // PBR exporter convention; there is no canonical FBX
            // 2014-Lambert name — these are the recognised PBR exporter
            // slots).
            "Maya|TEX_metallic_map" | "3dsMax|main|metalness_map" => {
                mat.metallic_roughness_texture = Some(texref);
            }
            // Occlusion-map aliases.
            "Maya|TEX_ao_map" | "AmbientOcclusion" => {
                mat.occlusion_texture = Some(texref);
            }
            _ => {
                // Unrecognised binding name: deferred. The texture +
                // material both still exist on the scene; only the
                // typed slot mapping is skipped.
            }
        }
    }

    // 6) Attach the materials connected to each Model to that model's
    //    mesh primitives.
    //
    //    Single-material case: every primitive's `material` slot is
    //    set to the first (and only) connected material. This matches
    //    the simple FBX-export shape every legacy renderer expects.
    //
    //    Multi-material case (round 178): a `Model` may receive more
    //    than one `Material -> Model` OO connection. The N connected
    //    materials occupy slots 0..N in connection order. The per-corner
    //    material-slot indices that `geometry::pull_layer_material_slots`
    //    stashed onto `Primitive::extras["fbx:face_material_slots"]`
    //    key into this same slot vector. We surface the slot table on
    //    `Primitive::extras["fbx:material_slots"]` as a JSON array of
    //    `MaterialId.0` numbers so a downstream consumer can split the
    //    primitive on material boundaries; the legacy
    //    `Primitive::material` field stays set to slot 0 for
    //    single-binding renderers.
    for (model_fid, fbx_material_ids) in &model_to_materials {
        let mesh_id = match model_to_mesh.get(model_fid) {
            Some(&m) => m,
            None => continue,
        };
        let mat_slots: Vec<oxideav_mesh3d::MaterialId> = fbx_material_ids
            .iter()
            .filter_map(|fid| material_lookup.get(fid).copied())
            .collect();
        if mat_slots.is_empty() {
            continue;
        }
        if let Some(mesh) = scene.meshes.get_mut(mesh_id.0 as usize) {
            for prim in &mut mesh.primitives {
                // Single-binding back-compat: default to slot 0.
                prim.material = Some(mat_slots[0]);
                // Always record the slot table when the model carries
                // more than one connected material — even if the
                // geometry's LayerElementMaterial mapping mode is
                // `AllSame`, downstream consumers may want to walk
                // every connected material (e.g. an editor surfacing
                // unused secondary slots).
                if mat_slots.len() > 1 {
                    prim.extras.insert(
                        "fbx:material_slots".to_string(),
                        serde_json::Value::Array(
                            mat_slots
                                .iter()
                                .map(|m| serde_json::Value::Number(serde_json::Number::from(m.0)))
                                .collect(),
                        ),
                    );
                }
            }
        }
    }
}

/// Read the user-facing element name from property[1] (FBX joins
/// `Name + \x00\x01 + SubType` in the binary encoding — we strip the
/// separator and return only the leading name).
fn element_name(node: &FbxNode) -> Option<String> {
    let raw = match node.properties.get(1)? {
        FbxProperty::String(b) => b,
        _ => return None,
    };
    if let Some(sep) = raw.iter().position(|&b| b == 0x00) {
        std::str::from_utf8(&raw[..sep]).ok().map(str::to_owned)
    } else {
        std::str::from_utf8(raw).ok().map(str::to_owned)
    }
}

/// Decode the FBX `Material`'s `Properties70` `P`-record block per
/// `docs/3d/fbx/fbx-binary-properties70.md` §4 and populate the
/// matching channels on the typed [`Material`].
///
/// The mapping is the standard FBX-classic-material → PBR translation:
/// diffuse colour × diffuse factor → base
/// colour; opacity → base-colour alpha (+ alpha mode); emissive colour
/// × emissive factor → emissive factor; specular shininess → roughness
/// (via the standard Blinn-Phong → GGX conversion `roughness ≈
/// sqrt(2 / (shininess + 2))`); reflection factor → metallic; raw
/// `ShadingModel` string captured into `extras["fbx:shading_model"]`
/// so downstream consumers can distinguish phong / lambert / unknown.
///
/// `template` is the class-default `Properties70` set from the
/// `Definitions` section's `ObjectType: "Material"` `PropertyTemplate`
/// (docs ascii-grammar §7b — *"a `PropertyTemplate` holding the
/// default `Properties70` for that class"*). The material's own
/// records overlay the template defaults, so a name the exporter left
/// at its class default still resolves to the documented value.
fn apply_properties70(node: &FbxNode, mat: &mut Material, template: Option<&PropertyMap>) {
    let own = PropertyMap::from_element(node);
    let pm = match template {
        Some(t) => own.with_template_defaults(t),
        None => own.clone(),
    };
    if pm.is_empty() {
        // Also accept a top-level `ShadingModel` leaf — the docs
        // show it lives in `Properties70` for newer exporters but
        // sometimes is a direct child for older FBX-2014 files.
        if let Some(shading) = read_string_child(node, "ShadingModel") {
            mat.extras.insert(
                "fbx:shading_model".into(),
                serde_json::Value::String(shading),
            );
        }
        return;
    }

    // Diffuse colour × DiffuseFactor → base_color rgb.
    //
    // `DiffuseColor` is the canonical FBX classic-material P-record
    // name; some exporters also write `Diffuse` (already-baked
    // rgb × factor).
    let mut diffuse_rgb = pm
        .as_vec3("DiffuseColor")
        .or_else(|| pm.as_vec3("Diffuse"))
        .map(|v| [v[0] as f32, v[1] as f32, v[2] as f32]);
    if let (Some(rgb), Some(factor)) = (diffuse_rgb.as_mut(), pm.as_f64("DiffuseFactor")) {
        let f = factor as f32;
        for c in rgb.iter_mut() {
            *c *= f;
        }
    }
    if let Some(rgb) = diffuse_rgb {
        mat.base_color = [rgb[0], rgb[1], rgb[2], mat.base_color[3]];
    }

    // Opacity → base-colour alpha. FBX defaults to 1.0; only switch
    // to AlphaMode::Blend when the file explicitly says < 1.
    if let Some(opacity) = pm.as_f64("Opacity") {
        let a = (opacity as f32).clamp(0.0, 1.0);
        mat.base_color[3] = a;
        if a < 1.0 {
            mat.alpha_mode = AlphaMode::Blend;
        }
    }

    // Emissive colour × EmissiveFactor → emissive_factor rgb.
    let mut emissive_rgb = pm
        .as_vec3("EmissiveColor")
        .map(|v| [v[0] as f32, v[1] as f32, v[2] as f32]);
    if let (Some(rgb), Some(factor)) = (emissive_rgb.as_mut(), pm.as_f64("EmissiveFactor")) {
        let f = factor as f32;
        for c in rgb.iter_mut() {
            *c *= f;
        }
    }
    if let Some(rgb) = emissive_rgb {
        mat.emissive_factor = rgb;
    }

    // Specular shininess → roughness.
    //
    // The Blinn-Phong specular exponent `n` (FBX `Shininess` /
    // `ShininessExponent` P-record) converts to GGX-style roughness
    // via the well-known relation
    // `roughness ≈ sqrt(2 / (n + 2))` — bright/mirror Phong (n→∞)
    // collapses to roughness → 0, matte Phong (n→0) goes to roughness
    // → 1. Cap on input to avoid NaN on n < 0.
    if let Some(n) = pm
        .as_f64("Shininess")
        .or_else(|| pm.as_f64("ShininessExponent"))
    {
        let n = n.max(0.0) as f32;
        let r = (2.0 / (n + 2.0)).sqrt().clamp(0.0, 1.0);
        mat.roughness = r;
    }

    // ReflectionFactor → metallic. FBX legacy shaders have no proper
    // metallic channel; `ReflectionFactor` is the closest analogue in
    // both Phong and Lambert classic materials. Phong defaults to a
    // matte-dielectric look (metallic = 0) when ReflectionFactor is
    // unset; we honour that by NOT touching `mat.metallic` here unless
    // the file supplies the value.
    if let Some(rf) = pm.as_f64("ReflectionFactor") {
        mat.metallic = (rf as f32).clamp(0.0, 1.0);
    }

    // ShadingModel → extras["fbx:shading_model"]. Captured raw so
    // downstream consumers can branch on `"phong"` / `"lambert"` /
    // `"Maya|standardSurface"` / etc. without re-walking the document.
    //
    // Per the docs §4 sample, ShadingModel may live as a direct-child
    // leaf (`box.fbx` Material →`ShadingModel: "phong"`) OR as a
    // Properties70 P-record on newer exporters — accept either.
    // Precedence keeps instance data ahead of class defaults: own
    // P-record > direct-child leaf > Definitions-template default
    // (the cubes fixture writes the leaf `ShadingModel: "lambert"`
    // on each Material while its FbxSurfaceLambert template carries
    // a `"Lambert"` P-record — the instance leaf must win).
    let shading = own
        .as_str("ShadingModel")
        .map(str::to_owned)
        .or_else(|| read_string_child(node, "ShadingModel"))
        .or_else(|| pm.as_str("ShadingModel").map(str::to_owned));
    if let Some(s) = shading {
        mat.extras
            .insert("fbx:shading_model".into(), serde_json::Value::String(s));
    }
}

/// Read a direct-child node's first string property (string-typed
/// FBX sub-records carry a single `S` property). Used for
/// `RelativeFilename` / `FileName` lookups on Texture + Video records.
fn read_string_child(parent: &FbxNode, name: &str) -> Option<String> {
    let n = parent.child(name)?;
    match n.properties.first()? {
        FbxProperty::String(bytes) => std::str::from_utf8(bytes).ok().map(str::to_owned),
        _ => None,
    }
}

/// Read a direct-child `Content` node's first `R` (raw blob) property.
/// FBX `Video` records carry the embedded media payload here (the
/// `Content` R-blob, per `docs/3d/fbx/fbx-binary-properties70.md` §3c).
fn read_content_blob(node: &FbxNode) -> Option<Vec<u8>> {
    let c = node.child("Content")?;
    match c.properties.first()? {
        FbxProperty::Raw(bytes) => {
            if bytes.is_empty() {
                None
            } else {
                Some(bytes.clone())
            }
        }
        // Some exporters mis-tag the embedded blob as `S` (string)
        // rather than `R` (raw). Accept either — both type codes have
        // identical wire layout (`u32 length | bytes`).
        FbxProperty::String(bytes) => {
            if bytes.is_empty() {
                None
            } else {
                Some(bytes.clone())
            }
        }
        _ => None,
    }
}

/// Lightweight extension-to-MIME guess for embedded textures. Covers
/// the formats every FBX exporter actually emits (PNG, JPEG, TGA,
/// BMP); anything else falls through to the
/// `application/octet-stream` default.
fn guess_mime_from_path(path: &str) -> Option<String> {
    let lower = path.to_ascii_lowercase();
    let ext = lower.rsplit('.').next()?;
    match ext {
        "png" => Some("image/png".into()),
        "jpg" | "jpeg" => Some("image/jpeg".into()),
        "tga" => Some("image/x-targa".into()),
        "bmp" => Some("image/bmp".into()),
        "tif" | "tiff" => Some("image/tiff".into()),
        "exr" => Some("image/x-exr".into()),
        "hdr" => Some("image/vnd.radiance".into()),
        "gif" => Some("image/gif".into()),
        "webp" => Some("image/webp".into()),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_mesh3d::{Mesh, Primitive, Topology};

    fn make_doc(objects: Vec<FbxNode>, connections: Vec<FbxNode>) -> FbxDocument {
        FbxDocument {
            version: 7400,
            root: FbxNode {
                name: String::new(),
                properties: Vec::new(),
                children: vec![
                    FbxNode {
                        name: "Objects".into(),
                        properties: Vec::new(),
                        children: objects,
                    },
                    FbxNode {
                        name: "Connections".into(),
                        properties: Vec::new(),
                        children: connections,
                    },
                ],
            },
        }
    }

    fn material_elem(id: i64, name: &str) -> FbxNode {
        let mut concat = name.as_bytes().to_vec();
        concat.extend_from_slice(b"\x00\x01Material");
        FbxNode {
            name: "Material".into(),
            properties: vec![
                FbxProperty::I64(id),
                FbxProperty::String(concat),
                FbxProperty::String(b"".to_vec()),
            ],
            children: Vec::new(),
        }
    }

    fn texture_elem(id: i64, name: &str, relative_filename: Option<&str>) -> FbxNode {
        let mut concat = name.as_bytes().to_vec();
        concat.extend_from_slice(b"\x00\x01Texture");
        let mut children: Vec<FbxNode> = Vec::new();
        if let Some(rf) = relative_filename {
            children.push(FbxNode {
                name: "RelativeFilename".into(),
                properties: vec![FbxProperty::String(rf.as_bytes().to_vec())],
                children: Vec::new(),
            });
        }
        FbxNode {
            name: "Texture".into(),
            properties: vec![
                FbxProperty::I64(id),
                FbxProperty::String(concat),
                FbxProperty::String(b"".to_vec()),
            ],
            children,
        }
    }

    fn video_elem(id: i64, name: &str, filename: Option<&str>, content: Option<&[u8]>) -> FbxNode {
        let mut concat = name.as_bytes().to_vec();
        concat.extend_from_slice(b"\x00\x01Video");
        let mut children: Vec<FbxNode> = Vec::new();
        if let Some(f) = filename {
            children.push(FbxNode {
                name: "Filename".into(),
                properties: vec![FbxProperty::String(f.as_bytes().to_vec())],
                children: Vec::new(),
            });
        }
        if let Some(c) = content {
            children.push(FbxNode {
                name: "Content".into(),
                properties: vec![FbxProperty::Raw(c.to_vec())],
                children: Vec::new(),
            });
        }
        FbxNode {
            name: "Video".into(),
            properties: vec![
                FbxProperty::I64(id),
                FbxProperty::String(concat),
                FbxProperty::String(b"".to_vec()),
            ],
            children,
        }
    }

    fn conn_oo(child: i64, parent: i64) -> FbxNode {
        FbxNode {
            name: "C".into(),
            properties: vec![
                FbxProperty::String(b"OO".to_vec()),
                FbxProperty::I64(child),
                FbxProperty::I64(parent),
            ],
            children: Vec::new(),
        }
    }

    fn conn_op(child: i64, parent: i64, prop: &str) -> FbxNode {
        FbxNode {
            name: "C".into(),
            properties: vec![
                FbxProperty::String(b"OP".to_vec()),
                FbxProperty::I64(child),
                FbxProperty::I64(parent),
                FbxProperty::String(prop.as_bytes().to_vec()),
            ],
            children: Vec::new(),
        }
    }

    /// Helper to seed a Scene3D with one mesh + one primitive so the
    /// Material -> Model -> Mesh attachment step has something to
    /// land on.
    fn seed_scene_with_mesh() -> (Scene3D, MeshId) {
        let mut scene = Scene3D::new();
        let mut mesh = Mesh::new(Some("Quad".into()));
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0]; 3];
        mesh.primitives.push(prim);
        let mesh_id = scene.add_mesh(mesh);
        (scene, mesh_id)
    }

    #[test]
    fn extracts_material_and_attaches_to_mesh() {
        // Model 200 -> Mesh ; Material 300 -> Model 200.
        let (mut scene, mesh_id) = seed_scene_with_mesh();
        let mut model_to_mesh: HashMap<i64, MeshId> = HashMap::new();
        model_to_mesh.insert(200, mesh_id);
        let model_nodes: HashMap<i64, NodeId> = HashMap::new();

        let doc = make_doc(vec![material_elem(300, "Steel")], vec![conn_oo(300, 200)]);
        extract_materials(&doc, &mut scene, &model_to_mesh, &model_nodes);

        assert_eq!(scene.materials.len(), 1, "one material surfaced");
        assert_eq!(scene.materials[0].name.as_deref(), Some("Steel"));
        let prim_mat = scene.meshes[mesh_id.0 as usize].primitives[0].material;
        assert_eq!(
            prim_mat.map(|m| m.0),
            Some(0),
            "material bound to mesh primitive"
        );
    }

    #[test]
    fn extracts_texture_from_relative_filename() {
        let mut scene = Scene3D::new();
        let model_to_mesh: HashMap<i64, MeshId> = HashMap::new();
        let model_nodes: HashMap<i64, NodeId> = HashMap::new();

        let doc = make_doc(
            vec![texture_elem(400, "BaseColor", Some("textures/wood.png"))],
            vec![],
        );
        extract_materials(&doc, &mut scene, &model_to_mesh, &model_nodes);

        assert_eq!(scene.textures.len(), 1);
        assert_eq!(scene.textures[0].name.as_deref(), Some("BaseColor"));
        match &scene.textures[0].image {
            oxideav_mesh3d::ImageData::External { uri, .. } => {
                assert_eq!(uri, "textures/wood.png");
            }
            other => panic!("expected External image, got {other:?}"),
        }
    }

    #[test]
    fn binds_diffuse_texture_to_base_color() {
        let (mut scene, mesh_id) = seed_scene_with_mesh();
        let mut model_to_mesh: HashMap<i64, MeshId> = HashMap::new();
        model_to_mesh.insert(200, mesh_id);
        let model_nodes: HashMap<i64, NodeId> = HashMap::new();

        // Texture 400 -> Material 300 ("DiffuseColor"); Material 300 -> Model 200.
        let doc = make_doc(
            vec![
                material_elem(300, "Wood"),
                texture_elem(400, "WoodColor", Some("textures/wood.png")),
            ],
            vec![conn_op(400, 300, "DiffuseColor"), conn_oo(300, 200)],
        );
        extract_materials(&doc, &mut scene, &model_to_mesh, &model_nodes);

        assert_eq!(scene.materials.len(), 1);
        let mat = &scene.materials[0];
        let texref = mat
            .base_color_texture
            .expect("base_color_texture wired from DiffuseColor OP");
        assert_eq!(texref.texture.0, 0);
        assert_eq!(texref.uv_set, 0);
    }

    #[test]
    fn embeds_video_content_into_texture() {
        let mut scene = Scene3D::new();
        let model_to_mesh: HashMap<i64, MeshId> = HashMap::new();
        let model_nodes: HashMap<i64, NodeId> = HashMap::new();

        // Texture 400 (no RelativeFilename) backed by Video 500 with a
        // tiny PNG-shaped embedded blob.
        let png_magic = [0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A];
        let mut content = png_magic.to_vec();
        content.extend_from_slice(b"<chunks elided>");
        let doc = make_doc(
            vec![
                texture_elem(400, "EmbeddedColor", None),
                video_elem(500, "WoodVideo", Some("wood.png"), Some(&content)),
            ],
            vec![conn_oo(500, 400)],
        );
        extract_materials(&doc, &mut scene, &model_to_mesh, &model_nodes);

        assert_eq!(scene.textures.len(), 1);
        match &scene.textures[0].image {
            oxideav_mesh3d::ImageData::Source(src) => {
                assert_eq!(src.mime(), Some("image/png"));
                assert_eq!(src.size_hint(), Some(content.len() as u64));
            }
            other => panic!("expected Source image, got {other:?}"),
        }
    }

    #[test]
    fn ignores_unknown_op_binding_names() {
        let (mut scene, _mesh_id) = seed_scene_with_mesh();
        let model_to_mesh: HashMap<i64, MeshId> = HashMap::new();
        let model_nodes: HashMap<i64, NodeId> = HashMap::new();

        // Bind a texture to a material via an exotic property name —
        // it should round-trip without panicking and without setting
        // any typed slot.
        let doc = make_doc(
            vec![
                material_elem(300, "Mat"),
                texture_elem(400, "Tex", Some("foo.png")),
            ],
            vec![conn_op(400, 300, "SomeFutureChannel")],
        );
        extract_materials(&doc, &mut scene, &model_to_mesh, &model_nodes);
        assert_eq!(scene.materials.len(), 1);
        assert!(scene.materials[0].base_color_texture.is_none());
        assert!(scene.materials[0].normal_texture.is_none());
        assert!(scene.materials[0].emissive_texture.is_none());
    }

    /// Build a `Material` element with a populated `Properties70`
    /// block so the §4 grammar tests can exercise the PBR factor
    /// path end-to-end through `extract_materials`.
    fn material_elem_with_props(id: i64, name: &str, props70: Vec<FbxNode>) -> FbxNode {
        let mut concat = name.as_bytes().to_vec();
        concat.extend_from_slice(b"\x00\x01Material");
        FbxNode {
            name: "Material".into(),
            properties: vec![
                FbxProperty::I64(id),
                FbxProperty::String(concat),
                FbxProperty::String(b"".to_vec()),
            ],
            children: vec![FbxNode {
                name: "Properties70".into(),
                properties: Vec::new(),
                children: props70,
            }],
        }
    }

    /// Build one `P` node from the §4 grammar: `(name, type, label,
    /// flags, value...)`.
    fn p_node(name: &str, ty: &str, label: &str, flags: &str, vals: Vec<FbxProperty>) -> FbxNode {
        let mut props = vec![
            FbxProperty::String(name.as_bytes().to_vec()),
            FbxProperty::String(ty.as_bytes().to_vec()),
            FbxProperty::String(label.as_bytes().to_vec()),
            FbxProperty::String(flags.as_bytes().to_vec()),
        ];
        props.extend(vals);
        FbxNode {
            name: "P".into(),
            properties: props,
            children: Vec::new(),
        }
    }

    #[test]
    fn properties70_diffuse_color_factor_applied_to_base_color() {
        let mut scene = Scene3D::new();
        let model_to_mesh: HashMap<i64, MeshId> = HashMap::new();
        let model_nodes: HashMap<i64, NodeId> = HashMap::new();

        // DiffuseColor (0.8,0.4,0.2) × DiffuseFactor 0.5 → (0.4,0.2,0.1).
        let doc = make_doc(
            vec![material_elem_with_props(
                300,
                "Wood",
                vec![
                    p_node(
                        "DiffuseColor",
                        "Color",
                        "",
                        "A",
                        vec![
                            FbxProperty::F64(0.8),
                            FbxProperty::F64(0.4),
                            FbxProperty::F64(0.2),
                        ],
                    ),
                    p_node(
                        "DiffuseFactor",
                        "Number",
                        "",
                        "A",
                        vec![FbxProperty::F64(0.5)],
                    ),
                ],
            )],
            vec![],
        );
        extract_materials(&doc, &mut scene, &model_to_mesh, &model_nodes);

        let mat = &scene.materials[0];
        let expected = [0.8f32 * 0.5, 0.4 * 0.5, 0.2 * 0.5];
        for (i, exp) in expected.iter().enumerate() {
            assert!(
                (mat.base_color[i] - exp).abs() < 1e-6,
                "base_color[{i}] {} != {}",
                mat.base_color[i],
                exp
            );
        }
    }

    #[test]
    fn properties70_opacity_sets_alpha_and_blend_mode() {
        let mut scene = Scene3D::new();
        let model_to_mesh: HashMap<i64, MeshId> = HashMap::new();
        let model_nodes: HashMap<i64, NodeId> = HashMap::new();
        let doc = make_doc(
            vec![material_elem_with_props(
                300,
                "Glass",
                vec![p_node(
                    "Opacity",
                    "double",
                    "Number",
                    "A",
                    vec![FbxProperty::F64(0.25)],
                )],
            )],
            vec![],
        );
        extract_materials(&doc, &mut scene, &model_to_mesh, &model_nodes);
        let mat = &scene.materials[0];
        assert!((mat.base_color[3] - 0.25).abs() < 1e-6);
        assert!(matches!(mat.alpha_mode, AlphaMode::Blend));
    }

    #[test]
    fn properties70_emissive_color_factor_applied() {
        let mut scene = Scene3D::new();
        let model_to_mesh: HashMap<i64, MeshId> = HashMap::new();
        let model_nodes: HashMap<i64, NodeId> = HashMap::new();
        // EmissiveColor (1,0.5,0.0) × EmissiveFactor 2.0 → (2.0,1.0,0.0).
        let doc = make_doc(
            vec![material_elem_with_props(
                300,
                "Lamp",
                vec![
                    p_node(
                        "EmissiveColor",
                        "Color",
                        "",
                        "A",
                        vec![
                            FbxProperty::F64(1.0),
                            FbxProperty::F64(0.5),
                            FbxProperty::F64(0.0),
                        ],
                    ),
                    p_node(
                        "EmissiveFactor",
                        "Number",
                        "",
                        "A",
                        vec![FbxProperty::F64(2.0)],
                    ),
                ],
            )],
            vec![],
        );
        extract_materials(&doc, &mut scene, &model_to_mesh, &model_nodes);
        let mat = &scene.materials[0];
        let expected = [2.0f32, 1.0, 0.0];
        for (i, exp) in expected.iter().enumerate() {
            assert!(
                (mat.emissive_factor[i] - exp).abs() < 1e-6,
                "emissive_factor[{i}] {} != {}",
                mat.emissive_factor[i],
                exp
            );
        }
    }

    #[test]
    fn properties70_shininess_converts_to_roughness() {
        // Shininess = 0 → roughness = sqrt(2/2) = 1 (fully matte).
        let mut scene = Scene3D::new();
        let model_to_mesh: HashMap<i64, MeshId> = HashMap::new();
        let model_nodes: HashMap<i64, NodeId> = HashMap::new();
        let doc = make_doc(
            vec![material_elem_with_props(
                300,
                "Matte",
                vec![p_node(
                    "Shininess",
                    "double",
                    "",
                    "A",
                    vec![FbxProperty::F64(0.0)],
                )],
            )],
            vec![],
        );
        extract_materials(&doc, &mut scene, &model_to_mesh, &model_nodes);
        assert!((scene.materials[0].roughness - 1.0).abs() < 1e-6);

        // Shininess = 98 → roughness ≈ sqrt(2/100) ≈ 0.1414.
        let mut scene2 = Scene3D::new();
        let doc2 = make_doc(
            vec![material_elem_with_props(
                300,
                "Glossy",
                vec![p_node(
                    "Shininess",
                    "double",
                    "",
                    "A",
                    vec![FbxProperty::F64(98.0)],
                )],
            )],
            vec![],
        );
        extract_materials(&doc2, &mut scene2, &model_to_mesh, &model_nodes);
        let expected = (2.0_f32 / 100.0).sqrt();
        assert!((scene2.materials[0].roughness - expected).abs() < 1e-6);
    }

    #[test]
    fn properties70_reflection_factor_sets_metallic() {
        let mut scene = Scene3D::new();
        let model_to_mesh: HashMap<i64, MeshId> = HashMap::new();
        let model_nodes: HashMap<i64, NodeId> = HashMap::new();
        let doc = make_doc(
            vec![material_elem_with_props(
                300,
                "Chrome",
                vec![p_node(
                    "ReflectionFactor",
                    "Number",
                    "",
                    "A",
                    vec![FbxProperty::F64(0.85)],
                )],
            )],
            vec![],
        );
        extract_materials(&doc, &mut scene, &model_to_mesh, &model_nodes);
        assert!((scene.materials[0].metallic - 0.85).abs() < 1e-6);
    }

    #[test]
    fn shading_model_top_level_leaf_captured_in_extras() {
        // The docs §6 explicitly note `ShadingModel` may live as a
        // direct child leaf (`box.fbx` → `ShadingModel: "phong"`) rather
        // than inside Properties70 — accept either.
        let mut scene = Scene3D::new();
        let model_to_mesh: HashMap<i64, MeshId> = HashMap::new();
        let model_nodes: HashMap<i64, NodeId> = HashMap::new();

        let mat_node = {
            let mut name = b"Phong_Material".to_vec();
            name.extend_from_slice(b"\x00\x01Material");
            FbxNode {
                name: "Material".into(),
                properties: vec![
                    FbxProperty::I64(300),
                    FbxProperty::String(name),
                    FbxProperty::String(b"".to_vec()),
                ],
                children: vec![FbxNode {
                    name: "ShadingModel".into(),
                    properties: vec![FbxProperty::String(b"phong".to_vec())],
                    children: Vec::new(),
                }],
            }
        };
        let doc = make_doc(vec![mat_node], vec![]);
        extract_materials(&doc, &mut scene, &model_to_mesh, &model_nodes);
        let mat = &scene.materials[0];
        assert_eq!(
            mat.extras.get("fbx:shading_model").and_then(|v| v.as_str()),
            Some("phong")
        );
    }

    #[test]
    fn material_without_model_is_still_created() {
        let mut scene = Scene3D::new();
        let model_to_mesh: HashMap<i64, MeshId> = HashMap::new();
        let model_nodes: HashMap<i64, NodeId> = HashMap::new();

        // Orphan material — no OO connection to any Model — must still
        // land in `scene.materials` (consumers can address it via the
        // FbxDocument id table).
        let doc = make_doc(vec![material_elem(300, "Orphan")], vec![]);
        extract_materials(&doc, &mut scene, &model_to_mesh, &model_nodes);
        assert_eq!(scene.materials.len(), 1);
        assert_eq!(scene.materials[0].name.as_deref(), Some("Orphan"));
    }
}
