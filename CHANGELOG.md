# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Round 200 — **ASCII FBX reader** (the headline `oxideav-fbx` "lacks"
  bullet for ~15 rounds).
  - New `ascii` module exposes `is_ascii_fbx(&[u8]) -> bool` (banner
    sniff) and `parse(&[u8]) -> Result<FbxDocument>` (full grammar).
    Produces the **same** typed `FbxDocument` / `FbxNode` /
    `FbxProperty` tree the `binary` reader produces, so every
    downstream consumer (`scene::build_scene` / geometry /
    material / animation / deformer / pose / properties70) handles
    ASCII inputs without further work.
  - `FbxDecoder::decode` now dispatches to either `binary::parse` or
    `ascii::parse` based on the leading bytes (binary `Kaydara FBX
    Binary  \0` magic vs. `; FBX <ver>` banner comment, optionally
    after a UTF-8 BOM). Bytes matching neither return a single
    sniff-failure error rather than the prior wholesale ASCII
    rejection. Grammar source is the observer trace in
    `docs/3d/fbx/fbx-ascii-grammar.md` (#5; observer-derived from
    the staged `docs/3d/fbx/fixtures/cubes-ascii-v7500.fbx`
    fixture; no FBX-implementation source consulted).
  - Coverage:
    - Comments (`;` to end-of-line, full-line + trailing forms).
    - Node-with-body (`Key: <value-list>? { ... }`) and leaf-node
      (`Key: <value-list>`) forms per §3.
    - Object opening lines `UID, "ClassTag::Name", "SubType"` per
      §7c, surfaced as 3-property `[I64, String, String]` — the
      exact shape `crate::scene` reads from the binary side.
    - Typed-array shorthand `Key: *N { a: v1,v2,... }` per §6.
      Element typing: float-shaped tokens (`.` / `e` / `E`)
      promote the whole array to `F64Array`; otherwise the array
      narrows to `I32Array` when every element fits in `i32`
      (matches the binary `i` variant the geometry puller of
      `PolygonVertexIndex` / `UVIndex` / `Materials` requires
      verbatim), or falls back to `I64Array` when any element
      overflows (matches the binary `l` variant the animation
      module's `KeyTime` puller accepts).
    - Scalar value lexing per §5: signed integers, decimal /
      exponent floats, double-quoted strings (backslashes
      preserved literally per §5), bare-letter `T` / `F`
      booleans. `T` / `F` are bare booleans **only** when the
      next byte is not an identifier-continuation character (the
      `TimeMode`-keyword regression is guarded).
    - UTF-8 strings preserved byte-for-byte (the fixture's
      Cyrillic `Model::Куб1` survives the round-trip).
    - `FBXVersion: 7500` inside `FBXHeaderExtension` surfaces as
      `FbxDocument::version`; defaults to `7400` if absent. UTF-8
      BOM at file start is stripped.
  - 15 new unit tests in `src/ascii.rs` cover the grammar's
    minimal shell, object opening lines, typed arrays (floats +
    ints + i32→i64 fall-back + trailing-brace-space + count
    mismatch), bare-boolean disambiguation, `P:`-record decoding
    with `Compound` / scalar / vec3 / backslash-path payloads,
    comment placement, value-then-body node form, and a full
    end-to-end decode of the staged
    `docs/3d/fbx/fixtures/cubes-ascii-v7500.fbx` fixture through
    both the document walker AND `scene::build_scene`. 2 new
    `decoder::tests` add the ASCII-dispatch path + the
    neither-binary-nor-ASCII sniff failure.
  - 3 new integration tests in `tests/ascii_fixture.rs` re-exercise
    the public sniff / decode entry points on the same fixture;
    the legacy `synthetic_quad::ascii_input_returns_unsupported`
    is updated to reflect the new accept-then-validate path.
  - Test count: 71 → 87 unit (+16), 90 → 93 integration (+3).
- Round 194 — multi-UV-set surfacing on `Primitive::uvs`.
  - Every `LayerElementUV` record on a `Geometry` element is now
    surfaced as a separate per-corner `[f32; 2]` buffer on
    `Primitive::uvs` (one entry per FBX UV channel, in document
    order). Mirrors the round-184 multi-channel pattern landed for
    `LayerElementColor` / `Primitive::colors`. Per
    `docs/3d/fbx/ufbx/reference.html` §`ufbx_mesh.uv_sets` /
    §`ufbx_uv_set`, an FBX mesh may carry several UV layers
    (diffuse + lightmap is the canonical pair) and the first set is
    additionally aliased at `ufbx_mesh.vertex_uv`; we surface every
    set without aliasing — `prim.uvs[0]` is the `vertex_uv`-equivalent
    first set and `prim.uvs[1..]` are the additional channels.
  - Decode shape is unchanged from round 1: the existing 2-component
    `pull_layer_vec2` puller honours
    `MappingInformationType ∈ {ByPolygonVertex, ByVertex}` and
    `ReferenceInformationType ∈ {Direct, IndexToDirect}` per
    `docs/3d/fbx/ufbx/elements-meshes.md` §"Attributes" and the
    `LayerElement*` sub-discriminator rules in
    `docs/3d/fbx/fbx-binary-properties70.md` §6. The only delta is
    swapping `.children_named("LayerElementUV").next()` for the
    full iterator.
  - New `tests/synthetic_uv.rs` carries two integration tests:
    1. `single_uv_set_matches_cubes_ascii_fixture` constructs a
       synthetic binary FBX whose `Vertices` / `PolygonVertexIndex`
       / `LayerElementUV` arrays are the same byte-values as the
       first mesh in the staged
       `docs/3d/fbx/fixtures/cubes-ascii-v7500.fbx` ASCII fixture
       (8 unique positions, 24 PVI with bitwise-NOT quad-end
       markers, 14 unique UV pairs, 24 UVIndex slots,
       `ByPolygonVertex` / `IndexToDirect`), decodes it through
       `FbxDecoder`, and asserts the reconstructed UV array has
       the expected 36-corner length (6 quads × 2 triangles × 3),
       that hand-checked spot-values match (corners 0–5 + last)
       and that every emitted UV pair is one of the 14 ground-truth
       values from the fixture.
    2. `two_uv_sets_surface_in_document_order` adds a second
       `LayerElementUV` (layer index 1, all-zero U, arithmetic
       V-ramp, reversed UVIndex) to the same cube and asserts both
       channels populate `prim.uvs[0]` and `prim.uvs[1]` correctly.
  - Test count: 88 → 90 integration (+2); unit unchanged at 71.

## [0.0.2](https://github.com/OxideAV/oxideav-fbx/compare/v0.0.1...v0.0.2) - 2026-05-30

### Other

- Round 191: Properties70 P-record decoder + Material PBR factor decode
- Round 184: vertex colours (LayerElementColor) on Primitive::colors
- Round 178: multi-material slot table via LayerElementMaterial
- drop Scene3D::validate() from pose test (published mesh3d lacks it)
- Round 97: bind-pose (Pose / "BindPose") surfacing on Scene3D
- Round 5: Material / Texture / Video surfacing on Scene3D
- Round 4: opt-in zlib deflate (Encoding == 1) for writer arrays
- Round 3: binary writer (decoder round-trip closure)
- Round 2: animation + deformer surfacing
- release v0.0.1 ([#1](https://github.com/OxideAV/oxideav-fbx/pull/1))

### Added

- Round 191 — `Properties70` `P`-record decoder + Material PBR
  factor decode.
  - New `properties70` module exposes a typed `PropertyMap` decoded
    from the five-field `P`-record grammar staged in
    `docs/3d/fbx/fbx-binary-properties70.md` §4
    (`name`, `typeName`, `label`, `flags`, `value...`). Supports
    `Compound` / scalar (`int` / `enum` / `double` / `Number` /
    `KTime` / `ULongLong` / `KString` / `bool`) / vec3
    (`ColorRGB` / `Color` / `Vector3D` / `Vector` /
    `Lcl Translation` / `Lcl Scaling`) value shapes per the
    `(NumProperties − 4)`-count branch rules in the docs §4 sample.
    Mixed `bool`-typed payloads with `I` / `L` wire codes (older
    FBX-2014 exporters) honour the `typeName` for unambiguous decode.
  - `material::apply_properties70` populates the matching channels on
    each FBX `Material` element's typed `oxideav_mesh3d::Material`:
    `DiffuseColor` × `DiffuseFactor` → `base_color` rgb;
    `Opacity` → `base_color[3]` + `AlphaMode::Blend` when < 1;
    `EmissiveColor` × `EmissiveFactor` → `emissive_factor`;
    `Shininess` / `ShininessExponent` (Blinn-Phong specular exponent)
    → `roughness` via `sqrt(2 / (n + 2))`; `ReflectionFactor` →
    `metallic`; `ShadingModel` (top-level leaf or Properties70
    P-record — docs §6 documents both placements) →
    `Material::extras["fbx:shading_model"]`.
  - 10 new unit tests across `src/properties70.rs` (`decode_scalar_*`,
    `decode_vec3_color`, `decode_kstring`, `decode_ktime_long`,
    `decode_compound_no_value`, `from_element_finds_properties70_child`,
    `missing_properties70_returns_empty_map`,
    `bool_typed_payload_with_int_wire`, `lcl_translation_triple`)
    and `src/material.rs::tests` (`properties70_diffuse_color_factor_applied_to_base_color`,
    `properties70_opacity_sets_alpha_and_blend_mode`,
    `properties70_emissive_color_factor_applied`,
    `properties70_shininess_converts_to_roughness`,
    `properties70_reflection_factor_sets_metallic`,
    `shading_model_top_level_leaf_captured_in_extras`).
    `cargo test -p oxideav-fbx` 71 unit tests + 17 integration tests
    pass.
  - Unblocks the "Material PBR-factor / colour decode" gap that was
    `Lacks`-tailed since round 5. The `Light` / `Camera` NodeAttribute
    gap that depended on the same `Properties70` grammar is now
    decodable but kept for a follow-up round (needs separate
    NodeAttribute element-graph plumbing).

- Round 184 — vertex-colour (`LayerElementColor`) surfacing on
  `Primitive::colors`.
  - New `pull_layer_vec4` puller in `geometry.rs` — the 4-component
    sibling of `pull_layer_vec3` (Normals / Tangents). Reads the
    `Colors` (`d`-array of RGBA quadruples) sub-record + optional
    `ColorIndex` (`i`-array) indirection per
    `docs/3d/fbx/ufbx/elements-meshes.md` §"Attributes". Mapping mode
    `ByPolygonVertex` and `ByVertex` flatten to one `[f32; 4]` per
    triangle corner via the same `Triangulation::corner_pvi_index` /
    `corner_indices` lookup `pull_layer_vec3` uses; reference modes
    `Direct` and `IndexToDirect` are both supported. The on-disk
    record name follows the same ufbx-field → FBX-7.x-PascalCase
    derivation rounds 1–5 used (`vertex_uv` → `LayerElementUV`,
    `vertex_normal` → `LayerElementNormal`, so `vertex_color` →
    `LayerElementColor`).
  - `extract_geometry_mesh_with_corners` walks every
    `LayerElementColor` sub-record in document order and pushes one
    per-corner buffer per layer onto `Primitive::colors`, mirroring
    ufbx's `vertex_color` (first colour set) + `color_sets[1..]`
    exposure pattern. Layers whose mapping mode the puller doesn't
    recognise (`AllSame`, `ByPolygon`, `NoMappingInformation`) skip
    rather than fabricate a misattributed per-corner buffer.
  - Five new unit tests in `src/geometry.rs::tests`:
    `layer_color_by_polygon_vertex_direct_flattens_per_corner`,
    `layer_color_by_vertex_index_to_direct_indirects_through_color_index`,
    `layer_color_rejects_non_multiple_of_four`,
    `layer_color_unknown_mapping_returns_none`,
    `extract_geometry_mesh_surfaces_two_color_sets_in_document_order`.
    One new integration test in
    `tests/synthetic_vertex_color.rs` builds a 2-triangle quad with
    a single `LayerElementColor` sub-record + four RGBA quadruples
    and asserts the per-corner colour buffer reaches
    `Primitive::colors[0]` via `FbxDecoder::decode`, with the
    correct fan-triangulated colour-at-corner assignment.
- Round 178 — multi-material slot table via `LayerElementMaterial`
  surfacing.
  - `geometry.rs` extends `Triangulation` with
    `tri_polygon_index: Vec<u32>` (per-triangle index into the source
    polygon array) and `polygon_count: u32` (negative-end-marker count
    in `PolygonVertexIndex`). Fan triangulation now records which
    source polygon each emitted triangle came from, so per-polygon
    layer payloads can be expanded to per-corner buffers without a
    second pass over the geometry.
  - New `pull_layer_material_slots(layer, &triangles)`: reads the FBX
    `LayerElementMaterial` sub-record per
    `docs/3d/fbx/ufbx/elements-meshes.md` §"Materials" + ufbx
    reference §`ufbx_mesh.face_material`. Supports both
    `MappingInformationType=AllSame` (single broadcast slot — the FBX
    default, also the exporter shorthand of a one-entry `Materials`
    array with no mapping mode header) and
    `MappingInformationType=ByPolygon` (one slot per source polygon,
    expanded to one slot per triangle corner via
    `Triangulation::tri_polygon_index`). Unknown mapping modes
    (`ByVertex` on materials etc.) return `None`, falling through to
    the legacy single-binding wiring per ufbx's "fall back to all-same"
    tolerance.
  - The per-corner slot index buffer lands on
    `Primitive::extras["fbx:face_material_slots"]` as a JSON array of
    `u32`s (length == `corner_indices.len()`); the original mapping
    mode lands on `Primitive::extras["fbx:material_mapping"]` for
    diagnostics.
  - `material.rs` widens the per-Model material wiring to record every
    `Material -> Model` OO connection in slot order: the resulting
    slot table lands on `Primitive::extras["fbx:material_slots"]` as a
    JSON array of `MaterialId.0`s (a key that indexes the same slot
    space as `fbx:face_material_slots`). Single-binding renderers see
    no change — `Primitive::material` still defaults to slot 0, and
    the table is only written when the model carries more than one
    connected material.
  - Six new unit tests in `src/geometry.rs::tests`:
    `triangulation_tracks_polygon_index`,
    `layer_material_all_same_broadcasts_single_slot`,
    `layer_material_by_polygon_per_polygon_payload`,
    `layer_material_by_polygon_length_mismatch_errors`,
    `layer_material_single_entry_treated_as_all_same`,
    `layer_material_unknown_mapping_returns_none`. One new integration
    test `tests/synthetic_multi_material.rs::multi_material_by_polygon_surfaces_slot_table_and_per_face_indices`
    exercises a 2-triangle / 2-material synthetic binary FBX through
    `FbxDecoder::decode`, asserting the slot table, the per-corner
    index buffer, the mapping-mode crumb, and the single-binding
    fallback are all present on the decoded `Primitive`.
- Round 97 — bind-pose (`Pose` element, subtype `"BindPose"`) surfacing
  on `Scene3D`.
  - New `pose` module: `extract_poses(&doc, &mut scene, &model_nodes)`
    walks the top-level `Objects` records for `Pose` elements whose
    subtype (property[2]) is `"BindPose"` and reads each
    `PoseNode { Node : i64 <bone Model id>, Matrix : d[16] }`
    sub-record. `Matrix` is a direct `d`-array (16 doubles, row-major),
    read with the same shape as the deformer module's `Transform` /
    `TransformLink` 4x4 reads — it does **not** live inside a
    `Properties70` `P`-record, so this round stays clear of the
    still-unstaged FBX `P`-record grammar that gates the `material`
    colour-factor decode.
  - Per `docs/3d/fbx/ufbx/reference.html` §`ufbx_pose` /
    §`ufbx_bone_pose`: a bind pose records each bone's world transform
    (`bone_to_world`, *"FBX only stores world transformations"*) and
    sets `is_bind_pose`. The on-disk record name follows the same
    ufbx-field → FBX-7.x-PascalCase derivation rounds 1–4 used for
    `Transform` / `TransformLink` / `Indexes` / `Weights`.
  - Two effects on the decoded `Scene3D`:
    - Each posed bone's world matrix is stashed into the bone
      `Node`'s `extras["fbx:bind_pose"]` (16-element `f64` JSON array,
      row-major), round-tripping the bind pose even for bones that are
      not part of any `Skeleton` (a `Pose` exported without a skin
      deformer).
    - Inverse-bind refinement: a `Skeleton` joint whose cluster did
      **not** carry an explicit `TransformLink` sub-record (the
      deformer module defaults that slot to identity, yielding an
      identity inverse-bind) is back-filled from the bind pose as
      `inverse(bone_to_world)` — the doc's documented *"approximated
      from the parent world transform"* case. Joints that already have
      a real inverse-bind (cluster carried a link matrix) are left
      untouched.
  - Called from `scene::build_scene` after `extract_deformers` so the
    refinement can see the skeletons the deformer module produced.
  - Six new unit tests in `src/pose.rs::tests`: `Matrix` row-major
    read, bind-pose-into-node-extras, non-`"BindPose"` subtype ignored,
    identity-inverse-bind refinement, real-inverse-bind not overwritten,
    no-`Pose`-element no-op. One new integration test in
    `tests/synthetic_pose.rs` exercises the full `FbxDecoder` pipeline
    through a hand-built binary fixture (Geometry + Model + LimbNode +
    Skin + link-less Cluster + `Pose`/`BindPose` + 6 connections),
    verifying the refined inverse-bind, the node-extras stash, and a
    clean `Scene3D::validate()`.
  - **Not surfaced (DOCS-GAP):** Light / Camera `NodeAttribute`
    decode. The on-disk `NodeAttribute` record name, the `"Light"` /
    `"Camera"` subtype discriminators, and every attribute *value*
    (`Color` / `Intensity` / `LightType` for lights; `FieldOfView` /
    `AspectWidth` / `NearPlane` for cameras) live inside
    `Properties70 { P: ... }` records whose grammar is absent from the
    staged `docs/3d/fbx/` corpus (verified: no `NodeAttribute`,
    `Properties70`, `"Light"` / `"Camera"` subtype literal appears in
    any staged doc). `oxideav_mesh3d::Camera` / `Light` + the
    `Node::camera` / `Node::light` slots are ready; blocked pending a
    staged FBX `Properties70` `P`-record grammar.

- Round 5 — Material / Texture / Video surfacing on `Scene3D`.
  - New `material` module: `extract_materials(&doc, &mut scene,
    &model_to_mesh, &model_nodes)` walks the top-level `Objects` records
    for `Material`, `Texture`, and `Video` element types, then walks
    `Connections` for the three documented wiring shapes:
    - `Material -> Model` OO connections assign a surface to a model.
    - `Texture -> Material` OP connections carry the channel name in
      `properties[3]` (`"DiffuseColor"`, `"NormalMap"`, `"EmissiveColor"`,
      plus the Maya/3ds-Max exporter aliases — see
      `docs/3d/fbx/ufbx/reference.html` §`ufbx_material_fbx_map`).
    - `Video -> Texture` OO connections wire embedded media into the
      texture record.
  - One `oxideav_mesh3d::Material` per FBX `Material` element, with its
    `name` field populated. PBR factors (`base_color`, `metallic`,
    `roughness`, `emissive_factor`) stay at the `Material::new()`
    glTF defaults pending a staged FBX `P`-record (Properties70)
    grammar in `docs/3d/fbx/` (deferred — the spec is mentioned but
    not transcribed in the currently-staged Gessler binary doc + ufbx
    site docs).
  - One `oxideav_mesh3d::Texture` per FBX `Texture` element. The
    decoder prefers the embedded `Video.Content` `R`-blob (built via
    `Texture::from_encoded(mime, bytes)` with the MIME inferred from
    `Video.Filename` / `Video.RelativeFilename`), falling back to
    `RelativeFilename` / `FileName` via `Texture::from_uri(uri)` for
    files that reference external assets.
  - `Connections OP Texture -> Material(prop_name)` wires the typed
    `Material::base_color_texture` / `normal_texture` /
    `emissive_texture` / `metallic_roughness_texture` /
    `occlusion_texture` slots when `prop_name` matches one of the
    recognised aliases. Unrecognised channels round-trip via the
    underlying `FbxDocument` but don't surface a typed binding.
  - `Connections OO Material -> Model` sets the first connected
    material on every `Primitive` of the model's mesh
    (`Primitive::material`). Multi-material meshes via
    `LayerElementMaterial` per-face indices are NYI — round 5 ships
    one material per mesh (`face_material` simplification).
  - Six new unit tests in `src/material.rs::tests`: material name +
    primitive binding, external-URI texture decode, DiffuseColor OP
    binding to `base_color_texture`, embedded Video.Content binding
    via `Texture::from_encoded`, unrecognised-OP-name no-op, and
    orphan-material (no Model OO) still surfaces in the materials
    arena. One new integration test in
    `tests/synthetic_material.rs` exercises the full `FbxDecoder`
    pipeline end-to-end through a hand-built binary fixture
    (Geometry + Model + Material + Texture + Video + 5 connection
    records).

- Round 4 — opt-in deflate (`Encoding == 1`) for writer array properties.
  - New `WriterOptions` struct + `write_document_with_options(&doc, &opts)`
    entry point. `WriterOptions::compress_arrays_at(threshold)` switches
    array properties whose raw payload (`ArrayLength * elemSize`) is at
    least `threshold` bytes from the round-3 `Encoding == 0` (raw) form
    to `Encoding == 1` (zlib deflate) per Alexander Gessler / Blender
    Foundation, *FBX Binary File Format Specification* §"Array types"
    (the only two `Encoding` values the doc enumerates).
  - `WriterOptions::compression_level(level)` forwards to
    `miniz_oxide::deflate::compress_to_vec_zlib`'s level argument
    (`0..=10`, default `6` to match zlib's `Z_DEFAULT_COMPRESSION`).
    The encoder writes the post-deflate buffer length into the
    `CompressedLength` field; `ArrayLength` remains the element count
    so the existing parser's "inflated array length mismatch" guard
    still applies.
  - **Never inflates on purpose**: when deflate would produce a buffer
    larger than the raw payload, the writer falls back to
    `Encoding == 0` so the on-disk size cannot regress versus the
    legacy `write_document` path.
  - Default `WriterOptions::default()` keeps `compress_arrays = None`,
    so the existing `write_document` (now a thin
    `write_document_with_options(doc, &WriterOptions::default())`
    wrapper) produces byte-identical output to round 3. The
    `parser_output_writes_back_unchanged` regression test still passes
    bit-for-bit.
  - Measured delta on a 32×32 quad-grid fixture (3 072-double
    `Vertices` array + 3 844-int `PolygonVertexIndex` array):
    raw 40 346 bytes → compressed 8 326 bytes (≈ 20.6 % of the raw
    size; `tests/writer_roundtrip.rs::deflate_compressed_grid_round_trips_through_full_decoder`).
    The compressed output is re-decoded through the full `FbxDecoder`
    pipeline and verified to round-trip the document tree + the
    geometric `Scene3D` (mesh count, primitive topology, per-corner
    position count).
  - Six new tests in `src/writer.rs::tests`: opt-in shrink behaviour,
    below-threshold skip, inflate-fallback guard, 64-bit layout under
    compression, and a default-options byte-for-byte regression
    against round 3.

- Round 3 — binary writer (decoder/parser round-trip closure).
  - New `writer` module: `write_document(&FbxDocument) -> Result<Vec<u8>>`
    emits the 27-byte header + recursive Node Records + final
    NULL-record sentinel per Alexander Gessler / Blender Foundation,
    *FBX Binary File Format Specification* (`docs/3d/fbx/blender-fbx-binary-format.html`).
    All property type codes — scalars `Y C I F D L`, arrays
    `f d l i b`, specials `S R` — are written; the 32-bit (pre-7500)
    vs 64-bit (≥ 7500) Node Record layout is auto-selected from
    `FbxDocument::version`. Arrays use `Encoding == 0` (raw) for
    byte-determinism (the Gessler doc allows both forms; readers
    accept either).
  - Round-trip closure: `binary::parse` + `writer::write_document` is
    deterministic and self-inverse on every `FbxDocument` the parser
    produces. Verified by `tests/writer_roundtrip.rs`: a hand-built
    `FbxDocument` mirroring the synthetic-quad fixture serialises +
    re-decodes to an equal scene at both layout widths, and a
    parser-output → writer → parser → writer chain yields the
    identical byte buffer twice.
  - **No Autodesk footer is emitted.** The Gessler doc records the
    bytes after the top-level NULL-record as *"a footer with unknown
    contents"*; our parser already tolerates files that end at EOF,
    so files this writer produces round-trip through our own pipeline
    but may be flagged by SDKs that validate the trailer signature.
  - Scene3D-level `Mesh3DEncoder` impl (the inverse of
    `scene::build_scene`) remains NYI; this round only ships the
    lower-level `FbxDocument` → bytes serialiser.
  - **ASCII FBX remains NYI** — and unblockable on the current docs
    corpus. The staged `docs/3d/fbx/README.md` §"What's covered (and
    what isn't)" explicitly records that no ASCII grammar reference
    is mirrored (Blender's writeup is binary-only; Kaydara's
    original FBX 6.x ASCII documentation is no longer on the public
    web). Implementing ASCII FBX without re-deriving the grammar
    from ufbx C source / Blender's GPL `io_scene_fbx` add-on would
    violate the project's clean-room policy.

- Round 2 — animation + deformer surfacing.
  - `AnimationStack` / `AnimationLayer` / `AnimationCurveNode` /
    `AnimationCurve` walk in the new `animation` module produces one
    `oxideav_mesh3d::Animation` per stack. Curves on `Lcl Translation`,
    `Lcl Rotation`, `Lcl Scaling` (default XYZ Euler order, degrees,
    Hamilton-product Euler→quaternion conversion) and morph
    `DeformPercent` are surfaced as typed `AnimationChannel`s. Per-axis
    component curves (`d|X` / `d|Y` / `d|Z`) are merged onto a unified
    keyframe grid with linear interpolation; `KeyTime` is converted
    from FBX KTime ticks (`46_186_158_000` ticks/second) to seconds.
    Per-layer compositing weights, `KeyAttrFlags` interpolation flags,
    and pivot/PreRotation/PostRotation chains stay NYI per the doc's
    `ufbx_evaluate_scene()` notes.
  - `Deformer` walk in the new `deformer` module:
    - `Deformer{Skin}` + `Deformer{Cluster}` produce one
      `oxideav_mesh3d::Skeleton` + `oxideav_mesh3d::Skin` per skin
      deformer; per-cluster `TransformLink` / `Transform` matrices are
      composed (`inverse(TransformLink) * Transform`) into the
      skeleton's per-joint inverse-bind matrix; `Indexes` / `Weights`
      are expanded onto the per-corner `Primitive::joints` /
      `Primitive::weights` buffers (top-4 weights per corner, sum-1
      normalised). Skinning method (`SKINNING_METHOD_*`) not surfaced
      — every skin produces LBS-compatible buffers.
    - `Deformer{BlendShape}` + `Deformer{BlendShapeChannel}` +
      `Geometry{Shape}` produce one `oxideav_mesh3d::MorphTarget` per
      channel (taking the most-recent `Shape` per the doc's
      `target_shape` simplification — in-between keyframes ignored).
      Sparse `Indexes` / `Vertices` / `Normals` deltas expand to
      per-corner `MorphTarget::position` / `MorphTarget::normal`.
      `Mesh::weights` is grown one slot per channel, default `0.0`.
  - `geometry::extract_geometry_mesh_with_corners` returns the
    per-corner shared-vertex index buffer alongside the `Mesh` so the
    deformer module can map per-shared-vertex skin / morph payloads
    onto the per-corner `Primitive` layout. Original
    `extract_geometry_mesh` retained as a thin wrapper.

## [0.0.1](https://github.com/OxideAV/oxideav-fbx/releases/tag/v0.0.1) - 2026-05-11

### Other

- Initial commit: oxideav-fbx round 1 (binary container reader + decoder)

### Round 1 details

- Round 1 — initial bootstrap.
  - Binary FBX container reader: 27-byte header parse (Kaydara magic +
    `0x1A 0x00` + version `u32`), recursive Node Record walker with
    pre-7500 (32-bit `EndOffset` / `NumProperties` / `PropertyListLen`)
    and post-7500 (64-bit) layouts auto-selected by the version byte,
    full property type-code dispatcher for primitives (`Y` `C` `I` `F`
    `D` `L`), arrays (`f` `d` `l` `i` `b`) including the
    `ArrayLength` / `Encoding` / `CompressedLength` shape with zlib
    (deflate) decompression of `Encoding == 1`, and special
    string/binary types (`S` `R`).
  - `Mesh3DDecoder` trait impl that walks `Objects { Geometry … }` +
    `Connections { C: "OO", child, parent … }` to produce a `Scene3D`:
    one `Mesh` per `Geometry` element, root-level `Node` per `Model` of
    subtype `Mesh` connected to the geometry, with the polygon-vertex
    array re-indexed into per-vertex glTF-style positions. Negative
    "polygon end marker" indices in `PolygonVertexIndex` are decoded
    per the binary format's two's-complement-bitwise-NOT convention.
  - Per-vertex normals lifted from the first `LayerElementNormal`
    sub-record when its `MappingInformationType` is one of
    `ByPolygonVertex` / `ByVertex` (with optional `IndexToDirect`
    indirection); other mapping modes pass through unmodified for now.
  - ASCII FBX is **explicitly NYI** — input that does not start with
    the binary magic returns `Error::Unsupported("ASCII FBX is not yet
    supported")`. ASCII grammar is documented in the staged
    `docs/3d/fbx/blender-fbx-binary-format.html` text-based-format
    section but not implemented in r1.
  - Encoder is **explicitly NYI** — followup round.
  - Skin / Cluster (deformer) wiring, AnimationStack / Layer / Curve,
    and BlendShape / BlendShapeChannel are all NYI in r1.
  - `register(&mut Mesh3DRegistry)` entry point under the default
    `registry` feature wires the decoder into the framework registry
    under format id `"fbx"` with extension `"fbx"`.
  - Standalone build path (`--no-default-features`) drops the
    `oxideav-core` dependency entirely; the decoder API + trait impl
    stay available through `oxideav-mesh3d`'s own standalone feature
    set.
