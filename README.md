# oxideav-fbx

[![CI](https://github.com/OxideAV/oxideav-fbx/actions/workflows/ci.yml/badge.svg)](https://github.com/OxideAV/oxideav-fbx/actions/workflows/ci.yml) [![crates.io](https://img.shields.io/crates/v/oxideav-fbx.svg)](https://crates.io/crates/oxideav-fbx) [![docs.rs](https://docs.rs/oxideav-fbx/badge.svg)](https://docs.rs/oxideav-fbx) [![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

Pure-Rust FBX (Filmbox) mesh **decoder + encoder** (binary + ASCII).

FBX is Autodesk's proprietary 3D scene-and-asset interchange format,
originally developed by Kaydara for MotionBuilder. There is no
Autodesk-published prose specification — this crate is implemented
clean-room from third-party documentation:

- **Binary container** — Blender Foundation, *FBX Binary File Format
  Specification* (August 2013, public-domain dedication). Staged at
  `docs/3d/fbx/blender-fbx-binary-format.html`.
- **Node-record / Properties70 / object-graph semantics** — clean-room
  observer trace `docs/3d/fbx/fbx-binary-properties70.md`, sample-RE'd
  from the staged fixture bytes (no FBX-implementation source read).
- **ASCII FBX grammar** — clean-room observer grammar
  `docs/3d/fbx/fbx-ascii-grammar.md`.

## What's covered

- Binary container reader: 27-byte header, recursive Node Record
  walker (32-bit pre-7500, 64-bit ≥ 7500), full property type-code
  dispatch (`Y` `C` `I` `F` `D` `L` scalars, `f` `d` `l` `i` `b` `c`
  arrays — `c` is the raw-byte array of the documented type-code
  alphabet (`docs/3d/fbx/README.md`), its 1-byte element width pinned
  from the staged box-binary-v7500.fbx thumbnail `ImageData` record —
  incl. zlib-deflated via `compcol` — the inflate path is
  bounded at the array's known post-inflate size so a hostile
  `CompressedLength` cannot expand into a decompression bomb — `S` /
  `R` strings & blobs). The `C` boolean wire byte follows the
  fixture-observed SDK form: the ASCII `T` token byte (0x54) is true
  (the LSB-only reading would misdecode it), `F` / plain 0x00/0x01
  decode via the LSB.
- **Binary footer** — the trailing block the public binary writeup
  leaves as *"unknown contents"* is observer-derived from the staged
  `box-binary-v7400.fbx` fixture's final 176 bytes: top-level NULL
  record, a 16-byte per-file id (opaque — its derivation is
  undocumented by every staged source), zero padding to the next
  16-byte file-offset boundary, 4 zero bytes, a uint32 LE echo of the
  header version, 120 zero bytes, and a constant 16-byte trailer
  signature ending exactly at EOF. `binary::parse_footer` decodes it
  tolerantly (`Option<FbxFooter>`; malformed/absent tails are `None`);
  the writer emits the full block by default (`WriterOptions::
  footer_id` / `emit_footer` knobs); the decoder surfaces the id on
  `Scene3D::extras["fbx:footer_id"]` and the encoder threads it back.
  The whole 17200-byte staged fixture re-encodes **byte-for-byte**
  through `parse` + `parse_footer` + `write_document_with_options`
  (which also pins the `Encoding == 0` `CompressedLength` = raw byte
  length rule and the explicit empty-nested-list form the fixture's
  `References` record demonstrates).
- Object-graph walker: indexes `Geometry` and `Model` from `Objects`,
  walks `Connections` `OO` records to wire Geometry → Model and
  Model → root.
- **Node local transforms — the full FBX node-transform chain** —
  per `docs/3d/fbx/fbx-node-transform-chain.md` §1, each `Model`'s
  local matrix is the product `T · Roff · Rp · Rpre · R · Rpost⁻¹ ·
  Rp⁻¹ · Soff · Sp · S · Sp⁻¹` (`Lcl Translation` / `RotationOffset`
  / `RotationPivot` / `PreRotation` / `Lcl Rotation` (inverse of
  `PostRotation`) / `ScalingOffset` / `ScalingPivot` /
  `Lcl Scaling`), resolved against the `ObjectType: "Model"`
  `PropertyTemplate` defaults. The chain composes **exactly** into
  the node's `Transform::Trs` via the closed form
  `t = T + Roff + Rp + Q·(Soff + Sp − Rp − S∘Sp)`,
  `Q = Rpre · R(order) · Rpost⁻¹`, `s = S` (no matrix decomposition;
  pinned in-tree against the literal 11-factor matrix product for
  all seven rotation orders). `RotationOrder` follows the doc §3
  table — order `ABC` applies `A` first (`R = R_C · R_B · R_A`);
  `0` = XYZ default, `6` (`SphericXYZ`) builds its rest matrix as
  XYZ while the raw enum stays recoverable on
  `extras["fbx:rotation_order"]`. When any chain extension is
  non-trivial the raw authored components also surface on
  `Node::extras` (`fbx:lcl_*` / `fbx:rotation_offset` /
  `fbx:rotation_pivot` / `fbx:pre_rotation` / `fbx:post_rotation` /
  `fbx:scaling_offset` / `fbx:scaling_pivot`) so the encoder
  re-emits the authored chain verbatim. The doc §2 **geometric
  transform** (`GeometricTranslation` / `GeometricRotation` /
  `GeometricScaling`) is a post-multiplied, **non-inheriting**
  mesh-only offset — never composed into `Node::transform` (children
  must not inherit it); it surfaces on `extras["fbx:geometric_*"]`
  and `node_transform::geometric_transform` rebuilds the
  `OT · OR · OS` product to post-multiply onto the node's world
  matrix for that node's own mesh. A non-default `InheritType`
  surfaces raw on `extras["fbx:inherit_type"]`, and the **doc §4
  composition products are implemented** by the `inherit` module:
  `inherit::world_transforms(&scene)` walks a decoded scene applying
  the three documented parent-scale propagation rules per node
  (`0` `RrSs` = `P_R·L_R·P_S·L_S`, `1` `RSrs` = `P_R·P_S·L_R·L_S` —
  exactly naive matrix concatenation, the value ordinary Maya exports
  carry — and `2` `Rrs` = `P_R·L_R·(P_S·p_s⁻¹)·L_S`), with the
  translation riding the parent's full world matrix in all three
  modes and chain-bearing nodes recomposed from their authored
  `fbx:lcl_*` extras at f64 precision. Only a
  `RotationOrder` enum int outside the documented `0..=6` table
  leaves a node at identity, with
  `extras["fbx:transform_incomplete"] = "rotation_order_unrecognized"`.
- Mesh extraction: `Vertices` + `PolygonVertexIndex` →
  per-corner `Primitive(Topology::Triangles)` (ngons fan-triangulated;
  end-of-polygon negatives bit-NOT decoded). `LayerElementNormal` /
  `LayerElementUV` / `LayerElementColor` / `LayerElementTangent` /
  `LayerElementBinormal` flattened for every `MappingInformationType`
  this crate resolves — `ByPolygonVertex`, `ByVertex` (`ByVertice`),
  `ByPolygon` (per-source-polygon flat attributes), and `AllSame`
  (one value broadcast to the whole mesh) — under both `Direct` and
  `IndexToDirect` `ReferenceInformationType` (a single shared
  `resolve_layer_indices` helper backs the scalar/vec2/vec3/vec4
  pullers; an `IndexToDirect` layer with no index sub-record — the
  shape the staged binary fixture's SDK-written `LayerElementColor`
  demonstrates — resolves as identity indexing, i.e. like `Direct`). `ByEdge` on the generic attribute pullers surfaces no
  per-corner buffer rather than mis-attribute (the smoothing layer,
  which owns that mode, is handled separately — see the Edges /
  smoothing bullet below). Each layer's
  `MappingInformationType` / `ReferenceInformationType` resolved
  independently. A `Geometry` carrying **more than one**
  `LayerElementNormal` (distinguished by its `Layer` / `TypedIndex`
  integer per `docs/3d/fbx/fbx-binary-properties70.md` §6.4) surfaces
  the first as the canonical `Primitive::normals` and the rest on
  `Primitive::extras["fbx:extra_normals"]` (one flattened per-corner
  buffer each, with `fbx:extra_normals_typed_index` /
  `fbx:extra_normals_mapping` metadata).
- Animation: `AnimationStack` / `AnimationLayer` /
  `AnimationCurveNode` / `AnimationCurve` → one
  `oxideav_mesh3d::Animation` per stack. `Lcl Translation` /
  `Lcl Rotation` (XYZ-Euler-degrees → quaternion) /
  `Lcl Scaling` (Vec3) and morph-target `DeformPercent` (Scalar)
  channels supported; component curves merged onto a unified linear
  grid; `KeyTime` ticks divided by the well-known FBX KTime constant.
  **Multi-target morph animation** merges every `BlendShapeChannel`'s
  `DeformPercent` curve on a node into ONE `MorphWeights` channel
  strided by the mesh's morph-target count (the `oxideav_mesh3d`
  sampler contract), union keyframe grid across channels, unanimated
  slots holding their static rest weight; wire percentages (0..100)
  scale to mesh3d's 0..1 §3.7.2.2 blend factors.
- Deformers: `Deformer{Skin}` + `Deformer{Cluster}` →
  `oxideav_mesh3d::Skeleton` + `Skin` (per-corner top-4 joints +
  weights, normalised; inverse-bind = `inverse(TransformLink) * Transform`).
  `Deformer{BlendShape}` + `BlendShapeChannel` + `Geometry{Shape}`
  → `MorphTarget` per channel (sparse `Indexes` deltas expanded to
  per-corner buffers), deformers walked in document order so
  morph-target slots are deterministic across multiple `BlendShape`
  deformers on one geometry. Each channel's **static**
  `DeformPercent` record (0..100) decodes ÷100 into the matching
  `Mesh::weights` slot (rest blend state), and the channels' authored
  display names land in slot order on the typed `Mesh::target_names`
  (`find_target("Smile")` → weight-slot index). The merged
  `MorphWeights` channel is built through
  `AnimationSampler::morph_weights` (the typed synthesis path), so
  `morph_weight_frames()` reads the per-key weight vectors back
  losslessly. `MorphTarget::inbetweens` stays empty — see the
  in-between note under "Notes & limitations".
- **Materials / Textures / Video**
  — one `oxideav_mesh3d::Material` per FBX `Material` element with
  PBR factors decoded from `Properties70` `P`-records per
  `docs/3d/fbx/fbx-binary-properties70.md` §4: `DiffuseColor` ×
  `DiffuseFactor` → `base_color` rgb, `Opacity` → `base_color[3]` +
  `AlphaMode::Blend` (< 1), `EmissiveColor` × `EmissiveFactor` →
  `emissive_factor`, `Shininess` → `roughness` via
  `sqrt(2 / (n + 2))`, `ReflectionFactor` → `metallic`,
  `ShadingModel` → `Material::extras["fbx:shading_model"]`. One
  `oxideav_mesh3d::Texture` per `Texture` element (embedded
  `Video.Content` via `Texture::from_encoded(mime, bytes)` preferred
  over `RelativeFilename` / `FileName` via `Texture::from_uri`).
  `Connections` walks wire `Texture -> Material` OP records
  (`DiffuseColor` / `NormalMap` / `EmissiveColor` plus Maya / 3ds-Max
  aliases) into typed `base_color_texture` / `normal_texture` /
  `emissive_texture` / `metallic_roughness_texture` /
  `occlusion_texture` slots; `Material -> Model` OO records set
  `Primitive::material` on the bound mesh. Each `Texture` element's
  `Properties70` is resolved against the staged `FbxFileTexture`
  template (`docs/3d/fbx/fbx-property-templates.md` §3.1), and the
  typed reference surfaces land both ways:
  - **`UVSet` → `TextureRef::uv_set`** — the effective `UVSet`
    KString joins against the bound meshes'
    `Primitive::extras["fbx:uv_set_names"]` channel labels (recorded
    from each `LayerElementUV` `Name` leaf in channel order) to pick
    the typed UV-channel index; the fixture's
    `UVSet = "UVChannel_1"` resolves to channel 0. The encoder emits
    the matching `UVSet` record + `Name` leaves (authored labels
    verbatim, `map{k+1}` synthesized for unnamed channels), closing
    the round trip for non-zero sets.
  - **`Translation` / `Rotation` / `Scaling` →
    `TextureRef::transform`** (typed `KHR_texture_transform`-style
    placement), literal reading: offset = translation x/y, rotation
    = the third component degrees → radians, scale = scaling x/y.
    Template defaults are the identity, so authored-vs-absent equals
    own-record presence (`None` = "no transform declared"; an
    authored identity stays `Some(IDENTITY)`). Typed only when the
    placement is purely 2D and pivot-free; non-zero
    `TextureRotationPivot` / `TextureScalingPivot` (composition
    order unstaged) or `UVSwap` keep the placement raw-only.
  - **Raw untypable records** (`WrapModeU` / `WrapModeV` — the
    wrap-enum value table beyond the observed default `0` is a
    staged-docs gap, so the typed `Sampler` keeps its default
    repeat / filters-undefined state — plus `UVSwap`, `UseMipMap`,
    `TextureTypeUse` / `CurrentMappingType` /
    `CurrentTextureBlendMode` / `PremultiplyAlpha` / `UseMaterial` /
    `Texture alpha`, pivots, and unrepresentable placements)
    round-trip verbatim via
    `Scene3D::extras["fbx:texture_records"]` (keyed by scene texture
    index). The staged fixture's authored
    `CurrentTextureBlendMode = 0` / `UseMaterial = 1` records are
    pinned test-side.
- **Vertex colours** — every `LayerElementColor` sub-record
  on a `Geometry` element is surfaced as a separate per-corner RGBA
  buffer on `Primitive::colors` (one slot per FBX colour set, the
  primary set first then the additional sets). Mapping / reference
  handling matches Normals (`ByPolygonVertex` / `ByVertex` with
  optional `IndexToDirect` indirection); the `d`-array `Colors`
  payload is 4-component RGBA.
- **Multi-UV-set surfacing** — every `LayerElementUV`
  sub-record on a `Geometry` element is now surfaced as a separate
  per-corner `[f32; 2]` buffer on `Primitive::uvs` (one entry per
  FBX UV channel, in document order). An FBX mesh may carry multiple
  UV channels (the canonical diffuse + lightmap pair), each a
  `LayerElementUV` record; the first set is the primary UV channel.
  Mapping / reference handling reuses the
  2-component puller, so `ByPolygonVertex` / `ByVertex` and
  `Direct` / `IndexToDirect` work for every channel. Round-trip
  tested against `docs/3d/fbx/fixtures/cubes-ascii-v7500.fbx`
  ground-truth UV / UVIndex arrays + a two-UV-set synthetic.
- **Tangents / Binormals** — `docs/3d/fbx/fbx-binary-properties70.md`
  §6 point 4 enumerates `LayerElementTangent` / `LayerElementBinormal`
  as `Geometry` LayerElement sub-discriminators alongside Normal / UV /
  Color / Material (the `docs/3d/fbx/fbx-ascii-grammar.md` §7c worked
  example + the staged `cubes-ascii-v7500.fbx` fixture carry both). The
  first `LayerElementTangent` populates the canonical
  `Primitive::tangents` slot glTF-style (`[x,y,z,w]` — xyz from the
  `Tangents` 3-component `d`-array, `w` handedness from the companion
  per-corner `TangentsW` sign array when present, else `+1.0`); extra
  tangent layers (distinguished by their `Layer` / `TypedIndex` integer
  per §6 point 4) ride on `Primitive::extras["fbx:extra_tangents"]`
  with `fbx:extra_tangents_typed_index` / `fbx:extra_tangents_mapping`
  metadata. `oxideav_mesh3d` has no first-class binormal slot (the
  bitangent reconstructs from the tangent `w` sign as `B = w·(N×T)`),
  so every `LayerElementBinormal` surfaces on
  `Primitive::extras["fbx:binormals"]` (xyz + `BinormalsW` sign) with a
  `fbx:binormals_mapping` companion, keeping the explicitly-authored
  binormal payload recoverable. Mapping / reference handling
  (`ByPolygonVertex` / `ByVertex` + optional `IndexToDirect`) reuses the
  shared puller.
- **Multi-material slot table** — `LayerElementMaterial`
  per-polygon slot indices (`MappingInformationType=ByPolygon`) +
  every `Material -> Model` OO connection in slot order land on
  `Primitive::extras` (`fbx:face_material_slots` / `fbx:material_slots` /
  `fbx:material_mapping`), preserving the full per-face material
  payload alongside the legacy single-binding `Primitive::material`
  (slot 0).
- **Edges + smoothing (`LayerElementSmoothing`)** — per
  `docs/3d/fbx/fbx-edges-smoothing-layer.md` (ask #220). The
  `Geometry`-level `Edges` array (each value a `PolygonVertexIndex`
  slot naming a unique edge's start corner; the second endpoint is
  the next corner within the same polygon, wrapping at the negative
  closing corner) decodes to undirected shared-vertex pairs on
  `Primitive::extras["fbx:edges"]`. `LayerElementSmoothing` branches
  on its mapping mode: `ByEdge` (one hard/soft flag per unique edge,
  `0` = hard) surfaces raw flags on `fbx:edge_smoothing` and a
  per-corner resolution on `fbx:smoothing` (each corner carries the
  flag of the polygon edge starting at its slot, matched by
  undirected pair); `ByPolygon` (one smoothing-group bitmask per
  polygon, adjacent faces smooth iff `mask_a & mask_b != 0`)
  broadcasts per corner. `fbx:smoothing_mapping` records the source
  form (the same values mean different things in the two modes). A
  `ByEdge` layer without an `Edges` array binds nothing (no edge
  domain); length mismatches error. The writer re-emits both
  (identity `Edges` enumeration over its per-corner layout +
  `ByEdge`/`Direct` or per-triangle `ByPolygon` smoothing), so
  per-corner smoothing survives decode→encode→decode in binary and
  ASCII forms; the decode is pinned to the doc's §2 hand-worked cube
  table on the staged `cubes-ascii-v7500.fbx` fixture.
- **GlobalSettings** — the top-level `GlobalSettings`
  node's `Properties70` block is decoded via the
  `PropertyMap`; every well-known `P`-record from the
  cubes-ascii-v7500.fbx fixture (`UpAxis` / `UpAxisSign` / `FrontAxis`
  / `FrontAxisSign` / `CoordAxis` / `CoordAxisSign` /
  `OriginalUpAxis*` / `UnitScaleFactor` / `OriginalUnitScaleFactor` /
  `AmbientColor` / `DefaultCamera` / `TimeMode` / `TimeProtocol` /
  `SnapOnFrameMode` / `TimeSpanStart` / `TimeSpanStop` /
  `CustomFrameRate` / `CurrentTimeMarker`) lands on `Scene3D::extras`
  under the `"fbx:<snake_case>"` key convention. `UnitScaleFactor` is
  additionally translated to `Scene3D::unit`: `100.0` →
  `Unit::Centimetres` and `1.0` → `Unit::Metres` (the two canonical
  values — centimetres is the de-facto FBX default and `1.0` denotes
  metre units). Other
  `UnitScaleFactor` values surface the raw factor on
  `extras["fbx:unit_scale_factor"]` without claiming a typed
  `Unit` mapping the docs don't provide. **Axis convention is typed
  both ways**: per the `docs/3d/fbx/fbx-node-transform-chain.md` §4a
  integer table (pinned from staged fixture bytes — `0 = X`, `1 = Y`,
  `2 = Z`, signs as separate `±1` ints), the `UpAxis` / `FrontAxis`
  pairs decode onto `Scene3D::up_axis` / `front_axis` (FBX `FrontAxis`
  semantics kept literal — the axis pointing towards the viewer, so a
  Maya export decodes `front_axis = PosZ`); the encoder synthesises
  the six records from the typed fields for fresh scenes (`CoordAxis`
  = the remaining index, `OriginalUpAxis` = the `−1` not-recorded
  sentinel) while round-tripped `fbx:*_axis*` extras re-emit verbatim.
  Out-of-table ints stay honest: raw on `extras`, typed fields at the
  `Scene3D::new` defaults. The §4a structural fact — the triple
  declares three *distinct* axes exhausting `{0, 1, 2}` — is enforced
  as a coherence guard: `UpAxis == FrontAxis` leaves both typed
  fields at their defaults with
  `extras["fbx:axis_convention_inconsistent"] = "up_front_equal"`;
  a `CoordAxis` colliding with a self-consistent up/front pair keeps
  up/front typed and surfaces `"coord_axis_collision"`. The guard is
  pinned silent on all seven staged fixtures (whose triples are the
  coherent Maya `1 / 2 / 0`).
- **`Definitions` / `PropertyTemplate` decoding + template-default
  resolution** — the top-level `Definitions` section (per
  `docs/3d/fbx/fbx-ascii-grammar.md` §7b: *"`Count` at the top is the
  total object count; each `ObjectType:` block names a class, its
  instance `Count`, and a `PropertyTemplate` holding the default
  `Properties70` for that class"*) decodes via the new `definitions`
  module into a typed `Definitions` value — section `Version` /
  `total_count` plus one `ObjectTypeDefinition` per class (class
  name, instance count, template name, default property set as a
  the `PropertyMap`). Classes without a template block (the
  fixture's `GlobalSettings`) surface count-only. The binary encoding
  renders the identical node tree (docs `fbx-binary-properties70.md`
  §4 isomorphism note) so one walker covers both front-ends. The
  companion `PropertyMap::with_template_defaults` resolves an
  object's *effective* properties (own records overlay class
  defaults), and material decode now applies it against the
  `ObjectType: "Material"` template — exporter-omitted class defaults
  (the cubes fixture's FbxSurfaceLambert `DiffuseFactor = 1`) decode
  the same as explicitly-written records, with `ShadingModel`
  precedence own P-record > direct-child leaf > template default.
  The scene builder's no-content fallback no longer discards a
  populated materials / textures arena when a document carries no
  meshes or nodes.
- **`Takes` section** — the top-level `Takes` node (per
  `docs/3d/fbx/fbx-ascii-grammar.md` §7e — the last of the §7 ordered
  sections) catalogues the file's animation *takes*: a `Current` leaf
  naming the active take plus one `Take : "<name>" { FileName,
  LocalTime, ReferenceTime }` node-with-body per take, where
  `LocalTime` / `ReferenceTime` are each the §5 two-integer
  `start,stop` KTime pair. The new `takes` module decodes them onto
  `Scene3D::extras` — `extras["fbx:current_take"]` (the active-take
  name) and `extras["fbx:takes"]` (a JSON array of
  `{ name, file_name?, local_time: [start,stop]?,
  reference_time: [start,stop]? }` per take). Because
  `oxideav_mesh3d::Animation` carries no `extras` map (only `name` +
  `channels`), the take time-spans live scene-wide and join back to
  each `Animation` by name: the `Take` name equals the
  `AnimationStack` display name the `animation` module keys each
  `Animation` by (`Take: "Take 001"` ⇔
  `AnimationStack: "AnimStack::Take 001"`). KTime integers stay i64-exact
  (the `KTIME_TICKS_PER_SECOND ≈ 4.6e10` constant is well outside f32
  range — same rationale as `GlobalSettings`' `TimeSpanStart` /
  `TimeSpanStop`). One walker covers both front-ends (the binary form
  renders the identical node tree). `takes_from_extras` /
  `current_take_from_extras` read the catalogue back off a scene.
- **`Documents` + `References` sections** — the two §7 top-level
  sections between `GlobalSettings` and `Definitions`. The
  `documents` module decodes the document catalogue (fixture body:
  `Count` + `Document: <uid>, "", "Scene" { Properties70 {
  SourceObject, ActiveAnimStackName }, RootNode: 0 }`) onto
  `Scene3D::extras["fbx:documents"]` (one
  `{ name, subtype, active_anim_stack? }` per record) and
  `["fbx:active_anim_stack"]` (the first document's stack name — the
  join key equal to the `AnimationStack` display name / the `Takes`
  `Current` name); UIDs are not surfaced (private to the source
  file's object arena). The encoder re-renders the catalogue
  (default: one `"Scene"` document, stack name resolved via
  `fbx:active_anim_stack` → `fbx:current_take` → first animation
  name) plus the observed-empty `References` section, so the full §7
  section set survives decode → encode → decode.
- **Hostile-input hardening + deterministic fuzz sweeps** — both
  front-end readers are total functions (every byte string → `Ok` /
  `Err`, never a panic or abort): bounds-checked `Y` (i16) scalar
  reads, `NumProperties` preallocation clamped by `PropertyListLen`
  and the bytes remaining, and a shared 128-level
  `binary::MAX_NODE_DEPTH` nesting cap in the binary reader and the
  ASCII `parse_node`/`parse_body` recursion (crafted ~14- and
  ~5-byte-per-level depth bombs previously overflowed the stack).
  Locked in by fixed-seed replayable fuzz sweeps in
  `tests/fuzz_mutation.rs`: prefix truncation, byte mutation, chunk
  splice, random-tail-after-valid-magic, and a generative
  write→parse closure over random typed documents.
- **Top-level provenance records** — the v7400-layout top-level
  siblings of `FBXHeaderExtension` observed in the staged binary
  fixture (`FileId` 16-byte `R` blob per
  `docs/3d/fbx/fbx-binary-properties70.md` §3c, `CreationTime` /
  `Creator` string leaves) surface on
  `Scene3D::extras["fbx:file_id"]` (hex) / `["fbx:file_creation_time"]`
  / `["fbx:file_creator"]` and are re-emitted in fixture order, so
  they survive the Scene3D round trip.
- **`FBXHeaderExtension` authoring metadata** — the first top-level §7
  section (per `docs/3d/fbx/fbx-ascii-grammar.md` §7a) carries the
  file's provenance: `Creator`, a `CreationTimeStamp` sub-node
  (`Year`/`Month`/`Day`/`Hour`/`Minute`/`Second`/`Millisecond` integer
  leaves), and a §7c-shaped `SceneInfo` object whose body holds the
  document `MetaData` block (`Title`/`Subject`/`Author`/`Keywords`/
  `Revision`/`Comment`) and a `Properties70` of `Original|*` /
  `LastSaved|*` application provenance. The `header_info` module
  decodes it onto `Scene3D::extras`: `extras["fbx:creator"]`,
  `["fbx:header_version"]`, `["fbx:creation_time"]` (the timestamp
  composed into an `YYYY-MM-DDThh:mm:ss.mmm` string), `["fbx:meta_*"]`
  (one per non-empty `MetaData` field — empty SDK-default fields are
  skipped), and `["fbx:application_name"]` / `["fbx:application_vendor"]`
  / `["fbx:application_version"]` / `["fbx:document_url"]` from the
  `Original|*` creating-application set. Existing extras keys are
  preserved (insert-if-vacant); one walker covers both front-ends.
- **Bind pose** —
  `Objects { Pose : "BindPose" }` elements surface each
  `PoseNode { Node, Matrix }` bone-world matrix onto the bone `Node`'s
  `extras["fbx:bind_pose"]` (16-double row-major JSON array). When a
  `Cluster` omitted its `TransformLink` sub-record (so the deformer
  module defaulted that joint's inverse-bind to identity), the bind
  pose back-fills it as `inverse(bone_to_world)` — the world-only
  case (FBX `Pose` records store world-space matrices). `Matrix` is a
  direct `d`-array sub-record, so this stays clear of the
  still-unstaged `Properties70` `P`-record grammar. Joints that
  already have a real inverse-bind are untouched; non-bind rest poses
  (subtype other than `"BindPose"`) are not promoted. The decoder
  also derives the parent-space form
  `bone_to_parent = inverse(parent_bone_to_world) * bone_to_world` for
  every posed bone whose parent in the scene graph is also posed,
  surfaced as `node.extras["fbx:bind_pose_parent_local"]` (16-double
  row-major JSON array). Root bones whose parent has no bind pose
  receive `bone_to_parent == bone_to_world` (implicit-root convention,
  parent world = identity). The parent-relative form is approximated
  from the parent's stored world transform, since FBX `Pose` records
  hold world-space matrices only. **Round-trip closed**: the encoder
  re-emits one `Pose : "BindPose"` element (a
  `PoseNode { Node, Matrix }` pair per posed node) from the
  `fbx:bind_pose` extras, so bind poses survive
  `decode → encode → decode` in both binary and ASCII forms (the
  derived `fbx:bind_pose_parent_local` entries are recomputed on
  decode rather than serialised).
- **Constraints** — the full `Constraint` object grammar per
  `docs/3d/fbx/fbx-constraint-grammar.md`, both directions.
  Decode surfaces each `Objects { Constraint }` element on
  `Scene3D::extras["fbx:constraints"]`: the `Constraint::<name>`
  header, the human-readable kind display string (written twice —
  header field and inner `Type:` leaf, doc §2), `MultiLayer`, the
  object's **own** `Properties70` records verbatim (space-bearing
  names like `"First Joint"`, the `"Weight"` property-*type* string,
  value-less `"object"` slots — each value kind-tagged JSON so the
  wire form re-emits deterministically), and the doc §3 load-bearing
  structural fact: **targets live in `Connections` `OP` records, not
  in `Properties70`** — each `C: "OP", <source>, <constraint>,
  "<slot>"` edge resolves to `{ slot, node: <scene node index> }`
  for Model endpoints (`{ slot, object: <name> }` otherwise). The
  `Definitions` **one-template-per-kind** rule (doc §1 — a parser
  assuming one `PropertyTemplate` per `ObjectType` loses every kind
  after the first) is honoured end-to-end:
  `Definitions::templates` keeps every template,
  `Definitions::template_named` looks one up by its concrete class
  name (`constraint::template_class_for_kind` maps `"Single Chain
  IK"` → `FbxConstraintSingleChainIK` per the doc's naming pattern),
  the bodies ride `extras["fbx:constraint_templates"]`, and the
  encoder re-emits the multi-template `ObjectType: "Constraint"`
  block + the constraint elements + their free-floating OP wiring —
  the whole catalogue survives `decode → encode → decode` in binary
  and ASCII forms. `MarkerSet` needs no implementation: doc §5
  establishes it is **not an FBX object class** (the token only
  occurs inside opaque MotionBuilder blind-data strings); the
  character / control-rig family remains a docs acquisition item.
- **`Properties70` typeName-discriminating accessors** —
  the existing [`PropertyMap::as_vec3`] and [`PropertyMap::as_str`]
  surface every triple-typed and string-typed `P`-record indiscriminately,
  but `docs/3d/fbx/fbx-binary-properties70.md` §4 documents prop1 (the
  typeName string) as the semantic discriminator (*"The typeName /
  label / flags strings carry the semantic type"*). Six typeName-aware
  accessors honour the docs §4 typeName mapping:
  - `as_color_rgb` — accepts `"ColorRGB"` and `"Color"` (the docs §4
    sample `AmbientColor S"ColorRGB"` and the cubes-ascii-v7500.fbx
    Material records `DiffuseColor "Color"`).
  - `as_vector3d` — accepts `"Vector3D"` and `"Vector"` (the cubes
    fixture's `PreRotation` / `PostRotation` / `GeometricTranslation` /
    `GeometricRotation` / `GeometricScaling` records).
  - `as_lcl_translation` / `as_lcl_rotation` / `as_lcl_scaling` — each
    requires its exact `"Lcl …"` typeName, so a caller pulling local
    transforms cannot accidentally pick up a `Vector3D` triple sitting
    under the same name.
  - `as_datetime` — accepts `"DateTime"` typeName (the cubes fixture's
    `Original|DateTime_GMT` / `LastSaved|DateTime_GMT` records carry
    the documented `MM/DD/YYYY HH:MM:SS.fff` string body); rejects a
    plain `"KString"` payload so the two surfaces stay disjoint.
  - `as_object_ref` — accepts `"object"` typeName (the cubes fixture's
    `SourceObject` / `LookAtProperty` / `UpVectorProperty` records);
    the empty-body case (`Compound` PValue when the exporter omits
    the trailing string) surfaces as `""` so the slot's presence is
    still detectable from the property map alone, with the resolved
    object UID still living on the corresponding `Connections` `OP`
    record.
  Existing `as_vec3` / `as_str` callers are unaffected — the typed
  accessors narrow on top of the generic ones rather than replacing
  them.
- **`Properties70` typeName-discriminating scalar accessors**
  — alongside the triple/string typeName-aware accessors above, the
  scalar half covers each typeName from the docs §8 ASCII-grammar
  scalar enumeration (`int`, `enum`, `bool`, `double`, `Number`,
  `KString`, `KTime`, `ULongLong`) gets its own narrow accessor on
  top of the generic [`PropertyMap::as_f64`] / [`as_i32`] /
  [`as_i64`] / [`as_bool`] / [`as_str`] widening surface:
  - `as_int_typed` — `"int"` typeName only (cubes fixture's
    `UpAxis` / `UpAxisSign` / `FrontAxis` / `OriginalUpAxis*`
    `GlobalSettings` records); rejects coincident `"enum"` and
    `"bool"` payloads whose wire encoding would otherwise widen.
  - `as_enum` — `"enum"` typeName only (the cubes fixture's
    `TimeMode` / `TimeProtocol` / `SnapOnFrameMode`); distinguishes
    a true enumeration index from a plain `"int"` slot even though
    docs §4 wires both as `I`.
  - `as_bool_typed` — `"bool"` typeName only (the cubes fixture's
    `Primary Visibility` / `Mute` records, and the docs §8
    worked sample `P: "Mute", "bool", "", "",0`); coerces `Int` /
    `Long` wires via `!= 0` once the typeName guard confirms the
    slot is semantically a bool.
  - `as_double` — `"double"` typeName only (`UnitScaleFactor`,
    `Opacity`, `OriginalUnitScaleFactor`); kept disjoint from
    `as_number` even though both share the `D` wire per docs §4.
  - `as_number` — `"Number"` typeName only (cubes Material records'
    `DiffuseFactor` / `EmissiveFactor` / `Shininess` /
    `ReflectionFactor`).
  - `as_kstring` — `"KString"` typeName only (`DocumentUrl` /
    `SrcDocumentUrl` / `currentUVSet` / `DefaultCamera`); rejects
    coincident `"DateTime"` and `"object"` records so the
    [`as_datetime`] / [`as_object_ref`] surfaces stay disjoint.
  - `as_ktime` — `"KTime"` typeName only with lossless `L` (int64)
    decoding (`TimeSpanStart` / `TimeSpanStop`); widens `I` / `Bool`
    payloads losslessly once the typeName guard fires per the docs
    §4 mixed-wire note.
  - `as_ulonglong` — `"ULongLong"` typeName only (the docs §8
    worked sample `P: "BlendModeBypass", "ULongLong", "", "",0`);
    same `L`-wire path as `as_ktime` with the matching guard.
  Generic widening accessors continue to surface every variant — the
  typed accessors narrow on top.
- **`Properties70` `"Compound"` typeName-discriminating accessor**
  — covers the last typeName from the
  `docs/3d/fbx/fbx-ascii-grammar.md` §8 enumeration. With the triple,
  string, and scalar accessors above, the
  full §8 typeName enumeration (`int / double / enum / bool /
  KString / KTime / Number / ULongLong / ColorRGB / Color / Vector3D
  / Vector / Lcl Translation / Lcl Rotation / Lcl Scaling / DateTime
  / object / Compound`) is now covered by typeName-narrow surfaces.
  `"Compound"` is the value-less typeName (docs §4 trailing-value
  rule *"0 (for Compound, and any value-less property)"*; the §4
  worked sample `P props=4 S"TimeMarker" S"Compound" S"" S""` and
  the §8 ASCII counterpart `P: "Original", "Compound", "", ""` are
  byte-for-byte equivalent). The accessor pair is:
  - `is_compound(name)` — `true` only when the record exists with
    `type_name == "Compound"` AND the payload is the zero-trailing
    [`PValue::Compound`] shape; `false` for absent records,
    non-`Compound` typeNames, and malformed Compound records
    carrying a trailing payload.
  - `compound_names()` — iterator over every well-formed
    `"Compound"` record name (useful for enumerating the structural
    / template placeholder slots in a `Properties70` block, e.g.
    `Original` / `LastSaved` parent keys that precede the sibling
    `Original|ApplicationName` / `LastSaved|DateTime_GMT` nested
    keys sharing the prefix).
  Disjoint from `as_object_ref`: an `"object"` slot
  the exporter wrote with no body lands in `PValue::Compound` but
  keeps its `"object"` typeName, so it surfaces via `as_object_ref`
  (returning `""`) and never via `is_compound`.
- **`Properties70` flag-discriminating iterators** —
  surfaces the third parsed-but-otherwise-unused string in every
  `P` record (`PRecord::flags`, prop3 of the
  `docs/3d/fbx/fbx-binary-properties70.md` §4 / `fbx-ascii-grammar.md`
  §8 grammar). The docs define the alphabet *"`""` (none), `"A"`
  (animatable), `"U"` (user / UI)"* — flags compose freely (observed
  `"AU"`), so the iterators match by character containment, not
  full-string equality. Three accessors: `animatable_names()` /
  `user_names()` / `names_with_flag(char)`. An animation walker
  enumerates `animatable_names()` to find the slots eligible for
  AnimCurve wiring through the `Connections` `OP` records; a UI
  layer enumerates `user_names()` to find the custom attributes the
  artist added in the source DCC.
- **`Geometry` non-`Mesh` subtype discriminator** — the
  `docs/3d/fbx/fbx-binary-properties70.md` §6 point 3 enumeration lists
  the `Geometry` prop2 subtype string as the fine class discriminator;
  the `"Mesh"` subtype is tessellated by [`crate::geometry`] and
  `"Shape"` is consumed by the blend-shape path in [`crate::deformer`]
  (a `Shape` geometry connects to a `BlendShapeChannel`, never to a
  `Model`), but the remaining subtypes — `"NurbsCurve"`,
  `"NurbsSurface"`, `"Boundary"`, `"TrimNurbsSurface"`, `"Line"` — have
  no first-class mesh3d tessellation in this crate and were previously
  dropped entirely by the scene walker (no `Mesh`, no node tag). Round
  271 records the §6 discriminator string verbatim on the owning
  `Model`'s `Node::extras["fbx:geometry_kind"]` via the
  `Geometry -> Model` `OO` connection, so a consumer can detect that a
  non-tessellated NURBS / line geometry exists and what kind it is
  without re-walking the `FbxDocument`. Coexists on a distinct key from
  the `"fbx:node_attribute_kind"` key. The per-subtype control-point
  / knot-vector grammar that a real curve / surface evaluation would
  need is absent from the staged docs (only the subtype *names* are
  enumerated), so the decode-side join is a follow-up gated on that
  grammar being staged — the evaluation engine itself is in place
  (next bullet).
- **B-spline / NURBS evaluation + tessellation engine** (`nurbs`
  module) — the format-independent half of NURBS support, implemented
  from the textbook definition of the B-spline basis (Cox–de Boor
  recursion, `0/0 := 0` repeated-knot convention, rational
  homogeneous-coordinate extension). Validated typed models
  `NurbsCurve` / `NurbsSurface` (open / closed / periodic forms —
  periodic wraps the control polygon with `C^{p-1}` continuity and
  validates the knot vector's constant-period shifts), point
  evaluation, first derivatives via the rational quotient rule,
  analytic surface normals with a degenerate-pole nudge (a collapsed
  pole row re-derives the limit normal a small step into the
  domain), and tessellators emitting `oxideav_mesh3d::Primitive`s at
  a configurable resolution (`TessellationOptions`): surfaces →
  indexed `Triangles` with positions / outward analytic normals /
  normalized-parameter UVs, open curves → `LineStrip`, periodic
  curves → seamless `LineLoop`. Construction validates everything
  (finiteness, knot monotonicity + exact counts, strictly positive
  weights, non-degenerate domain), making evaluation total, and
  resolutions are capped (`MAX_TESSELLATION_VERTICES`) against
  hostile-resolution memory bombs. Pinned against analytic ground
  truth: rational quadratic quarter / full circles, a cylinder and a
  full sphere of revolution exactly on their quadrics through the
  tessellator, tensor-linear-precision polynomial patches,
  watertight periodic seams, finite-difference derivative checks,
  and fixed-seed generative totality sweeps.
- **NodeAttribute `"LimbNode"` / `"Null"` discriminator** —
  the remaining well-known `NodeAttribute` subtype discriminators
  documented in `docs/3d/fbx/fbx-binary-properties70.md` §6 that
  don't map onto a first-class [`oxideav_mesh3d`] type. The owning
  `Model`'s scene-graph `Node::extras["fbx:node_attribute_kind"]`
  records the §6 discriminator string verbatim (`"LimbNode"` for a
  skeletal bone, `"Null"` for a locator / empty), so consumers can
  distinguish bone Models from locator Models from plain Mesh Models
  without re-walking the `FbxDocument`. Coexists with the light/camera
  surfacing on a distinct key (`"fbx:light_type"` vs this one).
- **Lights / Cameras** — `Objects { NodeAttribute }` records
  whose subtype string (third property — see
  `docs/3d/fbx/fbx-binary-properties70.md` §6) is `"Light"` or
  `"Camera"` are decoded into [`oxideav_mesh3d::Light`] /
  [`oxideav_mesh3d::Camera`] and bound onto the owning
  `Model`'s scene-graph `Node::light` / `Node::camera` via the
  `NodeAttribute -> Model` `OO` connection. Inner `Properties70`
  blocks are decoded with the existing `crate::properties70`
  machinery; the well-known `P`-record names this round consumes
  (the FBX-SDK Light / Camera attribute `P`-records observed on
  `NodeAttribute` records) are:
  - **Light**: `Color` × `Intensity` (with the DCC-percentage 0.01x
    scale) → typed `Point` / `Directional`
    / `Spot` variant selected by `LightType` (0/1/2; 3 Area + 4
    Volume fall back to `Point` with `Node::extras["fbx:light_type"]`
    set to `"Area"` / `"Volume"` so the lossy mapping is recoverable).
    `DecayType != 0` promotes `DecayStart` to the light's `range`;
    `Spot` reads `InnerAngle` / `OuterAngle` (full-cone degrees) and
    converts to mesh3d's half-cone radians convention.
  - **Camera**: `CameraProjectionType` picks `Perspective` (0) /
    `Orthographic` (1). `FieldOfViewY` maps directly to mesh3d's
    `yfov` (degrees → radians); `FieldOfView` / `FieldOfViewX`
    (horizontal) is converted via the aspect ratio (FBX
    horizontal-aperture mode) — `yfov = 2 * atan(tan(xfov/2)/aspect)`.
    `NearPlane` / `FarPlane` populate `znear` / `zfar`; `AspectWidth`
    / `AspectHeight` collapse to the `aspect_ratio` field, and the
    absolute pair round-trips through
    `Node::extras["fbx:camera_resolution"]`. Orthographic cameras
    read `OrthoZoom` as the vertical half-extent + derive `xmag` via
    the aspect ratio.
- **Binary writer** — `write_document(&FbxDocument)` round-trips
  the parser's output back to a byte buffer the parser re-reads as an
  equal `FbxDocument`. Every property variant (scalars `Y` `C` `I` `F`
  `D` `L`; arrays `f` `d` `l` `i` `b`; specials `S` `R`) is emitted;
  the 32-bit (pre-7500) vs 64-bit (≥ 7500) Node Record layout is
  auto-selected from `FbxDocument::version`. Arrays are written
  uncompressed (`Encoding == 0`) for byte-determinism by default;
  callers that want smaller output can opt in to zlib-deflate via
  `write_document_with_options(&doc, &WriterOptions::default().compress_arrays_at(256))`
  (`Encoding == 1` per Gessler §"Array types"; a 32×32 quad-grid fixture
  shrinks from 40 512 bytes to 8 496 bytes, ≈ 21.0 % of the raw size —
  both figures include the trailing footer block emitted by default).
- **ASCII writer** — `write_ascii_document(&FbxDocument)`
  emits the document back as ASCII text per the observer grammar at
  `docs/3d/fbx/fbx-ascii-grammar.md`. Output starts with the two-line
  `; FBX <maj>.<min>.<patch> project file` + `; ----` banner (§1 /
  §7a); every child of `FbxDocument::root` renders at depth 0 with
  TAB-per-depth indentation (§4); leaf nodes drop body braces (§3);
  body nodes reproduce the SDK's observed `Key:  {` two-space quirk
  for empty value-lists and `Key: v1, v2 {` single-space form for
  non-empty (§3a). Scalars render in their grammar §5 forms
  (integers, full-precision f64 via Rust's `{:?}` shortest-round-trip
  formatter, `"..."` strings with backslashes passed through
  literally, bare `T` / `F` booleans). Typed arrays use the §6
  shorthand `Key: *N { a: v1,v2,... }` for every numeric-array
  variant (`F32Array`, `F64Array`, `I32Array`, `I64Array`,
  `BoolArray` as `0` / `1`). Round-trip closure
  `parse(write(parse(src))) == parse(src)` holds at the typed-tree
  level for the staged `docs/3d/fbx/fixtures/cubes-ascii-v7500.fbx`
  fixture (8 top-level §7 sections, 4 Geometry + 4 Model + 2
  Material objects, both float and int typed arrays, Cyrillic
  identifiers, backslash paths). Output is valid UTF-8 by
  construction. `R` raw blobs render as a quoted base64 string —
  the form the staged texture-video-ascii-v7500.fbx fixture uses
  for its embedded `Video.Content` (its text decodes to a TGA
  header), which the ASCII reader + `Content` consumer decode back,
  so embedded media survives both forms; the binary-only top-level
  `FileId` / `CreationTime` / `Creator` provenance records are
  omitted in ASCII output (every staged ASCII fixture carries
  exactly the eight §7 sections). Strings carrying interior `"` or
  newline have no ASCII grammar form and surface a clean
  `Error::invalid` rather than silently produce broken text.
  `write_ascii_document_with_options(&doc, &AsciiWriterOptions::default().emit_banner(false))`.
- **`Scene3D` encoder (`Mesh3DEncoder`)** — `FbxEncoder` /
  `scene_writer::encode_scene` is the inverse of `scene::build_scene`:
  it builds a fresh `FbxDocument` (`FBXHeaderExtension` +
  `GlobalSettings` + `Documents` + `References` + `Definitions` +
  `Objects` + `Connections` + `Takes` — the full §7 section set in
  fixture order) from an `oxideav_mesh3d::Scene3D` and serialises it
  to binary or ASCII. The `Definitions` census is derived from the
  actually-emitted `Objects` tree (every class counted,
  `GlobalSettings` participating as in the fixture) and carries the
  nine fixture-staged `PropertyTemplate` default sets — `FbxAnimStack`
  / `FbxAnimLayer` / `FbxMesh` / `FbxSurfaceLambert` / `FbxNode` from
  cubes-ascii-v7500.fbx plus, per
  `docs/3d/fbx/fbx-property-templates.md` §3, `FbxFileTexture` (16
  records) / `FbxVideo` (20) / `FbxAnimCurveNode` (the single
  value-less `d` Compound) and the 106-record `FbxCamera` body,
  the latter emitted for `NodeAttribute` exactly when every attribute
  in the document is a `Camera` (the doc §2 rule 2 concrete-class /
  no-template-on-mixture behaviour). `Deformer` / `Pose` /
  `AnimationCurve` stay count-only **by rule**, not by gap (doc §2
  rule 1 — those classes declare no FBX properties, so no producer
  ever writes a template for them); `Constraint` re-emits its
  round-tripped one-template-per-kind set (see the Constraints
  bullet).
  - **Geometry** — one `Geometry` per mesh (per-corner `Vertices` +
    sequential-triangle `PolygonVertexIndex`), with one
    `LayerElementNormal` per normal buffer, one `LayerElementUV` per
    UV set, one `LayerElementColor` per vertex-colour set (RGBA), a
    `LayerElementTangent` for the canonical glTF-style tangent slot
    (`Tangents` xyz + `TangentsW` handedness split), and the
    extras-borne extra normal / tangent layers + explicitly-authored
    binormals (`LayerElementBinormal`) re-emitted for
    single-primitive meshes — all `ByPolygonVertex` / `Direct`, the
    mapping the decode side flattens 1:1. Indexed primitives expand
    every attribute through the index buffer.
  - **Nodes / hierarchy** — one `Model` per node with
    `Lcl Translation` / `Lcl Rotation` (XYZ-Euler degrees) /
    `Lcl Scaling` P-records + the parent/child OO edges. Nodes
    carrying the decode-side chain extras re-emit the **authored**
    chain instead (the `fbx:lcl_*` triple verbatim — never the
    composed `Node::transform`, which would double-apply the pivot
    terms — plus `RotationOffset` / `RotationPivot` / `PreRotation`
    / `PostRotation` / `ScalingOffset` / `ScalingPivot` `Vector3D`
    records, `RotationOrder` / `InheritType` enums, and the
    `Geometric*` TRS), so `decode → encode → decode` preserves both
    the composed transform and the authored chain;
    `fbx:node_attribute_kind` `"LimbNode"` / `"Null"` markers re-emit
    their `NodeAttribute` so bone / locator tags survive re-encode.
    The §7c trailing Model-body leaves (`Shading: T` /
    `Culling: "CullingOff"`) decode onto `Node::extras["fbx:shading"]`
    / `["fbx:culling"]` and are re-emitted, so they survive the
    Scene3D round trip in both forms; a non-`"Mesh"` Model prop2
    subtype (§6 — `"LimbNode"` / `"Null"` / ...) round-trips via
    `Node::extras["fbx:model_subtype"]`.
  - **Materials / Textures** — `DiffuseColor` / `Opacity` /
    `EmissiveColor` / `ReflectionFactor` P-records; `Texture`
    (+ backing `Video.Content` R-blob for embedded bytes) with the
    `Texture -> Material(prop_name)` OP connection, plus the §3.1
    reference records: `UVSet` naming the referenced UV channel,
    `Translation` / `Rotation` / `Scaling` `Vector` records from a
    typed `TextureRef::transform` (radians → degrees), and the
    `fbx:texture_records` raw set verbatim. One `Texture` element
    per `TextureId` (first-referencing slot wins when several slots
    share a texture with divergent per-reference settings — a
    documented lossy edge). Multi-material
    meshes re-emit the `LayerElementMaterial` `ByPolygon` per-face
    slot table + slot-ordered `Material -> Model` OO connections from
    the `fbx:face_material_slots` / `fbx:material_slots` extras.
  - **Deformers** — `Deformer{Skin}` + per-joint `Deformer{Cluster}`
    per skinned node (cluster order = skeleton joint order;
    `Transform` = inverse-bind + `TransformLink` = identity so the
    decode-side composition reproduces the authored inverse-bind
    matrices exactly); `Deformer{BlendShape}` + `BlendShapeChannel` +
    `Geometry{Shape}` (sparse `Indexes` + `Vertices` + `Normals`
    deltas) per morph target — each channel carrying its static
    `DeformPercent` record (the owning node's *effective* blend
    state × 100: `Scene3D::effective_morph_weights`' node > mesh
    §3.7.4 precedence, so a `Node::weights` per-instance override
    lands on that node's own emitted channels) and its
    authored name from `Mesh::target_names` (the pre-0.0.6
    `fbx:morph_target_names` extras key as a fallback, then
    `Target{i}`); a `MorphWeights` sampler's per-key vectors come
    from `morph_weight_frames()`, so a `CubicSpline` sampler emits
    its centre values as the `DeformPercent` keys (tangent triples
    have no FBX curve-key home); one `Pose : "BindPose"` element
    (`PoseNode { Node, Matrix }` per posed node) from the
    `fbx:bind_pose` extras.
  - **Lights / Cameras** — one `NodeAttribute` per bound node
    (`LightType` / `Color` / `Intensity`×100 / `DecayType` +
    `DecayStart` / cone angles; `CameraProjectionType` /
    `FieldOfViewY` / `NearPlane` / `FarPlane` / aspect pair /
    `OrthoZoom`), OO-connected to the owning `Model`.
  - **Scene-wide metadata** — `GlobalSettings` re-renders the full
    decode-side recognised-name set (axis ints, time-mode enums,
    i64-exact `KTime` spans, `DefaultCamera`, `AmbientColor`,
    `CustomFrameRate`; a round-tripped non-canonical
    `UnitScaleFactor` survives verbatim). `FBXHeaderExtension`
    re-renders `Creator` / `CreationTimeStamp` / `SceneInfo`
    `MetaData` + `Original|Application*` provenance from `fbx:*`
    extras; `Takes` re-renders the take catalogue
    (`fbx:current_take` / `fbx:takes`).
  - **Animations** — one `AnimationStack` / `AnimationLayer` per
    `Animation` plus per-channel `AnimationCurveNode` + per-axis
    `AnimationCurve` (Translation / Scale split `d|X`/`d|Y`/`d|Z`;
    Rotation quaternions → XYZ-Euler degrees; MorphWeights → one
    `DeformPercent` curve **per morph-target slot** (0..1 weights
    × 100 to wire percentages), each OP-connected to the node's
    matching `BlendShapeChannel`; `KeyTime` in KTime ticks) with the
    full OO/OP chain.
  - The complete `Scene3D → encode → decode → Scene3D` closure is
    round-trip-tested for positions / normals / multi-UV / vertex
    colours / tangents / binormals / hierarchy / multi-material slot
    tables / skins / morph targets / lights / cameras / external +
    embedded textures / unit + axis / header + takes metadata /
    translation + rotation + morph-weight animation /
    deflate-compressed arrays. Builder knobs:
    `FbxEncoder::new().form(FbxOutputForm::Ascii)`, `.version(7700)`,
    `.compress_arrays_at(256)`.

## Round-trip fidelity (round 455)

`tests/fixed_point.rs` is the writer's oracle: every staged fixture
(`docs/3d/fbx/fixtures/`, binary and ASCII) is decoded, re-encoded in
**both** output forms, and decoded twice more; the typed `Scene3D`
(every field, every `extras` key at every level, floats to 1e-4,
quaternions sign-canonical) is diffed feature-by-feature and the
`FbxDocument`'s record paths are counted. Three laws hold with **no
open allow-list** (only the documented ASCII form limits — the
binary-only `FileId` / `CreationTime` / `Creator` records and the
footer id — are excused):

- **parity** — nothing the reader surfaced from a fixture is dropped
  or degraded by one writer pass;
- **fixed point** — the writer converges after one pass
  (`decode(encode(decode(encode(x))))` is stable);
- **wire census** — every semantic record path in the fixture (bar the
  thumbnail / `OtherFlags` producer blocks and the optional `NormalsW`
  companion) reappears at least as often in the re-encode.

The mechanism behind it is **verbatim-unless-edited passthrough**: the
decoder keeps each element's own `Properties70` records and body leaves
on `extras` (`fbx:material_records`, `fbx:model_records` /
`fbx:model_leaves`, `fbx:node_attribute_records` / `_leaves` / `_name`,
`fbx:global_settings_records`, `fbx:scene_info_records` / `_leaves` /
`_header` + `fbx:meta_data_leaves`, `fbx:geometry_records`,
`fbx:texture_records[i].leaves` / `video_leaves` / `video_records`,
`fbx:property_templates`, `fbx:anim_stacks`, `fbx:aux_curve_nodes`,
`fbx:opaque_objects`, `fbx:key_attrs` with each curve's own key grid),
and the writer re-emits a raw set untouched as long as the typed fields
still decode to the same values from it — comparing rotations as
rotations, materials through the decoder's own PBR mapping, curves by
re-sampling — and lets the typed field win for a mapped name only when
it was actually edited, with every other raw record still riding along.
Geometry is re-emitted **welded** (the original control-point table
and n-gon `PolygonVertexIndex`, `Edges` domain, `ByEdge` / `ByPolygon`
smoothing, `AllSame` / `ByPolygon` materials, `IndexToDirect` UV pools,
`Layer` binding blocks) whenever the per-corner buffers still agree
with the round-tripped layout, falling back to the expanded triangle
list for an edited mesh. Skins round-trip from the fixture's
`Model → Cluster` edge direction, `CollectionExclusive` display layers
and custom-property curve nodes (`mr displacement …`) pass through
opaque, and the ASCII form carries embedded `Video.Content` as base64.
No third-party FBX consumer is installed on the round's machine, so the
crate's own reader is the black-box oracle.

## Decode

```rust
use oxideav_mesh3d::Mesh3DDecoder;
use oxideav_fbx::FbxDecoder;

let bytes = std::fs::read("model.fbx")?;
let scene = FbxDecoder::new().decode(&bytes)?;
println!("{} mesh(es), {} node(s)", scene.meshes.len(), scene.nodes.len());
# Ok::<_, Box<dyn std::error::Error>>(())
```

## Notes & limitations

Both the binary and ASCII front-ends are supported; the items below note
the partial-support edges and the not-yet-implemented surfaces.

- **ASCII FBX reader** (supported) — input starting with the
  `; FBX <version>` banner comment (observer grammar in
  `docs/3d/fbx/fbx-ascii-grammar.md`) is routed through
  `ascii::parse`, which produces the **same** typed `FbxDocument` tree
  the binary reader produces; every downstream consumer (scene /
  geometry / material / animation / deformer / pose / properties70)
  handles ASCII inputs transparently. Validated end-to-end against
  the staged `docs/3d/fbx/fixtures/cubes-ascii-v7500.fbx` fixture
  (8 top-level §7 sections; 4 Geometry + 4 Model + 2 Material +
  AnimationStack + AnimationLayer; first mesh's `Vertices: *24`
  decodes to a 24-double `F64Array`; UTF-8 / Cyrillic
  `Model::Куб1` name preserved). Typed-array bodies (`Key: *N { a:
  v1,v2,... }`) narrow integer arrays to `I32Array` when every
  element fits (matching the binary `i` variant the geometry puller
  needs verbatim for `PolygonVertexIndex` / `UVIndex` / `Materials`)
  and fall back to `I64Array` when any element overflows (matching
  the binary `l` variant the animation module's KTime puller
  accepts). A comma may be followed by a line break before the next
  value — the continuation form the staged
  texture-video-ascii-v7500.fbx fixture demonstrates on its
  embedded-media record (`Content: ,` with the base64 string on the
  following line) — so SDK-written embedded-texture files parse.
  Bytes matching neither the binary magic nor the ASCII
  banner return a single sniff-failure error. The ASCII writer is
  described under "ASCII writer" above.
- Encoder lossy edges —
  multi-primitive meshes skip the extras-borne extra-layer
  re-emission (no unambiguous per-primitive concatenation) and always
  take the expanded triangle-list form (welding needs the decoded
  single-primitive layout). An opaque object's edge to a peer this
  writer has no id for (an `object`-named endpoint) is not
  re-created. The
  `Mesh::weights` gap is closed: static per-target morph weights
  round-trip through each `BlendShapeChannel`'s `DeformPercent`
  record (×100 out, ÷100 back), and a `Node::weights` per-instance
  override (mesh3d 0.0.5) is what the owning node's channels emit —
  FBX's only static-weight home is geometry-level, so the re-decoded
  value returns on `Mesh::weights` with the effective node > mesh
  chain preserved exactly (a mesh *shared* by nodes with divergent
  overrides keeps the per-node emission granularity, but FBX cannot
  express two blend states on one shared geometry). A texture shared
  by several material slots emits one `Texture` element carrying the
  first reference's `UVSet` / placement records. The
  `Definitions` template gap is closed: every class with a staged
  body emits it (see the encoder bullet above) and the count-only
  remainder (`Deformer` / `Pose` / `AnimationCurve`; `NodeAttribute`
  on a mixed attribute set) is the **documented producer behaviour**,
  not a gap — the only unobserved bodies left are `FbxSkeleton` /
  `FbxNull` / `FbxBlendShapeChannel`
  (`docs/3d/fbx/fbx-property-templates.md` §5, needing
  differently-shaped exports).
- Binary footer id derivation — the footer's structure round-trips
  byte-faithfully (see the "Binary footer" bullet above), but the
  16-byte per-file id's *derivation* is undocumented by every staged
  source, so freshly-encoded scenes carry an all-zero id (a captured
  id from `parse_footer` / `fbx:footer_id` is reproduced verbatim).
- Animation: per-layer compositing weights and `KeyAttrFlags` cubic /
  step / TCB interpolation modes remain uninterpreted — linear
  sampling between keyframes only. The raw payloads are not dropped:
  every `AnimationCurve` carrying `KeyAttrFlags` / `KeyAttrDataFloat`
  / `KeyAttrRefCount` contributes one entry to
  `Scene3D::extras["fbx:key_attrs"]` (stack / target / property /
  axis join key, the integer arrays verbatim, the data floats as
  lossless IEEE-754 bit patterns, the curve's `Default` / `KeyVer`
  and its **own key grid + values**). Nothing is interpreted because
  `docs/3d/fbx/GAP-TRACKER.md` records **no value assignment** for
  the bitfield (the open acquisition item — an export sweep pinning
  one known mode per curve). On encode the writer re-emits the
  *original* per-axis curve verbatim — attributes included — whenever
  the typed channel still samples identically from it (all 90 curves
  of the staged skin-anim fixture); an edited channel falls back to
  the union-grid curve, which carries the attributes only when its
  key count is unchanged, since stretching an uninterpreted per-key
  table onto a new grid would require exactly those semantics.
  Stacks / layers (`fbx:anim_stacks`, curve-less takes included) and
  curve nodes outside the typed channel set (`fbx:aux_curve_nodes`)
  pass through verbatim. (`PreRotation` / `PostRotation` / pivot /
  `RotationOrder` composition **is** applied: channels bound to a
  chain-bearing Model re-compose the doc §1 product per merged
  keyframe, so the rotation channel is `Rpre · R(t) · Rpost⁻¹` and
  the translation channel carries the pivot swing.)
- Skin: `SKINNING_METHOD_DUAL_QUATERNION` / `BLENDED_DQ_LINEAR`
  surface as plain LBS buffers (the doc notes this is safe to ignore
  unless the renderer specifically needs it).
- BlendShape: in-between shapes are collapsed to the most-recent
  `Shape` per channel, and `MorphTarget::inbetweens` (mesh3d 0.0.6)
  is left empty in both directions. The in-between-shape grammar (a
  channel with several `Shape` targets and the station-weight table
  that would give each its `Inbetween::weight`) is pinned by no
  staged doc — `docs/3d/fbx/fbx-binary-properties70.md` §6 only
  names the `"Shape"` subtype and `fbx-property-templates.md` §4.1
  leaves the `FbxBlendShapeChannel` body unobserved — so no
  interpretation is attempted: a docs acquisition item.
- Specular workflow — FBX `Specular` / `SpecularFactor` aren't
  surfaced because the glTF metallic-roughness target has no separate
  specular colour channel. The values still round-trip through the
  `FbxDocument` for callers that need them; an FBX `Phong` →
  `KHR_materials_specular` mapping is a future-round option.
- Multi-material meshes via `LayerElementMaterial` per-face indices
  (partial) — the FBX `LayerElementMaterial` payload is surfaced:
  `MappingInformationType=ByPolygon` per-polygon slot indices land on
  `Primitive::extras["fbx:face_material_slots"]` (one `u32` per
  triangle corner, fanned through the same triangulation the position
  buffer uses); `AllSame` broadcasts a single slot. Every `Material ->
  Model` OO connection in slot order lands on
  `Primitive::extras["fbx:material_slots"]` (a JSON array of
  `MaterialId.0`s) so a downstream consumer can split the primitive
  into one Primitive-per-slot; `Primitive::material` stays at slot 0
  for single-binding renderers. Splitting the
  per-corner attribute buffers (positions / normals / UVs / skin /
  morph) into N parts is the consumer's job — the slot table + the
  per-corner index buffer are the only inputs that decision needs.
- **NURBS wire decode** — the `nurbs` module carries the complete
  evaluation + tessellation engine, but the FBX wire payload grammar
  for the non-`Mesh` geometry subtypes (`"NurbsCurve"` /
  `"NurbsSurface"` / `"TrimNurbsSurface"` / `"Boundary"` / `"Line"`)
  is a staged-docs gap: `docs/3d/fbx/fbx-binary-properties70.md` §6
  point 3 enumerates only the subtype *names*, and no staged fixture
  contains such a geometry, so the record names + layouts for knot
  vectors, control points, orders, forms and weights cannot be pinned
  from staged bytes. Until that grammar lands, the scene walker
  surfaces the subtype discriminator on
  `Node::extras["fbx:geometry_kind"]` and the payload rides the
  `FbxDocument` untyped.
- Coordinate-system / unit-scale **auto-conversion** —
  `GlobalSettings` is *decoded* (see "GlobalSettings"
  above) so the file's authored axis convention + unit factor land
  on `Scene3D::unit` (for the canonical 1.0 / 100.0 cases) +
  `Scene3D::extras`. Actually *transforming* the geometry into a
  target frame (e.g. rebuilding every `Primitive::positions` /
  `Transform::Trs` into a right-handed Y-up metre space when the
  source file is left-handed Z-up centimetres) is a separate
  follow-up — the `Scene3D` shape doesn't yet have a non-trivial
  axis-conversion primitive.
- **Light / Camera animation channels** — `AnimationCurveNode`
  records targeting the light/camera `Color` / `Intensity` /
  `FieldOfView` `P`-records round-trip through the `FbxDocument` but
  the [`oxideav_mesh3d::Animation`] channel set only models
  `Lcl Translation` / `Lcl Rotation` / `Lcl Scaling` / morph
  `DeformPercent`. Wiring light/camera-attribute curves into
  `AnimationTarget` is a follow-up; the static light/camera surfacing
  is supported.
- **Light / Camera aperture & film-back metadata** —
  `FilmWidth` / `FilmHeight` / `FocalLength` /
  area-light-shape / aperture-format presets don't fit the
  glTF-style `Camera::{Perspective, Orthographic}` /
  `Light::{Point, Directional, Spot}` enum surface; they round-trip
  through the `FbxDocument` for callers that need them. Area-light
  shape is tagged on the owning `Node::extras["fbx:light_type"]` so
  the lossy `Area`→`Point` collapse is recoverable.

## Standalone build

`oxideav-core` is gated behind the default-on `registry` cargo feature.
Drop the framework dependency with `default-features = false`; the
decoder API stays available and the `Error` alias falls back to
`oxideav_mesh3d`'s crate-local enum.

## License

Apache-2.0 — see [LICENSE](LICENSE).
