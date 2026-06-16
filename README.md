# oxideav-fbx

Pure-Rust FBX (Filmbox) binary mesh decoder + low-level binary writer.

FBX is Autodesk's proprietary 3D scene-and-asset interchange format,
originally developed by Kaydara for MotionBuilder. There is no
Autodesk-published prose specification — this crate is implemented
clean-room from third-party documentation:

- **Binary container** — Alexander Gessler / Blender Foundation,
  *FBX Binary File Format Specification* (August 2013, public-domain
  dedication). Staged at `docs/3d/fbx/blender-fbx-binary-format.html`.
- **Object-graph semantics** — ufbx project documentation (dual MIT /
  Unlicense). Staged under `docs/3d/fbx/ufbx/`.

## What's covered

- Binary container reader: 27-byte header, recursive Node Record
  walker (32-bit pre-7500, 64-bit ≥ 7500), full property type-code
  dispatch (`Y` `C` `I` `F` `D` `L` scalars, `f` `d` `l` `i` `b`
  arrays incl. zlib-deflated, `S` / `R` strings & blobs).
- Object-graph walker: indexes `Geometry` and `Model` from `Objects`,
  walks `Connections` `OO` records to wire Geometry → Model and
  Model → root.
- Mesh extraction: `Vertices` + `PolygonVertexIndex` →
  per-corner `Primitive(Topology::Triangles)` (ngons fan-triangulated;
  end-of-polygon negatives bit-NOT decoded). `LayerElementNormal` /
  `LayerElementUV` flattened when the mapping mode is `ByPolygonVertex`
  or `ByVertex` (with optional `IndexToDirect` indirection), each
  layer's `MappingInformationType` / `ReferenceInformationType`
  resolved independently. A `Geometry` carrying **more than one**
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
- Deformers: `Deformer{Skin}` + `Deformer{Cluster}` →
  `oxideav_mesh3d::Skeleton` + `Skin` (per-corner top-4 joints +
  weights, normalised; inverse-bind = `inverse(TransformLink) * Transform`).
  `Deformer{BlendShape}` + `BlendShapeChannel` + `Geometry{Shape}`
  → `MorphTarget` per channel (sparse `Indexes` deltas expanded to
  per-corner buffers).
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
  `Primitive::material` on the bound mesh.
- **Vertex colours** — every `LayerElementColor` sub-record
  on a `Geometry` element is surfaced as a separate per-corner RGBA
  buffer on `Primitive::colors` (one slot per FBX colour set,
  mirroring ufbx's `vertex_color` first slot + `color_sets[1..]`
  exposure). Mapping / reference handling matches Normals
  (`ByPolygonVertex` / `ByVertex` with optional `IndexToDirect`
  indirection); the `d`-array `Colors` payload is 4-component RGBA per
  ufbx reference §`ufbx_color_set.vertex_color`.
- **Multi-UV-set surfacing** — every `LayerElementUV`
  sub-record on a `Geometry` element is now surfaced as a separate
  per-corner `[f32; 2]` buffer on `Primitive::uvs` (one entry per
  FBX UV channel, in document order). Per
  `docs/3d/fbx/ufbx/reference.html` §`ufbx_mesh.uv_sets` /
  §`ufbx_uv_set`, an FBX mesh may carry multiple UV channels (the
  canonical diffuse + lightmap pair); the first set is also aliased
  at `ufbx_mesh.vertex_uv`. Mapping / reference handling reuses the
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
  `Unit::Centimetres` and `1.0` → `Unit::Metres` per the two values
  explicitly documented in `docs/3d/fbx/ufbx/elements-nodes.md` (the
  *"FBX files usually default to centimeters
  (`ufbx_scene_settings.unit_meters = 0.01`)"* + *"meter units
  (`ufbx_scene_settings.unit_meters = 1.0`)"* statements). Other
  `UnitScaleFactor` values surface the raw factor on
  `extras["fbx:unit_scale_factor"]` without claiming a typed
  `Unit` mapping the docs don't provide. Axis ints (`UpAxis = 1`,
  `FrontAxis = 2`, `CoordAxis = 0`) round-trip through `extras` but
  the FBX-int → `Axis` variant table is absent from the staged docs,
  so `Scene3D::up_axis` / `front_axis` stay at the `Scene3D::new`
  defaults pending a follow-up grammar staging.
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
- **Bind pose** —
  `Objects { Pose : "BindPose" }` elements surface each
  `PoseNode { Node, Matrix }` bone-world matrix onto the bone `Node`'s
  `extras["fbx:bind_pose"]` (16-double row-major JSON array). When a
  `Cluster` omitted its `TransformLink` sub-record (so the deformer
  module defaulted that joint's inverse-bind to identity), the bind
  pose back-fills it as `inverse(bone_to_world)` — the reference's
  documented *"FBX only stores world transformations so this is
  approximated"* case. `Matrix` is a direct `d`-array sub-record, so
  this stays clear of the still-unstaged `Properties70` `P`-record
  grammar. Joints that already have a real inverse-bind are untouched;
  non-bind rest poses (`is_bind_pose == false`) are not promoted. The decoder also derives the parent-space form
  `bone_to_parent = inverse(parent_bone_to_world) * bone_to_world` for
  every posed bone whose parent in the scene graph is also posed,
  surfaced as `node.extras["fbx:bind_pose_parent_local"]` (16-double
  row-major JSON array). Root bones whose parent has no bind pose
  receive `bone_to_parent == bone_to_world` (implicit-root convention,
  parent world = identity). Per `docs/3d/fbx/ufbx/reference.html`
  §`ufbx_bone_pose`, `bone_to_parent` is documented as *"approximated
  from the parent world transform"*.
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
  enumerated), so tessellation is a follow-up round.
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
  (sourced verbatim from `docs/3d/fbx/ufbx/reference.html`
  §`ufbx_light` / §`ufbx_camera` / §`ufbx_aperture_mode` /
  §`ufbx_aspect_mode`) are:
  - **Light**: `Color` × `Intensity` (with the documented 0.01x
    scale per §`ufbx_light.intensity`) → typed `Point` / `Directional`
    / `Spot` variant selected by `LightType` (0/1/2; 3 Area + 4
    Volume fall back to `Point` with `Node::extras["fbx:light_type"]`
    set to `"Area"` / `"Volume"` so the lossy mapping is recoverable).
    `DecayType != 0` promotes `DecayStart` to the light's `range`;
    `Spot` reads `InnerAngle` / `OuterAngle` (full-cone degrees) and
    converts to mesh3d's half-cone radians convention.
  - **Camera**: `CameraProjectionType` picks `Perspective` (0) /
    `Orthographic` (1). `FieldOfViewY` maps directly to mesh3d's
    `yfov` (degrees → radians); `FieldOfView` / `FieldOfViewX`
    (horizontal) is converted via the aspect ratio per
    §`ufbx_aperture_mode_horizontal` — `yfov = 2 * atan(tan(xfov/2)/aspect)`.
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
  shrinks from 40 346 bytes to 8 326 bytes, ≈ 20.6 % of the raw size).
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
  construction. `R` raw blobs (binary-only `R` properties) and
  strings carrying interior `"` or newline have no ASCII grammar
  form and surface a clean `Error::invalid` rather than silently
  produce broken text. Banner toggle via
  `write_ascii_document_with_options(&doc, &AsciiWriterOptions::default().emit_banner(false))`.

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
  accepts). Bytes matching neither the binary magic nor the ASCII
  banner return a single sniff-failure error. The ASCII writer is
  described under "ASCII writer" above.
- `Mesh3DEncoder` (Scene3D → bytes) — `write_document` operates on the
  parsed `FbxDocument` tree only; building a fresh `FbxDocument` from a
  `Scene3D` (the inverse of `scene::build_scene`) is a follow-up round.
- Autodesk binary footer — the Blender doc records its contents as
  "unknown"; `write_document` emits no footer at all. Files round-trip
  through our own parser but may be flagged by SDKs that validate the
  trailer signature.
- Animation: per-layer compositing weights, `KeyAttrFlags` cubic /
  step / TCB interpolation modes, `PreRotation` / `PostRotation` /
  pivot composition. Linear sampling between keyframes only.
- Skin: `SKINNING_METHOD_DUAL_QUATERNION` / `BLENDED_DQ_LINEAR`
  surface as plain LBS buffers (the doc notes this is safe to ignore
  unless the renderer specifically needs it).
- BlendShape: in-between keyframes are collapsed to the most-recent
  `Shape` per the doc's `target_shape` simplification.
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
  `UFBX_LIGHT_AREA_SHAPE_*` / aperture-format presets don't fit the
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
