# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Round 246 — **`Properties70` typeName-discriminating scalar
  accessors.** Round 243 closed the triple-typed half of the
  typeName-aware accessor surface (`as_color_rgb` / `as_vector3d` /
  `as_lcl_translation` / `as_lcl_rotation` / `as_lcl_scaling` /
  `as_datetime` / `as_object_ref`). Round 246 closes the matching
  scalar half, so each typeName the docs §8 ASCII-grammar scalar
  enumeration calls out — `int`, `enum`, `bool`, `double`, `Number`,
  `KString`, `KTime`, `ULongLong` — gets its own typeName-narrow
  accessor on top of the existing generic [`PropertyMap::as_f64`] /
  [`as_i32`] / [`as_i64`] / [`as_bool`] / [`as_str`] widening
  surface:
  - **`as_int_typed`** accepts `"int"` typeName only (the
    cubes-ascii-v7500.fbx fixture's GlobalSettings `UpAxis` /
    `UpAxisSign` / `FrontAxis` / `OriginalUpAxis*` records). The
    typeName guard keeps a coincident `"enum"` index or `"bool"`
    flag off the surface even though both wire as `I` per docs §4.
  - **`as_enum`** accepts `"enum"` typeName only (the cubes fixture's
    `TimeMode` / `TimeProtocol` / `SnapOnFrameMode` records; the
    docs §4 sample `TimeMode S"enum" S"" S"" I=0`). Distinguishes a
    true enumeration index from a plain `"int"` slot.
  - **`as_bool_typed`** accepts `"bool"` typeName only (the cubes
    fixture's `Primary Visibility` / `Mute` records; the docs §8
    worked sample `P: "Mute", "bool", "", "",0`). Coerces `Int` /
    `Long` wire payloads via `!= 0` once the typeName check confirms
    the slot is semantically a bool — older exporters mix the wire
    codes per the docs §4 mixed-wire note.
  - **`as_double`** accepts `"double"` typeName only
    (`UnitScaleFactor`, `Opacity`, `OriginalUnitScaleFactor`). Kept
    disjoint from [`Self::as_number`] even though both share the `D`
    wire encoding per docs §4 *"`double`/`Number` → `D`"*.
  - **`as_number`** accepts `"Number"` typeName only (the cubes
    fixture's Material records `DiffuseFactor` / `EmissiveFactor` /
    `Shininess` / `ReflectionFactor` all wire as
    `P: "...", "Number", "", "A", <D>`).
  - **`as_kstring`** accepts `"KString"` typeName only (the cubes
    fixture's `DocumentUrl` / `SrcDocumentUrl` / `currentUVSet` /
    `DefaultCamera` records). Rejects coincident `"DateTime"` and
    `"object"` records so the round-243 [`Self::as_datetime`] /
    [`Self::as_object_ref`] surfaces stay disjoint even though all
    three share the `S` wire encoding.
  - **`as_ktime`** accepts `"KTime"` typeName only with lossless `L`
    (int64) decoding (the cubes fixture's `TimeSpanStart` /
    `TimeSpanStop` GlobalSettings records; the docs §4 sample
    `TimeSpanStop S"KTime" S"Time" S"" L=46_186_158_000`). Widens
    `I` / `Bool` payloads losslessly once the typeName guard fires
    per the docs §4 mixed-wire note, so an older exporter wiring a
    short KTime as `I` still reads back correctly.
  - **`as_ulonglong`** accepts `"ULongLong"` typeName only (the docs
    §8 worked sample `P: "BlendModeBypass", "ULongLong", "", "",0`).
    Same `L`-wire decoding path as `as_ktime` with the matching
    typeName guard.
  Each accessor's `None` return covers three disjoint cases — record
  not present, typeName mismatch, payload-shape mismatch — so a
  caller can use the typed surface as a strict-mode validator
  without re-walking the underlying [`FbxDocument`]. Generic
  widening accessors continue to surface every variant; the typed
  accessors narrow on top.

- Round 243 — **`Properties70` typeName-discriminating accessors.**
  The existing [`PropertyMap::as_vec3`] and [`PropertyMap::as_str`]
  surface every triple-typed and string-typed `P` record
  indiscriminately, but `docs/3d/fbx/fbx-binary-properties70.md` §4
  documents prop1 (the typeName string) as the semantic
  discriminator (*"The typeName / label / flags strings carry the
  semantic type; the leading one-byte code carries the wire type"*).
  This round adds six typeName-aware accessors on top of the existing
  ones so a caller pulling, say, a `Lcl Rotation` triple cannot
  accidentally pick up a `Vector3D` triple sitting under the same
  name from a non-standard exporter:
  - **`as_color_rgb`** accepts `"ColorRGB"` and `"Color"` typeNames
    (the docs §4 worked sample `AmbientColor S"ColorRGB"` and the
    cubes-ascii-v7500.fbx Material records `DiffuseColor S"Color"`).
  - **`as_vector3d`** accepts `"Vector3D"` and `"Vector"` typeNames
    (the cubes fixture's `PreRotation` / `PostRotation` /
    `GeometricTranslation` / `GeometricRotation` / `GeometricScaling`
    records). The ASCII grammar §8 typeName list enumerates both.
  - **`as_lcl_translation`** / **`as_lcl_rotation`** /
    **`as_lcl_scaling`** each require their exact `"Lcl …"`
    typeName. The docs §4 trailing-value table calls out
    `"Lcl Translation"` and `"Lcl Scaling"` explicitly; `"Lcl
    Rotation"` is in the ASCII grammar §8 typeName enumeration
    alongside them.
  - **`as_datetime`** accepts the `"DateTime"` typeName documented
    in the docs §4 *"`KString`/`DateTime` → `S`"* row. The
    cubes-ascii-v7500.fbx fixture's `FBXHeaderExtension` block
    carries the worked sample form `P: "Original|DateTime_GMT",
    "DateTime", "", "", "07/01/2019 16:17:31.730"`; returning the
    raw string body matches the docs' refusal to specify a parsed
    `chrono`-style breakdown, while the typeName guard prevents a
    plain `"KString"` payload from surfacing here unintentionally.
  - **`as_object_ref`** accepts the `"object"` typeName enumerated
    in the ASCII grammar §8 typeName list (distinct from
    `"KString"`). The cubes fixture's `SourceObject` /
    `LookAtProperty` / `UpVectorProperty` records all carry an
    empty body that the decoder lands as `PValue::Compound`; the
    accessor surfaces `""` in that case so the slot's presence is
    still detectable from the property map alone, with the
    resolved object UID living on the corresponding `Connections`
    `OP` record. An exporter wiring the slot with an inline
    string body (e.g. `"Model::SomeNode"`) is also surfaced.
  - **Coverage** — 7 new unit tests in `src/properties70.rs::tests`:
    `as_color_rgb_accepts_colorrgb_and_color_typenames`,
    `as_color_rgb_rejects_non_color_typenames`,
    `as_vector3d_accepts_vector3d_and_vector_typenames`,
    `as_vector3d_rejects_color_and_lcl_typenames`,
    `as_lcl_translation_rotation_scaling_split_by_typename`,
    `as_datetime_accepts_datetime_typename_only`,
    `as_object_ref_accepts_object_typename_with_str_or_compound_body`.
    Test count: 143 → 150 unit (+7), 27 integration unchanged.
  Existing `as_vec3` / `as_str` callers are unaffected — the typed
  accessors narrow on top of the generic ones rather than replacing
  them; the round-191 material decoder, round-207 light/camera
  decoder, round-219 GlobalSettings decoder, and round-235
  NodeAttribute discriminator surfacer all stay as written, since
  they read names whose typeName is unambiguous in the worked
  fixture samples.

  References: `docs/3d/fbx/fbx-binary-properties70.md` §4
  (typeName-to-wire mapping, trailing-value-count table, worked
  GlobalSettings sample including `AmbientColor` / `DefaultCamera`
  / `TimeSpanStop` / `TimeMarker`),
  `docs/3d/fbx/fbx-ascii-grammar.md` §8 (the typeName enumerated
  list `int`, `double`, `enum`, `bool`, `KString`, `KTime`,
  `Number`, `ColorRGB`, `Color`, `Vector3D`, `Vector`, `Compound`,
  `ULongLong`, `DateTime`, `Lcl Translation`, `Lcl Scaling`,
  `object`).

- Round 240 — **`PropertyMap::as_i64` lossless `KTime` / `ULongLong` /
  `Long` accessor.** Per the §4 wire-code table in
  `docs/3d/fbx/fbx-binary-properties70.md`, the `KTime` and
  `ULongLong` typeNames are encoded as the `L` (int64) property code,
  which means their range routinely exceeds f64's safe-integer
  ceiling — the doc's own sample value `TimeSpanStop =
  46_186_158_000` is already past the i32 ceiling, and any `KTime`
  approaching the 2^53 boundary loses precision when round-tripped
  through the existing [`PropertyMap::as_f64`] path. The new
  accessor returns the underlying [`PValue::Long`] verbatim and
  losslessly widens [`PValue::Int`] / [`PValue::Bool`] payloads so
  exporters that wire an otherwise-`KTime` value as `I` (per the
  §4 note about older exporters mixing the integer wire codes for
  some `KTime` / `ULongLong` records) still decode correctly. Non-
  numeric records (`Str` / `Vec3` / `Compound` / `Double` / `Other`)
  return `None` so the caller can fall back without ambiguity.
  - **`globals.rs::ktime_long` refactor.** The previously-private
    helper that handled the same lossless `KTime` pull for
    `TimeSpanStart` / `TimeSpanStop` is now a one-line alias around
    the new public `PropertyMap::as_i64`. The behaviour is unchanged
    (every round-219 `GlobalSettings` test still passes verbatim);
    the change only removes the duplication so future callers
    (e.g. animation `KeyTime` pullers, or any new element that
    surfaces a `ULongLong` flag P-record) don't re-roll a third
    private copy of the same int64 widener.
  - **Coverage** — 4 new unit tests in `src/properties70.rs::tests`:
    `as_i64_preserves_int64_past_f64_safe_range` exercises the
    `2^53 + 1` precision-ceiling case and additionally asserts the
    `as_f64` path drops the low-order bit (documenting why the typed
    accessor is needed); `as_i64_widens_int_and_bool_wire_codes`
    exercises the int32 + bool widening path against `ULongLong` and
    `bool` typeNames; `as_i64_rejects_non_numeric_records` exercises
    the rejection branches for `KString` / `ColorRGB` / `Compound` /
    `double` payloads plus the missing-record `None`;
    `as_i64_handles_negative_ktime` exercises the signed range for
    `TimeSpanStart`-style negative values. The pre-existing
    `decode_ktime_long` unit test also gains an `as_i64`
    cross-check assertion. Test count: 139 → 143 unit (+4),
    27 integration unchanged.
  - References: `docs/3d/fbx/fbx-binary-properties70.md` §3
    (the `L` int64 wire code), §4 (the `KTime` / `ULongLong` →
    `L` typeName-to-wire mapping plus the older-exporter integer
    wire-mixing note), `docs/3d/fbx/fbx-ascii-grammar.md` §5
    (the ASCII counterpart for the same `KTime` typeName).

- Round 235 — **`NodeAttribute` `"LimbNode"` / `"Null"` discriminator
  surfacing.** The §6 ruleset in
  `docs/3d/fbx/fbx-binary-properties70.md` lists `"LimbNode"`
  (skeletal bone) and `"Null"` (locator / empty) as well-known
  `NodeAttribute` subtype discriminators, alongside the typed
  `"Light"` / `"Camera"` ones the round-207 path consumes. The new
  `src/node_attribute.rs` module records the §6 discriminator on the
  owning `Model`'s scene-graph `Node::extras["fbx:node_attribute_kind"]`
  (value = the subtype string verbatim) so downstream consumers can
  distinguish a skeletal-bone Model from a locator Model from a
  plain Mesh Model without re-walking the `FbxDocument`.
  - **Decode path.** `Objects { NodeAttribute }` records whose
    `prop2` subtype string is `"LimbNode"` or `"Null"` are indexed,
    then `Connections { C "OO" }` walks bind each attribute to its
    owning `Model`. The kind tag is written first-wins (so a Model
    with two NodeAttribute children of different kinds keeps the
    first-resolved discriminator deterministically).
  - **Idempotence with round 207.** The light/camera path writes
    `Node::extras["fbx:light_type"]` only for lossy `Area` / `Volume`
    fall-backs; this round writes a distinct key
    (`"fbx:node_attribute_kind"`) so the two surfacing passes never
    collide on the same scene node. Pre-existing `"fbx:light_type"`
    tags are preserved unchanged.
  - **Out of scope (documented in module-level comment).** The
    `ufbx_bone` `radius` / `relative_length` / `is_root` and
    `ufbx_empty` fields documented in
    `docs/3d/fbx/ufbx/reference.html` §`ufbx_bone` / §`ufbx_empty`
    are decoded ufbx-side fields whose specific FBX `P`-record names
    are not enumerated in the staged docs; a follow-up round can add
    them once a bone / empty `Properties70` P-record name table is
    staged. `"Root"` is only documented as a `Model` subtype (not a
    `NodeAttribute` subtype) per §6, so it isn't dispatched here.
  - 7 new unit tests in `src/node_attribute.rs::tests`: LimbNode and
    Null kinds land on owning-Model extras; unknown subtypes don't;
    orphan NodeAttribute (no OO wiring) is a no-op; the light_type
    key coexists with the new kind key without collision; first-kind
    wins on degenerate two-attribute Models; non-`OO` connection
    kinds (`OP` / `PP` / `PO`) don't trigger the tag. 1 new
    integration test in
    `tests/synthetic_node_attribute.rs::limbnode_and_null_node_attributes_round_trip_through_decoder`
    builds an `Objects { LimbNode-attr, Null-attr, Bone1-model,
    Locator1-model }` + `Connections { OO 600→700, OO 601→701 }`
    document, writes it through the round-3 binary writer, then
    decodes it through `FbxDecoder::new().decode()` and asserts the
    two named `Node`s carry `"fbx:node_attribute_kind"` =
    `"LimbNode"` / `"Null"`.
  - Test count: 132 → 139 unit (+7), 26 → 27 integration (+1).

- Round 226 — **bind-pose `bone_to_parent` derivation** (closes the
  round-97 "Pose `bone_to_parent`" entry on the README "Lacks" list).
  - Once `extract_poses` has stashed every `PoseNode { Matrix }`
    world matrix onto its bone's `extras["fbx:bind_pose"]` (round 97)
    and refined every identity-defaulted skeleton inverse-bind, the
    new step builds the scene-graph parent map from
    `scene.nodes[*].children` (back-pointers aren't stored on
    `oxideav_mesh3d::Node`, so the parent index is materialised on
    the fly), then for every posed bone derives
    `bone_to_parent = inverse(parent_bone_to_world) * bone_to_world`
    and writes the result onto the bone's
    `node.extras["fbx:bind_pose_parent_local"]` (16-double row-major
    JSON array, mirroring the existing `fbx:bind_pose` shape).
  - **Implicit-root convention** — a posed bone whose scene-graph
    parent has no bind pose (e.g. a root bone parented directly to
    the scene root, or to an un-posed `Null` Model) receives
    `bone_to_parent == bone_to_world`. This corresponds to the parent
    world transform being the identity, the natural extension of
    the doc's *"approximated from the parent world transform"*
    statement to the no-parent edge case.
  - Per `docs/3d/fbx/ufbx/reference.html` §`ufbx_bone_pose`,
    `bone_to_parent` is documented as: *"Matrix from node local
    space to parent space. FBX only stores world transformations so
    this is approximated from the parent world transform."* No new
    on-disk reading — the derivation runs
    entirely on the already-decoded bind-pose set and the
    `scene::build_scene` scene-graph parentage.
  - 6 new unit tests in `src/pose.rs::tests`: `mat4_mul` identity /
    translation-composition correctness, root-bone parent-local
    equals world, child-bone parent-local equals inverse-parent ×
    child, child with unposed parent falls back to world, no-pose
    no-op for the parent-local pass. 1 new integration test in
    `tests/synthetic_pose.rs::bind_pose_parent_local_chains_through_scene_graph`
    builds a two-bone chain (parent at world (10, 0, 0), child at
    (10, 5, 0)) with a `Pose: BindPose` posing both bones and the
    `Connections` wiring `301 → 300 → root`; asserts the parent
    bone's parent-local equals its world, and the child's
    parent-local resolves to a translation of (0, 5, 0).
  - Test count: 126 → 132 unit (+6), 25 → 26 integration (+1).
  - References: `docs/3d/fbx/ufbx/reference.html` §`ufbx_bone_pose`
    (the `bone_to_world` + `bone_to_parent` field definitions + the
    *"FBX only stores world transformations so this is approximated
    from the parent world transform"* note).

- Round 219 — **`GlobalSettings` element decode** (advances the
  "Coordinate-system / unit-scale auto-conversion" README "Lacks"
  tail to the *decoded but not auto-converted* state).
  - New `globals` module exposes
    `extract_global_settings(&FbxDocument, &mut Scene3D) -> usize`.
    Walks the top-level `GlobalSettings { Properties70 { P: ... } }`
    block via the existing `crate::properties70::PropertyMap` and
    surfaces every well-known P-record onto `Scene3D::extras` under
    the `"fbx:<snake_case>"` key convention (`fbx:up_axis`,
    `fbx:unit_scale_factor`, `fbx:ambient_color`, `fbx:default_camera`,
    `fbx:time_span_start`, ...). The recognised name list is sourced
    directly from the cubes-ascii-v7500.fbx fixture's GlobalSettings
    block + the box.fbx sample documented in
    `docs/3d/fbx/fbx-binary-properties70.md` §4 (`UpAxis` /
    `UpAxisSign` / `FrontAxis` / `FrontAxisSign` / `CoordAxis` /
    `CoordAxisSign` / `OriginalUpAxis` / `OriginalUpAxisSign` /
    `UnitScaleFactor` / `OriginalUnitScaleFactor` / `AmbientColor` /
    `DefaultCamera` / `TimeMode` / `TimeProtocol` /
    `SnapOnFrameMode` / `TimeSpanStart` / `TimeSpanStop` /
    `CustomFrameRate` / `CurrentTimeMarker`). Unrecognised names
    round-trip through `FbxDocument` but do not surface to extras
    (so a future round can opt in to additional names without an
    extras-key collision).
  - `UnitScaleFactor` is additionally translated to typed
    `Scene3D::unit`: `100.0` → `Unit::Centimetres`, `1.0` →
    `Unit::Metres`. These are the two values explicitly tied to
    `unit_meters` in `docs/3d/fbx/ufbx/elements-nodes.md` (*"Most
    unit-aware FBXs are expressed in centimeters
    (`ufbx_scene_settings.unit_meters = 0.01`)"* and *"meter units
    (`ufbx_scene_settings.unit_meters = 1.0`)"*) — the relation
    `unit_meters = 1 / UnitScaleFactor` holds for both. Other
    values leave `scene.unit` at the `Scene3D::new` default; the
    raw factor stays available on `extras["fbx:unit_scale_factor"]`
    for callers that need the literal exporter-side value.
  - `KTime` records (`TimeSpanStart` / `TimeSpanStop`) surface as
    i64 ticks to preserve every documented unit of precision (the
    KTime constant is ~`4.6e10` ticks/second and a long
    `TimeSpanStop` is in the `~4e14` range — beyond f64-exact
    integer territory). Downstream consumers can convert to seconds
    with the `animation::KTIME_TICKS_PER_SECOND` constant.
  - **No axis auto-conversion.** The `UpAxis` / `FrontAxis` /
    `CoordAxis` integer enum mapping to `oxideav_mesh3d::Axis`
    (positive/negative X/Y/Z) variants is not in the staged docs
    (the ufbx-side `ufbx_coordinate_axis` enum is documented as an
    enum but the *FBX-P-record-int → axis-variant* table itself is
    absent). The raw ints surface on `extras` and
    `Scene3D::up_axis` / `front_axis` stay at the `Scene3D::new`
    defaults. A follow-up docs-staging round can close this loop.
  - **No geometry transformation.** Module only *decodes* the
    settings into `Scene3D` metadata; transforming the geometry
    into a target axis/unit frame (e.g. converting cm → m by
    scaling every position by 0.01) is a separate follow-up — the
    `Scene3D` shape doesn't yet have a non-trivial axis-conversion
    primitive.
  - Called from `scene::build_scene` first (before
    `extract_deformers` / `extract_animations` / etc.) so any
    downstream module that consults `Scene3D::extras` finds them
    populated. The empty-scene fallback at end-of-`build_scene`
    now retains the GlobalSettings-derived `extras` + `unit`
    rather than discarding them.
  - 15 new unit tests in `src/globals.rs::tests` cover the missing
    / empty / unrecognised-name no-op paths, the per-type-bucket
    decode (int / KTime long / double / KString / Vec3), the
    UnitScaleFactor → Unit::{Centimetres, Metres} mapping + the
    "unknown factor leaves unit unchanged" path, the
    snake_case extras-key generator, an epsilon-tolerance check
    around the canonical UnitScaleFactor values, the
    no-clobber-prior-extras invariant, and a "full fixture" set
    that decodes all 19 P-records from the cubes fixture in one
    pass. 1 new integration test in
    `tests/synthetic_global_settings.rs` hand-builds a binary
    v7400 FBX with the full GlobalSettings P-record block and
    asserts the public `FbxDecoder::decode` path lifts every
    documented bucket onto `Scene3D::extras` + maps
    `UnitScaleFactor = 100.0` to `Scene3D::unit = Centimetres`.
    1 new integration test in `tests/ascii_fixture.rs` re-checks
    the ASCII round-trip on the real cubes fixture
    (`UnitScaleFactor=1` → `Unit::Metres`, `DefaultCamera` and
    `TimeMode` reach extras).
  - Test count: 110 → 126 unit (+16), 23 → 25 integration (+2).
  - References: `docs/3d/fbx/fbx-binary-properties70.md` §4
    (Properties70 grammar + the box.fbx GlobalSettings sample
    block); `docs/3d/fbx/fbx-ascii-grammar.md` §7 (top-level
    section list) / §8 (`P:` ASCII form);
    `docs/3d/fbx/ufbx/elements-nodes.md` (the cm:0.01 / m:1.0
    `unit_meters` documentation);
    `docs/3d/fbx/ufbx/reference.html` §`ufbx_scene_settings` (the
    typed scene-settings struct field list); the
    cubes-ascii-v7500.fbx fixture's GlobalSettings block (full P-
    record set).
- Round 213 — **ASCII FBX writer** (closes the round-200 "ASCII
  writer NYI" tail). New `ascii_writer` module exposes
  `write_ascii_document(&FbxDocument)` and
  `write_ascii_document_with_options(&doc, &AsciiWriterOptions)`.
  Emits the document back as ASCII text per the observer grammar at
  `docs/3d/fbx/fbx-ascii-grammar.md`:
  - §1 / §7a — two-line banner `; FBX <maj>.<min>.<patch> project
    file` + `; ----` (optional via
    `AsciiWriterOptions::emit_banner(false)`; the inner
    `FBXHeaderExtension { FBXVersion }` leaf is the parser's
    canonical version source, banner digits are informational).
  - §3 / §3a / §4 — body-form `Key:  { ... }` (two-space quirk for
    empty value-lists; single-space for non-empty) vs leaf-form
    `Key: <values>`; TAB-per-depth indentation.
  - §5 — scalar lexical forms: integer / full-precision f64 via
    Rust's `{:?}` shortest-round-trip formatter / `"..."` strings
    with backslashes copied through literally (per the §5 path
    string example) / bare `T` / `F` booleans.
  - §6 — typed array shorthand `Key: *N { a: v1,v2,... }` for every
    numeric-array variant (`F32Array`, `F64Array`, `I32Array`,
    `I64Array`, `BoolArray` rendered as `0` / `1`).
  - Round-trip closure `parse(write(parse(src))) == parse(src)`
    holds at the typed-tree level (the writer is canonical at the
    AST level; ASCII permits many lexically-distinct printings of
    the same tree). Validated against the staged
    `docs/3d/fbx/fixtures/cubes-ascii-v7500.fbx` fixture (full §7
    section coverage, both float and int typed arrays, Cyrillic
    identifiers, backslash paths).
  - Edge cases: empty arrays narrow to `I32Array([])` on re-read
    (grammar §6 carries no type evidence in zero-element form);
    binary-only `Raw` blobs surface a clean `Error::invalid`
    (grammar §5 has no `R` form); strings carrying an interior `"`
    or newline are rejected (grammar §5 strings stay on one line
    and use no escape mechanism).
  - 14 new module-level tests covering each grammar rule, an
    end-to-end fixture round-trip, and every error path.
  - New public exports: `ascii_writer::write_ascii_document`,
    `write_ascii_document_with_options`, `AsciiWriterOptions`.

- Round 207 — **Light / Camera `NodeAttribute` surfacing** (the
  long-standing DOCS-GAP "Light / Camera node attributes" bullet on
  the README "Lacks" list).
  - New `lights_cameras` module exposes
    `extract_lights_and_cameras(&FbxDocument, &mut Scene3D,
    &HashMap<i64, NodeId>)`. Walks the top-level
    `Objects { NodeAttribute }` records whose third-property subtype
    string (per `docs/3d/fbx/fbx-binary-properties70.md` §6) is
    `"Light"` or `"Camera"`, decodes the inner `Properties70` block
    via the existing `crate::properties70::PropertyMap`, and binds
    the result onto the owning `Model`'s scene-graph
    `Node::light` / `Node::camera` via the `NodeAttribute -> Model`
    `OO` connection.
  - **Light decode** — `LightType` (per ufbx §`ufbx_light_type`:
    0=Point, 1=Directional, 2=Spot, 3=Area, 4=Volume) picks the
    `oxideav_mesh3d::Light` variant. `Color` × `Intensity` populate
    the variant's color + intensity, with the documented
    `intensity × 0.01` scale applied per
    `docs/3d/fbx/ufbx/reference.html` §`ufbx_light.intensity`.
    `DecayType != 0` promotes `DecayStart` to the light's `range`;
    `Spot` reads `InnerAngle` / `OuterAngle` (full-cone degrees) and
    converts to mesh3d's half-cone radians convention. `CastShadows`
    + the raw `DecayType` int are stashed on the owning
    `Node::extras` (`fbx:cast_shadows` / `fbx:decay_type`). Area
    (3) and Volume (4) lights fall back to `Light::Point` and tag
    `Node::extras["fbx:light_type"]` so the lossy mapping is
    recoverable.
  - **Camera decode** — `CameraProjectionType` picks `Perspective`
    (0) / `Orthographic` (1). `FieldOfViewY` maps directly to
    mesh3d's `yfov` (degrees → radians); `FieldOfView` /
    `FieldOfViewX` (horizontal) is converted via the aspect ratio
    per `§ufbx_aperture_mode_horizontal` as
    `yfov = 2 * atan(tan(xfov/2) / aspect)`. `NearPlane` / `FarPlane`
    populate `znear` / `zfar`; `AspectWidth` / `AspectHeight` collapse
    to the `aspect_ratio` field, and the absolute pair round-trips
    through `Node::extras["fbx:camera_resolution"]` (per
    `§ufbx_aspect_mode_fixed_resolution`, where the same fields can
    carry pixel resolution). Orthographic cameras read `OrthoZoom`
    as the vertical half-extent and derive `xmag` via the aspect
    ratio.
  - All P-record property names are taken verbatim from
    `docs/3d/fbx/ufbx/reference.html` §`ufbx_light` / §`ufbx_camera`
    / §`ufbx_aperture_mode` / §`ufbx_aspect_mode`; the §6
    NodeAttribute discriminator and §4 P-record grammar live in
    `docs/3d/fbx/fbx-binary-properties70.md`. No FBX-implementation
    source consulted (not the Autodesk FBX SDK, assimp's FBX
    importer, Blender `io_scene_fbx`, nor `ufbx`'s C source).
  - 9 new unit tests in `src/lights_cameras.rs` cover each light
    variant (Point + Directional + Spot cone math + Area→Point
    kind-tag fallback), each camera projection (FoVY-priority
    Perspective + horizontal-FoV-derives-yfov + Orthographic), and
    both negative paths (missing owning Model → silent ignore;
    non-Light/non-Camera subtype → skipped). 1 new integration test
    in `tests/synthetic_light_camera.rs` assembles a binary v7400
    FBX with one Light + one Camera `NodeAttribute`, runs it
    through the public `FbxDecoder::decode` path, and asserts the
    bound `Scene3D::lights` / `.cameras` arenas + the owning node's
    `Node::light` / `.camera` / `.extras` payload.
  - Test count: 87 → 96 unit (+9), 13 → 14 integration (+1).
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
