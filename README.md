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
  end-of-polygon negatives bit-NOT decoded). First
  `LayerElementNormal` / `LayerElementUV` flattened when the mapping
  mode is `ByPolygonVertex` or `ByVertex` (with optional
  `IndexToDirect` indirection).
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
- **Materials / Textures / Video** (round 5, factor decode round 191)
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
- **Vertex colours** (round 184) — every `LayerElementColor` sub-record
  on a `Geometry` element is surfaced as a separate per-corner RGBA
  buffer on `Primitive::colors` (one slot per FBX colour set,
  mirroring ufbx's `vertex_color` first slot + `color_sets[1..]`
  exposure). Mapping / reference handling matches Normals
  (`ByPolygonVertex` / `ByVertex` with optional `IndexToDirect`
  indirection); the `d`-array `Colors` payload is 4-component RGBA per
  ufbx reference §`ufbx_color_set.vertex_color`.
- **Multi-UV-set surfacing** (round 194) — every `LayerElementUV`
  sub-record on a `Geometry` element is now surfaced as a separate
  per-corner `[f32; 2]` buffer on `Primitive::uvs` (one entry per
  FBX UV channel, in document order). Per
  `docs/3d/fbx/ufbx/reference.html` §`ufbx_mesh.uv_sets` /
  §`ufbx_uv_set`, an FBX mesh may carry multiple UV channels (the
  canonical diffuse + lightmap pair); the first set is also aliased
  at `ufbx_mesh.vertex_uv`. Mapping / reference handling reuses the
  round-1 2-component puller, so `ByPolygonVertex` / `ByVertex` and
  `Direct` / `IndexToDirect` work for every channel. Round-trip
  tested against `docs/3d/fbx/fixtures/cubes-ascii-v7500.fbx`
  ground-truth UV / UVIndex arrays + a two-UV-set synthetic.
- **Multi-material slot table** (round 178) — `LayerElementMaterial`
  per-polygon slot indices (`MappingInformationType=ByPolygon`) +
  every `Material -> Model` OO connection in slot order land on
  `Primitive::extras` (`fbx:face_material_slots` / `fbx:material_slots` /
  `fbx:material_mapping`), preserving the full per-face material
  payload alongside the legacy single-binding `Primitive::material`
  (which stays at slot 0 for round-5 single-binding consumers).
- **Bind pose** (round 97) — `Objects { Pose : "BindPose" }` elements
  surface each `PoseNode { Node, Matrix }` bone-world matrix onto the
  bone `Node`'s `extras["fbx:bind_pose"]` (16-double row-major JSON
  array). When a `Cluster` omitted its `TransformLink` sub-record (so
  the deformer module defaulted that joint's inverse-bind to identity),
  the bind pose back-fills it as `inverse(bone_to_world)` — the
  reference's documented *"FBX only stores world transformations so this
  is approximated"* case. `Matrix` is a direct `d`-array sub-record, so
  this stays clear of the still-unstaged `Properties70` `P`-record
  grammar. Joints that already have a real inverse-bind are untouched;
  non-bind rest poses (`is_bind_pose == false`) are not promoted.
- **Binary writer** (round 3) — `write_document(&FbxDocument)` round-trips
  the parser's output back to a byte buffer the parser re-reads as an
  equal `FbxDocument`. Every property variant (scalars `Y` `C` `I` `F`
  `D` `L`; arrays `f` `d` `l` `i` `b`; specials `S` `R`) is emitted;
  the 32-bit (pre-7500) vs 64-bit (≥ 7500) Node Record layout is
  auto-selected from `FbxDocument::version`. Arrays are written
  uncompressed (`Encoding == 0`) for byte-determinism by default;
  callers that want smaller output can opt in to zlib-deflate via
  `write_document_with_options(&doc, &WriterOptions::default().compress_arrays_at(256))`
  (round 4 — `Encoding == 1` per Gessler §"Array types"; a 32×32
  quad-grid fixture shrinks from 40 346 bytes to 8 326 bytes,
  ≈ 20.6 % of the raw size).

## Decode

```rust
use oxideav_mesh3d::Mesh3DDecoder;
use oxideav_fbx::FbxDecoder;

let bytes = std::fs::read("model.fbx")?;
let scene = FbxDecoder::new().decode(&bytes)?;
println!("{} mesh(es), {} node(s)", scene.meshes.len(), scene.nodes.len());
# Ok::<_, Box<dyn std::error::Error>>(())
```

## Lacks

- ASCII FBX — input not starting with the `Kaydara FBX Binary` magic
  returns `Error::Unsupported("ASCII FBX is not yet supported")`. The
  staged docs corpus deliberately omits an ASCII grammar source
  (Blender's writeup is binary-only and the original Kaydara FBX 6.x
  ASCII documentation is no longer on the public web — see
  `docs/3d/fbx/README.md` §"What's covered (and what isn't)").
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
- Multi-material meshes via `LayerElementMaterial` per-face indices —
  round 178 surfaces the FBX `LayerElementMaterial` payload:
  `MappingInformationType=ByPolygon` per-polygon slot indices land on
  `Primitive::extras["fbx:face_material_slots"]` (one `u32` per
  triangle corner, fanned through the same triangulation the position
  buffer uses); `AllSame` broadcasts a single slot. Every `Material ->
  Model` OO connection in slot order lands on
  `Primitive::extras["fbx:material_slots"]` (a JSON array of
  `MaterialId.0`s) so a downstream consumer can split the primitive
  into one Primitive-per-slot; `Primitive::material` stays at slot 0
  for single-binding renderers (the round-5 default). Splitting the
  per-corner attribute buffers (positions / normals / UVs / skin /
  morph) into N parts is the consumer's job — the slot table + the
  per-corner index buffer are the only inputs that decision needs.
- Coordinate-system / unit-scale auto-conversion.
- **Light / Camera node attributes** — DOCS-GAP. The on-disk FBX
  `NodeAttribute` record name, the `"Light"` / `"Camera"` subtype
  discriminators, and (critically) every attribute value
  (`Color` / `Intensity` / `LightType` / `DecayType` / `InnerAngle` /
  `OuterAngle` for lights; `FieldOfView` / `AspectWidth` /
  `AspectHeight` / `NearPlane` / `FarPlane` for cameras) live inside
  `Properties70 { P: name, type, label, flags, value… }` records whose
  grammar is not transcribed anywhere in the currently-staged
  `docs/3d/fbx/` corpus — the same gap that defers `Material`
  colour-factor decode. The ufbx reference documents the *semantic*
  model (`ufbx_light` / `ufbx_camera`, incl. the `Intensity` 0.01x
  quirk) but uses ufbx field names, not the raw FBX `P`-record names.
  Blocked pending a staged FBX `Properties70` `P`-record grammar in
  `docs/3d/fbx/`.
- **Pose `bone_to_parent`** — only the directly-stored `bone_to_world`
  matrix is surfaced; deriving the parent-space form needs the resolved
  ancestor chain and is left to a downstream consumer.

## Standalone build

`oxideav-core` is gated behind the default-on `registry` cargo feature.
Drop the framework dependency with `default-features = false`; the
decoder API stays available and the `Error` alias falls back to
`oxideav_mesh3d`'s crate-local enum.

## License

Apache-2.0 — see [LICENSE](LICENSE).
