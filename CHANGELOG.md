# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

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
