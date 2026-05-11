# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.0.1](https://github.com/OxideAV/oxideav-fbx/releases/tag/v0.0.1) - 2026-05-11

### Other

- Initial commit: oxideav-fbx round 1 (binary container reader + decoder)

### Added

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
