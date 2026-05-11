# oxideav-fbx

Pure-Rust FBX (Filmbox) binary mesh decoder.

FBX is Autodesk's proprietary 3D scene-and-asset interchange format,
originally developed by Kaydara for MotionBuilder. There is no
Autodesk-published prose specification — this crate is implemented
clean-room from third-party documentation:

- **Binary container** — Alexander Gessler / Blender Foundation,
  *FBX Binary File Format Specification* (August 2013, public-domain
  dedication). Staged at `docs/3d/fbx/blender-fbx-binary-format.html`.
- **Object-graph semantics** — ufbx project documentation (dual MIT /
  Unlicense). Staged under `docs/3d/fbx/ufbx/`.

## What's covered (round 1)

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
  returns `Error::Unsupported("ASCII FBX is not yet supported")`.
- Encoder — bytes-out is a follow-up round.
- Skin / Cluster (deformer) wiring, AnimationStack / Layer / Curve,
  BlendShape / BlendShapeChannel — parsed into the underlying
  `FbxDocument` but not surfaced on `Scene3D`.
- Material / Texture / Video — same.
- Coordinate-system / unit-scale auto-conversion.

## Standalone build

`oxideav-core` is gated behind the default-on `registry` cargo feature.
Drop the framework dependency with `default-features = false`; the
decoder API stays available and the `Error` alias falls back to
`oxideav_mesh3d`'s crate-local enum.

## License

Apache-2.0 — see [LICENSE](LICENSE).
