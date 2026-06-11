//! Pure-Rust FBX (Filmbox) binary decoder.
//!
//! Implements [`oxideav_mesh3d::Mesh3DDecoder`] for the binary
//! encoding of the FBX format originally developed by Kaydara for
//! MotionBuilder, acquired by Autodesk in 2006. ASCII FBX is
//! explicitly NYI in round 1 (see crate-level CHANGELOG).
//!
//! # References
//!
//! - `docs/3d/fbx/blender-fbx-binary-format.html` — Alexander
//!   Gessler / Blender Foundation, *FBX Binary File Format
//!   Specification* (August 2013, public-domain dedication). Covers
//!   the 27-byte header, the recursive Node Record layout, and the
//!   property type-code dispatch table.
//! - `docs/3d/fbx/ufbx/elements-meshes.md` — ufbx project,
//!   *Meshes* documentation (dual MIT / Unlicense). Documents the
//!   `Geometry` element, the `LayerElement*` attribute system, and
//!   the polygon-vertex-index encoding.
//! - `docs/3d/fbx/ufbx/elements-overview.md` — ufbx project,
//!   *Elements* overview. Documents the `Objects` / `Connections`
//!   shape used by the object-graph walker in [`scene`].
//!
//! # What's covered
//!
//! - Binary container reader: header + Node Record tree, full
//!   property type-code dispatch (`Y` `C` `I` `F` `D` `L` for
//!   scalars, `f` `d` `l` `i` `b` for arrays — uncompressed and
//!   `miniz_oxide`-deflated), `S` / `R` for strings & raw blobs.
//!   Auto-selects 32-bit or 64-bit Node Record header based on the
//!   `Version` field.
//! - **Binary container writer** (round 3) — round-trips an
//!   [`FbxDocument`] back to bytes that our own [`binary::parse`] reads
//!   into an equivalent document. See [`writer`]. The Autodesk-private
//!   footer (Gessler: *"unknown contents"*) is **not** emitted; files
//!   are valid against our parser's grammar but may be flagged by SDKs
//!   that validate the trailer signature.
//! - Object-graph walker: indexes `Geometry` and `Model` elements
//!   from `Objects`, walks `Connections` `OO` records to wire
//!   Geometry → Model and Model → root.
//! - Mesh extraction: `Vertices` + `PolygonVertexIndex` →
//!   per-corner [`oxideav_mesh3d::Primitive`] of
//!   [`oxideav_mesh3d::Topology::Triangles`] (ngons fan-triangulated).
//!   First `LayerElementNormal` / `LayerElementUV` flattened when the
//!   mapping mode is `ByPolygonVertex` or `ByVertex`.
//! - **Animation** (round 2) — `AnimationStack` / `AnimationLayer` /
//!   `AnimationCurveNode` / `AnimationCurve` map to one
//!   [`oxideav_mesh3d::Animation`] per stack with channels for
//!   `Lcl Translation`, `Lcl Rotation` (XYZ-Euler-degrees → quaternion),
//!   `Lcl Scaling`, and morph-target `DeformPercent`. See [`animation`].
//! - **Deformers** (round 2) — `Deformer{Skin}` + `Deformer{Cluster}`
//!   produce [`oxideav_mesh3d::Skeleton`] + [`oxideav_mesh3d::Skin`];
//!   `Deformer{BlendShape}` + `BlendShapeChannel` + `Geometry{Shape}`
//!   produce [`oxideav_mesh3d::MorphTarget`]s. See [`deformer`].
//! - **Materials / Textures / Video** (round 5) —
//!   `Objects { Material | Texture | Video }` records surface as
//!   [`oxideav_mesh3d::Material`] / [`oxideav_mesh3d::Texture`] on
//!   [`Scene3D`]. `Connections OP Texture -> Material(prop_name)`
//!   wires `DiffuseColor` / `NormalMap` / `EmissiveColor` (plus
//!   Maya / 3ds-Max exporter aliases) into the typed PBR slots.
//!   `Material -> Model` OO connections set `Primitive::material`.
//!   See [`material`].
//! - **GlobalSettings** (round 219) — the top-level `GlobalSettings`
//!   node's `Properties70` block is decoded via the round-191
//!   `PropertyMap`; every well-known `P`-record from the
//!   cubes-ascii-v7500.fbx fixture (`UpAxis` / `UpAxisSign` /
//!   `FrontAxis` / `FrontAxisSign` / `CoordAxis` / `CoordAxisSign` /
//!   `OriginalUpAxis*` / `UnitScaleFactor` / `OriginalUnitScaleFactor`
//!   / `AmbientColor` / `DefaultCamera` / `TimeMode` / `TimeProtocol`
//!   / `SnapOnFrameMode` / `TimeSpanStart` / `TimeSpanStop` /
//!   `CustomFrameRate` / `CurrentTimeMarker`) lands on
//!   `Scene3D::extras` under the `"fbx:<snake_case>"` key convention.
//!   `UnitScaleFactor` is additionally translated to `Scene3D::unit`:
//!   `100.0 → Unit::Centimetres` and `1.0 → Unit::Metres` per the
//!   `unit_meters` documentation in
//!   `docs/3d/fbx/ufbx/elements-nodes.md`. See [`globals`].
//! - **Definitions / PropertyTemplate** (round 280) — the top-level
//!   `Definitions` section (per `docs/3d/fbx/fbx-ascii-grammar.md`
//!   §7b) decodes via [`definitions::Definitions`]: the section
//!   `Version` / total `Count`, each `ObjectType`'s class name +
//!   instance count, and the class `PropertyTemplate` *"default
//!   `Properties70`"* set. Material decode resolves each element's
//!   own `P` records against the `"Material"` class template via
//!   [`properties70::PropertyMap::with_template_defaults`], so
//!   exporter-omitted class defaults decode like explicit records.
//! - **Bind pose** (round 97) — `Objects { Pose : "BindPose" }`
//!   elements with `PoseNode { Node, Matrix }` sub-records surface
//!   each posed bone's world matrix onto its
//!   [`oxideav_mesh3d::Node`]'s `extras["fbx:bind_pose"]`, and refine
//!   any [`oxideav_mesh3d::Skeleton`] inverse-bind matrix the deformer
//!   module had to default to identity (cluster without a
//!   `TransformLink`) to `inverse(bone_to_world)`. See [`pose`].
//!
//! - **ASCII FBX reader** (round 200) — input matching the
//!   `; FBX <version>` banner comment (observer grammar in
//!   `docs/3d/fbx/fbx-ascii-grammar.md`) is now routed through
//!   [`ascii::parse`] to produce the same typed [`FbxDocument`] tree
//!   the binary reader produces, so every downstream consumer
//!   ([`scene`] / [`geometry`] / [`material`] / [`animation`] /
//!   [`deformer`] / [`pose`] / [`properties70`]) handles ASCII inputs
//!   transparently. Validated against the staged
//!   `docs/3d/fbx/fixtures/cubes-ascii-v7500.fbx` fixture (8
//!   top-level sections, 4 Geometry + 4 Model + 2 Material +
//!   AnimationStack / AnimationLayer all surface; first mesh's
//!   `Vertices: *24` decodes to a 24-double `F64Array`).
//! - **ASCII FBX writer** (round 213) — [`ascii_writer::write_ascii_document`]
//!   emits an [`FbxDocument`] back as ASCII text per the observer
//!   grammar at `docs/3d/fbx/fbx-ascii-grammar.md`. The round-trip
//!   closure `parse(write(parse(src))) == parse(src)` holds at the
//!   typed-tree level for the staged
//!   `docs/3d/fbx/fixtures/cubes-ascii-v7500.fbx` fixture (full §7
//!   section coverage, both float / int typed arrays, Cyrillic
//!   identifiers, backslash paths). Banner toggle via
//!   [`ascii_writer::AsciiWriterOptions::emit_banner`].
//! - **Lights / Cameras** (round 207) —
//!   `Objects { NodeAttribute }` records with subtype `"Light"` /
//!   `"Camera"` decode through [`lights_cameras`] into
//!   [`oxideav_mesh3d::Light`] / [`oxideav_mesh3d::Camera`] bound to
//!   the owning `Model` node via the `NodeAttribute -> Model` `OO`
//!   connection. The well-known `P`-record names this round consumes
//!   (`Color` / `Intensity` / `LightType` / `DecayType` /
//!   `InnerAngle` / `OuterAngle` for lights; `CameraProjectionType` /
//!   `FieldOfViewY` / `FieldOfView` / `AspectWidth` / `AspectHeight` /
//!   `NearPlane` / `FarPlane` / `OrthoZoom` for cameras) are taken
//!   from `docs/3d/fbx/ufbx/reference.html` §`ufbx_light` /
//!   §`ufbx_camera`; the §6 NodeAttribute discriminator + §4 P-record
//!   grammar live in `docs/3d/fbx/fbx-binary-properties70.md`.
//!
//! # What's NOT covered
//! - **Scene-graph encoder (`Scene3D` → FBX bytes)** — bytes-out at
//!   the [`oxideav_mesh3d::Mesh3DEncoder`] level is a separate round.
//!   This round only ships the lower-level [`writer::write_document`]
//!   that serialises a parsed [`FbxDocument`] back to bytes; building
//!   a fresh `FbxDocument` from a `Scene3D` (the inverse of
//!   [`scene::build_scene`]) is not yet implemented.
//! - Animation: per-layer compositing, cubic / step / TCB
//!   interpolation, pivot / pre-rotation / post-rotation chains.
//! - Skin: `SKINNING_METHOD_DUAL_QUATERNION` produces plain LBS
//!   buffers (the doc notes this is safe to ignore in most cases).
//! - BlendShape: in-between keyframes are collapsed to `target_shape`.
//! - Material PBR colour / factor channels (round 191) — decode the
//!   element's `Properties70` `P`-record block via [`properties70`]
//!   and apply `DiffuseColor` / `DiffuseFactor` / `Opacity` /
//!   `EmissiveColor` / `EmissiveFactor` / `Shininess` /
//!   `ReflectionFactor` onto the matching [`oxideav_mesh3d::Material`]
//!   channels. The Blender writeup is binary-only and didn't cover
//!   the `P`-record grammar; the typed [`properties70::PropertyMap`]
//!   here is derived from the observer-doc at
//!   `docs/3d/fbx/fbx-binary-properties70.md` §4.
//! - Multi-material meshes via `LayerElementMaterial` per-face
//!   indices — round 5 ships one material per mesh.
//! - Coordinate-system / unit-scale conversion — files travel with
//!   their author's axis convention; downstream consumers handle
//!   re-orientation per the [`Scene3D::up_axis`] /
//!   [`Scene3D::front_axis`] / [`Scene3D::unit`] metadata.
//!
//! # Standalone build
//!
//! `oxideav-core` is gated behind the default-on `registry` cargo
//! feature. Drop the framework dependency entirely with:
//!
//! ```toml
//! oxideav-fbx = { version = "0.0", default-features = false }
//! ```
//!
//! The decoder API and parser modules stay available; only the
//! [`register`] entry point + [`oxideav_mesh3d::Mesh3DRegistry`]
//! plumbing disappear and the error type falls back to
//! `oxideav_mesh3d`'s crate-local enum.

#![forbid(unsafe_code)]
#![warn(missing_debug_implementations)]

pub mod animation;
pub mod ascii;
pub mod ascii_writer;
pub mod binary;
pub mod decoder;
/// `Definitions` section decoder — per-class instance counts +
/// `PropertyTemplate` default `Properties70` blocks resolved against
/// each object's own records (round 280).
pub mod definitions;
pub mod deformer;
pub mod geometry;
/// `Geometry` non-`Mesh` subtype-discriminator surfacing
/// (`NurbsCurve` / `NurbsSurface` / `Boundary` / `TrimNurbsSurface` /
/// `Line`) onto the owning Model's `Node::extras["fbx:geometry_kind"]`
/// (round 271).
pub mod geometry_kind;
/// `GlobalSettings` element decoder — scene-wide axis / unit / time /
/// ambient settings surfaced onto [`oxideav_mesh3d::Scene3D`] (round
/// 219).
pub mod globals;
/// `NodeAttribute` (`Light` / `Camera`) surfacing onto [`oxideav_mesh3d`]
/// (round 207).
pub mod lights_cameras;
pub mod material;
pub mod node_attribute;
pub mod pose;
pub mod properties70;
pub mod scene;
pub mod writer;

pub use ascii::is_ascii_fbx;
pub use ascii_writer::{
    write_ascii_document, write_ascii_document_with_options, AsciiWriterOptions,
};
pub use binary::{FbxDocument, FbxNode, FbxProperty, FBX_MAGIC, FBX_VERSION_64BIT_THRESHOLD};
pub use decoder::{is_binary_fbx, FbxDecoder};
pub use writer::{write_document, write_document_with_options, WriterOptions};

/// Format-id string used in the [`oxideav_mesh3d::Mesh3DRegistry`].
pub const FORMAT_ID: &str = "fbx";

/// File extensions handled by the FBX decoder.
pub const EXTENSIONS: &[&str] = &["fbx"];

/// Wire `oxideav-fbx` into a [`oxideav_mesh3d::Mesh3DRegistry`].
///
/// Registers a decoder factory under format id [`FORMAT_ID`] and
/// extensions [`EXTENSIONS`]. The encoder side is NYI (round 2+).
#[cfg(feature = "registry")]
pub fn register(registry: &mut oxideav_mesh3d::Mesh3DRegistry) {
    registry.register_decoder(
        FORMAT_ID,
        EXTENSIONS,
        Box::new(|| Box::new(FbxDecoder::new())),
    );
}
