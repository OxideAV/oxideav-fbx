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
//! # What's covered (round 1)
//!
//! - Binary container reader: header + Node Record tree, full
//!   property type-code dispatch (`Y` `C` `I` `F` `D` `L` for
//!   scalars, `f` `d` `l` `i` `b` for arrays — uncompressed and
//!   `miniz_oxide`-deflated), `S` / `R` for strings & raw blobs.
//!   Auto-selects 32-bit or 64-bit Node Record header based on the
//!   `Version` field.
//! - Object-graph walker: indexes `Geometry` and `Model` elements
//!   from `Objects`, walks `Connections` `OO` records to wire
//!   Geometry → Model and Model → root.
//! - Mesh extraction: `Vertices` + `PolygonVertexIndex` →
//!   per-corner [`oxideav_mesh3d::Primitive`] of
//!   [`oxideav_mesh3d::Topology::Triangles`] (ngons fan-triangulated).
//!   First [`LayerElementNormal`] / [`LayerElementUV`] flattened
//!   when the mapping mode is `ByPolygonVertex` or `ByVertex`.
//!
//! # What's NOT covered (round 1)
//!
//! - **ASCII FBX** — input not starting with the binary magic
//!   returns [`oxideav_mesh3d::Error::Unsupported`].
//! - **Encoder** — bytes-out is a separate round.
//! - Skin / Cluster (deformer) wiring, AnimationStack / Layer /
//!   Curve, BlendShape / BlendShapeChannel.
//! - Material / Texture / Video — parsed into [`FbxDocument`] but
//!   not surfaced on [`Scene3D`].
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

pub mod binary;
pub mod decoder;
pub mod geometry;
pub mod scene;

pub use binary::{FbxDocument, FbxNode, FbxProperty, FBX_MAGIC, FBX_VERSION_64BIT_THRESHOLD};
pub use decoder::{is_binary_fbx, FbxDecoder};

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
