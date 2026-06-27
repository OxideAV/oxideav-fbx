//! [`FbxEncoder`] — [`Scene3D`]-in, FBX bytes-out.
//!
//! The symmetric counterpart to [`crate::FbxDecoder`]. Builds a fresh
//! [`crate::FbxDocument`] from the scene via
//! [`crate::scene_writer::encode_scene`] (the inverse of
//! [`crate::scene::build_scene`]) and serialises it with
//! [`crate::writer::write_document`] (binary) or
//! [`crate::ascii_writer::write_ascii_document`] (ASCII text).
//!
//! ```rust
//! use oxideav_mesh3d::{Mesh3DEncoder, Scene3D};
//! use oxideav_fbx::FbxEncoder;
//!
//! let scene = Scene3D::new();
//! let bytes = FbxEncoder::new().encode(&scene).unwrap();
//! assert!(oxideav_fbx::is_binary_fbx(&bytes));
//! ```

use oxideav_mesh3d::{Mesh3DEncoder, Result, Scene3D};

use crate::ascii_writer::write_ascii_document;
use crate::scene_writer::{encode_scene_with_options, SceneEncodeOptions};
use crate::writer::{write_document_with_options, WriterOptions};

/// Which textual / binary form the encoder emits.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum FbxOutputForm {
    /// Kaydara binary container (the default; what most tools expect).
    #[default]
    Binary,
    /// `; FBX <version>` ASCII text per
    /// `docs/3d/fbx/fbx-ascii-grammar.md`.
    Ascii,
}

/// FBX encoder — implements [`Mesh3DEncoder`].
#[derive(Debug, Clone)]
pub struct FbxEncoder {
    /// Scene → document build knobs (version, layer emission).
    pub scene_options: SceneEncodeOptions,
    /// Binary serialisation knobs (array deflate threshold / level).
    pub writer_options: WriterOptions,
    /// Binary vs ASCII output.
    pub form: FbxOutputForm,
}

impl Default for FbxEncoder {
    fn default() -> Self {
        Self {
            scene_options: SceneEncodeOptions::default(),
            writer_options: WriterOptions::default(),
            form: FbxOutputForm::Binary,
        }
    }
}

impl FbxEncoder {
    /// Construct a binary-output encoder with default options.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder-style output-form selector.
    pub fn form(mut self, form: FbxOutputForm) -> Self {
        self.form = form;
        self
    }

    /// Builder-style file-format version override (also picks the
    /// 32-bit vs 64-bit binary Node Record layout).
    pub fn version(mut self, version: u32) -> Self {
        self.scene_options.version = version;
        self
    }

    /// Builder-style array-deflate opt-in for the binary form (arrays
    /// whose raw payload is at least `threshold` bytes are zlib-deflated).
    pub fn compress_arrays_at(mut self, threshold: usize) -> Self {
        self.writer_options = self.writer_options.clone().compress_arrays_at(threshold);
        self
    }
}

impl Mesh3DEncoder for FbxEncoder {
    fn encode(&mut self, scene: &Scene3D) -> Result<Vec<u8>> {
        let doc = encode_scene_with_options(scene, &self.scene_options);
        match self.form {
            FbxOutputForm::Binary => write_document_with_options(&doc, &self.writer_options),
            FbxOutputForm::Ascii => write_ascii_document(&doc),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_mesh3d::{Mesh, Mesh3DDecoder, Node, Primitive, Topology};

    use crate::FbxDecoder;

    fn one_triangle_scene() -> Scene3D {
        let mut scene = Scene3D::new();
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        let mut mesh = Mesh::new(Some("Tri".to_string()));
        mesh.primitives.push(prim);
        let mid = scene.add_mesh(mesh);
        let nid = scene.add_node(Node::new().with_name("N").with_mesh(mid));
        scene.roots.push(nid);
        scene
    }

    #[test]
    fn binary_encode_round_trips_through_decoder() {
        let scene = one_triangle_scene();
        let bytes = FbxEncoder::new().encode(&scene).unwrap();
        assert!(crate::is_binary_fbx(&bytes));
        let scene2 = FbxDecoder::new().decode(&bytes).unwrap();
        assert_eq!(scene2.meshes.len(), 1);
        assert_eq!(scene2.meshes[0].primitives[0].positions.len(), 3);
    }

    #[test]
    fn ascii_encode_round_trips_through_decoder() {
        let scene = one_triangle_scene();
        let bytes = FbxEncoder::new()
            .form(FbxOutputForm::Ascii)
            .encode(&scene)
            .unwrap();
        let text = std::str::from_utf8(&bytes).unwrap();
        assert!(text.starts_with("; FBX"), "ASCII banner present");
        let scene2 = FbxDecoder::new().decode(&bytes).unwrap();
        assert_eq!(scene2.meshes.len(), 1);
        assert_eq!(scene2.meshes[0].primitives[0].positions.len(), 3);
    }

    #[test]
    fn version_override_selects_64bit_layout() {
        let scene = one_triangle_scene();
        let bytes = FbxEncoder::new().version(7700).encode(&scene).unwrap();
        let doc = crate::binary::parse(&bytes).unwrap();
        assert_eq!(doc.version, 7700);
        let scene2 = FbxDecoder::new().decode(&bytes).unwrap();
        assert_eq!(scene2.meshes[0].primitives[0].positions.len(), 3);
    }
}
