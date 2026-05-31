//! [`FbxDecoder`] — bytes-in, [`Scene3D`]-out.
//!
//! Sniffs the 20-byte `Kaydara FBX Binary` magic; routes to
//! [`crate::binary::parse`] + [`crate::scene::build_scene`] when it
//! matches. When the bytes start with the `; FBX <version>` ASCII
//! banner comment instead, routes to [`crate::ascii::parse`] (added
//! in round 200) and feeds the resulting [`FbxDocument`] into the
//! same scene builder — the two front-ends produce interchangeable
//! tree shapes. Bytes matching neither form return
//! [`Error::Unsupported`].

use oxideav_mesh3d::{Error, Mesh3DDecoder, Result, Scene3D};

use crate::ascii;
use crate::binary::{self, FbxDocument, FBX_MAGIC};
use crate::scene;

/// FBX decoder — implements [`Mesh3DDecoder`].
#[derive(Debug, Default)]
pub struct FbxDecoder {
    /// Last successfully-parsed [`FbxDocument`], retained so callers
    /// can reach element kinds (Material / Texture / AnimationStack /
    /// ...) that the round-1 [`Scene3D`] surface doesn't yet
    /// materialise.
    pub last_document: Option<FbxDocument>,
}

impl FbxDecoder {
    /// Construct a fresh decoder with default options.
    pub fn new() -> Self {
        Self::default()
    }
}

impl Mesh3DDecoder for FbxDecoder {
    fn decode(&mut self, bytes: &[u8]) -> Result<Scene3D> {
        let doc = if is_binary_fbx(bytes) {
            binary::parse(bytes)?
        } else if ascii::is_ascii_fbx(bytes) {
            ascii::parse(bytes)?
        } else {
            return Err(Error::unsupported(
                "input is neither binary FBX (Kaydara magic) nor ASCII FBX (`; FBX` banner)",
            ));
        };
        let scene = scene::build_scene(&doc)?;
        self.last_document = Some(doc);
        Ok(scene)
    }
}

/// `true` if `bytes` begins with the 20-byte
/// `b"Kaydara FBX Binary  \0"` magic.
pub fn is_binary_fbx(bytes: &[u8]) -> bool {
    bytes.len() >= FBX_MAGIC.len() && &bytes[..FBX_MAGIC.len()] == FBX_MAGIC
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ascii_input_is_not_binary() {
        let ascii = b"; FBX 7.4.0 project file\nFBXHeaderExtension:  {\n";
        assert!(!is_binary_fbx(ascii));
    }

    #[test]
    fn binary_magic_detected() {
        assert!(is_binary_fbx(FBX_MAGIC));
        assert!(is_binary_fbx(&[FBX_MAGIC, &[0u8; 100]].concat()));
    }

    #[test]
    fn ascii_input_decodes_via_ascii_front_end() {
        // Minimal ASCII shell that survives scene::build_scene.
        let src = b"; FBX 7.5.0 project file\n\
                    FBXHeaderExtension:  {\n\
                    \tFBXVersion: 7500\n\
                    }\n\
                    Objects:  {\n\
                    }\n\
                    Connections:  {\n\
                    }\n";
        let mut dec = FbxDecoder::new();
        let scene = dec.decode(src).expect("ASCII parse + scene build");
        // No meshes in this empty Objects section — but the decoder
        // accepted the input and produced a Scene3D rather than
        // returning Unsupported.
        let _ = scene;
        assert_eq!(dec.last_document.as_ref().map(|d| d.version), Some(7500));
    }

    #[test]
    fn neither_binary_nor_ascii_returns_unsupported() {
        let mut dec = FbxDecoder::new();
        let err = dec.decode(b"this is not FBX at all").unwrap_err();
        let s = err.to_string();
        assert!(
            s.contains("ASCII") || s.contains("Kaydara") || s.contains("neither"),
            "expected sniff failure message, got: {s}"
        );
    }
}
