//! [`FbxDecoder`] — bytes-in, [`Scene3D`]-out.
//!
//! Sniffs the 20-byte `Kaydara FBX Binary` magic; routes to
//! [`crate::binary::parse`] + [`crate::scene::build_scene`] when it
//! matches. ASCII FBX is documented as "explicitly NYI in r1" and
//! returns [`Error::Unsupported`].

use oxideav_mesh3d::{Error, Mesh3DDecoder, Result, Scene3D};

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
        if !is_binary_fbx(bytes) {
            return Err(Error::unsupported("ASCII FBX is not yet supported"));
        }
        let doc = binary::parse(bytes)?;
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
    fn ascii_decode_returns_unsupported() {
        let mut dec = FbxDecoder::new();
        let err = dec.decode(b"; FBX 7.4.0\n").unwrap_err();
        let s = err.to_string();
        assert!(s.contains("ASCII"), "expected ASCII-mention in error: {s}");
    }
}
