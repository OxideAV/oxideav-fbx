//! Minimal standard-alphabet base64 (RFC 4648 §4, `=` padding) used
//! for the ASCII form of binary `R` payloads.
//!
//! The staged `docs/3d/fbx/fixtures/texture-video-ascii-v7500.fbx`
//! writes its embedded `Video.Content` as a quoted string whose text
//! is the base64 rendering of the same bytes the binary form carries
//! as an `R` blob (its leading `AAAKAAAAAAAAAAAAAAEAARgA` decodes to a
//! TGA run-length-truecolor header, `00 00 0A … 00 01 00 01 18`). The
//! ASCII reader therefore base64-decodes a `Content` string and the
//! ASCII writer base64-encodes a `Raw` property, so embedded media
//! survives both forms.

const ALPHABET: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/// Encode `bytes` as padded standard base64 text.
pub fn encode(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(bytes.len().div_ceil(3) * 4);
    for chunk in bytes.chunks(3) {
        let b0 = chunk[0] as u32;
        let b1 = chunk.get(1).copied().unwrap_or(0) as u32;
        let b2 = chunk.get(2).copied().unwrap_or(0) as u32;
        let n = (b0 << 16) | (b1 << 8) | b2;
        out.push(ALPHABET[(n >> 18) as usize & 63] as char);
        out.push(ALPHABET[(n >> 12) as usize & 63] as char);
        out.push(if chunk.len() > 1 {
            ALPHABET[(n >> 6) as usize & 63] as char
        } else {
            '='
        });
        out.push(if chunk.len() > 2 {
            ALPHABET[n as usize & 63] as char
        } else {
            '='
        });
    }
    out
}

fn value(c: u8) -> Option<u32> {
    Some(match c {
        b'A'..=b'Z' => (c - b'A') as u32,
        b'a'..=b'z' => (c - b'a') as u32 + 26,
        b'0'..=b'9' => (c - b'0') as u32 + 52,
        b'+' => 62,
        b'/' => 63,
        _ => return None,
    })
}

/// Strict decode: the text must be a whole number of 4-character
/// groups over the standard alphabet with at most two trailing `=`
/// pads. `None` for anything else (so a payload that merely *looks*
/// binary is never mis-read as base64). Empty input decodes to an
/// empty vector.
pub fn decode(text: &[u8]) -> Option<Vec<u8>> {
    if text.len() % 4 != 0 {
        return None;
    }
    let mut out = Vec::with_capacity(text.len() / 4 * 3);
    let groups = text.len() / 4;
    for (g, chunk) in text.chunks(4).enumerate() {
        let last = g + 1 == groups;
        let pad = chunk.iter().rev().take_while(|c| **c == b'=').count();
        if pad > 2 || (pad > 0 && !last) {
            return None;
        }
        let mut n: u32 = 0;
        for &c in &chunk[..4 - pad] {
            n = (n << 6) | value(c)?;
        }
        n <<= 6 * pad as u32;
        out.push((n >> 16) as u8);
        if pad < 2 {
            out.push((n >> 8) as u8);
        }
        if pad < 1 {
            out.push(n as u8);
        }
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips_every_padding_length() {
        for n in 0..16usize {
            let bytes: Vec<u8> = (0..n as u8)
                .map(|i| i.wrapping_mul(37).wrapping_add(200))
                .collect();
            let text = encode(&bytes);
            assert_eq!(text.len() % 4, 0);
            assert_eq!(decode(text.as_bytes()).as_deref(), Some(bytes.as_slice()));
        }
    }

    #[test]
    fn known_vectors() {
        assert_eq!(encode(b""), "");
        assert_eq!(encode(b"f"), "Zg==");
        assert_eq!(encode(b"fo"), "Zm8=");
        assert_eq!(encode(b"foo"), "Zm9v");
        assert_eq!(encode(b"foobar"), "Zm9vYmFy");
        assert_eq!(decode(b"Zm9vYmE=").as_deref(), Some(&b"fooba"[..]));
    }

    #[test]
    fn fixture_content_prefix_is_a_tga_header() {
        // The staged ASCII fixture's `Content` string begins with
        // this text; the decoded bytes are a TGA header (image type
        // 10 = RLE truecolor, 256 × 256, 24 bpp).
        let bytes = decode(b"AAAKAAAAAAAAAAAAAAEAARgA").unwrap();
        assert_eq!(&bytes[..3], &[0, 0, 10]);
        assert_eq!(u16::from_le_bytes([bytes[12], bytes[13]]), 256);
        assert_eq!(u16::from_le_bytes([bytes[14], bytes[15]]), 256);
        assert_eq!(bytes[16], 24);
    }

    #[test]
    fn rejects_non_base64() {
        assert!(decode(b"abc").is_none());
        assert!(decode(b"ab=c").is_none());
        assert!(decode(b"a===").is_none());
        assert!(decode(&[0x89, b'P', b'N', b'G']).is_none());
        assert!(decode(b"Zm9v!!!!").is_none());
    }
}
