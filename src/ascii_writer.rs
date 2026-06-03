//! ASCII FBX writer — serialise an [`FbxDocument`] back to its
//! human-readable text encoding.
//!
//! Produces output that
//! [`crate::ascii::parse`] reads back into an [`FbxDocument`]
//! interchangeable with the input under the round-trip closure
//!
//! ```text
//!     parse(write(parse(src))) == parse(src)
//! ```
//!
//! i.e. the writer is bit-faithful at the **typed-tree** level after
//! one normalising pass through the parser. (The byte-for-byte ASCII
//! shell isn't a stable target because the same typed tree has many
//! lexically-distinct printings — TAB vs space, trailing comma after
//! the last array element, comment annotations the exporter chose to
//! write, etc.; the parser canonicalises all of those away.)
//!
//! Grammar source: `docs/3d/fbx/fbx-ascii-grammar.md` (observer-derived
//! from `docs/3d/fbx/fixtures/cubes-ascii-v7500.fbx`; no FBX-
//! implementation source consulted). Highlights this module follows:
//!
//! - §1 / §7a — first line is a banner comment carrying the FBX
//!   version. We emit `; FBX <maj>.<min>.<patch> project file\n`
//!   followed by a `; ----` separator line so the banner is
//!   self-evidently a comment, mirroring the fixture's two-line
//!   header. `[FbxDocument::version]` is split into Maya-style
//!   `MMmm` digits (e.g. `7500 -> 7.5.0`); since the parser keys
//!   `FbxDocument::version` off the inner `FBXVersion: 7500` leaf
//!   inside `FBXHeaderExtension`, the banner version digits are
//!   informational and the parser will reach the same version with
//!   or without them.
//! - §2 — `;` starts an end-of-line comment. We do **not** emit any
//!   non-banner comments; the parser ignores them on the way back in.
//! - §3 — `Key: <value-list>opt { <children> }` (body form) vs
//!   `Key: <value-list>` (leaf form). A node with no children is a
//!   leaf; a node with children opens `{` on the same logical line as
//!   the key per the worked examples (`FBXHeaderExtension:  {`,
//!   `SceneInfo: "SceneInfo::GlobalInfo", "UserData" {`).
//! - §3a observed spacing quirk — empty value-list bodies render with
//!   **two spaces** between `:` and `{`
//!   (`FBXHeaderExtension:  {`); non-empty value-list bodies render
//!   with a **single space** after the last value
//!   (`Material: 1, "Material::Mat", "" {`). We reproduce both
//!   shapes so the output matches the SDK's idiom.
//! - §4 — indentation by TAB, one tab per nesting depth. We emit one
//!   `\t` per `depth` for every line that opens a new node and the
//!   matching close-brace line.
//! - §5 — scalar lexical forms:
//!     * Integer: `1003`, `-1`, etc. (Rust's default integer
//!       formatter — no thousands separator, optional leading `-`.)
//!     * Double: full f64 precision. We use `{:?}` for `f64` /
//!       `{:?}` for `f32` so the printed form round-trips through
//!       Rust's float-from-str (i.e. `0.8_f64.parse::<f64>()` after
//!       formatting recovers `0.8`). Rust's `{:?}` for floats already
//!       prints the shortest representation that round-trips per IEEE
//!       754, which is exactly the property we need.
//!     * Quoted string: `"..."` with bytes copied through verbatim
//!       (grammar §5 notes backslashes appear LITERALLY un-escaped in
//!       path strings, so we do not introduce any backslash escaping
//!       on output — the parser likewise does not interpret any
//!       escape sequence). Strings carrying an embedded `"` or
//!       newline are unrepresentable in ASCII per the grammar (the
//!       parser rejects either form on input); we surface the
//!       condition as `Error::invalid` rather than emit a
//!       silently-broken file.
//!     * Bare boolean: `T` (true) / `F` (false), un-quoted, observed
//!       as a lone capital letter token (grammar §5).
//! - §6 — typed array shorthand `Key: *N { a: v1,v2,... }`. We emit
//!   the count `*N` on the key line, the body `{`, then a single
//!   `\t`-indented `a:` line carrying every element separated by a
//!   plain `,` (no trailing space — matching the fixture's
//!   `0,1,3,-3,...` shape), and finally a `\t`-indented `}` line.
//!   Empty arrays render `*0 { a: }`.
//!
//! # Type-narrowing on the round-trip
//!
//! Binary FBX has 9 numeric scalar variants
//! ([`FbxProperty::I16`] / [`FbxProperty::I32`] / [`FbxProperty::I64`]
//! / [`FbxProperty::F32`] / [`FbxProperty::F64`]); ASCII only has
//! "integer" and "float" token shapes. A document carrying
//! [`FbxProperty::I32(7)`] therefore round-trips back through the
//! ASCII parser as [`FbxProperty::I64(7)`]. The
//! `parse(write(parse(...)))` closure is well-defined because the
//! second parse normalises both forms identically.
//!
//! Similarly, typed-arrays narrow on the parser side: an
//! [`FbxProperty::I32Array`] writes back as a numeric `a:` list and
//! re-parses to [`FbxProperty::I32Array`] when every element fits, or
//! [`FbxProperty::I64Array`] when any overflows. The writer does not
//! need to know which variant the round-trip will land on; it just
//! prints the integers verbatim.

use oxideav_mesh3d::{Error, Result};

use crate::binary::{FbxDocument, FbxNode, FbxProperty};

/// Tunable knobs for [`write_ascii_document_with_options`]. Mirrors
/// the binary writer's [`crate::writer::WriterOptions`] shape, so
/// callers can switch encodings at the entry point without restructuring.
#[derive(Clone, Debug)]
pub struct AsciiWriterOptions {
    /// When `true`, prepend the two-line banner
    ///
    /// ```text
    ///   ; FBX <maj>.<min>.<patch> project file
    ///   ; ----------------------------------------------------
    /// ```
    ///
    /// at the top of the output. Required for files that will be
    /// routed back through [`crate::ascii::parse`] without manual
    /// version overrides — [`crate::decoder`] uses the banner as its
    /// ASCII-vs-binary sniffer ([`crate::ascii::is_ascii_fbx`]
    /// requires the banner to recognise the file). Default `true`.
    pub emit_banner: bool,
}

impl Default for AsciiWriterOptions {
    fn default() -> Self {
        Self { emit_banner: true }
    }
}

impl AsciiWriterOptions {
    /// Builder helper — toggle the `; FBX <version>` banner.
    pub fn emit_banner(mut self, on: bool) -> Self {
        self.emit_banner = on;
        self
    }
}

/// Serialise an [`FbxDocument`] to ASCII bytes that
/// [`crate::ascii::parse`] reads back into an equivalent document
/// under the parse-write-parse round-trip closure (see module docs).
///
/// The output is valid UTF-8 by construction: identifiers stay
/// ASCII, scalar numbers stay ASCII, and string-property bytes are
/// copied through verbatim (grammar §1 — input files are UTF-8 and
/// the parser treats string bodies as opaque bytes; multi-byte
/// UTF-8 sequences in strings round-trip without re-encoding).
pub fn write_ascii_document(doc: &FbxDocument) -> Result<Vec<u8>> {
    write_ascii_document_with_options(doc, &AsciiWriterOptions::default())
}

/// Like [`write_ascii_document`] but parameterised by
/// [`AsciiWriterOptions`].
pub fn write_ascii_document_with_options(
    doc: &FbxDocument,
    opts: &AsciiWriterOptions,
) -> Result<Vec<u8>> {
    let mut out = Vec::new();
    if opts.emit_banner {
        write_banner(doc.version, &mut out);
    }
    // Every child of the synthetic `FbxDocument::root` is a top-level
    // section node (FBXHeaderExtension / GlobalSettings / Objects /
    // Connections / Takes / ...). Each renders at depth 0.
    for child in &doc.root.children {
        write_node(child, &mut out, 0)?;
    }
    Ok(out)
}

/// Emit the two-line banner per grammar §1.
///
/// Version digits are derived from `version` as Maya's `MMmm` packing
/// (e.g. `7500 -> 7.5.0`, `7100 -> 7.1.0`, `6100 -> 6.1.0`). The
/// parser sources the canonical version from the inner
/// `FBXHeaderExtension { FBXVersion }` leaf, not from the banner, so
/// the banner is informational; we still try to print sensible digits
/// so a human reader sees the right version.
fn write_banner(version: u32, out: &mut Vec<u8>) {
    let major = version / 1000;
    let minor = (version / 100) % 10;
    let patch = version % 100;
    // Banner line.
    out.extend_from_slice(b"; FBX ");
    out.extend_from_slice(major.to_string().as_bytes());
    out.push(b'.');
    out.extend_from_slice(minor.to_string().as_bytes());
    out.push(b'.');
    out.extend_from_slice(patch.to_string().as_bytes());
    out.extend_from_slice(b" project file\n");
    // Decorative separator — also a comment.
    out.extend_from_slice(b"; ----------------------------------------------------\n");
}

/// Recursive node printer. `depth` is the nesting level (0 for
/// top-level section nodes); each line of the node is prefixed with
/// `depth` TAB characters per §4.
fn write_node(node: &FbxNode, out: &mut Vec<u8>, depth: usize) -> Result<()> {
    // §6 typed-array shorthand fires when a node carries a single
    // numeric-array property and no children. Every binary-side
    // array variant (F32 / F64 / I32 / I64 / Bool) is representable;
    // the ASCII grammar only distinguishes "integer" vs "float"
    // tokens, so the writer chooses the shape based on the variant
    // and the parser narrows on re-read per the module docs.
    if node.children.is_empty()
        && node.properties.len() == 1
        && is_array_property(&node.properties[0])
    {
        return write_array_node(node, out, depth);
    }

    // Body form vs leaf: the §3 distinction. Body iff `children` is
    // non-empty. (A node with no children, regardless of how many
    // property values it carries, is a leaf — the fixture shows e.g.
    // `LocalTime: 1924423250,230930790000` as a leaf with two
    // values; `Material: <uid>, "Material::Mat", "" {` as a body with
    // three values.)
    let has_body = !node.children.is_empty();

    // `<TAB>...<key>:` prefix.
    indent(out, depth);
    out.extend_from_slice(node.name.as_bytes());
    out.push(b':');

    // Render the value-list.
    if node.properties.is_empty() {
        if has_body {
            // §3a quirk — empty value-list bodies have two spaces
            // between the colon and the `{`.
            out.extend_from_slice(b"  {");
        }
        // Empty leaf is just `Key:` (no trailing space).
    } else {
        // Non-empty value-list — single space after the colon and
        // single space after the last value if a body follows.
        out.push(b' ');
        for (i, prop) in node.properties.iter().enumerate() {
            if i > 0 {
                // Grammar §3 — comma-separated. The fixture writes a
                // single space after each comma; we do the same.
                out.extend_from_slice(b", ");
            }
            write_scalar(prop, out)?;
        }
        if has_body {
            out.extend_from_slice(b" {");
        }
    }
    out.push(b'\n');

    if has_body {
        for child in &node.children {
            write_node(child, out, depth + 1)?;
        }
        // Closing brace — same indentation as the opening line.
        indent(out, depth);
        out.extend_from_slice(b"}\n");
    }
    Ok(())
}

/// Specialised renderer for the §6 typed-array shorthand
/// `Key: *N { a: v1,v2,... }`. The element count is taken from the
/// array property's length; the body is always one `\t`-indented
/// `a: <elements>` line plus the matching closing brace.
fn write_array_node(node: &FbxNode, out: &mut Vec<u8>, depth: usize) -> Result<()> {
    indent(out, depth);
    out.extend_from_slice(node.name.as_bytes());
    out.push(b':');
    out.push(b' ');
    out.push(b'*');
    let count = array_len(&node.properties[0]);
    out.extend_from_slice(count.to_string().as_bytes());
    out.extend_from_slice(b" {\n");
    // Body: one extra TAB beyond the owning node's depth (grammar §4
    // says array continuation lines indent one level deeper than the
    // owning node).
    indent(out, depth + 1);
    out.extend_from_slice(b"a:");
    if count > 0 {
        out.push(b' ');
        write_array_elements(&node.properties[0], out)?;
    }
    out.push(b'\n');
    // Closing brace — same depth as the opening line.
    indent(out, depth);
    out.extend_from_slice(b"}\n");
    Ok(())
}

/// `true` when the property is one of the typed-array variants the
/// grammar §6 shorthand can carry. The §6 form is observed for
/// numeric arrays only; the binary-side [`FbxProperty::BoolArray`] is
/// also numeric-shaped (each element is `T` / `F` or `0` / `1`), so
/// we admit it too — the parser side then reconstructs an
/// [`FbxProperty::I32Array`] of `0`/`1` values, which the
/// [`crate::geometry`] / [`crate::material`] consumers don't reach
/// (no binary array property currently surfaces as `BoolArray`
/// downstream), but the round-trip stays well-defined.
fn is_array_property(p: &FbxProperty) -> bool {
    matches!(
        p,
        FbxProperty::F32Array(_)
            | FbxProperty::F64Array(_)
            | FbxProperty::I32Array(_)
            | FbxProperty::I64Array(_)
            | FbxProperty::BoolArray(_)
    )
}

fn array_len(p: &FbxProperty) -> usize {
    match p {
        FbxProperty::F32Array(v) => v.len(),
        FbxProperty::F64Array(v) => v.len(),
        FbxProperty::I32Array(v) => v.len(),
        FbxProperty::I64Array(v) => v.len(),
        FbxProperty::BoolArray(v) => v.len(),
        _ => 0,
    }
}

/// Write the comma-separated body of a typed-array. The elements
/// land on a single line (no per-element newlines) — the grammar
/// allows but does not require line-breaking and the fixture writes
/// everything on one logical `a:` line.
fn write_array_elements(p: &FbxProperty, out: &mut Vec<u8>) -> Result<()> {
    match p {
        FbxProperty::F32Array(v) => {
            for (i, x) in v.iter().enumerate() {
                if i > 0 {
                    out.push(b',');
                }
                write_f64(*x as f64, out);
            }
        }
        FbxProperty::F64Array(v) => {
            for (i, x) in v.iter().enumerate() {
                if i > 0 {
                    out.push(b',');
                }
                write_f64(*x, out);
            }
        }
        FbxProperty::I32Array(v) => {
            for (i, x) in v.iter().enumerate() {
                if i > 0 {
                    out.push(b',');
                }
                out.extend_from_slice(x.to_string().as_bytes());
            }
        }
        FbxProperty::I64Array(v) => {
            for (i, x) in v.iter().enumerate() {
                if i > 0 {
                    out.push(b',');
                }
                out.extend_from_slice(x.to_string().as_bytes());
            }
        }
        FbxProperty::BoolArray(v) => {
            // Print as `0` / `1` numerals. The parser admits any
            // integer token, so `0`/`1` are the most compact form
            // that round-trips cleanly.
            for (i, x) in v.iter().enumerate() {
                if i > 0 {
                    out.push(b',');
                }
                out.push(if *x { b'1' } else { b'0' });
            }
        }
        _ => {
            return Err(Error::invalid(format!(
                "ascii FBX writer: internal — write_array_elements called for non-array variant {p:?}"
            )));
        }
    }
    Ok(())
}

/// Write one scalar property in its ASCII form per grammar §5.
fn write_scalar(p: &FbxProperty, out: &mut Vec<u8>) -> Result<()> {
    match p {
        FbxProperty::I16(v) => out.extend_from_slice(v.to_string().as_bytes()),
        FbxProperty::I32(v) => out.extend_from_slice(v.to_string().as_bytes()),
        FbxProperty::I64(v) => out.extend_from_slice(v.to_string().as_bytes()),
        FbxProperty::F32(v) => write_f64(*v as f64, out),
        FbxProperty::F64(v) => write_f64(*v, out),
        FbxProperty::Bool(v) => out.push(if *v { b'T' } else { b'F' }),
        FbxProperty::String(bytes) => write_string(bytes, out)?,
        // Binary `R` blobs have no ASCII grammar form per §5; the
        // fixture never carries a raw-binary property at the node
        // level (only `Video.Content` does, and the round-200 ASCII
        // reader doesn't synthesise `Raw` properties either — the
        // ASCII path stops at the `Content:` byte stream level). We
        // surface the unsupported case cleanly so callers see why
        // the doc didn't round-trip.
        FbxProperty::Raw(_) => {
            return Err(Error::invalid(
                "ascii FBX writer: `Raw` blob has no ASCII representation \
                 (grammar §5 only defines string / integer / float / boolean \
                 / UID / time-pair scalar forms — binary-only `R` properties \
                 cannot round-trip through ASCII)",
            ));
        }
        // Arrays at the scalar slot are a writer bug — the dispatcher
        // in `write_node` should have routed them through
        // `write_array_node`.
        FbxProperty::F32Array(_)
        | FbxProperty::F64Array(_)
        | FbxProperty::I32Array(_)
        | FbxProperty::I64Array(_)
        | FbxProperty::BoolArray(_) => {
            return Err(Error::invalid(format!(
                "ascii FBX writer: internal — array variant {p:?} reached \
                 the scalar dispatcher (a non-array property accompanies \
                 the array on the same node, which §6 typed-array shorthand \
                 cannot express)"
            )));
        }
    }
    Ok(())
}

/// Print an f64 with full round-trip precision. Rust's `{:?}` for
/// floats prints the shortest string that recovers the original value
/// when parsed back, which matches the parser's
/// `parse_numeric_token` → `f64::parse` path: every value the writer
/// emits parses back to the same f64. We also normalise integer-valued
/// floats (e.g. `1.0`) to a form that carries a `.` so the parser's
/// "has_dot ⇒ Float" lexer classifies the token as a float rather
/// than promoting it to `Int` and losing the variant distinction in
/// the round-trip closure.
fn write_f64(v: f64, out: &mut Vec<u8>) {
    let s = format!("{v:?}");
    // Rust's `{:?}` prints `1.0`, `-0.5`, `1e308` etc. — every form
    // contains either `.` or `e`/`E`/`inf`/`NaN`. The lexer requires
    // `.` or `e`/`E` to classify as float; `inf`/`NaN` are out of
    // grammar (parser would reject them as identifiers). We let those
    // edge cases write through as-is — they re-parse as identifiers
    // and surface a clear parser error at re-read time rather than
    // silently corrupting.
    out.extend_from_slice(s.as_bytes());
}

/// Quote a string per grammar §5. Backslashes are passed through
/// literally (the parser does not interpret any escape sequence — see
/// the §5 path-string example). Bytes that the grammar cannot encode
/// (interior `"` would terminate the string; newline would terminate
/// the line) are surfaced as `Error::invalid`.
fn write_string(bytes: &[u8], out: &mut Vec<u8>) -> Result<()> {
    if bytes.contains(&b'"') {
        return Err(Error::invalid(
            "ascii FBX writer: string property contains a `\"` byte — \
             grammar §5 has no escape mechanism; cannot render",
        ));
    }
    if bytes.contains(&b'\n') || bytes.contains(&b'\r') {
        return Err(Error::invalid(
            "ascii FBX writer: string property contains a newline byte — \
             grammar §5 strings must stay on a single line",
        ));
    }
    out.push(b'"');
    out.extend_from_slice(bytes);
    out.push(b'"');
    Ok(())
}

fn indent(out: &mut Vec<u8>, depth: usize) {
    for _ in 0..depth {
        out.push(b'\t');
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ascii;
    use crate::binary::{FbxDocument, FbxNode, FbxProperty};

    /// Structural equality of two [`FbxNode`] trees — recurses on
    /// children, checks per-property equality through
    /// [`FbxProperty::PartialEq`] which is already derived.
    fn nodes_equal(a: &FbxNode, b: &FbxNode) -> bool {
        a.name == b.name
            && a.properties == b.properties
            && a.children.len() == b.children.len()
            && a.children
                .iter()
                .zip(b.children.iter())
                .all(|(x, y)| nodes_equal(x, y))
    }

    /// Pretty-printer for an [`FbxNode`] so a round-trip failure shows
    /// the disagreement at a glance. Used only in test panic messages.
    fn debug_tree(n: &FbxNode, depth: usize) -> String {
        let mut out = String::new();
        for _ in 0..depth {
            out.push_str("  ");
        }
        out.push_str(&format!("{} {:?}\n", n.name, n.properties));
        for c in &n.children {
            out.push_str(&debug_tree(c, depth + 1));
        }
        out
    }

    /// Round-trip closure: typed tree must equal itself after one
    /// normalising pass through the ASCII writer + parser.
    fn assert_ast_round_trips(doc: &FbxDocument) {
        let bytes = write_ascii_document(doc).expect("write_ascii_document");
        let s = std::str::from_utf8(&bytes).expect("output is UTF-8");
        let parsed = ascii::parse(&bytes)
            .unwrap_or_else(|e| panic!("re-parse failed: {e}\n----\n{s}\n----"));
        assert_eq!(parsed.version, doc.version, "version mismatch:\n{s}");
        if !nodes_equal(&parsed.root, &doc.root) {
            panic!(
                "root tree mismatch\n--- expected:\n{}\n--- got:\n{}\n--- bytes:\n{s}",
                debug_tree(&doc.root, 0),
                debug_tree(&parsed.root, 0)
            );
        }
    }

    fn doc_with_root(version: u32, children: Vec<FbxNode>) -> FbxDocument {
        FbxDocument {
            version,
            root: FbxNode {
                name: String::new(),
                properties: Vec::new(),
                children,
            },
        }
    }

    #[test]
    fn banner_only_doc_round_trips_minimal_shell() {
        // The minimal round-trippable shell is a FBXHeaderExtension
        // carrying just the FBXVersion leaf, which is how
        // `ascii::parse` keys `FbxDocument::version`.
        let doc = doc_with_root(
            7500,
            vec![FbxNode {
                name: "FBXHeaderExtension".to_string(),
                properties: vec![],
                children: vec![FbxNode {
                    name: "FBXVersion".to_string(),
                    properties: vec![FbxProperty::I64(7500)],
                    children: vec![],
                }],
            }],
        );
        assert_ast_round_trips(&doc);
    }

    #[test]
    fn banner_carries_version_digits() {
        let doc = doc_with_root(7500, vec![]);
        let bytes = write_ascii_document(&doc).unwrap();
        let s = std::str::from_utf8(&bytes).unwrap();
        assert!(s.starts_with("; FBX 7.5.0 project file\n"), "{s:?}");
        // 6100 and 7100 are documented in the GAP-TRACKER §C version
        // table — sanity check the digit split too.
        for (ver, digits) in [(6100u32, "6.1.0"), (7100, "7.1.0"), (7700, "7.7.0")] {
            let d = doc_with_root(ver, vec![]);
            let b = write_ascii_document(&d).unwrap();
            let head = std::str::from_utf8(&b[..32]).unwrap();
            assert!(head.contains(digits), "{head:?} missing {digits}");
        }
    }

    #[test]
    fn leaf_nodes_with_scalar_values_round_trip() {
        // Integers, doubles, strings, bare booleans — every scalar
        // shape grammar §5 enumerates.
        let doc = doc_with_root(
            7500,
            vec![
                FbxNode {
                    name: "FBXHeaderExtension".to_string(),
                    properties: vec![],
                    children: vec![FbxNode {
                        name: "FBXVersion".to_string(),
                        properties: vec![FbxProperty::I64(7500)],
                        children: vec![],
                    }],
                },
                FbxNode {
                    name: "Header".to_string(),
                    properties: vec![],
                    children: vec![
                        FbxNode {
                            name: "Version".to_string(),
                            properties: vec![FbxProperty::I64(1003)],
                            children: vec![],
                        },
                        FbxNode {
                            name: "Creator".to_string(),
                            properties: vec![FbxProperty::String(
                                b"oxideav-fbx round 213".to_vec(),
                            )],
                            children: vec![],
                        },
                        FbxNode {
                            name: "Pi".to_string(),
                            properties: vec![FbxProperty::F64(0.800000011920929)],
                            children: vec![],
                        },
                        FbxNode {
                            name: "Shading".to_string(),
                            properties: vec![FbxProperty::Bool(true)],
                            children: vec![],
                        },
                        FbxNode {
                            name: "Inverted".to_string(),
                            properties: vec![FbxProperty::Bool(false)],
                            children: vec![],
                        },
                    ],
                },
            ],
        );
        assert_ast_round_trips(&doc);
    }

    #[test]
    fn object_opening_line_three_values_round_trips() {
        // §7c shape: TypeKeyword: <UID>, "ClassTag::Name", "SubType" {
        let doc = doc_with_root(
            7500,
            vec![
                FbxNode {
                    name: "FBXHeaderExtension".to_string(),
                    properties: vec![],
                    children: vec![FbxNode {
                        name: "FBXVersion".to_string(),
                        properties: vec![FbxProperty::I64(7500)],
                        children: vec![],
                    }],
                },
                FbxNode {
                    name: "Objects".to_string(),
                    properties: vec![],
                    children: vec![FbxNode {
                        name: "Material".to_string(),
                        properties: vec![
                            FbxProperty::I64(2_359_823_919_504),
                            FbxProperty::String(b"Material::Mat_Green".to_vec()),
                            FbxProperty::String(b"".to_vec()),
                        ],
                        children: vec![FbxNode {
                            name: "Version".to_string(),
                            properties: vec![FbxProperty::I64(102)],
                            children: vec![],
                        }],
                    }],
                },
            ],
        );
        assert_ast_round_trips(&doc);
    }

    #[test]
    fn typed_array_floats_and_ints_round_trip() {
        let doc = doc_with_root(
            7500,
            vec![
                FbxNode {
                    name: "FBXHeaderExtension".to_string(),
                    properties: vec![],
                    children: vec![FbxNode {
                        name: "FBXVersion".to_string(),
                        properties: vec![FbxProperty::I64(7500)],
                        children: vec![],
                    }],
                },
                FbxNode {
                    name: "Geometry".to_string(),
                    properties: vec![
                        FbxProperty::I64(1),
                        FbxProperty::String(b"Geometry::".to_vec()),
                        FbxProperty::String(b"Mesh".to_vec()),
                    ],
                    children: vec![
                        FbxNode {
                            name: "Vertices".to_string(),
                            properties: vec![FbxProperty::F64Array(vec![
                                -0.5, -0.5, 0.5, 0.5, -0.5, 0.5,
                            ])],
                            children: vec![],
                        },
                        FbxNode {
                            name: "PolygonVertexIndex".to_string(),
                            properties: vec![FbxProperty::I32Array(vec![0, 1, 3, -3])],
                            children: vec![],
                        },
                    ],
                },
            ],
        );
        assert_ast_round_trips(&doc);
    }

    #[test]
    fn typed_array_overflowing_i32_round_trips_via_i64() {
        // KeyTime arrays carry KTime ticks past i32::MAX — the parser
        // surfaces them as I64Array, and we must write them back in
        // the same numeric form so the second parse re-narrows the
        // same way.
        let doc = doc_with_root(
            7500,
            vec![
                FbxNode {
                    name: "FBXHeaderExtension".to_string(),
                    properties: vec![],
                    children: vec![FbxNode {
                        name: "FBXVersion".to_string(),
                        properties: vec![FbxProperty::I64(7500)],
                        children: vec![],
                    }],
                },
                FbxNode {
                    name: "KeyTime".to_string(),
                    properties: vec![FbxProperty::I64Array(vec![1, 2, 230_930_790_000])],
                    children: vec![],
                },
            ],
        );
        assert_ast_round_trips(&doc);
    }

    #[test]
    fn empty_typed_array_narrows_to_i32array_via_grammar_ambiguity() {
        // An empty `*0 { a: }` body carries zero numeric tokens, so
        // the parser's "any element a float?" classifier has no
        // evidence to promote the array to `F64Array`. ASCII grammar
        // §6 has no syntactic distinction between an empty
        // float-array and an empty integer-array — both render
        // identically (`*0 { a: }`). The writer therefore CANNOT
        // round-trip an empty `F64Array` back as an `F64Array`; the
        // ascii path normalises it to `I32Array([])`. We document
        // the behaviour here so the loss-on-narrowing is explicit
        // rather than surprising callers downstream.
        let bytes = write_ascii_document(&doc_with_root(
            7500,
            vec![FbxNode {
                name: "Empty".to_string(),
                properties: vec![FbxProperty::F64Array(vec![])],
                children: vec![],
            }],
        ))
        .unwrap();
        let s = std::str::from_utf8(&bytes).unwrap();
        assert!(s.contains("Empty: *0 {\n\ta:\n}\n"), "{s:?}");
        let reparsed = ascii::parse(&bytes).unwrap();
        let e = reparsed.root.child("Empty").unwrap();
        match &e.properties[0] {
            FbxProperty::I32Array(v) => assert!(v.is_empty()),
            other => panic!("expected I32Array([]) (grammar ambiguity), got {other:?}"),
        }

        // An empty `I32Array` does round-trip identically (since both
        // map to the same `I32Array([])`). Embed the version leaf so
        // the round-trip closure agrees on `FbxDocument::version`
        // (the parser defaults to 7400 when `FBXHeaderExtension`
        // is absent).
        let doc = doc_with_root(
            7500,
            vec![
                FbxNode {
                    name: "FBXHeaderExtension".to_string(),
                    properties: vec![],
                    children: vec![FbxNode {
                        name: "FBXVersion".to_string(),
                        properties: vec![FbxProperty::I64(7500)],
                        children: vec![],
                    }],
                },
                FbxNode {
                    name: "Empty".to_string(),
                    properties: vec![FbxProperty::I32Array(vec![])],
                    children: vec![],
                },
            ],
        );
        assert_ast_round_trips(&doc);
    }

    #[test]
    fn empty_value_list_body_uses_two_spaces_per_quirk() {
        // Grammar §3a observed spacing quirk — an empty value-list
        // body renders `Key:  {` (two spaces). Spot-check the output
        // bytes so a future change that drops one of the spaces is
        // caught directly (the parser tolerates either, but matching
        // the SDK idiom is documented behaviour).
        let doc = doc_with_root(
            7500,
            vec![FbxNode {
                name: "FBXHeaderExtension".to_string(),
                properties: vec![],
                children: vec![FbxNode {
                    name: "FBXVersion".to_string(),
                    properties: vec![FbxProperty::I64(7500)],
                    children: vec![],
                }],
            }],
        );
        let bytes = write_ascii_document(&doc).unwrap();
        let s = std::str::from_utf8(&bytes).unwrap();
        assert!(
            s.contains("FBXHeaderExtension:  {"),
            "expected two-space `:` `{{` shape in:\n{s}"
        );
    }

    #[test]
    fn nested_children_indent_with_tabs() {
        // Worked example: `Header { Inner { Leaf: 1 } }` → three
        // levels of TAB indentation matching the parser's expected
        // form.
        let doc = doc_with_root(
            7500,
            vec![
                FbxNode {
                    name: "FBXHeaderExtension".to_string(),
                    properties: vec![],
                    children: vec![FbxNode {
                        name: "FBXVersion".to_string(),
                        properties: vec![FbxProperty::I64(7500)],
                        children: vec![],
                    }],
                },
                FbxNode {
                    name: "Outer".to_string(),
                    properties: vec![],
                    children: vec![FbxNode {
                        name: "Inner".to_string(),
                        properties: vec![],
                        children: vec![FbxNode {
                            name: "Leaf".to_string(),
                            properties: vec![FbxProperty::I64(1)],
                            children: vec![],
                        }],
                    }],
                },
            ],
        );
        let bytes = write_ascii_document(&doc).unwrap();
        let s = std::str::from_utf8(&bytes).unwrap();
        // Top-level node — zero TABs before the key.
        assert!(s.contains("\nOuter:  {\n"), "{s}");
        // Inner — one TAB.
        assert!(s.contains("\n\tInner:  {\n"), "{s}");
        // Leaf — two TABs.
        assert!(s.contains("\n\t\tLeaf: 1\n"), "{s}");
        assert_ast_round_trips(&doc);
    }

    #[test]
    fn utf8_multibyte_string_round_trips() {
        // Grammar §1 — input is UTF-8; the fixture's
        // `Model::Куб1` Cyrillic name is the canonical worked
        // example. Multi-byte sequences must round-trip verbatim.
        let doc = doc_with_root(
            7500,
            vec![
                FbxNode {
                    name: "FBXHeaderExtension".to_string(),
                    properties: vec![],
                    children: vec![FbxNode {
                        name: "FBXVersion".to_string(),
                        properties: vec![FbxProperty::I64(7500)],
                        children: vec![],
                    }],
                },
                FbxNode {
                    name: "Objects".to_string(),
                    properties: vec![],
                    children: vec![FbxNode {
                        name: "Model".to_string(),
                        properties: vec![
                            FbxProperty::I64(1),
                            FbxProperty::String("Model::Куб1".as_bytes().to_vec()),
                            FbxProperty::String(b"Mesh".to_vec()),
                        ],
                        children: vec![],
                    }],
                },
            ],
        );
        assert_ast_round_trips(&doc);
    }

    #[test]
    fn backslash_in_string_passes_through_literally() {
        // Grammar §5 observed: backslashes appear literally
        // un-escaped in path strings (`"U:\Some\Absolute\Path\cubes.fbx"`).
        let doc = doc_with_root(
            7500,
            vec![
                FbxNode {
                    name: "FBXHeaderExtension".to_string(),
                    properties: vec![],
                    children: vec![FbxNode {
                        name: "FBXVersion".to_string(),
                        properties: vec![FbxProperty::I64(7500)],
                        children: vec![],
                    }],
                },
                FbxNode {
                    name: "Documents".to_string(),
                    properties: vec![],
                    children: vec![FbxNode {
                        name: "Filename".to_string(),
                        properties: vec![FbxProperty::String(
                            br"U:\Some\Absolute\Path\cubes_with_names.fbx".to_vec(),
                        )],
                        children: vec![],
                    }],
                },
            ],
        );
        assert_ast_round_trips(&doc);
    }

    #[test]
    fn string_with_embedded_quote_is_rejected() {
        let doc = doc_with_root(
            7500,
            vec![FbxNode {
                name: "Bad".to_string(),
                properties: vec![FbxProperty::String(br#"has " inside"#.to_vec())],
                children: vec![],
            }],
        );
        let err = write_ascii_document(&doc).expect_err("should reject embedded quote");
        let msg = format!("{err}");
        assert!(msg.contains("quote") || msg.contains("`\"`"), "{msg}");
    }

    #[test]
    fn raw_blob_is_rejected_cleanly() {
        let doc = doc_with_root(
            7500,
            vec![FbxNode {
                name: "Bad".to_string(),
                properties: vec![FbxProperty::Raw(vec![0, 1, 2, 3])],
                children: vec![],
            }],
        );
        let err = write_ascii_document(&doc).expect_err("should reject Raw blob");
        let msg = format!("{err}");
        assert!(msg.contains("Raw"), "{msg}");
    }

    #[test]
    fn fixture_round_trip_closure_holds_via_ascii_writer() {
        // Cubes-ascii-v7500 fixture: parse → write_ascii → parse,
        // compare AST. The fixture exercises every §7 top-level
        // section, three-property object openings, typed arrays of
        // both float and int variants, Cyrillic identifiers, backslash
        // paths — i.e. it is the broadest single test we can run.
        const FIXTURE: &[u8] = include_bytes!("../tests/fixtures/cubes-ascii-v7500.fbx");
        let doc = ascii::parse(FIXTURE).expect("fixture parses");
        assert_ast_round_trips(&doc);
    }
}
