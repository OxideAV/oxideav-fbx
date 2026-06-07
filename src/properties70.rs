//! `Properties70` `P`-record decoder.
//!
//! Per the observer-derived `docs/3d/fbx/fbx-binary-properties70.md`
//! §4 grammar (and its ASCII counterpart in
//! `docs/3d/fbx/fbx-ascii-grammar.md` §8), a `Properties70` node is a
//! container whose children are all named `P` and follow the
//! five-field shape:
//!
//! ```text
//! P [NumProperties = 4 + valueCount]
//!    prop0 : S  name      e.g. "DiffuseColor"
//!    prop1 : S  typeName  e.g. "ColorRGB"
//!    prop2 : S  label     e.g. "Color"
//!    prop3 : S  flags     e.g. ""   ("A" animatable, "U" user)
//!    prop4.. : typed value(s)       — 0 (Compound), 1 (scalar), or 3 (vector/color)
//! ```
//!
//! The number of trailing value props is `NumProperties − 4` per the
//! docs §4: 0 for `Compound` (and any value-less property), 1 for
//! scalars (`int` / `enum` → `I`, `double` / `Number` → `D`,
//! `KTime` / `ULongLong` → `L`, `KString` / `DateTime` → `S`,
//! `bool` → `C`), and 3 for triples (`ColorRGB` / `Color` /
//! `Vector3D` / `Vector` / `Lcl Translation` / `Lcl Scaling`).
//!
//! ASCII and binary FBX render the same record using different
//! type-tag conventions; the typed [`PValue`] decoded here is
//! identical to what the ASCII grammar §8 documents.

use std::collections::HashMap;

use crate::binary::{FbxNode, FbxProperty};

/// Decoded typed value of one `P` record.
///
/// Variants mirror the value-count branches documented in
/// `docs/3d/fbx/fbx-binary-properties70.md` §4 (`Compound` / scalar /
/// triple) plus the wire type codes documented in the same file's §3
/// (binary) and `fbx-ascii-grammar.md` §5 (ASCII).
#[derive(Clone, Debug, PartialEq)]
pub enum PValue {
    /// No value field — `type == "Compound"` (or any value-less P).
    Compound,
    /// Single `I` (`int` / `enum`) value.
    Int(i32),
    /// Single `L` (`KTime` / `ULongLong`) value.
    Long(i64),
    /// Single `D` / `F` (`double` / `Number` / `float`) value.
    Double(f64),
    /// Single `C` (`bool`) value.
    Bool(bool),
    /// Single `S` (`KString` / `DateTime` / `object`) value.
    Str(String),
    /// Triple of doubles (`Vector3D` / `Vector` / `Lcl Translation`
    /// / `Lcl Scaling` / `ColorRGB` / `Color`). The value-count branch
    /// docs §4 calls *"3 for vectors/colours"*.
    Vec3([f64; 3]),
    /// Catch-all for trailing-value records whose shape we don't
    /// recognise: keeps the raw `FbxProperty` list so callers can
    /// fall back to ad-hoc decoding without re-walking the document.
    Other(Vec<FbxProperty>),
}

/// One `P` record after typed decoding.
///
/// `type_name` is the wire string from prop1 (`"int"`, `"double"`,
/// `"ColorRGB"`, `"Lcl Translation"`, `"Compound"`, …). `label` and
/// `flags` are the docs §4 prop2 + prop3 strings. `value` is the
/// typed payload assembled from the trailing `(NumProperties − 4)`
/// value props.
#[derive(Clone, Debug, PartialEq)]
pub struct PRecord {
    pub type_name: String,
    pub label: String,
    pub flags: String,
    pub value: PValue,
}

/// A `Properties70` block decoded into a name → record map.
///
/// Names are unique within a single `Properties70` block by FBX
/// convention (the docs §4 sample shows each `P` record's prop0 as a
/// distinct property identifier); when an FBX file does repeat a
/// name (rare; observed only for some exporter-emitted compound
/// substructures), this decoder keeps the **last** occurrence — the
/// same last-wins shape exporters use when they want to override a
/// template default.
#[derive(Clone, Debug, Default)]
pub struct PropertyMap {
    inner: HashMap<String, PRecord>,
}

impl PropertyMap {
    /// Decode a `Properties70` node into a typed property map.
    ///
    /// `parent` is the owning element node — *not* the `Properties70`
    /// node itself. The function finds the first `Properties70`
    /// direct child and returns an empty map if the parent has none.
    /// This matches the docs §4 placement — `Properties70` sits as a
    /// direct child of the element (e.g. `Material`, `Model`,
    /// `GlobalSettings`).
    pub fn from_element(parent: &FbxNode) -> Self {
        let Some(props70) = parent.child("Properties70") else {
            return Self::default();
        };
        Self::from_properties70(props70)
    }

    /// Decode a `Properties70` node directly.
    ///
    /// Same shape as [`Self::from_element`] but takes the
    /// `Properties70` node itself, useful when the caller already
    /// holds a reference to it.
    pub fn from_properties70(props70: &FbxNode) -> Self {
        let mut inner = HashMap::new();
        for p in props70.children_named("P") {
            if let Some((name, record)) = decode_p_record(p) {
                inner.insert(name, record);
            }
        }
        Self { inner }
    }

    /// Look up a `P` record by name.
    pub fn get(&self, name: &str) -> Option<&PRecord> {
        self.inner.get(name)
    }

    /// Number of `P` records decoded.
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    /// True when no records were decoded.
    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    // --- typed scalar / vector helpers ---

    /// Pull a `double` / `Number` / `float` factor by name. Accepts
    /// `Int` / `Long` / `Bool` too (so `Opacity: int 1` reads as
    /// `1.0`) — exporters mix these freely.
    pub fn as_f64(&self, name: &str) -> Option<f64> {
        match &self.inner.get(name)?.value {
            PValue::Double(v) => Some(*v),
            PValue::Int(v) => Some(*v as f64),
            PValue::Long(v) => Some(*v as f64),
            PValue::Bool(v) => Some(if *v { 1.0 } else { 0.0 }),
            _ => None,
        }
    }

    /// Pull an `int` / `enum` value by name.
    pub fn as_i32(&self, name: &str) -> Option<i32> {
        match &self.inner.get(name)?.value {
            PValue::Int(v) => Some(*v),
            PValue::Long(v) => Some(*v as i32),
            PValue::Bool(v) => Some(*v as i32),
            _ => None,
        }
    }

    /// Pull a `KTime` / `ULongLong` / `Long` value by name without
    /// loss of precision.
    ///
    /// Per `docs/3d/fbx/fbx-binary-properties70.md` §4, the wire
    /// codes for these typeNames are `L` (int64), so a `KTime`
    /// payload may exceed the i32 range (the doc's sample
    /// `TimeSpanStop = 46_186_158_000` already does) and the
    /// floating-point [`Self::as_f64`] path would lose precision
    /// near the 2^53 boundary. This accessor returns the underlying
    /// [`PValue::Long`] verbatim, widening [`PValue::Int`] /
    /// [`PValue::Bool`] losslessly so an exporter that wires an
    /// otherwise-`KTime` value as `I` (the docs §4 note about older
    /// exporters mixing the integer wire codes) still reads back
    /// correctly.
    ///
    /// Returns `None` for non-numeric records (`Str` / `Vec3` /
    /// `Compound` / `Double` / `Other`).
    pub fn as_i64(&self, name: &str) -> Option<i64> {
        match &self.inner.get(name)?.value {
            PValue::Long(v) => Some(*v),
            PValue::Int(v) => Some(*v as i64),
            PValue::Bool(v) => Some(if *v { 1 } else { 0 }),
            _ => None,
        }
    }

    /// Pull a `bool` value by name. `Int` / `Long` values are
    /// coerced via `!= 0` (FBX `bool` is wire-encoded as `int` in
    /// many older exporters per docs §4).
    pub fn as_bool(&self, name: &str) -> Option<bool> {
        match &self.inner.get(name)?.value {
            PValue::Bool(v) => Some(*v),
            PValue::Int(v) => Some(*v != 0),
            PValue::Long(v) => Some(*v != 0),
            _ => None,
        }
    }

    /// Pull a `KString` / `object` / `DateTime` string by name.
    pub fn as_str(&self, name: &str) -> Option<&str> {
        match &self.inner.get(name)?.value {
            PValue::Str(s) => Some(s.as_str()),
            _ => None,
        }
    }

    /// Pull a `ColorRGB` / `Color` / `Vector3D` / `Vector` triple by
    /// name.
    pub fn as_vec3(&self, name: &str) -> Option<[f64; 3]> {
        match &self.inner.get(name)?.value {
            PValue::Vec3(v) => Some(*v),
            _ => None,
        }
    }

    // --- typeName-discriminating accessors ---
    //
    // The base [`Self::as_vec3`] / [`Self::as_str`] flatten every
    // trailing-value typeName the docs §4 trailing-value table
    // enumerates. The following accessors honour
    // [`PRecord::type_name`] — the typeName string parsed from
    // prop1 — so a caller asking for a `ColorRGB` triple does not
    // accidentally pick up a `Lcl Translation` triple sitting on
    // the same name. Per `docs/3d/fbx/fbx-binary-properties70.md`
    // §4 *"The typeName/label/flags strings carry the semantic
    // type"*, this is the typeName's documented role.

    /// Pull a `ColorRGB` / `Color` triple by name.
    ///
    /// Per `docs/3d/fbx/fbx-binary-properties70.md` §4 trailing-value
    /// rule *"3 for vectors/colours (`ColorRGB`/`Color`/`Vector3D`)"*,
    /// the docs §4 worked sample (`AmbientColor S"ColorRGB" S"Color"
    /// S"" D=0 D=0 D=0`) and the ASCII grammar §8 enumerated typeName
    /// list, `"ColorRGB"` and `"Color"` are the colour-bearing
    /// typeNames in the same family. Returns `None` for records whose
    /// `type_name` falls outside that pair (e.g. a `Vector3D` triple
    /// — those go through [`Self::as_vector3d`]).
    pub fn as_color_rgb(&self, name: &str) -> Option<[f64; 3]> {
        let rec = self.inner.get(name)?;
        if !matches!(rec.type_name.as_str(), "ColorRGB" | "Color") {
            return None;
        }
        match &rec.value {
            PValue::Vec3(v) => Some(*v),
            _ => None,
        }
    }

    /// Pull a `Vector3D` / `Vector` triple by name.
    ///
    /// Per the docs §4 trailing-value rule and the ASCII grammar §8
    /// typeName list, `"Vector3D"` and `"Vector"` carry plain
    /// geometric triples (positions / directions / Euler angles) as
    /// distinct from colour triples (`ColorRGB`/`Color` — see
    /// [`Self::as_color_rgb`]) and transform triples (`Lcl …` — see
    /// [`Self::as_lcl_translation`] et al). The cubes fixture's
    /// `PreRotation` / `PostRotation` / `GeometricTranslation` /
    /// `GeometricRotation` / `GeometricScaling` all wire as
    /// `"Vector3D"`; `"Vector"` is observed as the label string but
    /// is also documented as a typeName variant.
    pub fn as_vector3d(&self, name: &str) -> Option<[f64; 3]> {
        let rec = self.inner.get(name)?;
        if !matches!(rec.type_name.as_str(), "Vector3D" | "Vector") {
            return None;
        }
        match &rec.value {
            PValue::Vec3(v) => Some(*v),
            _ => None,
        }
    }

    /// Pull a `Lcl Translation` triple by name.
    ///
    /// `"Lcl Translation"` is the typeName the docs §4 trailing-value
    /// table calls out explicitly (alongside `"Lcl Scaling"`) as a
    /// triple typeName; the cubes-ascii-v7500.fbx fixture's `Model`
    /// node carries it as `P: "Lcl Translation", "Lcl Translation",
    /// "", "A", -1.04…, 0.998…, -1.043…`. The accessor validates the
    /// typeName so a caller cannot accidentally pick up a `Vector3D`
    /// triple sitting under the same `"Lcl Translation"` name from a
    /// non-standard exporter.
    pub fn as_lcl_translation(&self, name: &str) -> Option<[f64; 3]> {
        self.as_typed_vec3(name, "Lcl Translation")
    }

    /// Pull a `Lcl Rotation` triple (XYZ Euler degrees) by name.
    ///
    /// `"Lcl Rotation"` is listed in the ASCII grammar §8 typeName
    /// enumeration and the binary-doc §4 P-record family; the
    /// cubes-ascii-v7500.fbx fixture has `P: "Lcl Rotation", "Lcl
    /// Rotation", "", "A", 0, 0, 0`. The triple is XYZ Euler in
    /// degrees per the ufbx-doc convention the [`crate::animation`]
    /// module already follows when converting `Lcl Rotation` curves
    /// to quaternions.
    pub fn as_lcl_rotation(&self, name: &str) -> Option<[f64; 3]> {
        self.as_typed_vec3(name, "Lcl Rotation")
    }

    /// Pull a `Lcl Scaling` triple by name.
    ///
    /// `"Lcl Scaling"` is the second `"Lcl …"` typeName the docs §4
    /// trailing-value table calls out explicitly; the cubes fixture's
    /// `Model` node carries `P: "Lcl Scaling", "Lcl Scaling", "", "A",
    /// 10, 10, 10`.
    pub fn as_lcl_scaling(&self, name: &str) -> Option<[f64; 3]> {
        self.as_typed_vec3(name, "Lcl Scaling")
    }

    /// Internal helper for the `as_lcl_*` family — match a
    /// triple-valued record with the requested typeName.
    fn as_typed_vec3(&self, name: &str, expected_type_name: &str) -> Option<[f64; 3]> {
        let rec = self.inner.get(name)?;
        if rec.type_name != expected_type_name {
            return None;
        }
        match &rec.value {
            PValue::Vec3(v) => Some(*v),
            _ => None,
        }
    }

    /// Pull a `DateTime` value by name.
    ///
    /// Per the docs §4 *"`KString`/`DateTime` → `S`"* row, a
    /// `"DateTime"` typeName is wire-encoded as an `S` string; the
    /// cubes-ascii-v7500.fbx fixture's `FBXHeaderExtension` block
    /// shows the documented sample form `P:
    /// "Original|DateTime_GMT", "DateTime", "", "", "07/01/2019
    /// 16:17:31.730"` (the doc's own §3 / §5 enumerates
    /// `"DateTime"` as one of the typeName values that wires as a
    /// quoted string). The accessor returns the raw string body
    /// (the docs do not specify a parsed `chrono`-style breakdown,
    /// so the bytes are surfaced verbatim for caller-side parsing);
    /// it validates the typeName so a `KString` payload doesn't
    /// surface here unintentionally.
    pub fn as_datetime(&self, name: &str) -> Option<&str> {
        let rec = self.inner.get(name)?;
        if rec.type_name != "DateTime" {
            return None;
        }
        match &rec.value {
            PValue::Str(s) => Some(s.as_str()),
            _ => None,
        }
    }

    /// Pull an `"object"` reference value by name.
    ///
    /// The ASCII grammar §8 typeName enumeration lists `"object"`
    /// as a distinct typeName variant (separate from `"KString"`).
    /// In the cubes-ascii-v7500.fbx fixture the `"object"` records
    /// (`P: "SourceObject", "object", "", ""`, `P:
    /// "LookAtProperty", "object", "", ""`, `P:
    /// "UpVectorProperty", "object", "", ""`) all carry an empty
    /// string body — the object reference itself is recorded
    /// elsewhere (in `Connections` `OP` records that wire the
    /// owning element to the referenced object). The accessor
    /// returns the (typically empty) string body for callers that
    /// want to detect the presence of an `"object"` slot
    /// independently of its `Connections` resolution. typeName
    /// validation prevents a `KString` body sneaking in.
    pub fn as_object_ref(&self, name: &str) -> Option<&str> {
        let rec = self.inner.get(name)?;
        if rec.type_name != "object" {
            return None;
        }
        match &rec.value {
            PValue::Str(s) => Some(s.as_str()),
            // The fixture's `"object"` records have zero trailing
            // values (the doc-§4 zero-value `Compound` shape applies
            // when an exporter omits the body entirely). Surface that
            // case as an empty string so the caller can still detect
            // the slot's presence without re-walking.
            PValue::Compound => Some(""),
            _ => None,
        }
    }

    // --- typeName-discriminating scalar accessors ---
    //
    // The base [`Self::as_f64`] / [`Self::as_i32`] / [`Self::as_i64`]
    // / [`Self::as_bool`] / [`Self::as_str`] flatten every scalar
    // typeName the docs §4 trailing-value table enumerates (older
    // exporters mix `I` and `D` and `C` payloads freely for the same
    // semantic slot, so the generic accessors widen across the
    // numeric variants). The following accessors honour
    // [`PRecord::type_name`] — the typeName string parsed from prop1
    // — so a caller asking for, say, a `"KTime"` value does not
    // accidentally pick up a plain `"int"` payload sitting under the
    // same name. Same shape as the round-243 triple accessors above,
    // applied to the §8 ASCII-grammar scalar typeName enumeration
    // (`int`, `double`, `enum`, `bool`, `KString`, `KTime`, `Number`,
    // `ULongLong`).

    /// Pull an `"int"` scalar by name.
    ///
    /// The docs §8 enumeration lists `"int"` as the typeName for
    /// integer-valued properties (the cubes-ascii-v7500.fbx fixture's
    /// `UpAxis` / `UpAxisSign` / `FrontAxis` / `FrontAxisSign` /
    /// `CoordAxis` / `CoordAxisSign` records, and the docs §4 sample
    /// `UpAxis S"int" S"Integer" S"" I=1`). Returns `None` for
    /// records whose typeName is `"enum"` (use [`Self::as_enum`]),
    /// `"bool"` (use [`Self::as_bool_typed`]), or any non-integer
    /// typeName — even though the wire payload may technically be
    /// `I`, the typeName is the semantic discriminator.
    pub fn as_int_typed(&self, name: &str) -> Option<i32> {
        let rec = self.inner.get(name)?;
        if rec.type_name != "int" {
            return None;
        }
        match &rec.value {
            PValue::Int(v) => Some(*v),
            PValue::Long(v) => Some(*v as i32),
            _ => None,
        }
    }

    /// Pull an `"enum"` scalar by name.
    ///
    /// Per docs §4 *"`int`/`enum` → `I`"* the wire encoding is the
    /// same as a plain `"int"`, but the semantic role differs:
    /// `"enum"` typeName records carry an exporter-defined
    /// enumeration index (e.g. the cubes fixture's `TimeMode`,
    /// `TimeProtocol`, `SnapOnFrameMode` records observed under
    /// `GlobalSettings`; the docs §4 sample shows
    /// `TimeMode S"enum" S"" S"" I=0`). The typeName discriminator
    /// lets a caller distinguish a true `enum` index from a plain
    /// integer attribute without re-walking the document.
    pub fn as_enum(&self, name: &str) -> Option<i32> {
        let rec = self.inner.get(name)?;
        if rec.type_name != "enum" {
            return None;
        }
        match &rec.value {
            PValue::Int(v) => Some(*v),
            PValue::Long(v) => Some(*v as i32),
            _ => None,
        }
    }

    /// Pull a `"bool"` scalar by name.
    ///
    /// Per docs §4 *"the typeName/label/flags strings carry the
    /// semantic type; the leading one-byte code carries the wire
    /// type"* — older exporters wire `"bool"` payloads as `I` (int32)
    /// freely (the docs §4 note about `Mute` / `BlendModeBypass`).
    /// The accessor coerces `Int` / `Long` payloads via `!= 0` once
    /// the typeName check confirms the slot is semantically a bool
    /// (the cubes fixture's `Primary Visibility S"bool" S"" S"" I=1`
    /// is a worked example). Returns `None` for non-bool typeNames
    /// even when the wire payload is `I` or `C` — the typeName guard
    /// keeps a plain `"int"` record off the bool surface.
    pub fn as_bool_typed(&self, name: &str) -> Option<bool> {
        let rec = self.inner.get(name)?;
        if rec.type_name != "bool" {
            return None;
        }
        match &rec.value {
            PValue::Bool(v) => Some(*v),
            PValue::Int(v) => Some(*v != 0),
            PValue::Long(v) => Some(*v != 0),
            _ => None,
        }
    }

    /// Pull a `"double"` scalar by name.
    ///
    /// Per docs §4 *"`double`/`Number` → `D`"* both typeNames decode
    /// the same wire payload (a single `D` double). The cubes fixture
    /// uses `"double"` for `UnitScaleFactor` / `OriginalUnitScaleFactor`
    /// / `Opacity` and `"Number"` for `DiffuseFactor`; the
    /// typeName-discriminating accessor narrows on top of the generic
    /// [`Self::as_f64`] so a caller pulling a `"double"` slot does
    /// not pick up a `"Number"` payload under the same name.
    pub fn as_double(&self, name: &str) -> Option<f64> {
        let rec = self.inner.get(name)?;
        if rec.type_name != "double" {
            return None;
        }
        match &rec.value {
            PValue::Double(v) => Some(*v),
            _ => None,
        }
    }

    /// Pull a `"Number"` scalar by name.
    ///
    /// `"Number"` is the second typeName the docs §4 *"`double`/`Number`
    /// → `D`"* row enumerates; the cubes fixture's Material records
    /// `DiffuseFactor` / `EmissiveFactor` / `Shininess` /
    /// `ReflectionFactor` all wire as `P: "...", "Number", "", "A",
    /// <D>`. The typeName guard distinguishes a `"Number"` factor from
    /// a `"double"` setting that happens to share a name.
    pub fn as_number(&self, name: &str) -> Option<f64> {
        let rec = self.inner.get(name)?;
        if rec.type_name != "Number" {
            return None;
        }
        match &rec.value {
            PValue::Double(v) => Some(*v),
            _ => None,
        }
    }

    /// Pull a `"KString"` scalar by name.
    ///
    /// Per docs §4 *"`KString`/`DateTime` → `S`"* both wire as a single
    /// `S` string. The round-243 [`Self::as_datetime`] surfaces the
    /// `"DateTime"` half; this accessor surfaces the `"KString"` half
    /// (the cubes fixture's `DocumentUrl` / `SrcDocumentUrl` /
    /// `currentUVSet` / `DefaultCamera` records all carry
    /// `"KString"`). The typeName guard prevents a `"DateTime"` body
    /// or an `"object"` reference from sneaking in.
    pub fn as_kstring(&self, name: &str) -> Option<&str> {
        let rec = self.inner.get(name)?;
        if rec.type_name != "KString" {
            return None;
        }
        match &rec.value {
            PValue::Str(s) => Some(s.as_str()),
            _ => None,
        }
    }

    /// Pull a `"KTime"` scalar by name without loss of precision.
    ///
    /// Per docs §4 *"`KTime`/`ULongLong` → `L`"* the wire encoding is
    /// `L` (int64); the generic [`Self::as_i64`] widens across `I` /
    /// `Bool` payloads but does not check the typeName, so a plain
    /// `"int"` record passes through unchallenged. This accessor
    /// narrows to `"KTime"` typeName only (the docs §4 sample
    /// `TimeSpanStop S"KTime" S"Time" S"" L=46_186_158_000` is a
    /// worked example, as are the cubes fixture's `TimeSpanStart` /
    /// `TimeSpanStop` records under `GlobalSettings`). `Int` / `Bool`
    /// payloads are still widened losslessly per the docs §4 note
    /// about older exporters using `I` for KTime values that fit; an
    /// `L` payload is returned verbatim so values past 2^53 round-trip
    /// without precision loss (see [`Self::as_i64`] for the precision
    /// rationale).
    pub fn as_ktime(&self, name: &str) -> Option<i64> {
        let rec = self.inner.get(name)?;
        if rec.type_name != "KTime" {
            return None;
        }
        match &rec.value {
            PValue::Long(v) => Some(*v),
            PValue::Int(v) => Some(*v as i64),
            PValue::Bool(v) => Some(if *v { 1 } else { 0 }),
            _ => None,
        }
    }

    /// Pull a `"ULongLong"` scalar by name without loss of precision.
    ///
    /// `"ULongLong"` is the second `L`-wire typeName the docs §4
    /// `KTime`/`ULongLong` → `L` row enumerates; the docs §8 worked
    /// sample lists `P: "BlendModeBypass", "ULongLong", "", "",0` as
    /// a representative case. The wire is the same `L` (int64) the
    /// `KTime` slot uses, so this accessor mirrors [`Self::as_ktime`]
    /// with the matching typeName guard.
    pub fn as_ulonglong(&self, name: &str) -> Option<i64> {
        let rec = self.inner.get(name)?;
        if rec.type_name != "ULongLong" {
            return None;
        }
        match &rec.value {
            PValue::Long(v) => Some(*v),
            PValue::Int(v) => Some(*v as i64),
            PValue::Bool(v) => Some(if *v { 1 } else { 0 }),
            _ => None,
        }
    }

    // --- `Compound` typeName-discriminating accessor ---
    //
    // Closes the last typeName the §8 ASCII-grammar enumeration calls
    // out that previously had no typeName-aware accessor. `"Compound"`
    // is the value-less typeName (docs §4 trailing-value rule *"0 (for
    // Compound, and any value-less property)"*; docs §8 *"`Compound`
    // properties end right after the flags field"*). The §4 worked
    // sample in `fbx-binary-properties70.md` shows it directly:
    // `P props=4 S"TimeMarker" S"Compound" S"" S""` (NO trailing
    // value). Round 243's [`Self::as_object_ref`] already widens an
    // `"object"` typeName whose body got dropped into `PValue::Compound`
    // (an `"object"` slot the exporter wrote with zero trailing
    // values); the present accessor instead narrows to the bare
    // `"Compound"` typeName itself so the two surfaces stay disjoint.

    /// Detect a `"Compound"` typeName slot by name.
    ///
    /// Per the docs §4 worked sample (`P props=4 S"TimeMarker"
    /// S"Compound" S"" S""` — *"Compound: NO value"*) and the docs §8
    /// ASCII enumeration (`P: "Original", "Compound", "", ""` —
    /// *"Compound: no value"*), a `"Compound"` typeName record is a
    /// structural placeholder: its presence is meaningful (the slot
    /// exists in the property template / file) but it carries no
    /// payload. The cubes-ascii-v7500.fbx fixture's
    /// `FBXHeaderExtension { SceneInfo { Properties70 } }` block uses
    /// it for compound-path prefixes such as `Original`, `LastSaved`
    /// (whose nested keys then appear as separate `P` records with
    /// names like `Original|ApplicationName`,
    /// `LastSaved|DateTime_GMT`, etc.).
    ///
    /// Returns `true` when a record with the given name exists with
    /// `type_name == "Compound"` AND the payload is the zero-trailing
    /// [`PValue::Compound`] shape the docs require. Returns `false` in
    /// every other case (record absent, non-`Compound` typeName, or
    /// any non-empty trailing-value payload — the latter is a
    /// malformed Compound and is rejected the same way the round-246
    /// typed-scalar accessors reject shape mismatches).
    ///
    /// This narrowing keeps the round-243 [`Self::as_object_ref`]
    /// surface disjoint from `is_compound`: an `"object"` slot the
    /// exporter wrote with no body lands in [`PValue::Compound`] but
    /// keeps its `"object"` typeName, so it surfaces via
    /// [`Self::as_object_ref`] (returning `""`) and never via
    /// `is_compound` (which only fires when the typeName itself is
    /// the literal string `"Compound"`).
    pub fn is_compound(&self, name: &str) -> bool {
        let Some(rec) = self.inner.get(name) else {
            return false;
        };
        rec.type_name == "Compound" && matches!(rec.value, PValue::Compound)
    }

    /// Iterate every `"Compound"` typeName record name.
    ///
    /// Order is HashMap-defined (no particular file order). Useful
    /// when a caller wants to enumerate the structural / template
    /// placeholder slots in a `Properties70` block (e.g. to drive a
    /// UI that lists compound parent keys before walking the
    /// `Parent|Child` nested keys that share the same prefix). The
    /// same typeName + payload-shape guard [`Self::is_compound`]
    /// applies — records whose `type_name` is `"Compound"` but whose
    /// payload is non-empty (a malformed Compound) are omitted, and
    /// `"object"` slots that happen to carry a `PValue::Compound`
    /// payload (the round-243 [`Self::as_object_ref`] case) keep
    /// their `"object"` typeName and are also omitted here.
    pub fn compound_names(&self) -> impl Iterator<Item = &str> {
        self.inner
            .iter()
            .filter(|(_, rec)| rec.type_name == "Compound" && matches!(rec.value, PValue::Compound))
            .map(|(name, _)| name.as_str())
    }

    /// Iterate every record name. Order is HashMap-defined (no
    /// particular file order).
    pub fn names(&self) -> impl Iterator<Item = &str> {
        self.inner.keys().map(String::as_str)
    }
}

/// Decode one `P` node into a `(name, PRecord)` pair. Returns `None`
/// when the node does not match the docs §4 shape (fewer than 4
/// string-typed leading props, etc.).
fn decode_p_record(node: &FbxNode) -> Option<(String, PRecord)> {
    if node.properties.len() < 4 {
        return None;
    }
    let name = string_prop(&node.properties[0])?;
    let type_name = string_prop(&node.properties[1])?;
    let label = string_prop(&node.properties[2])?;
    let flags = string_prop(&node.properties[3])?;
    let trailing = &node.properties[4..];
    let value = decode_value(&type_name, trailing);
    Some((
        name,
        PRecord {
            type_name,
            label,
            flags,
            value,
        },
    ))
}

/// Build a typed [`PValue`] from the trailing value-prop slice. The
/// shape branches mirror the value-count rules in docs §4:
///
/// - Empty trailing slice → `Compound` (`type == "Compound"` or any
///   value-less P).
/// - One element → typed scalar based on `type_name`.
/// - Three elements → `Vec3` (used for `ColorRGB` / `Vector3D` /
///   `Lcl Translation` / etc.).
/// - Anything else → `Other` (round-trips through the FbxDocument
///   without losing data).
fn decode_value(type_name: &str, trailing: &[FbxProperty]) -> PValue {
    match trailing.len() {
        0 => PValue::Compound,
        1 => scalar_value(type_name, &trailing[0]),
        3 => {
            // The docs §4 explicitly lists three-double triples for
            // ColorRGB / Color / Vector3D / Vector / Lcl Translation /
            // Lcl Scaling. The wire encoding is three D records.
            let a = trailing[0].as_f64_loose();
            let b = trailing[1].as_f64_loose();
            let c = trailing[2].as_f64_loose();
            if let (Some(a), Some(b), Some(c)) = (a, b, c) {
                PValue::Vec3([a, b, c])
            } else {
                PValue::Other(trailing.to_vec())
            }
        }
        _ => PValue::Other(trailing.to_vec()),
    }
}

/// Decode one scalar value. The wire encoding code in
/// `FbxProperty` is authoritative for what the bytes really mean;
/// the `type_name` string is used to disambiguate ambiguous cases
/// (e.g. an `Int` payload whose typeName is `"bool"`).
fn scalar_value(type_name: &str, prop: &FbxProperty) -> PValue {
    match prop {
        FbxProperty::Bool(b) => PValue::Bool(*b),
        FbxProperty::F32(v) => PValue::Double(*v as f64),
        FbxProperty::F64(v) => PValue::Double(*v),
        FbxProperty::I16(v) => PValue::Int(*v as i32),
        FbxProperty::I32(v) => match type_name {
            // Some FBX `P` records carry "bool" payloads as `I` wire
            // codes (the docs §4 sample for `BlendModeBypass` /
            // `Mute` shows ULongLong / bool with integer wire). We
            // honour the typeName for unambiguous decoding.
            "bool" => PValue::Bool(*v != 0),
            _ => PValue::Int(*v),
        },
        FbxProperty::I64(v) => match type_name {
            "bool" => PValue::Bool(*v != 0),
            _ => PValue::Long(*v),
        },
        FbxProperty::String(bytes) => match std::str::from_utf8(bytes) {
            Ok(s) => PValue::Str(s.to_owned()),
            Err(_) => PValue::Other(vec![prop.clone()]),
        },
        _ => PValue::Other(vec![prop.clone()]),
    }
}

fn string_prop(p: &FbxProperty) -> Option<String> {
    match p {
        FbxProperty::String(bytes) => std::str::from_utf8(bytes).ok().map(str::to_owned),
        _ => None,
    }
}

impl FbxProperty {
    /// Coerce numeric scalars to `f64`. Returns `None` for non-numeric
    /// variants (strings, raw blobs, arrays).
    ///
    /// Defined here rather than on the main `binary` module to keep
    /// the `Properties70` triple-value decode self-contained (a few
    /// callers in `material` / `light` / `camera` will reuse it).
    pub(crate) fn as_f64_loose(&self) -> Option<f64> {
        match *self {
            FbxProperty::F32(v) => Some(v as f64),
            FbxProperty::F64(v) => Some(v),
            FbxProperty::I16(v) => Some(v as f64),
            FbxProperty::I32(v) => Some(v as f64),
            FbxProperty::I64(v) => Some(v as f64),
            FbxProperty::Bool(v) => Some(if v { 1.0 } else { 0.0 }),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s(b: &[u8]) -> FbxProperty {
        FbxProperty::String(b.to_vec())
    }

    fn p(name: &str, type_name: &str, label: &str, flags: &str, vals: Vec<FbxProperty>) -> FbxNode {
        let mut props = vec![
            s(name.as_bytes()),
            s(type_name.as_bytes()),
            s(label.as_bytes()),
            s(flags.as_bytes()),
        ];
        props.extend(vals);
        FbxNode {
            name: "P".into(),
            properties: props,
            children: Vec::new(),
        }
    }

    fn props70(records: Vec<FbxNode>) -> FbxNode {
        FbxNode {
            name: "Properties70".into(),
            properties: Vec::new(),
            children: records,
        }
    }

    #[test]
    fn decode_scalar_double() {
        // docs §4 sample: `UnitScaleFactor S"double" S"Number" S"" D=100.0`.
        let block = props70(vec![p(
            "UnitScaleFactor",
            "double",
            "Number",
            "",
            vec![FbxProperty::F64(100.0)],
        )]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.len(), 1);
        assert_eq!(pm.as_f64("UnitScaleFactor"), Some(100.0));
        let rec = pm.get("UnitScaleFactor").expect("decoded UnitScaleFactor");
        assert_eq!(rec.type_name, "double");
        assert_eq!(rec.label, "Number");
        assert_eq!(rec.flags, "");
        assert_eq!(rec.value, PValue::Double(100.0));
    }

    #[test]
    fn decode_scalar_int_and_enum() {
        // docs §4: `UpAxis S"int" S"Integer" S"" I=1`.
        let block = props70(vec![
            p("UpAxis", "int", "Integer", "", vec![FbxProperty::I32(1)]),
            p("TimeMode", "enum", "", "", vec![FbxProperty::I32(0)]),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_i32("UpAxis"), Some(1));
        assert_eq!(pm.as_i32("TimeMode"), Some(0));
    }

    #[test]
    fn decode_vec3_color() {
        // docs §4 sample: `AmbientColor S"ColorRGB" S"Color" S"" D=0 D=0 D=0`.
        let block = props70(vec![p(
            "AmbientColor",
            "ColorRGB",
            "Color",
            "",
            vec![
                FbxProperty::F64(0.4),
                FbxProperty::F64(0.5),
                FbxProperty::F64(0.6),
            ],
        )]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_vec3("AmbientColor"), Some([0.4, 0.5, 0.6]));
    }

    #[test]
    fn decode_kstring() {
        // docs §4 sample: `DefaultCamera S"KString" S"" S"" S"Producer Perspective"`.
        let block = props70(vec![p(
            "DefaultCamera",
            "KString",
            "",
            "",
            vec![s(b"Producer Perspective")],
        )]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_str("DefaultCamera"), Some("Producer Perspective"));
    }

    #[test]
    fn decode_ktime_long() {
        // docs §4 sample: `TimeSpanStop S"KTime" S"Time" S"" L=46186158000`.
        let block = props70(vec![p(
            "TimeSpanStop",
            "KTime",
            "Time",
            "",
            vec![FbxProperty::I64(46_186_158_000)],
        )]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(
            pm.get("TimeSpanStop").map(|r| r.value.clone()),
            Some(PValue::Long(46_186_158_000))
        );
        assert_eq!(pm.as_f64("TimeSpanStop"), Some(46_186_158_000.0));
        // Lossless int64 accessor (the docs §4 sample value exceeds
        // the i32 range): `46_186_158_000` cleanly survives the
        // round trip, where the f64 path quietly drops precision for
        // values approaching 2^53.
        assert_eq!(pm.as_i64("TimeSpanStop"), Some(46_186_158_000));
    }

    #[test]
    fn as_i64_preserves_int64_past_f64_safe_range() {
        // 2^53 + 1 is the smallest positive integer not exactly
        // representable by f64, so `as_f64 -> i64 round trip` would
        // lose precision; `as_i64` returns the wire value verbatim.
        let big: i64 = (1_i64 << 53) + 1;
        let block = props70(vec![p(
            "TimeSpanStop",
            "KTime",
            "Time",
            "",
            vec![FbxProperty::I64(big)],
        )]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_i64("TimeSpanStop"), Some(big));
        // The f64 accessor still works but quietly truncates the
        // low-order bit — this assertion documents the precision
        // ceiling that motivates the typed `as_i64` path.
        let lossy = pm.as_f64("TimeSpanStop").unwrap() as i64;
        assert_eq!(lossy, big - 1);
    }

    #[test]
    fn as_i64_widens_int_and_bool_wire_codes() {
        // Per docs §4, older exporters wire some `KTime` / `ULongLong`
        // payloads as `I` (int32); the accessor widens losslessly.
        let block = props70(vec![
            p(
                "BlendModeBypass",
                "ULongLong",
                "",
                "",
                vec![FbxProperty::I32(7)],
            ),
            p("Mute", "bool", "", "", vec![FbxProperty::Bool(true)]),
            p("ZeroBool", "bool", "", "", vec![FbxProperty::I32(0)]),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_i64("BlendModeBypass"), Some(7));
        assert_eq!(pm.as_i64("Mute"), Some(1));
        assert_eq!(pm.as_i64("ZeroBool"), Some(0));
    }

    #[test]
    fn as_i64_rejects_non_numeric_records() {
        // String / triple / Compound records all return `None` so the
        // caller can fall back without ambiguity.
        let block = props70(vec![
            p(
                "DefaultCamera",
                "KString",
                "",
                "",
                vec![s(b"Producer Perspective")],
            ),
            p(
                "AmbientColor",
                "ColorRGB",
                "Color",
                "",
                vec![
                    FbxProperty::F64(0.0),
                    FbxProperty::F64(0.0),
                    FbxProperty::F64(0.0),
                ],
            ),
            p("TimeMarker", "Compound", "", "", vec![]),
            p(
                "UnitScaleFactor",
                "double",
                "Number",
                "",
                vec![FbxProperty::F64(100.0)],
            ),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_i64("DefaultCamera"), None);
        assert_eq!(pm.as_i64("AmbientColor"), None);
        assert_eq!(pm.as_i64("TimeMarker"), None);
        // Double payload is also rejected — callers wanting numeric
        // coercion across f64/int should call `as_f64`.
        assert_eq!(pm.as_i64("UnitScaleFactor"), None);
        // Missing record: also `None`.
        assert_eq!(pm.as_i64("DoesNotExist"), None);
    }

    #[test]
    fn as_i64_handles_negative_ktime() {
        // KTime values can be negative (e.g. `TimeSpanStart` before
        // time 0 in animations that loop). The wire is signed int64.
        let block = props70(vec![p(
            "TimeSpanStart",
            "KTime",
            "Time",
            "",
            vec![FbxProperty::I64(-1_924_423_250)],
        )]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_i64("TimeSpanStart"), Some(-1_924_423_250));
    }

    #[test]
    fn decode_compound_no_value() {
        // docs §4 sample: `TimeMarker S"Compound" S"" S""` — NO value.
        let block = props70(vec![p("TimeMarker", "Compound", "", "", vec![])]);
        let pm = PropertyMap::from_properties70(&block);
        let rec = pm.get("TimeMarker").expect("Compound P decoded");
        assert_eq!(rec.value, PValue::Compound);
    }

    #[test]
    fn from_element_finds_properties70_child() {
        // `Material` parent with a Properties70 child.
        let mat = FbxNode {
            name: "Material".into(),
            properties: vec![],
            children: vec![props70(vec![p(
                "DiffuseColor",
                "Color",
                "",
                "A",
                vec![
                    FbxProperty::F64(0.8),
                    FbxProperty::F64(0.4),
                    FbxProperty::F64(0.2),
                ],
            )])],
        };
        let pm = PropertyMap::from_element(&mat);
        assert_eq!(pm.as_vec3("DiffuseColor"), Some([0.8, 0.4, 0.2]));
    }

    #[test]
    fn missing_properties70_returns_empty_map() {
        let bare = FbxNode {
            name: "Material".into(),
            properties: vec![],
            children: vec![],
        };
        let pm = PropertyMap::from_element(&bare);
        assert!(pm.is_empty());
    }

    #[test]
    fn bool_typed_payload_with_int_wire() {
        // Older exporters wire `bool` as `I` — typeName disambiguates.
        let block = props70(vec![p("Mute", "bool", "", "", vec![FbxProperty::I32(1)])]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_bool("Mute"), Some(true));
    }

    #[test]
    fn lcl_translation_triple() {
        // docs §4 / ASCII §8 sample:
        // `P "Lcl Translation","Lcl Translation","","A",-1.04,0.99,-1.04`.
        let block = props70(vec![p(
            "Lcl Translation",
            "Lcl Translation",
            "",
            "A",
            vec![
                FbxProperty::F64(-1.04),
                FbxProperty::F64(0.998),
                FbxProperty::F64(-1.043),
            ],
        )]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_vec3("Lcl Translation"), Some([-1.04, 0.998, -1.043]));
        let rec = pm.get("Lcl Translation").expect("decoded translation");
        assert_eq!(rec.flags, "A");
    }

    // --- typeName-discriminating accessor tests ---

    #[test]
    fn as_color_rgb_accepts_colorrgb_and_color_typenames() {
        // Docs §4 sample uses "ColorRGB"; cubes-ascii-v7500.fbx Material
        // records use "Color" — both belong to the same triple-typeName
        // family and the accessor accepts either.
        let block = props70(vec![
            p(
                "AmbientColor",
                "ColorRGB",
                "Color",
                "",
                vec![
                    FbxProperty::F64(0.1),
                    FbxProperty::F64(0.2),
                    FbxProperty::F64(0.3),
                ],
            ),
            p(
                "DiffuseColor",
                "Color",
                "",
                "A",
                vec![
                    FbxProperty::F64(0.8),
                    FbxProperty::F64(0.4),
                    FbxProperty::F64(0.2),
                ],
            ),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_color_rgb("AmbientColor"), Some([0.1, 0.2, 0.3]));
        assert_eq!(pm.as_color_rgb("DiffuseColor"), Some([0.8, 0.4, 0.2]));
    }

    #[test]
    fn as_color_rgb_rejects_non_color_typenames() {
        // A `Vector3D` triple is structurally identical (three doubles)
        // but semantically a direction/Euler/etc. — the typeName guard
        // keeps the surfaces disjoint.
        let block = props70(vec![
            p(
                "PreRotation",
                "Vector3D",
                "Vector",
                "",
                vec![
                    FbxProperty::F64(0.0),
                    FbxProperty::F64(90.0),
                    FbxProperty::F64(0.0),
                ],
            ),
            p(
                "Lcl Translation",
                "Lcl Translation",
                "",
                "A",
                vec![
                    FbxProperty::F64(1.0),
                    FbxProperty::F64(2.0),
                    FbxProperty::F64(3.0),
                ],
            ),
            p("UpAxis", "int", "Integer", "", vec![FbxProperty::I32(1)]),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_color_rgb("PreRotation"), None);
        assert_eq!(pm.as_color_rgb("Lcl Translation"), None);
        assert_eq!(pm.as_color_rgb("UpAxis"), None);
        // Plain `as_vec3` still surfaces the Vector3D/Lcl triples —
        // the typeName-discriminating accessor narrows on top of the
        // generic one.
        assert_eq!(pm.as_vec3("PreRotation"), Some([0.0, 90.0, 0.0]));
        assert_eq!(pm.as_vec3("Lcl Translation"), Some([1.0, 2.0, 3.0]));
    }

    #[test]
    fn as_vector3d_accepts_vector3d_and_vector_typenames() {
        // Cubes fixture's `PreRotation` / `PostRotation` /
        // `GeometricTranslation` / `GeometricRotation` /
        // `GeometricScaling` all wire as `"Vector3D"`. `"Vector"`
        // is also enumerated in ASCII grammar §8.
        let block = props70(vec![
            p(
                "PreRotation",
                "Vector3D",
                "Vector",
                "",
                vec![
                    FbxProperty::F64(0.0),
                    FbxProperty::F64(0.0),
                    FbxProperty::F64(0.0),
                ],
            ),
            p(
                "GeometricScaling",
                "Vector",
                "",
                "",
                vec![
                    FbxProperty::F64(1.0),
                    FbxProperty::F64(1.0),
                    FbxProperty::F64(1.0),
                ],
            ),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_vector3d("PreRotation"), Some([0.0, 0.0, 0.0]));
        assert_eq!(pm.as_vector3d("GeometricScaling"), Some([1.0, 1.0, 1.0]));
    }

    #[test]
    fn as_vector3d_rejects_color_and_lcl_typenames() {
        let block = props70(vec![
            p(
                "AmbientColor",
                "ColorRGB",
                "Color",
                "",
                vec![
                    FbxProperty::F64(0.0),
                    FbxProperty::F64(0.0),
                    FbxProperty::F64(0.0),
                ],
            ),
            p(
                "Lcl Scaling",
                "Lcl Scaling",
                "",
                "A",
                vec![
                    FbxProperty::F64(2.0),
                    FbxProperty::F64(2.0),
                    FbxProperty::F64(2.0),
                ],
            ),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_vector3d("AmbientColor"), None);
        assert_eq!(pm.as_vector3d("Lcl Scaling"), None);
    }

    #[test]
    fn as_lcl_translation_rotation_scaling_split_by_typename() {
        // The cubes-ascii-v7500.fbx fixture's `Model` block carries
        // all three `Lcl …` records on the same element; each
        // accessor must surface only its own typeName.
        let block = props70(vec![
            p(
                "Lcl Translation",
                "Lcl Translation",
                "",
                "A",
                vec![
                    FbxProperty::F64(-1.04023893373156),
                    FbxProperty::F64(0.998288783259251),
                    FbxProperty::F64(-1.04375962988677),
                ],
            ),
            p(
                "Lcl Rotation",
                "Lcl Rotation",
                "",
                "A",
                vec![
                    FbxProperty::F64(0.0),
                    FbxProperty::F64(45.0),
                    FbxProperty::F64(0.0),
                ],
            ),
            p(
                "Lcl Scaling",
                "Lcl Scaling",
                "",
                "A",
                vec![
                    FbxProperty::F64(10.0),
                    FbxProperty::F64(10.0),
                    FbxProperty::F64(10.0),
                ],
            ),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(
            pm.as_lcl_translation("Lcl Translation"),
            Some([-1.04023893373156, 0.998288783259251, -1.04375962988677]),
        );
        assert_eq!(pm.as_lcl_rotation("Lcl Rotation"), Some([0.0, 45.0, 0.0]));
        assert_eq!(pm.as_lcl_scaling("Lcl Scaling"), Some([10.0, 10.0, 10.0]));
        // Cross-name rejection — `Lcl Translation` payload does not
        // surface under `as_lcl_rotation`, etc.
        assert_eq!(pm.as_lcl_rotation("Lcl Translation"), None);
        assert_eq!(pm.as_lcl_scaling("Lcl Translation"), None);
        assert_eq!(pm.as_lcl_translation("Lcl Rotation"), None);
        // And a plain `Vector3D` record (e.g. PreRotation) is not
        // promoted to `Lcl Rotation` just because its triple shape
        // matches.
        let pre = props70(vec![p(
            "PreRotation",
            "Vector3D",
            "Vector",
            "",
            vec![
                FbxProperty::F64(0.0),
                FbxProperty::F64(0.0),
                FbxProperty::F64(0.0),
            ],
        )]);
        let pm2 = PropertyMap::from_properties70(&pre);
        assert_eq!(pm2.as_lcl_rotation("PreRotation"), None);
    }

    #[test]
    fn as_datetime_accepts_datetime_typename_only() {
        // ASCII grammar §8 worked sample:
        // `P: "Original|DateTime_GMT", "DateTime", "", "", "07/01/2019 16:17:31.730"`
        let block = props70(vec![
            p(
                "Original|DateTime_GMT",
                "DateTime",
                "",
                "",
                vec![FbxProperty::String(b"07/01/2019 16:17:31.730".to_vec())],
            ),
            // A KString record on a similarly-named slot must NOT
            // surface through as_datetime; the typeName is the only
            // signal that disambiguates the two.
            p(
                "DocumentUrl",
                "KString",
                "Url",
                "",
                vec![s(b"U:\\path\\file.fbx")],
            ),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(
            pm.as_datetime("Original|DateTime_GMT"),
            Some("07/01/2019 16:17:31.730"),
        );
        assert_eq!(pm.as_datetime("DocumentUrl"), None);
        // Plain as_str still surfaces both — the typeName accessor
        // narrows on top.
        assert_eq!(
            pm.as_str("Original|DateTime_GMT"),
            Some("07/01/2019 16:17:31.730"),
        );
        assert_eq!(pm.as_str("DocumentUrl"), Some("U:\\path\\file.fbx"));
        // Missing record → None.
        assert_eq!(pm.as_datetime("MissingRecord"), None);
    }

    // --- typeName-discriminating scalar accessor tests ---

    #[test]
    fn as_int_typed_accepts_only_int_typename() {
        // Cubes-fixture GlobalSettings records: `UpAxis`, `UpAxisSign`,
        // `FrontAxis`, etc. all wire as `"int"` with an `I` payload.
        // A coincident `"enum"` or `"bool"` payload sitting under a
        // similarly-named slot must NOT surface here — that's the
        // round-243 typeName-discrimination invariant applied to the
        // scalar `as_i32` surface.
        let block = props70(vec![
            p("UpAxis", "int", "Integer", "", vec![FbxProperty::I32(1)]),
            p(
                "OriginalUpAxis",
                "int",
                "Integer",
                "",
                vec![FbxProperty::I32(-1)],
            ),
            p("TimeMode", "enum", "", "", vec![FbxProperty::I32(11)]),
            p(
                "Primary Visibility",
                "bool",
                "",
                "",
                vec![FbxProperty::I32(1)],
            ),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_int_typed("UpAxis"), Some(1));
        assert_eq!(pm.as_int_typed("OriginalUpAxis"), Some(-1));
        // `"enum"` and `"bool"` typeNames stay disjoint from `"int"`.
        assert_eq!(pm.as_int_typed("TimeMode"), None);
        assert_eq!(pm.as_int_typed("Primary Visibility"), None);
        // The generic `as_i32` still widens across all three (the
        // typeName-aware accessor narrows on top).
        assert_eq!(pm.as_i32("UpAxis"), Some(1));
        assert_eq!(pm.as_i32("TimeMode"), Some(11));
        assert_eq!(pm.as_i32("Primary Visibility"), Some(1));
        // Missing record → `None`.
        assert_eq!(pm.as_int_typed("DoesNotExist"), None);
    }

    #[test]
    fn as_enum_accepts_only_enum_typename() {
        // Docs §4 sample: `TimeMode S"enum" S"" S"" I=0`. The cubes
        // fixture's GlobalSettings block has `TimeMode`, `TimeProtocol`,
        // `SnapOnFrameMode` all as `"enum"`.
        let block = props70(vec![
            p("TimeMode", "enum", "", "", vec![FbxProperty::I32(0)]),
            p("TimeProtocol", "enum", "", "", vec![FbxProperty::I32(2)]),
            p("UpAxis", "int", "Integer", "", vec![FbxProperty::I32(1)]),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_enum("TimeMode"), Some(0));
        assert_eq!(pm.as_enum("TimeProtocol"), Some(2));
        // `"int"` typeName stays disjoint.
        assert_eq!(pm.as_enum("UpAxis"), None);
        assert_eq!(pm.as_enum("Missing"), None);
    }

    #[test]
    fn as_bool_typed_accepts_only_bool_typename() {
        // Docs §8 worked sample: `P: "Mute", "bool", "", "",0`. Older
        // exporters wire `"bool"` as `I` per docs §4; the typeName
        // discriminator confirms the bool intent before the wire
        // coercion fires.
        let block = props70(vec![
            p("Mute", "bool", "", "", vec![FbxProperty::Bool(true)]),
            p(
                "Primary Visibility",
                "bool",
                "",
                "",
                vec![FbxProperty::I32(0)],
            ),
            p("Solo", "bool", "", "", vec![FbxProperty::Bool(false)]),
            // Plain `"int"` record: must NOT surface as a bool even
            // though the wire payload could coerce.
            p("UpAxis", "int", "Integer", "", vec![FbxProperty::I32(1)]),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_bool_typed("Mute"), Some(true));
        assert_eq!(pm.as_bool_typed("Primary Visibility"), Some(false));
        assert_eq!(pm.as_bool_typed("Solo"), Some(false));
        assert_eq!(pm.as_bool_typed("UpAxis"), None);
        // Generic `as_bool` still surfaces the `"int"` payload via the
        // documented `!= 0` widening.
        assert_eq!(pm.as_bool("UpAxis"), Some(true));
        assert_eq!(pm.as_bool_typed("Missing"), None);
    }

    #[test]
    fn as_double_accepts_only_double_typename() {
        // Docs §4 sample: `UnitScaleFactor S"double" S"Number" S"" D=100.0`.
        // Cubes fixture also has `OriginalUnitScaleFactor S"double"` and
        // `Opacity S"double"`. `"Number"` is a distinct typeName even
        // though both share the `D` wire encoding.
        let block = props70(vec![
            p(
                "UnitScaleFactor",
                "double",
                "Number",
                "",
                vec![FbxProperty::F64(100.0)],
            ),
            p(
                "Opacity",
                "double",
                "Number",
                "",
                vec![FbxProperty::F64(1.0)],
            ),
            p(
                "DiffuseFactor",
                "Number",
                "",
                "A",
                vec![FbxProperty::F64(0.8)],
            ),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_double("UnitScaleFactor"), Some(100.0));
        assert_eq!(pm.as_double("Opacity"), Some(1.0));
        assert_eq!(pm.as_double("DiffuseFactor"), None);
        // Generic `as_f64` still surfaces both.
        assert_eq!(pm.as_f64("UnitScaleFactor"), Some(100.0));
        assert_eq!(pm.as_f64("DiffuseFactor"), Some(0.8));
        assert_eq!(pm.as_double("Missing"), None);
    }

    #[test]
    fn as_number_accepts_only_number_typename() {
        // Cubes fixture Material records: `DiffuseFactor`,
        // `EmissiveFactor`, `Shininess`, `ReflectionFactor` are all
        // `P: "...", "Number", "", "A", <D>`.
        let block = props70(vec![
            p(
                "DiffuseFactor",
                "Number",
                "",
                "A",
                vec![FbxProperty::F64(0.8)],
            ),
            p("Shininess", "Number", "", "A", vec![FbxProperty::F64(20.0)]),
            p(
                "UnitScaleFactor",
                "double",
                "Number",
                "",
                vec![FbxProperty::F64(100.0)],
            ),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_number("DiffuseFactor"), Some(0.8));
        assert_eq!(pm.as_number("Shininess"), Some(20.0));
        // `"double"` typeName stays disjoint even though both share `D` wire.
        assert_eq!(pm.as_number("UnitScaleFactor"), None);
        assert_eq!(pm.as_number("Missing"), None);
    }

    #[test]
    fn as_kstring_accepts_only_kstring_typename() {
        // Cubes fixture `DocumentUrl` / `SrcDocumentUrl` /
        // `currentUVSet` / `DefaultCamera` records all carry
        // `"KString"`. `"DateTime"` and `"object"` share the `S` wire
        // but must NOT surface here — the round-243 `as_datetime` /
        // `as_object_ref` accessors own those.
        let block = props70(vec![
            p(
                "DocumentUrl",
                "KString",
                "Url",
                "",
                vec![s(b"U:\\path\\file.fbx")],
            ),
            p(
                "DefaultCamera",
                "KString",
                "",
                "",
                vec![s(b"Producer Perspective")],
            ),
            p("currentUVSet", "KString", "", "U", vec![s(b"map1")]),
            // `"DateTime"` payload — disjoint surface.
            p(
                "Original|DateTime_GMT",
                "DateTime",
                "",
                "",
                vec![s(b"07/01/2019 16:17:31.730")],
            ),
            // `"object"` payload — also disjoint.
            p("SourceObject", "object", "", "", vec![]),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_kstring("DocumentUrl"), Some("U:\\path\\file.fbx"));
        assert_eq!(pm.as_kstring("DefaultCamera"), Some("Producer Perspective"));
        assert_eq!(pm.as_kstring("currentUVSet"), Some("map1"));
        // typeName guard rejects `"DateTime"` / `"object"`.
        assert_eq!(pm.as_kstring("Original|DateTime_GMT"), None);
        assert_eq!(pm.as_kstring("SourceObject"), None);
        // Generic `as_str` still surfaces every string-bodied record.
        assert_eq!(
            pm.as_str("Original|DateTime_GMT"),
            Some("07/01/2019 16:17:31.730")
        );
        assert_eq!(pm.as_kstring("Missing"), None);
    }

    #[test]
    fn as_ktime_accepts_only_ktime_typename() {
        // Docs §4 sample: `TimeSpanStop S"KTime" S"Time" S"" L=46186158000`.
        // Plain `"int"` / `"ULongLong"` payloads (also `L`-wire per
        // docs §4) must NOT surface here — the typeName is the
        // semantic discriminator.
        let block = props70(vec![
            p(
                "TimeSpanStart",
                "KTime",
                "Time",
                "",
                vec![FbxProperty::I64(-1_924_423_250)],
            ),
            p(
                "TimeSpanStop",
                "KTime",
                "Time",
                "",
                vec![FbxProperty::I64(46_186_158_000)],
            ),
            p(
                "BlendModeBypass",
                "ULongLong",
                "",
                "",
                vec![FbxProperty::I64(0)],
            ),
            p("UpAxis", "int", "Integer", "", vec![FbxProperty::I32(1)]),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_ktime("TimeSpanStart"), Some(-1_924_423_250));
        assert_eq!(pm.as_ktime("TimeSpanStop"), Some(46_186_158_000));
        // typeName guard rejects `"ULongLong"` and `"int"`.
        assert_eq!(pm.as_ktime("BlendModeBypass"), None);
        assert_eq!(pm.as_ktime("UpAxis"), None);
        // Generic `as_i64` still surfaces every `L`-wire numeric record.
        assert_eq!(pm.as_i64("BlendModeBypass"), Some(0));
        assert_eq!(pm.as_i64("UpAxis"), Some(1));
        assert_eq!(pm.as_ktime("Missing"), None);
    }

    #[test]
    fn as_ktime_widens_int_and_bool_wire_codes() {
        // Per docs §4, older exporters wire `"KTime"` payloads as `I`
        // (int32) when the value fits; the typeName-aware accessor
        // widens losslessly once the typeName check confirms the slot.
        let block = props70(vec![
            p(
                "ShortKTime",
                "KTime",
                "Time",
                "",
                vec![FbxProperty::I32(12345)],
            ),
            p(
                "BoolKTime",
                "KTime",
                "Time",
                "",
                vec![FbxProperty::Bool(true)],
            ),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_ktime("ShortKTime"), Some(12345));
        assert_eq!(pm.as_ktime("BoolKTime"), Some(1));
    }

    #[test]
    fn as_ulonglong_accepts_only_ulonglong_typename() {
        // Docs §8 worked sample:
        // `P: "BlendModeBypass", "ULongLong", "", "",0`.
        let block = props70(vec![
            p(
                "BlendModeBypass",
                "ULongLong",
                "",
                "",
                vec![FbxProperty::I64(7)],
            ),
            // Plain `"KTime"` must NOT surface here.
            p(
                "TimeSpanStop",
                "KTime",
                "Time",
                "",
                vec![FbxProperty::I64(46_186_158_000)],
            ),
            // Plain `"int"` must NOT surface here.
            p("UpAxis", "int", "Integer", "", vec![FbxProperty::I32(1)]),
            // `I` payload under `"ULongLong"` typeName widens losslessly
            // per the docs §4 mixed-wire note.
            p(
                "ShortBypass",
                "ULongLong",
                "",
                "",
                vec![FbxProperty::I32(42)],
            ),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_ulonglong("BlendModeBypass"), Some(7));
        assert_eq!(pm.as_ulonglong("ShortBypass"), Some(42));
        assert_eq!(pm.as_ulonglong("TimeSpanStop"), None);
        assert_eq!(pm.as_ulonglong("UpAxis"), None);
        assert_eq!(pm.as_ulonglong("Missing"), None);
    }

    #[test]
    fn typed_scalar_accessors_reject_non_matching_payload_shape() {
        // A `Compound` (zero-value) record cannot surface through any
        // typed-scalar accessor — the payload-shape guard catches the
        // mismatch even when the typeName check would otherwise pass.
        // (Compound is its own typeName; the guard fires on payload
        // structure, mirroring the round-243 triple-accessor pattern.)
        let block = props70(vec![
            // `"Compound"` typeName: zero-value record.
            p("Original", "Compound", "", "", vec![]),
            // `"KString"` typeName with a triple payload (synthetic
            // malformation): the shape guard rejects it.
            p(
                "Malformed",
                "KString",
                "",
                "",
                vec![
                    FbxProperty::F64(0.0),
                    FbxProperty::F64(0.0),
                    FbxProperty::F64(0.0),
                ],
            ),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        // Compound payload: no scalar accessor accepts it.
        assert_eq!(pm.as_int_typed("Original"), None);
        assert_eq!(pm.as_enum("Original"), None);
        assert_eq!(pm.as_bool_typed("Original"), None);
        assert_eq!(pm.as_double("Original"), None);
        assert_eq!(pm.as_number("Original"), None);
        assert_eq!(pm.as_kstring("Original"), None);
        assert_eq!(pm.as_ktime("Original"), None);
        assert_eq!(pm.as_ulonglong("Original"), None);
        // Malformed triple-under-KString: payload-shape guard fires.
        assert_eq!(pm.as_kstring("Malformed"), None);
    }

    #[test]
    fn is_compound_accepts_only_compound_typename_with_empty_payload() {
        // docs §4 worked sample: `P props=4 S"TimeMarker" S"Compound"
        // S"" S""` — Compound: NO value. docs §8 ASCII counterpart:
        // `P: "Original", "Compound", "", ""` — same shape.
        let block = props70(vec![
            // Canonical Compound from the docs.
            p("TimeMarker", "Compound", "", "", vec![]),
            p("Original", "Compound", "", "", vec![]),
            p("LastSaved", "Compound", "", "", vec![]),
            // The compound-prefix's nested keys appear as separate
            // `P` records with their own scalar typeNames — these
            // must NOT surface via is_compound.
            p(
                "Original|ApplicationName",
                "KString",
                "",
                "",
                vec![s(b"AnApp")],
            ),
            p(
                "LastSaved|DateTime_GMT",
                "DateTime",
                "",
                "",
                vec![s(b"07/01/2019 16:17:31.730")],
            ),
            // The round-243 `object` empty-body case (cubes fixture
            // `SourceObject` / `LookAtProperty` / `UpVectorProperty`):
            // payload lands as PValue::Compound but typeName stays
            // "object" — must NOT surface via is_compound.
            p("SourceObject", "object", "", "", vec![]),
            // A `"double"` slot — typeName guard rejects it.
            p(
                "UnitScaleFactor",
                "double",
                "Number",
                "",
                vec![FbxProperty::F64(1.0)],
            ),
        ]);
        let pm = PropertyMap::from_properties70(&block);

        // True only for the bare `"Compound"` typeName records.
        assert!(pm.is_compound("TimeMarker"));
        assert!(pm.is_compound("Original"));
        assert!(pm.is_compound("LastSaved"));

        // Compound-prefix nested keys keep their own typeNames.
        assert!(!pm.is_compound("Original|ApplicationName"));
        assert!(!pm.is_compound("LastSaved|DateTime_GMT"));

        // `object` empty-body must not collide with `Compound`.
        assert!(!pm.is_compound("SourceObject"));
        // The round-243 surface still recognises it.
        assert_eq!(pm.as_object_ref("SourceObject"), Some(""));

        // Non-Compound typeName: rejected.
        assert!(!pm.is_compound("UnitScaleFactor"));

        // Absent slot: rejected.
        assert!(!pm.is_compound("Missing"));
    }

    #[test]
    fn is_compound_rejects_malformed_compound_with_trailing_payload() {
        // A `"Compound"` typeName with a non-empty trailing value is
        // structurally malformed per docs §4 (*"0 (for Compound)"*).
        // The shape guard rejects it the same way the round-246
        // typed-scalar accessors reject mismatched payloads.
        let block = props70(vec![
            // Malformed: Compound typeName carrying a single scalar.
            p(
                "BadCompound",
                "Compound",
                "",
                "",
                vec![FbxProperty::I32(42)],
            ),
            // Malformed: Compound typeName carrying a triple. The
            // §4 trailing-value table only enumerates `Compound` /
            // scalar / triple shapes — the typeName-guarded accessor
            // honours the documented zero-trailing rule even when
            // the wire encoding happens to be a triple.
            p(
                "TripleCompound",
                "Compound",
                "",
                "",
                vec![
                    FbxProperty::F64(0.0),
                    FbxProperty::F64(0.0),
                    FbxProperty::F64(0.0),
                ],
            ),
        ]);
        let pm = PropertyMap::from_properties70(&block);

        // Both malformed Compound records exist in the map…
        assert!(pm.get("BadCompound").is_some());
        assert!(pm.get("TripleCompound").is_some());
        // …but is_compound only returns true for the documented
        // zero-trailing shape.
        assert!(!pm.is_compound("BadCompound"));
        assert!(!pm.is_compound("TripleCompound"));
    }

    #[test]
    fn compound_names_enumerates_only_well_formed_compound_records() {
        // Same fixture shape as the canonical is_compound test, plus
        // one malformed Compound (shape guard rejects it) and the
        // `object` empty-body case (typeName guard rejects it).
        let block = props70(vec![
            p("TimeMarker", "Compound", "", "", vec![]),
            p("Original", "Compound", "", "", vec![]),
            p("LastSaved", "Compound", "", "", vec![]),
            p(
                "BadCompound",
                "Compound",
                "",
                "",
                vec![FbxProperty::I32(42)],
            ),
            p("SourceObject", "object", "", "", vec![]),
            p(
                "Original|ApplicationName",
                "KString",
                "",
                "",
                vec![s(b"AnApp")],
            ),
            p(
                "UnitScaleFactor",
                "double",
                "Number",
                "",
                vec![FbxProperty::F64(1.0)],
            ),
        ]);
        let pm = PropertyMap::from_properties70(&block);

        let mut names: Vec<&str> = pm.compound_names().collect();
        names.sort_unstable();
        assert_eq!(names, vec!["LastSaved", "Original", "TimeMarker"]);
    }

    #[test]
    fn as_object_ref_accepts_object_typename_with_str_or_compound_body() {
        // Cubes fixture's `SourceObject`, `LookAtProperty`,
        // `UpVectorProperty` records carry an empty body which the
        // decoder lands as `PValue::Compound` (zero trailing values).
        // `as_object_ref` surfaces "" so the caller can still detect
        // the slot's presence; a typed body (someone wires the slot
        // with an inline name) also surfaces.
        let block = props70(vec![
            p("SourceObject", "object", "", "", vec![]),
            p("LookAtProperty", "object", "", "", vec![]),
            p(
                "InlineRef",
                "object",
                "",
                "",
                vec![FbxProperty::String(b"Model::SomeNode".to_vec())],
            ),
            // A KString sitting under a slot name common in the
            // fixture (`currentUVSet`) must NOT surface here.
            p("currentUVSet", "KString", "", "U", vec![s(b"map1")]),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_object_ref("SourceObject"), Some(""));
        assert_eq!(pm.as_object_ref("LookAtProperty"), Some(""));
        assert_eq!(pm.as_object_ref("InlineRef"), Some("Model::SomeNode"));
        assert_eq!(pm.as_object_ref("currentUVSet"), None);
        assert_eq!(pm.as_object_ref("MissingRecord"), None);
    }
}
