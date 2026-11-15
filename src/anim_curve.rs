//! `AnimationCurve` — the typed key-attribute model and the curve
//! evaluator.
//!
//! Everything here follows
//! `docs/3d/fbx/fbx-sdk-export-sweeps-2026-11.md` §1 (the vendor-exporter
//! key-attribute sweep, `fixtures/fbxsdk-keyattr-sweep/`): the record
//! set (§1.1), the `KeyAttrFlags` bit table (§1.2), the four-word
//! `KeyAttrDataFloat` layout (§1.3), the `KeyAttrRefCount` run lengths
//! (§1.4), the evaluation rules (§1.5) and the extrapolation records
//! (§1.8).
//!
//! # Record set
//!
//! | Child | Type | Content |
//! |---|---|---|
//! | `Default` | `D` | the channel's static value at write time (§1.7) — it does not enter interpolation |
//! | `KeyVer` | `I` | 4008 / 4009 observed; the word layout below is the 4008-and-later one (§1.6) |
//! | `KeyTime` | `l` array | one tick count per key, 46 186 158 000 ticks per second |
//! | `KeyValueFloat` | `f` array | one value per key |
//! | `KeyAttrFlags` | `i` array | one packed word per **attribute entry** |
//! | `KeyAttrDataFloat` | `f` array | four 32-bit words per attribute entry |
//! | `KeyAttrRefCount` | `i` array | how many consecutive keys each entry covers |
//! | `Pre-Extrapolation`, `Post-Extrapolation` | nodes | `Type` letter + `Repetition`, only when not the default |
//!
//! # Evaluation (§1.5)
//!
//! The interpolation of the segment from key *i* to key *i*+1 is chosen
//! by key *i*:
//!
//! - **constant, standard** — *v*ᵢ on `[tᵢ, tᵢ₊₁)`; **constant, next** —
//!   *v*ᵢ at exactly *t*ᵢ, *v*ᵢ₊₁ on `(tᵢ, tᵢ₊₁]`;
//! - **linear** — the chord;
//! - **cubic** — the cubic Bézier in the (time, value) plane with
//!   control points `(tᵢ, vᵢ)`, `(tᵢ + wR·Δt, vᵢ + sR·wR·Δt)`,
//!   `(tᵢ₊₁ − wL·Δt, vᵢ₊₁ − sL·wL·Δt)`, `(tᵢ₊₁, vᵢ₊₁)`, read at the
//!   parameter whose time coordinate is the query time. `sR` / `wR` are
//!   the slope and weight leaving key *i*, `sL` / `wL` the slope and
//!   weight arriving at key *i*+1; a weight is 1/3 unless its weighted
//!   bit is set (with both at 1/3 the segment is the cubic Hermite
//!   segment).
//!
//! Slopes come from the tangent mode: **user** (and a key with none or
//! several of the three mode bits) reads the stored words; **automatic**
//! and **TCB** store nothing usable and are computed from the
//! neighbouring keys (formulas on [`CurveEvaluator`]). Before the first
//! and after the last key the curve holds the end value.
//!
//! # What the staged material leaves open
//!
//! - The automatic slope under the progressive-clamp bits: two read-back
//!   data sets are staged and no formula is documented (§5 item 4). The
//!   rule implemented here is the simplest one that reproduces both
//!   sets and is described on [`CurveEvaluator`]; it is a fit, not a
//!   documented law.
//! - Tangent velocity changes no documented evaluator output (§5 item
//!   5): the two fields are decoded, stored and written, never applied.
//! - The extrapolation modes' numeric behaviour (§5 item 7): the
//!   records are typed and written back; evaluation outside the key
//!   range holds the end value for every mode.
//! - `KeyVer` ≤ 4007 lays the weight word out differently (§1.6, §5
//!   item 8): for such a curve the weights are taken as 1/3.
//! - Which key's mode decides a computed *arriving* slope is pinned by
//!   one mixed curve only (`D03`): an automatic or TCB key *i*+1
//!   computes its own arriving slope, any other key *i*+1 takes the
//!   stored next-left word of entry *i*.

use crate::binary::{FbxDocument, FbxNode, FbxProperty};

/// Ticks per second of `KeyTime`.
pub const TICKS_PER_SECOND: f64 = 46_186_158_000.0;

/// `KeyVer` this crate writes (the value on every curve of the
/// vendor-exporter sweep).
pub const KEY_VER: i32 = 4009;

/// First `KeyVer` whose `KeyAttrDataFloat` words follow the layout
/// decoded here (§1.6).
pub const KEY_VER_WEIGHT_LAYOUT: i32 = 4008;

/// Divisor of the packed int16 weight and velocity fields (§1.3).
pub const PACKED_FIELD_SCALE: f64 = 9999.0;

/// The default weight word: `0x0D05` = 3333 in both halves, one third.
pub const DEFAULT_WEIGHT_WORD: u32 = 0x0D05_0D05;

/// `KeyAttrFlags` bits (§1.2).
pub mod bits {
    /// Interpolation: constant.
    pub const CONSTANT: u32 = 0x0000_0002;
    /// Interpolation: linear.
    pub const LINEAR: u32 = 0x0000_0004;
    /// Interpolation: cubic.
    pub const CUBIC: u32 = 0x0000_0008;
    /// Cubic key: automatic tangent. Constant key: "next" mode.
    pub const AUTO_OR_NEXT: u32 = 0x0000_0100;
    /// Cubic key: TCB tangent.
    pub const TCB: u32 = 0x0000_0200;
    /// Cubic key: user tangent.
    pub const USER: u32 = 0x0000_0400;
    /// Left and right tangents independent.
    pub const BREAK: u32 = 0x0000_0800;
    /// Clamp.
    pub const CLAMP: u32 = 0x0000_1000;
    /// Time-independent.
    pub const TIME_INDEPENDENT: u32 = 0x0000_2000;
    /// Progressive clamp (always written together with
    /// [`TIME_INDEPENDENT`]).
    pub const PROGRESSIVE_CLAMP: u32 = 0x0000_4000;
    /// Show the left tangent handle (editor display only).
    pub const SHOW_LEFT: u32 = 0x0010_0000;
    /// Show the right tangent handle (editor display only).
    pub const SHOW_RIGHT: u32 = 0x0020_0000;
    /// Right tangent weighted.
    pub const WEIGHTED_RIGHT: u32 = 0x0100_0000;
    /// Next-left tangent weighted.
    pub const WEIGHTED_NEXT_LEFT: u32 = 0x0200_0000;
    /// Right velocity.
    pub const VELOCITY_RIGHT: u32 = 0x1000_0000;
    /// Next-left velocity.
    pub const VELOCITY_NEXT_LEFT: u32 = 0x2000_0000;
    /// Every bit the sweep assigns a meaning.
    pub const KNOWN: u32 = CONSTANT
        | LINEAR
        | CUBIC
        | AUTO_OR_NEXT
        | TCB
        | USER
        | BREAK
        | CLAMP
        | TIME_INDEPENDENT
        | PROGRESSIVE_CLAMP
        | SHOW_LEFT
        | SHOW_RIGHT
        | WEIGHTED_RIGHT
        | WEIGHTED_NEXT_LEFT
        | VELOCITY_RIGHT
        | VELOCITY_NEXT_LEFT;
}

/// Interpolation of the segment leaving a key.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum KeyInterpolation {
    /// Step.
    Constant,
    /// Straight line.
    Linear,
    /// Cubic Bézier in the (time, value) plane.
    Cubic,
    /// No interpolation bit, or more than one (§1.2: "treat these as
    /// malformed"). The evaluator reads such a segment as linear.
    Unspecified,
}

/// Which value a constant segment holds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum ConstantMode {
    /// This key's value.
    Standard,
    /// The next key's value.
    Next,
}

/// Where a cubic key's slopes come from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum TangentMode {
    /// Computed from the neighbouring keys.
    Auto,
    /// Computed from tension / continuity / bias.
    Tcb,
    /// The stored slopes.
    User,
    /// None of the three mode bits, or several: the stored slopes are
    /// used, like a user tangent (§1.2).
    Stored,
}

/// One packed `KeyAttrFlags` word.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct KeyAttrFlags(pub u32);

impl KeyAttrFlags {
    /// A linear key, `0x2104` — the word the vendor exporter writes for
    /// a key built stand-alone and set linear (§1.2 "producer
    /// defaults").
    pub const LINEAR: Self = Self(bits::LINEAR | bits::AUTO_OR_NEXT | bits::TIME_INDEPENDENT);

    /// A linear key.
    pub fn linear() -> Self {
        Self::LINEAR
    }

    /// A constant key in the given mode.
    pub fn constant(mode: ConstantMode) -> Self {
        Self(match mode {
            ConstantMode::Standard => bits::CONSTANT,
            ConstantMode::Next => bits::CONSTANT | bits::AUTO_OR_NEXT,
        })
    }

    /// A cubic key in the given tangent mode ([`TangentMode::Stored`]
    /// sets no mode bit).
    pub fn cubic(mode: TangentMode) -> Self {
        Self(
            bits::CUBIC
                | match mode {
                    TangentMode::Auto => bits::AUTO_OR_NEXT,
                    TangentMode::Tcb => bits::TCB,
                    TangentMode::User => bits::USER,
                    TangentMode::Stored => 0,
                },
        )
    }

    /// Set or clear `mask`.
    pub fn with(self, mask: u32, on: bool) -> Self {
        Self(if on { self.0 | mask } else { self.0 & !mask })
    }

    /// `true` when every bit of `mask` is set.
    pub fn has(self, mask: u32) -> bool {
        self.0 & mask == mask
    }

    /// Interpolation kind (exactly one of the three bits, else
    /// [`KeyInterpolation::Unspecified`]).
    pub fn interpolation(self) -> KeyInterpolation {
        match self.0 & (bits::CONSTANT | bits::LINEAR | bits::CUBIC) {
            bits::CONSTANT => KeyInterpolation::Constant,
            bits::LINEAR => KeyInterpolation::Linear,
            bits::CUBIC => KeyInterpolation::Cubic,
            _ => KeyInterpolation::Unspecified,
        }
    }

    /// Constant mode — meaningful on a constant key (bit `0x100`).
    pub fn constant_mode(self) -> ConstantMode {
        if self.has(bits::AUTO_OR_NEXT) {
            ConstantMode::Next
        } else {
            ConstantMode::Standard
        }
    }

    /// Tangent mode — meaningful on a cubic key.
    pub fn tangent_mode(self) -> TangentMode {
        match self.0 & (bits::AUTO_OR_NEXT | bits::TCB | bits::USER) {
            bits::AUTO_OR_NEXT => TangentMode::Auto,
            bits::TCB => TangentMode::Tcb,
            bits::USER => TangentMode::User,
            _ => TangentMode::Stored,
        }
    }

    /// Left and right tangents independent.
    pub fn is_break(self) -> bool {
        self.has(bits::BREAK)
    }

    /// Clamp.
    pub fn is_clamp(self) -> bool {
        self.has(bits::CLAMP)
    }

    /// Time-independent.
    pub fn is_time_independent(self) -> bool {
        self.has(bits::TIME_INDEPENDENT)
    }

    /// Progressive clamp.
    pub fn is_progressive_clamp(self) -> bool {
        self.has(bits::PROGRESSIVE_CLAMP)
    }

    /// Right tangent weighted.
    pub fn right_weighted(self) -> bool {
        self.has(bits::WEIGHTED_RIGHT)
    }

    /// Next-left tangent weighted.
    pub fn next_left_weighted(self) -> bool {
        self.has(bits::WEIGHTED_NEXT_LEFT)
    }

    /// Right velocity flag.
    pub fn right_velocity(self) -> bool {
        self.has(bits::VELOCITY_RIGHT)
    }

    /// Next-left velocity flag.
    pub fn next_left_velocity(self) -> bool {
        self.has(bits::VELOCITY_NEXT_LEFT)
    }

    /// Bits outside the documented table (kept and written back; a
    /// reader masks them, §1.2).
    pub fn unassigned_bits(self) -> u32 {
        self.0 & !bits::KNOWN
    }
}

/// One attribute entry: the flag word and its four data words.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct KeyAttr {
    /// `KeyAttrFlags` word.
    pub flags: KeyAttrFlags,
    /// `KeyAttrDataFloat` words as bit patterns.
    pub data: [u32; 4],
}

impl KeyAttr {
    /// An entry with zero slopes, default weights and zero velocities.
    pub fn new(flags: KeyAttrFlags) -> Self {
        Self {
            flags,
            data: [0, 0, DEFAULT_WEIGHT_WORD, 0],
        }
    }

    /// A linear key.
    pub fn linear() -> Self {
        Self::new(KeyAttrFlags::LINEAR)
    }

    /// Word 0 as the slope leaving this key (value units per second).
    pub fn right_slope(&self) -> f32 {
        f32::from_bits(self.data[0])
    }

    /// Word 1 as the slope arriving at the next key.
    pub fn next_left_slope(&self) -> f32 {
        f32::from_bits(self.data[1])
    }

    /// Stored right weight (low half of word 2 over 9999) — applied only
    /// when [`KeyAttrFlags::right_weighted`].
    pub fn right_weight(&self) -> f64 {
        f64::from(self.data[2] as u16 as i16) / PACKED_FIELD_SCALE
    }

    /// Stored next-left weight (high half of word 2 over 9999) — applied
    /// only when [`KeyAttrFlags::next_left_weighted`].
    pub fn next_left_weight(&self) -> f64 {
        f64::from((self.data[2] >> 16) as u16 as i16) / PACKED_FIELD_SCALE
    }

    /// Stored right velocity (low half of word 3 over 9999).
    pub fn right_velocity(&self) -> f64 {
        f64::from(self.data[3] as u16 as i16) / PACKED_FIELD_SCALE
    }

    /// Stored next-left velocity (high half of word 3 over 9999).
    pub fn next_left_velocity(&self) -> f64 {
        f64::from((self.data[3] >> 16) as u16 as i16) / PACKED_FIELD_SCALE
    }

    /// Words 0–2 as tension, continuity, bias (a TCB key).
    pub fn tcb(&self) -> [f32; 3] {
        [
            f32::from_bits(self.data[0]),
            f32::from_bits(self.data[1]),
            f32::from_bits(self.data[2]),
        ]
    }

    /// Store the two slopes.
    pub fn with_slopes(mut self, right: f32, next_left: f32) -> Self {
        self.data[0] = right.to_bits();
        self.data[1] = next_left.to_bits();
        self
    }

    /// Store the two weights and set both weighted bits. Each weight is
    /// quantised as the §1.3 table shows: clamped to the stored range
    /// 1 … 9899, otherwise `floor(weight × 9999)`.
    pub fn with_weights(mut self, right: f64, next_left: f64) -> Self {
        let q = |w: f64| -> u32 {
            let scaled = (w * PACKED_FIELD_SCALE).floor();
            if scaled.is_nan() {
                1
            } else {
                scaled.clamp(1.0, 9899.0) as u32
            }
        };
        self.data[2] = q(right) | (q(next_left) << 16);
        self.flags = self
            .flags
            .with(bits::WEIGHTED_RIGHT | bits::WEIGHTED_NEXT_LEFT, true);
        self
    }

    /// Store the two velocities and set both velocity bits. Each is
    /// `trunc(velocity × 9999)` kept modulo 2¹⁶ (§1.3 table).
    pub fn with_velocities(mut self, right: f64, next_left: f64) -> Self {
        let q = |v: f64| -> u32 {
            let scaled = (v * PACKED_FIELD_SCALE).trunc();
            if scaled.is_finite() {
                u32::from((scaled as i64) as u16)
            } else {
                0
            }
        };
        self.data[3] = q(right) | (q(next_left) << 16);
        self.flags = self
            .flags
            .with(bits::VELOCITY_RIGHT | bits::VELOCITY_NEXT_LEFT, true);
        self
    }

    /// Store tension, continuity and bias (word 3 zero) — the layout of
    /// a TCB key.
    pub fn with_tcb(mut self, tension: f32, continuity: f32, bias: f32) -> Self {
        self.data = [tension.to_bits(), continuity.to_bits(), bias.to_bits(), 0];
        self
    }
}

/// One `KeyAttrRefCount` run: `count` consecutive keys share `attr`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct KeyAttrRun {
    /// The shared entry.
    pub attr: KeyAttr,
    /// Number of consecutive keys it covers.
    pub count: u32,
}

/// Extrapolation mode letter (§1.8).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum ExtrapolationKind {
    /// `R` — repetition.
    Repetition,
    /// `M` — mirror repetition.
    MirrorRepetition,
    /// `K` — keep slope.
    KeepSlope,
    /// `A` — relative repetition.
    RelativeRepetition,
    /// Any other letter, kept verbatim.
    Other(u8),
}

impl ExtrapolationKind {
    /// The mode for a `Type` letter.
    pub fn from_letter(letter: u8) -> Self {
        match letter {
            b'R' => Self::Repetition,
            b'M' => Self::MirrorRepetition,
            b'K' => Self::KeepSlope,
            b'A' => Self::RelativeRepetition,
            other => Self::Other(other),
        }
    }

    /// The `Type` letter.
    pub fn letter(self) -> u8 {
        match self {
            Self::Repetition => b'R',
            Self::MirrorRepetition => b'M',
            Self::KeepSlope => b'K',
            Self::RelativeRepetition => b'A',
            Self::Other(b) => b,
        }
    }
}

/// A `Pre-Extrapolation` / `Post-Extrapolation` record. The default
/// mode (constant) has no record.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Extrapolation {
    /// `Type`.
    pub kind: ExtrapolationKind,
    /// `Repetition`.
    pub repetition: i32,
}

/// A typed `AnimationCurve`.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct AnimCurve {
    /// `Default`.
    pub default: f64,
    /// `KeyVer`.
    pub key_ver: i32,
    /// `KeyTime` ticks.
    pub key_times: Vec<i64>,
    /// `KeyValueFloat`.
    pub key_values: Vec<f32>,
    /// The attribute entries with their run lengths, file order.
    pub runs: Vec<KeyAttrRun>,
    /// `Pre-Extrapolation`.
    pub pre_extrapolation: Option<Extrapolation>,
    /// `Post-Extrapolation`.
    pub post_extrapolation: Option<Extrapolation>,
}

impl AnimCurve {
    /// A curve of linear keys: one attribute entry covering every key,
    /// `Default` = the first value.
    pub fn new(key_times: Vec<i64>, key_values: Vec<f32>) -> Self {
        let n = key_times.len().min(key_values.len());
        Self {
            default: key_values.first().copied().map_or(0.0, f64::from),
            key_ver: KEY_VER,
            runs: if n == 0 {
                Vec::new()
            } else {
                vec![KeyAttrRun {
                    attr: KeyAttr::linear(),
                    count: n as u32,
                }]
            },
            key_times,
            key_values,
            pre_extrapolation: None,
            post_extrapolation: None,
        }
    }

    /// Builder: `Default`.
    pub fn with_default(mut self, default: f64) -> Self {
        self.default = default;
        self
    }

    /// Builder: one attribute per key (one run each; see
    /// [`AnimCurve::compact_runs`]).
    pub fn with_key_attrs(mut self, attrs: impl IntoIterator<Item = KeyAttr>) -> Self {
        self.runs = attrs
            .into_iter()
            .map(|attr| KeyAttrRun { attr, count: 1 })
            .collect();
        self
    }

    /// Builder: the extrapolation records.
    pub fn with_extrapolation(
        mut self,
        pre: Option<Extrapolation>,
        post: Option<Extrapolation>,
    ) -> Self {
        self.pre_extrapolation = pre;
        self.post_extrapolation = post;
        self
    }

    /// Number of keys.
    pub fn len(&self) -> usize {
        self.key_times.len().min(self.key_values.len())
    }

    /// `true` when the curve has no key.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// `true` when the run lengths sum to exactly the key count — the
    /// well-formed case (§1.4: any other sum is "corrupted animation
    /// data" to the vendor reader).
    pub fn runs_cover_keys(&self) -> bool {
        self.runs.iter().map(|r| u64::from(r.count)).sum::<u64>() == self.len() as u64
    }

    /// The attribute of every key, runs expanded in order. Keys past
    /// the last run (a malformed curve) read as linear; surplus run
    /// length is ignored.
    pub fn key_attrs(&self) -> Vec<KeyAttr> {
        let n = self.len();
        let mut out = Vec::with_capacity(n);
        for run in &self.runs {
            for _ in 0..run.count {
                if out.len() == n {
                    return out;
                }
                out.push(run.attr);
            }
        }
        out.resize(n, KeyAttr::linear());
        out
    }

    /// Merge equal neighbouring entries into one run — the shape the
    /// vendor exporter writes after a read / write cycle (§1.4;
    /// equality includes the data words).
    pub fn compact_runs(&mut self) {
        let mut out: Vec<KeyAttrRun> = Vec::with_capacity(self.runs.len());
        for run in self.runs.iter().filter(|r| r.count > 0) {
            match out.last_mut() {
                Some(last) if last.attr == run.attr => {
                    last.count = last.count.saturating_add(run.count);
                }
                _ => out.push(*run),
            }
        }
        self.runs = out;
    }

    /// One run per key — the shape the vendor exporter writes for a
    /// scene built in memory (§1.4).
    pub fn expand_runs(&mut self) {
        self.runs = self
            .key_attrs()
            .into_iter()
            .map(|attr| KeyAttrRun { attr, count: 1 })
            .collect();
    }

    /// `true` when every segment is linear (or unspecified): the key
    /// values alone carry the curve.
    pub fn is_piecewise_linear(&self) -> bool {
        let attrs = self.key_attrs();
        let segments = attrs.len().saturating_sub(1);
        attrs[..segments].iter().all(|a| {
            matches!(
                a.flags.interpolation(),
                KeyInterpolation::Linear | KeyInterpolation::Unspecified
            )
        })
    }

    /// Parse an `AnimationCurve` element. `None` when `KeyTime` /
    /// `KeyValueFloat` are missing or disagree in length.
    ///
    /// Attribute arrays are read leniently where the vendor reader
    /// reports corruption (§1.4): a missing `KeyAttrRefCount` reads as
    /// one run over the whole curve for a single entry, or one key per
    /// entry otherwise; missing data words read as zero.
    pub fn from_element(element: &FbxNode) -> Option<Self> {
        let key_times: Vec<i64> = match element.child("KeyTime")?.properties.first()? {
            FbxProperty::I64Array(a) => a.clone(),
            FbxProperty::I32Array(a) => a.iter().map(|v| i64::from(*v)).collect(),
            _ => return None,
        };
        let key_values: Vec<f32> = match element.child("KeyValueFloat")?.properties.first()? {
            FbxProperty::F32Array(a) => a.clone(),
            FbxProperty::F64Array(a) => a.iter().map(|v| *v as f32).collect(),
            FbxProperty::I32Array(a) => a.iter().map(|v| *v as f32).collect(),
            FbxProperty::I64Array(a) => a.iter().map(|v| *v as f32).collect(),
            _ => return None,
        };
        if key_times.len() != key_values.len() {
            return None;
        }
        let ints = |name: &str| -> Option<Vec<i64>> {
            match element.child(name)?.properties.first()? {
                FbxProperty::I32Array(a) => Some(a.iter().map(|v| i64::from(*v)).collect()),
                FbxProperty::I64Array(a) => Some(a.clone()),
                _ => None,
            }
        };
        let flags = ints("KeyAttrFlags").unwrap_or_default();
        let words: Vec<u32> = match element
            .child("KeyAttrDataFloat")
            .and_then(|n| n.properties.first())
        {
            Some(FbxProperty::F32Array(a)) => a.iter().map(|v| v.to_bits()).collect(),
            Some(FbxProperty::I32Array(a)) => a
                .iter()
                .map(|v| u32::from_ne_bytes(v.to_ne_bytes()))
                .collect(),
            Some(FbxProperty::F64Array(a)) => a.iter().map(|v| (*v as f32).to_bits()).collect(),
            _ => Vec::new(),
        };
        let counts = ints("KeyAttrRefCount");
        let n = key_times.len();
        let runs = flags
            .iter()
            .enumerate()
            .map(|(k, f)| {
                let mut data = [0u32; 4];
                for (j, d) in data.iter_mut().enumerate() {
                    *d = words.get(k * 4 + j).copied().unwrap_or(0);
                }
                let count = match &counts {
                    Some(c) => c.get(k).copied().unwrap_or(0).clamp(0, i64::from(u32::MAX)) as u32,
                    None if flags.len() == 1 => n as u32,
                    None => 1,
                };
                KeyAttrRun {
                    attr: KeyAttr {
                        flags: KeyAttrFlags(*f as u32),
                        data,
                    },
                    count,
                }
            })
            .collect();
        let extrapolation = |name: &str| -> Option<Extrapolation> {
            let e = element.child(name)?;
            let letter = match e.child("Type")?.properties.first()? {
                FbxProperty::Char(c) => *c,
                FbxProperty::Bool(true) => b'T',
                FbxProperty::Bool(false) => b'F',
                FbxProperty::String(s) if s.len() == 1 => s[0],
                _ => return None,
            };
            let repetition = e
                .child("Repetition")
                .and_then(|r| r.properties.first())
                .and_then(FbxProperty::as_i64)
                .unwrap_or(0) as i32;
            Some(Extrapolation {
                kind: ExtrapolationKind::from_letter(letter),
                repetition,
            })
        };
        Some(Self {
            default: element
                .child("Default")
                .and_then(|n| n.properties.first())
                .and_then(FbxProperty::as_f64_loose)
                .unwrap_or(0.0),
            key_ver: element
                .child("KeyVer")
                .and_then(|n| n.properties.first())
                .and_then(FbxProperty::as_i64)
                .unwrap_or(i64::from(KEY_VER)) as i32,
            key_times,
            key_values,
            runs,
            pre_extrapolation: extrapolation("Pre-Extrapolation"),
            post_extrapolation: extrapolation("Post-Extrapolation"),
        })
    }

    /// The element body in the §1.1 record order.
    pub fn to_body(&self) -> Vec<FbxNode> {
        let leaf = |name: &str, p: FbxProperty| FbxNode {
            name: name.to_string(),
            properties: vec![p],
            children: Vec::new(),
        };
        let mut body = vec![
            leaf("Default", FbxProperty::F64(self.default)),
            leaf("KeyVer", FbxProperty::I32(self.key_ver)),
            leaf("KeyTime", FbxProperty::I64Array(self.key_times.clone())),
            leaf(
                "KeyValueFloat",
                FbxProperty::F32Array(self.key_values.clone()),
            ),
            leaf(
                "KeyAttrFlags",
                FbxProperty::I32Array(
                    self.runs
                        .iter()
                        .map(|r| i32::from_ne_bytes(r.attr.flags.0.to_ne_bytes()))
                        .collect(),
                ),
            ),
            leaf(
                "KeyAttrDataFloat",
                FbxProperty::F32Array(
                    self.runs
                        .iter()
                        .flat_map(|r| r.attr.data)
                        .map(f32::from_bits)
                        .collect(),
                ),
            ),
            leaf(
                "KeyAttrRefCount",
                FbxProperty::I32Array(
                    self.runs
                        .iter()
                        .map(|r| i32::try_from(r.count).unwrap_or(i32::MAX))
                        .collect(),
                ),
            ),
        ];
        for (name, e) in [
            ("Pre-Extrapolation", self.pre_extrapolation),
            ("Post-Extrapolation", self.post_extrapolation),
        ] {
            if let Some(e) = e {
                body.push(FbxNode {
                    name: name.to_string(),
                    properties: Vec::new(),
                    children: vec![
                        leaf("Type", FbxProperty::Char(e.kind.letter())),
                        leaf("Repetition", FbxProperty::I32(e.repetition)),
                    ],
                });
            }
        }
        body
    }

    /// The `AnimationCurve` element (empty object name, class
    /// `AnimCurve`, empty sub-type — the header every staged curve
    /// carries).
    pub fn to_element(&self, id: i64) -> FbxNode {
        FbxNode {
            name: "AnimationCurve".to_string(),
            properties: vec![
                FbxProperty::I64(id),
                FbxProperty::String(b"\x00\x01AnimCurve".to_vec()),
                FbxProperty::String(Vec::new()),
            ],
            children: self.to_body(),
        }
    }

    /// The precomputed evaluator.
    pub fn evaluator(&self) -> CurveEvaluator {
        CurveEvaluator::new(self)
    }

    /// The curve's value at `seconds` (builds a [`CurveEvaluator`] per
    /// call — use [`AnimCurve::evaluator`] to sample repeatedly).
    pub fn evaluate(&self, seconds: f64) -> f64 {
        self.evaluator().sample(seconds)
    }
}

/// How one segment is evaluated.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Segment {
    /// Holds this key's value.
    Hold,
    /// Holds the next key's value after the key instant.
    HoldNext,
    /// The chord.
    Line,
    /// The cubic Bézier: slopes and weights leaving the key / arriving
    /// at the next.
    Bezier {
        slope_out: f64,
        slope_in: f64,
        weight_out: f64,
        weight_in: f64,
    },
}

/// A curve prepared for repeated sampling: per-key times in seconds,
/// values, and the resolved shape of every segment.
///
/// # Computed slopes (§1.5)
///
/// With keys (*t*ᵢ, *v*ᵢ) and chord slopes
/// *d*₋ = (*v*ᵢ−*v*ᵢ₋₁)/(*t*ᵢ−*t*ᵢ₋₁), *d*₊ = (*v*ᵢ₊₁−*v*ᵢ)/(*t*ᵢ₊₁−*t*ᵢ):
///
/// - **automatic** — interior key: ½(*d*₋ + *d*₊); with the
///   time-independent bit: (*v*ᵢ₊₁ − *v*ᵢ₋₁)/(*t*ᵢ₊₁ − *t*ᵢ₋₁); first and
///   last key: the single neighbouring chord slope. With the clamp bit
///   the slope is 0 at a key whose value equals a neighbour's.
/// - **automatic with progressive clamp** — fitted to the two staged
///   read-back sets (`I10`, `A03`): 0 at the first and last key and at
///   an interior key that is a local extremum or level with a
///   neighbour; otherwise the time-independent slope.
/// - **TCB** — arriving ½(1−T)(1+B)(1−C)·*d*₋ + ½(1−T)(1−B)(1+C)·*d*₊,
///   leaving ½(1−T)(1+B)(1+C)·*d*₋ + ½(1−T)(1−B)(1−C)·*d*₊; first key
///   (1−T)(1−B)(1−C)·*d*₊, last key (1−T)(1+B)(1−C)·*d*₋.
#[derive(Clone, Debug)]
pub struct CurveEvaluator {
    times: Vec<f64>,
    values: Vec<f64>,
    segments: Vec<Segment>,
}

impl CurveEvaluator {
    /// Resolve every segment of `curve`.
    pub fn new(curve: &AnimCurve) -> Self {
        let n = curve.len();
        let times: Vec<f64> = curve.key_times[..n]
            .iter()
            .map(|t| *t as f64 / TICKS_PER_SECOND)
            .collect();
        let values: Vec<f64> = curve.key_values[..n]
            .iter()
            .map(|v| f64::from(*v))
            .collect();
        let attrs = curve.key_attrs();
        let weights_apply = curve.key_ver >= KEY_VER_WEIGHT_LAYOUT;
        let third = 1.0 / 3.0;
        let mut segments = Vec::with_capacity(n.saturating_sub(1));
        for i in 0..n.saturating_sub(1) {
            let a = &attrs[i];
            segments.push(match a.flags.interpolation() {
                KeyInterpolation::Constant => match a.flags.constant_mode() {
                    ConstantMode::Standard => Segment::Hold,
                    ConstantMode::Next => Segment::HoldNext,
                },
                KeyInterpolation::Linear | KeyInterpolation::Unspecified => Segment::Line,
                KeyInterpolation::Cubic => {
                    let slope_out = match a.flags.tangent_mode() {
                        TangentMode::Auto => auto_slope(&times, &values, i, a.flags),
                        TangentMode::Tcb => tcb_slopes(&times, &values, i, a.tcb()).1,
                        TangentMode::User | TangentMode::Stored => f64::from(a.right_slope()),
                    };
                    let next = &attrs[i + 1];
                    let next_mode = (next.flags.interpolation() == KeyInterpolation::Cubic)
                        .then(|| next.flags.tangent_mode());
                    let slope_in = match next_mode {
                        Some(TangentMode::Auto) => auto_slope(&times, &values, i + 1, next.flags),
                        Some(TangentMode::Tcb) => tcb_slopes(&times, &values, i + 1, next.tcb()).0,
                        _ => f64::from(a.next_left_slope()),
                    };
                    let stored = a.flags.tangent_mode() != TangentMode::Tcb && weights_apply;
                    Segment::Bezier {
                        slope_out: finite_or_zero(slope_out),
                        slope_in: finite_or_zero(slope_in),
                        weight_out: if stored && a.flags.right_weighted() {
                            a.right_weight().clamp(0.0, 1.0)
                        } else {
                            third
                        },
                        weight_in: if stored && a.flags.next_left_weighted() {
                            a.next_left_weight().clamp(0.0, 1.0)
                        } else {
                            third
                        },
                    }
                }
            });
        }
        Self {
            times,
            values,
            segments,
        }
    }

    /// Key times in seconds.
    pub fn key_times(&self) -> &[f64] {
        &self.times
    }

    /// Key values.
    pub fn key_values(&self) -> &[f64] {
        &self.values
    }

    /// `(arriving, leaving)` slope at every key, value units per second
    /// — the numbers the vendor reader reports as left / right
    /// derivative for cubic segments. A key next to a non-cubic segment
    /// reports that segment's own slope (the chord for a line, 0 for a
    /// step); the first key's arriving slope and the last key's leaving
    /// slope repeat the other side.
    pub fn key_slopes(&self) -> Vec<(f64, f64)> {
        let n = self.times.len();
        let side = |seg: usize, leaving: bool| -> f64 {
            match self.segments[seg] {
                Segment::Hold | Segment::HoldNext => 0.0,
                Segment::Line => {
                    let dt = self.times[seg + 1] - self.times[seg];
                    if dt > 0.0 {
                        (self.values[seg + 1] - self.values[seg]) / dt
                    } else {
                        0.0
                    }
                }
                Segment::Bezier {
                    slope_out,
                    slope_in,
                    ..
                } => {
                    if leaving {
                        slope_out
                    } else {
                        slope_in
                    }
                }
            }
        };
        (0..n)
            .map(|i| {
                let leaving = (i + 1 < n).then(|| side(i, true));
                let arriving = (i > 0).then(|| side(i - 1, false));
                (
                    arriving.or(leaving).unwrap_or(0.0),
                    leaving.or(arriving).unwrap_or(0.0),
                )
            })
            .collect()
    }

    /// `(arriving, leaving)` tangent weight at every key — the fraction
    /// of the adjacent key interval each handle spans: the stored value
    /// where the segment is cubic and its weighted bit is set, one
    /// third everywhere else.
    pub fn key_weights(&self) -> Vec<(f64, f64)> {
        let n = self.times.len();
        let third = 1.0 / 3.0;
        let side = |seg: usize, leaving: bool| -> f64 {
            match self.segments[seg] {
                Segment::Bezier {
                    weight_out,
                    weight_in,
                    ..
                } => {
                    if leaving {
                        weight_out
                    } else {
                        weight_in
                    }
                }
                _ => third,
            }
        };
        (0..n)
            .map(|i| {
                (
                    if i > 0 { side(i - 1, false) } else { third },
                    if i + 1 < n { side(i, true) } else { third },
                )
            })
            .collect()
    }

    /// `true` when segment `i` (from key `i` to key `i + 1`) is a
    /// straight line.
    pub fn segment_is_linear(&self, i: usize) -> bool {
        matches!(self.segments.get(i), Some(Segment::Line))
    }

    /// `Some(next)` when segment `i` is a step: `false` holds key `i`'s
    /// value, `true` the next key's.
    pub fn segment_step(&self, i: usize) -> Option<bool> {
        match self.segments.get(i)? {
            Segment::Hold => Some(false),
            Segment::HoldNext => Some(true),
            _ => None,
        }
    }

    /// The value at `seconds`. Outside the key range the end value
    /// holds; an empty curve reads 0.
    pub fn sample(&self, seconds: f64) -> f64 {
        let n = self.times.len();
        if n == 0 {
            return 0.0;
        }
        // A NaN query reads the first key.
        if seconds.is_nan() || seconds <= self.times[0] {
            return self.values[0];
        }
        if seconds >= self.times[n - 1] {
            return self.values[n - 1];
        }
        // Last key at or before the query.
        let i = self.times.partition_point(|t| *t <= seconds) - 1;
        let (t0, t1) = (self.times[i], self.times[i + 1]);
        let (v0, v1) = (self.values[i], self.values[i + 1]);
        if seconds == t0 {
            return v0;
        }
        let dt = t1 - t0;
        if dt <= 0.0 {
            return v1;
        }
        let u = (seconds - t0) / dt;
        match self.segments[i] {
            Segment::Hold => v0,
            Segment::HoldNext => v1,
            Segment::Line => v0 + (v1 - v0) * u,
            Segment::Bezier {
                slope_out,
                slope_in,
                weight_out,
                weight_in,
            } => {
                let s = bezier_parameter(u, weight_out, 1.0 - weight_in);
                let c1 = v0 + slope_out * weight_out * dt;
                let c2 = v1 - slope_in * weight_in * dt;
                let r = 1.0 - s;
                r * r * r * v0 + 3.0 * r * r * s * c1 + 3.0 * r * s * s * c2 + s * s * s * v1
            }
        }
    }
}

fn finite_or_zero(x: f64) -> f64 {
    if x.is_finite() { x } else { 0.0 }
}

/// Chord slope from key `a` to key `b` (0 for coincident times).
fn chord(times: &[f64], values: &[f64], a: usize, b: usize) -> f64 {
    let dt = times[b] - times[a];
    if dt > 0.0 {
        (values[b] - values[a]) / dt
    } else {
        0.0
    }
}

/// Automatic slope at key `i` (see [`CurveEvaluator`]).
fn auto_slope(times: &[f64], values: &[f64], i: usize, flags: KeyAttrFlags) -> f64 {
    let n = times.len();
    if n < 2 {
        return 0.0;
    }
    let interior = i > 0 && i + 1 < n;
    if flags.is_progressive_clamp() {
        if !interior {
            return 0.0;
        }
        let before = values[i] - values[i - 1];
        let after = values[i + 1] - values[i];
        if before * after <= 0.0 {
            return 0.0;
        }
        return chord(times, values, i - 1, i + 1);
    }
    if !interior {
        return if i == 0 {
            chord(times, values, 0, 1)
        } else {
            chord(times, values, n - 2, n - 1)
        };
    }
    if flags.is_clamp() && (values[i] == values[i - 1] || values[i] == values[i + 1]) {
        return 0.0;
    }
    if flags.is_time_independent() {
        chord(times, values, i - 1, i + 1)
    } else {
        0.5 * (chord(times, values, i - 1, i) + chord(times, values, i, i + 1))
    }
}

/// `(arriving, leaving)` TCB slope at key `i` (see [`CurveEvaluator`]).
fn tcb_slopes(times: &[f64], values: &[f64], i: usize, tcb: [f32; 3]) -> (f64, f64) {
    let n = times.len();
    if n < 2 {
        return (0.0, 0.0);
    }
    let [t, c, b] = tcb.map(f64::from);
    if i == 0 {
        let s = (1.0 - t) * (1.0 - b) * (1.0 - c) * chord(times, values, 0, 1);
        return (s, s);
    }
    if i + 1 == n {
        let s = (1.0 - t) * (1.0 + b) * (1.0 - c) * chord(times, values, n - 2, n - 1);
        return (s, s);
    }
    let before = chord(times, values, i - 1, i);
    let after = chord(times, values, i, i + 1);
    let h = 0.5 * (1.0 - t);
    (
        h * (1.0 + b) * (1.0 - c) * before + h * (1.0 - b) * (1.0 + c) * after,
        h * (1.0 + b) * (1.0 + c) * before + h * (1.0 - b) * (1.0 - c) * after,
    )
}

/// The parameter `s ∈ [0, 1]` at which the cubic Bézier with abscissae
/// `0, x1, x2, 1` reaches `u ∈ [0, 1]`.
///
/// With `x1, x2 ∈ [0, 1]` the abscissa polynomial is non-decreasing on
/// `[0, 1]` (its derivative `3[(1−s)²x1 + 2s(1−s)(x2−x1) + s²(1−x2)]`
/// has no negative minimum for such control values), so the root is
/// bracketed by `[0, 1]`: Newton steps are taken while they stay inside
/// the bracket, bisection otherwise.
fn bezier_parameter(u: f64, x1: f64, x2: f64) -> f64 {
    // Both handles at one third: the abscissa is the identity.
    if (x1 - 1.0 / 3.0).abs() < 1e-12 && (x2 - 2.0 / 3.0).abs() < 1e-12 {
        return u;
    }
    let x = |s: f64| {
        let r = 1.0 - s;
        3.0 * r * r * s * x1 + 3.0 * r * s * s * x2 + s * s * s
    };
    let dx = |s: f64| {
        let r = 1.0 - s;
        3.0 * (r * r * x1 + 2.0 * r * s * (x2 - x1) + s * s * (1.0 - x2))
    };
    let (mut lo, mut hi) = (0.0f64, 1.0f64);
    let mut s = u;
    for _ in 0..64 {
        let err = x(s) - u;
        if err.abs() < 1e-14 {
            break;
        }
        if err > 0.0 {
            hi = s;
        } else {
            lo = s;
        }
        let d = dx(s);
        let newton = s - err / d;
        s = if d > 1e-12 && newton > lo && newton < hi {
            newton
        } else {
            0.5 * (lo + hi)
        };
        if hi - lo < 1e-15 {
            break;
        }
    }
    s
}

/// An `AnimationCurve` with the join that names it: the stack and layer
/// it plays in, the object and property its curve node drives, and its
/// channel on that node.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct BoundCurve {
    /// `AnimationStack` name.
    pub stack: Option<String>,
    /// `AnimationLayer` name.
    pub layer: Option<String>,
    /// Display name of the driven object (a `Model`, a
    /// `BlendShapeChannel`, …).
    pub target: Option<String>,
    /// Driven property (`"Lcl Translation"`, `"DeformPercent"`, …).
    pub property: Option<String>,
    /// Channel on the curve node (`"d|X"`, `"d|DeformPercent"`, …).
    pub channel: Option<String>,
    /// The curve.
    pub curve: AnimCurve,
}

/// Every `AnimationCurve` of a document with its join, document order.
pub fn bound_curves(doc: &FbxDocument) -> Vec<BoundCurve> {
    use std::collections::HashMap;
    let Some(objects) = doc.root.child("Objects") else {
        return Vec::new();
    };
    let mut names: HashMap<i64, (&str, String)> = HashMap::new();
    let mut curves: Vec<(i64, AnimCurve)> = Vec::new();
    for child in &objects.children {
        let Some(id) = child.properties.first().and_then(FbxProperty::as_i64) else {
            continue;
        };
        let name = child
            .properties
            .get(1)
            .and_then(FbxProperty::as_str)
            .and_then(|s| s.split('\u{0}').next())
            .unwrap_or("")
            .to_owned();
        names.insert(id, (child.name.as_str(), name));
        if child.name == "AnimationCurve"
            && let Some(curve) = AnimCurve::from_element(child)
        {
            curves.push((id, curve));
        }
    }
    let class = |id: i64| names.get(&id).map(|(c, _)| *c);
    let mut curve_to_node: HashMap<i64, (i64, String)> = HashMap::new();
    let mut node_to_target: HashMap<i64, (i64, String)> = HashMap::new();
    let mut node_to_layer: HashMap<i64, i64> = HashMap::new();
    let mut layer_to_stack: HashMap<i64, i64> = HashMap::new();
    if let Some(conns) = doc.root.child("Connections") {
        for c in conns.children_named("C") {
            let kind = c.properties.first().and_then(FbxProperty::as_str);
            let child_id = c.properties.get(1).and_then(FbxProperty::as_i64);
            let parent_id = c.properties.get(2).and_then(FbxProperty::as_i64);
            let prop = c.properties.get(3).and_then(FbxProperty::as_str);
            let (Some(kind), Some(child_id), Some(parent_id)) = (kind, child_id, parent_id) else {
                continue;
            };
            match (kind, prop, class(child_id), class(parent_id)) {
                ("OP", Some(p), Some("AnimationCurve"), Some("AnimationCurveNode")) => {
                    curve_to_node.insert(child_id, (parent_id, p.to_owned()));
                }
                ("OP", Some(p), Some("AnimationCurveNode"), _) => {
                    node_to_target.insert(child_id, (parent_id, p.to_owned()));
                }
                ("OO", _, Some("AnimationCurveNode"), Some("AnimationLayer")) => {
                    node_to_layer.insert(child_id, parent_id);
                }
                ("OO", _, Some("AnimationLayer"), Some("AnimationStack")) => {
                    layer_to_stack.insert(child_id, parent_id);
                }
                _ => {}
            }
        }
    }
    let name_of = |id: &i64| names.get(id).map(|(_, n)| n.clone());
    curves
        .into_iter()
        .map(|(id, curve)| {
            let wiring = curve_to_node.get(&id);
            let node = wiring.map(|(n, _)| n);
            let target = node.and_then(|n| node_to_target.get(n));
            let layer = node.and_then(|n| node_to_layer.get(n));
            BoundCurve {
                stack: layer.and_then(|l| layer_to_stack.get(l)).and_then(name_of),
                layer: layer.and_then(name_of),
                target: target.and_then(|(t, _)| name_of(t)),
                property: target.map(|(_, p)| p.clone()),
                channel: wiring.map(|(_, c)| c.clone()),
                curve,
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn secs(s: f64) -> i64 {
        (s * TICKS_PER_SECOND).round() as i64
    }

    fn four_keys(attr: KeyAttr) -> AnimCurve {
        AnimCurve::new(
            vec![secs(0.0), secs(1.0), secs(2.0), secs(3.0)],
            vec![0.0, 10.0, 4.0, 7.0],
        )
        .with_key_attrs([attr; 4])
    }

    #[test]
    fn flag_words_decode_per_the_bit_table() {
        // §1.2 evidence column.
        let f = KeyAttrFlags(0x2002);
        assert_eq!(f.interpolation(), KeyInterpolation::Constant);
        assert_eq!(f.constant_mode(), ConstantMode::Standard);
        assert_eq!(KeyAttrFlags(0x2102).constant_mode(), ConstantMode::Next);
        assert_eq!(
            KeyAttrFlags(0x2104).interpolation(),
            KeyInterpolation::Linear
        );
        assert_eq!(KeyAttrFlags(0x108).tangent_mode(), TangentMode::Auto);
        assert_eq!(KeyAttrFlags(0x208).tangent_mode(), TangentMode::Tcb);
        assert_eq!(KeyAttrFlags(0x408).tangent_mode(), TangentMode::User);
        assert_eq!(KeyAttrFlags(0x808).tangent_mode(), TangentMode::Stored);
        assert_eq!(KeyAttrFlags(0x508).tangent_mode(), TangentMode::Stored);
        assert!(KeyAttrFlags(0x908).is_break());
        assert!(KeyAttrFlags(0x1108).is_clamp());
        assert!(KeyAttrFlags(0x6108).is_progressive_clamp());
        assert!(KeyAttrFlags(0x0300_0408).right_weighted());
        assert!(KeyAttrFlags(0x0300_0408).next_left_weighted());
        assert!(KeyAttrFlags(0x3000_0408).right_velocity());
        assert_eq!(KeyAttrFlags(0x4C0F_0104).unassigned_bits(), 0x4C0F_0000);
        assert_eq!(
            KeyAttrFlags(0x106).interpolation(),
            KeyInterpolation::Unspecified
        );
        // The sampled-curve block of the 2026-09 sweep is a linear key.
        assert_eq!(
            KeyAttrFlags(24836).interpolation(),
            KeyInterpolation::Linear
        );
        // Builders produce the documented words.
        assert_eq!(KeyAttrFlags::constant(ConstantMode::Standard).0, 0x2);
        assert_eq!(KeyAttrFlags::cubic(TangentMode::User).0, 0x408);
        assert_eq!(
            KeyAttrFlags::cubic(TangentMode::Auto)
                .with(bits::BREAK, true)
                .0,
            0x908
        );
    }

    #[test]
    fn packed_fields_quantise_like_the_table() {
        // §1.3 quantisation table.
        let weights = [
            (0.0, 1),
            (0.0001, 1),
            (0.00015, 1),
            (0.2, 1999),
            (0.5, 4999),
            (0.6, 5999),
            (0.99995, 9899),
            (1.0, 9899),
            (1.5, 9899),
            (4.0, 9899),
        ];
        for (w, stored) in weights {
            let a = KeyAttr::new(KeyAttrFlags::cubic(TangentMode::User)).with_weights(w, w);
            assert_eq!(a.data[2] & 0xFFFF, stored, "weight {w}");
            assert_eq!(a.data[2] >> 16, stored, "weight {w}");
        }
        let velocities: [(f64, u32); 9] = [
            (0.0, 0),
            (0.0001, 0),
            (-0.00015, 0xFFFF),
            (0.3, 2999),
            (0.5, 4999),
            (-1.0, 0xD8F1),
            (3.0, 29997),
            (-3.5, 0x774C),
            (40.0, 0x1A58),
        ];
        for (v, stored) in velocities {
            let a = KeyAttr::new(KeyAttrFlags::cubic(TangentMode::User)).with_velocities(v, v);
            assert_eq!(a.data[3] & 0xFFFF, stored, "velocity {v}");
        }
        // W04: right 0.6, next-left 0.2 → 07CF176F, flags 0x03000408.
        let w04 = KeyAttr::new(KeyAttrFlags::cubic(TangentMode::User))
            .with_slopes(1.5, -2.5)
            .with_weights(0.6, 0.2);
        assert_eq!(w04.data, [0x3FC0_0000, 0xC020_0000, 0x07CF_176F, 0]);
        assert_eq!(w04.flags.0, 0x0300_0408);
        assert!((w04.right_weight() - 0.59996).abs() < 1e-5);
        assert!((w04.next_left_weight() - 0.19992).abs() < 1e-5);
    }

    #[test]
    fn steps_lines_and_hermite_segments() {
        let hold = four_keys(KeyAttr::new(KeyAttrFlags::constant(ConstantMode::Standard)));
        assert_eq!(hold.evaluate(0.5), 0.0);
        assert_eq!(hold.evaluate(1.0), 10.0);
        let next = four_keys(KeyAttr::new(KeyAttrFlags::constant(ConstantMode::Next)));
        assert_eq!(next.evaluate(0.0), 0.0);
        assert_eq!(next.evaluate(0.125), 10.0);
        assert_eq!(next.evaluate(1.0), 10.0);
        assert_eq!(next.evaluate(1.125), 4.0);
        let line = four_keys(KeyAttr::linear());
        assert!((line.evaluate(1.5) - 7.0).abs() < 1e-12);
        assert!(line.is_piecewise_linear());
        // I06: slopes 1.5 / -2.5, Hermite midpoint
        // (v0+v1)/2 + (sR - sL)·dt/8 = 5 + 4/8 = 5.5.
        let user =
            four_keys(KeyAttr::new(KeyAttrFlags::cubic(TangentMode::User)).with_slopes(1.5, -2.5));
        assert!((user.evaluate(0.5) - 5.5).abs() < 1e-9);
        assert!(!user.is_piecewise_linear());
        // Outside the key range the end values hold.
        assert_eq!(user.evaluate(-1.0), 0.0);
        assert_eq!(user.evaluate(9.0), 7.0);
        assert_eq!(AnimCurve::new(Vec::new(), Vec::new()).evaluate(1.0), 0.0);
    }

    #[test]
    fn runs_expand_compact_and_round_trip_through_an_element() {
        let lin = KeyAttr::linear();
        let step = KeyAttr::new(KeyAttrFlags(0x2102));
        let mut curve = AnimCurve::new(
            (0..6).map(|i| secs(f64::from(i))).collect(),
            vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0],
        )
        .with_key_attrs([lin, lin, step, step, step, lin])
        .with_extrapolation(
            Some(Extrapolation {
                kind: ExtrapolationKind::Repetition,
                repetition: 3,
            }),
            None,
        );
        assert!(curve.runs_cover_keys());
        let again = AnimCurve::from_element(&curve.to_element(7)).expect("parse");
        assert_eq!(again, curve);
        curve.compact_runs();
        // R01's second-generation shape: {2, 3, 1}.
        assert_eq!(
            curve.runs.iter().map(|r| r.count).collect::<Vec<_>>(),
            vec![2, 3, 1]
        );
        assert!(curve.runs_cover_keys());
        let compact = AnimCurve::from_element(&curve.to_element(7)).expect("parse");
        assert_eq!(compact, curve);
        curve.expand_runs();
        assert_eq!(curve.runs.len(), 6);
    }

    #[test]
    fn weighted_bezier_solves_for_the_time_parameter() {
        // Abscissa handles at 0.6 and 1 - 0.2: x(s) must invert.
        for u in [0.0, 0.01, 0.25, 0.5, 0.75, 0.99, 1.0] {
            let s = bezier_parameter(u, 0.6, 0.8);
            let r = 1.0 - s;
            let x = 3.0 * r * r * s * 0.6 + 3.0 * r * s * s * 0.8 + s * s * s;
            assert!((x - u).abs() < 1e-12, "u = {u}");
        }
        // Extreme stored weights (1 / 9999 and 9899 / 9999).
        for u in [0.001, 0.5, 0.999] {
            let (x1, x2) = (1.0 / 9999.0, 1.0 - 9899.0 / 9999.0);
            let s = bezier_parameter(u, x1, x2);
            let r = 1.0 - s;
            let x = 3.0 * r * r * s * x1 + 3.0 * r * s * s * x2 + s * s * s;
            assert!((x - u).abs() < 1e-10, "u = {u}");
        }
    }
}
