# tests/fixtures/ — sample `.fbx` files used by integration tests

These are **sample data files** (encoded FBX scenes), not source code
of any FBX reader/writer. They are checked into the crate so the
crate-only CI runners (which clone `OxideAV/oxideav-fbx` without the
`OxideAV/docs` clean-room submodule) can compile + run the tests that
exercise the ASCII / binary parser front-ends end-to-end.

| Fixture | Form | Size | SHA-256 |
|---------|------|------|---------|
| `cubes-ascii-v7500.fbx` | ASCII FBX, version 7500 (text, `; FBX 7.5.0 project file` banner) | 88127 B | `1070eab19a0af80f31a18d49e47ee522cce86acd08daf2a80c63cfb615ed4006` |
| `box-binary-v7400.fbx` | Binary FBX, version 7400 (Kaydara magic, 32-bit Node Record layout) | 17200 B | `ad2d79fe89d4d646bc7930dc952eb28e69976a321b387bf7851ecd3f37e667f8` |

## Provenance

`box-binary-v7400.fbx` is the **assimp** project's
`test/models/FBX/box.fbx` model-data file (BSD-3-Clause), exported by
*FBX SDK/FBX Plugins version 2017.1* — the primary sample the
clean-room observer trace `docs/3d/fbx/fbx-binary-properties70.md`
was derived from; the fixture bytes are treated as opaque sample
data. One cube Geometry + Model + Material;
every array `Encoding == 0`. It is the byte-faithful round-trip
reference: the whole 17200-byte file (including the 176-byte tail
region past the record-tree walk: top-level NULL record + footer)
re-encodes byte-for-byte through `parse` + `parse_footer` +
`write_document_with_options`.

`cubes-ascii-v7500.fbx` is the **assimp** project's
`test/models/FBX/cubes_with_names.fbx` model-data file (BSD-3-Clause);
the fixture bytes are treated as opaque sample data. It was
exported by *FBX SDK/FBX
Plugins version 2018.1.1* from Maya (SceneInfo
`Original|ApplicationName: "Maya"`); four cube meshes, two materials,
one anim take. Useful because object nodes carry real names
(`Model::Cube2`, `Material::Mat_Green`, including a Cyrillic name
`Куб1` that exercises UTF-8 in names).

The same byte-identical fixtures live in
`docs/3d/fbx/fixtures/` (the docs submodule);
these copies are for in-crate CI consumption only.

## Method note

RE'd from sample bytes/text only — no FBX-implementation source was
read. The clean-room grammar handoff used to build the ASCII parser
lives in `docs/3d/fbx/fbx-ascii-grammar.md`.
