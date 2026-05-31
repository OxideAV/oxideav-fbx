# tests/fixtures/ — sample `.fbx` files used by integration tests

These are **sample data files** (encoded FBX scenes), not source code
of any FBX reader/writer. They are checked into the crate so the
crate-only CI runners (which clone `OxideAV/oxideav-fbx` without the
`OxideAV/docs` clean-room submodule) can compile + run the tests that
exercise the ASCII / binary parser front-ends end-to-end.

| Fixture | Form | Size | SHA-256 |
|---------|------|------|---------|
| `cubes-ascii-v7500.fbx` | ASCII FBX, version 7500 (text, `; FBX 7.5.0 project file` banner) | 88127 B | `1070eab19a0af80f31a18d49e47ee522cce86acd08daf2a80c63cfb615ed4006` |

## Provenance

`cubes-ascii-v7500.fbx` is the **assimp** project's
`test/models/FBX/cubes_with_names.fbx` model-data file (BSD-3-Clause),
distinct from assimp's own C++ FBX *importer* source under
`code/AssetLib/FBX/` (only the data file was consulted — no assimp
implementation source was read). It was exported by *FBX SDK/FBX
Plugins version 2018.1.1* from Maya (SceneInfo
`Original|ApplicationName: "Maya"`); four cube meshes, two materials,
one anim take. Useful because object nodes carry real names
(`Model::Cube2`, `Material::Mat_Green`, including a Cyrillic name
`Куб1` that exercises UTF-8 in names).

The same byte-identical fixture lives in
`docs/3d/fbx/fixtures/cubes-ascii-v7500.fbx` (the docs submodule);
this copy is for in-crate CI consumption only.

## Method note

RE'd from sample bytes/text only — no FBX-implementation source was
read. The clean-room grammar handoff used to build the ASCII parser
lives in `docs/3d/fbx/fbx-ascii-grammar.md`.
