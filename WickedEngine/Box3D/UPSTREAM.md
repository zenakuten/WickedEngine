# Vendored Box3D core

Source: https://github.com/erincatto/box3d

Revision: `a2e4816b4359b32408d5623784a7fe2d7259d3d6`

License: MIT; see `LICENSE`.

This copy contains the upstream core (`src`, `include`, root CMake file and
license), not the standalone samples, tests or documentation. WickedEngine
embeds it as a subdirectory, so standalone-only targets are not enabled.

Local change: apply the upstream static MSVC runtime default only to standalone
builds. Embedded builds inherit the engine's runtime selection.
