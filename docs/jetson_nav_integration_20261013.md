# Jetson RL navigation integration — 2026-10-13

User-requested source: https://github.com/tjdgus200/openpilot/tree/codex/jetson-nav-rl
at `38b06d6a481ad15e7a999abc53c85eb1e966743f`. Scope is **carrot-nav only**.
All four prerequisite/runtime/model commits were imported as single-parent
commits, preserving unrelated local work and existing eGPU/internal/DM choices.

The Jetson pin now selects the same `80eedcdd/12864` RL navigation checkpoint
as eGPU, using original ONNX SHA256
`1d18c42ea4cd95c8bffab4635844b78b8e2d4e25b9de44a9c5ac2e0961371a2e`
(814591106 bytes). Map pixels remain 0..255; the model normalizes them itself.
The 47-element maneuver vector and map are cleared on missing/expired inputs.
Five recurrent states remain on the Jetson host. Existing join, validity and
fallback policies are retained.

Unlike the contributor's manual local registration, this branch uses the
existing signed NAS update path. The exported legacy specification alias lets
installed bootstraps verify the new ONNX before importing the new updater.
Runtime and model are hash-pinned; failed device builds/probes withhold readiness.
The shared stable manifest and other maintained branches are untouched.

The contributor's CPU evidence is retained with provenance in
`tools/jetlink/validation/nav_rl_pc.json`; it is not a local Orin result.
Integration verification results are recorded below after package validation.

No vehicle installation, Jetson build or road validation has been performed.
The user deferred physical Jetson work until it is connected. Host recurrent
copies and finite checks need actual Orin profiling; do not claim 20 Hz or a
CPU improvement from PC equivalence tests.
