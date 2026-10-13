# Jetson RL navigation integration — 2026-10-13

Follow-up: the user later powered the Orin. See the
[device check](jetson_nav_device_20261013.md) for the successful engine build,
synthetic timing, prebuilt archive and still-unresolved physical USB link.
The no-device statements below describe the original integration phase.

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
Independent integration verification:

- Original and adapted ONNX passed full graph checks. ONNX Runtime 1.31.0 on
  Windows CPU compared 140 continuous frames plus 8 reset/replay frames through
  the actual client/session/state feedback and Carrot parser. All 18580 driving
  values and all five next states were exactly equal (maximum absolute error 0).
  An injected non-finite map state was rejected and all history cleared. Only
  socket writing was adapted in the private harness because Windows lacks
  `sendmsg`; production framing and state code were unchanged.
- Focused desktop checks: 121 passed, 14 skipped for Linux/root-only facilities.
  Initial broader Windows failures due to absent Unix socket APIs are covered by
  Linux CI, not counted as desktop passes. Two obsolete expectations that Jetson
  still requests Cinque were corrected; explicit new-model tests cover that path.
- Runtime bundle source `955a6f2d39a8cff5a46214113689e6cfba79533e` was built from
  committed sources, signed with the existing release authority and published to
  its immutable NAS directory. HTTPS bundle length/SHA256, manifest signature,
  original ONNX download length and both bootstrap pin filenames were verified.
  The private signing key is not in the bundle. Shared stable was not modified.
- Linux Jetson integration CI now also runs on `carrot-nav` and installs ONNX for
  the new graph/contract tests. Its run result is reported with the publication.

Private reproducible evidence: `.analysis/scratch/2026-10-13-jetson-nav/`,
including `cpu/full-cpu-report.json` and `publication.json`. CPU timing here is
not TensorRT or physical USB timing.

No vehicle installation, Jetson build or road validation has been performed.
The user deferred physical Jetson work until it is connected. Host recurrent
copies and finite checks need actual Orin profiling; do not claim 20 Hz or a
CPU improvement from PC equivalence tests.
