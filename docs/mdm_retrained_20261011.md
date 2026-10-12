# Retrained Mountain Dew eGPU selection, October 11, 2026

## Current status, October 12

The user subsequently requested full integration into carrot-wip. Single-parent
commit e4c9730510 includes this model selection, delivery and runtime compatibility
work; 2b6229340b simplifies the Web display name to `Mountain Dew`. The remote
carrot-signal-shadow branch was deleted at the user's request after verifying
integration through 5ce8faa735. Continue on carrot-wip only, with single-parent
commits; do not recreate the retired remote branch. The model artifacts remain
at the NAS URLs below. Internal/DM and Jetson model selections are unchanged.

The following sections record the original trial and its validation limits.

## Original trial scope

The user selected upstream [6f4c0faf](https://github.com/commaai/openpilot/commit/6f4c0faf1f85132734e94aa39c67143f823b9324)
for carrot-signal-shadow only, after redirecting the trial from carrot-wip.
The brief carrot-wip publication was reverted by b09ae3ddad; carrot-wip keeps
Cinque v3 until the user approves a combined promotion after testing. This is the
**8d736683** retraining of Mountain Dew, not the earlier 870a4823 export and not
an upstream model named MDM v2. Web identifies it as `Mountain Dew (8d736683)`.

## Artifact identity

- AMD PKL: 799,942,179 bytes, SHA256
  `256f2178366479a211933701c03518621ed3970289e6a8a43e5a2bcb15e55b5f`.
- Checkpoint extracted with inert pickle globals:
  `f22e06e5-3b3f-48d2-a6b9-2f32dc20e707/56320/8d736683-fed4-4e48-96fd-eae5d9d5bc56/12864`.
- tinygrad at the selected upstream commit:
  `d3f09c9bbd542fbfbe68c8569a1173550c408969`.
- Deterministic isolated runtime: 2,739,152 bytes, SHA256
  `90970e01f2ba70d553294c0f9d5c6fe1678827f750705cab679a7b19b7bcb0b2`.
- NAS directory: `comma4-big-mdm-8d736683`; vehicle manifest:
  `https://upload.shind0.synology.me/models/comma4-big-mdm-8d736683/manifest.json`.

The complete official PKL was downloaded from the upstream Hugging Face LFS
endpoint and verified against the committed pointer. The package builder pins
both that artifact and the tinygrad source archive hash. NAS publication used a
new directory with verified temporary files and published the manifest last.
No LFS dependency or model blob is added to this repository.

## First installation and runtime compatibility

This trial preserves the existing carrot-signal-shadow contents; no wholesale
carrot-wip merge is included. The new runtime changes `Buffer.view(element_count, dtype, byte_offset)` to
`Buffer.view(byte_count, byte_offset)` and requires an explicit dtype in
`UOp.from_buffer`. The existing adapter would fail during recurrent-state/input
binding despite a successful download. `input_view` now supports both pinned
APIs, retaining byte offsets, float element widths, packed camera inputs and
in-place recurrent-state aliases. C3 QCOM pre-upload warp and C4 AMD warp keep
their existing selection and pixel-verification policies.

The existing startup delivery gate is retained: with eGPU attached, install the
selected model and its isolated runtime before build validation and manager.
Network interruption resumes and retries; an old cached external model cannot
substitute for the selected model. Permanent preparation failure selects the
internal model for that boot. Installed bytes do not constitute GPU validation;
existing GPU initialization, boot-validation and inference checks still apply.

Tests now explicitly start without model state or cached files, interrupt either
the source or runtime download, verify resume, then reject network access during
cached reuse. Linux compatibility CI uses the newly pinned runtime and includes
startup ordering/build-cache tests. Its native runtime tests exercise packed
input views, recurrent-state aliasing, warp capture/reload and retargetable
persistent-buffer loading after stripping the export host's CPU dispatch binary.

## Jetson and ONNX search

The official [ONNX export catalog](https://huggingface.co/commaai/openpilot_driving_models/tree/main)
was queried again on October 11. It now contains **870a4823/12864**, in addition
to 1a421175 and f78ed37d. The earlier October 8 observation that 870a4823 was
absent is no longer current. That ONNX is 792,487,890 bytes, SHA256
`c392ba1aa1b4b89e850366024fa6c1beac1f1fdc3a7182e3c3b0234c55e08eeb`.

There is no **8d736683** export in that catalog. The older ONNX does not contain
the selected retrained weights. Jetson therefore retains its existing Cinque v2
contract/runtime; no host image or release changes are included. Internal and
driver-monitoring models, control gates and validity limits remain unchanged.

## Verification and limits

- 250 desktop model, delivery, startup, parser, warp-policy and cache tests pass.
- Real packed-buffer/alias tests pass on both the new d3f09c9 and prior 9d0446a4
  isolated runtimes. The repository's older bundled tinygrad is not the eGPU
  runtime and is deliberately not used to validate the new package.
- 13 Web tests pass; generated tools bundle and its asset hash are updated.
- Empty-cache installation through the actual vehicle HTTPS URLs succeeds;
  complete model/runtime hashes match and subsequent reuse requires no network.
- Ruff passes. Local full native warp testing requires clang, unavailable on
  this Windows workstation; the Linux CI job performs that check.

These checks do not execute the trained model on a physical AMD eGPU or validate
vehicle first boot, driving behavior or C3/C4 timing. No vehicle was updated or
rebooted. Reproduction scripts and download evidence are retained privately in
`.analysis/archive/2026-10-11-mdm-retrained/`.

Docs-Not-Needed: Model selection, runtime compatibility and first-install checks;
no user setting or user-guide workflow changes.
