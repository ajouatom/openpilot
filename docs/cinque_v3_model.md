# Cinque v3 experiment

## Integration on 2026-09-19

The user requested that `carrot-wip` incorporate this experiment in full,
including its pinned Cinque v3 runtime, AGNOS 19.8 Bluetooth image and remote
controls. After validation, the user retired `carrot-cinque_v3`; `carrot-wip`
now maintains this same eGPU artifact and the complete merged history. The
internal-GPU driving model stays unchanged. Driver monitoring separately moves
to Super Leicht; see [the integration review](cinque_v3_integration_20260919.md).
The original experiment and its validation history follow below.

`carrot-cinque_v3` was created on 2026-09-17 from `carrot-wip`
`91214ef52a`, plus the shared maintenance-scope commit `286ee33a32`.
The user requests that future common `carrot-wip` changes also reach this branch,
while its v3 model selection and runtime compatibility changes remain separate.
`carrot-wip` continues to select Cinque v2. The internal-GPU fallback model is unchanged.

## Upstream comparison

At inspection, comma master was `6080cc6168023229437b0ff06cf35bead00a5d0d`
([#38930](https://github.com/commaai/openpilot/pull/38930)). Its current eGPU
artifact is a precompiled version of Cinque v2. The earlier stock Cinque v2
merge, [#38850](https://github.com/commaai/openpilot/pull/38850), selects the exact
ONNX SHA-256 already pinned by `carrot-wip`: `09d080f36965bb2a0790500452bd328aa03c484d0222aa79d1ad9f021a522aec`.
Stock has since moved history queues into the graph and changed the compilation
format; identical trained-model lineage does not mean identical artifact bytes.

[Cinque v3 PR #38932](https://github.com/commaai/openpilot/pull/38932) was open,
not merged, and replaces only the precompiled eGPU artifact. This branch pins
its inspected head rather than following later force-pushes.

- Source commit: `892fc3a1256d8f083333e92b254d803b01f08d27`
- Checkpoint: `b9facbcc-4d47-410e-b3ce-dfcbad12ba92/56320/f78ed37d-afad-4dbc-8050-40ea885eedde/12864`
- Model ID: `comma-pr38932-cinque-v3-892fc3a1-e758b96d`
- File: `big_driving_tinygrad.pkl`, 776,634,338 bytes
- SHA-256: `e758b96df27858ea97122d18554930d04f9f8bda417417074edfb3a72b008d0b`
- Matching tinygrad commit: `d5e17c935daf11f6318e45aade9528f71b8fbdcc`
- Runtime archive: 2,731,184 bytes, SHA-256 `a25b81e90d5259c27bc49b3a93ab499c1909a33e133cba974cf023cd94727eeb`
- NAS directory: `\\DS1821P\openpilot\models\comma4-big-cinque-v3`
- Vehicle manifest: <https://upload.shind0.synology.me/models/comma4-big-cinque-v3/manifest.json>
- Runtime catalog: <https://upload.shind0.synology.me/models/comma4-big-cinque-v3/precompiled.json>

Upstream publishes no v3 ONNX in this PR. The NAS therefore hosts its verified
official PKL and matching runtime instead of relabeling the old ONNX as v3.
Vehicles obtain both files from NAS HTTPS, not GitHub LFS.

## Runtime integration

The generic artifact has `run`, `input_specs`, `output_specs`, and nested metadata.
It takes two uint8 images of shape `(2, 6, 128, 256)`, desire, traffic convention,
action timing, and three recurrent state queues. It returns 18,452 model values
plus the next image, desire, and feature queues. The worker aliases next-state
outputs to current-state buffers as upstream does. The parent does not feed an
additional feature history into this format, including during dropped-frame catch-up.

The runtime archive contains the pinned upstream `tinygrad` and
`examples/openpilot` sources and their license. Its camera warp is compiled and
cached separately for 1928x1208 and 1344x760 cameras using upstream's compiler.
The large driving model is already compiled. Worker startup permits 110 seconds,
within the model loader's 120-second budget; boot smoke validation permits 300
seconds for both camera formats. This does not change the per-frame timeout.

The selected model hash isolates all caches. A verified downloaded PKL is reused
when installing the runtime. A missing or rejected precompiled v3 artifact uses
the internal model; it is never sent to the ONNX compiler or silently replaced by
an older compiled big model. Explicit ONNX overrides retain their existing compiler path.

## Validation on 2026-09-17

- Official LFS size/SHA-256 and all 585 out-of-band buffer boundaries verified.
- Metadata fixture extracted without executing pickle globals; the existing output parser accepts the output contract.
- NAS model and runtime full hashes match their catalogs.
- The vehicle downloader fetched the complete model over public HTTPS; runtime download,
  extraction, cache reuse and selected-model matching passed.
- 83 focused Python tests and seven web model-name tests passed. Web assets were rebuilt.
- Changed Python files compile; new adapter and focused tests pass lint. Existing
  unrelated lint findings in the downloader/build files were preserved.

A subsequent C4 trial on AGNOS 19.6.3-carrot confirmed that the pinned artifact
loads and produces live camera inference through its matching runtime. A passive
10-second stationary observation received 200 consecutive model frames with
finite position/velocity values and median execution time 39.56 ms. This does
not validate driving behavior. The ongoing OS, Bluetooth and tinygrad review
is recorded in [the AGNOS trial](agnos_19_8_bluetooth_trial.md).

Docs-Not-Needed: This experiment changes a pinned model and its private runtime,
not a user setting. Its web-only model label is implemented in the localized UI;
no public settings guide or Wiki behavior changed.
