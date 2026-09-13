# Cinque v2 model integration

On 2026-09-13, `carrot-wip` incorporates the complete `carrot-cinque_v2` history
through `6756edb408e8a2343a1e37ec8a30c73f3a8c8480` and adopts its Cinque v2 eGPU
big model in place of TGC. `carrot-wip` is now the sole maintained top-level
`carrot-*` branch. This integration does not include the separate YOLO experiments.

The original model branch started from `carrot-wip` commit
`dbbf4969de64f271df5b40b71a31940e7704e896`. Retired branch tips are preserved under
`archive/2026-09-13/<branch>`. Existing vehicles on a retired branch must select
`carrot-wip`; deleting a remote branch does not automatically migrate them.

- Source: [commaai/openpilot#38823](https://github.com/commaai/openpilot/pull/38823)
- Source commit: `37bfa1413edcdc2e8844984b83727c33f81d8f46`
- Model ID: `comma-pr38823-cinque-v2-37bfa141-09d080f3`
- File: `big_driving_supercombo.onnx`, 766,040,736 bytes
- SHA-256: `09d080f36965bb2a0790500452bd328aa03c484d0222aa79d1ad9f021a522aec`
- Manifest: <https://upload.shind0.synology.me/models/comma4-big-cinque-v2/manifest.json>
- NAS directory: `\\DS1821P\openpilot\models\comma4-big-cinque-v2`

This integration replaces the eGPU big model. The internal fallback driving model stays
unchanged. The existing downloader verifies the pinned size and SHA-256, and the
compiled artifact name includes the model hash so an older model's compiled
artifact is not selected for Cinque v2.

Validation on 2026-09-09 confirmed the ONNX checker passes; input/output shapes,
dtypes, output slices, initializer layouts, graph connectivity, and operator
attributes match Cinque Terre. Node labels and trained weights differ. All 25
operator/opset combinations are supported by the checked-in tinygrad. The actual
tinygrad metadata reader and model output parser accept this model. No additional
runtime compatibility change was needed.

The NAS copy and the HTTPS manifest were verified, and the vehicle downloader
downloaded the complete model over HTTPS with the expected hash. Model downloader,
cache/helper, and model-name display tests pass. Device-side eGPU compilation,
inference timing, and on-road behavior have not been tested on this workstation.

The 2026-09-13 consolidation passed all 60 model downloader, cache/helper and
precompiled-artifact tests, plus all six web model-name tests. Rebuilding the web
assets reproduced the committed Cinque v2 assets. This consolidation does not
constitute new device-side compilation or on-road validation.

The same consolidation verified the complete SHA-256 and size of the NAS ONNX,
precompiled pickle and runtime archive. The NAS and public HTTPS model and
precompiled catalogs matched, and the precompiled catalog passed validation
against the selected ONNX hash.

Docs-Not-Needed: The web change only labels the existing eGPU model status as
Cinque v2; it adds no setting or interaction. Web-only model presentation belongs
in the localized UI rather than the public user guides. The pinned model and
branch transition are documented here.
