# Cinque v2 model integration

`carrot-cinque_v2` starts from `carrot-wip` commit
`dbbf4969de64f271df5b40b71a31940e7704e896`. The same driving-model selection is
also applied to `carrot-egpu-yolo2`, preserving its dedicated YOLO2 implementation.
Other branches keep their existing models.

- Source: [commaai/openpilot#38823](https://github.com/commaai/openpilot/pull/38823)
- Source commit: `37bfa1413edcdc2e8844984b83727c33f81d8f46`
- Model ID: `comma-pr38823-cinque-v2-37bfa141-09d080f3`
- File: `big_driving_supercombo.onnx`, 766,040,736 bytes
- SHA-256: `09d080f36965bb2a0790500452bd328aa03c484d0222aa79d1ad9f021a522aec`
- Manifest: <https://upload.shind0.synology.me/models/comma4-big-cinque-v2/manifest.json>
- NAS directory: `\\DS1821P\openpilot\models\comma4-big-cinque-v2`

This PR replaces the eGPU big model. The internal fallback driving model stays
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
