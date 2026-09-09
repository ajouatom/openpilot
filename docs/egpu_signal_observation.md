# Signal perception audio experiment (carrot-egpu-yolo2)

The current trial uses `signal-v33-observe-int8-s260911`, a 512x256 two-class
YOLO11n detector with a TRAIN-calibrated INT8 backbone and FP32 detection head.
An ONNX Runtime CPU worker on cores 4 and 5 observes the calibrated road crop
at up to three frames per second. It never opens the driving eGPU or QCOM queue.
The original COCO observer and this signal observer cannot run together.

The model remains experimental. The examined partial-label sets contain misses
and false detections; independent deployment acceptance was not achieved.
Quantization retained the examined FP32 counts on one set and improved one
localization on the other; this is compatibility evidence, not a new accuracy
claim. Classes do not identify the ego lane, turning direction, pedestrian
relevance, or permission to proceed. No control, navigation traffic-stop state,
or actuator command consumes this experiment.

## Enablement and display

A verified installer can create `/data/egpu_yolo/signal_cpu.json`:

```json
{"enabled": true, "model_id": "signal-v33-observe-int8-s260911"}
```

The worker requires the pinned model under `signal-v36-cpu-int8/model.onnx` and
its isolated ONNX Runtime installation. It refuses enablement if the resident
GPU observer is enabled or a QCOM observer marker exists. The model is hosted
in its own NAS directory, `models/carrot-signal-v33-int8-observe`.

The YOLO2 screen summarizes red/green observations. CPU summary text can remain
for at most 600 ms of capture age to bridge 3 Hz results. Camera boxes retain the
separate 200 ms frame-alignment check, so a summary may appear without a box.
They are observations of lamp colors, not stop/go instructions.

For sound, `/data/egpu_yolo/signal_observation.json` must explicitly enable the
same model ID. Red has a short lower-pitched tone; green a short higher-pitched
tone. Three distinct fresh frames must agree, with at least 300 ms between the
first and last. CPU candidate history can bridge up to 400 ms between received
frames, but each counted frame still requires capture and transport age at most
350 ms. Stale data never emits a tone. Mixed colors or invalid context reset
agreement. Repeated colors are suppressed, with at least three seconds between
tones; two seconds without valid color permits another observation. Existing
warning audio always takes priority and discards the observation tone.

## Validation and rollback

The FP32 eGPU attempt passed isolated 1,000-run timing and numerical tests, but
shared live execution still produced deadline overruns, including after a 5 Hz
limit. Its automatic execution and tones were disabled. Keep those failure
records; do not claim the isolated benchmark established stable live operation.

The CPU path must be checked on actual road frames after installation, including
capture age, primary model health, process ownership, and its rate. Existing
primary timing/drop checks can pause CPU observations too. CPU benchmark timing
alone does not establish live display/audio readiness.

Preserve the former resident bundle and enablement files before installing.
Disabling `signal_cpu.json` stops the CPU observer; disabling or removing
`signal_observation.json` stops tones. A guarded parked rollback restores the
prior resident bundle and markers. These are internal trial markers, not public
settings-catalog entries. The earlier FP32 model ID remains supported by the
sound filter for reversible experiments, but the marker must match its exact ID.
