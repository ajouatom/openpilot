# Signal perception audio experiment (carrot-egpu-yolo2)

This optional experiment replaces the resident COCO observation model with
`signal-v33-observe-s260911`, a two-class 512x256 YOLO11n lamp detector. It publishes
`red_visible` and `green_visible` observations on `carrotYolo`. No control process,
navigation signal, traffic-stop state, or actuator command is changed.

The model remains experimental. The examined partial-label sets contain misses
and false detections; local regression acceptance was not achieved. Vehicle
observation is explicitly requested to collect further evidence. The two classes
do not identify the ego lane, turning direction, pedestrian relevance, or permission
to proceed.

## Perception tones

After exclusive parked compilation and input/output validation, an installer may
create `/data/egpu_yolo/signal_observation.json` with:

```json
{"enabled": true, "model_id": "signal-v33-observe-s260911"}
```

Only fresh, valid road-camera results with that exact model ID can produce a tone.
At least three distinct frames spanning 300 ms must agree on one visible color.
Mixed red/green observations, stale data, wide-camera input, paused inference, and
missing enablement produce no tone. Red uses a short lower-pitched tone; green a
short higher-pitched tone. These mean only that a lamp color was observed. They
are not stop/go instructions. A color is announced once, with a three-second
minimum interval; two seconds without a valid color permit a new observation.

Existing alerts always take priority and immediately discard an observation tone.
Remove the marker or set `enabled` to `false` to disable perception audio. It is an
internal experimental opt-in, not a public settings-catalog entry.

## Preparation and rollback

Keep this model in its own NAS model directory and compile it into a separate
vehicle staging directory. Verify the source SHA256, native input rebinding,
serialized output, eGPU timing budget, and restored primary model before enabling
observation. Preserve the prior resident bundle and its enablement files before
activation; rollback restores those files with no driving-model weight change.

The PC export and tinygrad checks establish numerical compatibility, not vehicle
eGPU timing or successful live installation. Vehicle commissioning must record its
actual model ID, camera, fresh messages, and timing. The Linux soundd integration
also requires live verification; Windows lacks its msgq service transport.
