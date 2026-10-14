# Ioniq 5 PE 107c/107e: internal-model signal investigation (2026-10-09)

## Scope and model identity

The user supplied three new segments with both original road-camera HEVC streams
and full rlog, and explicitly selected the small internal model for experiments.
The objective is to understand and eventually improve the model's trajectory
predictions, not to modify Carrot's signal-start thresholds.

- `0000107c--1ea0f38099--0`
- `0000107e--96872bd188--1`
- `0000107e--96872bd188--2`
- Capture commit: `6eeb19999fe6a5204865d4a712784688b3d60763`.
- Actual 107e runtime diagnostics: `backend=JoiningModel, usbgpu=False`
  (58 and 42 reports respectively). These are internal-model recordings.
- Source: `openpilot/selfdrive/modeld/models/driving_supercombo.onnx`,
  60,792,584 bytes, SHA256
  `f73a9e535523d5e9acb9e642c64e33d631825dc8ba74123757d107cedd047bb5`.
- Checkpoint metadata:
  `6a7d09ad-bcc9-43bc-916d-29287e60cee2/200+a27b3122-733e-4a65-938b-acfebebbe5e8/100`.
- Model, warp source and compile source have no diff against the capture commit.

No production model, control threshold, startup behavior or vehicle setting was
changed. All experiments are offline. Private source logs, images, scripts and
numerical results are under `.analysis/archive/2026-10-09-107ce-signal/`.

## What the recordings show

### 107c--0: startup failure is a separate issue

The eGPU was detected but failed to load after a PCIe/USB transfer error and a
retry. The internal model loaded in 1.709 seconds; total model loading took
49.5 seconds. The first model publication in the segment is approximately
55.31 seconds after the first carState. Only 92 model/planner messages exist.
The early intersection wait cannot be attributed to an incorrect model output:
there was no published trajectory during that part of the recording.

### 107e--1: red signal, unstable continuation forecasts

The user confirmed that the school-zone signal was an ordinary red stop signal,
not a flashing amber signal. Original video shows it lit while the predicted
longitudinal trajectory alternates between stopping and continuing.

- Around 15.67 seconds, the planner's derived signal state changes to green.
- Further stop/continue transitions occur around 17.81 and 18.51 seconds.
- Recorded mean `v(+10 s)` is 6.90 m/s in the 15–18 s window and 6.09 m/s
  in the 18–21 s window.

These values are future trajectory predictions, not a directly exposed
red/green probability. They do not establish what class the network internally
assigns to the light. The planner derives its signal state from this trajectory.

### 107e--2: delayed response after green

Sequential decoding of the original HEVC establishes red-to-green transition
between approximately video frames 110 and 115 (5.50–5.75 seconds at 20 Hz).
The video-frame clock and rlog-relative clock differ by roughly 0.08 seconds;
the chart explicitly uses frame index / 20 rather than implying exact equality.

- Recorded mean `v(+10 s)` remains 0.69 m/s over rlog-relative 5.7–8.5 s.
- At approximately 8.0 s, the recorded prediction still ends at only about
  7.9 m forward, with `v(+10 s)=0.42 m/s`.
- The driver presses the accelerator at 8.83 s, near 10.6 km/h.
- The planner's derived signal state becomes green at 9.37 s.

The late rise in the trajectory is already present in the raw model output;
this is not solely a signal-start threshold delay. Accelerator timing does not
prove that the pedal caused the network's response: pedal state is not a direct
input to this ONNX, while subsequent vehicle motion changes its camera input.

## Replay method and validation

The 107e segments were replayed continuously using both cameras, keeping temporal
history across the segment boundary. HEVC was decoded sequentially with FFmpeg
to planar YUV420. Camera frames were associated through road/wide EncodeIdx and
model frame IDs. Calibration, OS04C10 intrinsics, nearest-neighbor clipping and
six-channel YUV packing follow the repository's model warp implementation.

The small model receives two images 0.2 seconds apart, 24 previous 512-element
features sampled at 5 Hz, and 25 desire entries. Replay advances queues at
20 Hz. The matching ORT CUDA session executes the original ONNX without changing
its graph or weights. Traffic convention is right-hand traffic. Experiment
windows have no lane-change desire. Action-time inputs are approximated as
`[0.475, 0.375]`; lateral dynamic smoothing was not reconstructed exactly.

After excluding the first six seconds of each segment for the reported metric:

| Segment | Mean absolute x(+10 s) error | Mean absolute v(+10 s) error | Mean absolute v(0) error |
|---|---:|---:|---:|
| 107e--1 | 1.98 m | 0.340 m/s | 0.083 m/s |
| 107e--2 | 2.29 m | 0.401 m/s | 0.103 m/s |

The relevant stop/continue patterns reproduce. This is approximate replay, not
bit-exact device equivalence. HEVC compression, runtime arithmetic, calibration
timing, action-time approximation and initially unavailable previous-segment
context remain sources of discrepancy.

The narrow model view crops out the overhead signal in the examined frames;
the wide model view carries the signal, with individual distant lamps often
represented by only a few pixels. Full-resolution human visibility therefore
does not imply equally clear model input.

## Controlled signal experiments

Each run forks identical baseline queues before the selected window and then
advances its own history. It changes only the wide-camera input. Baseline,
RGB/YUV roundtrip, lamp removal, matched nearby removals, green/red color changes,
red chroma-only and red luminance-only variants were compared.

For 107e--1, a small moving ROI follows the lit signal; for 107e--2, a tight band
covers the two distant signal heads. The chroma-only experiment preserves all
four packed Y channels byte-for-byte and substitutes only U/V in the ROI.
HSV hue is changed within the colored/bright mask; existing saturation and value
are retained before conversion back to YUV. It is an artificial intervention,
not a recommended production color correction.

Exploratory `ablation.csv` used broader ROIs that included part of a yellow road
sign in 107e--1. Its larger effects are confounded and **must not be cited as
lamp-only evidence**. The corrected `lamps.csv` results below use smaller ROIs.

Mean replay `v(+10 s)`, in m/s:

| Input variant | 107e--1, 15–18 s | 107e--1, 18–21 s | 107e--2, 5.5–8.5 s | 107e--2, 8.5–11 s |
|---|---:|---:|---:|---:|
| Original | 7.473 | 5.882 | 0.753 | 10.469 |
| ROI RGB/YUV roundtrip only | 7.581 | 5.998 | 0.753 | 10.466 |
| Lamp region removed | 8.177 | 6.394 | 1.375 | 10.655 |
| Left control region removed | 7.478 | 5.883 | 0.759 | 10.457 |
| Right control region removed | 7.476 | 5.880 | 0.765 | 10.477 |
| Signal made green | 8.013 | 7.067 | 0.800 | 10.463 |
| Signal made red, chroma only | 5.467 | 3.068 | 0.036 | 6.667 |
| Same red edit, luminance only | 7.505 | 5.914 | 0.782 | 10.490 |

In 107e--1 at 18–21 s, the chroma-only red intervention changes mean `x(+10 s)`
from 46.86 to 34.09 m, and mean `v(+2 s)` from 5.79 to 4.95 m/s. Thus the effect
is not confined to a displayed signal state or a single far-horizon scalar.

The model does respond to the signal region and its color. The captured red
lamp appears partly orange/white after imaging and reduction. Making its
chromatic component unambiguously red strengthens the stopping prediction,
while nearby controls and luminance-only edits have much smaller effects.
This supports inadequate robustness to this signal appearance as a contributor.
It does not prove that camera exposure is wrong, identify the original training
label distribution, or show that color is the only cause. Red recoloring still
does not consistently recover a complete stop.

For 107e--2, changing green to red suppresses the later continuation forecast,
confirming signal sensitivity. Making the existing signal green does not remove
the early delayed response. Removing it has only a modest early effect. The
cause of the green-response delay is therefore not resolved by a simple color
ambiguity explanation. Small image footprint, temporal features and scene
context remain hypotheses requiring additional controlled experiments.

## Can the model itself be changed?

Yes. The local ONNX contains roughly 30 million initializer elements and an
explicit longitudinal trajectory head. The final plan layer is
`policy/policy_model.temporal_hydra.final_layer.plan.weight`, shape `[990,256]`,
plus 990 biases and a scale layer. Plan means occupy the first 495 outputs
(33 time points × 15 fields), followed by uncertainty outputs.

There is a concrete narrow training experiment available: retain the backbone
and train the final mean-output rows for longitudinal position, speed and
acceleration (99 rows, 25,443 weights/biases). Keep the remaining rows fixed and
use original-model distillation on normal driving examples, plus consistent
stop/continue target trajectories for signal cases. This is a feasibility path,
not a claim that those rows alone contain all necessary signal information.
If the visual features cannot distinguish the difficult lamps, visual/temporal
layers or an explicit signal-training objective would also need work.

The data must include normal red, green, amber and flashing signals, small and
bright lamps, intersections, occlusions and ordinary no-signal driving. These
three clips alone cannot establish a broadly correct replacement model.
Longitudinal targets must remain kinematically consistent and compatible with
the visible road and obstacles. In this internal model, acceleration is derived
from the plan, so changing the longitudinal head also changes immediate driving
behavior; it is not merely an independent classification adjustment.

No training or replacement artifact was produced in this investigation. The
completed deliverable is a reproducible internal-model baseline and controlled
input experiments identifying one model-side weakness and one unresolved delay.

## Reproduction artifacts

Private archive index: `.analysis/archive/2026-10-09-107ce-signal/INDEX.md`.
Key results: `baseline.csv`, `lamps.csv`, `signal_experiment.png`, `results.txt`,
`lamps.txt`. Scripts: `decode.py`, `overview.py`, `replay.py`, `ablate_lamps.py`,
`report_results.py`. Rebuild temporary dependencies and warped caches as documented
in the archive index; keep all raw captures and generated camera imagery local.
