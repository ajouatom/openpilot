# Ioniq 5 PE 1079: red-signal model forecasts and model-side correction feasibility

## Scope and result

The user requested examination of segments 1–4 of `00001079--0d42e664fc`
on Ioniq 5 PE `07b62e389ed26c81`, and clarified that the intended correction
is to the model's output, **not** Carrot's signal-start conditions.
No driving code, start thresholds, deployed model, or model weights were changed.

The recorded model repeatedly predicts a departure trajectory while the signal
visible in the uploaded video remains red. This is present in the raw published
`modelV2.position/velocity` predictions, before Carrot's traffic-state conversion.
An actual unintended departure is recorded near the start of segment 3.

The model's precise visual cause is not established. The records establish the
incorrect trajectory and its timing, but do not distinguish inadequate signal
representation, reliance on surrounding vehicle motion, or temporal-history
effects. A graph inspection cannot assign this semantic cause to a weight.

The exact checkpoint's official ONNX is now available and was downloaded and
inspected. The `plan` head can be isolated and adjusted through additional
training. The earlier claim that this checkpoint had no public ONNX was based
on the October 8 catalog and is **outdated as of this investigation**.

## Evidence and alignment

- Vehicle code: `fc843551a776b7343bbd18f9ee11a1ec33a69a55`, `carrot-wip`, C4/mici.
- Full-schema decoding: `openpilot.cereal.log.Event.read_multiple_bytes` after
  Zstandard decompression. All four rlogs were read completely.
- Published model counts: 1,199 / 1,200 / 1,200 / 1,124; all carry `valid=true`.
  Valid publication does not certify correct perception or prediction.
- Uploaded video frames: 1,200 / 1,200 / 1,200 / 1,125. Video frames were mapped
  to `qRoadEncodeIdx` by sequential decode and index, using `timestampEof`.
- Times below are relative to each segment's first `carState`, not `initData`.
- Model predictions were associated with plans by `modelMonoTime`, with latest
  available service values used for contextual car/control state.
- External `carrotMan.trafficState` is zero throughout these four segments.
- Same-day settings snapshots consistently have `TrafficLightDetectMode=2`,
  `VEgoStopping=10`, `LongActuatorDelay=30`, and `UseWideCamera=True`.
  Live state and recorded predictions take precedence over snapshot inference.
- The four upload folders contain `rlog.zst` and `qcamera.ts`; no original
  `fcamera.hevc` or `ecamera.hevc` is included in those folders. qcamera is a
  compressed 526×330 road-camera view, not the original two-camera model input.

## Model identity

Do not attribute this incident to the current Cinque v3 selection.
The worker diagnostics explicitly identify SHA256
`e20cde17a9b0a397524c5f0745c5ac47393fd9db40d10f3ea928e6c1fafef4b7`
in all four segments, with 59 / 58 / 58 / 54 reports respectively.
This is Mountain Dew checkpoint:

`f22e06e5-3b3f-48d2-a6b9-2f32dc20e707/56320/870a4823-d2ec-4b9a-8015-b947e9901fc7/12864`.

The official [870a4823 ONNX](https://huggingface.co/commaai/openpilot_driving_models/blob/main/870a4823-d2ec-4b9a-8015-b947e9901fc7/12864/big_driving_supercombo.onnx)
download has 792,487,890 bytes and SHA256
`c392ba1aa1b4b89e850366024fa6c1beac1f1fdc3a7182e3c3b0234c55e08eeb`.
Its embedded full checkpoint string exactly matches the recorded model's
deployment manifest. Matching checkpoint metadata establishes the intended
weights/version; ONNX versus compiled-AMD numerical equivalence was not tested.

Before obtaining that file, the existing NAS Cinque v2 ONNX was also inspected.
Conclusions below about the incident's architecture use **870a4823 itself**, not
an assumption that the v2 graph is identical.

## What happened in the four segments

| Segment | Observed behavior |
| --- | --- |
| 1 | During the stationary red-light wait, model departure forecasts and Carrot's green state recur, including around 18–21 s and 34.5–39.8 s. Soft hold remains active and the car stays stopped. The later actual green/departure is a separate event. |
| 2 | The car travels between intersections. Signal-stop state starts near 51.23 s, the video shows the approaching red signal, and stopped state is reached near 59.57 s. The earlier 4.36–9.8 s stop-state episode occurs during travel and is not sufficient evidence of a red-light perception error. |
| 3 | At the red signal, the forecast changes from stopping to departure around 3.2–3.4 s. Soft hold is initially absent, longitudinal control is active, and the vehicle starts. Brake input subsequently disengages longitudinal control. Later false departure forecasts occur while soft hold prevents motion. |
| 4 | The same stationary wait continues. Around 13.9–22 s, departure predictions occur while the visible signal remains red. Video at 26.0 s is still red and at 27.0 s is green; departure follows near 27.87 s. |

Red-signal review windows were deliberately bounded away from the genuine green
transitions: segment 1, 0–44.5 s; segment 3, 0–59.95 s; segment 4, 0–26.5 s.
Within these windows, stationary (`vEgo < 1 km/h`) Carrot-green samples total
227 / 173 / 230 model-plan frames, approximately 11.35 / 8.65 / 11.50 s.
These are sampled state durations, not precise annotated signal-change times.

### Segment 3 model chronology

| Segment time | Model x at +10 s | Model v at +1 s | Model v at +10 s | Published immediate model accel |
| --- | ---: | ---: | ---: | ---: |
| 2.500 s | 0.82 m | 0.0066 m/s | 0.33 m/s | −0.0368 m/s² |
| 3.200 s | 23.34 m | −0.0020 m/s | 6.26 m/s | −0.0319 m/s² |
| 3.402 s | 63.47 m | 0.1403 m/s | 12.50 m/s | −0.0329 m/s² |
| 4.500 s | 76.06 m | 2.0453 m/s | 11.94 m/s | +0.4513 m/s² |

At 3.402 s the forecast covers only 0.028 m in its first second, yet forecasts
63.47 m by ten seconds. Thus the initial disagreement is predominantly in the
future departure trajectory, not a simultaneous large immediate model accel.
The near-future trajectory subsequently rises too: it is not valid to claim
that only the far end is ever erroneous.

The first recorded Carrot cruise/green transition is at 3.4017 s. A sampled
positive acceleration command appears by 3.5041 s. The model's published
immediate acceleration exceeds +0.1 m/s² by 4.2519 s. These are different
signals and should not be described as the same neural output.

Full-rate `carState` records brake input at **4.9258 s, 4.18 km/h**. Peak filtered
speed is **6.83 km/h at 5.5355 s**, after that brake edge. There is no gas-pedal
input in the first nine seconds. The initial commentary's 4.3 km/h referred to
the brake-time speed in a plan-aligned sample, not the peak speed.

Video shows oncoming and crossing traffic changing as the incorrect prediction
begins, while the observed signal stays red. This is an association worth testing,
not proof that a truck, clearing intersection, shadow, or exposure caused it.

## What the actual ONNX proves

The complete graph contains 653 initializers and 395,302,147 scalar initializer
elements (including constants; not all are necessarily learned parameters).
The following identities are directly established from graph connectivity:

1. `outputs[917:1907]` is `plan`: 990 values, parsed into 495 trajectory means
   and 495 uncertainty values, corresponding to 33 × 15 components.
2. That output is produced by
   `model.off_policy.temporal_hydra.*.plan`, ending in `mul_41`.
3. The immediate action output occupies `outputs[2062:2066]` and comes from a
   separate branch. The `plan`-specific weights are **not ancestors** of this
   action branch, nor of the returned visual hidden state.
4. The plan depends on current imagery, image history, visual-feature history,
   desire/history, traffic convention, and action timing. It is not fed an
   explicit red/green classifier result. This does not imply that visual signal
   information cannot be encoded implicitly.
5. The plan head consumes a **512-value latent representation** and has seven
   operations: LayerNorm, 512→1024 linear, GELU, 1024→512 linear, residual add,
   512→990 linear, and output scaling.
6. Its seven initializers contain **1,558,460 scalar elements**. An unmodified
   standalone `plan_head.onnx` was extracted, passed `onnx.checker`, and ran in
   ONNX's reference evaluator on a fixed synthetic input with finite `(1,990)`
   output. This validates extraction/execution, not traffic-light behavior.

The input/output names `off_policy` and `on_policy` are evidence of architecture,
not a complete reconstruction of the unpublished loss function or labels used
for this specific checkpoint. Do not claim that ONNX reveals the exact training
reason for the wrong forecast.

## Model-side adjustment, keeping start conditions unchanged

There is a concrete model-side correction surface: additional supervised
training of the plan output using signal-wait and genuine-departure sequences.
An original training repository is not an absolute prerequisite for all such
experiments: the exported head includes its operations and weights, and can be
reconstructed as a differentiable module. Original training data, objectives,
optimizer state, and a ready fine-tuning pipeline are not supplied by ONNX.

A bounded first experiment could freeze the visual/temporal representation and
train the final-layer **longitudinal mean rows**: for each of 33 times, component
0 (x), 3 (vx), and 6 (ax). This gives 99 selected output rows. Keeping other
rows, earlier head weights, and scaling fixed preserves other output rows for
identical latent inputs; simply training the entire shared head would not.
Positions, speeds, and accelerations need joint consistency targets.

The graph shows that plan-only parameter changes do not directly alter the
separate action or recurrent visual-state outputs for the same input sequence.
Downstream vehicle behavior can still change, so this is not a driving-safety
or closed-loop equivalence claim.

Whether this small correction suffices depends on the latent representation:
if it distinguishes relevant red/green signal states, its readout can be improved;
if signal information is lost or confused with surrounding traffic, the visual
or temporal network also needs adaptation. That cannot be resolved from the
head dimensions alone. A few waits at two intersections are not a representative
training/validation corpus, and forcing all red-light 10-second forecasts to zero
would also require care around imminent real signal changes.

The next causal experiment is to reproduce the baseline with original paired
camera frames, calibration and warmed temporal history, then vary **only** the
signal region, **only** surrounding vehicle regions, and **only** prior history
in separate trials with suitable control perturbations. Compare both the plan
latent/output and action branch. Color edits/masks and history resets can create
unfamiliar inputs; several controlled perturbations are needed rather than a
single heatmap or zero-mask result. No causal ablation or fine-tuning result is
claimed in this investigation.

## Local evidence

Private logs, video extracts, decoded data, graph inventories, trajectory plots,
the verified source ONNX and unmodified extracted head are retained under
`.analysis/archive/2026-10-09-1079-signal/` with an index. They are ignored local
analysis evidence, not committed recordings or published artifacts.
