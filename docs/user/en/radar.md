# Radar Tracks and Corner Radar

[한국어](../ko/radar.md)

> [!NOTE]
> This is the canonical English user guide maintained with the `carrot-wip` code. When user-visible behavior changes, update this document together with the related code and tests.

A radar fitted to the vehicle does not guarantee that carrotpilot can read the required CAN messages. Results can differ by model year, trim, radar part, firmware, message group, and harness connection even for the same vehicle name.

> [!WARNING]
> An unverified radar configuration can cause dashboard warnings, CAN faults, incorrect lead selection, or false cut-in detection. Record the original values and test one setting at a time only when the exact vehicle configuration has been validated.

<a id="front-radar"></a>
## Front radar tracks

`EnableRadarTracks` selects the source and processing used for front lead information.

| Value | Current code behavior | Guidance |
|---:|---|---|
| `-2` | VOACC vision-only experiment | Development testing only |
| `-1` | Always use SCC | Confirm the vehicle configuration |
| `0` | Use stock SCC radar | Default |
| `1` | Use raw front-radar tracks | Requires vehicle-specific activation and message support |
| `2` | Combine radar tracks with low-speed SCC | Test only on an identical validated configuration |
| `3` | Add cut-in and low-observability vehicle processing | Experimental; false detections are possible |

On non-CAN FD Hyundai/Kia vehicles, a positive value attempts to enable radar tracks during startup and stores the result in `EnableRadarTracksResult`. Confirm both the activation result and actual incoming tracks; physical radar presence alone is not enough.

<a id="corner-radar"></a>
## Corner radar

| `EnableCornerRadar` | Meaning |
|---:|---|
| `0` | Disabled |
| `1` | Use supported corner-radar tracks |
| `2` | Also use corner radar for cut-in detection |

Corner-radar objects are created only when the vehicle code recognizes a supported message group. The supported 0x430 message family is also classified as corner-radar input rather than front radar. The cut-in processing in mode `2` currently focuses on Hyundai-family implementations and must not be generalized to other manufacturers.

<a id="lead-fusion"></a>
## Source-separated radar models

| `RadarLeadModelMode` | Meaning |
|---:|---|
| `0` | Existing lead-selection method |
| `1` | Source-separated model method with independent front and corner decisions |

`RadarLeadModelMode=1` takes effect after the vehicle or device is restarted. It changes model lead selection independently of whether raw radar tracks are available, so do not change several radar options at once.

In model mode, `EnableRadarTracks` also selects the front-model input. Values `-1` and `0` use SCC only, `1` uses front-radar tracks only, and `2` or `3` use front-radar tracks together with low-speed SCC. SCC and front-radar inputs are never mixed into the corner-radar model.

The front and corner models do not share history or decision state. The front model always owns `leadOne` matching and that vehicle's CUT-OUT. On a corner-equipped vehicle, only the corner model owns adjacent-vehicle CUT-IN and `leadTwo` decisions. The front model also owns CUT-IN and `leadTwo` on a vehicle without corner radar or during PC validation with `--front-only`. When a front point matches a selected corner candidate, its control-quality distance, speed, and acceleration are used. A side candidate inside 5 m keeps the corner measurement instead of the noisy near-field front lateral position.

The front and corner model artifacts are trained and validated separately. Passing `--front-only` to the PC validator removes corner points and exercises the same front-only path used by a device without corner radar.

Vision/radar `leadOne` matching is independent of the new cut-in probability. Using only the current point and past radar history, the cut-in model emits the same vehicle's longitudinal distance `X(h)`, lane/path-center-relative lateral distance `Y(h)`, and per-axis `XStd(h)`/`YStd(h)` at 0.5/1.0/1.5/2.0 seconds. Scene-specific distance, low-speed, stationary, or curve exceptions do not rewrite those positions or probabilities.

The measured current path state is `P0`: 1 when the radar-return center is within the lane/path boundary and 0 when it is outside. No assumed vehicle half-width is added to that boundary. Future `I(h)` is the probability mass of the `X(h), XStd(h)` distribution remaining ahead of ego, multiplied by the probability mass of `Y(h), YStd(h)` inside the left and right boundaries. `O(h)` is the probability of remaining ahead while outside those boundaries. `IN` is the maximum of `P0` and future `I(h)`; `OUT` is the maximum of `1-P0` and future `O(h)`. Probability mass already behind ego contributes to neither value. Aggregated IN and OUT can both be high because they can come from different horizons.

CUT-IN activates whenever IN reaches the model threshold. A vehicle currently inside the path has `IN=1.00` by definition and activates immediately even when first observed there. An outside entry candidate uses only a release threshold 0.05 below activation. Once the same physical vehicle is measured inside, a momentary probability drop cannot release it; CUT-IN ends after it has been measured outside for at least 0.25 seconds. CUT-OUT uses the separate OUT output for an inside vehicle and also shows a measured inside-to-outside transition.

When `leadOne` exists, a farther CUT-IN keeps its raw probability but cannot become `leadTwo`. A duplicate of the same `leadOne` is also removed. The nearest remaining candidate is selected, and that physical identity remains `leadTwo` until it becomes `leadOne`, is measured outside, or its track is lost. No other distance-, speed-, or lane-specific exception is added to the new probability. A selected CUT-IN is passed to `leadTwo` with its measured radar distance, speed, and acceleration.

The PC validator `radar_lead_validation_review.py` executes the same model, features, and post-processing as the device and pauses only when a CUT-IN becomes final `leadTwo`.

### PC trajectory review

The PC validator opens each log only once even when it contains multiple validation windows and marks every window on the timeline. The radar map expands lateral motion by 4x. `POINTS` shows measured radar returns, while `PATH` shows recent history and the model's future position distributions. Gray history reprojects each old `dPath` onto the current path instead of joining measurements expressed in different old ego frames. Validation shows the same independent front/corner inputs as the device and omits the legacy `FUSED` comparison view. Point and detail labels show `P0`, `IN`, `OUT`, and all four raw `X/Y/XStd/YStd/I/O` values. Near the 0.5 m longitudinal boundary, `XStd(h)` continuously weights IN/OUT by the probability of remaining ahead; the raw position remains visible.

Point fill identifies path state: green means currently inside or confirmed CUT-IN, and purple means outside. An outside point above the displayed IN threshold gets a green outline; an inside point above its model OUT threshold gets a purple outline. Orange and yellow boxes are final `leadOne` and `leadTwo`. Point names use `F` for front and `C` for corner radar. The selected `leadTwo` line includes its actual decision source and `P0/IN/OUT/ON`. Points beyond 35 m retain their position and trajectory but omit text. At most three map labels and one highest-priority full raw-probability detail row are shown.

With no probability option, the PC validator displays rings using the embedded front/corner device thresholds. Playback pauses only when a CUT-IN becomes final `leadTwo`; an unselected high score never pauses. `radar_lead_validation_review.py --prob 0.70` changes only the ring display threshold, and `--manual-prob` restores the last saved display value. Neither option changes pause events, model output, or the on-device threshold. Seeking backward rearms later production `leadTwo` events. Forward seeking and manual Space pauses replace any stale automatic-pause message with `MANUAL SEEK` or `MANUAL PAUSE`.

Front and corner models are trained as separate artifacts. They use only the current point and its measured past history. Targets are the same physically continuous track's measured future longitudinal distance and its lateral distance from the future lane/path center at 0.5, 1.0, 1.5, and 2.0 seconds. A baseline first extends the recent measured longitudinal and lateral motion, then the neural network learns the position residual and standard deviation. Self-supervised rows that actually cross the path boundary within those two seconds receive more lateral-position loss weight than steady rows, so early entry motion is not overwhelmed by the much more common traffic that stays in its lane. The embedded entry threshold is selected from grouped self-supervised cross-validation at an 80% precision target; manual labels do not choose it. Future measurements are target-only and never inference inputs. Unmeasured front-radar slots and physically discontinuous reused IDs are excluded. Every log variant in a segment carrying a human `CUT-IN`/`CLEAR` label in either `radar_trajectory_labels.json` or `cutin_validation_cases.json`, including files such as `rlog.zst` and `rlog.1.zst`, is held out from fitting and used only for validation.

With corner radar present, only the corner CUT-IN decision drives actual `leadTwo` selection. A vehicle without corner radar, or PC validation with `--front-only`, uses the front CUT-IN decision. Raw probabilities and future positions remain visible at any range, while actual `leadTwo` control is limited to a point no farther than a closer `leadOne` and within ego travel over two seconds plus 10 m, with a 20 m minimum. Because `leadOne` is identified by front radar, its front-model CUT-OUT remains active regardless of corner-radar availability. Raw probabilities and histories remain source-separated; front and corner future positions or CUT-IN probabilities are never averaged. CUT-OUT does not directly remove the current `leadOne` or reduce time gap; it only releases a short stale hold after vision has already lost the match.

`radar_lead_validation_review.py --trajectory-table-only` prints the manual-label evaluation and a side-by-side comparison of the branch-point `carrot-wip` decision with the final production path-occupancy decision. It then repeats the comparison using measured future path entry as truth on the same scorable windows, making broadly marked human windows visible separately.

The `DISPLAY FUTURE` slider immediately changes only the displayed `X/Y/XStd/YStd/I/O` horizon. It is display-only and never changes model inference, aggregate IN/OUT, controller thresholds, or `leadTwo`. The model always evaluates its fixed 0.5/1.0/1.5/2.0-second outputs. Ego yaw uses a valid `livePose.angularVelocityDevice.z` no more than 0.20 seconds old, then falls back to an estimate from speed, steering angle, and vehicle geometry. This model does not consume `carState.yawRate`, whose units and sign are not consistent across every vehicle interface. With the livePose device-frame sign convention, radar lateral speed is corrected as `yvRel - yawRate × dRel`; PC replay, training, and device inference use the same priority and features.

Selecting `CUT-IN`, `CLEAR`, or `STATIONARY` inside a maintained validation window updates that case. A candidate found outside every maintained window is saved separately in `radar_trajectory_labels.json`, so it cannot overwrite existing ground truth.

## Radar detection sounds

When openpilot is enabled, a newly confirmed cut-in plays a two-tone cue. A continuously tracked object sounds only once. On the speakerless C3X Lite, the same event uses a GPIO buzzer pattern. The cue reports the selected radar result; it does not change lead selection or longitudinal control. A higher-priority safety alert can take precedence.

## Relationship to harness presets

The first-run presets currently start with these values:

| Configuration | `EnableRadarTracks` | `EnableCornerRadar` |
|---|---:|---:|
| ADAS-module harness | `0` | `1` |
| Camera harness | `0` | `0` |
| Retain stock SCC | `0` | `0` |

The ADAS preset enabling corner radar means that its harness can access such a configuration. It does not mean that every ADAS-equipped vehicle has validated corner-radar messages.

## Verification order

1. Confirm the vehicle, model year, trim, HDA generation, and exact harness location.
2. Record the current values and verify normal behavior at the default value `0`.
3. Find a validation record for the same configuration and supported radar messages.
4. Change only one setting, then restart the vehicle/device.
5. While stationary, check for dashboard warnings and CAN faults.
6. In a safe test environment, verify lead distance, relative speed, and false cut-in detections.
7. Restore the original value immediately if anything is abnormal.

## Code references

- Setting ranges and descriptions: `openpilot/selfdrive/carrot_settings.json`
- Source-separated model runtime: `openpilot/selfdrive/carrot/radar/radard_model.py`
- Hyundai/Kia radar parsing: `opendbc_repo/opendbc/car/hyundai/radar_interface.py`
- Non-CAN FD radar activation: `opendbc_repo/opendbc/car/hyundai/interface.py`
- First-run presets: `openpilot/selfdrive/carrot/server/features/intro/presets.py`
