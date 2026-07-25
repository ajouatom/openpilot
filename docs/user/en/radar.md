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

The front and corner models do not share history or decision state. Vehicles with corner radar use the front model for `leadOne` and the corner model for cut-in and secondary leads; without corner radar, the front model also owns the secondary decision. When a front point matches a selected corner candidate, its control-quality distance, speed, and acceleration are used. A side candidate inside 5 m keeps the corner measurement instead of the noisy near-field front lateral position, while identity association still prevents the same vehicle from being reported twice.

The front and corner model artifacts are trained and validated separately. Passing `--front-only` to the PC validator removes corner points and exercises the same front-only path used by a device without corner radar.

A fresh radar point outside the current driving path is not associated with `leadOne`, even when vision range uncertainty is large. A secondary candidate at the same position as `leadOne` is removed as a duplicate. An unmatched distant corner candidate is not reported even as a tentative cut-in until the vehicle body reaches the lane boundary, reducing false detections from parallel and adjacent-lane traffic.

The source-separated model method confirms a front-radar-only cut-in only when its lateral motion remains consistent across multiple history intervals. A previously matched small target corroborated by both front and corner radar is retained through brief lateral vision jitter to reduce lead dropouts.

A stationary vehicle may tolerate disagreement between sensor speeds only when vision probability is sufficient and front and corner radar corroborate an on-path object within 3 m longitudinally and 1.2 m laterally. Adjacent vehicles and distant moving points are not merged as stationary evidence. When a front match exists, its control distance, speed, and acceleration remain authoritative to limit braking from tunnel-entrance and structural reflections.

The corner-radar cut-in decision normally requires agreement between temporal-model probability and lane/body-entry geometry. A corner-only vehicle within 6 m can use a two-frame early confirmation only when multiple history intervals show meaningful sustained inward motion and the base model also strongly agrees. Merely having several history samples or weak lane-relative jitter cannot enable this early path. One current-frame base-model score alone cannot confirm it, limiting transient false detections from curves and parallel traffic.

On vehicles whose front radar does not provide instantaneous lateral velocity, the source-separated model method can confirm a cut-in before the vehicle fully reaches the lane line when both the 0.4-second and 0.6-second lane-relative histories show consistent inward motion. This history override is not applied when lateral velocity is available, which limits false detections from parallel traffic.

Model mode can classify a corner-radar candidate without a matching front-radar object as a `TENTATIVE` cut-in. This includes a candidate at 5 m or farther and a very close candidate inside 5 m when model probability is low but multiple history intervals show inward motion and the vehicle body has reached the lane boundary. A tentative candidate keeps its measured range but sends ego speed and zero acceleration to longitudinal control, reducing abrupt braking from uncertain side traffic or low-speed waiting traffic. A candidate confirmed by a front-radar match or sufficient model evidence is `CONFIRMED` and uses measured radar speed and acceleration.

The PC review tool `radar_lead_validation_review.py` pauses for both tentative and confirmed cut-ins and shows `TENTATIVE` or `CONFIRMED` in the result panel and pause reason.

### PC trajectory review

The PC validator opens each log only once even when it contains multiple validation windows, and marks every window on the timeline. The radar map expands lateral motion by 4x. `POINTS` shows current measured radar returns, while `PATH` shows recent history and predicted positions at 0.25-second intervals. `IN` is the maximum probability that a currently outside vehicle will occupy the ego path at 0.5, 1.0, 1.5, or 2.0 seconds while its predicted longitudinal position is still ahead of ego. A horizon after ego has already passed the object is excluded from `IN`; the detail row shows it as `--` in `ahead` while retaining its model output under `raw`. `OUT` is the maximum probability that a currently on-path vehicle will leave it at those horizons. An ego lane change that moves the lead outside the new ego path is also an `OUT`, so interpret it as PATH-EXIT rather than only the other vehicle's lane change. The detail panel lists all four raw path-occupancy probabilities; `H` is the separate geometric occupancy estimate at the selected display horizon. When `IN` is at or above the `PROB` slider value, the point gets a green double ring and an `!` prefix, and automatic review pauses there with an alert. When `leadOne` exists, farther points retain their raw probabilities and history but are marked `L` and excluded from highlighting, alerts, pauses, and cut-in candidates. `X` marks a PATH-EXIT decision. Changing `PROB` immediately rebuilds the current log's pause events. Seeking backward with the timeline or Left key rearms later events, so replaying the same section can alert and pause at the same point again.

The last `PROB` value is saved in the PC user settings and reused for the next log and the next invocation. It does not need to be specified each time; use `radar_lead_validation_review.py --prob 0.70` to set an initial value explicitly. This value controls only PC review highlighting and automatic pauses, not the on-device control threshold.

Front and corner models are trained as separate artifacts. They use only the current point and its measured past history. Each target says whether the same physically continuous track's measured position occupies the ego path 0.5, 1.0, 1.5, or 2.0 seconds later; training is not restricted to samples that are currently outside. Future measurements are target-only and never inference inputs. Unmeasured front-radar slots and physically discontinuous reused IDs are excluded. Human `CUT-IN`/`CLEAR` labels and their complete logs are held out from fitting and used only for validation.

Full-log validation found that recall at a high-precision CUT-IN threshold is still too low, so automatic `leadTwo` promotion by the new model remains disabled for both front and corner sources. The same features, weights, and post-processing still run on the device and in PC review. Only the high-precision front/SCC `OUT` decision is used to stop a brief stale hold after vision has already lost the `leadOne` match. It never removes the current `leadOne` or reduces time gap, and corner `OUT` remains display/evaluation-only.

`radar_lead_validation_review.py --trajectory-table-only` prints the manual-label evaluation and a side-by-side comparison of the branch-point `carrot-wip` decision with the final path-occupancy shadow decision. It then repeats the comparison using measured future path entry as truth on the same scorable windows, making broadly marked human windows visible separately.

The `PREDICT` slider selects the separate geometric display horizon from 0.25 to 2.0 seconds. That projection combines point history with ego `position.x/y`, `position.yStd`, `laneLines`, `laneLineProbs`, and `laneLineStds`. It also records `yawRate`, steering angle, and steering rate, and displays `yawRate x distance` as the apparent lateral speed that ego rotation can create. It falls back to the predicted ego path when lane confidence is low, and widens the future-position bar as uncertainty grows.

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
- Front/corner track fusion: `openpilot/selfdrive/controls/radard.py`
- Hyundai/Kia radar parsing: `opendbc_repo/opendbc/car/hyundai/radar_interface.py`
- Non-CAN FD radar activation: `opendbc_repo/opendbc/car/hyundai/interface.py`
- First-run presets: `openpilot/selfdrive/carrot/server/features/intro/presets.py`
