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

Vision/radar `leadOne` matching is independent of the new cut-in probability. The cut-in model uses only the current point and past radar history to emit `P(h)`, the probability that the same vehicle occupies the ego path 0.5/1.0/1.5/2.0 seconds later. Scene-specific distance, low-speed, stationary, or curve corrections do not rewrite that probability.

The measured current path state is `P0`: 1 when the vehicle is inside the path and 0 when it is outside. `IN` is the maximum of `P0` and future `P(h)` before ego passes the object. `OUT` is the maximum of `1-P0` and future `1-P(h)`. At one horizon the outside probability is exactly `1-P(h)`, so one number is sufficient. Aggregated IN and OUT are not complements and can both be high when a vehicle crosses the boundary.

CUT-IN activates when an outside vehicle's IN reaches the model threshold; CUT-OUT uses OUT for an inside vehicle. The release threshold is 0.05 below activation, providing only probability hysteresis. A measured outside-to-inside or inside-to-outside crossing by the same physically continuous vehicle is a probability-1 transition held for one second. A normal lead first observed inside the path is not labeled as a cut-in transition.

When `leadOne` exists, a farther CUT-IN keeps its raw probability but cannot become `leadTwo`. A duplicate of the same `leadOne` is also removed. No other distance-, speed-, or lane-specific exception is added to the new probability. A selected CUT-IN is passed to `leadTwo` with its measured radar distance, speed, and acceleration.

The PC validator `radar_lead_validation_review.py` executes the same model, features, and post-processing as the device and pauses only when a CUT-IN becomes final `leadTwo`.

### PC trajectory review

The PC validator opens each log only once even when it contains multiple validation windows and marks every window on the timeline. The radar map expands lateral motion by 4x. `POINTS` shows measured radar returns, while `PATH` shows recent history and a separate geometric projection. Point and detail labels show `P0`, `IN`, `OUT`, and `P(h)` for the selected display horizon. The second detail line retains the four unmodified model values `P0.5/P1.0/P1.5/P2.0`. A horizon after ego has passed the object is excluded only from actionable IN while its raw value remains visible.

Point fill identifies the sensor: front radar is cyan, corner radar purple, and SCC yellow. Only one outline ring is used. Purple means high IN/CUT-IN, orange means CUT-OUT, red means farther than `leadOne`, and green means final `leadTwo`. Orange and yellow boxes are final `leadOne` and `leadTwo`. Points beyond 45 m retain their position and trajectory but omit text. At most five map labels and three detail rows are shown.

With no probability option, the PC validator displays rings using the embedded front/corner device thresholds. Playback pauses only when a CUT-IN becomes final `leadTwo`; an unselected high score never pauses. `radar_lead_validation_review.py --prob 0.70` changes only the ring display threshold, and `--manual-prob` restores the last saved display value. Neither option changes pause events, model output, or the on-device threshold. Seeking backward rearms later production `leadTwo` events. Forward seeking and manual Space pauses replace any stale automatic-pause message with `MANUAL SEEK` or `MANUAL PAUSE`.

Front and corner models are trained as separate artifacts. They use only the current point and its measured past history. Each target says whether the same physically continuous track's measured position occupies the ego path 0.5, 1.0, 1.5, or 2.0 seconds later; training is not restricted to samples that are currently outside. Future measurements are target-only and never inference inputs. Unmeasured front-radar slots and physically discontinuous reused IDs are excluded. Human `CUT-IN`/`CLEAR` labels and their complete logs are held out from fitting and used only for validation.

The new path-occupancy CUT-IN drives actual `leadTwo` selection for both sources. A corner-equipped vehicle uses the corner decision; a vehicle without corner radar uses the front decision. CUT-OUT does not directly remove the current `leadOne` or reduce time gap; it only releases a short stale hold after vision has already lost the match.

`radar_lead_validation_review.py --trajectory-table-only` prints the manual-label evaluation and a side-by-side comparison of the branch-point `carrot-wip` decision with the final production path-occupancy decision. It then repeats the comparison using measured future path entry as truth on the same scorable windows, making broadly marked human windows visible separately.

The `DISPLAY FUTURE` slider immediately changes only the drawn future point and label `P(h)`. It is display-only and never changes model inference, IN/OUT, controller thresholds, or `leadTwo`. The model always evaluates its fixed 0.5/1.0/1.5/2.0-second heads. The geometric projection uses point history, ego `position.x/y`, `position.yStd`, `laneLines`, `laneLineProbs`, `laneLineStds`, `yawRate`, steering angle, and steering rate.

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
