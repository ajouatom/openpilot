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

The corner-radar cut-in decision normally requires agreement between temporal-model probability and lane/body-entry geometry. A corner-only vehicle within 6 m can use a two-frame early confirmation only when multiple history intervals show meaningful sustained inward motion and the base model also strongly agrees. Merely having several history samples or weak lane-relative jitter cannot enable this early path. One current-frame base-model score alone cannot confirm it, limiting transient false detections from curves and parallel traffic.

On vehicles whose front radar does not provide instantaneous lateral velocity, the source-separated model method can confirm a cut-in before the vehicle fully reaches the lane line when both the 0.4-second and 0.6-second lane-relative histories show consistent inward motion. This history override is not applied when lateral velocity is available, which limits false detections from parallel traffic.

Model mode classifies a corner-radar candidate at 5 m or farther without a matching front-radar object as a `TENTATIVE` cut-in. It keeps the measured distance but sends ego speed and zero acceleration to longitudinal control, reducing abrupt braking for an uncertain candidate. A front-radar match or a near corner candidate inside 5 m is `CONFIRMED` and uses the measured radar speed and acceleration.

The PC review tool `radar_lead_validation_review.py` pauses for both tentative and confirmed cut-ins and shows `TENTATIVE` or `CONFIRMED` in the result panel and pause reason.

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
