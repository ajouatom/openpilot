# EV9 roadside false CUT-IN, 2026-09-23

## Evidence

Route `KIA_EV9 76665b7de33d4586/00000125--53364ef9d7--20` was recorded
on `79047fb88f4fe6702376309ab0b73018789262c1` (`carrot-wip`, mici).
Full-schema decoding found 1,200 liveTracks, radarState, modelV2 and camera
index messages, plus 6,000 carState messages. Replay produces 1,199 frames.
The qcamera sequence shows roadside objects on the right staying outside
ego's lane; the dark video does not identify the exact reflecting surface.

At replay 14.099 s (video 14.249 s), corner 2302 was confirmed as CUT-IN:
its path-relative position history implied 2.71 m/s inward motion, but its
yaw-corrected reported inward velocity was only about 0.01 m/s. Its position
had moved from roughly -5.05 m to -4.20 m in 0.40 s. Associated front 52
remained outside at about -3.9 m with ground speed about 0.35 m/s and no
supporting inward motion. Corner ground speed briefly exceeded the existing
moving-target gate. Cross-sensor association supplied existence evidence
without corroborating the apparent movement.

Baseline replay selected the pair as L2 for 49 frames, 14.099-16.499 s.
The recorded planner acceleration falls to approximately -4 m/s² in this
region. This is recorded behavior, not a newly simulated planner result.

## Correction

Reject this corner position extrapolation when the associated front is
near-stationary (absolute ground speed at most 0.5 m/s), both returns remain
outside the path/body corridor, vision does not corroborate the corner,
both radars lack inward motion (less than 0.15 m/s), and the position-derived
corner rate exceeds its reported rate by more than 0.75 m/s.

The front motion vote includes its own coherent position history above the
existing range-dependent noise floor, as well as its reported velocity.
Current overlap, visual support, measured inward motion and moving front
targets remain eligible under the existing detector rules. The veto clears
CUT-IN/risk latches and an active L2 for the same identity immediately.
It does not change stationary primary selection, radar orientation, sensor
decoding, scheduling, model artifacts, sensitivity settings or validity limits.

## Verification

- 849 distinct focused detector/controller/replay/planner/vault test cases
  passed, including bilateral suppression, sensitivity 1-5, independent
  motion/vision/overlap preservation and immediate L2/risk release. Ruff and
  whitespace checks passed.
- The actual EV9 log passes in modes 2 and 3 at every sensitivity 1-5:
  no target CUT-IN, pre-deceleration, L1 or L2 in the full 12-18 s window.
  At the default sensitivity all 1,199 L1 outputs remain exactly unchanged;
  the 49 erroneous L2 frames are removed.
- Full maintained replay: 97 logs / 494 labels per mode, including the new
  target-scoped case. All 493 pre-existing report rows compare exactly with
  the baseline. No source logs are missing. The new case observes its target
  in 93 of 120 window frames and passes both detection and lead-role checks.
- Existing expectation failures remain 10 in mode 2 and 13 in mode 3;
  pre-deceleration failures remain 0 and 1. There are still 208 unverified
  labels with insufficient target input. This is a no-regression comparison,
  not a claim that the entire corpus passes.

Reproduce full comparison with
`python openpilot/selfdrive/carrot/radar/tools/validate_radar_lead_model.py --shadow-only --enable-radar-tracks 2 --report result.json`
and repeat with mode 3. Use `--case ev9-125-20 --strict-shadow --strict-predecel`
for the focused maintained case.

The shared module participates in the existing source fingerprint and deploys
through the Carrot Routes image workflow and scheduled NAS updater. Confirm
the committed source, updater verification, result page and recalculated EV9
frames before closing deployment. Offline replay does not validate closed-loop
vehicle braking or every physical stationary-to-moving cut-in.
