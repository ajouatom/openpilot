# Carnival braking interruption from a short stopping-path endpoint

## Finding

The full rlog and qcamera for Carnival segment
`000001dc--23e0551f1d--19`, running `db4aed1e`, confirm a 0.797-second
loss of both control leads while approaching the vehicle stopping ahead.
Front radar track 43 remains measured and the model's lead probability stays
between 0.9793 and 0.9886 throughout the 16 missing radarState frames.

The immediate cause is path projection at the endpoint of the model's short
stopping trajectory. A centimetre-scale final segment has an unstable direction.
Its normal converts the longitudinal distance beyond the path endpoint into a
large apparent lateral offset. The resulting dPath rejects the measured front
lead and also rejects the visual fallback. This is not evidence of the physical
vehicle moving sideways, a radar measurement dropout, or a camera frame gap.

The initial investigation changed no driving code. The subsequently authorized
projection correction and its validation are described below; selection
thresholds, settings and model artifacts remain unchanged.

## Recorded sequence

Times use the first carState message, monotonic time 1193.504644788 s, as zero.
These are log times, not exact video presentation timestamps. The clocks service
places the incident at approximately 12:13:48 KST on September 25.

| Time (s) | Recorded result |
|---|---|
| 26.98 | Track 43 selected; approximately 50 km/h; acceleration request approximately -2.17 m/s². |
| 27.04 | Track 43 retained, calculated dPath 3.70 m. |
| 27.08 | Track 43 retained through the bounded path-outlier allowance, dPath 7.62 m. |
| 27.136 | L1 and L2 both absent; track 43 still measured; dPath 10.93 m. Ego speed 49.1 km/h. |
| 27.18 onward | Planner changes from lead0 to cruise; the negative acceleration request progressively weakens. |
| 27.933 | Track 43 reacquired at aligned distance 41.61 m, dPath 1.80 m; ego speed 44.8 km/h. The planner still shows the preceding cruise result at this publication instant. |
| 27.99 onward | Planner follows lead0 again and strengthens braking; around 29–30 s requests approximately -2.8 to -3.0 m/s². |

Between 27.1 and 28.0 s, both carControl acceleration and carOutput acceleration
range from -2.199 to -0.255 m/s². The minimum braking request is still negative:
the record shows a large reduction in braking, not a literal zero-pressure
measurement. No brake-pressure measurement is used here. Actual ego acceleration
also weakens after the command change. Gas and brake pedal flags remain false
and longitudinal control remains active during this event.

Video shows the dark vehicle ahead staying in the same lane, a white vehicle
to its left and a blue bus to the right, approaching a red signal. It does not
show the lead leaving the lane during the interruption. Track 43 slows toward
zero speed; this is a moving-to-stopped lead, not a newly appearing radar ID.

## Geometry and rejection mechanism

At the first missing frame, using the recorded model timestamp and the
production radar-to-model time alignment:

| Quantity | Value |
|---|---:|
| Aligned front 43 position, radar coordinates | (52.0484, -0.0009) m |
| Visual lead position, camera-offset corrected | (49.2435, +0.2643) m |
| Model path endpoint, converted to radar coordinates | (40.9323, -0.0967) m |
| Last path segment delta | (+0.006073, -0.030914) m |
| Normalized final segment tangent | (+0.192766, -0.981245) |
| Front 43 calculated dPath | +10.9261 m |
| Visual lead calculated dPath | +8.2249 m |

The model path ends about 11.12 m before the aligned radar point. The last
segment is only 3.15 cm long and points almost sideways. In
`radar_motion/predictor.py::_project_to_model_path_cached`, the nearest projection
is clamped to that endpoint, and the normal offset is computed as:

```text
dPath = -tangent_y * (object_x - endpoint_x)
        + tangent_x * (object_y - endpoint_y)
```

The approximately 11 m longitudinal separation therefore dominates the alleged
lateral separation. The object's actual radar lateral position remains near
zero. Small endpoint-coordinate changes, not a multi-metre bend of the road,
produce the large dPath swings.

In `primary.py`, ordinary stationary retention allows 4 m, and the bounded
high-confidence path-outlier allowance permits up to 8 m for 0.20 s. The 10.93 m
value exceeds even that allowance; no retained candidate survives and stationary
identity is reset. The ordinary vision/radar matcher also rejects its dPath.
Fresh admission subsequently requires a tighter 2 m path gate. At recovery,
the ordinary matcher accepts track 43 at dPath 1.80 m; stationary confirmation
has not yet re-established its retained identity at that instant.

In `controller.py::_central_vision_fallback_allowed`, visual fallback requires
absolute dPath at most 1 m. Its first-loss value is 8.22 m and remains above
1 m throughout the missing interval, so the strong visual lead does not fill
the gap. Both physical and visual decisions depend on the same problematic
endpoint geometry.

## Verification and limits

- Full OpenPilot cereal decoding counts 1,199 radarState and liveTracks messages,
  1,200 modelV2 messages, 1,198 longitudinalPlan messages, 6,009 carControl
  messages, and 5,998 carState/carOutput messages. Services were inspected,
  not inferred missing from the reduced opendbc schema.
- The newest available settings snapshot preceding the incident is
  `toggles-20260925-115523.json`: EnableRadarTracks=1, EnableCornerRadar=2,
  RadarTrackFlip=false. Earlier same-day snapshots agree. No later snapshot
  was available at analysis time. Replay consumes recorded liveTracks without
  applying another lateral inversion.
- Direct controller replay uses mode 1, recorded model inputs, recorded
  carStateMonoTime, model timestampEof, preceding liveTracks publication, and
  recorded pose yaw. All 16 missing L1 frames and the recovery frame reproduce.
  L1 presence/ID matches 1,197 of 1,199 frames; the only differences are at
  0.135 and 0.189 s, where replay begins without the preceding segment's state.
  L2 presence/ID matches all 1,199 frames. These are role/ID comparisons, not a
  claim of bitwise equality of every output field.
- The controller, primary matcher, and projection source files used for replay
  have no differences from `db4aed1e`. Existing unrelated working-tree changes
  were not used to reconstruct raw radar inputs or modified by this analysis.
- From 26–29 s, all inspected carState, radarState, modelV2, liveTracks,
  longitudinalPlan, road/wide camera and livePose messages are valid.
  modelV2 frameDropPerc is zero; pose inputsOK/sensorsOK/posenetOK remain true.
  This incident does not exhibit the previously investigated camera/pose failure.
- The downstream acceleration sequence is the actual recorded sequence.
  No closed-loop braking simulation, patched-controller vehicle trial, or
  stopping-distance benefit is claimed.

The repair should address how path direction and lateral offset are defined
when a stopping trajectory ends before a continuously observed lead. Simply
increasing dPath limits or holding all lost targets longer would obscure the
geometry defect. A candidate needs regression checks for genuine curves,
lane departures, roadside objects, and front/corner selection before deployment.

## Projection correction

For an object beyond the maximum forward extent of the measured model path,
whose closest projection is in the last 2 m of path arc length, the projection
now derives its tangent from a spatial chord across those last 2 m. It keeps
the closest point and arc position clamped to the original polyline; no extra
ray or segment participates in the nearest-point search. Interior projections
remain unchanged. Long terminal segments already supply the same direction.

Paths shorter than the required spatial support, backwards terminal chords,
and folded tails with less than 1 m of net chord displacement retain the
original geometry. This avoids deriving another unstable heading from a tail
that travels back over itself. The same shared projection applies to radar and
vision; no lead identity, confidence, distance, speed, acquisition threshold,
retention duration, process placement or model output is overridden.

At the first lost frame, front 43 dPath changes from 10.926 m to 1.098 m.
The recorded-input controller keeps front 43 through all 16 formerly missing
frames. The production/NAS replay adapter also retains front 43 throughout
the maintained 26.8–28.2 s replay window in modes 1, 2 and 3. In the baseline,
modes 1/2 lose L1 for 16 frames; mode 3 uses its existing SCC fallback instead.
This correction preserves the front identity there rather than relying on SCC.

Sixteen focused regression cases cover both lateral sides, actual adjacent-lane
offsets, curved roads, unchanged interior projection, returning/folded paths and
controller continuity in modes 1/2/3. Thirteen fail against the original
projection, while the three preservation cases already pass. All sixteen pass
with the correction. The combined isolated matcher, controller, cut-in/cut-out,
lane-change, stationary-evidence and route-vault test run passes **866 tests**.
The source paths were explicitly checked to ensure the isolated committed
checkout plus this correction was imported, excluding other working-tree edits.

The full corpus comparison is recorded in
[the validation report](carnival_stopping_path_endpoint_validation.json).
The baseline contains 97 logs / 494 items; adding this incident yields
98 logs / 495 items in each of modes 1, 2 and 3. Every existing pass/fail,
input-coverage and continuity verdict remains unchanged; the new front-43
continuity case passes in all three modes. Missing logs: zero.

| Mode | Existing selection failures, before → after | Pre-deceleration failures, before → after | Unverified items |
|---|---:|---:|---:|
| 1 | 11 → 11 | 0 → 0 | 208 |
| 2 | 10 → 10 | 0 → 0 | 208 |
| 3 | 13 → 13 | 1 → 1 | 208 |

The strict validator remains nonzero when pre-existing failures or unverified
labels remain; these must not be described as a complete corpus pass.

The correction changes lead selection on recorded inputs. It does not provide
counterfactual brake-pressure measurements or establish closed-loop stopping
distance, on-device timing or vehicle-driving validation.

The shared predictor is already included in the committed Carrot Routes image
bundle and in its replay fingerprint. Deployment uses the existing image
workflow and scheduled NAS updater. Completion requires the intended image
commit, verified updater state, actual upload-result page and recalculated
incident data to agree with the fresh committed bundle.

Deployment was verified on September 25 at 13:43 KST for source commit
`1b7dbbd3fef8e18f24e203971e1df402ead3f07e`. The
[Carrot Routes image workflow](https://github.com/ajouatom/openpilot/actions/runs/36095183097)
passed all 1,454 Linux tests and published the image. The scheduled NAS updater
reports `updated` and the same source commit; the live health endpoint agrees.
The actual incident result page returns HTTP 200 with its radar viewer.
All 1,200 served frames and all seven graphs exactly match the fresh committed
bundle. Front 43 occupies all 28 frames in the maintained continuity window,
compared with 12/28 on the previous deployed replay.

The deployed source fingerprint is `351876445bb5b8110b8a`. Computing the same
fingerprint on Windows requires reproducing Linux's case-sensitive PosixPath
sort order; the native Windows path sort gives a different hash over the same
bytes. This ordering difference was resolved for verification without changing
the production fingerprint implementation or any replay frame/graph content.

Private reproduction scripts, selected telemetry CSV, frame contact sheet,
counts and summaries are indexed in
`.analysis/archive/2026-09-25/carnival-target-loss/INDEX.md`. Raw inputs remain
on the NAS; they and settings snapshots are not Git-tracked artifacts.
