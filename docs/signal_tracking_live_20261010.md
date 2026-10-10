# Live signal tracking observer — 2026-10-10

The user requested installation on their connected vehicle after the offline
tracking trial. The existing opt-in `signalcolord` process now dispatches to
`signal_tracking_shadow` when `/data/signal-color-shadow/tracking_enabled` is `1`.
Both that flag and the existing `enabled` flag are required. Without the new flag,
the previous color observer is unchanged. Select the mode while the observer is
stopped; do not run both camera observers concurrently.

The live worker uses the same `tools/signal_analysis/signal_tracker.py` as the
[offline trial](signal_tracking_trial_20261010.md). It copies active padded NV12
planes and converts them using OpenCV BT.601 limited-range NV12-to-RGB. The accepted
camera geometry is 1344x760. This conversion differs from encoded HEVC RGB and
requires real-scene validation. No labels, vehicle state or future frames enter
recognition. No model output, planning, actuation or departure threshold changes
are included. Green is an observation, never permission to move.

## Recording and resource limits

The road VisionIPC client is conflated. Reused buffers, stale inputs over 150 ms,
camera timeouts and reversed frame IDs reset tracking. Results older than 200 ms
are logged as unknown and reset the tracker. The tracker retains its existing
confirmation/gap rules; skipped frames can therefore reduce recognition coverage.

All existing worker threads use little CPUs 0–3, SCHED_OTHER and nice19. OpenCV
uses one thread and no OpenCL. The loop targets at most 20 Hz and at most half of
one CPU through post-work delays; this is not an OS quota or isolation guarantee.
There is no catch-up processing. Ordinary failures use the existing error latch.

`signalTrackingShadowLoaded`, `signalTrackingShadowSkipped`, and
`signalTrackingShadow` events go through the existing diagnostic logger. Records
include source hash, camera frame ID/EOF time, track boxes/identity/evidence,
red/green/unknown, freshness, input/result age, CPU and elapsed work time.
`control_permission` is always false. `tracking_latest.json` is updated at most
once per second; its timestamp must be checked because an old file is not proof
of a running process. The old color model files remain available but are not run
in tracking mode. The existing policy observer remains independent.

## Validation before installation

- 36 desktop integration/conversion/temporal tests passed.
- OpenCV 4.13 (the device version) reproduced all 1,447 prior six-clip state
  decisions from OpenCV 5.0. This does not prove raw-camera equivalence.
- A finite 25-second candidate run continuously required fresh valid Park,
  standstill, |vEgo|<0.01 m/s and disabled/inactive control. The prior color
  observer was temporarily stopped and restored afterward.
- Baseline: 200/200 valid model frames, 20.0015 Hz, no frame gaps/drop.
  Observation window: 640/640 valid model frames, 20.0001 Hz, no frame gaps/drop;
  mean model execution changed from 24.865 to 25.279 ms.
- 388 fresh tracking results, all unknown in the parked scene. Work median
  32.36 ms, p95 50.58 ms; CPU median 25.21 ms; result-age p95 112.55 ms.
  Camera-sample interval median 50.38 ms, with skipped frames; the run averaged
  about 15.5 observations per second. This is not a measured 20 Hz observer.

These establish execution and recording in a parked trial, not road recognition
accuracy, loaded driving performance, red stopping or prevention of false starts.
The user's acceptance priority remains red stopping and avoiding false departure;
this observer supplies live comparison evidence toward that objective.

Private staging scripts, trial records and installation evidence stay under
`.analysis/archive/2026-10-10-signal-tracking-live/`. No private video/model data
are included in this commit.
