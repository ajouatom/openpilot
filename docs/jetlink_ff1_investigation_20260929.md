# Ioniq 5 PE ff1 startup and process alert, 2026-09-29

Evidence: C4 `07b62e389ed26c81`, commit `250f14ed`, full rlogs
`00000ff1--64803a8344--0` and `--2`. Times below are relative to each
segment's first carState (monotonic 13175.953297910 and 13295.824863003).
All services were decoded with the full cereal schema, not the CAN-only reader.

## Mid-drive alert in segment 2

- At 46.178 s, Jetlink frame 1628 reports warp 4.32 ms, roundtrip 97.28 ms,
  parse 0.64 ms; server GPU/queue/total are 20.52/1.51/22.20 ms.
- modelV2 publication has a 107.193 ms gap and frame IDs jump 3325 -> 3327.
  modeld reports one dropped input. All three camera frame-ID sequences remain
  consecutive; maximum SOF intervals across the segment are road 56.024 ms,
  wide 56.026 ms, driver 55.967 ms. This is not an observed camera capture gap.
- cameraOdometry is invalid once at 46.214 s. livePose inputsOK becomes false
  from 46.224 to 46.530 s while its outer valid flag stays true. Downstream
  calibration/parameters/assistance/torque validity briefly drops; the final
  liveTorqueParameters recovery is at 46.830 s.
- At 46.500 s an enable attempt receives commIssue/noEntry naming
  liveTorqueParameters and driverAssistance. The displayed alert lasts until
  49.505 s; that display duration does not mean the inputs stayed invalid for
  three seconds. Engagement succeeds at 52.257 s.
- All 105 managerState samples report expected processes running, with no PID
  changes. This is an input-validity alert, not a recorded process crash.
- DM temporarily enters interaction fallback at 46.261 s because its required
  inputs include liveCalibration, and recovers at 48.511 s after its existing
  two-second health qualification. driverStateV2 remains valid throughout.

The excess roundtrip time is outside the reported server internal total, but
these logs cannot localize it to USB, host/gadget scheduling, or IPC. No timing
deadline, pose validity, warning, or model control behavior is relaxed.

## Startup DM notice in segment 0

Driver camera messages begin at -0.307 s; 1,203 valid messages have consecutive
frame IDs. DM model loading completes at 0.592 s, with first driverStateV2 at
2.574 s. Its startup publication gaps reach 2.782 s and 1.436 s; later output
recovers. All 990 driverStateV2 messages are valid. The duration of these gaps
is evidence of delayed inference output, not proof of a sensor failure or a
specific GPU contention cause.

Separately, modeld waits five seconds for absent eGPU enumeration, then loads
the internal/Jetlink wrapper for 8.7 s (including verified Jetlink warp setup).
First modelV2 arrives at 14.012 s; liveCalibration becomes valid at 14.835 s.
DM requires driverStateV2, modelV2 and liveCalibration plus two continuous
healthy seconds. cameraUnavailable clears at 16.984 s. The visible
driverMonitorFallback/permanent notice spans 9.032-17.005 s, with no sound.
The notice means camera-based monitoring is unavailable during preparation;
it does not establish that the physical DM camera failed.

All three camera frame-ID sequences remain consecutive; all 109 managerState
samples have expected processes running with stable PIDs. Startup camera
probing also records three ioctl op266/errno19 messages, but these cannot be
assigned to the DM sensor merely from that message. An Athena remote WebSocket
disconnect at 55.265 s is separate from the reported process alert in segment 2.

## Optional-host badge correction

The user requested hiding an absent Jetson and restoring READY on connection.
Previously a fresh `waiting` report with remembered peer identity was quiet
only after modeld published a fresh inactive report. Startup before that report
could therefore display ERROR despite the daemon explicitly confirming absence.
The badge JSON itself is not recorded in these rlogs, so the exact observed
screen transition cannot be reconstructed from them.

A fresh `waiting` report now hides the badge even before the first model report
or after an inactive report expires. Fresh active-session conflicts or model
errors remain visible. Stale/unknown link state, connection failures, and host
health errors remain visible. A fresh healthy ready report restores READY.
Only display diagnostics change; connection retries and actual model selection
are unchanged.

Validation: 17 focused status/host-health tests pass on Windows with
`python -m pytest --confcutdir=tools/jetlink tools/jetlink/test_status.py
tools/jetlink/test_host_health.py -q`. The root conftest requires unavailable
Windows params_pyx; these pure-Python tests use their own directory boundary.
Physical startup display behavior and the underlying isolated Jetlink latency
remain unvalidated. Reproduction scripts and private evidence are indexed in
`.analysis/archive/2026-09-29/ff1/` and are not committed.
