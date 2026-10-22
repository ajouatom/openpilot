# Jetson startup status and latest drive review — 2026-10-22

## Display correction

The user reported red Jetson status between ignition and connection. Two
reproducible status-policy defects are corrected:

* The always-running daemon retained `ready_once` across drives. After a
  confirmed offroad host absence, start a new bounded first-handshake window.
  Onroad detach, unknown road state and contradictory road flags do not reset
  this window. Repeated onroad detach cannot perpetually renew the 120-second
  grace. A successful session ends grace immediately.
* A `connecting` report expired after three seconds, although auto USB HELLO
  can block for three seconds and the regular HELLO for five. Keep only this
  progress report current for eight seconds (HELLO plus reconnect pause).
  A stalled daemon still becomes an error after that bound. Other states,
  model reports and actual health telemetry retain their three-second limits.

Unanswered initial HELLO remains "연결 대기"; model preparation remains
"모델 준비 중". Real model errors, USB failures, preparation errors, unsafe
cleanup, failed updates and failures after a working session remain errors.
No inference deadline, transport bytes, scheduling, control policy or model
is changed. This fixes demonstrated display conditions; no historical UI
snapshot identifies which condition produced each red flash the user saw.

## Actual drive evidence

Read-only collection: 21 recent qlogs, then all ten rlogs for the latest two
drives, 117,274,500 compressed bytes. Every rlog matches the vehicle's SHA-256.
Raw files, routes, addresses and device identifiers remain private. Both drives
ran the published watchdog correction `cd986d7e9d`, on the same comma boot with
the persistent Jetlink daemon. N16 hash remained
`01effc679eb46d6036ce0302673a87fe2dcfcc95a679daeeb929679e53c3647a`.

| Korean local time | Latest, approximately 12:09–12:13 | Previous, approximately 11:55–11:59 |
| --- | ---: | ---: |
| External-model outputs, including first call | 1,326 | 2,645 |
| External output first-to-last span | 66.223 s | 132.230 s |
| Mean model call | 42.369 ms | 42.259 ms |
| p99 model call | 46.531 ms | 46.764 ms |
| Maximum model call | 78.483 ms | 77.631 ms |
| Calls exceeding 50 ms | 5 | 6 |
| External model frame-ID gaps | 0 | 1 |
| Invalid external model messages | 0 | 0 |
| Logged external failure/fallback | 0 | 0 |

`modelExecutionTime` is seconds in cereal; the table converts it to
milliseconds. It measures the model call, not only GPU execution. The queued
"Jetlink active" warning arrives just after the first model publication; the
first call is included using its frame-1 timing record and adjacent model event.
The cold first calls were 66.544 and 62.728 ms respectively.

The latest drive lasted about four minutes, but N16 was active only for its
last 66 seconds. Earlier output came from the internal model while host/warp
preparation and the existing transition gate completed. Warp preparation was
26.322 s (previous drive 27.860 s); its completion can wait for the join gate.
Do not describe the entire trip as Jetson operation or change that gate here.

During the latest external interval all 1,326 model, DM, camera-odometry and
pose messages were valid, with no model frame-ID gap or pose input-health
failure. It averaged 20.008 Hz between first and last external output. One
`selfdrivedLagging` event occurred earlier, before external activation.

In the previous drive, 0.95 s after external activation, model camera
synchronization skipped one frame while both cameras logged continuous valid
frame IDs. `modelCameraGap`, one invalid camera-odometry result, one
`livePose.inputsOK=false` result and `locationdTemporaryError` agree at that
instant. `posenetOK` and `sensorsOK` stayed true. This is not evidence of a
locationd crash, USB disconnect or missing camera capture. The pose event's
outer `valid` bit stayed true; inspect its input-health bit as well.

## Remaining latency

The latest drive's 60.713 ms call has a same-frame slow record: warp 4.36 ms,
roundtrip 55.08 ms, server GPU 24.11 ms and server total 25.50 ms. That excursion
is outside pure inference. Other slow records show comma warp-read delays
(9–12 ms); the previous drive also contains a server-total excursion to
44.46 ms with GPU still 24.11 ms. There is more than one delay source.

The largest 78.483 ms model call is not individually localized by the retained
inner slow records; nearby aggregate maxima must not be added as though they
belonged to that frame. Therefore neither cable failure, CPU saturation nor a
complete timing fix is established. No external failure/fallback was logged;
this does not measure USB physical-layer retry counters. Display-media-byte
metrics were zero during these external windows, so they do not validate
simultaneous phone-map video load or prove absence of model NAV input.

## Checks and delivery

71 focused desktop status/startup/health checks pass on each branch. Native
daemon tests need Linux (`fcntl` is unavailable on this Windows host).
[NAV CI](https://github.com/ajouatom/openpilot/actions/runs/39729244111) passes:
94 updater tests, 1,008 broad tests (56 skips), 119 navigation/HUD tests,
11 real-filesystem storage tests and 11 network-namespace tests.
[Wip CI](https://github.com/ajouatom/openpilot/actions/runs/39729244377) passes:
801 broad tests (2 skips), 119 navigation/HUD and 11 storage tests. Both Windows
installer checks pass. Implementation commits are NAV `d7505b1cde` and
Wip `1b0bac6384`, both published with single-parent history.
The correction goes to both maintained
branches, preserving their distinct model architectures. Normal comma Git Pull
and restart/reboot applies it; no Jetson USB patch, new runtime bundle or model
installation is needed. Vehicle collection this turn was read-only; no live
process was restarted and no driving validation of this display fix is claimed.
