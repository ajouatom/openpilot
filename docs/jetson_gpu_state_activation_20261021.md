# N16 GPU recurrence activation and latency investigation - 2026-10-21

The owner asked to enable the reviewed GPU optimization if its real-link
performance and stability are satisfactory, and to investigate remaining calls
above 50 ms. This follows [the initial default-OFF review](jetson_gpu_usb_review.md).

## What changes

Inference already runs on the Jetson GPU. The optimization retains the model's
recurrent state in GPU memory instead of uploading/downloading that state each
frame. It changes neither model weights nor camera/NAV inputs nor the driving
output contract. Every recurrent output still receives a finite-value check,
and invalid outputs/reset/close keep their existing fail-closed behavior.

The NAV TensorRT stateful backend defaults `CARROT_TRT_DEVICE_STATE` to `1`.
Explicit `0` retains the host-state diagnostic path. Queued models do not call
state-feedback configuration. Wip retains its separate model/backend; only
this technical record is shared. No new user setting is introduced.

Installation uses the existing signed host release selection and boot updater,
including its isolated candidate probe and rollback. It does not require an SD
reflash. The existing N16 model and cached TensorRT plan remain selected.

## Controlled parked comparison

The vehicle stayed in Park with zero speed, inactive controls and all pandas
not allowing controls. Each server mode switch happened during a real offroad
transition, with restoration of the prior parameter afterward. A separate
bounded supervisor required fresh actual offroad/Park heartbeats and restored
the original service on timeout, loss of guard or stop. Only five candidate
modules were overlaid for the comparison; the production release was not
rewritten. GPU ON was verified from the actual captured-graph startup log.

An initial management Wi-Fi timeout delayed setup. USB management confirmed
the correct private-Wi-Fi address, and subsequent reachability recovered. This
was outside the timed comparison. Each arm starts after N16 is active/ready;
expected USB reconnects at deliberate offroad transitions are excluded.

Each arm uses the same N16 model/plan, unchanged serial USB protocol, FIFO2
completion helper, live external navigation and USB HUD screen mode 7. The
OFF/ON/ON/OFF sequence uses 120 seconds per arm, and the middle ON arms retain
the same server process. This is a bounded parked comparison with changing
live camera/map content, not a randomized road trial or a hard-deadline proof.

The preceding isolated 1,000-frame deterministic comparison found exact output
equality including resets, and a 2.153 ms mean internal reduction. Its full
binary16 finite-scan and binary32 edge checks also passed. The new default and
explicit disable are covered by 67 focused desktop tests with recurrence,
reset, capture failure and timing regressions.

## Remaining latency

The first ON arm includes a 60.985 ms call. Its same-frame slow log reports
4.24 ms warp, 55.32 ms roundtrip, 0.63 ms parse, and Jetson GPU/queue/total
26.47/1.16/27.89 ms. That event contains substantial delay outside the server's
reported processing interval. It is not evidence that inference took 61 ms;
those numbers alone do not identify which controller/thread caused it.
A separate bounded trace investigation follows the user's additional request.
Raw images, routes, identifiers and detailed diagnostic logs remain private.

## Completed OFF/ON/ON/OFF results

| Arm (120 s) | Model calls | Mean ms | p99 ms | Max ms | >50 ms | Server total mean ms | USB exchange mean ms |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| off1 | 2400 | 44.294 | 47.350 | 51.401 | 1 | 27.888 | 37.172 |
| on1 | 2400 | 42.414 | 45.342 | 60.985 | 1 | 26.022 | 35.281 |
| on2 | 2399 | 42.439 | 45.733 | 52.111 | 1 | 26.038 | 35.302 |
| off2 | 2400 | 44.314 | 47.161 | 50.030 | 1 | 27.955 | 37.232 |

Combined model-call means: OFF 44.304 ms, ON 42.427 ms, approximately 1.877 ms
or 4.24% faster. Average USB exchange fell from about 37.202 to 35.291 ms;
server processing fell from 27.922 to 26.030 ms. Model rate remains approximately
20 Hz; this reduces per-frame latency/headroom consumption, not requested FPS.
Both modes had two calls above 50 ms in their combined four-minute windows.
The optimization does not eliminate the latency tail.

Across all four arms: no observed invalid model/DM/camera/odometry/pose record,
no frame-ID gaps, and no failed Park guards. ON2 observed 2,399 consecutive
model/camera frames within the wall-time window, versus 2,400 in the other arms;
this is not reported as a strict >=20 Hz result. Model NAV was ready in
114/120, 120/120, 117/120 and 120/120 one-second samples respectively; the
remaining OFF1/ON2 samples reported position/route_position invalidity. HUD
was fresh in all 480 host samples; NAV preview was fresh in 479/480. These
occasional route-validity differences limit perfect load matching.

The supplemental ON2/OFF2 host measurement found server CPU 8.27%/10.55% of
one CPU core, and whole-Jetson aggregate CPU 11.37%/11.70%. The samples are
sequential with different process warm states, so this is not a generalized
CPU-saving or memory-stability claim. ON2 RSS changed by -35 pages; OFF2 by
+9,996 pages. Retained host allocations are intentional in both modes; there
is no claim of GPU-memory reduction.

The repeatable mean improvement, output parity and healthy bounded live
windows justify default enablement for the maintained NAV stateful TensorRT
runtime. Rare USB/scheduling delays remain a separate issue under investigation.

## Signed release

GPU-default source `9cfe86696ea728ac56097d7788bfb8a77aa3ac6a` passed
[Linux/Windows integration CI](https://github.com/ajouatom/openpilot/actions/runs/39589009089).
The immutable signed bundle is 64,688,054 bytes, SHA256
`1e87ea9af93d625531b21dba47aa46ef66ed4f24cded034e290f9765c7c21cb2`.
All 1,088 payload files match the published source (including the documented
legacy selected-model alias); no task diagnostics or signing private key are
included. The existing NAS HTTPS download was verified against this hash.
The NAV host pin now selects this release and keeps the same N16 ONNX hash.
21 local signature/update/device-state tests pass; four platform-specific
cases are deferred to Linux CI. Pin commit `9ea56e1ddc` also passed
[Linux/Windows integration CI](https://github.com/ajouatom/openpilot/actions/runs/39589340276).

## Automatic installation verified

Under fresh Park/zero/inactive guards, normal comma Git Pull installed the pin.
The owner-authorized actual offroad hold and ordinary Jetson reboot let the boot
updater download, verify, probe and activate source `9cfe86696ea7`. The candidate
probe passed with the unchanged N16 SHA. At approximately 93 seconds after boot,
the new release was active; its normal service logged GPU device state, finite
scan and reply event enabled. No GPU environment override or temporary service
override remained. Normal onroad state was restored and N16 became ready.
This is permanent default activation through the normal updater, not just a
published bundle or a diagnostic overlay. The owner also explicitly reaffirmed
upload authorization after the initial automated-review rejection was resolved.

The subsequent comma-only watchdog investigation is recorded in
[USB watchdog wakeups](jetlink_watchdog_latency_20261021.md). It requires no
additional Jetson runtime, model replacement or SD/USB patch.
