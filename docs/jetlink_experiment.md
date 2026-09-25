# Carrot Jetlink experiment

This experiment is scoped to `carrot-jetlink`, branched from `carrot-wip`
`db4aed1e22`. It does not change the maintained wip branch's eGPU model or OS.
It integrates [Jetlink](https://github.com/zoompilot/jetlink) at
`194ff6dc71cd27282378fff155e94daa7770152b` under its MIT license.

## Model and transport

The C3/C4 QCOM GPU performs the existing NV12 projective camera warp. The
FunctionFS USB gadget sends the two model-sized images and recurrent context
to the USB host. Jetson executes TensorRT; Apple Silicon can use upstream
ONNX Runtime/CoreML. The result returns to normal model parsing on the comma.
Vehicle CAN/control processes remain on the comma. A host cannot send CAN or
write vehicle settings through the display extension.

The pinned model is **Cinque v2**, SHA-256
`09d080f36965bb2a0790500452bd328aa03c484d0222aa79d1ad9f021a522aec`,
766,040,736 bytes. The full checkpoint, tensor layouts, queue cadence and output
slices are checked against `openpilot/selfdrive/modeld/jetlink/cinque_v2.json`.
The AMD Cinque v3 compiled artifact is not a TensorRT model; this experiment
does not implement Cinque v3 without its original portable model.

Host provisioning uses a distinct NAS directory, `models/carrot-jetlink-cinque-v2`,
with the original ONNX and `manifest.json`. Run
`python tools/jetlink/download_model.py --output /path/to/cinque-v2.onnx`
before building/importing an engine. It checks the manifest against the branch
pin and verifies the complete file's size and SHA-256. It never replaces a
different existing model. This path is separate from wip's eGPU v3 manifest;
GitHub LFS is not the model download source.

The comma is the USB **device**, Jetson/Mac the **host**. Use a SuperSpeed data
cable from the comma USB-C port to a host-capable Jetson port. The tested
Orin Nano developer kit uses its USB-A host port; its USB-C port is not an
equivalent SuperSpeed host connection. Current AGNOS has FunctionFS but no
ECM/NCM gadget driver, so this uses bulk USB rather than USB Ethernet.
The daemon checks source/host attachment and requires negotiated SuperSpeed.
The existing eGPU path retains priority when UsbGpuActive is true.

The upstream test VID/PID `1209:0001` is retained for this isolated experiment;
general distribution needs an appropriate USB identity allocation.

## Failure behavior and scheduling

Native inference executes three frames before an external join, warming the
fallback. Joining or rejoining requires fresh, valid, parked and fully
disengaged vehicle/control state. A connected external model can continue
while moving, but a failed connection requests the existing `commIssue`
event and restores the native model with recurrent queues cleared in place.
This retains the existing soft-disable policy; it is not a claim of immediate
disengagement. The local request has a total 150 ms deadline, including partial
reads, and rejects stale frame IDs, malformed lengths and nonfinite outputs.
Model/camera gaps retain normal odometry and pose invalidity.

Late startup is not a one-time selection: native inference runs while the host
is unavailable, readiness is refreshed about once per second, and a failed
join is retried after five seconds when the same parked/disengaged conditions
hold. Being onroad does not itself forbid a join. Being stopped with control
still enabled does forbid it; a host that becomes ready during driving waits
for a later valid stopped, fully disengaged state. No startup-wide wait is used.

On September25 after the user restarted vehicle power, C4 uptime was about69
minutes while Jetson uptime was about4 minutes. Both Jetson services were
active and C4 reported active Cinque v2 inference. A separate five-second
read-only parked observation received100 model/odometry/pose/camera messages,
with no invalidity or frame-ID gaps; DM was disabled. Jetson's monotonic boot
journal recorded service start at23.143 s, USB hello at34.789 s, and engine
ready at45.370 s. Systemd startup itself took31.659 s, so OS/service startup
must not be described as model-ready time. The wall clock jumped during time
synchronization; elapsed time is taken from the monotonic journal. These
observations verify the resulting active connection after late host startup,
not the precise vehicle-side switching instant or repeated cold-boot reliability.
Private evidence is under `.analysis/archive/2026-09-25/jetlink-late-start/`.

The bounded USB/IPC daemon and its receive worker use FIFO1 on core7 onroad,
below DM (FIFO5) and modeld (FIFO54). Offroad, the main thread returns to normal
priority and all daemon threads move to cores0..3. Existing camera,
control, radar, sensor and model priorities/affinities are unchanged. Display
snapshot/JPEG workers are separate processes using the existing display
SCHED_OTHER/nice19, core7/onroad and cores0..3/offroad policy.
The publisher is launched through `chrt --other 0` so it never inherits the
transport's realtime policy. Cyclic garbage collection runs between USB
sessions, outside inference deadlines.

## Jetson installation

See [deployment automation review](jetlink_deployment_review.md) for the current
packaging/checking tools, installation sequence and unimplemented update/rollback
automation. Host badges now identify active Jetson/Mac peers as `jetSON`/`MAC`;
the separate eGPU hardware diagnostics retain their existing meaning.

The prepared runtime layout is `RUNTIME/{venv,carrot,cache}`. Export committed
host sources with `python tools/jetlink/build_host_bundle.py host.tar.gz` and
extract into `RUNTIME/carrot`; `SOURCE_COMMIT` records the exact exported revision.
Use a system-site-packages venv to access the JetPack TensorRT installation and
install `tools/jetlink/requirements-jetson.txt`. Do not replace JetPack's driver
or TensorRT packages using generic pip packages.

`prepare_cache.py` can import an existing verified TensorRT plan, checking its
model, device and runtime metadata. Otherwise build the verified original ONNX
with the vendored server before installing boot services. Boot never compiles
an engine or waits for Internet model downloads.

Run the installers as root with `RUNTIME` and the normal service-account name:

```
bash RUNTIME/carrot/tools/jetlink/install_performance.sh RUNTIME
bash RUNTIME/carrot/tools/jetlink/install_server.sh RUNTIME USER
bash RUNTIME/carrot/tools/jetlink/install_hud.sh RUNTIME USER
bash RUNTIME/carrot/tools/jetlink/install_headless.sh RUNTIME USER
```

The performance installer requires the already-selected rated MAXN_SUPER
power mode and enables `jetson_clocks` at boot; it does not overclock or bypass
thermal controls. Select the appropriate supported power mode beforehand.
Headless setup backs up Xorg configuration, saves the previous default target,
and uses a dedicated NVIDIA X server with an ephemeral authorization cookie.
SSH and Wi-Fi autoconnection remain available without the desktop. The server
and HUD restart through systemd and wait for the USB peer.

Rollback: disable the `carrot-jetlink`, `carrot-jetlink-hud`,
`carrot-jetlink-xorg` and `carrot-jetlink-performance` services; restore the
saved Xorg configuration/default target and prior supported power/clock mode.
On the comma, return to the known-good wip branch and reconnect the eGPU and
USB display in their original topology. Never copy experiment-only model
selection into wip while doing so.

## USB display

The tested TURZX 1920x462 display attaches to a separate Jetson USB host port.
An optional capability-negotiated extension forwards bounded, read-only cereal
snapshots, selected settings and small full-field JPEG camera previews. Stale
snapshots expire. Model inference bytes and output contracts are unchanged.
The existing Carrot renderer uses these snapshots and sends H.264 to the USB
display at a requested 10 FPS. Orin Nano lacks the video encoder block, so this
uses software libx264, with low-priority CPU placement. Actual achieved display
rate can be lower under load. USB send heartbeat is returned to the comma.

Navigation guidance messages are forwarded; the separate navigation map-media
stream is not implemented in this adapter. A Linux host advertises the display
extension; the Mac launcher currently provides inference only. `run_mac.sh`
requires Apple Silicon and a verified Cinque v2 ONNX. Intel Mac support and
Mac performance have not been tested. The Mac launcher requires Python3.10+.

## Validation record (2026-09-25)

The actual C4 to Orin Nano Super connection negotiated 5 Gbit/s; the USB
display negotiated 480 Mbit/s on a separate host bus. Vehicle observations
were parked, disengaged, with no steering or acceleration commands enabled.
C3, Mac and driving behavior are not established by these observations.

* Prewarped USB+IPC benchmark, locked rated clocks: 1,200 frames, mean36.24 ms,
  p9541.35 ms, maximum49.95 ms. This excludes camera warp and display workload.
* Early 600 s actual-camera trial with display gauges but without camera
  previews, DM disabled: 12,000 model/odometry/camera messages, no model frame
  gaps or invalid odometry/pose; mean39.45 ms, p9542.27 ms, maximum85.51 ms.
* Disconnect testing found a cold native fallback around970 ms. Warming it
  before joining reduced the observed first fallback to56.91 ms; an induced
  daemon stop requested commIssue and rejoined while parked about5.08 s later.
* Adding camera previews exposed rare >100 ms tails and real dropped model
  inputs/invalid odometry. JPEG conversion, camera process isolation and a
  FIFO1 USB reader alone did not eliminate them. These are failed intermediate
  configurations, not evidence of validated driving safety.

The final onroad configuration (`ea58c2d0d4`) completed a separate600 s parked
trial with **DM enabled, both camera previews and USB HUD output**:

| Measurement | Result |
|---|---:|
| Model executions observed | 11,999 |
| Mean / p95 / p99 execution | 43.064 / 49.312 / 51.586 ms |
| Maximum execution | 57.456 ms |
| Executions over50 ms | 428 (3.57%) |
| Model frame-ID gaps / invalid odometry / invalid pose inputs | 0 / 0 / 0 |
| Camera SOF gaps over75 ms | 0 |
| Maximum camera SOF interval | 58.940 ms |
| External-model status samples | 599 active, none inactive/stale |
| USB HUD reported output | 10 FPS |

All three cameras and livePose had12,000 observations; odometry and DM had
11,999. Counts at the observation boundaries differ by one; no model frame-ID
gap was observed. Model time includes camera warp, USB/IPC, TensorRT and model
processing, not merely GPU kernel time. Above50 ms samples are reported rather
than hidden by an average; no deadlines or validity thresholds were relaxed.
This is not a simultaneous controlled comparison with the disconnected eGPU.

The final60 s whole-core observation measured C4 core7 mean60.46%, maximum71%.
Core4 still briefly reached100%; average headroom is not a driving guarantee.
The new USB owner used about87 MiB RSS, unchanged across observed checkpoints.
Jetson samples with inference and display were about12.4 W VDD_IN and65 C;
VDD_IN is board input power, not GPU-only power. Fan and thermal limits remain
active. C4, Jetson server, Xorg and USB renderer all recovered after software
reboots. Jetson reported20.832 s for kernel plus userspace startup; this does
not include a measured physical ignition-to-display interval.

An earlier600 s DM trial changed USB-owner priority partway through and had
three frame-ID gaps before the change. It is retained as a mixed-condition
failed trial and is not substituted for the separate final run above. Its
120 s FIFO1 subset had2,400 frames, no invalidity, mean43.10 ms/max55.07 ms.
The combined isolation, GC and scheduling changes improved the measured tail;
these trials do not independently prove a single cause for every earlier stall.

After the final run, restoring the user's DisableDM=2 stopped the DM processes
and coincided with one150 ms local IPC timeout and two skipped model inputs.
The native fallback ran and Jetlink rejoined automatically while parked. The
hot DM-setting transition remains a known limitation; no claim is made that
changing this setting under control is validated. DisableDM was restored to2.

Separate process tests under comma UID1000 verified repeated onroad/offroad
core7-to-cores0..3 transitions and that spawned display work uses SCHED_OTHER.
They do not substitute for a physical ignition-off/hotplug test. Fifteen
focused tests passed on C4, including Unix-socket deadline/error handling,
stale display rejection and model-download integrity/preservation. The new
NAS HTTPS model was downloaded in full and matched the pinned size/hash.

After deployment of `89453fd137`, a final induced USB-owner SIGINT produced
`commIssue`, restored native inference (first observed fallback69.66 ms), and
rejoined the external model about5.05 s after the fault. Model publication
remained alive throughout the80 s observation after initial subscription.
The manager restarted the USB owner and the display publisher without manual
intervention. This tests process/link loss, not physical cable integrity.

Private captures, device settings and access material are excluded from this
repository. Full measurements and reproduction tools are archived locally
under `.analysis/archive/2026-09-25/jetlink-integration/`.

## Incomplete latency isolation (2026-09-25)

A subsequent parked, disengaged 120 s observation kept DM enabled while
disabling all HUD packets, camera-preview production and the Jetson USB HUD
renderer. C4 per-frame bounded timing records confirmed zero HUD bytes.
The full five-condition comparison was stopped when the user needed the car;
the capnp-only, preview, display and repeated-baseline conditions did not
complete. Original DisableDM=2 and full display publication were restored,
the Jetson HUD service was restarted, and external inference was active.

The completed condition observed 2,401 model messages: mean41.574 ms,
p9950.586 ms, maximum53.498 ms, with48 executions above50 ms. No frame-ID
gaps, invalid odometry/pose inputs or camera intervals above75 ms were observed.
Thus HUD/capnp traffic is not necessary for an execution to exceed50 ms.
The difference from the earlier full-display run is not a controlled estimate
of HUD cost: the planned repeat and other conditions remain incomplete.

The following decomposition joins2,400 model/USB-owner records by session
frame counter; it excludes about0.18 ms of enclosing model-loop overhead.

| Interval | Mean | p99 | Maximum |
|---|---:|---:|---:|
| Instrumented model call | 41.398 ms | 50.410 ms | 53.322 ms |
| C4 camera warp, wall time | 7.860 ms | 16.262 ms | 17.651 ms |
| C4 warp thread CPU time | 5.819 ms | 6.352 ms | 6.592 ms |
| Local request IPC | 2.535 ms | 6.817 ms | 8.700 ms |
| USB request submission | 3.744 ms | 6.802 ms | 10.334 ms |
| USB response wait/receive/parse | 25.774 ms | 30.608 ms | 35.799 ms |
| Jetson reported GPU execution | 19.629 ms | 19.665 ms | 20.129 ms |
| Jetson reported total processing | 21.066 ms | 21.146 ms | 21.736 ms |
| Local reply IPC | 0.658 ms | 5.450 ms | 6.299 ms |
| Model output parsing | 0.658 ms | 0.771 ms | 0.857 ms |

These rows overlap (GPU is part of response time); maxima must not be added.
For the44 instrumented calls above50 ms, mean warp time was14.927 ms versus
7.728 ms in other calls, while warp thread CPU was6.035 versus5.814 ms.
Mean response time was29.196 versus25.710 ms; GPU time remained19.629 ms
in both groups. The longest call contained17.018 ms warp and29.357 ms
response time, including19.619 ms GPU execution.

This localizes the varying latency primarily to warp elapsed time and the
response path rather than TensorRT execution. It does not distinguish QCOM
queue/synchronization waits, CPU scheduling, kernel I/O or Python thread
handoff. Transport read-wait includes peer/idle time, and handoff timing is
not a pure scheduler measurement. Next device tests must measure scheduling
and warp synchronization separately, then repeat identical DM/display
conditions for any candidate change. No latency fix or driving validation is
claimed from this interrupted investigation. Private raw captures, analysis
and the opt-in instrumentation patch are archived under
`.analysis/archive/2026-09-25/jetlink-latency/`.
