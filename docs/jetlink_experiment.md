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

Further final-configuration results belong below. Private captures, device
settings and access material are excluded from this repository.
