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
fallback. As authorized September26, joining or rejoining requires fresh,
valid vehicle/control state and either standstill with abs(vEgo)<0.01 m/s or
carState.steeringPressed. Engagement alone no longer forbids that transition.
A connected external model can continue
while moving, but a failed connection requests the existing `commIssue`
event and restores the native model with recurrent queues cleared in place.
This retains the existing soft-disable policy; it is not a claim of immediate
disengagement. The local request has a total 150 ms deadline, including partial
reads, and rejects stale frame IDs, malformed lengths and nonfinite outputs.
Model/camera gaps retain normal odometry and pose invalidity.

Late startup is not a one-time selection: native inference runs while the host
is unavailable, readiness is refreshed about once per second, and a failed
join is retried after five seconds when the same stopped/steering-override
conditions hold. Being onroad does not itself forbid a join. No startup-wide
wait is used. Freshness and invalid-output checks are unchanged; the new
moving steering-override case has unit coverage, not vehicle-driving validation.

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

## Two-sided USB timing investigation (2026-09-25)

Follow-up parked observations kept DM enabled and used bounded per-frame
records. Full HUD, capnp-only HUD and no HUD packets were each observed for
90 s. Mean model executions were43.747,43.583 and41.128 ms respectively;
maxima were54.904,54.885 and85.453 ms. The no-packet observation included one
model input gap and invalid odometry/pose inputs despite complete camera
streams. Its longest C4 request submission was49.307 ms. HUD traffic is
therefore not necessary for the long transport tail. This comparison leaves
the Jetson renderer service running; it is different from the earlier trial
that stopped the renderer too. Capnp-only publication averaged about260 KB/s
and full HUD about428 KB/s in these observations.

Request submission is a synchronous FunctionFS write. A separate C4 thread
receives responses; this does not make request writes asynchronous. A long
send call can include waiting for the receiver, kernel work, or delayed CPU
resumption. It must not be presented as measured wire transfer time or as
proof that the sending endpoint is the cause.

A temporary Jetson transport wrapper recorded request reads, inference
handling and response writes. C4 kernel traces recorded writev/readv and
thread wakeups/switches. Frames were joined by protocol frame ID; clocks on
the two machines were not assumed synchronized. One120 s repeated baseline
had2,400 valid model messages, mean44.510 ms, maximum55.271 ms and78 over50 ms,
without camera/model gaps or invalid pose inputs. C4 request submission
averaged3.936 ms, and the overlapping response interval28.026 ms. Jetson
response writes averaged6.225 ms and reached13.092 ms. These intervals are
not independent and must not be added to GPU time.

For frame19802, C4 submission took13.914 ms including13.466 ms inside writev,
while the complete Jetson receive call took4.139 ms. C4 then received the
result in19.434 ms despite19.647 ms of GPU execution plus a4.197 ms Jetson
response write. Work on the Jetson overlaps the tail of the C4 submission;
the submission duration is not a direct measure of USB bandwidth.

A subsequent60 s scheduler trace localized part of the response delay to C4.
For frame23953, the reader woke at1280.664255 s but resumed at1280.667865 s,
after dmonitoringmodeld yielded:3.610 ms runnable waiting. It was subsequently
preempted for another0.973 and0.452 ms around model/DM execution. The same
frame's Jetson response write lasted14.343 ms. The reader and USB main thread
were both FIFO1 on core7, below unchanged modeld FIFO54 and DM FIFO5. The
normal-priority watchdog was also repeatedly repinned there by update_affinity.
This establishes a scheduling contribution to response delay, not that every
transport tail has the same cause.

Trace buffers were bounded and overwrote early events; scheduler conclusions
use the retained core7 interval starting1268.512155 s. The largest82.239 ms
model execution in that run occurred before the retained interval and cannot
be assigned a kernel cause from that capture. Jetson schedstat accounting was
disabled, so its zero runqueue samples do not establish absence of scheduling
delay. No camera/control/model/DM priorities or validity limits were changed
for this investigation.

### USB-only affinity comparison

After restarting with the same diagnostic build, four90 s observations used
full HUD and DM, original warp execution, and no kernel or Jetson tracing:

| USB placement | Model mean | p99 | Maximum | Above50 ms |
|---|---:|---:|---:|---:|
| A: all threads on core7 | 43.972 | 51.883 | 55.576 | 86/1,801 |
| Reader/watchdog on cores0..3; main on7 | 44.098 | 51.298 | 61.252 | 55/1,801 |
| Main/reader/watchdog on cores0..3 | 41.036 | 50.025 | 54.850 | 19/1,798 |
| A repeated: all threads on core7 | 44.022 | 51.466 | 54.039 | 66/1,800 |

Times are milliseconds. Every condition had no observed model frame gaps,
invalid pose/odometry/CAN inputs or camera intervals above75 ms. Device thread
affinities verified the placements; main/reader remained FIFO1 and watchdog
SCHED_OTHER. The return to A supports a roughly3 ms mean improvement from
moving the whole USB process, rather than moving only its reader.

The instrumented response interval fell from27.424 to25.722 ms mean, and
the maximum reader-to-consumer handoff from5.933 to0.564 ms. Request submission
mean increased from3.782 to4.564 ms; overall improvement is not faster request
transmission. GPU means stayed near20.1 ms. Warp remained variable, with a
maximum18.494 ms in the all-little condition. These overlapping intervals
locate the improvement in response/IPC handling while retaining a separate
preprocessing tail.

Among the1,797 joined all-little records,17 instrumented calls exceeded50 ms
(the enclosing published duration includes another approximately0.17 ms).
Their warp mean was16.920 ms versus8.033 ms in other calls, while request
submission means were4.556 versus4.564 ms. GPU means were21.092 versus20.103 ms
and response means27.525 versus25.704 ms. The remaining slow-call group is
dominated by additional warp elapsed time; this does not by itself distinguish
QCOM queue/synchronization from CPU submission work. Cached output/binding
micro-optimizations did not establish a fix for this tail.

The adapter now keeps USB main/reader/watchdog on cores0..3. All existing
camera/control/model/DM and display-worker placements/priorities are retained,
as are watchdog deadlines, fallback/rejoin gates and model validity policies.
This is a measured mitigation: maxima still exceeded50 ms, and the earlier
49 ms submission outlier has not been reproduced with complete kernel tracing.
Neither C3/Mac nor loaded driving is validated by these parked C4 trials.

Private cached-warp-output and TinyJit input-binding candidates were also
compared with exact pixel validation. Their small CPU reductions did not
establish stable timing; both were discarded. The committed code retains the
original warp implementation. Temporary recording modules and diagnostic
mode switches are not part of the deployed change. Local tests passed18 host
checks and4 model checks;4 Unix-socket cases require target Linux execution.

On the target C4, all8 model/link tests subsequently passed using the existing
isolated test dependencies. C4 and Jetson were updated to source
`f5749653f5`; the Jetson source was staged separately with the old source
retained for rollback. Windows checksum sidecars initially included CRLF,
which Linux sha256sum interpreted as a carriage return in the archive name.
The builder now emits LF and tests exact checksum bytes; the rebuilt bundle
passed sha256sum on the Jetson. Its renderer resumed USB H.264 at10 Hz with
26/26 forwarded services alive. The temporary host instrumentation override
and C4 diagnostic flags/modules were removed before extended validation.

### Clean deployment,600 s with DM and full USB HUD

The deployed, uninstrumented runtime observed12,001 model/odometry messages,
12,001 messages per camera and12,002 DM/pose messages. Model execution mean
was40.721 ms, p9545.462 ms, p9949.215 ms and maximum61.053 ms. There were76
executions above50 ms (0.633%) and none above75 ms. No model frame-ID gaps,
invalid odometry/pose/CAN inputs or camera intervals above75 ms were observed;
camera maximum was58.820 ms. DM mean/p99/maximum were20.770/28.584/32.949 ms.
All599 periodic external-model status samples were active.

Compared with the earlier600 s full-HUD/DM observation, mean and p99 improved
from43.064/51.586 to40.721/49.215 ms, and above50 ms executions fell from
428/11,999 (3.57%) to76/12,001 (0.633%). However, the new maximum61.053 ms
exceeded the earlier57.456 ms. This is improved typical/tail frequency, **not
a demonstrated bound of50 ms or elimination of rare long executions**. The
long-run comparison spans different boots; the within-boot A/B/A above is
the placement comparison. The clean run has no detailed stage trace and
cannot assign its61 ms maximum to one stage.

Read-only deployment checks confirmed C4 main/reader FIFO1 and watchdog
SCHED_OTHER on cores0..3, no diagnostic source/flags/tracing instance, a live
`jetSON` badge, and a fresh Jetson HUD snapshot with `external_compute_label`
`jetSON`. All four host services were active, with no temporary instrumentation
override. MAC identification is tested in software, not on a physical Mac.

Original DisableDM=2 was restored before reboot. After automatic external
rejoin, a60 s observation with the original DM-disabled setting and full HUD
had1,200 model/odometry/pose/camera messages, no invalid inputs or frame gaps,
and model mean39.501 ms, p9943.582 ms, maximum47.625 ms, zero above50 ms.
The fresh `jetSON` status and little-core USB placement remained active.
This short restoration check is not evidence of an all-conditions50 ms bound.
Both devices retain tested runtime source `f5749653f5`; subsequent investigation
documentation commits do not change that runtime.

Private captures, diagnostic source, exact candidate comparisons, target
verification and reproduction notes are retained with an index under
`.analysis/archive/2026-09-25/jetlink-stability/`. No vehicle captures or
credentials are part of the experiment commits.

### Further deadline work: C4 fused warp and bounded IPC buffers, September25

The upstream comparison confirmed all45 vendored Python runtime files match
Jetlink194ff6dc after newline normalization. Its separate comma integration
uses endpoint lending/direct modeld IO, FIFO51 receive scheduling, precompiled
warp and VM dirty/free-memory tuning. Carrot retains the separate USB process,
FIFO1/little-core placement and150 ms IPC failure deadline. Do not import the
upstream500 ms timeout as a latency fix. Upstream source comments describe
100-350 ms allocation/reclaim stalls and a244-to72 ms maximum after VM tuning;
these are author-reported observations, not a diagnosis of this C4's tail.
The upstream31 ms table is recorded-segment replay, not the full camera/DM/HUD
load measured here. Comparison sources and exact revisions are privately
indexed in `.analysis/archive/2026-09-25/jetlink-upstream-review/`.

A live modeld libusb_event thread inherited core7/FIFO54. A90-second-per-arm
parked A/B/A moving only this helper to little/SCHED_OTHER gave mean model
40.597/40.497/40.546 ms, maxima53.688/52.572/50.465 ms and10/7/4 executions
above50 ms. There was no demonstrated causal tail improvement; its original
policy was restored. Other model/DM/control/camera priorities remain unchanged.

Further bounded instrumentation split warp preparation, dispatch, buffer
resolution, GPU synchronization and CPU copy. In the original1801-frame arm,
dispatch averaged4.749 ms (p9912.403), explicit GPU synchronization0.722 ms
(p997.695), and the following copy1.987 ms (p992.356). Slow warp calls had
9.180 ms dispatch and3.514 ms synchronization but1.960 ms copying. Their
dispatch CPU time was3.801 ms and scheduling wait was negligible, localizing
the variable time to blocking inside submission/completion rather than proving
a CPU runqueue or USB bandwidth cause. The exact KGSL operation was not traced.

The IPC optimization uses scatter/gather send and a per-connection bounded
receive buffer, initially retaining the byte framing and copying model outputs
so the next reply cannot overwrite retained results. Header/payload partial
timeouts abandon the stream. The client's single existing150 ms deadline now
also covers partial sends. Positive multi-frame, partial-write, fragmentation,
oversize, stale/nonfinite/truncated reply and total-deadline tests exercise the
protocol. No model inputs, recurrent state or validity thresholds change.

In90-second arms with DM/full HUD, original/IPC-only/borrowed-GPU-view/original
means were40.959/40.630/40.164/40.839 ms, maxima54.488/52.630/51.943/55.340 ms,
and above50 counts14/10/6/15. Borrowing the GPU mapping moved most of the saved
copy time into socket submission: warp7.927-to5.959 ms but roundtrip31.853-to
33.350 ms. That candidate was discarded; production retains an owned CPU array.
Replacing numpy() with data() alone is not allocation-free in this tinygrad:
both use Buffer.as_memoryview(), which allocates and copies by default.

The selected C4-only warp samples NV12 directly into Y00/Y10/Y01/Y11/U/V planes,
fusing separate chroma extraction, plane warps and packing while preserving
projection operation order, rounding and border clamping. A separate direct
chroma-gather candidate did not materially improve timing. The fused candidate
matched every393,216-byte output in27 synthetic geometry/rounding probes and
32 real camera pairs. It also matched distinct transforms and random image
contents for both cameras. Synthetic benchmarks ran alongside the live system
on little cores: original/direct-chroma/fused/original means12.442/12.287/7.963/
12.387 ms. These are not standalone GPU kernel times or production frame times.

With IPC improvement held constant,90-second original/fused/original C4 arms
with DM and full HUD gave:

| Warp | Model mean | p99 | Maximum | Above50 ms |
| --- | ---: | ---: | ---: | ---: |
| Original A |40.826 ms|49.452 ms|53.202 ms|14/1802|
| Fused B |38.383 ms|44.974 ms|49.169 ms|0/1800|
| Original A2 |41.092 ms|49.409 ms|53.177 ms|11/1800|

All three arms had no model frame gaps or pose/CAN invalidity. Fused warp mean
was5.686 ms versus8.064/8.217 ms; fused dispatch mean/max1.660/2.094 ms.
GPU completion still had an11.328 ms isolated wait, so this is not proof of
bounded GPU latency. DM execution means also fell20.351-to18.457-to20.410 ms.

Production selects the fused warp only on C4/mici, after exact startup GPU
comparison against the original warp on27 seeded probes. Any compilation or
pixel failure retains the original warp; C3 keeps the original path. Slow
model frames record warp prepare/submit/read, IPC roundtrip and server timings
at most once per second. Diagnostic mode switches/mmap recorders are excluded.
No VM tuning, GPU context priority, process-priority or power changes are part
of this candidate. Parked observations do not establish loaded driving, C3,
Mac, thermal extremes or a hard50 ms guarantee.

After removing the diagnostic modules and rebooting, the first clean600 s
DM/full-HUD run produced12,000 valid model messages: mean37.906 ms,
p9944.432 ms, maximum52.954 ms and4 above50 ms (0.0333%). There were no model
frame gaps, pose/CAN invalidity or camera intervals above75 ms in that window;
camera maximum59.694 ms. DM mean/p99/max18.534/25.763/32.735 ms. Original
DisableDM=2 was restored afterward. Relative to the earlier600 s deployed run,
above50 executions fell76-to4 and maximum61.053-to52.954 ms across boots;
the same-boot A/B/A above supplies the controlled warp comparison.

This clean observation began at uptime158.048 s. Earlier in the same boot,
the bounded log recorded external frame826 with7.07 ms warp,149.92 ms IPC
roundtrip and23.85 ms server total, followed by two dropped input frames and
invalid pose inputs. That event is outside the clean window and is not fixed
or explained by the steady-state result. Later over50 frames included both
long GPU completion waits and elevated roundtrip times. Global direct-reclaim
counters also increased, but without per-frame tracing that is not causal
attribution. Further memory/GPU scheduling comparisons were therefore required.

The full cold-start rlog and console localize frame826 further: camera maximum
intervals were56.89/56.94/56.94 ms, while USB send took123.2 ms, response24.9 ms
and local reply0.2 ms. Thus this event is in the request transmission path,
not slow Jetson inference or a camera gap. This does not distinguish C4 kernel
allocation, USB completion or a temporarily unavailable host reader. The same
boot's cold native-model warmup also took764 ms before external join; startup
and steady-state timings must remain separate.

An180 s-per-arm VM A/B/A tested upstream dirty_bytes=16 MiB,
dirty_background_bytes=8 MiB and min_free_kbytes=128 MiB against the original
ratio20/10 and min_free_kbytes7423. Model means38.004/38.025/37.945 ms,
p9944.415/44.386/44.300 ms, maxima49.047/50.736/48.873 ms and above50 counts
0/2/0 showed no demonstrated latency benefit. All arms had3,600 models,
normal DM and no pose/CAN failures. Direct reclaim counter increments were
2/0/0; the two B deadline misses therefore cannot be attributed to those global
reclaim counters. The original VM settings were restored; no global VM tuning
is part of the selected change.

GPU debugfs confirmed both the normal native-model and DM contexts use
priority12, while UI uses8 (lower numeric value has higher GPU priority).
CPU FIFO priorities do not order those separate GPU command streams. A private
second-QCOM-context prototype at GPU priority8 left native fallback/DM at12
and all CPU policies unchanged. It passed27 distinct-camera exact pixel probes
and32 live-camera comparisons. Its120 s-per-arm original/priority8/original
comparison measured mean38.134/37.069/38.225 ms, p9944.362/40.650/45.199 ms,
maximum62.289/44.927/61.054 ms and above50 counts2/0/2. Each arm had about2,400
models, normal20 Hz DM, no camera/model gaps and no pose/CAN invalidity.
DM means18.389/17.732/17.981 ms, maxima29.468/29.664/30.785 ms.
This short comparison supports a GPU scheduling contribution but does not
establish a hard deadline or solve the separate long USB-send event.

The prototype needs a QCOM allocator map hook for same-process KGSL signal
buffers before creating a second context; stock tinygrad rejects that mapping.
KGSL page tables are shared by process, and HCQ retains borrower timelines
until free. This private backend experiment is not part of the current clean
fused-warp/IPC change. A subsequent DM-enable warmup also exceeded the150 ms
IPC deadline and correctly fell back/rejoined while parked; it is outside the
settled comparison windows and must not be counted as a successful deadline.

A thread inventory found two HUD snapshot/preview workers pinned to core7,
using3.6% and8.9% CPU in that10 s sample. Model/DM also have mostly idle helper
threads there; the inventory includes per-CPU kernel workers and a low-activity
Wi-Fi receive worker, so application affinity is not exclusive core ownership.
Moving only the two HUD workers to cores0..3 at normal/low priority in90 s
A/B/A arms gave means38.365/38.002/37.895 ms, p9945.158/44.063/43.774 ms and
maxima49.096/48.995/48.018 ms. All three had1,800 models, zero above50 and no
pose/CAN/frame failures. The reversal did not reverse the improvement, so this
does not establish a benefit from removing HUD CPU work. Original placement
was restored. Jetson HUD/server services stayed active and the live C4 HUD
connected parameter was true; the handshake's initial false telemetry is stale.

A separate bounded kernel trace selected only the USB owner's writev syscall
and direct-reclaim/compaction events in an independent trace instance. Across
1,453 model writes of475,136 bytes, writev mean/p99/max were3.483/5.819/7.550 ms;
703 HUD writes averaged1.522 ms, maximum3.942 ms. No selected reclaim/compaction
event or100+ ms send reproduced, including this DM-enable transition. Trace
settings and DisableDM were restored. This trace does not identify the cause
of the earlier123.2 ms event. The AGNOS4.9 FunctionFS source allocates a kernel
buffer for each request before queuing USB IO; allocation/reclaim is a candidate,
not an established explanation. Splitting the message is not adopted because
the upstream transport deliberately uses one write to avoid a known DWC3 replay.

### Aligning DM after the current C4 image upload

DM normally runs immediately after receiving its driver-camera buffer, without
waiting for the driving-model warp/upload. The CPU FIFO54/5 ordering does not
prevent an earlier DM GPU submission from overlapping a later driving warp.
The user's proposed upload-then-DM ordering was tested without changing either
model, cadence, recurrent state or validity policy.

An initial notification carrying only wall-clock send completion was inadequate:
the preceding road frame's upload can complete after the current driver SOF.
That experiment barely waited and is not evidence for ordering the same frame.
A follow-up explicitly joined the selected driving camera's SOF to its local
inference request and upload completion. Without gating, DM began about46.7 ms
after its SOF, while road upload completed about63.7 ms after its source SOF.
The latest completed road image was normally one frame older than the driver's.

With a12 ms maximum requested wait, only80/1801 samples in the selected arm
reached the corresponding road upload before proceeding; mean actual wait was
12.597 ms. Model means in90 s A/B/A arms were38.042/36.798/37.925 ms,
p9944.380/40.308/44.200 ms and maxima49.084/44.125/49.235 ms. DM means changed
18.365/27.189/18.490 ms including the wait, with B maximum33.542 ms. No frame
gaps or pose/CAN failures occurred inside these settled arms.

A25 ms requested cap reached the matching camera upload in1199/1200 B samples;
the remaining sample proceeded on timeout. Actual wait mean/p99/max were
17.061/23.309/25.124 ms. Separate60 s A/B/A arms gave:

| DM scheduling | Model mean | p99 | Maximum | DM mean / maximum |
| --- | ---: | ---: | ---: | ---: |
| Immediate A |38.076 ms|44.610 ms|49.864 ms|18.157 /30.270 ms|
| After matching upload B |36.838 ms|40.808 ms|41.794 ms|31.062 /41.960 ms|
| Immediate A2 |38.271 ms|45.517 ms|49.338 ms|18.103 /26.973 ms|

All arms retained20 Hz driving/DM outputs and had no camera/model/DM frame
gaps or pose/CAN failures. The benefit trades later DM publication for less
driving-warp contention; do not describe DM execution alone as unchanged or
exclude its wait from modelExecutionTime. A scheduler timeout caps intentional
waiting, not all kernel/scheduling delays, and is not a hard realtime proof.

The clean candidate carries source SOF in the C4-local request header, then
uses a nonblocking16-byte datagram after USB upload. The USB/Jetlink protocol
and host source remain unchanged. Only C4 supplies the phase timestamp and
only a Jetson peer emits notifications; native eGPU, C3 and Mac keep their
existing scheduling. Local client/daemon must be restarted together for the
expanded internal request header. The receiver rejects malformed, stale,
future and reordered notifications, tolerates5 ms cross-camera SOF skew and
waits at most25 ms intentionally. With no fresh publisher, missing current
upload, bind failure or socket error, DM proceeds independently. No frame is
skipped, no rate is reduced and no driver-monitoring threshold is changed.

Private GPU-context changes, HUD placement switches, shared-memory timing
recorders and runtime mode files are excluded from the clean candidate.
Linux target tests passed28 cases, including actual local IPC and abstract
datagram sockets, same-frame release, previous-frame timeout, absent publisher/
DM, bind failure and malformed timestamps. Desktop tests passed18 with10
platform socket cases skipped. A pre-existing unused `lat_delay` in modeld
remains outside this change; the adapter/DM/new tests pass focused lint.


### Clean phase candidate, full DM and USB display: 600 seconds

The clean production candidate ran from boot157.339 to757.354 seconds with
12,001 driving-model outputs and12,002 DM outputs. Model execution mean/p99/
maximum were36.820/40.744/47.728 ms, with zero above50 ms. DM execution including
phase wait was30.173/37.111/45.551 ms. No model/DM frame gap, invalid monitored
message, pose input/sensor/posenet failure or CAN invalidity occurred. All599
external-status samples were active Cinque v2 on Orin-sm87 TensorRT10.3. The
USB display remained connected. The original DisableDM=2 was restored by the
runner's finally block.

This does not imply an exactly50 ms publication interval: model publication
p99/max were61.487/69.072 ms and DM60.631/67.273 ms, with means49.993 ms. Driver
camera SOF to DM publication mean/p99/max were77.670/87.023/91.963 ms, including
camera delivery, waiting and inference. Camera interval maximum was58.477 ms.
Execution duration, output cadence and source-image age are distinct metrics.

The same boot outside that settled window still failed during DM initialization:
a local send exceeded the150 ms deadline and fell back, with camera/model gaps.
A later DM start logged FunctionFS write ENOMEM, a temporary write_chunk decrease
to256 KiB, and USB frame1533 send83.1/response26.3/local reply0.2 ms, followed by
one skipped model input and invalid odometry. The vendored implementation invokes
that shrink only after ENOMEM and resets its512 KiB quantum on the next message.
This directly associates allocation failure with this new send outlier; it does
not prove the earlier123.2 ms event had the same cause. No transport chunk policy
or kernel memory setting was changed. Full startup console is retained privately.

The final diagnostic-only adjustment retains USB send/response timing in a
finally block if the already-timed-out local client rejects the reply. It does
not change successful inference, transport deadlines or fallback policy. The
local IPC test also verifies a nonzero camera SOF survives request framing.

Evidence, scripts, captures and file hashes are retained privately under
`.analysis/archive/2026-09-25/jetlink-realtime/`; they are not public model files,
Git-tracked captures or a remote backup. This is a parked C4/Jetson improvement,
not hard realtime, loaded-driving, C3 or Mac validation. Startup memory pressure
and rare USB send tails remain unresolved work.


Runtime commit `4c5be00b8d` was pushed only to `carrot-jetlink`, installed by
verified fast-forward on C4 and rebooted. Linux target tests again passed28.
The restored original DisableDM=2 observation ran90.002 seconds with1,800
models: mean/p99/max36.682/40.237/46.810 ms, no >50 ms execution, no frame gaps
or monitored pose/CAN invalidity, and90/90 external-active samples. DM was
intentionally absent in this restoration check; full-DM evidence is the600 s
run above. Live ClusterHudConnected was true. The tracked C4 tree was clean;
model core7 FIFO54, USB owner/reader little cores FIFO1, HUD core7 normal/nice19
and native GPU priority12 were verified. Original VM ratios20/10 and
min_free_kbytes7423 were restored; no private GPU/phase trial module remains.
Jetson server, HUD, Xorg and performance services were all active on unchanged
compatible host source f5749653. No new host USB protocol or install is needed.

Full cereal decoding of the pre-final boot additionally locates the two startup
incidents at boot110.038 and153.048 seconds, before the157.339 s clean benchmark.
The first included model execution844.926 ms on timeout/fallback and camera
intervals up to500.974 ms; the second included116.034 ms execution, one skipped
input and one invalid pose message. Do not attribute every startup failure to
USB allocation or omit these failures when assessing ignition-on reliability.

The final boot console also logged a later frame2413 beyond the90 s restoration
window: warp4.24 + local roundtrip45.52 + parse0.66 ms (about50.42 ms), with
server GPU22.28/queue1.43/total23.92 ms. Thus even the restored-DM-off session
cannot be described as universally below50 ms. The bounded diagnostic remains
enabled to preserve evidence of occasional communication/scheduling tails.


## Navigation video and vehicle system display (2026-09-25)

A live12.040 s `carrotNaviMedia` sample contained116 events and1,678,140 payload
bytes (about139 kB/s):98 video access units, five codec configurations, nine
images and four removals. The map was960x540; keyframes reached249,715 bytes.
This is low average bandwidth but exceeds the96 KiB HUD snapshot limit in one
burst. Guidance capnp was already forwarded; the media service was not.

The negotiated `carrot_navi_v1` extension fragments complete capnp media events
into at most32 KiB payloads plus a24-byte header. A separate low-priority C4
publisher reads non-conflated media, bounds event size to1 MiB and rejects input
over1 s old. The USB owner sends at most one fragment per inference window.
The existing snapshot bound and model150 ms failure deadline stay unchanged.
Partial, out-of-order, oversized or more-than2 s fragment assemblies are rejected;
a missed event or video sequence makes the receiver wait for an H.264 keyframe.
The host uses nonblocking local datagrams, so an absent/slow renderer cannot
block inference. The existing NaviIpcMediaSource decodes with PyAV17.1.0, using
its existing bounded decode queue and stale-map handling. No model input,
recurrent state, navigation speed-selection or vehicle-control policy changes.

Upstream host Session performs inference synchronously and did not post the next
USB read during inference. Merely moving the C4 display send earlier could wait
for that inference to finish. The host now has a bounded two-message read-ahead
queue: it owns copies of borrowed transport payloads before the next recv can
overwrite them, while the original inference thread retains model execution.
The C4 sends HUD and one map fragment after infer_begin, during Jetson inference,
then consumes infer_end. Older peers without the capability retain their previous
post-inference HUD scheduling. This overlaps transfer and computation; it does
not make a hard USB completion or scheduling deadline guarantee.

System panels previously read the renderer host's `/proc`, despite receiving C4
`deviceState`. The host-only statistics adapter now takes all CPU cores (including
0% idle), memory percentage and disk percentage from valid/live vehicle capnp.
Missing/stale input shows unavailable rather than host numbers. Memory byte totals
remain unavailable because deviceState does not carry them. The small footer uses
the vehicle's average CPU percentage; it no longer describes Jetson render-process
CPU. The external compute badge moves19 design pixels right, one font-size width.
Local C3/C4 renderers otherwise keep their existing statistics samplers.

Actual960x540 map imagery and turn graphics were verified through the live USB
renderer, and the system panel showed eight C4 cores. Private one-shot rendering
hooks were removed before timing measurement. The saved screen mode was2 (system
details); mode0 automatically shows trip summary while parked in P, so mode6
(existing navigation screen) was used for the map test. The original mode2 and
DisableDM=2 are restored after the test. No setting definition/default was changed.

Unit checks exercise large-keyframe reassembly, missing/reordered/stale/oversized
fragments, keyframe recovery, vehicle-only CPU data including idle cores, invalid
statistics, and read-ahead buffer ownership/bounded shutdown. Live timing and
frame-cadence results follow below; read-ahead transfer adds a model-payload copy
on Jetson and must be evaluated with actual video/DM load.


Settled live-video timing, each120 seconds and2,400 model outputs:

| Load | Model mean / p99 / max | >50 ms | DM mean / max |
| --- | --- | ---: | --- |
| Map + HUD, original DisableDM=2 |36.954 /40.564 /46.953 ms|0|off|
| Map + HUD + full DM |36.912 /40.325 /45.119 ms|0|30.259 /41.770 ms|

Both windows had no model/DM frame skips or monitored pose/CAN invalidity.
Model publication interval maxima were67.419/64.548 ms, so zero execution
misses must not be described as an exact50 ms cadence. DM camera-to-publication
mean/max were78.601/89.526 ms. All120 external-active samples per arm were true.
The live stream rather than the saved video sample was used in these trials.

Outside the settled windows, enabling DM logged frame12923 warp4.47 + roundtrip
110.63 + parse0.68 ms, one skipped input and invalid camera odometry. Jetson GPU
was19.63 ms and response write14.9 ms; the additional wait is not proven to be
GPU compute or fixed by the new read-ahead path. Earlier startup limitations
therefore still apply. No inference deadline/validity policy was relaxed.

A330-s display observation had325 fresh status samples,327 with a decoded map,
and312 distinct map sequences. Eight C4 CPU cores were present in323 samples.
A six-second renderer restart occurred around DM initialization because missing
snapshots incorrectly returned IsOnroad=false. The final DisplayParams fix retains
last-known configuration on snapshot absence; only a fresh C4 offroad snapshot
turns it off. Vehicle SubMaster alive/valid flags still clear immediately when
snapshots expire, and CPU statistics remain unavailable during actual data loss.
A focused test verifies both transient-loss retention and a genuine offroad update.
The final metadata-hold fix does not change model or media transfer scheduling.

Map decode ages occasionally reached1.2 s during steady observation and2.512 s
later; no configured map-stalled flag occurred. These values measure time since
host decoding, not end-to-end phone capture latency. This is proof of live video
operation with bounded recovery, not a guarantee of uninterrupted frame delivery
or a demonstrated maximum phone-to-display delay. Sustained driving, reconnects
under load, C3 and Mac remain unvalidated. Display captures and console evidence
are kept privately in `.analysis/archive/2026-09-25/jetlink-hud/`.
