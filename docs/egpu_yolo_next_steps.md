# YOLO continuation plan

Continue on the owner-requested `carrot-egpu-yolo2`, branched from
`carrot-egpu-yolo` at `1310ed43fe`. This remains a separate feature experiment
based on `carrot-cinque-terre`. Do not activate these features on the three
maintained model branches. The owner approved the following five stages.
The present outputs are observational; automatic vehicle control is not
validated or enabled by this plan.

## Current baseline

### Resident budget starvation and camera boxes (2026-09-09)

Analyzed all 13 locally retained rlog segments of `00000efa--3bcb5439f9`
on the owner's vehicle, emitting only YOLO timing/state and model timing
aggregates. YOLO ran 4,018 times from boot time 27362.227 until about 27781
(roughly seven minutes). At segment 7, its required reservation rose from
7.470 to 10.194 ms; subsequent free slots were typically 8–9 ms. The run
counter remained 4,018 through segment 12 at 28087.218. Overruns stayed zero.
The supervisor repeatedly retried after ten seconds without a result, eventually
waiting 30 seconds between attempts, but the lifetime maximum reservation never
decreased. This is direct evidence of admission starvation, not a GPU execution
error. It does not isolate what delayed that individual host/GPU transaction or
establish a causal relationship with the external display's USB hub.

The resident runtime now uses a 30-second rolling maximum above the unchanged
artifact/startup measurement floor. Completed, within-deadline measurements can
expire even when no new work is admitted, breaking the circular wait. Lowering
the reservation starts five revalidation runs limited to 1 Hz. The 20% margin,
1 ms guard, stable primary cadence, next-camera priority and GPU-error latch
remain mandatory. A real deadline miss permanently raises this owner's floor;
it is not aged away. A monotonic maximum queue avoids scanning the sample window
on each primary frame. No artifact/kernel/input transfer changes or recompilation
are needed. Recovery history now includes the YOLO state, run/overrun counters,
required reservation and available time at the pause.

The cluster's live camera background draws the existing calibrated four-corner
`cameraPoints`, object names and confidence. Projection uses exactly the video's
crop, zoom and panel offset. Boxes require fresh valid detections, the same road
or wide camera, and camera/result EOF timestamps within 200 ms. Pauses, stale
results, mismatched cameras and missing coordinates clear boxes. Road detections
are not projected onto wide images or a distance-based synthetic scene. The
owner's existing camera-view mode is 2 (road), so no setting change is necessary.
The status panel distinguishes budget wait, camera priority and stabilization.
This remains display-only and exclusive to carrot-egpu-yolo2; no radar/lead or
replay-service code changed.

Validation before deployment: 173 focused timing, automatic recovery, decoder,
cluster and camera tests passed; three unchanged native-messaging tests skipped
on Windows. Regression coverage reproduces the non-overrun starvation, bounded
revalidation, sustained slow work, real-overrun floor, camera mismatch, stale
frames and calibrated corner projection. Hidden renderer previews confirmed
Korean labels and boxes. These are simulated UI/timing checks, not a new moving
vehicle or long-duration GPU measurement; the vehicle was offroad during work.

### Persistent automatic start and recovery (2026-09-08)

On the next ignition, four unnecessary pauses were traced to onroadEvents
frequency checks. selfdrived publishes that service every second **and on each
event change**, so a burst above 1.2 Hz is valid. The supervisor now excludes only
onroadEvents from average-frequency checks; its alive/valid checks and all other
service, camera and inference guards remain mandatory. This prevents valid event
changes from increasing the recovery backoff to 30 seconds.

The yolo2 cluster now consumes the existing carrotYolo service for a compact
status panel: run/wait/paused/error state, current inference milliseconds, run
counter, and up to three object classes with counts and best confidence. Korean
and English text follow the existing display language. Expired images (over
350 ms), invalid results and paused/error messages clear the object summary;
stopped publishers become stale. This adds neither inference nor camera USB
transfers. The display does not infer distances, alter radar associations, or
feed detections into vehicle control. Only carrot-egpu-yolo2 receives this work;
no shared radar detection/lead code or NAS radar replay dependencies changed.

Validation: 76 cluster display/live tests and 20 supervisor/automatic tests
passed on Windows, with three native messaging cases skipped there. Native
vehicle checks confirmed event frequency bursts are accepted while stale/invalid
events and bad carState frequency remain rejected. An additional cluster
performance suite had 34 passes and one existing fixture failure (its fake Params
lacks get_bool, in unchanged cluster_autorun code). The edited cluster files have
no new Ruff findings relative to HEAD; their 15 existing findings are unchanged.
Hidden-window rendered run/paused previews were inspected in both status colors.

Deployed after fresh Park/standstill/disabled checks and one normal manager
restart. The actual live cluster source produced `YOLO 실행 10.1ms · #597` and
`의자 2 (49%)`; the managed cluster process was running. The following 30-second
sample had 352 YOLO run messages, runs 812 to 1163, median 8.82 ms, maximum
15.34 ms, zero overruns and zero supervisor retries. All three cameras delivered
600 valid messages and primary/camera gaps stayed below 120 ms. Modeld delivered
598 valid results across a 600-frame ID span (two missing results), maximum
execution 37.84 ms and peak drop metric 0.493%; one non-run YOLO status also
occurred. Thus the repeated event-triggered recovery delays are addressed, but
this sample does not establish elimination of every transient input-frame miss.
The supervisor averaged 8.48% of one CPU core.

The first onroad automatic-start check exposed a supervisor sampling bug:
`SubMaster(frequency=20)` sets frequency-check expectations but does not pace
`update(50)`. Independently arriving messages woke the supervisor hundreds of
times per second; healthy 100 Hz carState/selfdriveState/carControl streams then
failed its configured 20 Hz upper bound, leaving prepared YOLO permanently paused.
The supervisor now samples nonblocking on an explicit 50 ms schedule, without
catch-up bursts after slow iterations. Non-conflated camera/model timing guards
remain in place. Status includes the individual failed service checks. A live
read-only 20 Hz probe passed every required check before deployment. Local tests
passed 19 cases (two native messaging cases skipped on Windows); equivalent
healthy-100-Hz and rejected-5-Hz cases passed using the vehicle's actual
FrequencyTracker without pytest. The manager preimports Python daemons, so
restarting only its child still forked the old code. A normal manager restart
was performed after fresh Park/standstill/disabled checks, using the existing
restart helper and CPU0-3 inheritance fix; no standalone YOLO daemon was added.

After normal startup, a 30-second live observation recorded 329 valid YOLO
results, runs 97 to 425, median full pipeline 7.39 ms, maximum 13.98 ms and zero
overruns. Modeld produced 601 valid results, maximum execution 37.43 ms and zero
frame-drop metric. The three camera streams each produced 600 or 601 valid
frames; none of the primary/camera publication gaps exceeded 120 ms. The new
supervisor remained in display with no health failures or retries, averaging
8.67% of one CPU core. This verifies actual automatic preparation/admission
after a normal manager startup on ignition; moving recovery and another full
power cycle remain separate checks.

The owner explicitly requested operation after every ignition cycle and recovery
without pulling over. This supersedes the manual-session-only behavior below.
On this separate experiment, `/data/egpu_yolo/auto_enabled.json` containing
`{"version":1,"enabled":true}` opts the installed, verified native reuse artifact
into automatic operation. It is a device-local experiment activation file, not
a shared vehicle-control setting. The manager owns `egpu_yolo_supervisor`, an
ordinary CPU0-3 process which remains available offroad and restarts if it exits.
No network connection, manual command, four-hour allowance or periodic A/B pause
is required for normal automatic use.

The existing GPU owner prepares YOLO during its normal model startup even when
the supervisor has not yet published a lease. Artifact fingerprint, shape and
serialized rebinding checks still apply. Actual inference requires a fresh,
current-boot, exact-owner lease, fresh onroad data and every existing per-frame
deadline/budget/camera check. Startup uses the normal model lifecycle, not an
additional manager or GPU process. No automatic recovery path restarts modeld,
camerad, controls or the whole manager, nor compiles an artifact on live frames.

Transient timing, message-validity, eGPU-ready and competing-process conditions
revoke execution immediately but leave supervision running. It resumes after
two uninterrupted healthy seconds whether the vehicle is in Park, stopped in
Drive or moving with controls enabled. Repeated interruptions require 5, 10,
then 30 healthy seconds; five minutes of healthy operation resets the backoff.
There is no terminal three-retry limit. A completed YOLO deadline miss can clear
its latch on a new recovery generation, retaining its worst-time reservation
and cumulative overrun count. Failed GPU calls remain quarantined until a
different normal model owner starts; restarting a shared GPU in traffic is not
a recovery technique. A dead CPU-only decoder is restarted independently on
CPU4 with bounded backoff and a nonblocking handoff (CPUs0-3 if offroad power
saving has offlined CPU4); modeld skips optional GPU
work while it is unavailable. Persistent GPU timing that cannot fit the budget
still cannot be admitted merely because recovery was requested.

155 focused tests cover repeated ignition cycles, supervisor-before/after-model
startup ordering, stale-boot and wrong-owner leases, moving/engaged gates,
continuous-health recovery, bounded repeated backoff without permanent exit,
GPU-error quarantine, preserved budget and nonblocking CPU-worker replacement.
Fifteen Web/tracking tests also pass. On-device offroad deployment confirmed
the persistent activation file, manager-owned automatic supervisor and its
CPUs0-3 affinity. A deliberately terminated test CPU decoder was replaced in
2.51 seconds, survived initialization and held no GPU device descriptors; the
offroad CPU4-offline fallback was exercised. Terminating the automatic supervisor
also caused the same manager to replace it, without restarting a driving model
or camera. The final state was automatic/offroad, awaiting ignition. Actual
next-ignition inference and moving recovery are not yet live-validated; neither
these checks nor unit tests prove moving-scene accuracy or every hardware fault.

The previous evening's saved fault window had a 160.123 ms **model publication
gap**, not a 160 ms inference. Frames 6225 to 6228 skipped two model frame IDs;
their model executions were 36.90 and 35.96 ms. All camera frame IDs in that
window were continuous. The last YOLO result before the gap was frame 6223,
5.73 ms full pipeline / about 4.39 ms GPU submission and readback, with no YOLO
overrun. This does not identify why model input/host scheduling stalled. The
old supervisor recovered after 30 seconds, but later exited permanently on a
fresh-state guard change. Matching rlog/procLog is needed to distinguish CPU
scheduling, buffer delivery and other host delays; the NAS did not yet contain
the September 7 return trip at inspection time.

### Explicit road-observation session (2026-09-07 follow-up)

The owner requested viewing and logging YOLO while driving. The supervisor now
accepts `--road-observation` with explicit stationary maintenance authorization.
Preparation and the initial manager restart still require fresh Park,
standstill and disabled controls. Road mode is fixed in the GPU owner's startup
state and must match each fresh lease; editing a stationary lease cannot promote
an existing stationary owner. Road mode cannot attach to an existing owner.

After preparation, a known gear, finite speed, fresh vehicle/control/camera
services and active ignition replace the stationary/disabled restriction.
Engaged driving controls do not by themselves disable observational inference.
The primary frame still must be timely and undropped; pending-camera admission,
measured worst-time reservation, nonblocking CPU delivery, GPU-error/overrun
latches, raw publication-gap supervision and bounded timing recovery remain.
YOLO publishes observational results only. Sharing the GPU can still affect
primary timing, so lack of a direct control consumer is not a no-impact claim.

This is a manual current-ignition session, with a hard maximum of four hours;
ignition off, an expired supervisor lease or a non-recoverable guard stops it.
It does not activate again at boot. Web status labels this mode explicitly.
`carrotYolo`, `liveTracks`, camera metadata and calibration already participate
in route logging; Web-only IDs/candidates can be reconstructed from the raw
timestamped streams rather than being treated as a new control signal. Moving
accuracy and radar-candidate identity remain unvalidated.

### Restart scheduling incident and recovery (2026-09-07)

The first road-mode preparation timed out without a prepared YOLO owner or any
YOLO inference. The owner reported recurring lag and CPU7 saturation. Preserve
the incident separately from the earlier successful stationary trials: road
observation has not yet been established on the vehicle.

Before another restart, live inspection found `isolcpus=6,7`, CPU7 at 100%,
modeld about 60% and plannerd about 18%, plus ordinary micd, hardwared, proclogd
and other daemons running on CPU7 despite permitting all eight CPUs. The tmux
server also occupied CPU7. Its newly created pane can leave ordinary manager
children on an isolated CPU, where automatic load balancing does not rescue
them. The sampled microphone callback had accumulated about 199 seconds of
run-queue wait. Available memory was about 1.4 GB and CPU temperature below
54 C; no second driving GPU owner was found.

At fresh Park/standstill/disabled state, 30 ordinary threads belonging to the
manager and explicitly identified management/audio/logging daemons were moved
to CPUs0-3, recording their original affinities. Modeld, camera and control
processes were not moved or restarted. The owner confirmed recovery. CPU7 fell
to about 80%, soundPressure returned to 10 Hz, and commIssueAvgFreq disappeared.
CPU2 briefly saturated while proclogd caught up (73 messages in 20 seconds
versus its nominal 0.5 Hz); a later sample was about 62%. Its cumulative lag
counted down rather than representing a single multi-minute operation.

A 10.22-second raw pre-change sample contained 205 model publications, no frame
gaps/drops, and maximum model time 38.35 ms. The 20-second post-change sample
contained 400 model publications and no internal frame ID gaps, with maximum
model time 37.95 ms; its decaying drop metric reached 0.312%, so this recovery
must not be reported as entirely drop-free. An earlier conflating observer's
apparent 14.6 Hz reception was not the raw model publication rate.

The experimental supervisor now constrains both itself and the verified tmux
server to ordinary scheduling on CPUs0-3 before stopping a manager. This covers
both existing-server pane creation and new-server startup. Unexpected server
identity, scheduling policy or CPU availability fails before mutation. It does
not alter primary process affinity or loosen runtime timing/admission guards.
Four focused launcher tests pass. After the owner's manual reboot, the corrected
supervised restart succeeded: manager and hardwared inherited CPUs0-3, while
modeld retained CPU7. Road mode reached actual inference (1,106 runs, zero YOLO
overruns, latest pipeline 6.11 ms and primary 35.26 ms / zero reported drops),
with every sampled service passing frequency checks. The Web API was fresh and
the rendered badge subsequently showed 1,414 runs and the road-observation label.
The owner then departed; the vehicle-local supervisor continues without SSH.
The full 30/180/30 trial and subsequent moving results require later log review.
The earlier absent UsbGpuActive flag remains unexplained; it was active after
reboot and was never forced to bypass the supervisor's checks.

Detailed incident evidence and affinity backups remain device-local under
`/data/egpu_yolo/lag-road-preparation`. Only this aggregate summary is committed.

### Stage 2 display prototype (2026-09-07)

The Web backend now assigns session-scoped visual IDs using class-compatible,
mutually unambiguous box overlap. It remembers a missed observation for at most
400 ms, keeps at most 40 tracks, and never draws a remembered box as a new
detection. Geometry/model changes and backwards timestamps reset track state;
IDs are not reused within a backend session. Crossing/occluded-object identity
is not validated; disputed pairs receive new IDs.

The read-only backend also buffers raw `liveTracks`, calibration and camera
metadata. Only explicitly sourced, measured `frontRadar` points can become
association candidates. Unknown sources, SCC and corner radar are excluded.
It aligns message time minus `CarParams.radarDelay` with camera EOF (120 ms
maximum difference), projects a ground footpoint with calibrated camera
intrinsics/extrinsics and the existing provisional 1.52 m longitudinal offset,
and requires mutually unambiguous overlap with the visual box's lower region.
The API carries radar source/ID, raw range, relative speed, alignment age and a
heuristic geometry score. This score is not a probability. Web labels use `R?`
and an explicitly unvalidated-candidate status. Missing/invalid/stale data or
missing calibration never supplies a distance. No control service consumes
these Web-only associations, and no GPU input/output schema changed.

In a 30.06-second stationary capture from IONIQ 5 PE / dongle
`07b62e389ed26c81`, the available raw radar records were exclusively `corner235`
(595 point observations, seven IDs); there were no front-radar points. Replay
produced one bicycle ID across all 451 bicycle observations in 458 valid YOLO
frames, including seven empty frames, and zero radar association candidates.
This establishes the observed stationary identity continuity and source
exclusion, not moving-object matching accuracy. Reproduce with
`python -m openpilot.tools.egpu_yolo.replay_tracking capture.json`; capture
format and output limitations are documented in that tool. Unit cases cover
projection sign, time delay, ambiguity, calibration failure and stale data.

Moving/front-radar validation remains pending; Park testing does not require
changing gear. Before treating candidates as matched identities, validate
per-vehicle radar/camera alignment, object motion, false matches and ID switches
against synchronized video and raw radar. Camera-only depth is not inferred.

### Supervised automatic recovery and remaining CPU cost

The owner requested automatic recovery after transient timing stops. The tracked
`openpilot.tools.egpu_yolo.supervise_reuse` command retains the existing exclusive
lock, expiring lease, single GPU owner, fresh Park/disabled conditions and raw
camera/model timing guards. For a raw publication gap or primary execution/drop
guard only, it first removes the enabled lease and saves the failed window, then
requires 30/60/120 seconds of uninterrupted healthy disabled observation before
retrying. At most three attempts are allowed per 15 minutes. Operator signals,
changed Park/control/owner state, invalid streams, GPU errors, YOLO overruns and
missing results do not auto-retry. Recovery neither restarts the manager nor
recompiles. A manual session is limited to at most one hour and is not enabled
at boot. Web status distinguishes the cooldown from active inference.

The supervisor records the observed guard trigger and correlated camera/model
statistics; it does not claim those observations identify the root cause.
On the first Web-only restart at commit `40be30b941`, the old supervisor stopped
after a 152.090 ms model publication gap, at 18,015 YOLO runs / zero YOLO overruns.
The interrupted window reported nonzero driving drops (maximum 0.493%); primary
and camera PIDs remained unchanged. Web startup is temporally associated, but
causality has not been isolated. This longer display observation must not be
described as zero driving drops. The earlier bounded A/B trials remain separate.

CPU corner projection is now batched for two or more boxes. An alternating
1,000-sample-per-variant CPU4 experiment (ordinary scheduling, nice 10, no GPU
work) measured projection wall-time medians of 0.630 -> 0.335 ms for 10 boxes
and 2.637 -> 0.865 ms for 40 boxes; corresponding thread CPU medians were
0.631 -> 0.339 and 2.533 -> 0.859 ms. Zero/one-box frames retain the original
projection path. This is a saved synthetic-box CPU microbenchmark, not a dense
live-scene result or a reduction in neural-network GPU time. Tests compare the
old/new projection across box counts, perspective transforms and invalid depths.

### Post-reboot stationary validation

The owner reported micd/general lag and suggested reboot. Before reboot the
device load average was 16.24/11.76/12.10 with about 1.1 GB available memory.
After a fresh Park/standstill/disabled check, a device reboot restored
`UsbGpuActive=1`; a 20-second read-only check observed about 20 Hz on the driving
model and all three cameras, no driving drops and no alerts. The owner confirmed
the lag improved. These observations do not establish the original lag's cause.
`ShareData=0` remained unchanged.

The supervisor also now starts new frequency history after deliberately
restarting the manager and records preparation checks every five seconds. A
prior startup timed out without a prepared YOLO owner; retaining the old
manager-stop interval in the new epoch's frequency history was an avoidable
preparation failure mode. The new startup succeeded without changing active
inference admission or camera/model timing limits.

Runtime commit `d170013064` completed another 30/180/30-second stationary trial:

| Metric | Baseline | YOLO enabled | Recovery |
| --- | ---: | ---: | ---: |
| Driving model p50 / p99 / max, ms | 36.18 / 37.73 / 38.42 | 36.04 / 37.53 / 38.67 | 36.11 / 37.93 / 39.14 |
| Maximum driving publication gap, ms | 76.45 | 76.87 | 70.35 |
| Reported driving drops | 0 | 0 | 0 |
| Maximum consecutive observed frame-ID delta, all streams | 1 | 1 | 1 |

The enabled window received 2,248 YOLO results in 180.014 seconds (12.49 Hz),
with zero YOLO overruns. GPU submission plus completion/readback measured
4.336 / 4.663 / 5.008 ms p50/p99/max; full result latency was
6.889 / 14.331 / 34.355 ms, including CPU postprocessing at
0.730 / 5.246 / 27.225 ms. Thus CPU/delivery tail latency is still present.
The enabled observer retained 3,598 messages for each primary/camera stream
(window/drain boundaries), with no internal frame-ID gaps; baseline retained
600 each, recovery 600 each except 599 wide-camera messages. Camera maximum
gaps were below 66 ms during enabled observation. Thermal status stayed green,
with a maximum reported CPU temperature of 53.6 C at the enabled-window end.

An actual Chrome live-camera check showed the bicycle box labelled `#6 bicycle`
and the Korean corner-only radar status, with no distance attached. A separate
30.18-second Web API sample completed 143 requests without errors: 111 distinct
fresh frames, all showing bicycle ID 6, and zero radar candidates. Earlier
observations in the session had six created IDs; identity may expire after a
longer miss and is not claimed continuous across the entire trial. The CPU
worker ran ordinary scheduling on CPU4 with no GPU device descriptors.

These are sequential stationary observations with a browser video client
started during the enabled window, not a controlled old/new optimization A/B.
The dense-box benefit is established only by the separate CPU microbenchmark.
Automatic timing-recovery sequencing, backoff, exhaustion and refusal on changed
guards passed focused tests; the successful live trial had no fault triggering
automatic recovery. Actual live fault recovery and moving front-radar matching
remain unvalidated. The supervisor proceeded to bounded continuous display.
Retained local evidence: `.cache/egpu-yolo/live-stage2/` and
`.cache/egpu-yolo/stage2_stationary_capture.json`.

The follow-up has completed the initial 640 x 384 saved-input eGPU measurement
and independent numerical check. See [eGPU timing](egpu_yolo2_timing.md).
Its 5,000 resident-input runs took 5.45 ms p50 / 6.24 ms p99 / 7.39 ms maximum,
but uploading the original padded NV12 frame at 5 Hz took the total to
31.73 / 33.67 / 33.95 ms. The original input upload alone took 25.99 ms median.
This does not establish admission alongside live driving. The owner then
prioritized reusing the already-resident driving image for minimum latency,
explicitly accepting bicycle misses. Additional full-camera upload and the
higher-resolution input are no longer the immediate implementation target.
The same-frame 512 x 256 comparison took 3.64 ms resident-input median and
29.74 ms with original-buffer upload. Both trials passed serialization and
independent ONNX checks. Each has 5,000 resident runs and 300 uploads at 5 Hz;
neither measures live driving concurrency or a controlled thermal A/B.

The native-buffer follow-up now completes a stationary 30/180/30-second
baseline/enabled/recovery comparison: 703 live results at 3.91 Hz, zero YOLO
admission overruns and zero driving frame drops. Submission plus GPU completion
and readback measured 4.84 ms p50 / 5.24 ms p99 / 5.59 ms maximum. CPU NMS and
projection run in a separate ordinary-scheduling CPU worker; full result latency
was 9.28 / 19.69 / 25.70 ms. It reads the existing 512 x 256 driving queue with no
second camera upload. Changed-input/allocation/serialized-replay checks prevent
reuse of the prototype's stale capture-time buffer. See the timing report for
excluded attempts, primary comparisons, retained samples and validation limits.

The owner then requested considering every primary frame. Removing only the
native runtime's 200 ms interval produced 2,091 results in 180 seconds (11.62 Hz)
with zero YOLO overruns or reported driving drops in another 30/180/30-second
stationary comparison. Retained GPU work measured 4.80 ms p50 / 5.17 ms p99 /
5.69 ms maximum; full result latency was 8.73 / 21.18 / 28.70 ms. Pending-camera,
deadline, Park and overrun guards remain active, so this is per-frame admission
with conditional execution, not a guaranteed 20 Hz output. Continuous supervised
stationary display now uses this mode. The focused suite passes 114 tests.

The subsequent optimization caches validated input metadata for unchanged YOLO
replays and halves result transport using FP16, with FP32 NMS in the separate CPU
worker. It passed another 30/180/30-second stationary comparison: retained GPU
cost fell from 4.80 to 4.35 ms median and from 5.69 to 4.99 ms maximum. The median
remaining budget after GPU work increased from 4.78 to 5.20 ms on admitted
frames. No guard was relaxed; zero YOLO overruns and driving drops were reported.
Result rate was 11.32 Hz, so this trial did not improve rate over 11.62 Hz.
CPU delivery/postprocessing tails remain unresolved (70.18 ms maximum full
result latency). See the timing report for details. Continuous stationary
display now uses the verified native FP16 artifact; 117 focused tests pass.

The last internal-GPU continuous-display attempt had already stopped before
the follow-up: 421 results, then a 150.47 ms model-publication gap with frame-ID
delta 3 on the non-conflating observer. Its root cause remains unisolated.
The QCOM display worker and its automatic activation marker remain absent.
Native-buffer work now uses the same modeld USB GPU owner under a short manual
session lease. Consult the latest timing report and device-local
`/data/egpu_yolo/live_reuse_status.json` before assuming any session is active.

See `egpu_yolo_experiment.md` for measurements and numerical checks.
The internal-GPU artifact uses YOLOv8n COCO FP32 at 512 x 256, confidence
0.35, direct NV12 preprocessing, Winograd, pinned Mesa IR3 and nine cooperative
batches. Saved input takes about 69 ms; live publication measured 120 ms
median at 3.85 Hz. Its 180-second trial published 688 results without reported
driving frame drops, but increased driving tail latency. The existing 350 ms
display expiry can leave gaps between boxes.

In that earlier internal-GPU experiment the owner requested continuous stationary display instead of a
three-minute trial. Device-local `qcom_display_coordinator.py` performs the
same guarded camera-stop/prewarm/normal-restore sequence, keeps the warmed
worker alive, and runs `qcom_display_observer.py`. This is a manual session,
not reboot-persistent activation. Fresh Park, controls disabled, active eGPU,
existing DM-disabled configuration and healthy camera/model services are
required. The observer stops the worker on changed conditions, camera/model
gaps above 120 ms, primary execution above 60 ms, frame drops above 1%, or
missing YOLO results for 10 seconds. It retains bounded current status rather
than accumulating an unlimited in-memory trial report. Inspect
`/data/egpu_yolo/qcom_display_status.json` and
`/data/egpu_yolo/qcom_display_coordinator.log` before any new GPU work.

Do not start a second worker or compile while cameras are live. If an earlier
warm worker is still active, stop its supervisor cleanly before new maintenance.
Its PID file is `/data/egpu_yolo/qcom_display_coordinator.pid`; verify the process
command before signaling it. Automatic `qcom_enabled`
activation remains absent, and cold `qcom_yolod` launches are rejected.

Continuous-display retries at 4 Hz and 2 Hz stopped after 175 and 94 runs
respectively on a sampled model-publication gap above 120 ms. A subsequent
30-second baseline received all 600 publications with a 77.57 ms maximum gap.
The observer now checks a non-conflating model subscription, preserving the
120 ms limit, to distinguish actual publication gaps from missed intermediate
messages in a sampled view. The original outlier cause is not established.
The display retry uses a 500 ms interval without changing input resolution.

### Bicycle miss: resolution evidence

A saved current road frame visibly contains parked bicycles. In the actual
512 x 256 input, the bicycle candidate occupies approximately 38 x 22 pixels.
Independent ONNX Runtime inference on matching NV12 preprocessing produced a
maximum bicycle score of 0.0276, below the 0.35 display threshold. A plant class
won at that anchor; lowering the display threshold is not a reliable fix.

The same YOLOv8n weights were also tested on the saved RGB image using standard
Ultralytics CPU preprocessing. These are input dimensions, width x height:

| Input | Best bicycle confidence |
| --- | ---: |
| 512 x 256 | 0.0215 |
| 640 x 384 | 0.5764 |
| 896 x 512 | 0.8532 |
| 1344 x 768 | 0.8927 |
| Manually selected bicycle-region crop, 384 x 256 | 0.8861 |

The 640 x 384 result also contained a motorcycle classification at 0.4696
over the same bicycles. This is one-scene evidence, not a general accuracy
benchmark. The crop was selected after looking at the image; it does not
validate automatic region selection. CPU preprocessing differs slightly from
the live NV12 kernel, explaining the different low-resolution scores. The later
640 x 384 eGPU benchmark uses matching full-camera NV12 preprocessing and
does not use the historical driving-warp input.
The workstation now has a static 640 x 384 ONNX candidate (12,756,343 bytes),
SHA-256 `e02d75ddde2a1792e0275af57e3c23dd8dc307aa64a6c2f86c8f1889618d7f76`.
Its three random-input comparisons against PyTorch passed with maximum
absolute errors below 0.00087. The verified file was copied into a separate
vehicle benchmark directory. Its prospective NAS URL is still unpublished,
and neither live runtime has been switched to the 640 x 384 artifact.

## 1. Minimize eGPU latency with the existing native buffer

The latest instruction is to reuse the native 512 x 256 driving `img_q` and
minimize execution time, even if bicycle detection fails. Do not add a second
full-camera USB transfer to recover those detections. A fresh same-frame CPU
reconstruction using current driving calibration scored the bicycle 0.841 in
the driving warp, 0.024 in full-view 512 x 256 and 0.638 in full-view 640 x 384.
The narrower driving field of view retains more object pixels in this scene;
these are CPU/ONNX comparisons, not reads of the live eGPU queue. Reusing the
queue uses current-buffer ownership, timing guards and its own sustained
primary-latency comparison in the new manually leased runtime.
The initial stationary comparison is complete; longer sessions, USB fault cases
and moving-scene accuracy are still separate validation work. Preserve the
CPU-only postprocessing boundary and current-frame ownership when extending it.

Use 512 x 256 as the timing baseline, and evaluate 640 x 384 or a justified
region strategy as an accuracy candidate given the bicycle miss. Do not reduce
resolution further. The historical shared-eGPU path measured 5.27 ms
median and 5.73 ms maximum in a short trial, but later overran at 24.887 ms.
That automatic path remains retired. The new native color kernel and leased
same-owner runtime are described in `egpu_yolo2_timing.md`. A realized input
slice initially retained saved pixels in the HCQ graph; direct whole-queue
binding and changed-image/allocation/serialized-replay checks are now required.
Separate GPU kernels, queue waits,
USB readback, host scheduling, NMS and full camera-to-publication age. Its
already-resident driving-warp input differs from the internal GPU's full-camera
letterbox; do not treat equal tensor dimensions as equal field of view.

An 8 ms target is not an enforceable interruption of an in-flight kernel.
Evaluate bounded submissions and current primary-frame admission, then compare
baseline/enabled sustained tail latency under the same scene and thermal state.
Keep measured dropped-frame, timestamp and fault evidence. Reduce frequency
before reducing image resolution. Region crops require their own timing budget.

## 2. Track vehicles and associate radar objects for Web display

First establish which front/corner tracks this vehicle actually publishes;
schema fields and parser support do not establish valid live data. Calibrate
camera/radar geometry and align capture timestamps before association. Maintain
track IDs, assignment uncertainty and unmatched visual objects. Use raw tracks
where available rather than only already-selected leads, and avoid circular
validation against existing vision-fused lead outputs.

Display ID, class/confidence, matched radar ID, range, relative speed and match
quality. Log identity switches, association errors, misses and result age.
No longitudinal or lateral control consumer is introduced at this stage.

## 3. Validate cut-in detection

Combine temporal vehicle tracks, radar lateral motion and the ego path/lane
geometry. Distinguish adjacent vehicles, approaching boundary crossings and
vehicles already occupying the path. Compare with existing `leadsCutIn` and
`leadCutInRisk` without replacing them. Curves, ego lane changes, occlusion and
track handoff need separate examples. Score false alarms and missed cut-ins,
as well as advance warning time, using independently reviewed footage.

## 4. Extend to pedestrians and lateral hazards

Retain visual detections that have no radar match. COCO is not a general
unknown-obstacle detector. Confirm available BSM/corner data before adding it.
Forward/wide cameras do not cover the entire side/rear region; an unobserved
region must not be classified as clear. Measure small-object, occlusion and
night performance. Full-camera and wide-camera mappings must be explicit.

## 5. Add dedicated traffic-light and stop-line perception

COCO has a traffic-light location class, not red/green/arrow state or a stop-line
class. Evaluate dedicated training and original-resolution crops for signal
state, stop-line geometry, and association to the ego lane/turn. Stop signs are
not stop lines. Require labeled local examples and temporal validation before
considering any stopping behavior. Prepare and test model changes as experiment
artifacts without overwriting the driving model or its NAS manifest.

## Validation and handoff

The baseline code passed 72 focused YOLO tests and Ruff. Later changes require
their own relevant checks. Browser visual inspection was unavailable when the
UI tool could not verify its URL; Web API delivery was verified. Recheck actual
boxes when a browser surface is available. Stationary timing tests are not
moving-vehicle control validation. Carry forward measured limitations rather
than reporting the five stages complete after a prototype or successful demo.
