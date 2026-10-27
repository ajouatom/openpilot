# Jetson frame skips and locationd input validity

Scope: carrot-nav only. This follows the user's report after the October 14
drive and request to improve and apply the result. Raw recordings stay private.

The latest examined drive has four post-join locationdTemporaryError events.
locationd's PID stays unchanged. All four missing model frames exist in both
camera streams, with about 0.013--0.019 ms SOF skew. Before each skip, model
output age has accumulated to 117--121 ms after camera EOF. The skipped frame
marks cameraOdometry invalid, livePose.inputsOK becomes false, and selfdrived
raises the temporary error. Do not relax that validity check to hide latency.

An unchanged 60-second parked baseline after downloads finished has 1,200
camera/DM frames, 1,198 valid model outputs, two frame-ID skips and two invalid
cameraOdometry samples. Inference including comma preparation/transport is
45.258 ms mean, 50.020 ms p99 and 60.526 ms maximum; the entire model loop
averages 50.015 ms. Host GPU/total means are 25.620/28.943 ms. MAXN_SUPER and
fixed CPU/GPU/EMC clocks are already active. Comma aggregate CPU is 72.5%,
Jetson 11.4%; that does not prove CPU load alone causes every delay.

## Changes

Stateful output validation can use a fused native IEEE exponent check instead
of materializing multiple NumPy scratch passes. All output/state shape, dtype,
finiteness and reset checks remain. A private temporary build happens once
before processing frames; unavailable compiler or failed startup self-check
retains NumPy. There is no model/engine, recurrent state or pixel change.

For a Jetson with the receiver-pump capability, move one additional ready media
fragment into the early GPU window and remove post-reply media bursts. At most
three display messages (HUD, normal fragment, extra fragment) fit the existing
two-message host queue plus reader-owned message. Admit the extra only within
4 ms after request submission; keep the transport watchdog and H.264 ordering.
Other peers retain their previous scheduling. Update ClusterHudConnected only
on transitions, avoiding repeated identical asynchronous Params writes.

## Initial verification

Native ARM A/B/A on 1,572,864 finite FP16 values measures mean 0.723 / 0.239 /
0.729 ms (NumPy/native/NumPy); this is a validation microbenchmark, not full
inference. Every one of 65,536 FP16 bit patterns matches NumPy, with NaN and
both infinities checked at every vector-tail position for both FP16 and FP32.

49 focused Windows tests pass; 26 native/Linux-specific tests skip. A separate
socket suite cannot run on Windows because sendmsg is unavailable. Final Linux
CI passes 789 integration checks (11 skipped), including native validation and
socket paths; eGPU compatibility also passes. One legacy test that inspected
NumPy scratch storage now explicitly selects that fallback; the native path has
its own exhaustive binary16 test.

## Parked installation and first measurement

After a fresh valid/alive Park, standstill and controls-disabled guard, the
vehicle fast-forwarded from 1b42724c42 to cb9bd2a135 with a recorded previous
revision. Tracked files were clean and untracked build products did not collide.
Startup rebuilt changed native targets. Jetson activated the signed source
989386eb85, with the same RL Nav 80eedcdd ONNX and cached TensorRT engine.
The production server's /proc maps confirm the new finite.so is loaded;
compiling it in a separate probe alone would not establish live activation.

The first 120-second sample after joining the external model has 2,401 valid
model and cameraOdometry messages, 2,400 consecutive frame increments, zero
invalid livePose inputs, no alerts and no modeld/jetlinkd/locationd restart.
DM produces 2,400 valid messages. Inference is 43.982 ms mean, 47.463 ms p99
and 49.964 ms maximum. Publication interval is 49.991 ms mean, 55.532 ms p99
and 60.024 ms maximum; this is not a claim that every output arrives within
50 ms. Host GPU/total means are 25.432/27.931 ms. Post-reply display work
falls from 0.626 ms mean / 11.772 ms maximum to 0.007 / 0.145 ms.

A separate 10-second CPU sample is 69.8% aggregate on comma, with busiest core
82.1%; Jetson is 10.5%, busiest cores 24.6%. Short before/after samples and a
reboot do not isolate the patch as the cause of every timing difference.
The first sample has no external route (mapsd reason=route), so it does not
validate active map rendering. 40 ms has not been achieved; parked evidence
is not closed-loop or road stability validation.

The additional 300-second sample has 5,999 valid model/odometry messages,
all 5,998 successive frame IDs consecutive, 6,000 healthy livePose samples
and 6,000 valid DM messages, with no alerts or process restart. Inference is
44.219 ms mean / 48.324 ms p99 / 56.398 ms maximum. Publication interval is
50.003 ms mean / 56.819 ms p99 / 63.696 ms maximum. Thus occasional >50 ms
work still occurs even though this window has no skipped camera frame.

This longer window includes navigation reconnection: 110 one-second samples
have no route, 147 lack fresh position, one rejects a slow render, and the last
41 report ready. The receiver later shows fresh nonzero vehicle coordinates
and increasing source sequence at approximately 1 Hz. No GPS freshness rule
was relaxed or old position timestamp renewed to obtain ready status. The
mixed window cannot be represented as five minutes of active map rendering.

## Navigation-connected check and remaining limits

The final separate 120-second navigation-connected sample has 2,401 valid model
and odometry outputs, all 2,400 successive IDs consecutive, 2,400 healthy pose
and DM samples, no alerts and the same process PIDs. Inference measures
44.275 ms mean / 48.482 ms p99 / 52.171 ms maximum. Publication interval is
50.000 ms mean / 57.062 ms p99 / 65.376 ms maximum. Host GPU/total means are
25.425/27.914 ms, and USB request send averages 5.320 ms. Model throughput
stays about 20 Hz in these parked samples; neither 40 ms nor a hard per-frame
50 ms deadline has been established.

Map status is ready in 94 of 120 one-second samples; 12 reject stale/missing
position and 14 reject route distance over 50 m. This is not uninterrupted
navigation availability. A separate read-only MapInput consumer confirms fresh
nonzero model-map pixels are delivered with the normal route/time checks; its
poll counts include repeated frames and are not a rendering-frequency metric.
Final Jetson HUD status shows a decoded 960x540 map, age 331 ms, not stalled,
zero decode drops, and 4,125 received items. The comma media publisher has
3,938 sent, 35 stale rejections and zero abandoned events since startup.
Its 488 ms historical maximum is the low-priority producer's backpressure,
not inference USB-write time. Physical panel appearance was not inspected.

Across the three separate windows (nine measured minutes, not one continuous
nine-minute trace), 10,801 model results are valid and every within-window
successive frame ID is consecutive. No locationd error was observed. The host
service remains active on source 989386eb85 with no post-activation restart.
Code and signed release are published only on carrot-nav. No road-response
validation, guaranteed error elimination, GPS transport repair, or physical
eGPU USB repair is implied. Remaining work would need to target measured warp/
roundtrip spikes and external position continuity without weakening validity.

## Follow-up: sender identity, phone positioning and USB completion

The PC-connected USB phone was positively matched to the active navigation
sender by reading its wlan0 address through ADB after reconnection. The phone's
TMap version also matches the receiver's app-version field. Device identifiers,
addresses, coordinates, Android dumps and raw kernel traces remain private.
The receiver has one navigation control connection. This was not observation
of a different phone.

Independent navigation streams explain why the displayed map can remain live
while model-map position expires. During a 30-second sample, vehicle sequences
318--321 arrive without sequence gaps at roughly eight-second intervals while
map video continues near 10 Hz. Later the vehicle sequence stays at 329 while
video and other navigation items continue. Bringing the map foreground changes
app_status and app_foreground on that same session, but does not restore vehicle
updates in the subsequent 30-second sample (position age reaches 498 seconds).
Source timestamps also stop advancing, so this is upstream of vehicle receiver
processing, not simply delayed consumption of a fresh packet stream. Phone and
vehicle clocks are not synchronized; their timestamp difference is not a
measured one-way network latency.

Direct Android inspection finds TMap requesting GPS at one-second intervals
with fine-location permission granted. In the final snapshot the GPS provider's
last fix is 1,177.8 seconds old, with reported horizontal accuracy 43.6 m and
three satellites in that fix. Network and fused locations are much newer
(9.2 and 6.3 seconds). The navigation vehicle item is 434.9 seconds old although
map video remains fresh. This confirms stale phone GPS alongside stopped
position publication; it does not yet prove which phone location callback the
navigation bridge consumes, or distinguish callback filtering from deliberate
unchanged-position suppression. Foreground status alone is not the cause.
Fresh vehicle GNSS now supplies the existing model-map fallback; no stale
phone timestamp was renewed and no validity/freshness gate was relaxed.

USB traces separate transfer from completion processing. A first 45-second
comma trace has 901 inference writes of 475,136 bytes: syscall mean 3.413 ms,
p99 7.448 ms, maximum 15.697 ms. Most of the maximum is between gadget queue
and giveback, which must not be described as physical wire time alone.
A second simultaneous host/comma capture finds 900 host request groups:
1,024-byte header, 262,144-byte block and 211,968-byte tail. Header-complete to
full payload averages 2.246 ms, p99 2.551 ms, maximum 2.905 ms; host read
resubmission gaps average about 0.2 ms. Approximate cross-clock sequence
alignment suggests delayed comma completion after host reception, but lacks
shared-clock or explicit packet-ID proof.

A separate comma-only trace establishes this delay without cross-clock
assumptions. Of 887 fully retained requests, write time averages 3.409 ms,
p99 7.521 ms, maximum 13.724 ms. In the worst request, dwc3_bh_work is queued
2.912 ms after syscall entry, waits 10.685 ms before starting, gives back the
request 0.015 ms later, and writev returns another 0.112 ms later. This directly
locates a large spike in scheduling of the USB completion worker, rather than
Jetson inference or payload read speed. It does not establish why that worker
waits, nor imply every spike has the same cause. The initial CPU0 trace buffer
wrapped by 648 entries; analysis excludes that prefix and uses the common
retained interval only. The worst request lies within the retained interval.

All captures used separate trace instances with continuous Park/inactive
checks and removed those instances afterward. No production process restart,
USB reset, kernel scheduling/affinity change, model change or deployment was
performed. Further optimization should measure worker wakeup/run-queue delay
and interrupt placement before changing scheduling. An asynchronous sender
would also require bounded ownership and completion synchronization; an early
return must never permit reuse of a buffer still owned by USB. These findings
do not establish 40 ms execution or road stability.

## Parked position continuity and scoped Jetson placement

Navigation now retains an in-process position anchor only after at least one
second of continuously observed Park, standstill and speed below 0.01 m/s,
with a fresh position observed during that parked period and position speed
below 0.1 m/s. Fresh carState receipt/validity/liveness remains mandatory.
Leaving Park, rolling, missing carState, a polling gap or process restart
clears the anchor. Old startup coordinates are never revived. The anchor
retains its original position timestamp and uses zero extrapolation speed;
no GPS timestamp is renewed. mapsd and Mapbox navd use the same rule, so
Mapbox instructions do not disappear solely because a parked fix expires.
Route/instruction/renderer validity rules and moving GPS freshness stay intact.
This supports parked map continuity; it does not repair the phone GPS provider
or introduce unbounded moving dead reckoning. Position tests include a
20-minute synthetic parked outage, motion, stale state, restart and reacquisition.

A reversible 45-second IRQ-placement comparison did not remove completion
worker delay: moving only USB IRQ 994 to core4 gives request-write p99 7.382 ms
versus 8.913 ms in the preceding baseline, but worker wait still reaches
8.013 ms and model mean is 44.463 versus 44.596 ms. Neither window has invalid
model/pose/DM output or skipped frame. Short runs do not establish a reliable
improvement; IRQ affinity was restored and is not changed in production.
The AGNOS driver queues completion onto an ordered high-priority workqueue;
changing interrupt affinity does not pin that worker. See the upstream
[gadget handler](https://github.com/commaai/agnos-kernel-sdm845/blob/master/drivers/usb/dwc3/gadget.c)
and [queue allocation](https://github.com/commaai/agnos-kernel-sdm845/blob/master/drivers/usb/dwc3/core.c).

The next measured candidate moves only jetlinkd's userspace framing/IPC/reader
threads to core4 during an established onroad Jetson session. Priority remains
FIFO1 for the existing realtime threads and normal for the watchdog. Discovery,
offroad and other peer types retain cores0--3; camera6/model/DM7 and shared
kernel workers are untouched. This requires native measurement before a
performance improvement can be claimed.

The core4-only userspace candidate **failed** native validation and is reverted:
120 seconds produce mean 46.649 ms / p99 53.080 / maximum 56.811 ms, with 52
skipped camera frames, 52 invalid cameraOdometry samples and 127 livePose
inputsOK=false samples. DM stays valid, no process restarts or dashboard alert
occur. jetlinkd CPU work decreases, but scheduler runqueue wait reaches
108.1 ms/s. Overall comma CPU is 76.3%; little cores are 81.7--88.9% busy.
Jetson is 10.8% overall, busiest core 24.8%. Do not retain core4 pinning or call
this an improvement. Keep the established cores0--3 policy and retest.

The parked-position change works independently: a native read-only shadow
test preserves the original anchor timestamp through a simulated outage, and
actual mapsd logs also report parked_position_hold=true while generating maps
near 5 Hz, with mean CPU time around 7 ms per render. A strengthened regression
also requires the ORIGINAL fix to have occurred after parking began; a freshly
extrapolated coordinate cannot seed a parked anchor from a pre-parking fix.

Route-distance rejection also now rolls back the tentative route progress.
Previously mapsd called the monotonic RouteTracker.project before checking
the existing 50 m distance gate: an off-route GPS jump could advance progress,
then be rejected for rendering, yet prevent a later correct fix from recovering.
RouteTracker.update_pose commits only an accepted projected/extrapolated pose;
normal accepted projections remain identical. Regression tests cover invalid
first fixes, a far-off forward jump followed by recovery, and extrapolation
rejection. The 50 m threshold is unchanged. This fixes a state-poisoning path,
not real GPS error: a live independent fresh tracker also measured a 99.5 m
separation from the current sender position to the current route's start, which
must remain rejected even without poisoned progress.

The live, reversible userspace-placement A/B/A also rejects core5: 40 seconds
on cores0--3 average 44.504 ms without skips; core5 averages 51.722 ms with 89
skips; returning to cores0--3 averages 44.267 ms with no skips (two initial pose
inputsOK=false samples carry over from the transition). Original affinities
are restored. No big-core userspace pin is retained.

A default-OFF diagnostic candidate instead pipelines four 16 KB FunctionFS
reads with Linux AIO. This preserves the small per-read kernel allocation and
wire framing while letting the controller receive several response pieces
without waiting for a new userspace read after each completion. C-owned buffers
are copied only after their matching completion and remain allocated through
io_destroy; sequence/order checks fail closed. Partial submission or cleanup
failure cannot switch to serial reads while old requests are pending. Ordinary
boots, other peers, internal models and eGPU paths remain serial/unaffected.
The volatile /dev/shm/carrot-jetlink-aio-test diagnostic flag additionally
requires fresh Park/inactive state before activation. Native FunctionFS
performance, cancellation and reconnect validation are required before this
candidate can become a default. It is not yet a claimed latency improvement.

## Native AIO comparison and final installed state

Vehicle c437f7454b includes the parked-position and transactional route-progress
fixes. Installation fast-forwarded only carrot-nav after fresh Park/inactive
checks, preserving both locally generated radar DBC modifications and all
untracked build products. Jetson remains on signed host 989386eb85 and the
same RL Nav 80eedcdd engine; no engine rebuild or new host package was needed.

The volatile AIO flag was enabled only while parked. The live reader mapped
the compiled AIO library and completed warmup before joining. Two separate
120-second measurements, first AIO and then restored serial reads, gave:

| Reader | Model mean ms | p99 ms | Maximum ms | Valid model results | Frame skips |
| --- | ---: | ---: | ---: | ---: | ---: |
| Four queued 16 KB AIO reads | 42.131 | 46.894 | 63.042 | 2,400 | 0 |
| Existing serial reads | 44.071 | 48.188 | 57.589 | 2,400 | 0 |

Both windows have valid odometry and DM outputs, healthy livePose flags, no
dashboard alerts and no unexpected modeld/locationd/jetlinkd restart. Model
publication averages 50.005 / 50.002 ms respectively, but maximum intervals
are 74.754 / 61.829 ms. Mean USB response scope decreases with AIO, while
Jetson engine and request-processing means remain around 25.4 / 27.9 ms.
Response scope includes server inference and is not pure USB transfer time.

An intentional SIGINT of jetlinkd while parked exercised AIO shutdown and
USB re-enumeration. The old process exited, a fresh serial reader connected,
and the external model rejoined automatically. No cleanup failure was
reported. This is one native teardown/reconnect check, not exhaustive cable-
fault or kernel cancellation validation.

The AIO run uses 71.8% aggregate comma CPU (busiest core 83.9%); serial uses
72.6% (busiest 84.2%). A separate ten-second Jetson sample is 10.9% aggregate,
busiest core 26.1%. Existing little-core affinity is retained. These short
sequential runs do not isolate every scheduling/thermal effect.

**Do not enable AIO by default yet.** It improves the measured average by
about 1.94 ms, but does not achieve 40 ms or eliminate outliers, and no route
was received during either window. The phone is no longer visible to ADB and
the restarted receiver has no vehicle/route snapshot. Thus this comparison
does not cover active navigation rendering or routed driving. The diagnostic
flag was removed and the final vehicle is on its original serial USB path,
with the accepted navigation fixes installed. AIO remains a default-OFF
candidate for a connected-navigation and broader disconnect/fault test.

Linux CI for c437f7454b passes 801 integration tests (11 skipped), 119
navigation/cluster checks and 11 protected-storage checks, including actual
Linux AIO regular-file order/ownership/partial-read/cleanup tests. eGPU model
compatibility and Windows installer checks pass. Regular-file tests do not
substitute for FunctionFS fault tests. Neither this work nor the parked map
hold repairs absent phone GPS fixes, accepts a real 99.5 m route mismatch,
or establishes road stability. Raw timings and sender diagnostics stay private.
