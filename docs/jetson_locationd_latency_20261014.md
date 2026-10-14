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
