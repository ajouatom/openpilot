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
socket suite cannot run on Windows because sendmsg is unavailable; Linux CI
must cover that path. Live deployment/timing results will be recorded separately;
this initial evidence does not establish a 40 ms model or road stability.
