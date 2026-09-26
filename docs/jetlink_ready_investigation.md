# Jetlink READY reconnect investigation, September 26, 2026

Scope: carrot-jetlink, C4 and Orin Nano Super, originally deployed 29012bc5.
The user reported WAIT/READY/RETRY cycling before external-model activation,
and a USB HUD that did not recover after unplugging/replugging. These are two
separate defects. The main carrot-wip/eGPU path is outside this change.

## Evidence and cause

The natural startup journal contains seven host `bad magic` disconnects from
08:36:42 through 08:42:20, followed by sustained inference after 08:42:26.
C4 traces for the same reconnect sequence end in `client.ping()` waiting for
PONG, while no local model client has joined. Thus READY was being lost in the
standby USB path, not because the model was too slow or Cap'n Proto decoding
rejected vehicle data. The two machines' wall clocks differ; sequence/nonce
and byte identity must be used instead of directly subtracting their clocks.
The previous night's occurrence is not independently established by these
logs: the Jetson boot journal also contains a large wall-clock adjustment.

A parked, continuously guarded experiment denied new local model connections
temporarily using the existing UNIX socket's permissions. Native inference
continued. The permissions were restored by a timed helper, also on any loss
of fresh stopped/disengaged state. The resulting PermissionError in model
status was intentional test instrumentation, not the original failure.

Sixteen captured protocol failures had the same mechanism. After receiving a
valid HUD message, the host started a 1024-byte header read. At about 99 ms,
the timed-out USB transfer reported 1024 bytes containing the previous header.
The next normal message then arrived where the parser expected a body; the
following payload/padding became the next supposed header and failed magic.
The diagnostic also recorded raw libusb outcomes: `LIBUSB_ERROR_TIMEOUT`
with `transferred=1024`. The repeated blocks differ from their predecessor
only at bytes 512..519. This locates the failure below the message parser,
at the timed USB IN completion/cancellation path; it does not identify which
kernel/controller component is responsible for the stale block.

A simultaneous C4 syscall trace recorded 553 HUD writes, 32 PINGs, 40 warmup
INFERs, and four HELLO/ENGINE pairs across four sessions, with no duplicate
message sequence submitted within a session. For example HUD seq123,
payload47606, was submitted once and returned the complete49152-byte padded
write; the host later reported that header again at its timeout. This excludes
an application-level retry of that message, not all possible gadget/host
controller behavior below writev.

Changing only the USB read wait from100 to500 ms gave about130 seconds without
the fault. Returning to100 ms reproduced four further protocol failures.
This was a diagnostic comparison, not the final fix. The C4 IPC deadline was
150 ms throughout. The prior 20 Hz inference tests exercised fewer idle read
cancellations than the roughly104 ms HUD intervals in standby.

libusb documents that timeouts can legitimately include partial data, so
discarding every timeout's bytes is not a valid repair:
[libusb synchronous I/O](https://libusb.sourceforge.io/api-1.0/group__libusb__syncio.html).

## Repair

`tools/jetlink/host_usb.py` uses one asynchronous USB IN request with no USB
timeout. The reader services completion events while waiting; it does not
cancel and resubmit an idle request every100 ms. Consumer queue timeouts and
the independent C4 model deadline remain bounded. Closing signals the reader
to cancel its outstanding request and wait for the completion callback before
freeing its buffer. Handle teardown is refused while a transfer is submitted.
The existing bounded two-message queue and owning payload copies remain.
This changes only the Linux USB host adapter; the Mac and existing eGPU paths
are unchanged.

The HUD boot log explicitly said `TURZX USB cluster display not found on PC;
rendering to window only`. The service remained alive in an invisible window,
even after the display was present. The Jetson adapter now waits for the USB
panel before starting the common renderer and rejects disappearance between
scan and open. It cannot enter the PC window fallback. The existing service
restart handles subsequent renderer/device errors. Device absence does not
stop or restart the inference service.

## Validation and limits

The new USB tests cover idle event polling without cancellation/resubmission,
partial cancellation and callback-before-free ordering, disconnected input,
and refusing premature teardown. HUD tests cover late appearance and the
scan/open race. C4 Linux candidate tests passed54, including existing model
output/deadline/warp/DM phase/HUD tests and the user-authorized stopped-or-
steeringPressed join conditions. A subsequent close/drain regression brought
the final Linux total to55; desktop host tests passed32. No model validity
threshold was relaxed.

The initial repaired standby run lasted180 seconds, with3600 valid native
model/camera/pose messages, no frame skips or invalidity and no host protocol
disconnect. Native execution mean24.13 ms, max32.40 ms is not Jetson inference
timing. The previously missing panel recovered to USB/H.264 output at10 Hz.
With the HUD publisher suspended for30 seconds,149 sampled link states stayed
READY while only the2-second PING kept the otherwise-idle link alive. The
publisher was resumed automatically. Releasing the model-join hold activated
Jetson in5.63 seconds (including the existing5-second retry interval).

A120-second full-DM plus actual USB HUD run then recorded2400 model and2400 DM
messages, no frame-ID skips or invalidity, and120/120 active samples. Model
execution mean37.276, p9941.024, max47.275 ms, zero >50 ms. DM execution,
including the existing phase wait, mean30.262, max43.526 ms. The original
DisableDM=2 was restored. These timings exclude the intentional server
restarts and DM initialization outside the measured window; they do not
establish a maximum end-to-end camera-to-control latency. Navigation video
was not being supplied during this run.

Private journals, raw receive rings, syscall traces and reproduction scripts
are retained under `.analysis/archive/2026-09-26/jetlink-desync/`; they must not
be committed or published. This is parked C4 evidence, not C3, Mac, loaded
driving, or a hard-realtime guarantee.
