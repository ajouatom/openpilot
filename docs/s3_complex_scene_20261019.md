# S3 complex-scene reproduction, 2026-10-19

## Scope and isolation

The user reports nearly 10 FPS while stationary but frequent blank/reconnect
when the picture changes, on both company Wi-Fi and a phone hotspot, and asks
for deliberately complex traffic. Tests use the user's parked C4 and paired S3.
UART is unavailable: the S3 remains in the vehicle. No router, firmware, model,
DM or eGPU change is part of these experiments.

The vehicle was already on a new boot with base 68e7785c8b when this turn began.
Memory was around 78% during isolated tests. This reproduces failures without
requiring the earlier 90-92% memory condition; it does not solve that growth.

Each synthetic test requires fresh Park/standstill/inactive controls and panda
controls disallowed, stops only managed HUD, retains model publication/eGPU and
memory headroom checks, runs at low priority with a wall-time bound, and restores
ClusterHud=1 in finally. Input is native NV12, 462x1920 visible / 464x1920 encoded,
10 FPS, GOP 10, 4096-byte requested slices. Random tests use a deterministic seed.
The camera/model inputs are not modified. Raw diagnostics remain private.

## Findings

1. Existing 1 Mbps VBR-CFR is not guaranteed to publish every accepted input.
   Static quadrants produce 300 access units in 30 s, about 30 kbps. Switching
   to full-resolution scrolling random luma accepts 751 inputs in 75 s but
   produces just one residual static unit. Subsequent independently refreshed
   random luma accepts 601 inputs in 60 s and produces zero units. The pipeline
   reports no error while TCP is CLOSE-WAIT and the connected flag is stale.
   Synthetic input FPS must not be reported as delivered video FPS.
2. Removing all network traffic and Python packet parsing reproduces this:
   raw native callbacks for VBR-CFR and MBR-CFR give one residual unit during
   10 s motion and zero during 10 s noise, then recover when static resumes.
   CBR-CFR produces 101 units for each 10 s motion/noise phase. This establishes
   a content-dependent encoder output failure, not a Wi-Fi bandwidth failure
   for those zero-output phases. It is not yet proof of every real-drive blank.
3. CBR is not by itself a safe complete correction. A gradual static -> changing
   16-pixel blocks -> scrolling noise -> independent noise -> static test passes
   approximately 90 s through the actual S3, with 301/201/201 complex units,
   about 1 Mbps and no transport exception. An abrupt static -> noise transition
   can instead produce an 831472-byte unit. With the original parser one trial
   survives but loses cadence (183 units / 20 s, maximum submit 1.447 s). With
   the fast parser another trial produces multiple huge units and fills the
   ordered three-slot queue, reaching its 3 s timeout. Do not call CBR alone a
   disconnect fix or discard arbitrary dependent H.264 frames to hide it.
4. Annex-B start-code scanning in Python adds substantial avoidable latency.
   Isolated on-device 100000-byte scans take 394-487 ms with the old byte loop,
   versus 0.212-0.228 ms with bytes.find. At 831472 bytes the respective ranges
   are 2.986-3.407 s and 1.745-3.635 ms. These are low-priority synthetic parser
   benchmarks, not full-pipeline/network timing. The replacement preserves
   three/four-byte prefixes, leading zeros and prefix boundary semantics.
5. Independent link failures also exist before video. Header capture shows S3
   FIN roughly 2 s after the last acknowledged small command. Several trial
   initializations fail on discovery or sync before sending any image. Later
   an unchanged connection lasts a 300 s header-only capture without FIN/RST.
   The association shown in Omada is about -73 dBm on a 2.4 GHz AP; this does
   not establish a mesh configuration defect, nor explain the hotspot report.
   A missed authenticated capability response can also select JPEG in auto
   mode; this was observed in the real restored HUD, increasing traffic.

The S3 firmware receive timeout and successful-video idle-off are both about
2 s. Encoder silence is consistent with that shutdown mechanism. Without UART,
the exact firmware reason for each close remains unconfirmed.

## Memory observer correction

The previously pending restored-state observer was interrupted by the reboot:
322 samples span 1639.90 s, not two hours. Available memory goes 470.42 to
578.63 MiB (minimum 394.99), with changing HUD retry/activity states. Shared
memory rises 218.11 to 224.26 MiB; RAM log/shared-memory mounts also grow.
These mixed states do not prove a long memory plateau or clear the overnight
increase. Do not reuse its old-boot snapshots as current-boot readings.

## Validation and disposition

Only the byte-equivalent Annex-B search optimization is retained in production.
CBR default changes were withdrawn after the abrupt-scene failure. Three
additional isolated raw-encoder trials set per-frame minimum QP controls to
28/36/44 after stream creation. The ioctls return success but the packet sizes
remain identical, including the 831472-byte maximum. These settings are not a
validated frame-size limit and are not installed.

The parser correction was selectively installed on C4 after checking exact
base hashes and backing up the old file. HUD was restarted under Park guards;
modeld PID 49147 and DM PID 49149 remained unchanged. Existing compression,
firmware and router settings remain in place. Exhaustive short prefix-boundary
checks and a >830 KB mixed-NAL fixture match the old scanner exactly. Both
maintained branches pass 52 focused tests with one Linux-only skip on Windows.
A stale orientation-test stub was updated for the existing shared_usb argument.

This fixes measured packet-processing cost, not encoder silence, burst size,
radio loss, JPEG fallback on a missed capability response, or overnight memory
growth. No loss-free road, optical-FPS or complete reconnect fix is claimed.
The remaining encoder work requires a verified pre-stream peak/quality bound
and actual moving-scene validation; merely switching rate-control names failed.


Final read-only 180.106 s observation after the parser installation records
3601 valid consecutive model frames at 20.000236 Hz and 3601 valid consecutive
DM frames at 20.000119 Hz, no missing IDs. Connection is false for the first
five 5-second samples and true for the following 31 (initial recovery about
25 s, then about 155 s). The subsequent actual HUD diagnostics report 100
H.264 units per 10.0-10.1 s, 971-1009 kbps, maximum unit around 98 KB, queue
maximum one and zero dropped units. TCP is established at the end. Available
memory is 692.37 -> 689.74 MiB (minimum 659.99), around 81% in the restored HUD
state. This bounded parked result is not proof of long-term leak clearance or
stable operation under the failing full-resolution random scene.


## Follow-up: apply bounded C4 transmission, not CBR alone

The user explicitly rejected stopping at the parser-only correction. Subsequent
native tests establish why the earlier QP calls had no effect: the pinned MSM
VIDC driver accepts individual per-frame QP S_CTRL calls without applying their
range. Its S_EXT_CTRLS path requires the layer selector and all six I/P/B limits.
See the [comma kernel implementation](https://github.com/commaai/agnos-kernel-sdm845/blob/master/drivers/media/platform/msm/vidc/msm_venc.c).
Pre-stream extended controls with all-layer ID 255 work on the actual device;
the standard generic H.264 minimum-QP control and the old packed control return
EINVAL. These experiments use isolated per-process diagnostic shims, never
installed globally. Production uses a new native bridge setter before open,
and aborts initialization on a missing API or failed control instead of silently
ignoring the limit. Wired displays do not request this setting.

S3 native defaults are now CBR-CFR, 500 kbps target, QP range 44..51, GOP 10.
Explicit bitrate/rate-control CLI choices remain respected. QP is a quality
constraint, NOT a strict frame-size bound: luma-only transitions shrink from
831472 bytes to about 28 KB, but independently random luma and chroma still
reach 176980 bytes. A fixed QP 51 trial does not eliminate that worst case.
CBR+QP alone passes a 110-second luma sequence, then fails three full-colour
trials. Smaller 16 KB chunks alone also fail. These failures are retained in
the private evidence; they must not be reported as successful releases.

The retained S3 sender additionally paces chunks of at most 16384 bytes at
125000 bytes/s, counting the 512-byte display header (not TCP/IP/MAC overhead).
It accrues no idle burst credit. The three ordered access-unit slots still
backpressure only the HUD encoder. Its three-second dead-sender timer now
resets on completed chunk transmission, rather than declaring a progressing
large frame disconnected simply because the access-unit queue stays full.
The old timer demonstrably fired during paced output despite ACK waits of
less than 5 ms. No dependent H.264 reference units are discarded.

An authenticated capability query now retries lost UDP queries every 250 ms
within the existing one-second deadline. Old firmware still falls back to JPEG;
forged/replayed replies remain rejected. A lost first reply no longer forces
JPEG if a subsequent valid response arrives within that deadline.

The exact new native library and Python paths were tested on the actual S3.
The first final-profile run still loses the connection early in colour noise,
and its next initialization fails before video. A subsequent full run passes:

| Input phase | Duration | Encoded units | Largest unit | Encoded kbps |
|---|---:|---:|---:|---:|
| Static | 5.01 s | 50 | 5392 B | 380 |
| Independent full-colour noise | 30.01 s | 90 | 176980 B | 982 |
| Static recovery (includes queued noise) | 5.03 s | 33 | 176372 B | 545 |
| Scrolling random luma | 20.01 s | 201 | 26516 B | 351 |
| Independent random luma | 20.03 s | 201 | 26564 B | 621 |
| Changing 16-pixel blocks | 20.01 s | 201 | 9872 B | 746 |

This is graceful throughput reduction to about 3 FPS for deliberately
incompressible colour noise, not a 10 FPS worst-case claim. Subsequent luma and
block phases recover about 10 FPS with no transport exception. Full-colour
reference backlog briefly delays static recovery. Neither encoder target nor
QP is misrepresented as a hard network limit. The passing run's monitor records
2201 valid consecutive model and DM updates each, at 20.000639 / 20.000625 Hz.

Header-only captures also identify independent link loss: even a 568-byte sync
command can be retransmitted without acknowledgement until the S3 sends FIN
around its two-second receive timeout. C4 transmit evidence cannot identify
whether the loss is at its radio, AP, or S3 receive path. Current S3 UART is not
available, so no firmware-internal cause or universal hotspot fix is claimed.
No router or firmware changes accompany this correction.

Guarded selective installation backs up six source files and the native library,
atomically replaces the library, and restarts only HUD. Native library SHA256:
`7305711dd3a9aa3abd96c6615412a405a173101327dea7712dd98d3f0a1c685b`.
Modeld 49147 and DM 49149 remain unchanged, eGPU active. Early actual-HUD windows
report 100 units per 10.1 seconds, approximately 279-280 kbps and 8.4 KB maximum
units, compared with earlier approximately 1 Mbps windows. The scenes/time
are not identical; this is observed current output, not a matched codec ratio.
Physical legibility and optical frame rate require user confirmation.

This correction addresses reproduced content-dependent encoder silence,
uncontrolled host bursts and a false queue timeout. Independent packet loss
and the historical overnight memory increase are still unresolved; short
synthetic memory observations do not establish leak clearance.


The first installed-HUD 180.111-second observation does NOT pass connection
stability: only 16/36 five-second samples are connected (an approximately
70-second continuous interval followed by renewed failures), with no TCP
connection at the final sample. Model/DM remain consecutive and valid:
3601/3600 updates at 19.999995 / 20.000150 Hz, respectively. The first is
approximately 20, not a strict >=20 measurement. Available memory is
637.78 -> 703.92 MiB, minimum 615.68, across changing HUD active/retry states.
Do not infer a memory plateau from that. The authenticated UDP retry change
is installed in a second guarded HUD-only restart after this observation.

Both branches' focused regression suite covers exact prefix semantics, explicit
rate settings, missing/failed QP API, ordered backpressure, dead sender, sender
progress beyond three seconds, shutdown, bounded pacing with no idle credit,
and an actually lost first UDP capability response. Final result: 59 pass,
one Linux-only skip on Windows. The native library is built and loaded on the
vehicle; the Windows tests are not a replacement for this native check.


After the second HUD restart, a separate 60.028-second read-only capture has
1201 valid consecutive model and DM frames each at 20.002008 / 20.003827 Hz,
no missing IDs, eGPU active at every sample. S3 reconnects only near the end:
3/12 samples connected and an established socket at completion. This is
reconnection evidence, emphatically not a stability pass. Final available
memory is 632.89 MiB (82.45% used) with HUD active, not a long-term memory claim.
No physical readability response had arrived when the code checks finished.
The remaining task is to diagnose actual S3 receive/close events together with
packet loss; UART access is required for firmware-internal logs on the current
board. The current remote path has no firmware-update facility.
