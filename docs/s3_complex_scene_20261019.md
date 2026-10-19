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
