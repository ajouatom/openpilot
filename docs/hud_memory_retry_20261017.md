# HUD memory pressure during reconnects

The user reported memory above 92%, then clarified that they had unplugged and
reconnected HUD power before the investigation. The first observed 88% value was
therefore **after that intervention**, not evidence against the reported warning.
The user explicitly authorized transferring vehicle logs for diagnosis. Raw logs
and device credentials remain private.

## Evidence and scope

The C4 remained on the same boot throughout this investigation, running
`2ef833b201` with eGPU active. Fresh Park, standstill, inactive carControl and
panda permission checks preceded each HUD intervention. Model/control code was
not installed or restarted. The unrelated later signal-notice branch change was
not installed on the vehicle.

The initial available memory was about 437 MiB, then 414 MiB. The old historical
510 leaked graphics objects were absent: captured orphan allocations totaled
4.75 MiB, insufficient to explain the current pressure. No kernel OOM kill was
found in the captured dmesg. Stopping only HUD recovered availability from
408.10 MiB to 607.02 MiB over 60 seconds. This includes concurrent system
variation and is not a precise measurement of a HUD leak.

Three current-boot routes provide 11,046 device-memory observations and 210
procLog snapshots. The retained data reaches 91%, including a minute with 63
samples above 90%; it does not contain every instant of the user's 92% report.
The same HUD PID held roughly 200–220 MiB PSS when rendering, and approximately
131–135 MiB while disconnected. DM rises during warmup and then levels near
151 MiB; this was also seen before this report. A separate two-minute, 0.5-second
RSS observation with HUD stopped shows loggerd rising about 21 MiB at successive
60-second boundaries, then releasing memory. It does not establish a loggerd
leak. These are separate contributors to limited headroom, not a complete
attribution of every byte or a claim that total memory is normal.

## Demonstrated lifecycle defects and changes

1. The previous fix unfreezes GC at renderer exit, but production autorun did
   not collect after leaving the exception handler. The next explicit collection
   happened late in the next renderer initialization. Actual failed JPEG and
   H.264 sessions retained display objects until collection; a regression test
   executes the real autorun loop and fails before this correction. Collect after
   the session stack and handled exception have unwound, including normal exits
   and the desktop-window fallback.
2. Collection alone did not meaningfully return RSS. In an isolated native C4
   failure test, returning unused glibc heap pages reduced PSS from 103,931 to
   69,695 KiB for JPEG and from 106,433 to 73,341 KiB for H.264. That is about
   33.4 and 32.3 MiB. Another test using actual live inputs recovered about
   8.6–10.0 MiB after initialization and 24.7–33.2 MiB after teardown. These
   observations include small diagnostic-allocation effects and are not promises
   of a fixed saving on every frame/session.
3. Authenticated UDP discovery now precedes expensive graphics/encoder setup.
   When S3 is missing, autorun waits without initializing those resources. TCP
   still opens after renderer initialization, preserving the S3 idle-timeout
   correction. Discovery failure remains a concise, bounded retry message.

Unused heap pages are returned only before the active frame loop and after a
session ends. There is no per-frame collection/trim, frame-rate reduction,
process-cycling policy, warning-threshold change, or router/firmware change.
Non-Linux/non-glibc environments keep ordinary collection without native trim.

## Validation and remaining limits

- Desktop: 76 lifecycle/transport/graphics tests plus 38 autorun/performance
  tests pass; one platform-specific test is skipped. New cases cover missing
  display discovery before allocation, eventual discovery, normal and failed
  session collection, and collection before optional native heap return.
- Native isolated C4: six JPEG failures followed by six H.264 failures, using
  real live input, renderer and native hardware encoder, with transport failure
  injected instead of sending to S3. Every closed session loses its weak
  display reference and has zero test-owned ION rows. Post-cleanup JPEG PSS
  rises from 73.13 to 78.43 MiB during warmup; H.264 from 79.72 to 80.80 MiB.
  This is a bounded reclamation check, **not** proof of a flat long-duration
  plateau. A 3 MiB client allocation from an earlier separate probe remains in
  the kernel listing; it did not recur in these 12 cycles and is not explained
  or attributed to the full warning.
- Selective vehicle installation: only `cluster_autorun.py` and
  `cluster/cluster_profile.py`, with preimage hashes and private backups.
  HUD setting restored to 1. No full pull or C4 reboot.
- Final independent 60.0035-second frame observation: model 1,200 valid
  consecutive frames at 20.0049 Hz; DM 1,201 at 20.0025 Hz, no missing IDs.
  Original model/DM PIDs remain and eGPU stays active. This is a measured
  interval, not a hard real-time guarantee.

S3 discovery/TCP failures continued during this investigation. A 600-second
observation included HUD pauses for isolation and had no connected sample, so it
must not be presented as ten minutes of successful video or stable active-HUD
memory. The initial final snapshot had about 448.66 MiB available with S3 disconnected and
HUD RSS about 80.35 MiB. The fixes reclaim demonstrated retained memory and
avoid needless startup allocations, but **the user's overall 92% issue and
long-duration connected-HUD behavior are not fully cleared**. Keep investigating
if memory pressure returns; do not tell the user that every leak is solved.

A subsequent 120-second check of the final installed code captured actual S3
retries in the same HUD PID. RSS repeatedly reached about 194 MiB during startup
and returned to about 87 MiB after failure. The observed post-cleanup values were
89,268--89,328 KiB rather than accumulating another session each time. Available
memory ranged from 377.61 to 459.56 MiB and ended at 455.84 MiB (device display
87%). Only one of 24 samples reported S3 connected; the latest state was
disconnected. This supports actual retry reclamation, not sustained streaming
stability or a long-duration leak clearance. Both maintained branches received
the fix with single-parent commits; the vehicle retains the selective HUD install.
