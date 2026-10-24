# Jetson regression isolation — 2026-10-24

The owner requests one-change-at-a-time device comparisons after October 23 updates. These diagnostic startup environment switches preserve the existing default behavior and do not change models, freshness checks, validity or deadlines:

- `JETLINK_RAM_INPUT=0`: existing legacy comma-local socket input path.
- `JETLINK_COMPACT_REPLY=0`: request full validated N16 output without enabling raw-prediction publication.
- `JETLINK_CPU_OVERLAP=0`: prepare the same frame's context before submitting camera warp, retaining the ordinary completion fence.
- `JETLINK_REQUEST_CHUNKS=0`: negotiate the legacy USB request writer, for comparison with today's bounded requests. This can reproduce the recorded large contiguous allocation stalls; it is not a recommended workaround.
- Jetson already supports `CARROT_TRT_DIRECT_MAP=0` and `CARROT_TRT_NATIVE_IMAGE=0` for its original input preparation paths.

Temporary trials require a fresh parked/inactive guard, an offroad transition before restarts, explicit restoration, and identical NAV/HUD conditions. They are diagnostic experiments, not manual installation requirements. Normal delivery remains Git Pull and restart, with Jetson reboot when changing its signed release. No diagnostic override is enabled by this commit.

Desktop checks: 123 pass / 36 platform skips. Actual adapter tests compare input bytes and desire history with overlap on/off; actual local IPC tests verify compact/full selection independently of raw prediction publication. Linux/device results follow after collection. No causal attribution or zero-exceedance claim yet.


## Initial physical comparisons (still in progress)

Source 3032c4c613 passed Linux integration and Windows installer run 40068132845 before installation. Normal fast-forward Git Pull and guarded restart applied it; the signed Jetson source remains 69c4642789 and N16 identity is unchanged. Every temporary setting is applied only to the restarted process environment; original tmux globals are restored immediately. Maintenance holds have independent restoration timers. No persistent user setting or per-device source overlay is used.

| Condition | Duration | Model mean / max (ms) | Over 50 ms | NAV ready samples |
| --- | ---: | ---: | ---: | ---: |
| Current defaults before comparison | 120 s | 40.852 / 51.488 | 2 | 120/120 |
| Shared input OFF only | 120 s | 41.563 / 55.644 | 2 | 120/120 |
| Compact output OFF only | 120 s | 44.158 / 52.368 | 8 | 103/120 |

All three windows have valid consecutive model/DM/camera/odometry outputs and healthy pose, with link/model/HUD fresh in all 120 samples. The compact-output window includes six position-invalid and eleven route-position-invalid samples: it is not fully matched NAV exposure. Short parked windows do not establish road stability or identify the cause of an infrequent disconnect.

Shared input OFF changes measured local IPC from 60 to 655616 bytes, while USB inference request size remains 458996 bytes. Jetlink thread CPU increases from 6.368 to 7.032 ms, server total remains 25.628 versus 25.589 ms. In the full-output window server total is 25.592 ms and exchange rises from 34.528 to 38.046 ms. These observations support keeping these two optimizations for latency in the measured windows; they do not rule out all long-term failure modes.

An initial diagnostic restart failed to pass the selected environment into a newly created tmux server; it was excluded, its hold restored, and the launcher corrected to set both inherited and tmux environments. A readiness observer incorrectly expected a boolean NAV field and exited without collecting a comparison; it was corrected to the actual reason=ready contract. Actual modeld/jetlinkd environments and local IPC aggregates confirm the measured switch states. These diagnostic failures are not attributed to production code.


## Confounded window and lower-overhead verification

A first CPU-overlap OFF observer window had one model frame gap, one invalid odometry message and one unhealthy pose. Two closed rlogs (20,392,725 bytes, source/local SHA256 matching) show that both cameras and DM actually published all frames: the observer omitted their closely spaced messages. Camera frame 3125 publication was delayed to approximately 99 ms after road-camera EOF (wide camera about 86 ms), while capture SOF/EOF remained continuous. Model frame 3126 is genuinely absent, followed by invalid odometry at 3127. Same-frame model timing is 56.997 ms; its slow log reports server total 24.91 ms. Thus this was not a missing physical capture or a 57 ms TensorRT execution.

Memory usage rose from 90 to 93 percent near that event. A new SSH stdin Python process grew from 18.8 to 82.2 MiB RSS and used 1.16 CPU seconds across the same two-second procLog interval. Its exact script was not established; it had exited by the later identity check. This window is confounded by concurrent diagnostic load and is excluded from attributing the event to CPU overlap. Original reported drive segments were mostly 79?80 percent memory usage, so this new high-memory observation does not explain all original drive errors.

A follow-up status field reports options from inside modeld, avoiding ambiguous /proc environment snapshots after process-title/fork handling. N16 explicitly reports CPU overlap OFF, shared input ON and compact output ON. Subsequent comparison uses only standard-library status sampling (no cereal subscriptions or on-device log decoding), followed by download of closed production rlogs. Three files (30,705,939 bytes) match source/local hashes. The 120.055-second overlap-OFF window has 2401 valid consecutive model/DM/camera/odometry outputs, healthy pose, mean41.135 ms, P9945.247, maximum48.878 and zero above50. All recorded car/control/panda samples preserve P, stationary and inactive. NAV ready61/120, position-invalid59/120; lowMemory remains at92 percent. This is a bounded result with incomplete valid NAV exposure, not proof that disabling overlap fixes a tail. A matching default-mode comparison follows.


## Restart memory accumulation and shutdown candidate

ION reports 569663488 bytes (543.273 MiB) of orphaned imports, with repeated prior UI PIDs accounting for about43.35 MiB each. This accumulated across the agent's repeated manager restarts and makes high-memory trial conditions different from the earlier drive. A guarded normal manager reboot restores absent IsTakingSnapshot and changes the kernel boot. After active N16 recovery, orphaned bytes fall to4980736 (~4.75 MiB) and MemAvailable rises from247752 to992644 KiB. This demonstrates a reboot-cleared resource accumulation, not proof of the origin of the reported midday Jetson reboot or original drive errors.

UI shutdown currently closes the GL window without explicitly retiring camera EGL imports retained by widget cycles, and only SIGINT enters the explicit close handler. The candidate records native image ownership, releases remaining imports before closing the window, makes repeated destruction safe when fd numbers are reused, and handles SIGTERM/SIGHUP through the same close path. It changes shutdown only. Four desktop tests execute actual allocation/destruction and the actual GuiApplication.close body with native API doubles, including descriptor reuse, a native failure among multiple images, failed import and shutdown ordering. Physical restart verification is pending; do not describe this as a validated driver fix yet.

The clean-boot default120.077-second rlog window has2401 valid consecutive model/odometry and2402 valid consecutive DM/camera/pose outputs; all pose healthy and all recorded P/inactive guards pass. NAV/link120/120 ready, memory73?74 percent. Model mean40.846, max52.441, one above50; DM max53.220, one above50 at a different time. The slow model's same-frame log has warp4.09, roundtrip47.01, parse0.84 and server total24.97 ms. Only the low-footprint8.3 MiB sampler appears as SSH stdin Python around that event. Clearing restart accumulation therefore does not cure the residual transport tail.


## Legacy USB comparison

The guarded normal restart installs shutdown candidate710da0d285 after Linux/Windows run40072104541 and eGPU run40072104567 pass. Its first installation still exits the previous UI, so it is not a cleanup test yet: orphaned ION increases from4980736 to52092928 bytes, with new UI PID recorded privately. The following restart exits the candidate itself and is used for validation.

With only JETLINK_REQUEST_CHUNKS=0, a120.048-second window has2401 valid consecutive model/DM/both-camera/odometry/pose outputs, all pose healthy and all recorded P/inactive guards passing. NAV/link/HUD120/120 ready. Model mean38.121, P9942.134, max46.249 ms, zero above50; memory74-75 percent. Server total25.593 ms is comparable to clean-default25.614, while send4.820 versus7.316 and exchange31.653 versus34.517 explain much of the mean difference. This supports attributing roughly2.7 ms of current average overhead to today's bounded-request path. The short legacy window does not eliminate previously recorded large contiguous allocation stalls, so it is not sufficient evidence to recommend reverting that protection. All other October23 optimizations remain active during this comparison.


The candidate's own normal restart increases orphaned ION only from52092928 to55410688 bytes (3.164 MiB), versus47,112,192 bytes (44.93 MiB) when exiting the previous UI. Old candidate UI's large camera imports are absent; only three548864/4096-byte pairs remain attributed to its framebuffer resources. This is evidence of camera-import cleanup on the physical C4, not zero residual graphics allocations or a solution to USB tails. The active candidate UI recovers normally. A subsequent default-mode sampler waited180 seconds without valid NAV (position/route-position) and produced no measurement; it is excluded. Remaining trials report actual NAV reasons even when position is unavailable and retain all input validity gates.

Wip receives common UI resource cleanup only; its model and selected host runtime remain unchanged. NAV diagnostic switches are not ported.
