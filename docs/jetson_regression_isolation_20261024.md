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
