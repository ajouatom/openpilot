# Jetson regression isolation — 2026-10-24

The owner requests one-change-at-a-time device comparisons after October 23 updates. These diagnostic startup environment switches preserve the existing default behavior and do not change models, freshness checks, validity or deadlines:

- `JETLINK_RAM_INPUT=0`: existing legacy comma-local socket input path.
- `JETLINK_COMPACT_REPLY=0`: request full validated N16 output without enabling raw-prediction publication.
- `JETLINK_CPU_OVERLAP=0`: prepare the same frame's context before submitting camera warp, retaining the ordinary completion fence.
- `JETLINK_REQUEST_CHUNKS=0`: negotiate the legacy USB request writer, for comparison with today's bounded requests. This can reproduce the recorded large contiguous allocation stalls; it is not a recommended workaround.
- Jetson already supports `CARROT_TRT_DIRECT_MAP=0` and `CARROT_TRT_NATIVE_IMAGE=0` for its original input preparation paths.

Temporary trials require a fresh parked/inactive guard, an offroad transition before restarts, explicit restoration, and identical NAV/HUD conditions. They are diagnostic experiments, not manual installation requirements. Normal delivery remains Git Pull and restart, with Jetson reboot when changing its signed release. No diagnostic override is enabled by this commit.

Desktop checks: 123 pass / 36 platform skips. Actual adapter tests compare input bytes and desire history with overlap on/off; actual local IPC tests verify compact/full selection independently of raw prediction publication. Linux/device results follow after collection. No causal attribution or zero-exceedance claim yet.
