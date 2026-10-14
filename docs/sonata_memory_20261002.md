# Sonata memory investigation, 2026-10-02

Analysis only; no runtime code or settings changed.

Evidence: HYUNDAI_SONATA, route `00000417--368f2340bc--39`,
full rlog SHA256 `72e881ee7c690518a806a649f7bff5141e0392a1d03f3da3c1da3691932edf57`.
initData confirms `abe1a232d81fd5b0f8db4b7212587952514e0274`, device `tizi`.
The supplied NAS directory contains only segment 39 of this route.
There are 112 deviceState and 29 procLog samples. Times below are relative
to the first deviceState, monotonic 2432.603867128 s; this is not the exact
video start. Device-state coverage is 59.632 s, process coverage 55.998 s.

## Findings

- Memory usage is 85% for 98 samples, 86% for 10 and 87% for 4; it returns
  to 85% at the end. This segment does not show runaway overall growth.
- MemTotal is 3607.10 MiB. MemAvailable is 535.88 MiB at the first procLog,
  falls temporarily to 481.56 MiB, and ends at 530.98 MiB: a net reduction
  of 4.89 MiB. MemFree alone is only 48.05–80.43 MiB, but excludes reusable
  memory and is not the amount available for new allocations.
- At +47.39 s, six ffmpeg-related processes are present with aggregate
  RSS 75.01 MiB while MemAvailable reaches its minimum. Their command lines
  include recorded `qcamera.ts` inputs, `-vframes 1 -vf scale=640:-1`, and
  `/data/carrot/cache/dashcam/thumb/` outputs. Four to six such processes
  remain in snapshots through +53.39 s; none is present at +55.39 s, when
  MemAvailable has recovered to 531.05 MiB. This strongly supports transient
  dashcam-thumbnail work contributing to the late usage spike. RSS includes
  shared pages and cannot be equated exactly with the available-memory drop.
  Two-second process sampling misses short-lived jobs and peak concurrency.
- Cached pages grow from 538.46 to 551.20 MiB before the thumbnail burst,
  fall to 484.03 MiB during it, then end at 519.20 MiB. Shared memory changes
  only from 158.02 to 158.18 MiB. Cache reclamation is visible; cache growth
  alone is not evidence of a leak.

## Long-lived processes

Values are RSS in MiB at the first and last procLog sample:

| Process | First | Last | Change |
| --- | ---: | ---: | ---: |
| radard_dpath | 126.758 | 128.031 | +1.273 |
| UI | 339.410 | 339.625 | +0.215 |
| Carrot Web | 85.156 | 85.328 | +0.172 |
| driving model | 306.832 | 306.816 | -0.016 |
| driver-monitoring model | 239.922 | 239.906 | -0.016 |

Radar has the largest net RSS increase among continuously observed processes.
It rises in roughly 0.25 MiB steps and stops increasing in the last 16 seconds.
This is a candidate for longer-term tracking, not proof of a leak or proof of
which allocations grew. History/track cleanup exists in radar code, but source
inspection alone cannot distinguish retained objects, allocator high-water
behavior, fragmentation or other growth in this run.

Carrot Web virtual size increases by 504.55 MiB with thread count 6 to 13,
but RSS rises only 0.17 MiB. Virtual address space must not be reported as
an equivalent physical-memory leak.

PSS totals for continuously observed processes change from 1611.95 to
1614.32 MiB, but these are cached measurements, not instantaneous endpoint
readings. The recorded revision refreshes smaps every 20 procLog cycles
(about 40 seconds). Radar PSS changes 38.057 to 38.889 MiB at the observed
refresh near +19.39 s. All Pss_Anon/Pss_Shmem fields are zero in this log;
they cannot establish zero private/shared allocations. The implementation
supports older kernels without those subfields. Some system processes also
have zero PSS, so summed PSS does not account for all system memory.

The 112 managerState snapshots have unchanged process records/PIDs. No
memory-related application log message or memory alert was found. This is
not a complete kernel OOM audit and does not rule out problems outside this
one-minute window.

## What remains unknown

This log identifies the transient spike and a modest radar increase, but
cannot explain how usage reached its starting 85% over the preceding drive.
Compare early, middle and late segments from the same uninterrupted drive
(for example segments 0/10/20/30/39, with startup warm-up treated separately).
If radar PSS/RSS continues growing, measure object/history counts and allocation
profiles in a replay or parked reproduction. Exact heap objects, kernel slab,
GPU allocations and swap behavior are not resolved by the recorded fields.

Reproduction script and reduced evidence are retained privately in
`.analysis/archive/2026-10-02/sonata-memory/`; raw log remains on NAS.
