# Casper C3 CPU and UI optimization review — 2026-10-03

Follow-up to [route analysis](casper_c3_ui_20261003.md), same two full rlogs and
incident commit076e5cf4. Review only; no production code/settings/affinity changes.
The user explicitly excludes core7 from UI placement because of model work.
That supersedes the initial report's proposed core7 comparison.

## Core4 bursts: detailed memory accounting in proclogd

The steady drive contains three matching bursts exactly40 seconds apart.
deviceState reports core4 at88%, then100% in two successive approximately0.5 s
samples, then64–65%, before returning near its ordinary30–40% range.

| Process CPU accounting interval, route seconds | Core4 100% samples | proclogd CPU time | User / kernel CPU | PSS values changed |
| --- | --- | ---: | ---: | ---: |
| 38.505–40.505 | 39.424,39.927 | 1,280 ms | 830 /450 ms | 46 processes |
| 78.505–80.505 | 79.424,79.924 | 1,310 ms | 690 /620 ms | 45 processes |
| 118.505–120.505 | 119.424,119.924 | 1,300 ms | 780 /520 ms | 44 processes |

Outside these bursts, proclogd consumes60–70 ms per2-second interval (median70 ms).
Its PID47423 is sampled on core4 before all three bursts, core4 after the last two,
and core5 after the first. The process is free to migrate; process CPU is not an
exact per-core execution trace.

`openpilot/system/proclogd.py` runs at0.5 Hz and `_SMAPS_EVERY=20` expires every
eligible process's PSS cache together every40 seconds. Processes with RSS>5 MiB
are read through `/proc/<pid>/smaps_rollup` if available, else `/proc/<pid>/smaps`.
PSS is proportional memory accounting, which requires kernel page-table work.
This route uses kernel4.9; per-VMA smaps fallback is expected, but the chosen
filename is not logged and no live filesystem readback was performed.

Timing, CPU deltas, refreshed PSS fields and source cadence jointly identify
the periodic detailed-memory sweep as the best-supported source of the bursts.
They are much stronger evidence than merely observing a last-CPU sample.
A scheduler/syscall trace would be needed to assign every millisecond exactly.

Accounting caveat: procLog's Event timestamp and process stat snapshot are taken
before the costly PSS sweep. PSS and CPU totals are gathered later inside the
message builder. Refreshed PSS appears in the message stamped38.505/78.505/118.505,
while the sweep's own process CPU increment appears in the next procLog sample.
This is not evidence that the PSS update preceded the CPU burst by two seconds.

Planner does not show a matching calculation explosion: nearby runtime summaries
remain roughly7.6–8.4 ms mean work per frame, about20 Hz. Its maximum measured
work in those nearby summaries remains below8.8 ms. Radarcan/planner already have
realtime priority over normal proclogd.

This periodic core4 load is distinct from sustained UI/core6 contention: UI is
near19 FPS around the first burst while stopped, and can be12 FPS far from any
PSS sweep. Removing the burst alone is not a demonstrated UI fix.

First review candidate: retain lightweight CPU/RSS monitoring while distributing
expensive PSS reads across time, preserving explicit freshness semantics.
One process per scheduling turn may still be expensive, so measure the largest
single-process read and impose a work budget between reads; an elapsed-time check
cannot preempt a read already in progress. Native parsing can reduce user CPU
but cannot remove the450–620 ms kernel CPU component of these observed sweeps.
Do not move this whole burst to a different core and describe it as optimization.

## Raylib, Python and the actual Carrot drawing workload

Raylib is a C graphics library; Python calls it through CFFI. The current timing
`thread_cpu_ms` includes Python, binding conversion, native rendering/driver CPU,
and synchronous kernel work on that thread. It does not isolate Python cost.
The44.18 ms render wall time versus16.66 ms thread CPU includes descheduling and
other waits; their difference is not Python interpreter time or a GPU-only timer.

No matched NanoVG-versus-Raylib replay was performed. The observed regression
cannot establish a universal4x framework/language penalty, or a4x benefit from
rewriting the whole process. The old and current draw workloads and placements
must be held constant for such a comparison.

Second-segment active CPU per instrumented draw:

| UI section | CPU ms/frame | Share of16.66 ms |
| --- | ---: | ---: |
| Model/road overlays | 7.85 | 47% |
| HUD | 4.97 | 30% |
| Border/extensions | 1.77 | 11% |
| Camera draw | 0.85 | 5% |

The overlay's nested `path` timer includes `_draw_path_carrot`, including path-end
graphics and tire trajectories. Its1.86→3.74 ms segment comparison is not an
isolated path-projection measurement. The tire subcomponent has no separate
timing in these logs, so its exact contribution remains unknown.

Recorded startup settings relevant to this drive: ShowPathMode=0,
ShowPathModeLane=0, ShowPathColor=20, ShowPathColorLane=20,
CarrotTireTrajectory=1, ShowLaneInfo=1, ShowRadarInfo=3, ShowDebugUI=1;
uiDebug confirms recording=false and plotMode=0. Startup snapshots do not prove
no live setting changes, but these identify the configured workload.

Specific source-level candidates:

- Polygon outlines call `draw_line_ex` separately for every segment and construct
  two Vector2 objects per segment. Batch equivalent geometry/native submissions;
  preserve stroke width, joins, clipping and transparency.
- `shader_polygon.triangulate` builds a Python list, converts it to a NumPy array,
  then returns a list again for the drawing call. Reuse a contiguous vertex buffer
  and avoid repeated list/array/binding conversions where equivalence permits.
- Tire trajectories project two ribbons and configure/draw separate gradient
  shaders, plus styled labels. Split their timing before assigning blame; reuse
  buffers/uniform data and reduce state changes without changing their appearance.
- Styled text draws eight outline offsets, a shadow and foreground: up to ten
  native text draws per label. The current code already caches text measurement
  and uses a raw CFFI text symbol to avoid repeated wrapper/encoding work. Do not
  present those existing optimizations as new work. Reusing unchanged text or
  batching the outline geometry may help; an atlas/shader replacement needs
  font fallback, antialiasing and visual-equivalence checks.
- `path_geometry.py` already batches interpolation/projection with NumPy, and
  solid polygons already avoid the custom gradient shader. Remaining costs
  need profiling; blindly translating already-vectorized operations may not help.

A productive first native boundary is an overlay batch: take arrays/state once,
build/project vertices, and submit batches without returning per-vertex Python
objects. Retain Python for UI state/layout where appropriate. Compare same-input
pixels/geometry and CPU on C3 before making any speed claim.

The existing `PROFILE_RENDER` option uses cProfile's default elapsed-time timer
and exits the UI after the requested frame count. Do not enable it blindly on a
live drive, or interpret its cumulative times as pure CPU. Targeted thread-CPU
timing or an isolated replay can separate path/tire/text/conversion costs.

## Sensors and localization: native work already exists

Steady-state process CPU from this route, percentages of one core:

| Process | Total CPU % | User % | Kernel % | Placement |
| --- | ---: | ---: | ---: | --- |
| sensord | 11.31 | 6.33 | 4.98 | core1 |
| locationd | 33.05 | 31.17 | 1.87 | cores0..3 |
| UI | 33.48 | 28.38 | 5.10 | sampled core6 |

User CPU includes native library code; it is not synonymous with Python CPU.

sensord's interrupt/message loop and I2C object handling are Python, but GPIO
poll/read and I2C ioctl run in the kernel. Native sensor acquisition/packing could
reduce allocation/call overhead and jitter; simply rewriting userspace does not
remove bus/kernel costs. Both interrupt threads and their timestamp/validity
behavior must be preserved. No native sensor benchmark was performed.

locationd is Python orchestration around `PoseKalman` → `EKF_sym_pyx` → C++
`EKFSym`/Eigen and generated model functions. Core filtering, prediction/update
and rewind machinery already execute natively. Remaining candidates include
message decoding/sorting, per-observation array creation, repeated native-state
copies and Python↔C++ calls. The Cython wrapper copies state/covariance/results
to NumPy; the Python caller sometimes consumes only part of those returned data.
Profile those boundaries before a full daemon rewrite. Timestamp ordering,
rewind behavior, floating-point results and all validity limits must stay intact.

Optimizing these two services would primarily free little-core capacity in the
current placement. It does not directly free the UI's congested core6. The same
core's selfdrived/controlsd together use48.69% and merit separate profiling if
more core6 capacity is required, with greater behavioral-verification scope.

## Why comma selected Python

Comma's [UI rewrite issue33301](https://github.com/commaai/openpilot/issues/33301)
explicitly favors simpler dependencies/concepts than Qt, cross-platform support,
a small library and Python bindings; it does not claim Python is faster.
Its [0.10 release discussion](https://blog.comma.ai/010release/) similarly places
the Python CAN parser rewrite under maintainability and car-port simplification.
These support a development/maintenance rationale, not a measured performance
advantage for this heavily extended Carrot UI. They do not establish the exact
motivation or performance tradeoff of every sensord/locationd rewrite.

Primary library references: [Raylib](https://github.com/raysan5/raylib),
[Python CFFI bindings](https://github.com/electronstudio/raylib-python-cffi).

## Priority, with core7 excluded

1. Bound/distribute the identified detailed-memory accounting burst; validate PSS
   freshness and core4 peaks while preserving lightweight process telemetry.
2. Profile and batch/native-optimize UI overlays, outlines, vertex conversion and
   styled text. Preserve visible features; first target actual CPU cost.
3. Measure UI frame CPU, runnable wait and camera/model/control health at fixed
   placement. If placement is revisited, investigate the ineffective little-core
   mask and core4 only after its burst is addressed; no core7 trial.
4. Review locationd wrapper costs, sensord userspace overhead and control-loop
   costs independently, prioritizing measured savings over whole-language rewrites.

Private reproduction: `.analysis/archive/2026-10-03/casper-ui-review/`.
No on-device experiment, performance patch, native conversion or deployment
was performed in this review.

Subsequent authorized work: [paced memory sampling and native UI benchmark](casper_c3_ui_native_pss_20261003.md).
That follow-up implements telemetry pacing and keeps UI conversion experimental.
