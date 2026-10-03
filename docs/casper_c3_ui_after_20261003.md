# Casper C3 follow-up: a0f1a004 UI frame-rate complaint

Analysis only. No production code, placement, priority, device setting or vehicle
installation is changed by this follow-up.

## Finding

UI remains too slow: about 13.79 Hz during continuous driving. The new route's
overall average is lower than the previous route's 15.86 Hz, but the previous
route spends 43.3% of the comparison interval stopped. Its moving windows were
already 13.08-13.55 Hz. These two different drives do not establish an update-
induced driving regression, nor an identical-input performance improvement.
The complaint remains valid as an unresolved low-frame-rate condition.

The native drawing backend is active (`uiModel.native_draw=1` throughout the
steady diagnostic windows). PSS pacing also works on this actual C3: proclogd's
two-second CPU peak falls from 65.50% to 9.50%. Neither change resolves the
dominant UI/core6 scheduling contention.

An additional finding corrects the previous headroom interpretation: core4/5
whole-core accounting is inconsistent with elapsed time and task runtime.
deviceState and procLog both use `/proc/stat`; their agreement is not independent
confirmation of true idle capacity. In particular, core5's new 12% display must
not be interpreted as 88% available for UI.

## Sources and method

- Same vehicle: HYUNDAI_CASPER_EV, tici, serial ac960474, UnregisteredDevice.
- Before: `00001e83--386b0dd089`, segments0/1, clean `076e5cf4`.
- After: `00001e84--ae3755779e`, segments0/1, clean
  `a0f1a0048e490e1ab115d89eb89a9f757cd10f0a` in both initData records.
- Both use AGNOS19.8-carrot-bt1, the same 4.9.103 kernel and `isolcpus=6,7`.
- Full cereal decoding, not the reduced CAN schema. Time zero is each route's
  first carState; primary window is 10 <= t < 121 seconds, limited to available
  records. Exclude startup explicitly rather than mixing model initialization
  with steady frame performance. Runtime metric means are weighted by counts.
- FPS is the frequency of uiDebug emissions. This measures UI redraw progress,
  not an independent panel scanout/presentation capture.
- After segment SHA256: `dcf58f8303f9b6fcbb85bfa28d743fe4068e7fa34c156b075e97e7c281bdc4d8`
  and `b75541ca78c6e0638ac7c965918ce886661594f9a2f1b8f0ffeb2a7ef51c00d7`.

[Uploaded after route](https://upload.shind0.synology.me/routes/HYUNDAI_CASPER_EV%20UnregisteredDevice/00001e84--ae3755779e--0:2).
Private reproduction and the comparison figure are retained under
`.analysis/archive/2026-10-03/casper-ui-after/`; raw logs remain on NAS.

## Separate stopping from driving

| Window | Before UI Hz | After UI Hz | Before speed mean | After speed mean |
| --- | ---: | ---: | ---: | ---: |
| Entire steady interval | 15.86 | 13.79 | 25.7 km/h | 82.2 km/h |
| 10-20 s, both moving | 13.55 | 13.62 | 60.2 km/h | 77.1 km/h |
| 80-120 s, predominantly moving | 13.08 | 13.85 | 50.1 km/h | 84.6 km/h |
| Before-only stop, 35-75 s | 19.00 | no corresponding stop | approximately0 | — |

These are descriptive windows, not matched road scenes or a controlled A/B.
The new route has no steady stationary samples, versus 43.3% before. Its
ten-second bins stay around 13.4-14.2 Hz. The earlier moving bins reached12.1 Hz.

For the 80-120 s windows, frame interval mean/p95/max is
76.47/91.45/118.21 ms before and 72.20/89.17/100.25 ms after. The after route's
entire steady maximum is108.93 ms. These measurements do not show a new long
steady freeze; they also do not measure every aspect of perceived smoothness.

Relevant UI settings are unchanged: ShowPathMode/ModeLane=0, ShowPathColor/
ColorLane=20, ShowPathColorCruiseOff=8, ShowLaneInfo=1, CarrotTireTrajectory=1,
ShowPlotMode=0. CarrotVision remains1, DM remains0, EnableRadarTracks remains0.
Calibration and live-estimation records differ; this is not identical input.

Displayed scene complexity also differs. Left blindspot is asserted in17.5%
of old carState samples versus53.1% new; visible model lanes average2.51 versus
3.04. Lead distance averages21.4 versus43.9 m when present. Model path last-x
averages66.4 versus225.5 m; this is model geometry, not actual rendered length,
which is clipped by the drawing code. Tire ribbons are independently capped
at40 m and shortened for a close lead. Thus a stopped scene cannot substitute
for a driving rendering benchmark.

## What consumes the frame

All values below are CPU milliseconds per rendered frame, except the stated
wall time and scheduler wait. The last two columns use the same route-relative
80-120 s window, while still having different scenes/speeds.

| Metric | Before steady | After steady | Before moving late | After moving late |
| --- | ---: | ---: | ---: | ---: |
| Complete drawing CPU | 15.58 | 17.22 | 18.76 | 17.07 |
| Model overlay CPU | 6.76 | 8.34 | 9.96 | 8.19 |
| Path CPU, nested within overlay | 2.78 | 4.30 | 5.49 | 4.25 |
| Lanes CPU, nested within overlay | 2.85 | 2.45 | 2.68 | 2.37 |
| Blindspot CPU, nested within overlay | 0.35 | 0.75 | 0.98 | 0.73 |
| HUD CPU | 4.99 | 4.98 | 4.90 | 5.01 |
| Drawing wall time, ms | 40.11 | 48.03 | 51.97 | 46.91 |
| Main-thread runnable wait, ms/s | 584.5 | 601.7 | 609.2 | 602.4 |

Do not sum the nested path/lanes/blindspot rows with the model total. The
wall-minus-CPU difference includes descheduling and possibly blocking; it is
not itself a pure scheduler measurement. The separately logged scheduler wait
is strong evidence of CPU contention.

The new UI main thread gets about28.27% of one core and spends601.75 ms/s
runnable but waiting. Its55 procLog last-CPU samples all report core6, nice19,
SCHED_OTHER priority39. Source permits cores0,1,2,3,6, checked for all threads,
but sampled processor is not a full affinity or migration trace. This route
again does not demonstrate useful distribution onto the little cores.

Core6 also hosts selfdrived26.95%, controlsd23.73%, and camerad11.17% process
CPU. UI process total is31.48%, including its workers; do not assume every
worker's runtime belongs to the main thread's sampled core. Controlsd rises
about2 percentage points from the earlier route, while camera/selfdrived are
similar. Carrot web server rises14.54 ->24.60% and carrot_man19.55 ->24.29%,
mostly sampled on little cores. These are workload differences, not proven
effects of the new UI code or proof of a particular connected web viewer.

The native outline/ribbon calls do not cover complete projection, path-end
labels, tire geometry, font/layout, HUD or all renderer update work. Path CPU
includes tire trajectory and the path-end overlay; it cannot be assigned to
the native polygon submission alone. Raylib remains the same renderer.

At this route's current allocation, main-thread CPU per frame is about20.5 ms
(28.27% /13.78 Hz). If that allocation stayed fixed,20 Hz would allow about
14.1 ms/frame: approximately6.4 ms less total main-thread work would be needed.
This is a budget illustration, not a predicted achievable gain or a guarantee
that unchanged scheduling/GPU behavior would permit20 Hz.

## PSS pacing result and the misleading core5 display

Proclogd CPU mean/max across successive steady snapshots is6.75%/65.50% before
and9.16%/9.50% after. Average cost increases about2.42 percentage points while
the large synchronized scan disappears. Forty-six final entries have explicit
completed-scan timestamps; observed ages stay below62.63 seconds. This is not
a claim that every system process is readable or sampled.

UI scans start at6.395 and67.488 s, first published complete by9.510 and69.519 s.
The latter publication bounds the completion time, not an exact read interval.
Low UI FPS persists before and after this scan, rather than appearing only
during a single PSS sweep. These observational logs cannot exclude every
indirect memory-accounting effect, but they do not show the old periodic burst
as the cause of sustained low FPS.

| Core | Reported mean before | Reported mean after | Reported after max |
| --- | ---: | ---: | ---: |
| 0 | 55.54% | 60.10% | 83% |
| 1 | 57.68% | 62.00% | 82% |
| 2 | 52.33% | 55.49% | 79% |
| 3 | 51.28% | 55.48% | 77% |
| 4 | 38.28% | 25.25% | 42% |
| 5 | 67.50% | 12.12% | 29% |
| 6 | 91.70% | 92.18% | 96% |
| 7 | 21.58% | 19.44% | 32% |

These are recorded `/proc/stat`-derived percentages, not independently measured
headroom. In the new108.02 s procLog interval, core5's total counter advances
only60.71 s, including7.58 s busy. Before, its total advances162.25 s during
110.00 s elapsed. Meanwhile card runs100 Hz and its scheduler runtime is
45.72% before/45.19% after, with last-CPU5 in every steady snapshot; its thread
CPU is4.49/4.44 ms per100 Hz cycle. The apparent core5 load collapse is not a
corresponding drop in card work. Core4 also disagrees with summed pinned task
runtime and its counter total drifts from elapsed time.

Linux documents that tick-based CPU-state accounting can misrepresent periodic
workloads: [kernel CPU-load documentation](https://cdn.kernel.org/doc/html/latest/admin-guide/cpu-load.html).
That is a plausible class of explanation, not proof of this BSP's exact fault.
Kernel configuration, per-thread affinity, high-resolution runtime and IRQ
accounting would be needed to resolve it. Do not move UI to core5 based on12%,
or describe the mean/max90% targets as verified from these counters alone.
Core6's counters are much closer to elapsed time and its high runnable wait
independently supports the contention diagnosis. Core7 remains excluded.

## Other checks and next measurement

After t=10 s, road/wide/driver cameras and modelV2 remain approximately20 Hz,
with no frame-ID gaps. livePose has no bad inputs/sensors flags; camera/model/
pose/carState validity stays true. Highest steady camera SOF interval is55.992 ms.
CPU maximum temperature is64.0 C versus61.4 C before; all thermal states remain
green. Clock-frequency history is absent, so this is not proof of no frequency
variation. GPU usage reports3% throughout both routes and is too coarse to
exclude GPU/driver stalls. Memory usage stays about62%, without obvious growth.

Both routes separately contain an initial camera synchronization timeout,
model-history skips and initialization invalidity in roughly the first8 seconds.
They precede the steady comparison and are not new evidence of a sustained
post-update camera failure. Both also log `stats dir full` around57/117 s;
the after filesystem still has about31.6% free. This recurring message does
not establish disk exhaustion or the UI root cause.

Next useful work is a fixed-input C3 renderer comparison and finer timings for
path construction, path-end text, tire geometry/gradients and HUD layout/text,
followed by selective native batching/caching where measured. Any CPU placement
trial must first resolve the core4/5 accounting discrepancy and use thread
runtime plus camera/control/model timing, keeping core7 excluded. The present
logs do not justify a rollback, a blind migration or a claim that UI is fixed.
