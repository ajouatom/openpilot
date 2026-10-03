# Casper C3: spread memory accounting and evaluate native UI work

Follow-up to [route analysis](casper_c3_ui_20261003.md) and
[optimization review](casper_c3_ui_optimization_review_20261003.md).
Source: `00001e83--386b0dd089`, segments 0/1, recorded on `076e5cf4`.

Actual C3 follow-up on `a0f1a004` is analyzed in
[the after-update comparison](casper_c3_ui_after_20261003.md): native activation
and PSS burst smoothing are confirmed, while steady driving UI remains about
13.8 Hz. Different stop/driving proportions preclude a whole-route A/B claim.

The user authorized spreading memory-accounting work and evaluating UI native
conversion. Core7 remains excluded. The user clarified that mean CPU 50% is an
aspirational headroom target, not a hard acceptance gate. Prioritize smoother
frames and lower bursts; do not throttle important services to satisfy a graph.
The user subsequently authorized C4 validation on device178 and shared C3/C4
function changes. The implementation now includes telemetry pacing and native
outline/ribbon submission, with the original Python fallback. The first desktop
prototype below remains separate from complete-frame measurements.

## Memory accounting change

The old proclogd refresh synchronously scanned every eligible process every
40 seconds. Approximately 45 processes refreshed together at three periodic
core4 spikes. Proclogd consumed 1.28/1.31/1.30 seconds of CPU in corresponding
two-second intervals, versus a normal median of 70 ms. Core4 reached 100% around
t=39/79/119 seconds. This is distinct from sustained UI contention on core6.

`system/proclog_smaps.py` now queues eligible process identities and samples one
process at a time, with one unbuffered read of at most 32 KiB per step. Each step
is followed by at least 5 ms of rest, extended to 19 times that step's thread
CPU time when larger. This targets at most 5% sampler CPU over work-plus-rest
cycles, not a limit on the whole daemon or core. Ordinary CPU/RSS/system-memory
snapshots remain at 0.5 Hz; their builder performs no detailed PSS reads.

Retain `smaps_rollup` where supported, preserving its anonymous/shared
proportional counts. Otherwise use chunked `smaps`, relevant to C3's 4.9 kernel
base. The actual selected file was not logged, so absence on this exact BSP is
an inference. Linux exposes Pss_Anon/Pss_Shmem specifically in rollup output;
unconditionally using smaps would lose those fields. See the
[kernel implementation](https://github.com/torvalds/linux/blob/master/fs/proc/task_mmu.c).
A rollup read still performs an indivisible process-wide kernel walk; a large
individual VMA can also make a smaps read expensive. Userspace pacing cannot
preempt a syscall or guarantee a whole-core peak below 90%. The change removes
the application-level all-process sweep and spaces the remaining work.

Completed scans become eligible again after 40 seconds; queueing can extend
that interval. There is no catch-up sweep. Publish only complete samples checked
against PID plus start ticks. Exited, reused and ineligible PIDs lose their cache.
A sample or unfinished scan older than 120 seconds is unavailable, represented
by zeros. Additive cereal field `memPssMonoTime` records scan-start monotonic
nanoseconds; zero means unavailable or an older log without this field. Older
logs may still have valid PSS with zero timestamp. Consumers must not interpret
unavailable zero as measured zero use. These are asynchronous estimates, not an
atomic system-wide memory snapshot.

Validation: 16 focused Windows tests pass; a real-Linux-procfs smoke test is
skipped on Windows. All 17 pass on the user's C4, including real procfs.
Coverage includes chunk boundaries, 45 queued processes, CPU-based pauses,
publication cadence across a five-second stall, expiration, PID reuse,
malformed/missing files, rollup retention and real cereal field assignment.
Ruff passes. C4 measurements are below; C3 still needs its own follow-up log.

## Native UI prototype: same Raylib, less Python boundary work

Cython/C++ prototype, strict floating-point flags. Keep the same Raylib line/text
functions and call order; move loops and argument setup across a single native
boundary. Also test native path projection and triangle-strip vertex interleaving.
The projection/text variants remain prototype-only. The subsequent production
change batches outlines and complete ribbon submission as described below.

Inputs: all 2,281 recorded valid model paths, with a fixed nominal C3-size
transform/clip rectangle rather than recorded per-frame calibration or full UI
state. All projected arrays match bit-for-bit for these inputs; 1,295 nonempty
polygons also match strip values exactly. Twelve outline scenes and four distinct
styled-text scenes match pixel-for-pixel in a 2160x1080 offscreen render target.
Text uses the default test font and ASCII labels, not production font fallback
or Korean text. This does not validate all possible inputs or drawing modes.

Desktop Windows x64, Raylib 5.5.0.4; device declared dependency differs.
Seven rounds per implementation; projection/interleaving have 20,000/100,000
operations per round, outlines/text 10,000/20,000. Medians below are elapsed
microseconds per operation. Thread CPU results are retained privately; Windows
CPU timing is visibly quantized for short calls. Render timings exclude frame
swap/pacing and measure submission, not complete GPU execution.

| Selected operation | Existing Python | Native prototype | Local ratio |
| --- | ---: | ---: | ---: |
| Path projection | 24.003 us | 1.821 us | 13.2x |
| Strip vertex interleaving | 13.208 us | 0.553 us | 23.9x |
| Polygon outline submission | 162.704 us | 3.590 us | 45.3x |
| Outlined/shadowed text submission | 6.640 us | 2.730 us | 2.4x |

These are selected-operation ratios, not a 45x UI or C3 FPS claim. Interleaving
returns a contiguous array instead of the current Python list; realizing its
benefit requires a drawing boundary that consumes that buffer directly. Full
polygon submission/binding conversion is excluded. Text excludes shared layout,
measurement and encoding from both variants. Existing text already uses raw
CFFI, which explains its smaller remaining opportunity.

The outline result is especially useful: the Raylib primitive is unchanged,
yet removing per-edge Python calls and Vector2 construction cuts submission
cost substantially. This implicates the calling pattern and conversion costs;
it does not establish Raylib itself as the bottleneck or compare NanoVG and
Raylib under equivalent complete scenes. Path mode0 with outlined colors in this
route uses the measured helper; lane outlines/labels are additional candidates.

## Expected frame benefit and next step

Segment1 drawing CPU is 16.66 ms/frame: overlay 7.85, HUD 4.97, border 1.77 and
camera 0.85 ms. If a complete implementation saved 25% or 50% of drawing CPU,
it would save approximately 4.2 or 8.3 ms/frame. These are planning scenarios,
not measured whole-frame gains. Actual savings depend on the share of time in
the tested functions; existing diagnostics do not resolve that share.

UI also records about 593 ms/s of runnable wait on core6. Reducing CPU should
help headroom as well as drawing time, but FPS will not scale with a kernel
benchmark ratio. Restoring more frames spends some savings on additional draws;
compare at a fixed target of 20 FPS.

Implemented first stage: native outline submission plus contiguous ribbon
construction/submission. `native_draw.py` validates the loaded Raylib Vector2/
Color ABI and obtains functions from that same loaded library. It preserves
draw order, colors, alpha, width, odd-point trimming and shader setup. Missing
extensions/bindings retain the Python path; `CARROT_UI_NATIVE=0` allows an
isolated comparison. C3 outlines, C4 lead rectangles and shared solid/gradient
polygons use it. C4 torque-bar polygons also use that shared helper. Projection
and styled text remain candidates, not part of the promoted native code.

The exact SConscript builds on C4 ARM, and both x86/ARM CI targets include it.
The first full-build CI exposed a missing source-level C++ directive masked by
the focused harness's `--cplus` flag. The directive is now explicit and the
harness no longer forces that flag, matching the production Cython invocation.
27 native/ABI/fallback/C4-call-order tests pass on Windows and C4. Existing
polygon and lane tests pass (4 and 11), as do 2 compact UI schema tests. Desktop
production-path pixel checks match all 32 solid/gradient/outline/combined scenes
at C3 2160x1080 and C4 536x240 sizes, including translucent and odd-point inputs.
This does not replace physical-display or complete-scene validation.

## C4 parked validation

User device178, dongle `07b62e389ed26c81`, kernel4.9.103, original source
`864355ef`; same affected source functions as incident commit. The user confirmed
ignition ON and Park. UI remains core6, controls/camera placement unchanged,
model/DM remain core7. No control/CAN/validity settings were modified. The full
setup has more little-core/DM work than the Casper route.

The first child-only restarts inherited preimported Python modules from manager.
Their missing new timing/PSS-age fields proved the changes were not active;
`paced_python`/`paced_native` are excluded. A fresh manager loaded the changes.
The first 50 ms minimum sampler pause was too slow: after roughly 68 seconds,
only 10 of 65+ eligible processes had completed samples. The final 5 ms minimum
retains CPU-based duty pauses and reaches 53 completed samples out of 65 eligible
processes; the remaining names are privileged system services. Reported sample
ages stay below 68.6 seconds in the final measured windows. Zero/unavailable
entries remain explicit; completeness is not inferred from the count alone.

Original baseline: 110 seconds. Final UI Python/native/Python windows: 85/85/65
seconds, discarding the first 10 seconds of each. The latter three keep the
same UI process and memory implementation, switching only the native path using
a temporary, once-per-second comparison file. Logged `native_draw` is 0/1/0.
The temporary switch is not production code. A/B/A reduces time-order ambiguity
but is still a changing camera scene, not identical-input full-UI replay.

| Measurement | Python A | Native B | Python A2 |
| --- | ---: | ---: | ---: |
| UI redraw Hz | 19.45 | 19.51 | 19.53 |
| Drawing thread CPU ms/frame | 9.157 | 9.054 | 9.255 |
| Model-overlay CPU ms/frame | 4.291 | 4.164 | 4.348 |
| UI process CPU %, one core | 26.59 | 26.45 | 26.92 |
| Whole core6 mean % | 81.6 | 81.8 | 82.3 |

Against the mean of the two Python windows, native drawing saves about 0.152 ms
(1.65%) and model overlay about 0.155 ms (3.60%). This is a small improvement in
this parked C4 scene; FPS and total core6 utilization do not show a material
improvement. It does not justify multiplying a whole UI by the 45x primitive
benchmark, or predicting the same small gain for the heavier C3 scene. The next
step should profile projection/text/HUD costs and a representative C3 workload,
not assume these first two kernels remove most UI CPU.

Memory accounting has a larger measured burst benefit. Original proclogd
two-second-interval CPU mean/max were 7.28%/66.55%; final means are 8.89-9.05%
with maxima 9.50%. Whole core4 peaks were 100% originally and 67/72/67% across
the final windows. Average daemon CPU rises approximately 1.6-1.8 percentage
points here; this is burst smoothing, not demonstrated total-work reduction.
The sampler's 5% duty budget excludes normal process telemetry work. Core4's
other workloads and per-syscall limits still prevent a universal peak guarantee.

All final windows have zero invalid carState/modelV2/livePose/cameraOdometry
messages and no road/wide/driver/model/DM frame-ID gaps. Camera SOF maximum is
58.637 ms; maximum CPU temperature is 59.8 C. Controls remain inactive/Park.
The read-only collector itself uses approximately 21% of one little core, with
the same settings in each phase; these are instrumented-load measurements.
Loaded driving, a physical C3 and another thermal regime remain untested.

The C4 SCons build and 44 combined native/PSS tests pass. The original device
files were restored against their saved hashes after the experiment. During
standard-launch recovery the existing automatic update/reboot path advanced
the device from `864355ef` to `076e5cf4`; the final runtime is therefore not the
original commit. Temporary comparison code is not left as a vehicle installation.
Repository changes are kept separately for the normal branch update path.
Post-recovery 25-second capture (first 10 seconds excluded) confirms UI19.47 Hz,
zero invalid monitored camera/model/pose/car messages, no camera/model/DM ID
gaps and no reported errors. Tracked device diff is empty and trial modules
are absent. This short check confirms restoration, not another performance trial.

Mean 50% and peak 90% are useful headroom directions, not reasons to alter
control timing or drop features. Current steady core6 is 91.7%, core5 67.5%;
core4 averages 38.3% but has the identified burst. These are separate improvement
opportunities. Broaden profiling where frame stability/load justify it, rather
than requiring every core to meet one number before accepting useful progress.

Private reproduction: `.analysis/archive/2026-10-03/casper-ui-native/` contains
sources, exact benchmark output and an index. Original rlogs remain on NAS;
raw logs and prototype binaries are not committed.
Private C4 evidence and scripts are in the sibling `c4-ui-validation/` archive.
