# C3/C4 path and tire geometry batching (2026-10-03)

The user requested optimization of path and tire trajectory calculations,
including the corresponding C4 functions. The earlier native draw change
only batched Raylib outline/triangle-strip calls; it did not move these
geometry calculations. See `casper_c3_ui_after_20261003.md` for the preceding
Casper frame-rate analysis.

## Implementation

- The existing optional `_draw_native` Cython/C++ module now builds left/right
  coordinate buffers and performs perspective division, paired screen clipping,
  hill inversion filtering and ordered vertex packing in native loops.
- C3 and C4 `_map_line_to_polygon` use one shared `native_geometry.project_ribbon`.
  C3 retains its interpolated distance endpoint, start index and lateral shift;
  C4 retains node-only endpoints. This includes both C3 tire ribbons and C4
  path/lane/road-edge geometry.
- C3 sampled path ribbons use native side construction and clipping. Existing
  two-stage NumPy interpolation, including repeated/reversed x nodes, remains.
- Shared dashed-lane clipping is one native batch with independent dash
  boundaries; shared blindspot barriers use native clipping/hill filtering.
- NumPy matrix products and the barrier's original three-term projection stay
  unchanged. These were already native operations; retaining their precision
  and operation order avoids moving clipping boundaries through rounding.
- Float32 constant offsets, float64 variable-width path calculations, inclusive
  clipping, near-zero depth rejection, vertex order, tire widths/distances,
  colors, labels and draw order are preserved. Strict floating-point compiler
  flags remain. No scheduling, core7, camera/control/model or validity change.
- `CARROT_UI_NATIVE=0`, missing extensions and the previously shipped extension
  lacking geometry functions select the Python fallback. `uiModel` and `uiC4`
  report `native_geometry`; C3 also separates `path_geometry`, `path_labels`
  and `tire` CPU/wall times within the existing `path` total. Nested times must
  not be added again to the parent.

## Equivalence and build validation

The uploaded `00001e84--ae3755779e` segments 0/1 provided 2,269 valid 33-point
model paths. The comparison used recorded calibration and camera intrinsics
with a reconstructed full-screen 2160x1080 road-camera viewport, not recorded
per-frame UI transforms. Both implementations received identical inputs.
The exact pre-change Python functions were extracted before editing.

All 36,304 nonempty polygons matched exactly: 4,538 tire ribbons, 2,269 C4
shared projections, 2,269 sampled path ribbons, 24,959 dash polygons and 2,269
barriers. This checks geometry, not actual display pixels or driving response.

Desktop validation passes 50 native drawing/geometry tests, including float32/
float64, read-only/strided arrays, clipping/depth boundaries, NaN/infinity,
empty inputs, malformed shapes/dash lengths, hill filtering, both renderer
entry points and missing/older extension fallback. Another 21 existing path,
lane and diagnostic tests pass with the native backend enabled and disabled.
New/changed helper and test files pass Ruff. Existing renderer-wide Ruff
findings predate this change. The full renderer test cannot import `msgq` on
this Windows environment; this is not a passing full UI integration test.

C4 178 builds the exact production UI SConscript in an isolated temporary
directory, with the `.pyx` C++ language directive and no harness `--cplus`
override. No running UI/manager source or affinity is changed. The device's
venv lacks pytest, so device checks use the independent original/new function
comparisons in the benchmark; automated native tests also run in x86/ARM CI.

## C4 isolated calculation measurements

These are median **thread CPU microseconds per call**, five rounds of 3,000
calls, with warmup excluded. The separate benchmark uses 100 deterministic
synthetic 33-point paths, nice19 and cores0–3, while the ordinary device runs.
This is not the UI's core6 or a complete frame benchmark. The C4 row measures
its shared function, using the same synthetic viewport for comparison.

| Calculation | Previous Python | Native | Speed ratio |
| --- | ---: | ---: | ---: |
| Both tire ribbons | 1,055.9 µs | 386.5 µs | 2.73× |
| C4 shared ribbon | 532.7 µs | 112.2 µs | 4.75× |
| Sampled main-path ribbon | 398.1 µs | 136.9 µs | 2.91× |
| One dashed-lane batch | 745.4 µs | 207.5 µs | 3.59× |
| One blindspot barrier | 546.2 µs | 163.2 µs | 3.35× |

The corresponding recorded-input desktop calculations improve approximately
2.5–6×; Windows thread CPU timer resolution limits the precision of those
short measurements. Neither result is a whole-UI speedup or an FPS claim.

The preceding Casper log still showed substantial UI runnable wait on a busy
core6. This reduces geometry compute cost; it does not prove that the current
~13.8 Hz UI reaches 20 Hz, that all cores remain below 90%, or that average
load reaches the user's aspirational 50%. A subsequent loaded C3 log with the
new stage diagnostics is needed to measure that outcome.

## Evidence

Private reproduction scripts, original function snapshots, build logs and
summaries are retained under `.analysis/archive/2026-10-03/ui-geometry/`.
Raw route logs and extracted coordinate data are not committed or sent to C4.
Automatic approval review rejected the proposed recorded-coordinate transfer;
the device package was narrowed to source code and synthetic tests.
