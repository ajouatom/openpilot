# C3/C4 UI reuse and projection batching — 2026-10-03

The user approved three follow-ups together: finished-label image caching,
larger native geometry boundaries, and reuse of unchanged projected geometry.
This follows `f5b056c397` (native text layout reuse) and `6be41624` (geometry
kernels). No CPU placement, priority, control/model/camera or validity policy
changes are included.

## Implementation

- C3/C4 styled text shares a bounded complete-label texture cache. Text, absolute
  subpixel position, size, font identity, colors, outline and shadow participate
  in the key. A changed value draws immediately with the existing native text
  path; it never substitutes an older label. Plain text retains its layout cache.
- Admission requires eight consecutive observed frames. Ink accumulation,
  coverage accumulation and conversion to a finished image run in separate
  frames. Only one build stage runs per frame; disappearance cancels the job.
  Shader compilation happens during window setup, before normal rendering.
- The finished image uses an ordinary alpha-blended quad, preserving Raylib's
  GPU batching. A preliminary two-pass compositing candidate was slower on C4
  (roughly 6–8 ms for the six-label test) and was discarded.
- Raylib's ordinary alpha mode applies source alpha to the alpha channel too.
  Regrouping layers preserves visible RGB with small 8-bit rounding differences,
  but cannot reproduce that intermediate alpha with one ordinary quad. Therefore
  complete-label caching is limited to direct, unscaled display rendering.
  Recording, burn-in/scaled intermediate targets and transformed text use the
  original primitives. Scissor clipping remains active on cached draws.
- At most 64 label images / 524,288 retained pixels, about 4 MiB with depth,
  plus two temporary targets of at most 65,536 pixels each. Pending observations
  are bounded to 128. Cleanup precedes font/context destruction. Missing native
  support, unfamiliar ABI or a shader/build failure leaves original text usable.
- C3/C4 ribbon preparation now fills both sides directly in a single array,
  including forward filtering and the C3-only interpolated distance endpoint.
  C3 sampled-path projection also crosses the native boundary once. NumPy still
  owns interpolation and matrix products, preserving their rounding/edge cases.
- Ribbon projection, C3 path sampling and sampled-path projection each use a
  content-keyed LRU: 64 entries / approximately 512 KiB each. Full array bytes,
  dtype, shape, transform, clip rectangle, offsets, widths, endpoint distances
  and inversion policy participate. In-place mutation invalidates the result.
  Returned cached arrays are immutable; colors, warnings and animation still
  execute each frame. No model timestamp alone is treated as a complete key.
- After 64 consecutive misses, geometry reuse probes only once per 16 calls;
  other calls immediately use the native calculation. Any hit restores normal
  lookup. This avoids paying full cache-key/storage costs for continuously
  changing model inputs, without delaying a new coordinate or frame.
- `CARROT_UI_TEXT_TEXTURE=0` disables finished labels;
  `CARROT_UI_GEOMETRY_CACHE=0` disables geometry reuse. Existing native/text
  switches remain available. `uiModel`/`uiC4` include current-frame texture hits
  and cache-build CPU time; build work happens outside the nested draw timer.

## Validation and measurements

Desktop: 114 native/cache tests and 36 locally runnable UI regression tests.
Tests cover old/missing backend fallback, C3/C4 endpoint policy, float32/64,
NaNs, repeated interpolation nodes, clipping/inversion, read-only strided input,
cache mutation/size, new text/style/position/font, frame-stage admission and
cancellation, recording/scale/transform fallback and resource cleanup.

Actual desktop and C4 offscreen Raylib comparisons use three fonts, opaque and
translucent styles, fractional coordinates, clipping, four backgrounds and
immediate text changes: 48 comparisons on each platform. Cached visible RGB
differs by at most 4/255 per channel from repeated original layers; this is **not
bit-identical rendering**. Changed uncached values match exactly. Intermediate
alpha is intentionally excluded from this RGB comparison and handled by the
production FBO fallback described above.

The C4 test uses the installed font atlases and Raylib 6.0.0.1.post98, a separate
EGL context on the render node, cores0..3 and nice19. It does not acquire display
master, restart UI, replace device source/binaries or alter saved settings.
Results are medians of five rounds and represent synthetic repeated inputs,
not full UI frames or driving FPS.

| Six styled labels | Existing native layouts | Finished-label reuse |
| --- | ---: | ---: |
| Inter | 0.929 ms | 0.624 ms |
| Pretendard | 0.813 ms | 0.563 ms |
| KaiGen Korean | 0.782 ms | 0.654 ms |

| Both tire ribbons | CPU time |
| --- | ---: |
| Previous native geometry | 380.3 us |
| Batched native preparation, changing/uncached calculation | 217.4 us |
| Identical input reuse | 112.4 us |

The table isolates native work with caching disabled for the middle row. A
follow-up continuously changing-input test exposed an important distinction:
unconditional cache misses took 427 us versus the previous native 394 us, an
8% regression. That candidate was replaced by the adaptive probe policy above.
In the final changing-input comparison, previous native was 390.7 us, batched
calculation with caching disabled 222.4 us, and enabled adaptive caching with
continuous misses **249.9 us (36% lower than previous native)**. Identical-input
hits were 117.7 us. Seven rounds of 4,000 two-ribbon operations include cold
cache resets and input mutation; these remain isolated CPU measurements.

For 108 measured label-build stages after shader initialization, median CPU was
1.450 ms and maximum 2.191 ms. The initial unpartitioned prototype's shader/build
cost reached 19.24 ms; moving shader setup to window initialization and splitting
the three build stages addresses that measured concentration. These samples are
not a hard execution-time or per-core utilization bound.

Whole-frame C3/C4 scheduling waits, loaded driving p95/FPS and the user's average
50% CPU target are not validated by this test. Geometry reuse helps only when
all inputs match, and label admission/build costs must amortize over repeated
draws. C4 remains on its original `6be41624` installation after isolated testing.
Private scripts/results are archived under `.analysis/archive/2026-10-03/ui-reuse/`.
