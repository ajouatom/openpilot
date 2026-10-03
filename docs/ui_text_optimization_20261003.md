# C3/C4 text, shader and recording cost reduction (2026-10-03)

The user authorized implementing and testing additional UI optimizations after
the shared native path/tire geometry change, 6be41624. This change reduces work;
it does not change CPU placement, scheduling class/nice, control/model/camera
policies, display refresh targets, warning freshness or draw order.

## Changes

- Shared styled text submits its border/shadow/body from one native call.
  More significantly, a bounded native cache reuses glyph selection and layout.
  The same installed Raylib `GetCodepointNext` and `GetGlyphIndex` are used for
  cold labels. Rendering submits the same `DrawTexturePro` primitives in the
  same order, with the original intermediate float32 rounding. There is no
  offscreen label texture, changed alpha composition, delayed label update,
  shader outline approximation or added GPU memory allocation.
- The existing global text-size/font-fallback wrapper also uses that cache for
  ordinary text, including nonzero/negative letter spacing. C3 and C4 share it.
  Position, color and animation are applied afresh on every draw. A changed
  font, size, spacing or string selects a different layout immediately.
- Cache bounds: at most 256 layouts / 8,192 drawn glyphs; strings over 512 UTF-8
  bytes and multiline strings use original `DrawTextEx`. Multiline fallback
  preserves Raylib's mutable global line spacing. Font/texture/pointer ABI is
  validated against the actual CFFI binding before enabling the backend.
  Missing/old extensions or unsupported ABI retain Python drawing.
- Width measurements now use full tuple keys rather than bare hashes, with an
  LRU bound of 2,048 labels, excluding strings longer than 512 characters.
  Both caches clear before application font resources are unloaded.
- The polygon shader retains its last uploaded uniform values. Equal colors,
  stops and coordinates no longer require repeated CFFI/GL uploads. Mutable
  values are copied into keys; draw order and shader begin/end remain unchanged.
- Recording-only render targets are released at the next frame boundary after
  stopping. Scaling, burn-in and CLI recording retain their targets. A target
  snapshot pairs begin/end correctly even when a widget starts/stops recording
  during the frame. An already-full encoder queue skips GPU readback and copy.

For isolation: `CARROT_UI_TEXT_NATIVE=0` disables only the new text backend;
`CARROT_UI_TEXT_CACHE=0` retains native styled batching but uses original text
layout. The existing `CARROT_UI_NATIVE=0` still disables drawing/geometry native
backends. `native_text` activation is included in C3 uiModel / C4 uiC4 timing.
These are diagnostic environment overrides, not new user settings.

## Why layout reuse matters

The current Raylib text path resolves each character to a glyph and computes
its quad. Styled text can repeat this up to ten times. The C4 installed Korean
font contains 12,063 glyphs. Merely moving the outer Python loop to native code
retains most of that work; reusing the layout removes repeated searches and
layout calculation while preserving the exact submitted geometry.

Reference: [comma Raylib text implementation](https://github.com/commaai/raylib/blob/master/src/rtext.c).
Runtime behavior was additionally compared with the actual installed C4 binding;
the moving upstream source alone is not treated as its pinned build identity.

## Verification

- Windows native build; 81 native geometry/drawing/text ABI and primitive-stream
  tests. Text coverage includes Korean, unknown glyphs, space/tab, embedded NUL,
  fractional coordinates, border/shadow order, alpha, spacing, cold/warm cache,
  eviction bounds, font identity and optional-backend fallback.
- 47 locally runnable geometry, text-wrapper, shader, recording lifecycle and
  timing regressions. The complete renderer import needs Linux `msgq`; the full
  build/UI suite and native x86/ARM suites are included in GitHub CI.
- Hidden desktop rendering and separate offscreen C4 EGL rendering compare
  the original, native-batch-only and cached paths. Three fonts, three scales,
  translucent text/border/shadow, scissor clipping and plain text with spacing:
  21 comparisons per platform, zero differing pixel channels on C4.
- C4 uses comma-deps-raylib 6.0.0.1.post98; desktop uses Raylib 5.5.0.4.
  The ARM build uses the strict floating-point flags. The offscreen test opens
  only `/dev/dri/renderD128`; it does not acquire display control or replace the
  running UI. Test process runs on cores0..3, nice19.

An actual UI startup check caught an annotation incompatibility: pyray exposes
`Color` as a factory, so evaluating `Color | None` fails. The original `Optional`
annotations are retained, with an import regression test. This was corrected
before promotion; the failed startup is not a valid performance sample.

Final C4 offscreen results, six styled labels per operation (median thread CPU):

| Installed font | Python | Native loop only | Native cached layout |
| --- | ---: | ---: | ---: |
| Inter Medium, 219 glyphs | 1.470 ms | 1.189 ms | 0.908 ms |
| Pretendard Semibold, 226 glyphs | 1.459 ms | 1.225 ms | 0.909 ms |
| KaiGen Gothic KR Bold, 12,063 glyphs | 7.188 ms | 6.893 ms | 0.934 ms |

These measurements are on little cores, with live vehicle processes running,
and include offscreen target begin/end. They are not a full UI frame, a core6
measurement or proof of a corresponding FPS gain. CPU frequency/load may vary.
The Korean test deliberately includes Korean labels and is not a measured
distribution of every label in a driving frame.

The two-polygon desktop shader test renders identical pixels and reduces median
wall time from 28.26 to 11.00 microseconds per pair. This is a targeted benchmark,
not a measured whole-frame saving on C3/C4.

## Actual parked C4 OFF / ON / OFF comparison

Fresh valid Park, zero raw speed, inactive/disengaged controls and onroad state
were checked. After a fresh manager loaded the candidate, all three 65-second
windows used the same UI PID (88368), omitting the first ten seconds per window.
`native_text` was confirmed as 0 / 1 / 0 in the actual uiC4 diagnostics;
`native_geometry` and `native_draw` remained 1 throughout. Shader/recording/cache
bound changes were held constant, so this comparison isolates text optimization.

| Metric | OFF A | ON B | OFF A2 |
| --- | ---: | ---: | ---: |
| uiC4 render CPU / frame | 7.784 ms | 6.136 ms | 7.829 ms |
| UI main-thread CPU, one-core equivalent | 22.61% | 19.64% | 22.73% |
| uiDebug draw elapsed mean | 16.53 ms | 13.35 ms | 17.03 ms |
| uiDebug draw elapsed p95 | 24.54 ms | 24.81 ms | 24.22 ms |
| UI rate | 19.49 Hz | 19.59 Hz | 19.54 Hz |
| Core6 total mean, /proc/stat | 78.95% | 75.11% | 78.38% |

Render CPU falls about 21.4% versus the two OFF windows' mean; total UI thread
CPU falls about 13.4%. The reversal strengthens attribution to text processing.
This parked scene was already near the 20 Hz cap. Runnable wait remained high
(about 542–548 ms/s), and p95 draw elapsed did not improve, so do not describe
this as eliminating scheduling stalls or proving a C3 driving FPS fix.

Core means during ON for cores0..7 were 72.69 / 69.34 / 72.03 / 80.58 /
47.33 / 63.82 / 75.11 / 48.27%. Collection itself used about 12.3–12.4% of one
little core, held constant across the three modes. These are short parked
samples, not new CPU placement recommendations or guaranteed utilization caps.

All three measured windows have zero invalid monitored UI/model/DM/pose/camera
messages, zero observed camera/model/DM frame-ID gaps, and no camera SOF gap over
75 ms. Startup/restart intervals are excluded from these statements. Original
device files were restored byte-for-byte, tracked Git status was clean at
6be41624, and the standard launcher returned to healthy UI/model/pose readiness.

An earlier same-PID trial is excluded: the original manager had preloaded the
old UI modules, so UI-only restarts did not install the candidate. The missing
activation metric caught this. Failed startup/recovery intervals are retained
only as debugging evidence, never as before/after performance data.

## Remaining scope

GPU outline shaders, complete label texture caching and asynchronous geometry
workers are not enabled. Their visual/state/synchronization tradeoffs remain
separate from these exact-geometry changes. Per-core averages of 50% remain an
aspirational target; this patch neither enforces utilization caps nor guarantees
that every sampled maximum stays below 90%. Loaded C3 driving remains necessary.

Private scripts and numeric results are retained under
`.analysis/archive/2026-10-03/ui-text/`; no vehicle captures/settings are tracked.
