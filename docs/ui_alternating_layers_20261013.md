# Alternating onroad display work and receipt-cached health checks

## Current status: restore alternating work with the real window target

The C3 report was clarified: the camera remains visible while overlays disappear
and reappear. The first mitigation withdrew the UI changes on both branches.
The subsequent code audit found a concrete target-restoration defect in the
withdrawn implementation, and the corrected implementation restores alternating
model/HUD painting while still compositing both layers every frame.

The old application set `render_framebuffer = 0` whenever its own recording
texture was absent. However, comma's `BeginDrawing()` starts drawing into its
internal `CORE.Window.rotated_fb`; `EndDrawing()` later presents that texture.
The camera reached this internal target, but refreshing a cached layer restored
FBO 0 and composited overlays there. Final presentation then replaced those
writes with the internal texture. Urgent frames bypass caching and draw into the
correct target, explaining intermittent reappearance. The previous offscreen
benchmark always supplied a known outer texture and therefore missed this path.

Source: [comma raylib rcore.c, pinned caa64e15](https://github.com/commaai/raylib/blob/caa64e15ac20a47804a8048029c921ac091fef12/src/rcore.c#L908).
This platform code is shared by the C3/C4 paths; a C4 road failure was not reported.

The application now captures the actual framebuffer **after** `BeginDrawing()`.
On devices it uses ES2 `glGetIntegerv(GL_FRAMEBUFFER_BINDING)` because the shipped
ES2 raylib wrapper returns zero. The function and output storage are bound once
at window initialization; the query runs once per unrecorded display frame, not
once per overlay. Recording/scaled targets retain their explicit IDs. Allocation
also checks framebuffer completeness before using a layer and otherwise keeps
direct drawing. Control, camera and warning cadence remain unchanged.

### Regression evidence

- Hidden desktop GPU reproduction uses the production application frame loop
  and layer code with a nonzero internal window target, matching comma's target
  structure. The simulated camera background stays visible.
- Before the correction, overlays are incomplete/missing in 37/40 unrecorded
  frames; the three urgent direct-paint frames restore both. Recording ON passes
  40/40, matching the earlier benchmark's blind spot.
- After the correction, all 80 paired recorded/unrecorded frames preserve both
  layers and the background. Both variants still paint each layer only 23 times
  in 40 frames, including urgent bypasses.
- A checked-in GPU regression covers C3 2160x1080 and C4 536x240 layouts, recording
  ON/OFF and urgent/reuse transitions: all 160 frames pass. Run explicitly with
  `CARROT_UI_GPU_TEST=1` and `test_overlay_window_gpu.py` in a desktop GPU context.
- 134 focused CPU-side UI tests pass, including nonzero window targets, ES2 query
  storage reuse, incomplete FBO fallback and recording transitions.
- The separate model-renderer suite cannot collect on this Windows environment
  because native `msgq` is absent; it is not included in those passing counts.
  Python compilation and focused lint/whitespace checks pass.

This reproduces and fixes a code-level cause with real desktop GPU pixels; it is
not a physical C3 panel/camera test. No new live-core utilization measurement or
vehicle installation is claimed. Historical performance results below remain
benchmarks, not a promise of a particular live-core reduction. The independent
messaging/selfdrived caches and later model/Jetlink changes are retained.
Private reproduction output lives under `.analysis/scratch/2026-10-13-ui-flicker/`.

## Historical experiment

The user requested lower CPU occupancy while retaining the existing driving
behavior. This change applies to the Python C3/C3X and C4 onroad UI, plus common
SubMaster checks and selfdrived's manager-process decoding. Publish it to both
maintained `carrot-nav` and `carrot-wip` with single-parent commits.

## Display behavior

- Camera presentation, alert widgets, driver monitoring display, input handling,
  C3 border indicators and C4 traffic-light widget keep the existing UI cadence.
- Model/path/radar drawing and the general HUD use two alternating transparent
  layers. At the normal 20 Hz UI cadence each steady layer is painted at 10 Hz;
  both layers are composited on every camera frame. This skips projection and
  drawing work, rather than comparing full model arrays before doing the work.
- A changed driving/traffic/lead/camera/calibration state or changed layout
  immediately repaints the layer. Alerts, blind spots, lane changes and touch
  frames bypass reuse. No retained layer older than 100 ms is used. Camera
  buffers are never cached by this helper.
- HUD state and graph sampling still run each UI frame. Short speed popups stay
  at that cadence. Moving path patterns and display-only filters account for
  elapsed time so halving the paint rate does not halve their speed. The driver
  may notice less fluid ordinary path/HUD motion during rapid changes; the
  camera rate itself is unchanged.
- A persistent receive-frame number replaces transient `sm.updated` checks in
  the cached model painters, so a message received on a reused frame is consumed
  at the next paint. C4 lamp/countdown expiry is evaluated before reuse.

Raylib's OpenGL ES 2 `rlGetActiveFramebuffer()` returns zero. The GUI therefore
records its actual frame-start target explicitly. Layer allocation and painting
restore that target, including when recording starts or stops during a frame.
The caller's projection, viewport, scaling and clipping remain in force. Layers
accumulate coverage alpha separately and composite premultiplied RGB. A failed
allocation falls back to ordinary rendering.

Two full-resolution RGBA textures plus Raylib depth attachments are retained:
roughly 36 MiB for C3's 2160×1080 layout and 2 MiB for C4's 536×240 layout if
attachments occupy four bytes per pixel. Actual driver allocation is not
measured. Resize and close release the old targets. This trades bounded graphics
memory and compositing work for fewer Python/geometry/drawing operations.

## selfdrived and messaging

The selfdrived loop, state machine, alerts, validity and liveness/age checks keep
their existing cadence. Only the manager process-name set is decoded again on
a new `managerState` receive frame; dynamic DM exceptions remain per-loop.

SubMaster caches frequency-window validity between message receipts. Those
windows and their construction-time bounds do not change without a receipt;
age-based liveness still runs every loop. Mutable ignore lists and simulation
mode are evaluated when used. Health predicates use ordinary short-circuit
loops instead of allocating generators and repeatedly calling the frequency
eligibility helper. No control, radar, model inference or DM rate is reduced.

## Validation and measurements

All device experiments ran in separate offroad processes on the user's C4.
Publishers and Params writes were intercepted; production files and settings
were not installed or changed. Only little cores 0–3 were online. Nice 19 was
used; selfdrived was pinned to core 0. No onroad forcing or core hotplug was used.

Final ABBA comparisons used identical recorded inputs and thread CPU time:

| Workload | Before | After | Reduction |
| --- | ---: | ---: | ---: |
| C4 UI, 536×240 | 18.374 ms/frame | 12.013 ms/frame | 34.6% |
| C3 UI code, 2160×1080, on C4 hardware | 32.918 ms/frame | 23.845 ms/frame | 27.6% |
| selfdrived, 100 Hz steps | 2.767 ms/step | 2.246 ms/step | 18.8% |

Each UI pass replayed 320 frames with 20 warmup frames excluded. C3 recreated
the view for each pass to equalize graph-history length; failing to do this
made later passes slower even without caching. C4 measured the main layout;
C3 measured its augmented road view plus input/state updates. These are
different workloads, not a hardware comparison. The camera was disconnected;
recorded calibration supplied the projection. GPU readback was outside timing.

The final selfdrived replay compared all 1,601 steps of state, enabled/active,
events/counters, alive/frequency/validity results and alert type/text: identical.
A real-message regression additionally checked 2,500 frames and 50,000 health
predicates with irregular receipt, complete dropout, invalid messages, changing
ignore lists and simulation transitions.

Fresh-layer versus direct GPU image comparisons:

- C4: mean RGB difference 0.0182/255, maximum 2, no pixel above 2.
- C3: mean RGB difference 0.00736/255, maximum 3, three of 2,332,800 pixels above 2.

Both images were inspected. These checks establish the tested offscreen
composition, not live panel/camera synchronization or full onroad GPU contention.
Native UI/messaging tests pass 193 cases. Two pre-existing strict float32
vectorization comparisons differ by at most 0.0000611 pixel on this ARM runtime;
the unchanged HEAD is checked separately for that baseline failure. The tests
and projection implementation are not weakened to hide the difference.
Another 77 selfdrived startup/readiness/state tests pass on the desktop with
native-dependency adapters. New cadence, warning/touch invalidation, allocation
fallback, recording lifetime, graph sampling and popup-duration checks pass.
The changed lines add no Ruff findings; compilation and whitespace checks pass.

These percentages are reductions in the measured workloads, not percentage
points of a live core's occupancy. Neither an 80%→70% core reduction, actual C3
hardware timing, onroad FPS nor model/DM co-load performance is established.
The original ONNX models and vehicle installation remain unchanged.

Private scripts, recorded inputs, screenshots and raw measurements are retained
under `.analysis/archive/2026-10-13-core6-audit/` and are not published.
