# Passive onroad driver-monitoring inset

The September 28 request adds an automatically visible DM inset to both main
device UIs. It replaces the old onroad face-state icon, without changing the
offroad driver-camera dialog, monitoring policy, Params, model, camera process,
engagement, lockout, control outputs or scheduling.

## Placement and visibility

- C3/C3X: 260 x 180 logical pixels, at content-relative (40, 220), below the
  clock and above the speed/device panel. Position follows the content rectangle
  when the sidebar opens. The existing plot begins farther to the right.
- C4: 56 x 64 pixels, at (width - 58, height - 68), in the 60-pixel right strip.
  While visible, reserve its bottom 72 pixels from the confidence dot's travel,
  including its black masking ring and disengagement animation. The top traffic
  light keeps its existing position and priority over the confidence dot.
- Visible onroad while selfdriveState.enabled or AlwaysOnDM is true; hidden
  during any rendered alert (including C4 alert fade-out), offroad or ordinary
  disengagement. This reuses existing monitoring visibility inputs; no new toggle.
- External cluster camera suppression continues to suppress the road/model view;
  the device DM inset remains available, like the retained device HUD.

## Camera and state handling

`DriverPreview` consumes the existing driver VisionIPC stream, nonblocking and at
most 10 polls/second. It never instantiates DriverCameraDialog, sets
IsDriverViewEnabled, publishes selfdriveState, resets DriverTooDistracted, or
modifies any other monitoring state. Its textures/shader are closed with the
parent view. Hidden/offroad transitions discard cached frames and reconnect.

Both monitoring and driver-model messages must be valid, alive, received in the
current onroad session and less than 0.5 seconds old. The camera's actual SOF
timestamp must also be positive, not future-dated and less than 0.5 seconds old.
Stale data blanks the face instead of leaving a reassuring frozen image.

The inset uses a single mirrored crop around the selected LHD/RHD face, using
the existing driver-preview's approximate face projection. Invalid/missing face
coordinates show a fitted full image. This is a display projection, not a change
to model input or driver gaze assessment. Real vehicle framing still needs review.

Status labels: DM FACE (face tracked; not proof of alertness), DM FACE? (face not
detected), DM ALERT / DM LIMIT (monitoring warning / lockout), DM HANDS (interaction
monitoring), DM CHECK (waiting/stale data). With unavailable camera and fresh DM,
the inset draws a steering-wheel symbol. Unknown DM never claims a camera-free
fallback. With an available camera, the current wheel policy can retain the live
face while displaying DM HANDS.

## Validation

Desktop tests cover state precedence, validity/freshness, nonblocking polling,
stalled/old/future camera frames, camera fallback/recovery, RHD selection, lost-face
full view, parent-relative geometry, alerts, AlwaysOnDM, disengagement, offroad and
idempotent cleanup. Existing HUD, cluster suppression and UI telemetry tests are
included. The cluster AST test was already incompatible with timing.call on HEAD;
it now follows the instrumentation callback while checking the same suppression.
All 86 focused desktop tests passed; lint and the user-docs scope validator passed.

Desktop Raylib renders exercise the actual inset and CameraView NV12 shader with
synthetic image data at 2160 x 1080 and 536 x 240. Surrounding HUD elements are
schematic placement references, not captured vehicle screens. Camera, wheel,
stale and alert states were rendered for each size. Evidence and reproduction
script: `.analysis/archive/2026-09-28/dm-preview/` (local, ignored).

These checks do not establish physical C3/C4 camera cropping, EGL zero-copy
behavior, display readability in the vehicle, or additional CPU/GPU load during
driving. Frame binding uses the existing CameraView per-draw EGL binding path.
