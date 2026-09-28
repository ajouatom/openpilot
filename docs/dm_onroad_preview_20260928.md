# Passive onroad driver-monitoring inset

The September 28 request adds an automatically visible DM inset to both main
device UIs. It replaces the old onroad face-state icon, without changing the
offroad driver-camera dialog, monitoring policy, Params, model, camera process,
engagement, lockout, control outputs or scheduling.

## Placement and visibility

- C3/C3X: 260 x 180 logical pixels, at content-relative (40, 220), below the
  clock and above the speed/device panel. Position follows the content rectangle
  when the sidebar opens. The existing plot begins farther to the right.
- C4 (September 29 revision): 84 x 84 pixels at (width - 154, height - 96).
  At 536 x 240 this is (382, 144), 9 pixels right of D and 10 pixels before
  the 60-pixel right strip. Restore the confidence dot's full-height travel.
  Move the VISION card above the inset to (382, 56), and keep its right BSD
  label to the left and warning bar to the right. Compact status text is 11px.
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

## September 29 sound and layout verification

DM event stage 1 remains visual-only. driverDistracted2/driverUnresponsive2
with promptDistracted receive a final PCM gain floor of 0.7; stage 3 with
warningImmediate always receives gain 1.0, including user multipliers above 1.
The first audible stage keeps larger existing gains. Apply this after ambient
and user volume calculations, keyed by the event's alertType and sound together.
Navigation's reused promptDistracted asset does not inherit the DM floor.
Finishing a one-shot retains its original event identity; a new event using the
same WAV replaces that identity. Monitoring timers and policies are unchanged.

148 focused desktop tests pass, including PCM output from selfdriveState events
at seven volumes, both DM event families, shared-asset transitions and existing UI
regressions. The native IPC timeout test is excluded on Windows; adapters replace
IPC, Params and hardware imports, not the tested PCM/UI logic. Raylib captures use
the actual inset, NV12 shader, VISION card and confidence dot with synthetic inputs
and a schematic surrounding HUD. Camera, wheel, stale and alert states were checked
at both sizes. Evidence: `.analysis/archive/2026-09-29/dm-ui-sound/`.

No physical-device display or speaker measurement has been performed. Gain 1.0
means software unity gain, not guaranteed hardware loudness or sound pressure.
