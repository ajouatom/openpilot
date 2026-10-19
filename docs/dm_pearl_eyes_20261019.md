# Pearl driver-monitoring eyes (2026-10-19)

The user selected the pearl cartoon-eye design and requested implementation for
both C3 and C4. The shared `DriverPreview` now draws two warm pearl eyes
(`#f4eddf`), dark pupils and small highlights directly on the existing onroad
view. The old camera rectangle, border and camera image are removed. Existing
C3/C3X and compact C4 placement remains unchanged; the widget is display-only.

## Data and presentation

- Existing `driverStateV2` left/right driver selection follows `isRHD`.
  Changing sides discards the previous expression immediately.
- Pupil offsets are a bounded, mirrored illustration of `faceOrientation`,
  smoothed with an 80 ms time constant. The model does not publish ocular gaze
  coordinates: this must not be described as precise eye tracking. Invalid or
  uncertain head pose centers the pupils.
- Each eyelid follows its own `leftBlinkProb` / `rightBlinkProb`, with rapid
  closing and softened reopening. There are no random or periodic decorative
  blinks in the production UI. Blink display requires finite probabilities,
  eye confidence above 0.65 and sunglasses probability below 0.9. These are
  presentation gates only; monitoring thresholds are not modified.
- Unknown eyes are gray short marks without pupils. `DM EYES?` distinguishes
  unreliable eyes from a tracked face; an individual unknown eye stays gray.
  Face loss is `DM FACE?`, stale/invalid data is `DM CHECK`. `DM FACE` means
  face recognition, not a claim of attention or safety.
- Existing distracted/lockout states use warm warning color and eyebrows with
  `DM ALERT` / `DM LIMIT`. Existing interaction monitoring/camera fallback is
  a steering-wheel symbol with `DM HANDS`, not simulated live eyes.
- Existing valid/alive/current-session/500 ms message checks remain. A model
  frame ID that makes no progress for 500 ms also clears the expression even
  if the same model frame is republished with a new message timestamp.
- Alerts, offroad and ordinary disengagement hide the character and reset its
  expression. `AlwaysOnDM` visibility is preserved. Closing is idempotent.

`DriverPreview` is now a plain `Widget`: it creates no driver-camera VisionIPC
client, camera shader, EGL image or video texture. The real monitoring model and
camera continue operating as before. Offroad camera setup/preview is unchanged.
There are no Params, model/runtime, process scheduling, monitoring policy,
engagement, control or sound changes.

## Validation

75 focused tests pass, including both C3 and C4 widget sizes, real cereal list
readers, head-pose bounds, independent blink/confidence gates, RHD selection,
stalled-frame recovery, freshness boundaries, warning priority, alert hiding,
camera-free construction and resource-free close. The focused UI/sound/HUD
regression run passes 180 tests, with the native IPC timeout case deselected on
Windows. Desktop adapters replace OS/IPC/Params and widget hosting, not the eye
presentation or tested sound logic. Ruff and whitespace checks pass.

Desktop Raylib renders exercise the actual production eye renderer for front,
left, closed eyes, lost face, warning, wheel fallback, stale data and alert
hiding at 2160 x 1080 and 536 x 240. Surrounding HUD elements are schematic;
the DM inputs are synthetic. Local render evidence and the reproduction script
are under `.analysis/dm-eyes-design/` and are not distributed.

This validates desktop layout and state handling, not physical C3/C4 display
readability, real blink/pose alignment, native device CPU savings or road use.
Branch publication is separate from installation on a vehicle. No vehicle
processes have been restarted or files installed for this change.
