# Mapbox navigation instruction serialization — 2026-10-19

## Failure and correction

An offroad test on the connected C4, base 18ca21501e, used the registered public
Mapbox token with the actual Carrot Web search/destination handlers, in-memory
Params, and public Seoul addresses. Search returned HTTP 200; Directions returned
Ok with five steps and 81 geometry points. No actual destination was set.

The nav branch's `np.cumsum` produces `numpy.float64` maneuver distances.
`RouteEngine.send_instruction()` failed assigning `allManeuvers`: the real
Cap'n Proto list-of-struct converter rejects that scalar type. Cast each distance
to Python `float` at the serialization boundary. Distance calculation, route
progress, guidance selection and model behavior remain unchanged.

The wip branch uses ordinary sums and did not reproduce this exception: all six
new regression cases pass before its change. Apply the same serialization
boundary conversion there without importing nav's route engine or model policy.

## Validation

- Six new cases call real `send_instruction()` and round-trip cereal bytes,
  covering current/past/future maneuvers, zero/negative distances and missing
  arrival banners. Nav: all six fail before and pass after the conversion.
- Nav related suites: 93 passed, nine native-rasterizer tests skipped on Windows.
  Wip: 22 passed. Desktop hardware/Params/messaging OS boundaries use the existing
  harness doubles; real cereal schemas/serialization are exercised.
- The exact modified navd source was copied only to a `/tmp` diagnostic path on
  C4. SHA256: 8bac8c4367641a32ebd525bb79519739a3d2ca086f0546a596b00d891fccecff.
  A separate process loads that source; operational navd was not replaced.
- Replay the captured real route through RouteEngine, Carrot's route selection
  and instruction bridge, route-relative renderer with live Mapbox tiles, and
  the actual MapInput client using a uniquely named native VisionIPC server.
  All 44 simulated positions (0–2150 m, 50 m samples) produce usable images and
  commands; received image pixels match the rendered bytes. Route steps 0–3
  are exercised; this is not a destination-arrival test.
- Fill declared NAV-RL-shaped host arrays `(1,1,256,256)` / `(1,47)` and verify
  finite commands and image equality. These are diagnostic input buffers, not
  an executing driving-model worker. Actual neural inference, eGPU/Jetson
  transfer, 20 Hz under onroad load and steering are not tested.
- Corrected-source native warm renders: 10.109–20.089 ms; IPC receipt plus input
  checks: 0.913–1.623 ms. These are bounded offroad measurements, not deadlines.
  Latest cold fetch reaches 537.116 ms; the earlier review reached 884.875 ms.
  Existing mapsd discards render work >=500 ms, so tile-download map gaps remain.
  No tile-fetch policy, CPU budget or freshness gate is changed by this fix.
- Stale-map, mismatched-route and unsupported-model guards pass. Production
  destination remains unchanged; no synthetic data was published on operational
  cereal/vision channels. Operational code/model/settings and process lifetime
  are unchanged; no vehicle install or restart is part of this correction.

Publish source, regression tests and these records to carrot-nav and carrot-wip
as separate single-parent commits. Tokens and private diagnostic files are not
included in the commits.
