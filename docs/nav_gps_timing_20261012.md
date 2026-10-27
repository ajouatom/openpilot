# External navigation timing and GPS heartbeat correction

All work in this conversation belongs to `carrot-nav`; commit and push only this branch. Preserve unrelated workspace changes. Raw vehicle recordings, coordinates, identifiers and credentials remain private.

## Current input path

The latest drive contains five moving/full-log segments. The navigation map prefers Carrot's external-navigation/phone position and falls back to a usable device GPS fix. All 2,996 recorded `gpsLocationExternal` messages in these segments have `hasFix=false`; this service is the device's UBlox source, not the phone navigation protocol. External navigation supplied the usable position in this drive.

Carrot extrapolates position from the last received coordinates using **vehicle CAN speed**, not the delayed external navigation speed. It currently holds the external bearing between fixes; the old bearing-offset block does not incorporate pose. The model map projects the result onto the route and uses the route's +/-50 m chord heading, as in the upstream rendering contract. Directly rotating this image to pose yaw would change that contract. Pose is potentially useful for estimating ego displacement between position updates, without replacing the model map's route heading.

## Confirmed adapter bug and correction

The `carrotNavi` service republishes its latest items on a 2 Hz heartbeat. `_update_carrot_navi` previously reapplied the vehicle item on every heartbeat, even with the same item sequence and original receipt time. `_apply_carrot_navi_vehicle` reset `last_calculate_gps_time` to the current time. Thus the already extrapolated position could jump back to the unchanged GPS anchor. The map tracker's monotonic progress can turn that backward jump into a temporary hold rather than visible backward map motion. Neither behavior represents a new GPS measurement.

For example, one moving interval contains a fresh vehicle receipt around monotonic 235.134 s, followed by repeated application of its identical coordinates around 235.240 and 235.731 s. The car was travelling about 30 km/h. This is a local adapter error independent of any delay inside the navigation application.

The correction carries `vehicle.meta.receivedMonoTimeNanos` through the control adapter and applies each vehicle sequence once per session. Prediction and freshness use that original local receipt time. Legacy records without receipt metadata use the first observation time once; a heartbeat cannot refresh them. A new sequence may carry identical coordinates, and reconnecting sessions may reuse sequence numbers. Future local receipt times are rejected. Traffic-light Params heartbeat behavior and legacy navigation ingestion are unchanged. No cereal schema or native rebuild is needed for this Python-only change.

Recorded-event replay through the corrected parser and CarrotServ adapter accepts 236 new vehicle fixes and ignores 578 unchanged present-item heartbeats. The median position difference from the recorded old fused output is 1.89 m over 3,533 compared moving samples (95th percentile 9.46 m, maximum 41.45 m). This is a change in extrapolation, **not an accuracy improvement measurement**; route/source changes and missing data affect larger differences. It does not establish successful route following.

## Testing the one-second hypothesis

The navigation protocol's original vehicle records have a median update interval of 1.001 s, 95th percentile 2.000 s, and a longest interval of 20.298 s. The old adapter's synthetic fix timestamps obscured that cadence. A separate moving interval also has Carrot fix age above two seconds, eventually exceeding 16 seconds; the existing map freshness rejection must remain in force. A fixed one-second advance cannot repair a long input outage.

External speed compared with CAN speed suggests approximately 1.10-1.25 s lag in four acceleration/deceleration windows. Over a longer moving interval, shifting the speed samples by 1.15 s reduces speed RMS disagreement from 3.89 to 1.97 km/h. This supports delayed external **speed**, but Carrot already uses CAN speed for extrapolation; it does not establish the same delay in navigation coordinates.

To test coordinates separately, integrate CAN speed and the logged calibrated carControl yaw on a 10 ms grid. For each window, rigidly align the resulting relative path to the original navigation coordinates (rotation and translation, no scale change), then vary the time shift. Selected results are:

| Window | Zero-shift RMS (m) | One-second-shift RMS (m) | Best fitted shift (s) |
| --- | ---: | ---: | ---: |
| A | 7.10 | 6.09 | 0.75 |
| B | 5.28 | 6.56 | 0.20 |
| C | 8.34 | 9.14 | 0.00 |
| D | 3.16 | 5.04 | 0.10 |

These are shape/timing consistency fits, not surveyed position error or closed-loop validation. Map matching, pose drift, quantization, turns that differ from guidance, and fitting a rotation/translation limit the inference. The old fused Carrot coordinates produce different fitted shifts because the heartbeat bug changes their timing; they must not be treated as the original fixes. In particular, delayed navigation speed alone does not justify adding one second to all coordinates. No fixed one-second advance was activated.

## Pose and turn timing

Four substantial turns have continuous pose observations with valid input/posenet/sensor flags in all 1,100 samples. Median pose sample age at publication is approximately 25 ms and the per-window maximum is below 37 ms. Navigation heading, in contrast, often stays at the old road direction and then changes abruptly. Relative half-turn timing (using settled before/after directions, not assuming absolute pose north alignment) gives interpolated heading delays of approximately 3.55, 0.61, 2.90 and 0.82 s. The adjacent one-Hz sample brackets are respectively 3.00-4.00, 0.12-1.12, 2.35-3.34 and 0.36-1.29 s. These are not constant transport delays: road snapping and rerouting contribute. One of these turns still had a U-turn instruction while the vehicle made an approximately 90-degree turn.

Pose-based relative rotation/displacement is a reasonable next experiment, but blindly replacing GPS heading with absolute pose yaw or renewing old coordinates would be incorrect. It needs explicit calibration, sample/arrival freshness, rotation wrapping, source/session reset and bounded outage handling. Map position and maneuver distance must also remain temporally consistent; advancing only one input can move their turn boundaries apart or discard a maneuver early. This change fixes the demonstrated heartbeat error only; pose fusion and guaranteed turn response are not implemented or claimed.

## Validation and deployment scope

100 focused navigation, protocol, adapter and source-priority tests pass on the desktop harness; nine native raster tests are skipped because its compiled extension is not available in that harness. Regressions cover repeated heartbeats, original receipt preservation, first-observation compatibility, identical-coordinate new fixes, reconnects and future receipts. The corrected adapter was also exercised against the newly collected recorded events. Lint passes for the parser and touched tests; the existing large CarrotServ module retains unrelated pre-existing lint findings. No new source/guidance thresholds, CPU workers, GPU work, model selection or steering policy were added. This follow-up is committed/pushed on `carrot-nav`; the timing correction has not been deployed to the vehicle in this turn.
