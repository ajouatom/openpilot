# vNavi motorway signal suspension — 2026-10-13

Fresh vNavi current-road categories 0 (motorway) and 1 (urban expressway)
suspend traffic-signal-specific control. This applies to both maintained
branches, independently of SignalAssistMode. It does not disable radar ACC,
lead braking, driver overrides, soft hold, or an explicit user stop request.

## Incident and cause

The reviewed K5 segment at `0611d489` has startup settings SignalAssistMode=0
and TrafficLightDetectMode=2. The first native e2eStop entry occurs at about
15:46:35 KST, 79.95 km/h, with model terminal position 90.97 m and terminal
velocity 2.04 m/s. LeadOne and LeadTwo are both unavailable. The native
classifier's filtered terminal velocity is 2.56 m/s. Its no-lead distance
comparison uses 1,000 m, not the distance to an established vehicle.

The plan acceleration reaches -2.84 m/s² before the recorded gas override.
There is another brief native stop entry at 15:46:46; a lead at 101.44 m is
acquired shortly afterward and the state returns to lead following. No
e2eStopPrepare or shouldStop samples occur. This segment's minimum speed is
15.08 km/h; it does not contain a complete standstill.

The incident classifier, conditioned on the preceding logged xState and using
the exact modelMonoTime, reproduces all 1,199 logged trafficState values. Its
AST is identical to the pre-helper `fc843551` classifier. This identifies a
native model-trajectory-to-stop interpretation, not a signal-helper OFF leak.
It does not identify which image feature caused the network's trajectory.

## Input contract

- Subscribe plannerd to carrotNavi; it remains an optional input and is not
  added to longitudinal-plan validity prerequisites.
- Require seen/valid/alive transport no older than 2 seconds, schema version 1,
  a connected nonempty session, and no explicit off-route status.
- Require laneCurrent.meta.present and an original item receipt no older than
  10 seconds, matching the v2 JSON stream's advertised stale timeout. Reject
  future timestamps. Publisher heartbeats never renew item receipt time.
- Add Lane.roadCategoryValid. The publisher sets it only for an explicit JSON
  integer road_category in 0..12. An absent lane, omitted category, malformed
  value, or old publisher's default zero cannot suspend signal detection.
- Categories 0/1 suspend; other categories, missing/stale information, session
  loss, and disconnection restore ordinary signal policy. User settings are
  not overwritten. Do not infer motorway status from speed or a road-name word.

The app must supply current-road road_category even when a lane-guidance graphic
is not visible, and refresh it before the advertised item timeout. A cleared
lane_current or a field omitted by the app does not prove a motorway.

**The supplied incident does not have usable road classification:** all 130
carrotNavi messages have laneCurrent.meta.present=false, and the connection
later goes false. Therefore this patch alone cannot establish motorway status
from the original incident data. Do not claim that the unmodified log now
avoids the incident; usable source data is required.

## Control and display

While suspended, skip the native signal classifier, clear its confirmation
history, disable auxiliary moving-red assistance, departure veto and stop
preparation, and ignore external red/green transitions. Release existing
signal states into normal cruise/lead selection while preserving soft hold and
explicit user stops. Restore fresh classification on leaving suspension.
No ONNX weights or radar selection thresholds change. Experimental blended
trajectory control is not converted into ACC by this feature.

LongitudinalPlan.signalDetectionSuspended carries the actual planner decision
to the UI. C3/USB HUD and C4 lamps replace the red/green icon with a gray circle
containing white pause bars. C4 navigation lamp countdowns are replaced too;
old red/green is not shown underneath. The overlay invalidation key includes
the flag. The C4 lamp changes immediately and clears pause opacity on exit.

## Validation

- 724 focused signal, navigation-payload, schema, planner-state and UI tests
  pass on carrot-nav. Tests cover missing/default category, stale/future
  timestamps, exit, lead priority, held-red release, active preparation
  cancellation, external signal transitions, manual stops and soft hold.
- The Windows runner substitutes Params storage; no native device runtime or
  physical display is implied. The Linux-only carrot_serv import was not run
  under Windows (fcntl is unavailable); carrot_serv is unchanged.
- Baseline/new state-machine replay on 1,199 recorded inputs produces identical
  state, traffic, stop distance, cruise speed, comfort brake and model-lead
  offset with the actually missing road evidence.
- Injecting fresh category-0 evidence into that same replay removes all 75
  replayed signal-stop state samples and preserves all 940 lead-following
  samples. This is a counterfactual classification test, not full MPC or
  vehicle simulation. Replay starts with default internal state because the
  preceding segment is unavailable; 75 is not the recorded incident's count.
- The actual shared pause drawing function was rasterized for visual review;
  both C3 and C4 render entry points are exercised by tests. Native panel and
  on-road validation remain outstanding.

Source logs, video, decoded data and replay scripts remain private under
`.analysis/scratch/2026-10-13-k5-31c/`. Pull/build must regenerate the cereal
schema before running the new publisher/planner/UI together. No vehicle
installation or road validation was performed for this change.
