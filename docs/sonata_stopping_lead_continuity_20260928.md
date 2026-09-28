# Sonata braking lead continuity (2026-09-28)

## Incident and cause

The user reported weak initial braking in Sonata segment
`000003f2--3246ccd0e1--27`, vehicle revision `ef586a41637a`.
Full cereal decoding shows longitudinal control remained active, with no brake
or accelerator pedal intervention during the first stop. SCC12 carried negative
acceleration requests; standstill became true at 5.611 seconds relative to the
first carState. Recorded lead distance settled near 1.4 m. This is a radar range,
not independently measured bumper clearance.

Front #50 remained measured near the center. Recorded leadOne changed to #42
at 1.157 seconds, briefly returned to #50 at 1.657 seconds, selected #42 again
at 1.707 seconds, and returned to #50 at 3.455 seconds. At 2 seconds the selected
#42 was 13.35 m away while #50 remained at 10.94 m in leadsCenter. leadTwo was
absent. The returns had similar longitudinal speeds and about 2.3 m separation.
This establishes a farther-target selection, not a sensor detection dropout;
the physical origin of #42 is not established.

The two returns also fall inside the existing primary-duplicate distance gates
(3.5 m longitudinal, 1.8 m lateral). Thus selecting #42 does not automatically
give the closer #50 an independent leadTwo role. The correction belongs at the
primary selection boundary, rather than weakening duplicate rejection.

`VisionRadarMatcher._match_radar_only_moving` discarded even a confirmed front
identity when its speed reached the 4 m/s stationary-acquisition boundary.
The visual range likelihood could then favor the farther #42. The separate
stationary matcher required its own evidence, leaving a continuity hole exactly
while the known lead was braking. The synthetic stop and original-log replay
both reproduce the farther selection at this boundary before the correction.

The outgoing SCC12 command at 1/2 seconds was approximately -1.01/-1.56 m/s²,
while measured ego acceleration was -0.33/-0.69 m/s². A gradual initial plan and
weaker early vehicle response also reduced the stopping margin. Correcting lead
selection does not establish that all of this response difference is fixed.
The recorded `LongActuatorDelay=20` means 0.20 s, and the planner used a 0.25 s
base action time including DT_MDL. That setting is unchanged.

## Correction

Keep an already confirmed, measured, nonzero front-radar identity eligible as it
slows through 4 m/s to rest, provided its position and timestamps remain
continuous. The existing 0.25 s continuity-gap limit, central-path checks,
distance limits, source rules, native tentative-track rejection and independent
moving-vision conflict filter remain in force. A -1 m/s lower bound accommodates
small negative speed estimates around standstill (about -0.4 m/s here).

This does not acquire an unknown stationary object through the moving fallback.
Unconfirmed history, ID/source changes, missing measurements, loss of quality,
path departure, range discontinuity, excessive negative speed, an input gap or
reset remove this retention. Corner/SCC acquisition and new stationary-object
confirmation are unchanged. No setting, actuator delay, MPC tuning, validity
limit or scheduling policy is changed.

## Validation and limits

- Synthetic braking through the threshold fails before the change in modes
  1/2/3 and passes afterward through complete standstill.
- Negative tests cover ten loss-of-continuity/quality conditions and four
  unconfirmed low-speed cases. The complete matcher suite passes 459 tests.
- Focused detector, controller, replay, lead-dynamics and cut-in/out tests:
  841 passed. Route-vault tests: 29 passed, 4 platform-dependent skips locally.
- Planner fast-radar and longitudinal-preview tests: 254 passed. Ruff and
  whitespace checks passed for the changed code.
- The maintained Sonata case checks continuous leadOne #50 from replay time
  0.3 through 6.0 s and forbids #42. It fails before and passes after the fix.
  All 1,199 available replay frames are processed; the corrected first stop
  keeps #50 throughout. Replay frame zero is about 0.10 s before video time.
- Replay reconstructs inputs on model frames and is not a bit-for-bit replay
  of every original process delivery. Before-change replay reproduces the
  initial farther selection, but returns to #50 earlier than the recorded live
  output. Do not equate replay timing with every recorded handoff.

- Full maintained corpus: 99 pre-existing logs / 496 labelled items in each
  of modes 2 and 3. Every result row is identical before and after the change.
  Existing expectation failures remain 10 / 13, pre-deceleration failures
  remain 0 / 1, missing logs are zero, and 208 items remain unverified because
  their labelled inputs are absent. This is a no-new-failures comparison,
  not an all-pass claim. The added Sonata log makes 100 logs / 497 items;
  its continuity case passes in both modes after failing before the change.

Replaying recorded trajectories verifies selection; it does not simulate
closed-loop braking or prove a new physical stopping distance. GitHub image
publication and NAS verification are recorded after deployment.

Private source logs, decoded data and reproduction scripts remain in
`.analysis/archive/2026-09-28/sonata-stop` and the corresponding fix archive.
They are not included in Git.
