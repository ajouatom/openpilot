# Jerk-confirmed lead braking persistence

## Problem and scope

Carnival `c4dcf95545dafe68/000001ce--c9d286b046/7` on `d2b2d628`
contains a rapid reduction in the selected track's speed. The old dynamics
policy multiplies `aLeadTau` by 0.9 at each 50 ms update whenever either
`abs(aLead)` or `abs(jLead)` reaches 0.5. Strong braking and small motion
therefore reduce the coefficient at exactly the same rate.

The coefficient is used as `a(t) = aLead * exp(-aLeadTau * t**2 / 2)`.
Lowering it retains more of the measured negative acceleration in the future
obstacle trajectory. It is not an ego jerk command or an acceleration filter
time constant. Measured acceleration is already used on the current update.

The shared `radar_motion/lead_dynamics.py` policy is used by full radard,
planner fast-radar overlays and NAS production replay. It changes only lead
acceleration persistence. Sensor acceleration/jerk filtering, lead selection,
distance/velocity measurements, MPC costs, following-time preferences, comfort
brake settings, acceleration limits, scheduling and validity limits are intact.
No setting or user-guide behavior is added or redefined.

## Attack and recovery

- Ordinary motion retains the existing quiet reset to 1.5 and 10% decay.
- Extra attack requires measured `aLead < -0.5 m/s²` and
  `jLead < -1.5 m/s³`. Strength ramps continuously to full at acceleration
  -1.5 and jerk -3.0; their two strengths are multiplied.
- Two distinct successive measurements must agree. Use the weaker strength
  of the pair, with no more than 150 ms between them. One-frame spikes,
  repeated use of the same radar timestamp and observations across a longer
  gap cannot confirm extra attack. Existing ordinary update cadence is kept.
- At full strength, the attack coefficient is 0.5 per update instead of 0.1
  (50 ms attack RC instead of 450 ms). Strength smoothly interpolates between
  those coefficients; no guessed future acceleration is added to `aLead`.
- As jerk returns to zero during sustained negative acceleration, the reduced
  coefficient is retained and continues ordinary decay. A quiet measurement
  restores 1.5 as before. A separately maintained ordinary coefficient restores
  the unboosted history when acceleration becomes nonnegative, so this state's
  braking correction does not prolong a subsequent positive acceleration.
- State follows sensor source and track ID, including front-radar kinematics
  borrowed by a corner detection. Missing tracks discard their state. Invalid,
  unmeasured or rejected fast-overlay inputs clear extra-attack evidence.

These thresholds are a bounded initial policy, not a claim that every vehicle's
jerk noise is below -1.5 or that two samples independently prove physical braking.
Both quantities are derived from filtered measured velocity. Existing source
selection, publication validity and acceleration warmup still apply.

## Offline evidence

Times below are relative to the first recorded `radarState` in segment 7.
Feeding the recorded track-49 acceleration/jerk through old and new policies:

| Time | aLead | jLead | Old Tau | New Tau |
| --- | --- | --- | --- | --- |
| 8.986 s | -2.486 | -3.673 | 0.7972 | 0.2343 |
| 9.189 s | -3.428 | -4.282 | 0.5230 | 0.01464 |
| 9.489 s | -4.191 | -2.345 | 0.2780 | 0.000520 |

- Tau first falls below 0.1 at 9.080 s instead of 9.982 s: about 0.90 s sooner.
  This is prediction adaptation timing, **not measured ego braking improvement**.
- The first 8 s reproduce the old Tau exactly, despite recorded jerk excursions
  from -1.249 to +1.188 m/s³. Existing small-motion Tau oscillations remain;
  this change does not retune the jerk filter or its ordinary reset thresholds.
- Full controller replay compares all 1,200 frames with old/new dynamics:
  all output fields other than `aLeadTau` are identical; 91 frames contain a
  Tau difference somewhere in their lead/list output. The 12.04 s track switch
  is preserved and not smoothed across distinct vehicles.
- Tests cover negative-jerk noise, isolated spikes, duplicate timestamps,
  dropouts, source/ID changes, borrowed front kinematics, steady braking,
  nonfinite data, recovery and radard/fast-overlay agreement.
- The maintained 97-log, 494-item detection corpus produces identical old/new
  reports in modes 2 and 3. There are no missing logs or new failures. Existing
  expectation failures remain 10 / 13 and pre-deceleration failures 0 / 1;
  208 items remain unverified because their input coverage is insufficient.
  These are detection regressions, not evidence of braking comfort or safety.
- The production MPC extrapolation, update and weight methods are executed
  with a recording solver for three comfort/following/jerk configurations.
  Predicted obstacles move closer, while all other solver parameter columns,
  acceleration-change costs and jerk costs remain identical. The native solver
  and closed-loop vehicle response are not simulated by that recording solver.

No on-vehicle C3/C4 braking or ride-comfort improvement has been established.
Earlier predicted braking necessarily can change commanded motion; retaining
configured comfort costs and constraints is not a guarantee of identical feel.

## Deployment

The new module is automatically included by `build_bundle.py`'s radar-motion
directory and by the replay source fingerprint. The image workflow runs the
new dynamics tests and fast-overlay/preview regression tests before publication.
Use the normal image workflow and NAS scheduled updater; verify the deployed
commit, updater's fresh-replay verification and an actual recalculated route.
