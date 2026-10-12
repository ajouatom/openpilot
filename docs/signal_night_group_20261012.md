# Night lamp groups connected to gentle preparation

This supersedes the unsuccessful night-height trial in
[the earlier report](signal_night_prepare_20261012.md). The revised connection
produces an earlier, brief preparation request in one reviewed night encounter.
It does not establish a complete automatic stop, improved driving-model response,
unseen-scene precision, or vehicle deployment.

## Why the earlier connection failed

Night detections supply a lamp core or current-image template match, not a
measured housing. Requiring a full metric distance interval from housing size
excluded all of them. A separate height-only trial still required the entire
possible distance interval to be bounded within 25–140 m and confidently covered
by model road geometry. Distant lamps failed this condition even when visible.
Treating every template observation as a new untrusted object also repeatedly
interrupted confirmation of the same real lamp.

Gentle preparation now has a distinct night contract. It verifies a persistent
group and a plausible road direction, rather than claiming a stopping distance.
The original model still owns the stopping target and stronger braking.

## Night contract

- Require two to four distinct confirmed red lamp identities, at least four
  qualifying captures spanning 600 ms per retained identity, compatible row and
  scale, no overlapping actual cores, and overlapping road-support intervals.
  Inferred housing boxes may overlap; their actual lamp cores must not.
- Direct compact night cores require quality >=0.5, red score >=0.4, green score
  <0.08, diameter 3–22 pixels, and a consistent inferred-box layout.
- A same-ID template may bridge a direct core for at most 500 ms, with current
  raw red, confirmed red, match score >=0.9, bounded displacement and scale.
  A template does not establish or renew the direct-core anchor.
- Fresh forward raw green immediately cancels this night path, even if green
  has not yet accumulated confirmation. The daylight central-red priority
  contract remains separate; this night path does not adjudicate arrow mixtures.
- Missing pair evidence makes output inactive immediately. Confirmation can
  survive up to 350 ms from the last qualifying pair, but missing observations
  do not add votes and a new identity does not inherit an old identity's history.
- Night capture freshness is explicitly **350 ms**, including inference latency,
  rather than the measured-housing path's unchanged 200 ms. Capture gaps above
  250 ms still reset confirmation. Session changes, invalid context, malformed
  data, pedals/cooldown, leads, turns and model stopping cancel the path.

The night geometry projects the red-lamp center through camera calibration. It
searches model-supported depths 25–190 m for a contiguous >=20 m interval in
which a 3–9 m mounting-height hypothesis and lateral road inclusion are possible.
One pixel of localization margin is included. It never extrapolates road curves.
The actual signal may be farther away: this is existential directional support,
not a measured or fully bounded range. Logs therefore set `distance_range=null`
and report `evidence_kind=night_lamp_group` and a separate `support_interval`.

Confident inner lanes retain their original probability/std checks and 3.5 m
outer allowance. For this night path only, road-edge stds up to 1 m are considered
by shrinking the usable corridor inward by `0.3 m + 2*std` on each side. The
remaining corridor must be at least 2 m wide and must agree with confident lanes.
This trades the prior binary rejection for an uncertainty allowance; model stds
are not demonstrated calibrated confidence bounds, nor is this lane assignment.
Measured-housing geometry retains its previous contract.

`SignalAssistMode` remains 0=OFF, 1=display/alerts and stopped-red departure veto,
2=add moving preparation. Only mode 2 actuates. Existing acceleration behavior
ramps at 0.25 m/s³ toward −0.5 m/s² and releases immediately when preparation is
inactive. No synthetic `x/v`, stop-line distance, `shouldStop`, or go command is
introduced. CPU observer selection and DM GPU protection are unchanged.

## Recorded-input results

The latest nine-minute evening drive supplies 10,276 sampled planner times and
3,894 original observer publications. The gate replay is followed by an actual
`CarrotPlanner.update` state-machine replay and the unchanged final acceleration
cap method. Camera frames are reviewed separately; no future images or labels
are inputs to the decision.

| Added observer delay | Night preparation samples | Entries |
|---|---:|---:|
| 0 ms | 23 | 1 |
| 20 ms | 22 | 1 |
| 100 ms | 19 | 1 |
| 250 ms | 0 | 0 |

All entries are the same visually reviewed red pair in segment 21. No preparation
entry occurs elsewhere in these nine minutes. This is coverage of one encounter,
not an exhaustive false-positive-rate estimate.

Nominal preparation starts at 19:15:25.640 and ends after 19:15:26.738, at about
52 km/h. The replayed target acceleration goes from +0.0048 to −0.2696 m/s²;
22 samples are negative. With +20 ms delay, the minimum is −0.2565 m/s².
The original model's stopping transition is at 19:15:31.739, approximately
6.10 seconds after preparation begins. At this point preparation yields and
the original stop calculation runs. During preparation `stop_dist` remains the
1000 m sentinel, not the geometric support interval.

In segment 21, replayed and logged state match on 1,148 of 1,171 sampled updates;
the 23 differences are precisely the new preparation updates. The original
model-stop transition time is unchanged in recorded-input replay. The latest
drive still contains a driver-completed stop: earlier model recognition or a
successful autonomous stop cannot be inferred from this experiment.

Exact encoded camera indices 195, 201, 207 and 215 show the two real distant red
lamps at the new entry/continuation. The pair loses usable identity/evidence
after approximately 1.1 seconds, so this encounter does not reach the −0.5 limit.
Continuity remains a limitation; release is intentional when evidence fails.

Prior night/morning/day regression uses 4,329 retained observer inputs across
14 segments / 17 source variants. Both prior and revised gates have zero
preparation entries in that corpus. This verifies no added entries on those
recorded inputs; it does not demonstrate correct rejection of every object or
positive night coverage beyond the newly reviewed encounter.

## Validation and deployment boundary

- 492 focused observer/runtime/display/start-veto/preparation/mode/worker tests
  pass, including 44 new night cases. These include actual planner linkage,
  observation-only operation, model-stop handover, no inferred distance, direct
  anchor expiry, conflicting green, context cancellation and malformed inputs.
- 25 Wiki generator/validator/workflow tests pass. Korean/English guides and all
  three localized setting summaries are updated. Local Wiki generation validates
  with zero schema/Markdown errors; no separate live Wiki publication is claimed.
- Ruff passes for the preparation module and new night tests; the user-docs check
  passes. Existing NumPy deprecation warnings remain.

The state-machine replay uses recorded driving mode, pedals, speed, model curves,
lead state, source validity and pose/calibration. Native Params storage, driving
mode detection and lane-change bookkeeping are replaced for desktop execution;
other parameter defaults and empty model lead hypotheses are used. Observer-log
arrival approximates tmpfs delivery. Recorded-input cap evaluation is not native
MPC, actuator execution, vehicle response, measured stopping-distance improvement,
or a C4 workload/latency benchmark. The driving ONNX and DM models are unchanged.

The vehicle was read-only checked on `carrot-nav` at `135020d1` with onroad=1 and
engaged=0. This task changes only `carrot-wip`; it does not overwrite that vehicle
branch or other local navigation work. No vehicle files/settings were changed.

Private recordings remain in `.analysis/archive/2026-10-12-signal-drive-a4/`.
Replays, pinned source snapshots, cap traces and reviewed frames are retained in
`.analysis/archive/2026-10-12-night-geometry/`; raw footage is not committed.
