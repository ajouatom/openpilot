# Night red preparation: cadence fix and unsuccessful geometry trial

The latest nine-minute drive still does **not** demonstrate auxiliary moving-red
deceleration. This change fixes confirmation being erased between inference
arrivals. The separately tested night-lamp geometry candidate produced zero
preparation entries and is **not connected to vehicle control**. There is no
vehicle installation, driving-model retraining, or claim of improved stopping.

## Confirmed failure

The completed segments 14–22 of the evening 10a4 drive contain 3,894 observer
outputs. Of these, 1,063 contain fresh confirmed red tracks. All 1,881 such track
observations use inferred boxes: 1,613 `night_lamp_core`, 268 `causal_template`,
and zero measured `detected` housings. Mode 2 moving preparation requires a
measured housing; therefore the night detections cannot reach its distance gate.
The stopped-red guard and display have different contracts and can use them.

In the last reviewed red approach, the first confirmed auxiliary red occurs
around 19:15:20.910 at 51.4 km/h. The original model first enters stopping around
19:15:31.738. Driver braking appears at 19:15:37.295 around 43.4 km/h. Auxiliary
preparation never enters. The visible red signal is real; this is not a successful
autonomous stop. The earlier segment-16 transition includes green and amber
before red; those earlier frames must not be labelled missed red.

Another obstruction is timing: inference runs approximately 6.3–9.7 Hz in this
drive, with median capture-to-result ages of 103–129 ms. The planner can read an
already accepted sample after its 200 ms control deadline while waiting for the
next normal result. Previously each such read cleared the entire 600 ms
confirmation history, despite no new image or negative color evidence.

## Production correction

For a repeated, previously accepted session/frame/capture timestamp only:

- A capture older than 200 ms produces inactive `awaiting_fresh`; no acceleration
  cap or permission to depart is granted by that sample.
- Confirmation history is retained for at most 450 ms from that capture: the
  250 ms maximum capture gap plus the unchanged 200 ms freshness deadline.
- Only a new timely capture can resume confirmation. Repeated reads add no votes
  or elapsed confirmation time. The incoming capture-gap check still applies.
- A newly arriving stale sample, changed session, reordered/inconsistent frame,
  longer outage, fresh missing/unknown/green result, missing geometry, changed
  track identity, or ineligible driving context cannot inherit confirmation.
- Mode changes, pedals/cooldown, lead, turning, model-stop handover, speed bounds,
  final acceleration ramp, and stopped-red departure veto retain their policies.

The final acceleration cap still clears during stale intervals. Thus this is a
confirmation fix, **not a solution to intermittent braking authority**. Extending
control freshness or retaining a braking command would be separate changes and
were not done here. The one-sound-per-ten-seconds bound remains effective across
brief inactive intervals.

## Night-lamp experiment (offline only)

The inferred 2:1 box around a saturated lamp is not a measured housing. The trial
uses its known lamp-center ray and calibration, not its width as physical scale.
It intersects an assumed 3–9 m installation height with trustworthy lane/road XYZ
over 1–250 m in one-metre samples. It rejects truncated/disjoint support, requires
the complete plausible interval within 25–140 m, and requires lateral inclusion
throughout that interval. These are hypotheses, not measured distances or proof
that a signal applies to the ego movement. Current forward raw green vetoes the
trial; templates cannot establish new night evidence.

Producer-time screening of the 1,613 fresh confirmed night-core red observations:

| Outcome | Track observations |
|---|---:|
| No trustworthy lane/road corridor | 1,318 |
| Height interval truncated by available road geometry | 129 |
| Full plausible interval outside 25–140 m limits | 145 |
| Lateral inclusion fails over the interval | 9 |
| Geometry candidate accepted | 12 |

The 12 accepted observations span nine publications and do not establish usable
moving preparation. In the final approach the only accepted producer-time
observation is around 19:15:36.876, after the original model has already entered
stopping. Relabelling cores as housings or bypassing unavailable road confidence
would hide the failure rather than validate signal range/applicability.

## Validation

- 448 focused observer, display, runtime, start-guard, preparation, mixed-color,
  mode and worker tests pass on Windows with a native Params storage stub.
  Twenty new cases cover delayed 6.67 Hz arrivals, stale inactivity, bounded
  retention, no duplicate votes, new evidence, restart, and driving cancellation.
- A controlled 125 ms inference-delay / 150 ms capture-interval test now reaches
  confirmation; stale planner reads remain inactive and reset the acceleration
  cap. This isolates timing, not road perception accuracy.
- Same-recorded-input replay: 10,276 sampled planner times across the nine-minute
  drive, plus a separate +20 ms publication-delay trial. Original, corrected,
  and offline night candidate all have zero active preparation samples/entries.
  In the nominal replay, 488 of the original 500 stale resets become inactive
  waits; 12 still reset. This is not 488 recovered detections or braking events.
- Prior night/morning/day regression: 4,329 retained observation inputs across
  14 segments / 17 source variants. Original and corrected active decisions match
  exactly; both have zero preparation entries. This cannot establish positive
  braking coverage or perception precision.
- Ruff passes for the changed production module and new test module.

Arrival replay approximates tmpfs publication by the observer log timestamp;
planner samples are not exact consumer reads. It uses recorded sensor validity,
calibration, pose, pedals, gear, leads and model state, but treats unrecorded
internal entry counters permissively. No counterfactual vehicle motion, native
MPC, actual braking-distance improvement, or exhaustive false-positive labels
are supplied by these replays. Original driving and DM models are unchanged.

Private source recordings and hashes remain in
`.analysis/archive/2026-10-12-signal-drive-a4/`. Scripts, old/new gate snapshots,
screening and replay results are retained in
`.analysis/archive/2026-10-12-signal-night-fix/`; no raw footage is committed.

The remaining problem is obtaining persistent, credible signal applicability
when night road geometry is unavailable. An independently validated object and
movement association is needed before promoting this night braking path; simply
collecting more of the same red-color observations does not fix that contract.
