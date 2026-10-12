# Green departure hold and interrupted night-red tracking

The 10a5 drive used carrot-nav `ac1da01591` with SignalAssistMode=2.
This was not an update omission or an OFF setting. Six completed segments
provided 24 files (317,673,732 bytes), SHA256-verified against the device.
Keep all recordings, decoded telemetry, crops and experiment outputs private
under `.analysis/scratch/2026-10-12-signal-a5/`.

## What failed

At approximately 20:49:49 the stopped-red guard acquired a small extra red
candidate just above the main right-hand lamp. Its fixed anchor was roughly
`[673,281,685,287]`, while the main housing was about `[667,284,703,302]`.
Choosing whichever red was closest to image centre favoured the small candidate.
After the two real signals turned green, the observer and original planner
reported green, but neither green housing overlapped that sole anchor enough.
The guard remained held until the driver disengaged around 20:49:54.95. There
was no green release in the logged drive. Exact images confirm the transition.

Around 20:52:03-19, the car approached another red pair at about 52 km/h.
The more distant lamps occupied only 3-4 pixels. Vertical camera motion could
exceed their box height; even a strong current-image template match then failed
the box-overlap association and acquired a new ID. A second problem prevented
template tracking until red was already confirmed: small lamps lost the third
observation required to become confirmed. The preparation gate repeatedly lost
the two persistent identities required for its 600 ms confirmation. Road support
also intermittently failed. The live drive had **zero** preparation entries.

## Corrections

The start guard remembers up to eight red housing anchors during the same stop.
Each must independently receive two fresh red samples spanning 150 ms before
it may support a transition. Two disjoint, similarly sized and aligned housings
that each become green for at least three samples and 700 ms can release the
auxiliary veto even if the originally selected candidate disappears. A fresh
forward red vetoes release. Unknown, duplicate, unseen, stale, invalid and
restarted observations do not provide the missing votes. Green cannot move a
red anchor. The original single-anchor release remains available; the new
group release is logged as `green_group`. Release does not command acceleration.

For the verified-lamp observer only, a night-core track with two pending red
samples may use the immediately previous image to find a third fresh sample.
A red-to-red template may retain identity despite zero box overlap only with
a match score of at least 0.9 and an isolated peak at least 0.05 above alternatives
outside a two-pixel neighbourhood. Existing bounded displacement, size, capture
gap and current color checks remain. The new motion path requires a previous
direct night core; it does not relax green matching or daytime tracking.

Preparation geometry, two-red requirement, 600 ms confirmation, freshness,
lead/turn/pedal gates and acceleration ramp are unchanged. Driving models,
navigation, GPU/DM policy and SignalAssistMode semantics are unchanged.

## Verification and limits

- 512 focused tests pass. Twenty new tests exercise group release and its
  negative cases, pending-red image tracking, ambiguous peaks and invalid motion
  reassociation. The new test files pass Ruff; existing files retain unrelated
  pre-existing lint findings.
- Same recorded observer inputs now release the reviewed green hold at
  20:49:53.740, before the recorded driver disengagement. Nominal and added
  20/100/250 ms arrival-delay runs release; no new red-time release appears.
- Paired image replay covers 3,246 mapped observations in six videos. Two
  observations crossing segment boundaries are not included in this image
  replay. On encoded pixels, the baseline still never prepares. The corrected
  tracker supplies 17 active planner updates from 20:52:13.709 to 20:52:14.608,
  about **5.60 seconds before** the recorded original model-stop transition.
  The acceleration-cap replay reaches approximately **-0.19 m/s2**. Added
  20/100 ms delay retains 16/13 updates; +250 ms produces no entry.
- This is a brief request, **not a complete stop or sustained -0.5 m/s2 braking**.
  Pair loss still terminates preparation. Original model outputs and vehicle
  motion are recorded inputs: earlier model recognition or actual speed reduction
  cannot be inferred from this replay. Carrot state-machine replay uses the
  existing desktop parameter harness, not native MPC or closed-loop simulation.
- Prior 13-video paired regression processes 3,347 locally mapped observations:
  3,342 entire tracker results match, all fresh raw-color counts match, and no
  current green box differs. The five changes concern night-red tracking history.
  This is regression evidence, not exhaustive recognition accuracy.
- The prior 43,090-input start-guard corpus preserves archived hold counts and
  release times at nominal arrival times and at +20 ms. Three already-valid
  releases are now labelled `green_group` instead of `green`; reviewed-red releases remain zero.
  both 1095 false-start windows remain blocked. Existing misses, including the
  unacquired 1091-20 hold, are not fixed by this change.

The code is published to both maintained branches with single-parent commits.
This task does not install or restart the vehicle. Native timing and the next
physical drive remain unverified; desktop timings are not a device benchmark.
