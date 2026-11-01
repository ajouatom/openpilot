# Manual indicator directional exclusions and drive review — 2026-11-01

The owner reports missed manual indications and a second consecutive left turn,
then explicitly replaces yesterday's adjacent-lane admission with left centre-line
classification and right road-edge distance only. This supersedes the admission
section of `manual_indicator_recent_intent_20261031.md`, not its recent-intent,
freshness, ownership or steering-feedback isolation behavior.

## Change

Left: require the existing fresh OEM marking timestamp (at most 250ms) and a known
white marking (10/11/14/15/17). Yellow, unknown, edge/barrier or stale classification
still cannot establish that this is not a centre line. No left road-edge or outer
lane-line check remains.

Right: require finite road-edge clearance of at least 2.5m beyond the approached
right boundary at model samples 0/4/8. Do not use OEM colour/timestamp, outer lane
probability, adjacent lane width or edge standard deviation for this exclusion.
Missing/truncated/nonfinite right geometry blocks requests. A wide shoulder can
also pass; this is not proof of a drivable lane or a safe traffic gap.

Both the detector and source use the same function. Target-boundary confidence,
continuous crossing geometry, driver-intent retention, speed and vehicle checks,
three-second retry/one-second dark period and owned-lamp isolation remain.
ATC, the 250m add-on, normal/BT control, automatic steering, CAN and receiver logic
are unchanged. No additional model-output trigger is enabled.

## Uploaded-drive evidence

Three full-schema rlogs (42,059,119 bytes) and their three qcamera videos were
copied from the exact owner-supplied NAS paths with SHA-256 verification. Actual
source is 7e88bb724; ManualLaneChangeIndicator=1 and AutoBlinkerControl=2.
Adjacent segments 3 and 5 are unavailable: replay state resets across those gaps.
Evidence, device identity, raw logs and videos remain private.

All 3,600 recorded model messages contain no manual indicator request. Video shows
a rightward crossing near segment 6 seconds 52–54 and a leftward crossing near
segment 7 seconds 1–3, with a visible adjacent paved lane. Yesterday's mandatory
outer-line confidence/edge-uncertainty gates prevent admission during these moves.
At segment 7 +2.364s, outer-left probability is about 0.175 despite an OEM white
line. The deployed detector gives zero requests. Recorded-input replay of the
new directional gates produces one left request there. This is a detector result,
not physical lamp acceptance or a full closed-loop simulation.

The right move remains undetected after this change: target-boundary confidence
itself falls below 0.5 before crossing and exceeds the existing one-second memory
by the qualifying geometric approach. Merely removing outer-lane gates does not
fix that separate failure. The previous pre-715 detector also misses this right
move. Do not describe all missed indications as cured or solely newly introduced.

## Consecutive-turn diagnosis (not changed here)

In segment 4, a left cue occurs at +33.084s; CAN lamp feedback starts at +33.278s.
At approximately +38.05s, carrotMan publication exceeds the source's 350ms freshness
limit. Actual source diagnostics change to `inputs` and release ownership at
+38.142s; lamp feedback turns off at +38.976s, during the turn. Source-function
replay reproduces the reset with four of five 10ms sample phases; the fifth misses
the short stale interval. Recorded publication times cannot recover exact source
receive times, but the actual diagnostic transition agrees with this mechanism.

The next left cue at +41.134s arrives during latActive=false (+40.032–42.580s).
With the earlier hold erased, a moving new turn fails automatic_control_allowed;
the cue is consumed and is not restarted when lateral activation returns. There
is no second physical request. Current no-replay policy and loss of turn ownership
are relevant, not an established USB/CAN transmission failure.

`modelV2.meta.desire` is the helper's control intent, used as the next iteration's
model input (converted to a rising-edge pulse). `desireState` is neural probability
output, not a command. First-turn helper turnLeft is +37.682–39.041s; during the
second turn the helper desire stays none, while model turnLeft probability reaches
0.9974. Conversely, the manual right/left windows only reach lane-change output
probabilities 0.0903/0.1320. Thus simply enabling both sources would neither cure
both manual cases nor establish reliable early indication.

Recommended follow-up: retain a bounded maneuver identity through transient
publication gaps, release wire leases while invalid, and recover only with fresh
matching navigation/vehicle inputs and no driver cancellation. Handle a fresh next
turn during temporary steering suspension separately from reconnect replay. Use
explicit control intent as supporting evidence and sustained output probability
only with matching navigation/physical context; do not let synthetic lamps feed
back into a new automatic steering request. These turn changes are not deployed
by this manual-exclusion patch.

## Validation and delivery

Desktop indicator/turn/TBT/observer suites: 540 passed; Wiki generator/validator
tests: 25 passed. Focused regression cases cover both directions, unknown/stale
left classifications, right threshold/missing geometry, low outer-line confidence,
uncertain edge standard deviation and unchanged intent/ownership behavior.
Production Python lint passes. Native CI results are tracked with publication.

Normal carrot-wip Git Pull and comma restart applies this source-only change.
No Jetson bundle/model update, live vehicle settings write, restart, CAN actuation
or road acceptance was performed. The separately reproduced one-shot source event
overwrite remains unresolved and was not the cause of these zero-request samples.
