# Read-only manual boundary approach experiment — 2026-10-31

The owner approved starting the proposed log-only evaluation before changing lamp
behavior. A separate BoundaryObserver runs in modeld only for the supported Ioniq5PE
profile with the existing ManualLaneChangeIndicator option enabled. It has no output
API for lamps/desire, never writes Params/journals, never sends CAN/USB data and does
not replace the existing BoundaryIndicator or change its admission and feedback mask.

## Observation hypotheses

Driver-directed torque must persist0.15s to create a candidate. Remember recent intent
for up to1.5s after it ends, with4s maximum candidate life: releasing/countersteering
alone does not erase continued lateral approach. Track continuous lane geometry and
recent target-boundary confidence<=1s. Reject frame gaps, jumps>0.4m, width inconsistency
and existing physical/automatic maneuver ownership. ATC/navigation preparation does
not gate this independent observation. Vehicle gates remain D, >=30km/h, valid CAN,
no trailer or steering faults. BSD is recorded as context, not evidence of intent.

Diagnostic clearance assumes0.9m half-width because measured tire span is not provided
by CarParams. A hypothetical indication needs0.15m accumulated boundary approach,
closing rate>0.1m/s, clearance<=0.5m and estimated time to that proxy<=0.6s. These are
provisional observation thresholds, NOT calibrated activation limits. A tracked center
crossing or0.25m retreat is recorded as geometry, never labelled as driver intention,
evasion, safe space or successful lane-change execution. Jump/relabel may prevent
outcome classification, and this is retained rather than inventing a completed change.

## Diagnostics and validation

manualBoundaryObservation emits at most1Hz with bounded reason counts, one sample and
at most8 timestamped transitions; dropped transition count is retained. Candidate ID,
model frame, clearance/rate, width-rate, confidence/intent ages, BSD and legacy direction
support correlation with rlog/video. No coordinates, device identity, images or extra
USB payload. Logging exceptions do not interrupt model publication; disabled state
stops reports after a possible pending final observation. Existing model runtime
measurements include this work; no device timing claim is made.

27 scenario/integration tests pass, including both directions, continued approach after
release/countersteer, curve-following, small correction, widening/jumps, weak lines,
retreat after a prediction, invalid inputs, bounded logs and logging failure. Actual
modeld observation statement runs against the Capnp schema with unchanged published
metadata. Existing indicator/helper suite695 pass/26native excluded. Native CI follows
publication.

Private read-only replay of12 previously hash-verified117f segments:14263 model frames,
36 candidates,19 confidence-loss endings and17 width-change endings,0 hypothetical
indications. Running the observer between two existing detector instances gives zero
state/output differences. This proves neither improved detection nor a false-positive
rate: the drive lacks human-labelled lane-change/evasion truth for these candidates.
The one-frame count difference from the prior14262-frame turn replay is because this
observer requires model/carState only, not all seven turn-policy services.

Desktop replay observation mean6.815us/P9918.5us/max224.9us with an in-memory logging
callback, excluding production logging cost. It is not a comma/Jetson CPU,50ms-deadline
or road result. Evidence remains private under .analysis/scratch/2026-10-31-boundary-observation.

Delivery is normal carrot-wip Git Pull and comma restart. No Jetson bundle/model change,
no live installation/reboot or physical actuation. Next real drives should be reviewed
with video to distinguish true changes, in-lane corrections and evasions before any
observer threshold is connected to lamp requests. Do not call this observer an enabled
new automatic indicator implementation.
