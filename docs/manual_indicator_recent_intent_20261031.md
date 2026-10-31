# Manual indicator: recent intent and adjacent-lane evidence — 2026-10-31

The owner requests preserving manual steering intent after torque release and
blocking indications across a centre line, road edge or unconfirmed adjacent lane.
This changes only ManualLaneChangeIndicator on the existing Ioniq5PE USB2CAN path.
ATC, the 250m add-on, normal/BT lane changes and automatic steering are unchanged.

## Behavior

Confirm signed driver torque/steeringPressed for 0.15s and adjacent-lane evidence
for 0.2s. Retain that episode for up to 1.5s since last confirmed same-direction
torque, with a four-second overall limit. Torque release or countersteer alone
does not cancel continuous approach. Cancel on more than 0.3s without approach,
more than 0.15m retreat, lost adjacent lane, invalid inputs or geometry discontinuity.
The existing near-boundary/rate/width-consistency and one-second target-confidence
gates still apply; require at least 0.15m cumulative approach within the episode.
No approach-memory expiry generates a request. Fresh source edges no longer require
steeringPressed a second time; all vehicle/freshness/ownership gates remain.

Positive lane evidence requires the exact cached OEM lane-type/colour CAN timestamp
(new additive CarState ordinal112), no more than 250ms old, and known white marking
types10/11/14/15/17. Yellow including dashed yellow, edge/barrier, unknown colours
and undecided types block the selected side. Require the model outer boundary
probability above0.5, ordered adjacent width2.5..4.8m and road-edge clearance of
at least2.5m after subtracting edge standard deviation, at model samples0/4/8.
Missing/truncated/nonfinite or uncertain edge inputs fail closed. A distant edge
without an outer lane boundary is insufficient. This is evidence, not a guarantee
of same-direction drivable pavement or a safe traffic gap.

Timestamp publication follows the exact parser dictionary selected by the existing
CAM_0x2a4/CAM_0x362 preference; a different bus/cache cannot refresh it. Existing
leftLaneLine/rightLaneLine values and all existing consumers remain unchanged.
The source independently rechecks selected-side evidence before journalling/sending.
The prior3s retry,1s continuous dark,12s owned-feedback isolation and restart journal
remain; no CAN/receiver/USB protocol changes, held sequence or steering request.

## Validation and limits

Desktop indicator/observer/turn suites557 passed; existing desire/stalk/side-object/
lever tests50 passed using isolated PC hardware/Params/IPC. Windows cannot import
Tici fcntl directly; the offline harness stubs only hardware selection, not detector
or steering logic. Focused tests cover release/countersteer with continuing motion,
retreat/stall/expiry, both directions, centre lines/edges/shoulders, stale/future/
missing timestamps, positive outer-lane evidence and fresh-source cancellation.
Two real CAN parser and actual carState-publication checks also pass locally and are included in native Linux signal CI.
Final CI results are recorded separately; desktop tests are not road acceptance.

The latest two private hash-verified drives contain three actual manual requests.
At all three, OEM leftLaneLine=21 (yellow solid) and outer-left probability is below
0.5 (0.298/0.123/0.241). The requested new gates intentionally reject those samples;
this is not a replay claim of improved physical indication. No raw routes/identifiers
or coordinates are published.

The separately reproduced Process.latest one-shot event overwrite remains an open
transport issue; this change does not assert that all missed lamps are cured.
The observation-only logger remains separate and does not replace these gates.

Delivery: normal carrot-wip Git Pull and comma restart, including matching schema
and carState publication. No Jetson model/runtime bundle or Jetson reboot is needed.
No live vehicle settings writes, restart or CAN actuation were performed for this change.
