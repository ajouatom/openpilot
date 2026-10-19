# Signal stop memory after lead-follow departure — 2026-10-19

## Incident and cause

An Ioniq 9 segment recorded on carrot-wip `5032afca77` shows actual automatic braking after lead loss at about 76.5 km/h. The planner target reaches -4.4967 m/s², carOutput reaches -4.0 m/s² and measured acceleration reaches -3.6156 m/s². Driver gas starts around segment 6.395 s at 42.67 km/h; brakePressed remains false through the first seven seconds. All 74 observer records in the initial 6.45 s have empty tracks. The red planning state was injected by retained stopped-car memory, not fresh red evidence.

The Oct 17 lead-follow handover suppressed the effective decision without clearing the guard's internal hold. The moving reset required that hold to be false already. Lead disappearance therefore reactivated a stopped-red veto at road speed. The 19th segment is the only uploaded segment for this route; original latch acquisition is unavailable. Raw logs, identifiers and reproduction data remain in the private analysis archive.

## Correction and related-path audit

- Limit stopped-red memory to stop/creep speeds through 5 km/h. Above this boundary, clear the old latch and confirmation history without reporting green, a green release, or acceleration permission.
- Apply the same retirement to the older SignalAssist stopped hold. Preserve its separate moving-red commitment, model-distance admission and fresh-red confirmation.
- Retire the auxiliary-injected e2eStopped state too. Track its ownership so a mode disable/invalid reset cannot strand a stop after the guard latch disappears. Explicit user-stop distance, soft hold and stay-stop remain authoritative.
- Independently reject a stale stopped-red decision above that speed at the final longitudinal output gate. Do not clear braking or shouldStop already requested by MPC/model/lead control.
- Preserve low-speed lead-loss reapplication, normal lead obstacles, fresh red acquisition after a later stop, same-housing green confirmation, mode 0, road policy, and mode 2 gentle preparation.

The 5 km/h boundary matches the existing stop/preparation transition; it is an explicit scope limit, not a new detector of whether a junction has been crossed. No claim is made that every signal or lead association issue is solved.

## Validation

New regression coverage includes both guards independently and together, lead loss/range exit below/at/above 5 km/h and at 76.5 km/h, returning to rest, already applied stops, disabled guards leaving an owned stop, invalid/stale/missing observations, clock rollback, reverse motion, explicit driver holds, fresh later red confirmation, fresh moving-red model-distance stops and final-output protection. The initial baseline produced 30 behavior failures plus five calls to the new speed-aware output signature; 18 low-speed cases already passed.

Recorded-input replay uses 127 exact referenced model/radar pairs in the initial 6.45 s. Recorded holds: 56; old source replay: 55 (one subscriber/context-alignment difference); corrected source: 0, with no forced zero-distance stops. The initial hold is seeded because earlier segments are absent. The 74 empty-track observations are represented as empty packets; non-recorded Params/navigation use the existing replay defaults and context is assumed valid. This checks production Carrot state transitions, not native MPC, actuator replay, a changed vehicle trajectory or closed-loop driving.

Desktop tests use a Params double for unavailable native storage and the existing production-method replay harness. Both maintained branches pass all 704 focused signal/analysis tests, including 61 new regression cases. The test environment emits existing NumPy 2.5 shape-deprecation warnings; no test is skipped. Both branches reproduce the same 127-pair correction. No vehicle installation, process restart, settings change, model change or road validation is performed by this correction.

User-doc validation passes. The unchanged Wiki tool suite passes 24/25 tests; its existing CI-check test hard-codes 195 settings while the catalog already contains 196. This failure reproduces in the untouched main checkout, before these documentation changes.
