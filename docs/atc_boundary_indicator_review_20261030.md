# ATC preparation and manual indicator review — 2026-10-30

> Superseded later on 2026-10-30 at the owner’s request. The boundary-indicator option and audio change were removed. See [ATC restoration](atc_restore_20261030.md) for the current implementation.

## Findings

Thirteen hash-verified closed rlogs from the owner's morning drive (06:43–06:55 KST) were decoded with the full cereal schema. ATC was mode 1 and the independent 250 m right-indicator add-on was enabled.

- At 06:46:53 and 06:49:04, the log contains both `audioLaneChange`/`laneChange` sound requests and right-lane preparation. This verifies an audio request, not speaker audibility.
- At 06:53:12, `turn right prepare` changed to `atc right` and entered right-lane preparation without the lane-change audio event. The event method only handled fork transitions. Add both exact ATC directions; canceled forks do not announce preparation.
- The existing readiness-to-steering logic sometimes remained unarmed when the target lane was already wide before ATC preparation. Road-edge availability and later side-object conditions also blocked starts. These gates are unchanged. This review does not establish that every reported missed maneuver is fixed.
- The add-on previously rejected every non-off lane state. Existing matching ATC right preparation could therefore prevent its indicator request. Permit only `preLaneChange/right` with `atc right`; retain the same lane, rear-gap, BSD, fresh-input and driver gates. Active changes and other directions remain excluded.
- Recorded-input replay covers 14,558 model frames. Both old and revised add-on emit zero requests in this drive: other qualification conditions were not met. The new coexistence behavior is covered by synthetic regression cases, not a new road observation.

## Separate manual-indicator option

The reviewed contributor boundary detector produced two candidates in this replay: one with very low current lane confidence, and another pointing opposite to current driver steering. Its SPAS/APRK output is not imported.

`ManualLaneChangeIndicator` defaults OFF, is limited to the existing Ioniq 5 PE Jetson USB2CAN indicator path and AutoBlinkerControl 1/2, and does not operate with SPAS selected. It requires fresh valid inputs, drive gear, at least 30 km/h, sustained driver torque toward the boundary, reliable current recognition of both boundaries and coherent approach motion. Active ATC or lane changes block new requests. This stricter detector produces zero requests in the recorded morning drive; that does not establish real crossing sensitivity.

A model-owned edge requests one existing physical-indicator tap. Own lamp feedback and synthesized stalk counts are excluded from lane-change admission before publication, for at least 12 seconds and until sustained lamp extinction. Origin is journaled before sending and recovered on model restart. Rejected/stale edges are not replayed. The option never supplies a lane-change or turn desire and does not assess a safe traffic gap. Physical lamp behavior and actual manual-crossing acceptance remain untested; it stays OFF.

## Validation and delivery

Desktop: 480 checks pass with 26 Linux-native safety cases excluded; 25 Wiki generator checks pass. Desktop regression tests cover the actual event method and source loop, preparation coexistence, negative rear/BSD/driver gates, detector expiry/restart, own-lamp isolation in DesireHelper and existing indicator/driver-intent behavior. Windows tests use explicit platform doubles; native safety checks require Linux CI. Raw routes and device identifiers remain private. No live vehicle writes, settings changes, restart or CAN trial were performed.

Delivery is ordinary carrot-wip Git Pull and comma restart/reboot. The existing USB2CAN tap protocol and Jetson runtime/model are unchanged; no new Jetson bundle is needed.

### Published source validation

Source commit `bd1ea4eb006b9e6ca8059a0a64fa01c37ae7dee7` passed:

- [Signal integration](https://github.com/ajouatom/openpilot/actions/runs/40968732540): 1,277 passed, 40 skipped, 362 subtests; native CAN kernels/safety compiled.
- [Jetson integration](https://github.com/ajouatom/openpilot/actions/runs/40968732767): broad suite 1,810 passed/56 skipped, plus monitoring 232, HUD 62, update 94, navigation 119, storage 11 and NAT 11; Windows installer passed. Counts overlap with desktop checks.
- [eGPU compatibility](https://github.com/ajouatom/openpilot/actions/runs/40968732464) and [user docs](https://github.com/ajouatom/openpilot/actions/runs/40968732502) passed.
- Wiki manual regions published as `a25d12c`; [follow-up synchronization](https://github.com/ajouatom/openpilot/actions/runs/40968894527) passed.

The common `Lane-mode model speed fallback` CI job failed during collection because its environment did not install `pycapnp`; the preceding source commit had the same failing job. Add that test dependency only. Its two suites pass 26 local checks with the dependency installed. No planner implementation changes accompany this CI repair.
