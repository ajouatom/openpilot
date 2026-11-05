# Restore existing ATC and retain only the 250 m indicator add-on

> Follow-up: the owner clarified that the manual indicator is a separate requested feature. It is [restored separately](manual_lane_indicator_restore_20261030.md); the ATC audio restoration remains in force.

The owner explicitly requested the existing ATC behavior back, retaining the independent 250 m right-indicator addition. This supersedes the manual-boundary indicator and audio expansion in `bd1ea4eb00`.

## Restoration

- Restore DesireHelper, its modeld call, the indicator source and SelfdriveD audio logic to `f5e4f8b8a1` (before the boundary trial).
- DesireHelper and SelfdriveD are also byte-identical to `8f17e43d77^`, immediately before the original 250 m feature. This restores existing preparation, admission, cancellation and announcement behavior; no lane/edge/BSD/driver gate is removed.
- Remove ManualLaneChangeIndicator from Params, the catalog and UI, all executing detector/source code and its trial tests. Retain the two unused schema ordinals to avoid reusing a published wire tag. No value is published or consumed.
- Keep AutoTurnControlRightLaneChange as a separate add-on to ATC 1/2. It only emits a right indicator through the existing physical output after the lane, speed, rear/BSD, freshness and driver conditions pass. Matching ATC-right preparation can coexist; active changes/other directions cannot be joined. The add-on never changes carrotCmd, ATC navigation or DesireHelper input/desire directly.
- No Jetson runtime/model update, vehicle Params write, reboot or CAN trial was performed. Ordinary carrot-wip Git Pull and comma restart/reboot delivers the change.

## Evidence and limits

The same 14,558 morning model frames were replayed through the pre-consolidation Wip helper (`f29102fa83`) and the pre-restoration current helper with the trial disabled. Lane-change state, direction, desire, automatic arming and ATC state matched on every frame (zero differences). The older helper also left the reported windows in preparation. This is not evidence that the user’s daily-road observation is mistaken; it means the tested controller revision alone does not explain it. Model/navigation/sensor inputs and the exact previously successful run still matter.

The 06:46:53 window contains a preparation sound request but no lane-change start; the 06:53:12 window has preparation without that sound and no start. Restoring code has not established that these physical maneuvers work again. Do not describe the failure as audio-only, or claim road restoration based on synthetic/recorded-input checks.

Desktop regression: 450 pass, 26 Linux-native safety cases excluded. Wiki generator: 25 pass. Native CI results are recorded with the published commit; raw routes and replay details remain private.
