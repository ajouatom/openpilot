# Morning indicator review and configurable ATC distance — 2026-11-03

The owner requests morning TBT dropout diagnosis, manual steering-force/approach
analysis against the previous deliberate drive, and a distance setting for the
right-indicator ATC add-on. Subsequent instructions keep normal turn completion
separate from driver cancellation and narrow normal wheel return to ±15 degrees.

## Recorded morning behavior

Ten closed full rlogs total 137,168,022 bytes. Source and downloaded SHA256 values
match. The vehicle runs 03a28b256c with the relevant indicator options enabled.
Full cereal decoding gives 11,800 model frames. Raw logs and detailed traces
remain private.

Three TBT requests abort at 06:54:34, 06:54:45 and 06:54:54 KST. All show receiver
`driver_override` before a recorded matching completed-send acknowledgement.
The first right turn is about 31m ahead with steering -4.8 degrees. The next left
turn is about 38m ahead with steering -9.6 degrees. The third is a consecutive
left cue, about 35m ahead with steering +46.5 degrees. None of these source
cancellations is reported as wheel-return completion.

The receiver's `unchanged` predicate combines original B1 payload equality,
CAN reception age below 400ms and ownership lifetime. Failing any component
produces the same `driver_override` state. Comma-side eight-byte B1 observations
show tap/release-like packets followed by the same neutral value at about 200ms
intervals. They do not show the receiver's local input ages or exact aborting
payload. Actual lever operation and receiver-local stale/change causes cannot
be distinguished conclusively. The owner does not remember lever use and cannot
power Jetson for the remaining receiver investigation in this task.

## TBT code changes

Normal completion requires prior selected-side steering above 30 degrees and
measured heading rotation of at least 20 degrees, followed by wheel return within
±15 degrees while moving at least 1km/h. Existing distance expiry also applies.
Driver cancellation requires prior selected-side steering of at least 30 degrees,
opposite pressed torque and an angle strictly across zero into the opposite side
while moving. It applies in that source cycle.

A receiver override alone no longer discards the tracked TBT journey. A local
TBT hold whose exact request previously received completed-send acknowledgement
and matching lamp feedback can continue with fresh ready/waiting/driver_override
telemetry until turn completion, driver steering cancellation or input failure.
An override does not authorize a new tap or a new unacknowledged hold. A missing
initial acknowledgement still prevents physical hold activation and an unstarted
request expires after four seconds. Therefore this change does not establish a
cure for the three morning initial-send failures. Jetson-side evidence remains
required for that issue.

Source-loop regressions exercise acknowledged and unacknowledged starts with
receiver override, stale telemetry, error and guard expiry. A confirmed override
case retains one request ID and its physical hold through approach and turn,
then releases at the configured completion. Unacknowledged/fault cases never
start a physical hold. Sender/CAN receiver binaries are not changed.

Five-phase 20Hz recorded-input journey replay covers the morning and previous
lunch return. Independent morning turns complete roughly 0.20–0.55s later with
the narrower band. The previous lunch turn completes about 0.65s later. The
consecutive left-turn sequence retains its queued cue and finishes later through
the distance fallback. This replay isolates journey decisions. It does not
simulate the missing receiver acknowledgements or measure new lamp extinction.

## Manual steering-force and approach review

The previous deliberate drive has 12,755 model frames. Fixed recorded-input and
ownership replay yields 20 manual requests there and 7 in the morning drive.
Six morning requests have matching lamp-on feedback within three seconds, with
median delay 0.284s. One has no matching lamp onset in that window.

The current detector already combines pressed steering direction confirmed for
150ms, remembered intent, target-boundary approach above 0.4m/s, consistent motion
of both boundaries and 0.3s projection capped at a 1.2m centre-to-boundary threshold.
It requires 0.15m actual movement after episode admission.

| Offline trial | Morning | Previous deliberate drive |
|---|---|---|
| Projection 0.5s with existing 1.2m cap | 7 requests, one 0.204s earlier | 21 requests, five earlier by 0.046–0.298s and one additional candidate |
| Projection 0.5s only with same-direction pressed raw torque at least 350 | 7 requests, one 0.204s earlier | 20 requests, no earlier requests |
| Same torque trial plus 100ms intent confirmation when boundaries approach consistently | Same result | Same result |

The additional candidate in the first trial is the previously reviewed roadside
parking scene. Raw torque 350 is an offline Ioniq 5 PE test value, not a calibrated
cross-vehicle torque threshold. Strong torque alone does not establish a crossing.
One late morning right request follows an active left lever/lamp and only later
gets right pressed torque, then waits for the required actual boundary movement.
The strongest old missed episodes also include inconsistent/widening boundary
motion. Increasing torque weight alone does not resolve those geometry cases.
Manual production timing is unchanged in this review.

## ATC distance setting

`AutoTurnControlRightLaneChange` now stores integer metres, with 0 disabling the
add-on and a UI range of 0–500m in 50m steps. Selecting 250m preserves the former
window. Startup migrates legacy enabled value 1 to 250, preserves 0 and explicit
metre values, and keeps retired ATC mode-4 migration interruption-safe. The runtime
also recognizes a legacy value restored from an older settings backup.

The live source passes the selected distance through both entry and ongoing
TBT checks. Reducing it outside the current distance or setting zero ends the
add-on request. ATC modes 1/2, speed, right lane/edge, BSD and current/projected
front-gap admission still apply. Changing the distance does not change the base
ATC setting.

## Validation and delivery

Local indicator/HUD/migration suites pass 832 tests with two platform skips.
Wiki generation tests pass 25 checks. Changed production Python files pass lint.
Korean/English guides and localized setting descriptions accompany the change.
Normal carrot-wip Git Pull and comma restart apply it, including setting migration.
This task performs read-only vehicle log acquisition and offline verification.
Vehicle installation, physical lamp acceptance and the initial-send root cause
remain pending. No new Jetson runtime bundle is required for these comma changes.


## Publication checks

Source1196a59d34 is published. Jetson CI41535583787 passes2329 checks/56 skips,
232 monitoring,62 HUD,94 update,119 navigation and11 each storage/network checks,
with Windows passing. Native Signal41535582601 passes1285 checks/40 skips and362
subtests. eGPU41535583839 and user docs41535583883 pass. Local web schema/navigation
checks pass52 tests. Wiki c815bd5 has12 reviewed pages with matching semantic hashes.
The first web test run retained the former toggle expectation. Updating that
expectation to the numeric slider/range/step passes the30 corresponding Node tests.
