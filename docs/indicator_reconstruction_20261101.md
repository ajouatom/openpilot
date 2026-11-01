# Indicator reconstruction ? 2026-11-01

The owner approved separate model-desire and model-path indication, a 10% target-line admission threshold, recent-intent manual indication, low-speed 250m indication, and distance-anchored TBT turn holding. Opposite steering means a deliberate driver abort with measured reverse heading, not merely crossing steering angle zero.

## Behavior

- Preserve the existing DesireHelper ATC preparation, audio and steering state machine. Observe accepted lane starts through existing autoTurnSignal metadata.
- Add default-OFF ModelDesireIndicator and ModelPathIndicator for Ioniq 5 PE / Jetson USB2CAN / AutoBlinkerControl 1 or 2. Both start only with active lateral control and create physical indication, never another steering request.
- Desire admission: probability >=0.5, dominant over none and opposite direction by >=0.3. Turn minimum 10km/h; lane-change minimum 30km/h.
- Path admission: minimum 30km/h, target-line probability >=0.10, continuous geometry, predicted center path crossing a curved boundary over 1?3s, and agreeing command curvature relative to boundary curvature. Four observations spanning >=0.2s within 0.4s, at least half of the window since first matching observation; reject opposing predictions. Left fresh known-white OEM classification and right edge clearance >=2.5m apply to lane indications. A wide shoulder can pass clearance; this is not proof of a safe lane.
- Manual admission remains independent of ATC/navigation. Anchor target-line confidence >=0.10 within one second when an intent episode starts, then retain only continuous geometry/intent for at most four seconds. Existing torque, stall, retreat and directional exclusion gates remain. Follow an emitted request through completion/return/invalidity, bounded to 12s. Actual geometry can still cancel before a human judges the maneuver complete.
- Journal ownership before transmission. Suppress owned lamp/synthesized-stalk feedback before DesireHelper interpretation, refresh isolation during holds and preserve both direction masks. Same-direction observation/ATC handover retains the request ID. No automatic repeated tapping after an unconfirmed start.
- 250m add-on keeps base ATC 1/2, fresh BSD and full right-front current/projected 5m gates. It additionally checks edge clearance. Minimum indication speed becomes 10km/h. Starts below 30km/h require modeld ownership acknowledgement before transmission and stay indication-only for their entire bounded session, including acceleration. Normal lane-change minimum speed is unchanged.
- TBT source integrates fresh wheel speed, anchors the cue distance plus 30m, freezes distance at a stop, bridges up to one second of navigation publication age only during an already-owned turn, and still requires fresh model/vehicle/pose inputs. Same-direction next-turn anchors can extend the hold; opposite turns wait for completion. Continued rotation can extend another 30m; overall bound remains 300s.
- Abort requires opposite driver torque and angle beyond five degrees, measured reverse heading (over eight degrees from peak or five degrees against the requested direction), and 0.2s confirmation. Natural unwind alone does not cancel. Explicit navigation cancellation/off-route state, distance epoch loss, driver commands and invalid permission cancel.
- Source diagnostics add TBT/observation/origin reasons to existing bounded comma-side physicalBlinkerState reporting. These fields do not enter USB wire packets. Receiver format, CAN bytes/rates, Panda safety, model artifacts and signed Jetson bundle remain unchanged.

## Recorded-input evidence

Four previously hash-verified full-schema segments were replayed locally; raw records and device identifiers remain private. Detector replay uses recorded lamps/controls, not counterfactual lamp feedback, so it is not closed-loop vehicle acceptance.

| Reported window | New logical observation |
|---|---|
| Model path moving right around segment 8 +51s | Right model-path request at 51.313s; geometry return ends it at 53.513s |
| Manual right approach in segment 6 | Right request at 53.541s; geometry return ends it at 54.143s |
| Manual left approach in following segment | Left request at 2.364s; geometry validity ends it at 3.513s |
| Consecutive left turns in segment 4 | TBT output stays left from 33.1s to 53.2s in the zero-phase source replay |
| Second left turn model output | Independent model-turn observation begins at 42.832s; TBT takes precedence in the actual arbiter |

Five 10ms-offset source replays are retained privately. Recorded source receive times are reconstructed from publication times; receiver acceptance and physical lamp holding are not proven by replay. Do not infer that every intended crossing is now detected or that immediate physical extinction works.

## Validation and delivery

- Desktop focused detector, source-loop, policy, receiver and recovery checks: 577 passed.
- Wiki generator checks: 25 passed; user-docs validator and production Python lint passed.
- Published source: `077211fe216f4539624e98ad56a0d55321162567`. Native [Signal CI](https://github.com/ajouatom/openpilot/actions/runs/41265772848) passed 1277 checks, 40 skipped and 362 subtests. Windows cannot execute the complete Linux hardware/clock suite; the native CI supplies that coverage.
- [Jetson CI](https://github.com/ajouatom/openpilot/actions/runs/41265772883) passed 2174 broad checks, 56 skipped, plus supporting suites of 232/62/94/119/11/11 and the Windows installer checks. eGPU compatibility and user-docs CI also passed. Counts overlap; they are not a unique-test total.
- Wiki manual commit `84d263d` and generated publication `bd3f34a` are published. All 15 affected localized pages have current review metadata and verified indexed content hashes; both Wiki workflows passed.
- Normal carrot-wip Git Pull and comma restart deliver the source/schema/settings. No model download, selection change, Jetson bundle update or CAN patch is required.
- New model options remain OFF until selected. No live vehicle write, restart, CAN transmission or road acceptance was performed in this task.
