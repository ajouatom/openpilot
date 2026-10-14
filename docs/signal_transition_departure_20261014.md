# Signal transition departure and USB status - 2026-10-14

The latest stopped red-to-green failures were reproduced from the Ioniq's
recorded camera and planner inputs. This change corrects the stop guard's
identity/grouping failures and, as requested, supplies a confirmed transition
as the existing planner's green input. It does not retrain or replace the ONNX.

## Evidence

147 completed-segment qlogs were screened. Seven selected stopped segments
were collected with rlog, full road video and qcamera: 168 files total,
453,916,442 bytes, SHA256 checked against the device. The device read-only
inventory reported carrot-nav `5e85695197` and SignalAssistMode 2. Unrelated
Jetlink work was present on the device and was not modified.

* **October 14, 06:46:23, route 10b8 segment 2:** producer track 317 retained
  identity/red history and reported a real green. The stopped guard received
  neither the ID nor that history. Its last red box `[647,288,683,298]` and
  green box `[647,286,671,298]` have IoU 0.588, below its 0.6 association gate.
  The guard stayed held until the driver's override.
* **October 13, 22:13:49, route 10b7 segment 1:** two overhead signals changed
  from red to green together. Much smaller reds lower in the image (visually
  a separate, more distant signal group) vetoed both transitions. Exact road
  EncodeIdx mapping and sequential HEVC decoding confirm the real green
  circles beside left arrows in both reviewed cases. Arbitrary HEVC seeking
  was not used for final frame verification.
* Correcting the guard alone released these two holds but left the native
  classifier red. Consequently the reconstructed planner remained stopped.
  This explains why a veto-only correction was insufficient.

## Behavior

The producer now carries its session-local track ID and red-history flag in
the existing stopped-guard transport. Ordinary IoU matching remains intact.
Only the same ID with producer red history can use the bounded resize match:
IoU >=0.4, area ratio 0.5 to 2, centre displacement <= one quarter of the fixed
red box's width/height. Green never moves the red reference.

The two-housing fallback may exclude another red only when its width and
height are <=70% of BOTH red-anchored green housings, and its top is at least
two housing heights below BOTH bottoms. Both green transitions still require
independent confirmed red history, >=3 green samples and >=700ms. Same-row,
larger, overlapping, ambiguous or insufficiently separated reds continue to
veto. A single green cannot use this exclusion. Conflicting red restarts green
confirmation. The anchors remain after release so fresh paired green does
not immediately reacquire the same separate lower reds; loss of that pair
removes the exception. Identity remains session scoped and is cleared by
movement beyond the initial launch, disable or restart.

This is an image grouping hypothesis, not measured distance or proven
signal-to-ego-lane association. It is not permission to ignore arbitrary red.

A confirmed release can now supply `trafficState=green` to the existing
stop/departure state machine while below 5 km/h, including its usual stopped
countdown. It requires active/valid Drive context and SignalAssistMode 1 or 2,
TrafficLightDetectMode 2, and a non-High driving mode. It does not override
soft hold, brake/gas, either blinker, navigation stay-stop, or a manual stop.
It does not directly set acceleration or bypass radar/MPC obstacles. After
5 km/h the helper transition is cleared and ordinary moving-model behavior
continues. Freshness stays 650ms; unknown, invalid context, stale packets or
camera restart cannot supply a retained green command. A repeated packet
never extends capture freshness. The master mode 0 and highway suspension
still clear helper participation.

## USB / HUD display

LongitudinalPlan appends `signalObservationColor` and `signalAssistAction`.
The USB/C3 traffic circle and C4 model lamp consume these planner fields;
the cache key includes both. No additional inference or GPU use is added.

* Outer ring: current helper evidence, red / green / amber for mixed.
* Centre dot: red = actual departure veto, amber = actual stop preparation,
  green = confirmed transition supplied to the departure state machine.
* Grey ring with red dot: current color is unavailable but the red veto
  remains latched. This is not a fresh red detection.
* No marks: no current helper evidence or action. Invalid/dead plans hide
  marks; highway pause still replaces the lamp with its pause symbol.

The existing lamp itself retains its previous source/meaning. A green ring
alone is evidence, not a go command. The native C4 fade cannot hide an active
helper mark. Marks fit within the existing lamp bounds. The preview was
rendered from the drawing function with a desktop drawing adapter, not a
photograph or hardware verification of the USB panel. Schema changes require
the ordinary native rebuild after pull.

## Validation and limits

* Focused and related desktop suite: 760 tests pass (Windows Params storage
  stub, real Python logic and Cap'n Proto schema; no vehicle inference).
* Latest recorded-input A/B replay: 42,064 variant/planner records across
  seven segments and nominal/+20ms/+100ms arrival delays. Old guard has no
  release in the two reviewed failures; corrected guard releases both.
  With the green input connection, the reconstructed state machine reaches
  e2eCruise with a positive cruise target about 0.55s / 0.70s after release,
  before the recorded manual overrides; no rehold in those windows.
  Cruise target is NOT predicted vehicle speed or acceleration.
* Historical replay: 43,090 planner inputs at nominal and again +20ms.
  Zero reviewed-red release events; the six release events remain in the
  same reviewed-green windows. The two 1095 red-departure windows remain
  blocked. Historical 1091 segment 20 still has four non-stop state samples
  without acquisition, unchanged from the previous baseline; this change
  does not solve every missed-red case.
* No image detector thresholds, ONNX, model selection, moving preparation
  policy, or DM policy changed. The previously diagnosed 10a9 housing-color
  miscrop remains a separate perception limitation.

These are open-loop recorded-input state-machine checks, with parameter
polling/navigation lookup stubbed and recorded/default context. They are not
closed-loop MPC/CAN/vehicle-response validation. Vehicle installation and
physical display/native timing validation were not performed in this task.

Private evidence/scripts/video remain in
`.analysis/scratch/2026-10-14-signal-transition/`; no recordings are published.
Publish the shared correction to maintained carrot-nav and carrot-wip with
single-parent commits, preserving each branch's model/navigation choices.
