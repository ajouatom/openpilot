# Tesla Vehicle Integration

On supported Tesla Model 3 and Model Y vehicles, carrotpilot reads vehicle speed, steering state, accelerator input, and brake input from CAN. The harness must match the model year and vehicle hardware. Control is not allowed when a required CAN message is invalid.

## Brake-input compatibility

Some older vehicles transmit the `0x39D` iBooster status message, but this message is absent on some vehicles, including the refreshed Model Y. carrotpilot uses the driver-brake state in `0x145 ESP_status`, which is common across the supported vehicles.

The same signal is used by both layers:

- Vehicle state determines whether the driver is pressing the brake pedal.
- Panda safety independently monitors the brake input and clears control permission when the driver presses the brake while moving.

The former behavior could repeatedly report `0x39D IBST_status not valid` and make `canValid` false because it required a message that the newer vehicle does not transmit. If `0x145` is also absent or has an invalid counter, checksum, or frequency, check the harness and CAN wiring and provide a drive log that includes a brake-pedal press.

> This change does not alter how braking is commanded. It standardizes the CAN source used to detect the driver's brake input.

<a id="automatic-cruise-speed"></a>
## Automatic cruise set speed

Carrot Web lists all six Tesla switches under **Driving > Tesla > Tesla Features**. `TeslaAutoSpeedLimit` (Tesla Automatic Speed Wheel) defaults to OFF and requires a full device restart after changing it. Supported Tesla vehicles also need the additional vehicle CAN bus connected and detected and **alpha longitudinal** (`AlphaLongitudinalEnabled`). While cruise and carrotpilot are engaged, ON adjusts the cruise set speed toward the limit reported by the vehicle's DAS; OFF disables automatic wheel messages and their Panda permission without disabling the driver's wheel, ordinary longitudinal control or other speed-control features. Vehicles using stock longitudinal control do not use it.

Previously eligible vehicles enabled this automatically; after this update they must explicitly turn this switch ON and restart to retain that behavior. It follows the original vehicle limit, not a new Carrot target-speed source, and does not engage cruise by itself. Software checks do not establish physical speed response or road acceptance.

Navigation/Carrot speed control takes precedence: while it is active, DAS-based set-speed adjustment pauses. When it ends, the controller waits for a stable DAS limit again before resuming; any manual wheel pause remains in effect. Simply connecting navigation without active speed control does not pause this feature.

The controller waits for a stable limit, then adjusts the set speed one displayed unit at a time: 1 km/h or 1 mph. It waits at least 0.5 seconds between adjustments and checks that the vehicle has responded before sending another one. Missing or stale limit data, braking, cancellation, or disengagement stops automatic output.

- Turning the right speed wheel manually pauses automatic adjustment, including when limit data is temporarily unavailable. A new speed limit does not clear this pause.
- To resume automatic adjustment, turn the wheel in one direction and then the opposite direction within one second. Disengaging and re-engaging also clears the manual pause.
- If the vehicle does not acknowledge an adjustment, the controller stops retrying that unchanged target and set speed.

## CAN red-light stopping

`TeslaCanTrafficStop` is an independent, default-OFF switch; restart after changing it. It applies only to Model 3/Y with **active openpilot longitudinal control**, not stock TACC or `AlwaysLateral` alone. It does not require automatic turn signals, Share Data, ONNX lane classification or Signal Assist. Other brands ignore it even if their saved parameter is ON.

The receive-only input is bus 2 `0x25D`, exactly eight bytes, decoded as `APP_trafficControl` in metres. Only an available/active traffic-light feature, vision-backed source, vision-light flag, no suppression/unavailable reason and a known colour qualify. Six-byte `DAS_road` uses a different distance scale and is explicitly rejected; do not identify the protocol from the address or vehicle year alone. `carState.teslaTrafficControl` records validity, original receive time, distance, colour and input status. Optional reception does not create missing-hardware CAN faults.

At least three distinct red observations spanning 0.2 seconds may add a stop constraint when the positive stop-line distance is below 126 m. The provisional target is 2 m before the reported line; the vehicle's actual distance reference must be checked before use. Both ACC and blended planning retain other obstacles and model/manual stops. While a CAN stop is active, blended temporarily uses the ACC stop solver, retaining stronger model braking; release restores the selected mode. The code does not forge a model prediction or radar lead, increase braking limits or grant new Panda permissions. A moving lead does not release the CAN constraint.

At least three continuous, same-target green observations spanning 0.5 seconds release only this constraint. Red hold continues at standstill, including a zero reported distance after acquisition; zero alone cannot start a stop. Yellow/unknown is not green. A signal older than 0.5 seconds, inconsistent target or clock discontinuity during an owned stop requests takeover through existing soft-disable while retaining the old stop target during that handover. Soft-disable ultimately disengages control: the driver must brake and take over, not rely on indefinite automatic holding. A target passed by more than 5 m is not replayed inside the junction.

Pedal input cancels this feature for the current engagement; disengage and re-engage before it can acquire a new stop from new samples. A late red can exceed the existing braking capability: a visible/audible warning does not guarantee stopping. Reasons and bounded progress are logged as `tesla_can_traffic_stop`; these are not a new HUD badge. Missing or unsupported input cannot provide red-light stopping.

**Experimental, not vehicle/road acceptance.** First verify raw CAN, stop-line reference, current-direction relevance and actual brake permissions on the exact vehicle, then validate red/green, lead departure, dropout, driver takeover and near-line changes on a closed course. Do not enable this on public roads based only on software tests.

## Automatic turn signals

`TeslaAutoTurnSignals` defaults to OFF and applies after reboot. On Model 3/Y with the additional vehicle bus and fresh physical stalk input, turn lamps start with the existing turn desire, not navigation preparation or a distance-only window. Automatic lane changes still request lamps during preparation and require matching TX and lamp confirmation before starting. It does not enable navigation, change other lane-change gates, or grant steering or departure permission. Unsupported or stalkless signal layouts do not become supported by enabling the switch.

Extension detection is shared with automatic cruise-speed scrolling: `0x3DF` must be observed on CAN1. Working speed scrolling does not replace the separate `0x3E9` idle template and UI/physical lamp feedback required for lamp control. Stalk input prefers CAN1 once observed, with CAN0 compatibility when CAN1 has never supplied that signal; stale or conflicting input blocks automatic output. A short App lane-change command keeps its lamp request through the existing maneuver, not just the command pulse.

The controller preserves original CAN fields and explicitly cancels ATC/App requests after completion or control loss. The separate moving lamp-only permission below does not depend on cruise. Driver operation takes priority. Automatic feedback and cancellation tails are not driver confirmation. Shutdown requires fresh UI and physical OFF confirmation; faults or unavailable feedback are not success. Status is recorded in `carState.autoTurnSignalStatus` and `tesla_auto_turn_signal` events. An unconfirmed cancellation disables further automatic output until the next onroad cycle. CAN loss or device power loss cannot guarantee physical cancellation.

During a maneuver, repeated requests require a new idle template and confirmed TX echo each time; missing templates or lost lamp feedback end the automatic session rather than claiming it remains active. Turn completion uses fresh measured `livePose` heading, not predicted model heading or the steering desire's early turn-disable flag. Selected-side steering must reach 30 degrees and measured rotation 20 degrees; after a peak above 30 degrees, returning within 15 degrees while moving at least 1 km/h completes the lamp request. Stopped drift, repeated poses, discontinuities and missing data cannot establish completion.

Only an already TX/lamp-confirmed turn may retain its lamps at true standstill when lateral output becomes inactive: fresh lateral-enable/active-or-`AlwaysLateral` permission, healthy pose, no steering fault, D and all existing lamp interlocks remain required. This neither starts a new stopped request nor preserves a lane-change session. A pending following turn waits for current completion, a still-current turn desire and a cue no older than three seconds. Same-side handoff retains the request ID; reversal uses the existing cancel/OFF handshake. Distance jumps are only a cue heuristic, not verified route identity. The original 60-second lifetime is not extended.

A physical press, hazards or an unexpected opposite lamp stops automatic output immediately; cancellation is deferred until fresh idle stalk input and both lamps are stably OFF, so it does not deliberately extinguish a driver's signal. Opposite driver torque must move the wheel past zero while moving, after selected-side steering reached 30 degrees, to abandon the lamp journey. Unknown lamp feedback alone never confirms a lane change. Desktop validation does not establish that the exact vehicle will sustain lamps beyond three flashes.

`blocked:vehicle_bus_unavailable` means the extension was not detected; `blocked:driver_input_unavailable` means the physical input is missing, stale or ambiguous. `waiting:body_template`, `waiting:ui_feedback` and `waiting:front_feedback` identify the missing lamp-specific input. `blocked:restart_required` means the enabled setting is absent from the running CarParams. These are logged diagnostics, not new on-screen alerts; a matching Panda firmware is still required.

Validate both directions, manual takeover, hazards, completion and failure handling on the exact vehicle in a closed course before road use. Other Tesla control features and traffic-light departure restrictions remain unchanged.

### Optional lane-change lamp observations

Three independent Tesla-only switches default to OFF and apply after reboot. All require `TeslaAutoTurnSignals`, the existing vehicle CAN/stalk/feedback prerequisites, **Share Data** (`ShareData`) for shared ONNX lane classification, and at least 30 km/h. `OnnxBsdEnabled` remains independent; enabling visual blindspot inference is not required. No ONNX model, inference interval or classification threshold is changed.

- `TeslaManualLaneSignals`: sustained driver torque and paired-boundary approach can request either lamp without a stalk press, even with cruise and lateral control inactive. A small correction or an ordinary curve without this approach/intent evidence does not qualify.
- `TeslaModelDesireSignals`: with lateral control active, a dominant lane-change prediction repeated across at least four unique model frames over 0.2 seconds can request lamps.
- `TeslaModelPathSignals`: independently, with lateral control active, repeated predicted crossing relative to the curved lane boundaries can request lamps. Neither predictive option uses navigation turn announcements or adds turn/U-turn prediction.

Both sides require at least two distinct ONNX dashed updates spanning 0.2 seconds, a completion timestamp at most 0.8 seconds old, confident inner/outer lane geometry and road-edge clearance. The default 400 ms inference cadence remains unchanged. Solid, unknown, invalid, future or stale results prevent new lamp-only requests; genuine stalk operation is never gated by this classifier. Classification distinguishes **solid/dashed, not white/yellow or same-direction traffic**. Geometric support cannot prove legal lane access, free space or safe crossing.

ONNX timestamps mark inference completion, **not camera capture**; bounding that age does not bound total image age. Fresh valid model context is also required, repeated source timestamps cannot establish new evidence, and model discontinuity resets confirmation. Predictions may light early or falsely; detection before tire crossing and detection of every lane change are not guaranteed.

These observations never set a steering desire, enable lateral control, or satisfy the existing ATC/App steering confirmation with their separate request IDs. ATC/App intents retain priority and their original lifecycle; navigation turn preannouncement and distance add-ons remain absent. Owned lamp feedback is not a physical stalk press. Driver input/hazards, invalid context, lost lane evidence, return away from the boundary or a completed crossing end the observation; a lane observation lasts at most 12 seconds within the unchanged 60-second executor bound. Cancellation still uses existing TX/OFF confirmation without retrying a failed session. Desktop tests are not physical lamp or road acceptance.

## Cooperative steering and tire pressure

Cooperative steering blends the driver's steering torque with the planned angle, ramps steering back in on engagement, and removes excess override when an angle limit is reached. Existing EPS override and steering-angle limits still apply.

When the vehicle bus supplies tire-pressure data, the four tire readings are available to the display. A sensor's unavailable value is shown as unavailable rather than as a pressure reading.

<a id="engagement-and-standstill"></a>
## Engagement and standstill

With the vehicle CAN extension, a three-finger contact on the infotainment screen toggles `AlwaysLateral` between 0 and 1, applied live on the next control update. Release before the next contact; holding or duplicate press events do not toggle repeatedly. The shortcut no longer changes Experimental/Chill mode or requires `ExperimentalModeConfirmed`. The existing initialization default and saved values are preserved.

OFF removes only cruise-independent lateral permission: steering can still operate while driving control is active. This gesture is not an emergency disengagement or a longitudinal-control switch.

Tesla standstill detection uses the vehicle's ESP signal independently of the stock cruise state. Supported Tesla vehicles can steer at true standstill when driving control is active, or when `AlwaysLateral` is enabled in a forward-driving gear and lateral control is already on. Steering faults and the lateral-enable switch still block steering; minimum steering-speed limits still apply while moving.

Normal stock TACC operation is not treated as an Autosteer conflict. Where the vehicle provides its Autosteer setting, Autosteer or FSD must remain disabled. Existing steering-conflict checks still apply. A continuously idle stock ACC state no longer repeatedly cancels a new stalk engagement; a real transition from active or holding ACC to a cancel state is still forwarded briefly.

## Wake and parked operation

Valid Tesla power-state messages can keep the device awake during accessory or conditioning operation without putting openpilot on-road. R, N, or D selects the on-road ignition state. In P, an unlatched seatbelt or an open door switches it off-road; with the belt latched and doors closed, P preserves the previous ignition state. Entering P alone therefore does not always switch the driving screen off.

Wake and ignition indications expire independently when their corresponding CAN messages stop. These states control device power and on-road status; they do not engage driving control by themselves.
