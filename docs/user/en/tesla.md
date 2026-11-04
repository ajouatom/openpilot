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

On supported Tesla vehicles with the additional vehicle CAN bus connected and detected, enabling **alpha longitudinal** (`AlphaLongitudinalEnabled`) also enables automatic cruise set-speed adjustment. While cruise and carrotpilot are engaged, the set speed follows the speed limit reported by the vehicle's DAS. This feature has no separate Carrot Web toggle; vehicles using stock longitudinal control do not use it.

Navigation/Carrot speed control takes precedence: while it is active, DAS-based set-speed adjustment pauses. When it ends, the controller waits for a stable DAS limit again before resuming; any manual wheel pause remains in effect. Simply connecting navigation without active speed control does not pause this feature.

The controller waits for a stable limit, then adjusts the set speed one displayed unit at a time: 1 km/h or 1 mph. It waits at least 0.5 seconds between adjustments and checks that the vehicle has responded before sending another one. Missing or stale limit data, braking, cancellation, or disengagement stops automatic output.

- Turning the right speed wheel manually pauses automatic adjustment, including when limit data is temporarily unavailable. A new speed limit does not clear this pause.
- To resume automatic adjustment, turn the wheel in one direction and then the opposite direction within one second. Disengaging and re-engaging also clears the manual pause.
- If the vehicle does not acknowledge an adjustment, the controller stops retrying that unchanged target and set speed.

## Automatic turn signals

`TeslaAutoTurnSignals` defaults to OFF and applies after reboot. On Model 3/Y with the additional vehicle bus and fresh physical stalk input, it requests lamps for the existing ATC turn/lane-change decisions, including turn preparation. It does not enable navigation, change lane-change gates, or grant steering or departure permission. Unsupported or stalkless signal layouts do not become supported by enabling the switch.

Extension detection is shared with automatic cruise-speed scrolling: `0x3DF` must be observed on CAN1. Working speed scrolling does not replace the separate `0x3E9` idle template and UI/physical lamp feedback required for lamp control. Stalk input prefers CAN1 once observed, with CAN0 compatibility when CAN1 has never supplied that signal; stale or conflicting input blocks automatic output. A short App lane-change command keeps its lamp request through the existing maneuver, not just the command pulse.

The controller preserves original CAN fields and explicitly cancels its own request after completion or control loss. Driver operation takes priority. Automatic feedback and cancellation tails are not driver confirmation. Shutdown requires fresh UI and physical OFF confirmation; faults or unavailable feedback are not success. Status is recorded in `carState.autoTurnSignalStatus` and `tesla_auto_turn_signal` events. An unconfirmed cancellation disables further automatic output until the next onroad cycle. CAN loss or device power loss cannot guarantee physical cancellation.

During a maneuver, repeated requests require a new idle template and confirmed TX echo each time; missing templates or lost lamp feedback end the automatic session rather than claiming it remains active. Turn lamps use separate started-turn and stable straightening evidence, not the steering desire's early turn-disable flag. A physical press, hazards or an unexpected opposite lamp stops automatic output immediately; cancellation is deferred until fresh idle stalk input and both lamps are stably OFF, so it does not deliberately extinguish a driver's signal. Unknown lamp feedback alone never confirms a lane change.

`blocked:vehicle_bus_unavailable` means the extension was not detected; `blocked:driver_input_unavailable` means the physical input is missing, stale or ambiguous. `waiting:body_template`, `waiting:ui_feedback` and `waiting:front_feedback` identify the missing lamp-specific input. `blocked:restart_required` means the enabled setting is absent from the running CarParams. These are logged diagnostics, not new on-screen alerts; a matching Panda firmware is still required.

Validate both directions, manual takeover, hazards, completion and failure handling on the exact vehicle in a closed course before road use. Other Tesla control features and traffic-light departure restrictions remain unchanged.

## Cooperative steering and tire pressure

Cooperative steering blends the driver's steering torque with the planned angle, ramps steering back in on engagement, and removes excess override when an angle limit is reached. Existing EPS override and steering-angle limits still apply.

When the vehicle bus supplies tire-pressure data, the four tire readings are available to the display. A sensor's unavailable value is shown as unavailable rather than as a pressure reading.

<a id="engagement-and-standstill"></a>
## Engagement and standstill

Tesla standstill detection uses the vehicle's ESP signal independently of the stock cruise state. Supported Tesla vehicles can steer at true standstill when driving control is active, or when `AlwaysLateral` is enabled in a forward-driving gear and lateral control is already on. Steering faults and the lateral-enable switch still block steering; minimum steering-speed limits still apply while moving.

Normal stock TACC operation is not treated as an Autosteer conflict. Where the vehicle provides its Autosteer setting, Autosteer or FSD must remain disabled. Existing steering-conflict checks still apply. A continuously idle stock ACC state no longer repeatedly cancels a new stalk engagement; a real transition from active or holding ACC to a cancel state is still forwarded briefly.

## Wake and parked operation

Valid Tesla power-state messages can keep the device awake during accessory or conditioning operation without putting openpilot on-road. R, N, or D selects the on-road ignition state. In P, an unlatched seatbelt or an open door switches it off-road; with the belt latched and doors closed, P preserves the previous ignition state. Entering P alone therefore does not always switch the driving screen off.

Wake and ignition indications expire independently when their corresponding CAN messages stop. These states control device power and on-road status; they do not engage driving control by themselves.
