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

The controller waits for a stable limit, then adjusts the set speed one displayed unit at a time: 1 km/h or 1 mph. It waits at least 0.5 seconds between adjustments and checks that the vehicle has responded before sending another one. Missing or stale limit data, braking, cancellation, or disengagement stops automatic output.

- Turning the right speed wheel manually pauses automatic adjustment, including when limit data is temporarily unavailable. A new speed limit does not clear this pause.
- To resume automatic adjustment, turn the wheel in one direction and then the opposite direction within one second. Disengaging and re-engaging also clears the manual pause.
- If the vehicle does not acknowledge an adjustment, the controller stops retrying that unchanged target and set speed.

## Cooperative steering and tire pressure

Cooperative steering blends the driver's steering torque with the planned angle, ramps steering back in on engagement, and removes excess override when an angle limit is reached. Existing EPS override and steering-angle limits still apply.

When the vehicle bus supplies tire-pressure data, the four tire readings are available to the display. A sensor's unavailable value is shown as unavailable rather than as a pressure reading.
