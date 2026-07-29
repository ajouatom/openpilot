# Tesla Vehicle Integration

On supported Tesla Model 3 and Model Y vehicles, carrotpilot reads vehicle speed, steering state, accelerator input, and brake input from CAN. The harness must match the model year and vehicle hardware. Control is not allowed when a required CAN message is invalid.

## Brake-input compatibility

Some older vehicles transmit the `0x39D` iBooster status message, but this message is absent on some vehicles, including the refreshed Model Y. carrotpilot uses the driver-brake state in `0x145 ESP_status`, which is common across the supported vehicles.

The same signal is used by both layers:

- Vehicle state determines whether the driver is pressing the brake pedal.
- Panda safety independently monitors the brake input and clears control permission when the driver presses the brake while moving.

The former behavior could repeatedly report `0x39D IBST_status not valid` and make `canValid` false because it required a message that the newer vehicle does not transmit. If `0x145` is also absent or has an invalid counter, checksum, or frequency, check the harness and CAN wiring and provide a drive log that includes a brake-pedal press.

> This change does not alter how braking is commanded. It standardizes the CAN source used to detect the driver's brake input.
