# Driver monitoring

`DriverMonitoringMode` defaults to `0: Standard`; select `1: Experimental` separately. Mode changes apply live at roughly half-second intervals without rebooting. Switching preserves accumulated monitoring time, warning counts and lockout, and ends the previous interaction grace and forward-attention streak. Old `DisableDM` values do not opt into experimental monitoring.

> [!CAUTION]
> Experimental mode may violate applicable law. Use only for experiments in a controlled test environment. The times below are implementation choices, not statutory allowances or certification. Even mode 0 uses different timing from stock comma when camera monitoring is unavailable.

## Four operating cases

| Camera and mode | Alert timing | Interaction and forward attention |
|---|---|---|
| Available · 0 | Stock comma: vision 5 / 8 / 13 seconds | Stock recovery and reset conditions |
| Available · 1 | 10 / 16 / 26 seconds; 20 / 32 / 52 with empty-road conditions | Fresh input resets and starts 45/90-second grace; two seconds of confident forward attention resets |
| Unavailable · 0 | 15 / 30 / 45 seconds | Fresh eligible input resets |
| Unavailable · 1 | 15 / 30 / 45 seconds; 30 / 60 / 90 with empty-road conditions | Fresh eligible input resets |

Times start from full attention and assume uninterrupted distraction or no response. Do not add the three numbers together. Stock low-speed exemptions below approximately 10 km/h, detection filtering and previous attention debt affect actual warning times. Terminal alerts and lockout are excluded from the resets described below.

Experimental timing and interaction grace below apply outside the 20-second standard hold triggered by traffic appearance.

## When camera monitoring is available

Mode 0 retains comma face, head-pose, eye-closure, sleep and phone detection, input handling and warnings. Normal forward attention recovers monitoring, so lack of interaction alone does not cause periodic vision warnings. Added vehicle/BT buttons do not reset this mode.

Mode 1 uses twice the stock camera warning times, or four times with empty-road conditions, and widens head-pose tolerance by 20%. Eye-closure and sleep detection probability thresholds remain unchanged; the phone threshold is relaxed to 0.98. Warning times for these detections are also extended. Additional head-pose tolerance is not applied during orange or terminal alerts.

Fresh eligible driver or BT input resets monitoring before the terminal stage, including orange alerts. It then **defers camera warnings for 45 seconds, or 90 seconds with empty-road conditions**. The camera warning clock starts after this grace expires. If distraction persists immediately after input and conditions stay constant, the first warning can occur around 45+10=55 seconds later, or 90+20=110 seconds on an empty road. Terminal timing can likewise extend to approximately 71 or 142 seconds.

> [!WARNING]
> This grace also delays sleep, eye-closure and phone warnings. An interaction does not prove wakefulness or forward attention. Camera detections continue during grace, but its warning clock does not accumulate.

A forward-attention evidence score of at least 0.9 for two seconds resets monitoring. It combines face/eye confidence, head pose, eye closure, sleep, phone and sunglasses signals; it is not a calibrated wakefulness probability. This reset does not start another interaction grace. The former two-second credit and 1.5x recovery bonus are no longer used.

Healthy camera data with a missing face or uncertain model can still select the stock internal interaction fallback. Mode 0 retains stock timing; mode 1 uses the same interaction timing as unavailable-camera mode 1: 15/30/45 or 30/60/90 seconds.

## When camera monitoring is unavailable

Missing hardware, faults, malformed output or interrupted data automatically select interaction monitoring. This does not identify the physical fault, and no installation setting is required. Two seconds of healthy camera data restore camera monitoring without resetting monitoring progress merely because its source changed.

Both modes start with 15/30/45-second interaction timing. Only mode 1 doubles this to 30/60/90 with empty-road conditions. Fresh eligible input before the terminal alert resets the entire allowance. For example, a BT press after 20 seconds without input restarts the first warning approximately 15 seconds later under base conditions.

Without a camera, forward attention, eye closure and sleep cannot be observed directly; forward-attention reset cannot apply. A persistent camera-unavailable notice identifies interaction monitoring. Road-camera, vehicle-communication and monitoring-process failures retain separate handling.

## Empty road and new traffic

The additional doubling requires enabled, supported Hyundai/Kia corner radar, healthy radar and road-model data, and ten seconds of stable straight-road conditions without moving traffic in the observed area. Lane confidence, steering angle, yaw rate, acceleration and turn indicators are checked. Missing or stale context cannot earn the bonus.

Moving observations must be within -10 to 150 m longitudinally and 6 m on either side of the path, with **absolute ground speed at least 2 m/s (about 7.2 km/h)**. Equal-speed lead traffic counts; stationary and slow observations are excluded. This does not prove that stopped vehicles or blind spots are absent. Even a single candidate immediately revokes the empty-road bonus; approximately 0.2 seconds of continuous confirmation starts the 20-second hold. This is not complete radar-noise rejection.

Only the transition from **no moving traffic to approximately 0.2 seconds of confirmed moving traffic starts 20 seconds of standard monitoring**. A healthy camera uses stock 5/8/13-second timing, detection and input handling; an unavailable camera uses 15/30/45-second interaction monitoring. Traffic presence alone does not force a warning while forward attention is normal.

Neither continued observation nor additional vehicles extend the hold. Experimental monitoring resumes after 20 seconds, but remaining traffic still prevents the empty-road bonus. All observations must disappear beyond the two-second retention of the last observation before a later moving appearance starts another hold.

Entry ends previous camera interaction grace and the forward-attention streak; BT, button and touch events cannot start experimental grace during the hold. Accumulated monitoring time, warnings and lockout remain; expiry cannot restore old grace or erase orange/terminal alerts. Invalid radar/road context uses standard criteria until healthy information returns.

## Eligible interactions

Hyundai/Kia/Genesis CAN-FD platforms process original `STEER_TOUCH_2AF` reception without a vehicle-name whitelist. Message layout, checksum, counter progression and freshness must pass validation; contact starts at the lowest reported level, `TOUCH_DETECT=1`. Small fluctuations in raw `TOUCH1/2`, or CAN address `0x2AF` alone, are not contact evidence.

Signals arriving after initial startup detection are also checked. Missing or invalid signals provide no touch credit. Physical touch behavior on other vehicles has not been validated; existing ADAS transmissions and torque-based steering detection are unchanged.

With camera monitoring unavailable, both modes reset the interaction timer while valid contact continues. Releasing the wheel or losing the signal for more than 0.25 seconds resumes the normal timer. With a healthy camera in mode 1, only a new contact after a valid release starts interaction grace; holding the wheel or reconnecting does not repeatedly renew it. Healthy-camera mode 0 remains stock. Touch does not prove forward attention or wakefulness and cannot clear terminal alerts or lockout.

New DM2 input handling recognizes the start of vehicle-reported pedal/steering input and new presses of supported cruise, gap and steering-assistance buttons. Held inputs, automatic speed changes and BT repeat events do not count again. Stock steering/gas handling remains unchanged in camera mode 0.

Vehicle speed buttons are excluded where stock ACC uses automatic speed-button injection because physical input cannot reliably be distinguished from an echo. Pedals, steering and BT remain available. Registered and enabled BT remotes count actual clicks or the first long-press event, even for an unmapped button. Connection keepalives, learning/test events and stale events do not count.

## Terminal warnings and web video

Once a terminal alert is reached, input, forward attention or context changes alone cannot clear it. Existing deceleration requests and lockout remain while driving. This does not introduce guaranteed emergency stopping, and stock ACC cannot be assumed to execute equivalent deceleration.

**One continuous second of valid Park, standstill and disengaged status** resets accumulated warnings and the usage restriction. This exception to stock comma behavior applies in both modes with or without a camera. Engagement remains manual after parking, and monitoring continues.

The vehicle must report zero raw speed, standstill and Park together; only a tiny settling residue in filtered speed is allowed. Zero speed in Drive, Neutral or Reverse, engagement OFF/ON alone, stale or invalid signals, and driver-camera preview cannot release the restriction. Vehicles that do not report Park cannot use this release condition.

`CarrotVisionEnabled` independently controls web road video. Only the video function of old `DisableDM=2` is migrated once; monitoring starts in standard mode. Carrot Vision is unavailable while the USB cluster is enabled.

## Onroad DM display and warning sounds

C4 displays an 84×84 DM inset immediately to the right of the D gear indicator, clear of the right status strip. VISION information sits above it; C3/C3X retain their existing position below the clock.

The inset appears while engaged or with Always On DM enabled, and hides during displayed alerts. An unavailable camera shows a steering-wheel symbol; stale data shows a check state instead of retaining an old face image.

Warnings progress from a silent visual notice to the first audible warning and then the final audible warning. **The first sound has a 70% minimum gain; the final sound always uses 100%.** General/engagement volume settings and ambient-noise attenuation cannot reduce these levels.

This sound policy applies to modes 0 and 1 with camera or interaction monitoring. Navigation and other events sharing the same sound assets retain their existing volume; percentages describe software output gain, not measured speaker sound pressure.
