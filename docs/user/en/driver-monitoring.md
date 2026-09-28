# Driver monitoring

`DriverMonitoringMode` defaults to `0: Standard`; `1: Experimental` must be selected separately. Reboot the device after changing it. Old `DisableDM` values do not enable experimental mode.

> [!CAUTION]
> Experimental mode may violate applicable law. Use only for experiments in a controlled test environment. Public-road legality, safety and country-specific certification are not assured. Mode 0's comma criteria are not a statement of the legal limit in every country.

## When camera monitoring is available

Mode 0 uses comma's face, head-pose, eye-closure, sleep, phone and alert criteria. Mode 1 increases head-pose tolerance by 20%. Eye-closure, sleep and phone criteria, and the existing terminal-alert response, remain unchanged.

In mode 1, a new pedal, steering, vehicle-button or Bluetooth-button interaction restores up to two seconds of ordinary distraction allowance. This extra recovery is limited to once per second and cannot accumulate beyond full awareness. It does not apply to orange or terminal alerts, current eye-closure, sleep or phone detections, or remaining attention debt from those detections.

When a forward-attention evidence score remains at least 0.9 for two seconds, ordinary distraction recovers 1.5 times faster. The score combines face direction, both eyes' detection confidence, and eye-closure, sleep, phone and sunglasses probabilities; it is not a calibrated probability or guarantee of gaze or wakefulness. This additional recovery is limited to mode 1 and does not apply to orange or terminal alerts or remaining sleep, eye-closure or phone attention debt.

## When the camera is absent or fails

Missing hardware, failure, malformed output or stale data automatically selects monitoring through button, pedal and steering interactions. This does not diagnose physical absence, and no camera-installation setting is needed. Camera monitoring resumes after two continuous seconds of healthy camera DM data.

| Mode | Alert-stage criteria without interaction | Condition |
|---|---|---|
| 0: Standard | 5 / 15 / 25 seconds | Comma's standard interaction-monitoring times |
| 1: Experimental | 10 / 30 / 50 seconds | Twice the standard budget with healthy surrounding data |
| 1: Additional relaxation | Up to 12 / 36 / 60 seconds | Conditionally up to 2.4 times the standard budget |

Actual alert timing also depends on low-speed exemptions, interaction recovery and the current alert stage. A pedal or eligible speed-button interaction adds 0.2 to the multiplier for 15 seconds. Ten seconds of stable travel on an observed empty straight road adds another 0.2. Losing these conditions can shorten the applicable allowance.

The empty-road bonus requires supported Hyundai/Kia corner radars to be enabled and healthy radar and road-model data. It also requires no valid observed objects ahead or to either side and suitable lane confidence, steering angle, yaw rate and acceleration. An empty detection list is not evidence that sensor blind spots are empty. Missing or stale surrounding data selects standard criteria.

Persistent camera unavailability displays “Driver camera unavailable / Monitoring driver controls.” Changing monitoring source preserves monitoring progress and orange/terminal alerts. Interaction monitoring cannot directly verify sleep or forward gaze.

## Vehicle entry and accepted interactions

A newly observed moving vehicle ahead or in an adjacent lane selects standard criteria for ten seconds, including in mode 1. Continuous observation of the same vehicle and brief detection flicker do not continually restart the timer; another new vehicle may restart it. Expiry does not erase accumulated nonresponse time or strong alerts.

Pedal and steering signals count on a new press; vehicle buttons count on a new button-down event. A continuously held input or an automatic speed change is not a new response. Vehicle speed-button credit is excluded on stock-ACC configurations that inject automatic speed-button commands, where physical input and gateway echoes can be ambiguous. Pedal, steering and Bluetooth inputs remain available.

A registered, enabled Bluetooth remote contributes actual clicks and the first recognized long press, including buttons without a driving action assigned. Connection presence, learning/test input, stale events and automatic hold repeats do not earn recovery. An interaction is evidence of a response, not proof that drowsiness has ended.

## After alerts and web video

The existing terminal-alert deceleration request and lockout remain connected. This change does not provide a new emergency-stop feature or guarantee a stop; equivalent deceleration cannot be guaranteed on stock-ACC vehicles. It adds no immediate steering release solely because a DM timeout expires. Road-camera, vehicle-communication and DM-process failures remain separately monitored.

Web road video uses the independent `CarrotVisionEnabled` setting. On first migration, old `DisableDM=2` preserves its video function through this setting while driver monitoring starts in standard mode. Carrot Vision is unavailable while the USB cluster is enabled.
