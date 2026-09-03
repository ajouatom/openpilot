# Understanding Settings

[한국어](../ko/settings.md)

> [!NOTE]
> This is the canonical English user guide maintained with the `carrot-wip` code. When user-visible behavior changes, update this document together with the related code and tests.

Use **Carrot Web** to view and change all carrotpilot-specific settings. The device settings screen remains useful for Wi-Fi, device information, standard openpilot toggles, and software updates. Parameters defined by `carrot_settings.json` belong in the **Settings** screen in Carrot Web.

> [!IMPORTANT]
> **Current support status**
>
> - Supported settings interface: **Carrot Web**
> - CarrotMan: not supported as a user app or external integration
> - CarrotLink: not supported
> - Older CarrotMan or CarrotLink connection instructions no longer describe the current workflow
>
> A `carrotMan` name may still appear inside code or messages. It does not mean that the former user app is supported.

## Connecting to Carrot Web

1. Connect the comma device and your phone or computer to the same network.
2. Open `http://DEVICE-IP:7000` in a browser.
3. Select **Settings** from the top menu.

For example, if the device IP is `192.168.0.25`, open:

    http://192.168.0.25:7000

See [Carrot Web](https://github.com/ajouatom/openpilot/wiki/Guide-Carrot-Web) for connection troubleshooting and an overview of the other screens.

## Using the Settings screen

Carrot Web provides:

- **Category navigation** through category, group, and section
- **Search** by display title or parameter name
- **Favorites** by pressing and holding an item
- **Profiles** to save and restore named groups of values
- **Compare** to inspect a profile or backup before applying it
- **Reset** to restore the catalog defaults
- **File and QR backups** from the tools screen

A full reset or a backup from another vehicle can change many values at once. Save the current settings as both a profile and a file first.

### Per-setting detailed guides

Select the body of a setting, outside its value control, to open its detail screen. The information box below the existing setting box always shows **Description**, **Popular**, and **History** tabs, with **Description** selected by default. A tab remains available and shows an empty state when it has no data.

- **Description** first uses the setting guide for the current language from the validated GitHub published content.
- Content is shown only when both the published index and guide-page hashes match.
- If the request or validation fails, Carrot Web uses the last validated browser copy.
- If neither a validated remote version nor a validated browser copy exists, it shows an empty state instead of using an on-device document.
- **Popular** shows the collected value distribution as a reference, not as a recommendation.
- **History** shows the latest three changes first and offers the full history when more records exist.
- Network or documentation processing failures do not disable viewing or changing the setting.
- Korean and English Carrot Web sessions use their matching guide language.
- Until a Chinese guide exists, Chinese titles and short descriptions remain Chinese while the detailed panel explicitly falls back to English.
- In the GitHub Wiki sidebar, `User Guide > Understanding Settings > All Settings` follows
  Carrot Web's Korean menu hierarchy and order. The central settings catalog remains the
  multilingual Korean, English, and Chinese directory.

The information box stays vertically below the existing setting box in both portrait and landscape layouts. Multiplier and default actions keep their existing position and behavior inside the setting box.

Validated published guides are generated against the current setting code's defaults, ranges, and options. If a short catalog description and the detailed guide differ, diagnose the issue together with the current branch's setting code.

## Reading values and defaults

Each entry in `carrot_settings.json` contains:

| Field | Meaning |
|---|---|
| `name` | Unique name used by the code and Params |
| `title` | Title shown in Carrot Web |
| `descr` | Direction of adjustment, mode numbers, and cautions |
| `min` / `max` | Allowed range |
| `unit` | Step used by the `+` and `-` controls |
| `default` | Initial catalog value |

### The current value comes first

Persistent values stored on the device may remain after an update. Record the value currently shown by your device before relying on the JSON `default`, a number in a title, or another user's value.

A catalog default is not a recommended tune for every vehicle. Vehicle tuning, existing Params, and the branch from which the device was updated can all affect the starting value.

### Check the scale and unit

| Display example | Stored value | Effective value |
|---|---:|---:|
| `TFollowGap1 x0.01s` | `110` | 1.10 seconds |
| `SteerActuatorDelay` | `30` | 0.30 seconds |
| `LateralTorqueAccelFactor x0.001` | `2500` | 2.500 |
| `StopDistanceCarrot` | `600` | 6.00 m (600 cm) base value |
| Percentage `%` | `105` | 105% |

Ignoring `x0.01`, `x0.001`, `cm`, `km/h`, or `%` can make a value appear one hundred or one thousand times larger than it really is.

## Settings map

The current `carrot_settings.json` contains **171 parameters**. Every entry is assigned to one of these menus:

| Category | Count | Groups |
|---|---:|---|
| Driving control | 107 | Startup and auto, buttons and presets, steering, speed and deceleration, cruise and following gap |
| Vehicle and hardware | 16 | Hyundai/Kia, CAN FD/HDA, radar, driver monitoring, vehicle assistance, device hardware |
| Display | 37 | Information, path, brightness/on-road view, external HUD |
| System | 11 | Recording/power, network/map, sound, software |

## Driving control

These 107 settings can affect vehicle motion. Change one item at a time.

<a id="start-auto"></a>
### Startup and auto — 8 settings

| Section | Parameters | Purpose |
|---|---|---|
| Startup | `AlwaysLateral`, `AutoEngage`, `DisableMinSteerSpeed` | Always-on lateral control, automatic engagement, and low-speed steering limits |
| Auto cruise | `AutoCruiseControl`, `AutoGasTokSpeed`, `AutoGasCancelSpeed`, `AutoGasSyncSpeed`, `CruiseOnDist` | Automatic cruise activation and accelerator-pedal behavior |

- `AlwaysLateral` permits lateral control even when cruise is not engaged.
- `AutoEngage`: `0` off, `1` lateral on, `2` lateral on with cruise ready.
- `AutoCruiseControl` covers Hyundai/Kia auto-cruise and soft-hold behavior.
- `DisableMinSteerSpeed` is vehicle-specific and relates to low-speed steering restrictions on SMDPS-equipped cars.

### Buttons and presets — 15 settings

Select a section title for the code-based state machine, units, and application conditions.

| Section | Parameters | Purpose |
|---|---|---|
| [Button modes](buttons-presets.md#button-modes) | `CruiseButtonMode`, `CancelButtonMode`, `LfaButtonMode`, `PaddleMode` | Cruise, cancel, LFA, and paddle-button behavior |
| [Speed units](buttons-presets.md#speed-units) | `CruiseSpeedUnit`, `CruiseSpeedUnitBasic`, `CruiseButtonLongDelay` | Short/long-press increments and long-press timing |
| [Button-message tests](buttons-presets.md#button-spam) | `CruiseButtonTest1`, `CruiseButtonTest2`, `CruiseButtonTest3` | Values used to synchronize the stock SCC set speed |
| [Speed presets](buttons-presets.md#speed-presets) | `CruiseSpeed1` through `CruiseSpeed5` | Speed table used by custom mode 3 |

The result depends heavily on whether the car uses stock SCC and which button message the vehicle accepts. Diagnose unexpected behavior with the normal `CruiseButtonMode=0` behavior first.

<a id="vehicle-steering"></a>
### Vehicle steering — 36 settings

| Section | Parameters | Purpose |
|---|---|---|
| Centering | `PathOffset`, `CameraYawTrimDeg` | Path position and camera-yaw trim |
| Steering feel | `SteerActuatorDelay`, `LatSmoothSec`, `LatSuspendAngleDeg`, `CustomSR`, `SteerRatioRate` | Timing, smoothing, suspension angle, and steering ratio |
| [Lane change](lane-change.md) and automatic turn | `LaneChangeNeedTorque`, `LaneChangeDelay`, `LaneChangeBsd`, `LaneLineCheck`, `AutoTurnControl`, `AutoTurnControlSpeedTurn`, `AutoTurnControlTurnEnd`, `AutoTurnMapChange` | Lane-change entry conditions and ATC behavior |
| Lane mode | `LatMpcPathCost`, `LatMpcMotionCost`, `LatMpcAccelCost`, `LatMpcJerkCost`, `LatMpcSteeringRateCost`, `LatMpcInputOffset`, `UseLaneLineSpeed`, `UseLaneLineCurveSpeed`, `AdjustLaneOffset` | Lane-mode MPC weights and lane-line conditions |
| Advanced torque | `LateralTorqueCustom`, `LateralTorqueAccelFactor`, `LateralTorqueFriction`, `LateralTorqueKpV`, `LateralTorqueKiV`, `LateralTorqueKf`, `LateralTorqueKd` | Custom torque-control gains |
| Steering limits | `CustomSteerMax`, `CustomSteerDeltaUp`, `CustomSteerDeltaDown`, `CustomSteerDeltaUpLC`, `CustomSteerDeltaDownLC` | Maximum torque and torque-rate limits |

A larger `SteerActuatorDelay` compensates by commanding earlier. A larger `LatSmoothSec` is smoother but may respond more slowly. Changing both together makes diagnosis difficult.

The default `SteerRatioRate` of `100%` applies the learned steering ratio without scaling. It is used when `CustomSR=0`; a stored rate outside the allowed range (`30–200%`) safely falls back to `100%`.

`LateralTorqueCustom` and `CustomSteer*` are advanced settings that can affect the vehicle tune and safety limits. Do not alter them without a vehicle-specific validated baseline and a recovery path.

### Speed and deceleration — 22 settings

| Section | Parameters | Purpose |
|---|---|---|
| [Speed cameras](speed-deceleration.md#speed-camera) | `AutoNaviSpeedCtrlMode`, `AutoNaviSpeedCtrlEnd`, `AutoNaviSpeedDecelRate`, `AutoNaviSpeedSafetyFactor`, `AutoNaviCountDownMode`, `VehicleNaviCanControl`, `VehicleNaviSchoolZoneControl`, `VehicleSpeedCameraControlMode`, `VehicleSpeedCameraDistanceTime` | Event types, stock-navigation CAN, deceleration start, and target speed |
| [Road speed limit](speed-deceleration.md#road-speed-limit) | `AutoRoadSpeedLimitOffset`, `AutoRoadSpeedAdjust`, `AutoSpeedUptoRoadSpeedLimit` | Desired-speed adjustment from the road limit |
| [Speed bumps](speed-deceleration.md#speed-bump) | `AutoNaviSpeedBumpTime`, `AutoNaviSpeedBumpSpeed` | Completion time and crossing speed |
| [Curves and turns](speed-deceleration.md#curve-turn) | `AutoCurveSpeedFactor`, `AutoCurveSpeedLowerLimit`, `TurnSpeedControlMode`, `MapTurnSpeedFactor`, `ModelTurnSpeedFactor`, `ApplyModelSpeed` | Curve speed from model curvature and route data |
| [Traffic lights](speed-deceleration.md#traffic-light) | `TrafficLightDetectMode`, `TrafficStopDistanceAdjust` | Stop/go detection, stop-position adjustment, and automatic stopped-vehicle alignment |

`AutoNaviSpeedCtrlMode` is `0` off, `1` fixed speed cameras, `2` cameras plus speed bumps, or `3` those events plus mobile-camera events.

`VehicleSpeedCameraControlMode=2` treats a new accelerator press after vehicle-received camera deceleration has actually begun as a request to ignore the current event. It keeps the highest speed reached while accelerating as the floor until the event ends; an accelerator held from before deceleration began does not start the override.

A lower `AutoNaviSpeedDecelRate` begins slowing farther away. `AutoNaviSpeedSafetyFactor` applies a percentage of the event limit as the target. Before tuning either value, confirm that the event type, limit, and remaining distance are being received correctly.

`TrafficLightDetectMode` is `0` off, `1` stop detection, or `2` stop and go detection. This is model-based assistance; the driver must always verify the signal.

### Cruise and following gap — 31 settings

| Section | Parameters | Purpose |
|---|---|---|
| [Driving mode](cruise-gap.md#driving-mode) | `MyDrivingMode`, `MyDrivingModeAuto` | Eco, safe, normal, high-speed modes and automatic selection |
| [Speed-based acceleration](cruise-gap.md#acceleration-table) | `CruiseMaxVals0` through `CruiseMaxVals6` | Maximum acceleration tendency by speed band |
| [Stopping and restarting](cruise-gap.md#stop-resume) | `StopDistanceCarrot`, `StoppingAccel`, `VEgoStopping`, `AChangeCostStarting` | Stop position, stop entry, and restart behavior |
| [Longitudinal tuning](cruise-gap.md#longitudinal-tuning) | `LongTuningKpV`, `LongTuningKiV`, `LongTuningKf`, `LongActuatorDelay` | Default Kp/Ki/Kf `100/0/100` and vehicle-response delay |
| [Following gap](cruise-gap.md#following-gap) | `TFollowGap1` through `TFollowGap4`, `DynamicTFollow`, `DynamicTFollowLC`, `EnableSpeedTF`, `TFollowDecelBoost` | Gap times, dynamic gap, and deceleration margin |
| [Lead response](cruise-gap.md#lead-response) | `LeadAccelResponse`, `JLeadFactor3`, `RadarReactionFactor` | TF1 acceleration and other responses to lead-vehicle changes |
| [Carrot cruise](cruise-gap.md#carrot-cruise) | `CruiseEcoControl`, `CarrotCruiseDecel`, `CarrotCruiseAtcDecel` | Economy control and cruise deceleration limits |

`MyDrivingMode` is `1` eco, `2` safe, `3` normal, or `4` high speed. High-speed mode ignores traffic-light control and increases acceleration tendency, so read its behavior before selecting it.

`TFollowGap1` through `TFollowGap4` are stored in hundredths of a second. Lower values reduce the time gap. Establish a stable fixed-gap baseline before enabling `DynamicTFollow` features.

`LeadAccelResponse` adjusts how quickly the vehicle follows a lead starting or accelerating at following-distance level 1. Levels 1–2 are gentle, 3 is the everyday balance, and 4 strongly maintains distance without direct overshoot. Test level 5 prioritizes `TFollowGap1` only during positive lead acceleration, permits up to `0.2 m/s²` acceleration overshoot, and immediately returns to normal control when lead acceleration ends. See [Lead-vehicle response](cruise-gap.md#lead-response) for its activation gates and per-level limits.

`LongTuning*`, `LongActuatorDelay`, and `StoppingAccel` are advanced settings that directly affect vehicles using openpilot longitudinal control. Some have no effect when stock ACC remains responsible for acceleration and braking.

<a id="vehicle-hardware"></a>
## Vehicle and hardware

These 16 settings describe the car, harness, and device hardware configuration. Do not enable them merely as a display experiment.

| Group | Parameters | Purpose |
|---|---|---|
| Hyundai/Kia | `HyundaiCameraSCC`, `IsLdwsCar`, `HapticFeedbackWhenSpeedCamera` | SCC connection, LDWS behavior, and speed-event haptics |
| CAN FD/HDA | `CanfdHDA2`, `CanfdDebug`, `HDPuse` | HDA2 selection, CAN FD diagnostics, and HDP |
| Radar | `EnableRadarTracks`, `EnableCornerRadar`, `CarrotRadarMode`, `CarrotRadarCutInSensitivity` | SCC radar, raw tracks, corner radar, and Carrot Radar processing and cut-in sensitivity |
| Driver monitoring | `DisableDM`, `MuteDoor`, `MuteSeatbelt` | Driver monitoring and selected vehicle alerts |
| Vehicle assistance | `MaxAngleFrames`, `SpeedFromPCM` | Steering-angle frames and stock-SCC speed control |
| Device hardware | `HardwareC3xLite` | Speakerless C3X Lite audio and process configuration |

> [!CAUTION]
> Incorrect `HyundaiCameraSCC`, `CanfdHDA2`, `EnableRadarTracks`, `CarrotRadarMode`, `CarrotRadarCutInSensitivity`, or `SpeedFromPCM` values can change vehicle identification, SCC, radar, or longitudinal behavior. Confirm the vehicle, model year, HDA generation, harness location, and whether stock ACC is retained.

See [Radar tracks and corner radar](radar.md) before changing radar modes.

For dPath RadarD, `EnableRadarTracks=-2` is the vision-only experiment; `-1` always uses SCC without vision matching; `0` matches SCC to vision; `1` matches front radar without SCC; `2` matches front radar plus low-speed SCC; and `3` uses SCC unconditionally after front-radar/vision matching fails. Matching modes use central vision at probability `0.40` or higher when matching fails. Modes `-1` and `3` use vision only when SCC is absent, and ignore the lateral coordinate of an SCC selected unconditionally. Legacy Mando radar variants with 32 or 64 slots are handled automatically.

`CarrotRadarMode` continuously tracks vehicles with the front and corner radars to detect cut-ins, then matches camera and radar information in a new way to select the vehicle ahead. On vehicles with neither corner-radar nor radar-track support, it behaves the same as the existing mode. It can change acceleration and braking, so enable it only on the same vehicle after completing validation. The value is latched when the next OnRoad session starts, so end the current drive and restart the vehicle or reboot the device after changing it. The previous `RadarMotionMode` value is migrated to the new name once on the first startup after updating.

`CarrotRadarCutInSensitivity` controls only Carrot Radar Mode CUT-IN detection: `0` disables it, `1` is insensitive, `3` is normal (default), and `5` is very sensitive; `2` and `4` are the intermediate levels. Levels `1` through `5` require `0.50`, `0.40`, `0.35`, `0.25`, and `0.20 s` of continuing measured motion evidence, while the physical future prediction remains fixed at 5.0 seconds. A front-radar track with at least 0.50 m of strongly one-way progress in its recent measured history may receive at most one 20 Hz radar-frame credit so timestamp quantization does not discard a completed dwell; small adjacent drift does not. It does not affect conventional radar mode or `EnableCornerRadar`. The value is read at the next OnRoad start, so restart the vehicle or reboot the device after changing it.

`HardwareC3xLite` must remain off on standard C3 and C3X hardware. Enable it only on a C3X Lite, then reboot the device. The setting skips the unavailable amplifier so startup is not delayed by I2C retries, uses the GPIO buzzer for alerts, disables `micd`, `soundd`, and `loggerd`, and turns off `RecordAudio`. Normal route logging is unavailable while this hardware mode is enabled.

<a id="display"></a>
## Display

Display contains 37 settings. Most on-road display settings are easy to reverse; external-HUD settings include hardware and performance choices.

| Group | Parameters | Purpose |
|---|---|---|
| Information | `ShowDebugUI`, `ShowTpms`, `ShowDateTime`, `ShowPathEnd`, `ShowDeviceState`, `ShowLaneInfo`, `ShowRadarInfo`, `ShowRouteInfo`, `ShowPlotMode` | Debug, tire, time, lane, radar, and route information |
| Path | `ShowPathMode`, `ShowPathColor`, `ShowPathColorCruiseOff`, `ShowPathModeLane`, `ShowPathColorLane` | Path shape and color by driving state |
| Brightness/on-road view | `ShowCustomBrightness`, `ShowModelView`, `ShowCameraWithCluster` | Brightness, camera/model composition, and the on-device camera while the external HUD is connected |
| External HUD | `ClusterHud`, `ClusterHudBrightness`, `ClusterHudOrientation`, and related `ClusterHud*` settings | Supported TURZX HUD layout, live brightness, screen rotation, camera, radar, encoder, and performance options |

An APN label remaining in the `ShowRouteInfo` description refers to route-input state. It is not an indication that CarrotMan or CarrotLink is supported.

`ShowCameraWithCluster=0` keeps the existing default: while the external HUD is connected, the on-device camera is hidden. Set it to `1` to show the on-device camera video.

`ClusterHudBrightness=0` follows camera exposure automatically; values `1` through `100` select fixed brightness. `ClusterHudOrientation` supports only `0` (0 degrees) and `2` (180 degrees); values `1` and `3` are ignored. The running TURZX process checks both stored settings every 100 ms. Brightness applies live; a managed H.264 orientation change automatically restarts the HUD and applies it through the capture-compatible stream setup.

`ClusterHudPanelLayout=0` places the driving view selected by `ClusterHudCameraViewMode` on the left and the information panel selected by the screen, debug, and navigation state on the right. `1` swaps them, placing information on the left and the driving view on the right. The running HUD applies the setting within about one second without a restart. Modes without two side regions, such as the full-screen graph and full-screen navigation, are unchanged. `ClusterHudDebug` forces always-on output and optional debug UI or navigation input; any resulting debug or navigation information panel follows the selected layout.

While the external HUD is connected over USB, `ShowCameraWithCluster=0` switches the on-device driving view on both regular C3/C3X hardware and mici to a black background and skips camera-video and model-path rendering. Setting it to `1` removes that connection-specific suppression and uses the normal on-device camera and on-road rendering path. A live change takes effect within about five seconds. Speed, speed limit, driver state, alerts, and the driving-state border remain visible with either value, and the option has no display effect after the external HUD disconnects. With value `0`, `camerad` and model input continue running; only duplicate rendering on the device display is reduced.

The final `ClusterHudScreenMode` layout is:

- `-1` uses the full width only in 3D camera views `0` and `1`, with no information panel or world shift. Left-side HUD items retain their margins, right-side gauges and TPMS align to the physical right edge, and the clock, world, and turn signals use the full-display center axis. In road-camera view `2`, it behaves exactly like mode `0`, including automatic navigation/report selection and `ClusterHudPanelLayout`.
- `0` is the default screen. It shows navigation while live navigation is received and automatically shows the driving report otherwise.
- `1` is the general live-debug screen, grouping delay, torque, steering, and lateral-plan state into four 2×2 cards. In 3D camera views, the world and driving HUD use the same 1124 px driving region as mode `0`, while the four cards fill the opposite 792 px information region. The acceleration, steering, fuel, and DEF gauges plus TPMS remain inside the driving region's right edge.
- `2` is the fixed system-debug screen. Its detail card shows the network address, output resolution/frame rate, camera state, memory capacity, and per-core CPU use. Its system-health card adds 2×2 circular gauges for CPU load, temperature, memory, and disk use, plus the device pitch/yaw target and numeric angles. In 3D camera views, the world and driving HUD use the same 1124 px driving region as mode `0`, while both cards use the opposite 792 px information region. It remains fixed regardless of connected, live, disconnected, or debug navigation state and never switches automatically to navigation, `NAVI DISCONNECTED`, the route overlay, or the driving report.
- `3` disables the driving scene and shows the `ShowPlotMode` graph at large size.
- `4` fits the 3D world and driving HUD into the same 1124 px driving region as mode `0` and uses the complete opposite 792 px information region for the graph. The acceleration, steering, fuel, and DEF gauges plus TPMS remain inside the driving region's right edge. Swapping the panel layout exchanges the graph and driving regions together.
- `5` always shows the driving report.

`ClusterHudScreenMode=5` shows a live driving report in the information panel. In default screen mode (`0`), the same report is shown automatically while no live navigation is being received, and the navigation panel returns when reception starts. The report background, cards, outlines, primary text, secondary text, and unavailable-value colors follow the active `ClusterHudTheme`, including Auto, Dark, and Light modes. Its large card summarizes driving time, distance, average and maximum speed, the automated-driving ratio, maximum acceleration/deceleration, and hard acceleration/braking/corner counts. The small card presents CPU load, temperature, memory, and disk use as a 2×2 set of circular gauges. Its lower target plots the stored device pitch (P) vertically and yaw (Y) horizontally relative to the calibrated center while retaining the numeric angles. The driving area retains the branch, network address, and frame-rate status; the core-usage text is omitted when it would overlap the report. In road-camera view, detected vehicles are enclosed by transparent rounded frames whose border retains the existing detection color; ungrouped radar detections use smaller transparent rounded markers in their source color. Vehicle frames use one lightweight outline, and frames that would be partially projected at the screen edge or stretched by a noisy radar heading are omitted.

The external HUD follows the device `LanguageSetting` and updates driving-report, driving-mode, and navigation status labels live in Korean (`ko`) or English (`en`). Other language values, including Chinese, fall back to English. With `IsMetric` enabled, vehicle/cruise/limit speeds, navigation, radar labels, and the driving report use `km/h`, `m`, and `km`; with it disabled they are converted to `mph`, `ft`, and `mi`. Acceleration and temperature remain `m/s²` and `°C`. Both settings are polled about once per second and do not require a HUD restart.

The enlarged driving-mode text beside the speed display has no background or border and remains above the final speed digit. Its text is green for Efficiency, orange for Safety, white for Normal, and red for High Speed. The gear badge also has a transparent center, retaining only its letter and outline. The upper-left steering/LFA, Wi-Fi, and clock row keeps extra outer and inter-icon spacing, and the speed-limit sign is offset slightly left.

`ClusterHudTheme=1` (Dark) renders the normal HUD's empty background in the same pure black used behind maps and while navigation is disconnected. Auto (`0`) uses the same dark palette from 18:00 to 06:00. The road, gauges, and regular information panels retain distinct dark shades for separation and readability.

On Hyundai/Kia CAN-FD hybrids, the external HUD's green `EV` indicator is enabled only when ECAN `0xFA` and `0x230` are both present with DLC32. It decodes the four-bit hybrid power-flow mode in `0x230` and shows `EV` for the observed motor/regen modes 1, 2, and 6. The normal HUD shows it between vehicle speed and cruise-set speed; full navigation intentionally omits it. The indicator remains hidden for other mode values and when the capability or sample is missing, invalid, or stale.

The normal external HUD also shows the current driving mode beside the traffic-state dot above vehicle speed. `MyDrivingMode` value `1` is a green Eco badge, `2` an orange Safe badge, `3` a white Normal badge, and `4` a red High badge. The badge is hidden for an unavailable, invalid, or stale `longitudinalPlan` and for values outside that range; full navigation omits it. The adjacent red or green dot is an independent model traffic-state indicator, not a driving-mode state.

The normal and road camera HUDs use the same fixed TPMS position below the acceleration, steering, fuel, and DEF gauges. The pressure font size is unchanged, with each value placed inside one of the enlarged tires of a simple toy-car diagram. The whole display is hidden only when all four pressure values are unavailable; an individually missing value shows `--`. Pressures below 31 psi are red, and no surrounding card or outline is drawn. When external navigation is active or its dashboard is connected, a green `NAV` appears below the Wi-Fi icon instead of the former lower-right `NAVI` label. The center clock, EV indicator, and fuel/DEF gauges remain unchanged.

### Carrot Vision AR and replay navigation events

Carrot Vision provides separate **Show AR** and **AR debug** controls outside the `carrot_settings.json` catalog. **Show AR** overlays driving guidance on the Vision video and requests the additional real-time data only while it is enabled. **AR debug** adds a troubleshooting panel with sign, anchor, and draw counts, the current blocking reason, and copyable history.

The Replay event timeline also identifies Carrot Navi connection and route-session changes, current and next maneuvers, lane guidance, road-safety alerts, average-speed zones, traffic signals, and intersection guidance. These entries are labels for reviewing transitions recorded in the replay.

<a id="system"></a>
## System

The 11 system settings cover recording, power, network, maps, sound, and software menus.

| Group | Parameters | Purpose |
|---|---|---|
| Recording and power | `RecordRoadCam`, `MaxTimeOffroadMin` | Road-camera storage and delayed shutdown |
| YouTube Live | `CarrotYouTubeLive`, `CarrotYouTubeQuality`, `CarrotYouTubeTimestamp` | Video streaming, quality, and timestamp |
| Network and map | `HotspotOnBoot`, `MapboxStyle` | Boot hotspot and map background style |
| Sound | `SoundLanguageSetting`, `SoundVolumeAdjust`, `SoundVolumeAdjustEngage` | Prompt language and volume |
| Software | `SoftwareMenu` | Carrot Web software-menu availability |

Check storage use for recording and network use, heat, and privacy before enabling live streaming.

## Safe adjustment order

1. Record the current value in Carrot Web.
2. Save both a baseline profile and a file backup.
3. Write down the parameter, previous value, and reason for the change.
4. Change only one item by one `unit` step.
5. If a restart is required, restart only while parked and off-road.
6. Repeat the same test in a permitted, controlled environment.
7. Restore the previous value or baseline profile immediately if the result is worse or unclear.

See the Wiki [Tuning introduction](https://github.com/ajouatom/openpilot/wiki/Guide-Tuning) for the recommended steering and longitudinal adjustment order.
