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

Select the body of a setting, outside its value control, to open its detail screen. The **Detailed guide** panel reads only the relevant section from the `docs/user/en` Markdown files installed on the device.

- It uses the on-device document without requiring GitHub or internet access.
- Select the **Detailed guide** heading to collapse or expand it.
- A documentation read failure does not disable viewing or changing the setting.
- Korean and English Carrot Web sessions use their matching guide language.
- Until a Chinese guide exists, Chinese titles and short descriptions remain Chinese while the detailed panel explicitly falls back to English.

Several parameters in the same subsection can share one explanation. If a short catalog description and the detailed guide differ, diagnose the current branch against this code-owned guide.

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

The current `carrot_settings.json` contains **166 parameters**. Every entry is assigned to one of these menus:

| Category | Count | Groups |
|---|---:|---|
| Driving control | 107 | Startup and auto, buttons and presets, steering, speed and deceleration, cruise and following gap |
| Vehicle and hardware | 15 | Hyundai/Kia, CAN FD/HDA, radar, driver monitoring, vehicle assistance, device hardware |
| Display | 33 | Information, path, brightness/on-road view, external HUD |
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
| Lane change and automatic turn | `LaneChangeNeedTorque`, `LaneChangeDelay`, `LaneChangeBsd`, `LaneLineCheck`, `AutoTurnControl`, `AutoTurnControlSpeedTurn`, `AutoTurnControlTurnEnd`, `AutoTurnMapChange` | Lane-change conditions and ATC behavior |
| Lane mode | `LatMpcPathCost`, `LatMpcMotionCost`, `LatMpcAccelCost`, `LatMpcJerkCost`, `LatMpcSteeringRateCost`, `LatMpcInputOffset`, `UseLaneLineSpeed`, `UseLaneLineCurveSpeed`, `AdjustLaneOffset` | Lane-mode MPC weights and lane-line conditions |
| Advanced torque | `LateralTorqueCustom`, `LateralTorqueAccelFactor`, `LateralTorqueFriction`, `LateralTorqueKpV`, `LateralTorqueKiV`, `LateralTorqueKf`, `LateralTorqueKd` | Custom torque-control gains |
| Steering limits | `CustomSteerMax`, `CustomSteerDeltaUp`, `CustomSteerDeltaDown`, `CustomSteerDeltaUpLC`, `CustomSteerDeltaDownLC` | Maximum torque and torque-rate limits |

A larger `SteerActuatorDelay` compensates by commanding earlier. A larger `LatSmoothSec` is smoother but may respond more slowly. Changing both together makes diagnosis difficult.

`LateralTorqueCustom` and `CustomSteer*` are advanced settings that can affect the vehicle tune and safety limits. Do not alter them without a vehicle-specific validated baseline and a recovery path.

### Speed and deceleration — 18 settings

| Section | Parameters | Purpose |
|---|---|---|
| [Speed cameras](speed-deceleration.md#speed-camera) | `AutoNaviSpeedCtrlMode`, `AutoNaviSpeedCtrlEnd`, `AutoNaviSpeedDecelRate`, `AutoNaviSpeedSafetyFactor`, `AutoNaviCountDownMode` | Event types, deceleration start, and target speed |
| [Road speed limit](speed-deceleration.md#road-speed-limit) | `AutoRoadSpeedLimitOffset`, `AutoRoadSpeedAdjust`, `AutoSpeedUptoRoadSpeedLimit` | Desired-speed adjustment from the road limit |
| [Speed bumps](speed-deceleration.md#speed-bump) | `AutoNaviSpeedBumpTime`, `AutoNaviSpeedBumpSpeed` | Completion time and crossing speed |
| [Curves and turns](speed-deceleration.md#curve-turn) | `AutoCurveSpeedFactor`, `AutoCurveSpeedLowerLimit`, `TurnSpeedControlMode`, `MapTurnSpeedFactor`, `ModelTurnSpeedFactor`, `ApplyModelSpeed` | Curve speed from model curvature and route data |
| [Traffic lights](speed-deceleration.md#traffic-light) | `TrafficLightDetectMode`, `TrafficStopDistanceAdjust` | Stop/go detection and stop-position adjustment |

`AutoNaviSpeedCtrlMode` is `0` off, `1` fixed speed cameras, `2` cameras plus speed bumps, or `3` those events plus mobile-camera events.

A lower `AutoNaviSpeedDecelRate` begins slowing farther away. `AutoNaviSpeedSafetyFactor` applies a percentage of the event limit as the target. Before tuning either value, confirm that the event type, limit, and remaining distance are being received correctly.

`TrafficLightDetectMode` is `0` off, `1` stop detection, or `2` stop and go detection. This is model-based assistance; the driver must always verify the signal.

### Cruise and following gap — 30 settings

| Section | Parameters | Purpose |
|---|---|---|
| [Driving mode](cruise-gap.md#driving-mode) | `MyDrivingMode`, `MyDrivingModeAuto` | Eco, safe, normal, high-speed modes and automatic selection |
| [Speed-based acceleration](cruise-gap.md#acceleration-table) | `CruiseMaxVals0` through `CruiseMaxVals6` | Maximum acceleration tendency by speed band |
| [Stopping and restarting](cruise-gap.md#stop-resume) | `StopDistanceCarrot`, `StoppingAccel`, `VEgoStopping`, `AChangeCostStarting` | Stop position, stop entry, and restart behavior |
| [Longitudinal tuning](cruise-gap.md#longitudinal-tuning) | `LongTuningKpV`, `LongTuningKiV`, `LongTuningKf`, `LongActuatorDelay` | Control gains and vehicle-response delay |
| [Following gap](cruise-gap.md#following-gap) | `TFollowGap1` through `TFollowGap4`, `DynamicTFollow`, `DynamicTFollowLC`, `EnableSpeedTF`, `TFollowDecelBoost` | Gap times, dynamic gap, and deceleration margin |
| [Lead response](cruise-gap.md#lead-response) | `JLeadFactor3`, `RadarReactionFactor` | Response to lead-vehicle changes |
| [Carrot cruise](cruise-gap.md#carrot-cruise) | `CruiseEcoControl`, `CarrotCruiseDecel`, `CarrotCruiseAtcDecel` | Economy control and cruise deceleration limits |

`MyDrivingMode` is `1` eco, `2` safe, `3` normal, or `4` high speed. High-speed mode ignores traffic-light control and increases acceleration tendency, so read its behavior before selecting it.

`TFollowGap1` through `TFollowGap4` are stored in hundredths of a second. Lower values reduce the time gap. Establish a stable fixed-gap baseline before enabling `DynamicTFollow` features.

`LongTuning*`, `LongActuatorDelay`, and `StoppingAccel` are advanced settings that directly affect vehicles using openpilot longitudinal control. Some have no effect when stock ACC remains responsible for acceleration and braking.

<a id="vehicle-hardware"></a>
## Vehicle and hardware

These 15 settings describe the car, harness, and device hardware configuration. Do not enable them merely as a display experiment.

| Group | Parameters | Purpose |
|---|---|---|
| Hyundai/Kia | `HyundaiCameraSCC`, `IsLdwsCar`, `HapticFeedbackWhenSpeedCamera` | SCC connection, LDWS behavior, and speed-event haptics |
| CAN FD/HDA | `CanfdHDA2`, `CanfdDebug`, `HDPuse` | HDA2 selection, CAN FD diagnostics, and HDP |
| Radar | `EnableRadarTracks`, `EnableCornerRadar`, `RadarLeadModelMode` | SCC radar, raw tracks, corner radar, and lead selection |
| Driver monitoring | `DisableDM`, `MuteDoor`, `MuteSeatbelt` | Driver monitoring and selected vehicle alerts |
| Vehicle assistance | `MaxAngleFrames`, `SpeedFromPCM` | Steering-angle frames and stock-SCC speed control |
| Device hardware | `HardwareC3xLite` | Speakerless C3X Lite audio and process configuration |

> [!CAUTION]
> Incorrect `HyundaiCameraSCC`, `CanfdHDA2`, `EnableRadarTracks`, `RadarLeadModelMode`, or `SpeedFromPCM` values can change vehicle identification, SCC, radar, or longitudinal behavior. Confirm the vehicle, model year, HDA generation, harness location, and whether stock ACC is retained.

See [Radar tracks and corner radar](radar.md) before changing radar modes.

`HardwareC3xLite` must remain off on standard C3 and C3X hardware. Enable it only on a C3X Lite, then reboot the device. The setting skips the unavailable amplifier so startup is not delayed by I2C retries, uses the GPIO buzzer for alerts, disables `micd`, `soundd`, and `loggerd`, and turns off `RecordAudio`. Normal route logging is unavailable while this hardware mode is enabled.

<a id="display"></a>
## Display

Display contains 33 settings. Most on-road display settings are easy to reverse; external-HUD settings include hardware and performance choices.

| Group | Parameters | Purpose |
|---|---|---|
| Information | `ShowDebugUI`, `ShowTpms`, `ShowDateTime`, `ShowPathEnd`, `ShowDeviceState`, `ShowLaneInfo`, `ShowRadarInfo`, `ShowRouteInfo`, `ShowPlotMode` | Debug, tire, time, lane, radar, and route information |
| Path | `ShowPathMode`, `ShowPathColor`, `ShowPathColorCruiseOff`, `ShowPathModeLane`, `ShowPathColorLane` | Path shape and color by driving state |
| Brightness/on-road view | `ShowCustomBrightness`, `ShowModelView` | Brightness and camera/model composition |
| External HUD | `ClusterHud` and related `ClusterHud*` settings | Supported TURZX HUD layout, camera, radar, encoder, and performance options |

An APN label remaining in the `ShowRouteInfo` description refers to route-input state. It is not an indication that CarrotMan or CarrotLink is supported.

On Hyundai/Kia CAN-FD hybrids, the external HUD's green `EV` indicator is enabled only when ECAN `0xFA` and `0x230` are both present with DLC32. It decodes the four-bit hybrid power-flow mode in `0x230` and shows `EV` for the observed motor/regen modes 1, 2, and 6. The normal HUD shows it between vehicle speed and cruise-set speed; full navigation intentionally omits it. The indicator remains hidden for other mode values and when the capability or sample is missing, invalid, or stale.

The normal external HUD also shows the current driving mode beside the traffic-state dot above vehicle speed. `MyDrivingMode` value `1` is a green Eco badge, `2` an orange Safe badge, `3` a white Normal badge, and `4` a red High badge. The badge is hidden for an unavailable, invalid, or stale `longitudinalPlan` and for values outside that range; full navigation omits it. The adjacent red or green dot is an independent model traffic-state indicator, not a driving-mode state.

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
