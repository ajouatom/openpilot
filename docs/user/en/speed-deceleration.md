# Speed and Deceleration Settings

[한국어](../ko/speed-deceleration.md)

> [!NOTE]
> This is the canonical English user guide maintained with the `carrot-wip` code. When user-visible behavior changes, update this document together with the related code and tests.

This page explains all **18 speed and deceleration settings** in the current code: event targets, deceleration distance, road-limit adjustment, speed bumps, curve/route/model speed, and traffic-light stop adjustment.

Change them in **Carrot Web → Settings → Driving control → Speed and deceleration**.

> [!CAUTION]
> These settings can affect desired speed and the deceleration plan. Route, speed-limit, distance, and model inputs can be missing, late, or incorrect. The driver must always monitor the road and intervene immediately.

## Five sections

1. [Speed cameras](#speed-camera)
2. [Road speed limit](#road-speed-limit)
3. [Speed bumps](#speed-bump)
4. [Curves and turns](#curve-turn)
5. [Traffic-light detection](#traffic-light)

## The lowest candidate becomes the final desired speed

The code collects several candidates and selects their minimum as `desired_speed`:

| Candidate | Representative setting/input |
|---|---|
| Camera, average-speed zone, speed bump | Safety-event input, `AutoNaviSpeed*` |
| Road limit plus offset | `AutoRoadSpeedLimitOffset` |
| Vision curve | `AutoCurveSpeedFactor`, `TurnSpeedControlMode` |
| Route turn | `MapTurnSpeedFactor`, `TurnSpeedControlMode` |
| Future model speed | `ModelTurnSpeedFactor` |
| Separate automatic turn control | `AutoTurnControl*` |

If raising one value produces no change, another source may already be lower. Check the displayed source, event type, target limit, and remaining distance together.

> [!NOTE]
> CarrotMan and CarrotLink are not currently supported. Features requiring external route, turn-by-turn, or road input may be unavailable in a given setup; describing their internal calculations does not imply support for those former connection methods.

### Catalog defaults and initial Params can differ

| Parameter | Catalog default | Initial Params value |
|---|---:|---:|
| `AutoCurveSpeedFactor` | 100 | 120 |
| `AutoNaviSpeedCtrlEnd` | 6 | 7 |
| `AutoNaviSpeedDecelRate` | 200 | 120 |
| `MapTurnSpeedFactor` | 100 | 90 |
| `AutoRoadSpeedAdjust` | 0 | 50 |

Record the value currently shown on the device before changing anything.

<a id="speed-camera"></a>
## 1. Speed cameras

### `AutoNaviSpeedCtrlMode`

| Value | Events used for deceleration |
|---:|---|
| `0` | Disabled |
| `1` | Fixed cameras and average-speed zones |
| `2` | Mode 1 plus speed bumps |
| `3` | Mode 2 plus mobile-camera events |

The event type, limit, and distance must all be valid. An average-speed zone retains its target until the end distance; mobile-camera events are used only in mode `3`.

### `AutoNaviSpeedSafetyFactor`

    camera target = received limit × safety factor / 100

| Limit | Factor | Target |
|---:|---:|---:|
| 60 km/h | 100% | 60 km/h |
| 60 km/h | 105% | 63 km/h |
| 100 km/h | 95% | 95 km/h |

This does not guarantee speedometer error or any legal tolerance. A wrong input limit produces a wrong target.

### `AutoNaviSpeedCtrlEnd`

This is not time spent decelerating after an event. It creates a distance margin at the target speed:

    completion margin = target speed (m/s) × completion time (s)

At a 60 km/h target (about 16.7 m/s), 6 seconds is about 100 m and 10 seconds is about 167 m before the camera. Raising the value aims to finish slowing earlier but can hold the lower speed longer.

### `AutoNaviSpeedDecelRate`

The stored value is multiplied by `0.01 m/s²`:

| Stored value | Rate used | Perceived direction |
|---:|---:|---|
| 80 | 0.80 m/s² | Start earlier and more gently |
| 120 | 1.20 m/s² | Middle |
| 200 | 2.00 m/s² | Can start closer and slow more strongly |

The important direction is: **a lower value starts deceleration earlier**. The speed ceiling follows:

    allowed_speed² = target_speed² + 2 × rate × (remaining_distance - completion_margin)

If slowing begins too late, lower this value one step. If it begins too early, raise it. Do not change `CtrlEnd` at the same time.

### `AutoNaviCountDownMode`

| Value | Countdown display |
|---:|---|
| `0` | Off |
| `1` | Turn points and speed events, excluding bumps |
| `2` | Turn points, speed events, and bumps |

The countdown estimates seconds from distance and current speed. It does not alter the deceleration calculation.

Recommended tuning order: validate event data, set the target with `SafetyFactor`, set the completion position with `CtrlEnd`, and finally tune the approach curve with `DecelRate`.

<a id="road-speed-limit"></a>
## 2. Road speed limit

### `AutoRoadSpeedLimitOffset`

With an active, valid road limit of at least 30 km/h:

    road target = road limit + offset

| Value | Meaning as a final-speed candidate |
|---:|---|
| `-1` | Do not use the limit-plus-offset candidate |
| `0` | Use the limit as received |
| `5` | Use 5 km/h above the limit |

`-1` is not a global off switch. `CruiseSpeed1=0` and `AutoRoadSpeedAdjust=-1` use a negative offset as the condition for a separate safety-factor calculation.

### `AutoSpeedUptoRoadSpeedLimit`

While following a lead, cruise set speed can rise in 5 km/h steps up to:

    automatic ceiling = road limit × setting / 100

All of these must be true: automatic increase is not paused; a lead is within about 60 m; lead speed plus 5 km/h exceeds set speed; and set speed is below the ceiling. `0` disables this feature. SET/- or braking can pause it; RES/+, accelerator input, or a stopped state can release the pause.

### `AutoRoadSpeedAdjust`

> [!IMPORTANT]
> The current code returns before this setting is evaluated when the ceiling calculated by `AutoSpeedUptoRoadSpeedLimit` is below 1 km/h. Therefore `AutoSpeedUptoRoadSpeedLimit=0` effectively disables `AutoRoadSpeedAdjust` too.

| Value | Behavior when the road limit changes |
|---:|---|
| `-1` | Immediately synchronize to the new road target on either increase or decrease |
| `0` | Keep the existing set speed |
| `1–99` | On a decrease only, move to a weighted value between current set speed and new limit |
| `100` | On a decrease, move directly to the new limit |

At 50%, a current set speed of 100 and new limit of 60 produces 80 km/h. This 1–100 branch uses the raw road limit, not the offset or safety factor.

For `-1`, a non-negative offset selects limit + offset; a negative offset selects limit × camera safety factor. Immediate synchronization is especially sensitive to incorrect road-limit input.

<a id="speed-bump"></a>
## 3. Speed bumps

Speed-bump control requires `AutoNaviSpeedCtrlMode >= 2`, a bump event and distance, and a road category that the code does not treat as highway.

### `AutoNaviSpeedBumpSpeed`

This is the crossing target in km/h. Raise it to cross faster and lower it to cross more slowly. Vehicle height, suspension, and bump shape matter; an unrecognized bump is unaffected.

### `AutoNaviSpeedBumpTime`

    bump completion margin = bump target (m/s) × setting time (s)

At 36 km/h (10 m/s), 1 second aims to reach target about 10 m before the bump; 3 seconds aims for about 30 m. A larger value finishes earlier. The approach curve still uses `AutoNaviSpeedDecelRate`.

If there is no slowing, check mode 2+, event type 22, remaining distance, and road category. For late slowing, lower `DecelRate` or raise `BumpTime`; for an incorrect crossing speed, adjust only `BumpSpeed`.

<a id="curve-turn"></a>
## 4. Curves and turns

There are four distinct sources:

- **Vision curve speed** from predicted yaw rate and speed
- **Route-turn speed** from route/turn input
- **Future model speed** at a selected future time
- **Applied model driving speed** from the model's overall desired velocity

### `AutoCurveSpeedFactor`

The code scales model yaw rate and calculates a curve speed around a 1.9 m/s² lateral-acceleration target. A larger factor treats the same curve as sharper and produces a lower target.

The relationship is approximately inverse-square-root: changing 100% to 120% produces about `1 / √1.2`, or 91% of the previous calculated speed. Raise it one step if curves are too fast; lower it if they are too slow.

### `AutoCurveSpeedLowerLimit`

This floor applies to vision-curve, route-turn, and future-model candidates. Raising it prevents those sources from selecting a lower speed, which can leave insufficient slowing for a sharp curve. It is not an automatically safe minimum.

### `TurnSpeedControlMode`

| Value | Vision curve | Route turn |
|---:|---:|---:|
| `0` | Off | Off |
| `1` | On | Off |
| `2` | On | Near the turn point |
| `3` | Off | Always included as a candidate |

In mode 2, route-turn speed enters only when turn distance is roughly between -500 m and +500 m. Mode 3 can cause unexpected slowing when route data is inaccurate.

> [!IMPORTANT]
> Mode `0` does not disable every model-derived speed function. Check `ModelTurnSpeedFactor` and `ApplyModelSpeed` separately.

### `MapTurnSpeedFactor`

    route-turn candidate = route speed × factor / 100

80% lowers the received value; 100% keeps it; 120% raises it. `AutoCurveSpeedLowerLimit` is then applied as the floor. Valid supported route-speed input is required.

### `ModelTurnSpeedFactor`

The stored value is multiplied by `0.1 seconds` to choose a future point in the model prediction:

| Value | Future point |
|---:|---:|
| `0` | Disabled; candidate set to 200 km/h |
| `10` | About 1.0 s ahead |
| `30` | About 3.0 s ahead |
| `50` | About 5.0 s ahead |

The chosen speed is multiplied by 1.2 and smoothed. A larger setting does not necessarily mean slower; it depends on the predicted speed at that future point.

### `ApplyModelSpeed`

This is not curve-only. It applies the driving model's `desiredVelocity` to cruise set speed:

    applied model speed = desiredVelocity × abs(setting) / 100
    effective ceiling = min(applied model speed, road limit × 110%)

| Value | Current behavior |
|---:|---|
| `0` | Disabled |
| Positive | Raise set speed to the ceiling only when currently below it |
| Negative | Directly overwrite set speed with the calculated ceiling |

> [!WARNING]
> A negative value is a strong continuous overwrite, not a “deceleration only” switch. Without a valid road limit, the ceiling can become zero. Leave it at `0` unless you fully understand the input path and behavior.

For isolated tuning, start with mode 1, `ModelTurnSpeedFactor=0`, and `ApplyModelSpeed=0`; adjust the curve factor, then the floor, then add route and future-model sources one at a time.

<a id="traffic-light"></a>
## 5. Traffic-light detection

This uses the model's predicted path end and future speed changes, together with incoming signal events. It is not a simple direct camera color reader.

### `TrafficLightDetectMode`

| Value | Intended behavior | Current-code note |
|---:|---|---|
| `0` | Disabled | Detection state forced off |
| `1` | Detect stops only | Blocks automatic departure from a complete stop |
| `2` | Detect stop and go | May transition to departure on a green decision |

> [!IMPORTANT]
> While decelerating, mode `1` still has a branch that can cancel the stop plan after a green decision. It cannot be interpreted as “never use any departure judgment.”

Detection is disabled or limited in high-speed driving mode (`MyDrivingMode=4`), above roughly 20 degrees of steering angle, when the model lacks stop-path confidence, during selected strong-deceleration conditions, or when a lead is ahead of the stop target.

### `TrafficStopDistanceAdjust`

The stored value is divided by 100 and added to the stop-obstacle position in meters:

| Stored value | Adjustment | Direction |
|---:|---:|---|
| `-200` | -2.0 m | Stop earlier/farther from the line |
| `-150` | -1.5 m | Initial Params value |
| `0` | 0 m | Model position |
| `100` | +1.0 m | Stop later/closer to the line |

Near a complete stop, the current code uses a fixed -2.0 m instead of the user value, so the final position may not exactly match the setting. Adjust by 100 (1 m) at a time under comparable conditions. If false signal detection is the problem, diagnose the mode/model decision instead of the distance offset.

## Quick troubleshooting

| Symptom | Check first |
|---|---|
| Slowing starts too late for a camera | Event distance → lower `DecelRate` → raise `CtrlEnd` |
| Camera target is unexpected | Received limit, `SafetyFactor`, other lower candidates |
| No response to a changed road limit | Valid limit, auto-increase setting, adjust dependency |
| No speed-bump slowing | Mode 2+, event type, road category |
| Curve slowing on a straight road | Which of vision, route, or model selected the minimum |
| Slowing with turn mode 0 | Future-model speed, applied-model speed, separate ATC |
| Unwanted traffic-light stop/go | Detection mode, driving mode, model decision |

Automatic CAN diagnostic logs are generated only when currently received vehicle or radar state from the present onroad session reports an actual error. CAN timeouts from the previous drive's shutdown are not used, and capture occurs five seconds after detection to include the immediate aftermath. When investigating a speed or deceleration issue, upload the affected drive from Carrot Web even if no automatic diagnostic log was generated.

## Code references

- Event/limit/bump/turn candidate selection: `openpilot/selfdrive/carrot/carrot_serv.py`
- Vision-curve speed: `openpilot/selfdrive/carrot/carrot_man.py`
- Future-model speed: `openpilot/selfdrive/controls/lib/desire_helper.py`
- Automatic road/model set speed: `openpilot/selfdrive/car/cruise.py`
- Traffic-stop state and adjustment: `openpilot/selfdrive/carrot/carrot_functions.py`
- Traffic-stop MPC: `openpilot/selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py`
- Setting ranges/defaults: `openpilot/selfdrive/carrot_settings.json`, `openpilot/common/params_keys.h`

[Back to Understanding Settings](settings.md)
