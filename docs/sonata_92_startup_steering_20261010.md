# Sonata 2024 startup steering step — 2026-10-10 investigation

## Conclusion

The supplied `00000092--f356974e0e--0` recording establishes a startup control
defect, not merely an aggressive ordinary engagement ramp. Before model and
live vehicle parameters arrive, AlwaysLateral activates the torque controller
using default messages. The unreceived steer ratio becomes 0.1 and stiffness
factor becomes 0.1; desired curvature remains zero. The resulting torque
request saturates. When the delayed camera-SCC LFA transmission path becomes
available, its first host command is already +375 because the internal rate
limiter has been advancing without actually sending LFA commands.

Panda bus-128 TX echoes confirm the delivered transition from zero/no request
to +375/request enabled. This is not a plot of an unsent upstream request.
The recorded wheel moves from -2.1 to +20.2 degrees, followed by -28.9 degrees.
The subsequent driver/EPS/vehicle contributions cannot be separated completely
from this recording, but the invalid calculation and first-command step are
directly established.

The initial investigation changed no production code. The user subsequently
requested a correction; implementation and validation are recorded below.

## Source and reference clock

- Full NAS `rlog.zst` and `qcamera.ts`, matching supplied vehicle/segment.
- Clean incident commit `eb5bbf2720286f17fc55d6eab8a2f7aacecf1f3d`.
- Full cereal schema: 5,992 carState and carOutput, 5,969 carControl and
  controlsState, 984 modelV2 and liveParameters publications.
- Times below are relative to first carState, monotonic 50.634403780 seconds.
  Video is associated through qRoadEncodeIdx.timestampSof and segmentId.
- CP: HYUNDAI_SONATA_2024, torque control, flags 8204, wheelbase 2.84 m,
  nominal steer ratio 12.81. This is not angle-control handover mode 3.
- Startup Params: AlwaysLateral=1, CustomSR=0, SteerRatioRate=88,
  CustomSteerMax=409, CustomSteerDeltaUp=3, CustomSteerDeltaDown=0,
  UseLaneLineSpeed=0, NNFF=NNFFLite=0, LateralTorqueCustom=0.
- Internal GPU backend (`JoiningModel`, usbgpu=false); no basis for blaming
  the eGPU model choice. Model loading completes at 8.174 s, with the first
  modelV2 publication at 10.609 s.

## Observed sequence

| Time | Observation |
|---|---|
| 0.221 s | First carControl already has latActive=true, enabled=false. Vehicle is in Drive, moving at 30.3 km/h. No modelV2/liveParameters yet. Requested torque is -1.0, target curvature zero. |
| 6.091–6.092 s | selfdriveInitializing disappears; selfdrived logs timeout=true at its six-second initialization timeout. modelV2, liveParameters and other required services remain absent. commIssue exists. |
| 6.111 s | carOutput begins reporting internally limited torque 3, 6, 9, 12…; no host LFA 0x12A frames are sent yet. |
| 6.156 s | Panda switches from diagnostic to hyundaiCanfd safety mode. Forwarded stock 0x12A remains torque=0, STEER_REQ=0. |
| 7.342 s | First host sendcan 0x12A requests +375 with STEER_REQ=1, at about 57 km/h; wheel is -2.1 degrees. |
| 7.363 → 7.375 s | Consecutive logged Panda TX echoes change 0/request-off → +375/request-on. These are host log batch timestamps, separated by 12.2 ms, not sub-millisecond wire timing measurements. |
| 7.400 / 7.425 s | Host / TX-echo torque reaches +393. Values are CAN command units, not Nm or physical assistance percentages. |
| 7.637 s | Actual wheel reaches +20.2 degrees. |
| 8.016 s | Actual wheel reaches -28.9 degrees with substantial column torque. Do not identify every part of this reversal as autonomous or deliberate driver input. |
| 10.609 / 10.650 s | First modelV2 / liveParameters arrive. Live ratio is 14.3573; configured scaling gives approximately 12.6344. |
| 10.666 s | Control calculation starts reflecting received live parameters; normalized request drops to about +0.206. |
| 11.666 / 11.982 s | liveParameters Event first becomes valid / commIssue clears from onroadEvents. |

The main selfdrive state is disabled throughout the segment while lateral
control remains active through AlwaysLateral. The early commIssue therefore
does not block this lateral path.

## Causal code paths

1. `controlsd.py:state_control` updates VehicleModel from liveParameters even
   before the service has been received. The steer-ratio helper clamps the
   initial zero to 0.1; the stiffness clamp likewise produces 0.1. At 6.001 s,
   wheel -2.2 degrees and speed 53.05 km/h produce calculated curvature
   0.0592312 /m and calculated lateral acceleration 12.8634 m/s². Re-running
   VehicleModel with those fallback values reproduces the log. These are
   invalid model-derived estimates, not actual measured vehicle acceleration.
   Using nominal CP ratio/stiffness on the same input gives 0.00093546 /m.
2. `lateral_control_allowed` permits Drive + AlwaysLateral without requiring
   first valid model/live parameter inputs or honoring the initialization/
   communication events. Missing model action supplies zero desired curvature.
   1,027 of 1,039 control samples before first model publication have absolute
   normalized torque output at least 0.99999. PID integral is zero at the
   sampled onset; this is not accumulated PID integral windup.
3. `selfdrived.py:data_sample` in the incident revision exits initialization
   after six seconds even when services are unavailable. `card.py` uses absence
   of selfdriveInitializing to start applying carControl. It checks that
   carControl is alive, without independently rejecting missing model data;
   it passes model_v2=None to the interface.
4. Hyundai CarController advances `apply_torque_last` on every application.
   Camera-SCC `create_steering_messages_camera_scc` emits torque LFA only when
   `CS.lfa` exists. CarState deliberately waits until ControlsReady count 121
   and a real accepted camera frame before exposing that template. The LFA
   readiness log reports count 122 near 7.348 s (logging is asynchronous).
   The first 125 internal outputs are the 3-unit ramp to 375, but the car sees
   the first host LFA only after that ramp has largely completed. Internal
   smoothing therefore does not smooth the first transmitted command.
5. Incident safety source calls `steer_torque_cmd_checks`, but its rejection
   statement is commented out (`//tx = false;`). The resulting LFA can enter
   the existing RX-paced forwarding queue. Decoded TX echoes prove the step
   was delivered; zero safetyTxBlocked does not prove it was safe. This source
   observation is not a separate binary-identity validation of flashed firmware.

## Current branch and correction scope

At inspected HEAD `5479d1279a`, initialization timeout has changed from six to
ten seconds in `26c6fa54b3`. The relevant controlsd, steer-ratio helper, torque
controller, camera-template discovery and first-LFA emission paths still match
the incident behavior. Hyundai CarController's intervening difference is an
unrelated cluster-popup platform addition. A longer timeout can shift the
timing of this particular incident, but is not an input-readiness guarantee or
a correction for the accumulated first-TX command.

A correction should cover all of these boundaries:

- Require received, fresh, valid lateral inputs before allowing AlwaysLateral
  as well as ordinary lateral control; initialization timeout must not grant
  steering permission by itself.
- Preserve sane nominal/validated VehicleModel parameters until valid live
  values exist. Do not replace a real ratio with the 0.1 numerical floor.
- Keep rate-limit history aligned with commands actually eligible for
  transmission. Initial or resumed camera-template availability must not expose
  a command ramp that ran only inside the host.
- Review the independent Panda rejection path for this torque-control case;
  changing that path requires its own regression/firmware validation.

Reducing CustomSteerMax or merely extending startup delay could lessen or hide
the symptom without removing the causes. Desktop recorded-input checks can
verify readiness and first-command behavior; they cannot establish steering
feel or closed-loop safety in the vehicle.

Private evidence and reproduction scripts are indexed under
`.analysis/archive/2026-10-10/sonata-92-startup-steering/README.md`.

## Authorized correction

- A shared lateral-readiness predicate now gates both controlsd (ordinary
  engagement and AlwaysLateral) and card's final application boundary. Required
  services must have been received and pass existing validity, liveness and
  frequency checks: modelV2, liveParameters, livePose, selfdriveState and
  onroadEvents. selfdriveInitializing still blocks. The gate additionally checks
  live geometry, pose health, car CAN/fault state and finite steering inputs.
  A selected lane-line plan must be ready, have a valid solution and finite
  17-element trajectories. The numerical fallback never permits steering.
- Before live parameters become valid, inactive calculations use nominal CP
  geometry, with existing manual ratio/scaling settings still applied. Zero
  default messages no longer replace nominal geometry with a 0.1 ratio.
- Inactivity and each lateral activation synchronize desired curvature to the
  current measured curvature. Torque PID integral and NN desired-input histories
  reset, preventing state accumulated before readiness from carrying into entry.
- Hyundai CAN-FD CAMERA_SCC locally treats lateral control as inactive until
  its relevant steering TX template exists (LFA for torque, LFA_ALT for angle).
  This resets torque/handover state and follows measured wheel angle while
  waiting. Both initial discovery and subsequent template loss/reappearance are
  tested. The source carControl message and longitudinal fields are preserved.
- card converts locally neutralized messages back to readers before passing
  them to car controllers; this also preserves the existing impact-reboot guard.

No steering gain, normal angle/torque rate limit, actuator maximum, model,
longitudinal policy, CAN forwarding queue, or Panda firmware is changed. The
commented Panda rejection noted above remains a separate defense-layer issue;
this correction blocks the demonstrated host-side causes independently. It
does not claim to restore independent firmware protection against every host
fault. No new setting or vehicle-specific exception is introduced.

### Validation

- 191 focused desktop tests pass: common readiness and complete state_control
  execution with the production torque controller, live ratio settings, Hyundai
  steering modes/handover, template discovery and CAN counter handling.
- A 15-second synthetic input delay exceeds both historical initialization
  timeouts and still leaves lateral control inactive with zero torque. This is
  tested with normal engagement requested and with AlwaysLateral alone.
- Missing, invalid, stale and low-frequency inputs; timeout without a model;
  malformed live geometry; selected lane-plan errors; queued active commands;
  recovery after input loss; and torque/angle TX template reappearance are
  covered. A saturated request held through 200 unavailable-template ticks first
  transmits torque 3 when the template becomes available, rather than 375.
- The full 5,969-frame recorded-input desktop replay runs the production
  state_control, torque controller and Hyundai steering packing. All 1,145
  control frames before first valid liveParameters remain inactive with zero
  torque. The first available LFA at 7.344 s is zero/request-off. The first
  permitted steering sample is 11.674 s and starts at CAN torque 3/request-on.
  There are zero nonzero requests before first model publication.
- Replay uses logged publication times as receipt approximations and optimistic
  frequency status, preserving actual Event validity. Runtime frequency settling
  can delay readiness further. Longitudinal processing and unrelated cluster/
  button paths are stubbed; this is a lateral input/output check, not native IPC,
  exact original scheduler timing, vehicle response, or steering-feel validation.
- Windows tests substitute Params storage and hardware identification only where
  needed; production lateral calculations and CAN packing execute unchanged.
  Korean/English setting guides and the localized catalog/Wiki describe readiness.

Fix reproduction evidence is local under
`.analysis/archive/2026-10-10/sonata-startup-fix/README.md`.
