#!/usr/bin/env python3
import os
from cereal import car
from math import fabs, exp
from panda import Panda

from openpilot.common.basedir import BASEDIR
from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params
from openpilot.selfdrive.car import create_button_events, get_safety_config
from openpilot.selfdrive.car.gm.radar_interface import RADAR_HEADER_MSG
from openpilot.selfdrive.car.gm.values import CAR, CruiseButtons, CarControllerParams, EV_CAR, CAMERA_ACC_CAR, CanBus, GMFlags, CC_ONLY_CAR, SDGM_CAR
from openpilot.selfdrive.car.interfaces import CarInterfaceBase, TorqueFromLateralAccelCallbackType, FRICTION_THRESHOLD, LatControlInputs, NanoFFModel
from openpilot.selfdrive.controls.lib.drive_helpers import get_friction

ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName
GearShifter = car.CarState.GearShifter
TransmissionType = car.CarParams.TransmissionType
NetworkLocation = car.CarParams.NetworkLocation
BUTTONS_DICT = {CruiseButtons.RES_ACCEL: ButtonType.accelCruise, CruiseButtons.DECEL_SET: ButtonType.decelCruise,
                CruiseButtons.MAIN: ButtonType.altButton3, CruiseButtons.CANCEL: ButtonType.cancel,
                CruiseButtons.GAP_DIST: ButtonType.gapAdjustCruise}

PEDAL_MSG = 0x201
CAM_MSG = 0x320  # AEBCmd
                 # TODO: Is this always linked to camera presence?
ACCELERATOR_POS_MSG = 0xbe

NON_LINEAR_TORQUE_PARAMS = {
  CAR.CHEVROLET_BOLT_EUV: [2.6531724862969748, 1.0, 0.1919764879840985, 0.009054123646805178],
  CAR.CHEVROLET_BOLT_CC: [2.6531724862969748, 1.0, 0.1919764879840985, 0.009054123646805178],
  CAR.GMC_ACADIA: [4.78003305, 1.0, 0.3122, 0.05591772],
  CAR.CHEVROLET_SILVERADO: [3.29974374, 1.0, 0.25571356, 0.0465122]
}

NEURAL_PARAMS_PATH = os.path.join(BASEDIR, 'selfdrive/car/torque_data/neural_ff_weights.json')


class CarInterface(CarInterfaceBase):
  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

  # Determined by iteratively plotting and minimizing error for f(angle, speed) = steer.
  @staticmethod
  def get_steer_feedforward_volt(desired_angle, v_ego):
    desired_angle *= 0.02904609
    sigmoid = desired_angle / (1 + fabs(desired_angle))
    return 0.10006696 * sigmoid * (v_ego + 3.12485927)

  def get_steer_feedforward_function(self):
    if self.CP.carFingerprint in (CAR.CHEVROLET_VOLT, CAR.CHEVROLET_VOLT_CC):
      return self.get_steer_feedforward_volt
    else:
      return CarInterfaceBase.get_steer_feedforward_default

  def torque_from_lateral_accel_siglin(self, latcontrol_inputs: LatControlInputs, torque_params: car.CarParams.LateralTorqueTuning, lateral_accel_error: float,
                                       lateral_accel_deadzone: float, friction_compensation: bool, gravity_adjusted: bool) -> float:
    friction = get_friction(lateral_accel_error, lateral_accel_deadzone, FRICTION_THRESHOLD, torque_params, friction_compensation)

    def sig(val):
      return 1 / (1 + exp(-val)) - 0.5

    # The "lat_accel vs torque" relationship is assumed to be the sum of "sigmoid + linear" curves
    # An important thing to consider is that the slope at 0 should be > 0 (ideally >1)
    # This has big effect on the stability about 0 (noise when going straight)
    # ToDo: To generalize to other GMs, explore tanh function as the nonlinear
    non_linear_torque_params = NON_LINEAR_TORQUE_PARAMS.get(self.CP.carFingerprint)
    assert non_linear_torque_params, "The params are not defined"
    a, b, c, _ = non_linear_torque_params
    steer_torque = (sig(latcontrol_inputs.lateral_acceleration * a) * b) + (latcontrol_inputs.lateral_acceleration * c)
    return float(steer_torque) + friction

  def torque_from_lateral_accel_neural(self, latcontrol_inputs: LatControlInputs, torque_params: car.CarParams.LateralTorqueTuning, lateral_accel_error: float,
                                       lateral_accel_deadzone: float, friction_compensation: bool, gravity_adjusted: bool) -> float:
    friction = get_friction(lateral_accel_error, lateral_accel_deadzone, FRICTION_THRESHOLD, torque_params, friction_compensation)
    inputs = list(latcontrol_inputs)
    if gravity_adjusted:
      inputs[0] += inputs[1]
    return float(self.neural_ff_model.predict(inputs)) + friction

  def torque_from_lateral_accel(self) -> TorqueFromLateralAccelCallbackType:
    if self.CP.carFingerprint in (CAR.CHEVROLET_BOLT_EUV, CAR.CHEVROLET_BOLT_CC):
      self.neural_ff_model = NanoFFModel(NEURAL_PARAMS_PATH, self.CP.carFingerprint)
      return self.torque_from_lateral_accel_neural
    elif self.CP.carFingerprint in NON_LINEAR_TORQUE_PARAMS:
      return self.torque_from_lateral_accel_siglin
    else:
      return self.torque_from_lateral_accel_linear

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "gm"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.gm)]
    ret.autoResumeSng = False
    ret.enableBsm = 0x142 in fingerprint[CanBus.POWERTRAIN]
    if PEDAL_MSG in fingerprint[0]:
      ret.enableGasInterceptor = True
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_GM_GAS_INTERCEPTOR

    useEVTables = Params().get_bool("EVTable")

    if candidate in EV_CAR:
      ret.transmissionType = TransmissionType.direct
    else:
      ret.transmissionType = TransmissionType.automatic

    ret.longitudinalTuning.deadzoneBP = [0.]
    ret.longitudinalTuning.deadzoneV = [0.15]

    ret.longitudinalTuning.kpBP = [0.]
    ret.longitudinalTuning.kiBP = [0.]

    if candidate in CAMERA_ACC_CAR:
      ret.experimentalLongitudinalAvailable = candidate not in CC_ONLY_CAR
      ret.networkLocation = NetworkLocation.fwdCamera
      ret.radarUnavailable = True  # no radar
      ret.pcmCruise = True
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_GM_HW_CAM
      ret.minEnableSpeed = 5 * CV.KPH_TO_MS
      ret.minSteerSpeed = 10 * CV.KPH_TO_MS

      # Tuning for experimental long
      ret.longitudinalTuning.kpV = [2.0]
      ret.longitudinalTuning.kiV = [0.72]
      ret.stoppingDecelRate = 2.0  # reach brake quickly after enabling
      ret.vEgoStopping = 0.25
      ret.vEgoStarting = 0.25

      if experimental_long:
        ret.pcmCruise = False
        ret.openpilotLongitudinalControl = True
        ret.safetyConfigs[0].safetyParam |= Panda.FLAG_GM_HW_CAM_LONG

    elif candidate in SDGM_CAR:
      ret.experimentalLongitudinalAvailable = False
      ret.networkLocation = NetworkLocation.fwdCamera
      ret.pcmCruise = True
      ret.radarUnavailable = True
      ret.minEnableSpeed = -1.  # engage speed is decided by ASCM
      ret.minSteerSpeed = 30 * CV.MPH_TO_MS
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_GM_HW_SDGM

    else:  # ASCM, OBD-II harness
      ret.openpilotLongitudinalControl = True
      ret.networkLocation = NetworkLocation.gateway
      ret.radarUnavailable = False # kans
      ret.pcmCruise = False  # stock non-adaptive cruise control is kept off
      # supports stop and go, but initial engage must (conservatively) be above 18mph
      ret.minEnableSpeed = -1 * CV.MPH_TO_MS
      ret.minSteerSpeed = (6.7 if useEVTables else 7) * CV.MPH_TO_MS

      # Tuning
      ret.longitudinalTuning.kpV = [1.75]
      ret.longitudinalTuning.kiV = [0.36]
      ret.stoppingDecelRate = 0.2
      if ret.enableGasInterceptor:
        # Need to set ASCM long limits when using pedal interceptor, instead of camera ACC long limits
        ret.safetyConfigs[0].safetyParam |= Panda.FLAG_GM_HW_ASCM_LONG

    # Start with a baseline tuning for all GM vehicles. Override tuning as needed in each model section below.
    ret.steerActuatorDelay = 0.2  # Default delay, not measured yet

    ret.steerLimitTimer = 0.4
    ret.radarTimeStep = 0.0667  # GM radar runs at 15Hz instead of standard 20Hz
    ret.longitudinalActuatorDelayUpperBound = 0.5  # large delay to initially start braking

    if candidate in (CAR.CHEVROLET_VOLT, CAR.CHEVROLET_VOLT_CC):
      ret.minEnableSpeed = -1
      ret.tireStiffnessFactor = 0.469  # Stock Michelin Energy Saver A/S, LiveParameters

      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

      ret.longitudinalTuning.kpBP = [0.]
      ret.longitudinalTuning.kpV = [1.75]
      ret.longitudinalTuning.kiBP = [0.]
      ret.longitudinalTuning.kiV = [0.36]
      ret.stoppingDecelRate = 0.2 # brake_travel/s while trying to stop
      ret.stopAccel = -0.5
      ret.startAccel = 0.8
      ret.vEgoStarting = 0.25
      ret.vEgoStopping = 0.25
      ret.enableBsm = 0x142 in fingerprint[CanBus.POWERTRAIN]

      # softer long tune for ev table
      if useEVTables: 
        ret.longitudinalTuning.kpBP = [0.]
        ret.longitudinalTuning.kpV = [1.75]
        ret.longitudinalTuning.kiBP = [0.]
        ret.longitudinalTuning.kiV = [0.36]
        ret.stoppingDecelRate = 0.1 # brake_travel/s while trying to stop
        ret.stopAccel = -0.5
        ret.startAccel = 0.8
        ret.vEgoStarting = 0.25
        ret.vEgoStopping = 0.25

    elif candidate == CAR.GMC_ACADIA:
      ret.minEnableSpeed = -1.  # engage speed is decided by pcm
      ret.steerActuatorDelay = 0.2
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.BUICK_LACROSSE:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.CADILLAC_ESCALADE:
      ret.minEnableSpeed = -1.  # engage speed is decided by pcm
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate in (CAR.CADILLAC_ESCALADE_ESV, CAR.CADILLAC_ESCALADE_ESV_2019):
      ret.minEnableSpeed = -1.  # engage speed is decided by pcm

      if candidate == CAR.CADILLAC_ESCALADE_ESV:
        ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[10., 41.0], [10., 41.0]]
        ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.13, 0.24], [0.01, 0.02]]
        ret.lateralTuning.pid.kf = 0.000045
      else:
        ret.steerActuatorDelay = 0.2
        CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate in (CAR.CHEVROLET_BOLT_EUV, CAR.CHEVROLET_BOLT_CC):
      ret.steerActuatorDelay = 0.2
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

      if ret.enableGasInterceptor:
        # ACC Bolts use pedal for full longitudinal control, not just sng
        ret.flags |= GMFlags.PEDAL_LONG.value

    elif candidate == CAR.CHEVROLET_SILVERADO:
      # On the Bolt, the ECM and camera independently check that you are either above 5 kph or at a stop
      # with foot on brake to allow engagement, but this platform only has that check in the camera.
      # TODO: check if this is split by EV/ICE with more platforms in the future
      if ret.openpilotLongitudinalControl:
        ret.minEnableSpeed = -1.
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate in (CAR.CHEVROLET_EQUINOX, CAR.CHEVROLET_EQUINOX_CC):
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate in (CAR.CHEVROLET_TRAILBLAZER, CAR.CHEVROLET_TRAILBLAZER_CC):
      ret.steerActuatorDelay = 0.2
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate in (CAR.CHEVROLET_SUBURBAN, CAR.CHEVROLET_SUBURBAN_CC):
      ret.steerActuatorDelay = 0.075
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.GMC_YUKON_CC:
      ret.steerActuatorDelay = 0.2
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.CADILLAC_XT4:
      ret.steerActuatorDelay = 0.2
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.CADILLAC_CT6_CC:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    if ret.enableGasInterceptor:
      ret.networkLocation = NetworkLocation.fwdCamera
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_GM_HW_CAM
      ret.minEnableSpeed = -1
      ret.pcmCruise = False
      ret.openpilotLongitudinalControl = True
      ret.stoppingControl = True
      ret.autoResumeSng = True

      if candidate in CC_ONLY_CAR:
        ret.flags |= GMFlags.PEDAL_LONG.value
        ret.safetyConfigs[0].safetyParam |= Panda.FLAG_GM_PEDAL_LONG
        # Note: Low speed, stop and go not tested. Should be fairly smooth on highway
        ret.longitudinalTuning.kpBP = [5., 35.]
        ret.longitudinalTuning.kpV = [0.35, 0.5]
        ret.longitudinalTuning.kiBP = [0., 35.0]
        ret.longitudinalTuning.kiV = [0.1, 0.1]
        ret.longitudinalTuning.kf = 0.15
        ret.stoppingDecelRate = 0.8
      else:  # Pedal used for SNG, ACC for longitudinal control otherwise
        ret.safetyConfigs[0].safetyParam |= Panda.FLAG_GM_HW_CAM_LONG
        ret.startingState = True
        ret.vEgoStopping = 0.25
        ret.vEgoStarting = 0.25

    elif candidate in CC_ONLY_CAR:
      ret.flags |= GMFlags.CC_LONG.value
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_GM_CC_LONG
      ret.radarUnavailable = True
      ret.experimentalLongitudinalAvailable = False
      ret.minEnableSpeed = 24 * CV.MPH_TO_MS
      ret.openpilotLongitudinalControl = True
      ret.pcmCruise = False

      ret.longitudinalTuning.deadzoneBP = [0.]
      ret.longitudinalTuning.deadzoneV = [0.56]  # == 2 km/h/s, 1.25 mph/s
      ret.stoppingDecelRate = 11.18  # == 25 mph/s (.04 rate)
      ret.longitudinalActuatorDelayLowerBound = 1.  # TODO: measure this
      ret.longitudinalActuatorDelayUpperBound = 2.

      ret.longitudinalTuning.kpBP = [10.7, 10.8, 28.]  # 10.7 m/s == 24 mph
      ret.longitudinalTuning.kpV = [0., 20., 20.]  # set lower end to 0 since we can't drive below that speed
      ret.longitudinalTuning.kiBP = [0.]
      ret.longitudinalTuning.kiV = [0.1]

    if candidate in CC_ONLY_CAR:
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_GM_NO_ACC

    # Exception for flashed cars, or cars whose camera was removed
    if (ret.networkLocation == NetworkLocation.fwdCamera or candidate in CC_ONLY_CAR) and CAM_MSG not in fingerprint[CanBus.CAMERA] and not candidate in SDGM_CAR:
      ret.flags |= GMFlags.NO_CAMERA.value
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_GM_NO_CAMERA

    if ACCELERATOR_POS_MSG not in fingerprint[CanBus.POWERTRAIN]:
      ret.flags |= GMFlags.NO_ACCELERATOR_POS_MSG.value

    if 608 in fingerprint[CanBus.POWERTRAIN]:
      ret.flags |= GMFlags.SPEED_RELATED_MSG.value

    return ret

  # returns a car.CarState
  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam, self.cp_loopback)

    buttonEvents = [] # kans
    # Don't add event if transitioning from INIT, unless it's to an actual button
    if self.CS.cruise_buttons != CruiseButtons.UNPRESS or self.CS.prev_cruise_buttons != CruiseButtons.INIT:
      buttonEvents.extend(create_button_events(self.CS.cruise_buttons, self.CS.prev_cruise_buttons, BUTTONS_DICT,
                                               unpressed_btn=CruiseButtons.UNPRESS)) # kans
    # kans : long_button GAP for cruise Mode(safety, ecco, high-speed..)
    if self.CS.distance_button_pressed:
      buttonEvents.append(car.CarState.ButtonEvent(pressed=True, type=ButtonType.gapAdjustCruise))
    ret.buttonEvents = buttonEvents # kans

    # The ECM allows enabling on falling edge of set, but only rising edge of resume
    events = self.create_common_events(ret, extra_gears=[GearShifter.sport, GearShifter.low,
                                                         GearShifter.eco, GearShifter.manumatic],
                                       pcm_enable=self.CP.pcmCruise, enable_buttons=(ButtonType.decelCruise,))
    if not self.CP.pcmCruise:
      if any(b.type == ButtonType.accelCruise and b.pressed for b in ret.buttonEvents):
        events.add(EventName.buttonEnable)

    # Enabling at a standstill with brake is allowed
    # TODO: verify 17 Volt can enable for the first time at a stop and allow for all GMs
    below_min_enable_speed = ret.vEgo < self.CP.minEnableSpeed or self.CS.moving_backward
    if below_min_enable_speed and not (ret.standstill and ret.brake >= 20 and
                                       (self.CP.networkLocation == NetworkLocation.fwdCamera and not self.CP.carFingerprint in SDGM_CAR)):
      events.add(EventName.belowEngageSpeed)

    # kans: 정지 상태이면서, 자동재개 신호(self.CP.autoResumeSng)가 비활성화되어 있고, 
    # resumeRequired 이벤트가 비활성화되어 있지 않으면, resumeRequired 이벤트를 활성화하고, 
    # resumeRequired 이벤트를 한번 보여주게 한다.
    if ret.cruiseState.standstill and not self.CP.autoResumeSng and not self.CS.disable_resumeRequired:
      events.add(EventName.resumeRequired)
      self.CS.resumeRequired_shown = True

    # kans: resumeRequired 이벤트가 표시된 후에는, 자동으로 재개될 때까지 resumeRequired 이벤트를 비활성화한다.
    if self.CS.resumeRequired_shown and not ret.cruiseState.standstill:
      self.CS.disable_resumeRequired = True

    # kans: 속도가 최소조향속도 미만이고, belowSteerSpeed 이벤트가 비활성화되어 있지 않으면, 
    # belowSteerSpeed 이벤트를 활성화하고,
    # belowSteerSpeed이벤트를 한번 보여주게 한다.
    if ret.vEgo < self.CP.minSteerSpeed and not self.CS.disable_belowSteerSpeed:
      events.add(EventName.belowSteerSpeed)
      self.CS.belowSteerSpeed_shown = True

    # kans: belowSteerSpeed 이벤트가 한번 표시된 후에는, 속도가 최소조향속도보다 높아질 때까지 belowSteerSpeed 이벤트를 비활성화한다.
    if self.CS.belowSteerSpeed_shown and ret.vEgo > self.CP.minSteerSpeed:
      self.CS.disable_belowSteerSpeed = True # kans

    if (self.CP.flags & GMFlags.CC_LONG.value) and ret.vEgo < self.CP.minEnableSpeed and ret.cruiseState.enabled:
      events.add(EventName.speedTooLow)

    if (self.CP.flags & GMFlags.PEDAL_LONG.value) and \
      self.CP.transmissionType == TransmissionType.direct and \
      not self.CS.single_pedal_mode and \
      c.longActive:
      events.add(EventName.pedalInterceptorNoBrake)

    ret.events = events.to_msg()

    return ret
