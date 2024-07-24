import json
import os
import numpy as np
import tomllib
from abc import abstractmethod, ABC
from difflib import SequenceMatcher
from enum import StrEnum
from json import load
from typing import Any, Callable, Dict, List, NamedTuple, Optional, Tuple, Union
from collections.abc import Callable

from cereal import car
from openpilot.common.basedir import BASEDIR
from openpilot.common.conversions import Conversions as CV
from openpilot.common.simple_kalman import KF1D, get_kalman_gain
from openpilot.common.numpy_fast import clip
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car import apply_hysteresis, gen_empty_fingerprint, scale_rot_inertia, scale_tire_stiffness, STD_CARGO_KG
from openpilot.selfdrive.car.values import PLATFORMS
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, get_friction
from openpilot.selfdrive.controls.lib.events import Events
from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel

ButtonType = car.CarState.ButtonEvent.Type
GearShifter = car.CarState.GearShifter
EventName = car.CarEvent.EventName

MAX_CTRL_SPEED = (V_CRUISE_MAX + 4) * CV.KPH_TO_MS
ACCEL_MAX = 2.0
ACCEL_MIN = -4.0
FRICTION_THRESHOLD = 0.3

TORQUE_PARAMS_PATH = os.path.join(BASEDIR, 'selfdrive/car/torque_data/params.toml')
TORQUE_OVERRIDE_PATH = os.path.join(BASEDIR, 'selfdrive/car/torque_data/override.toml')
TORQUE_SUBSTITUTE_PATH = os.path.join(BASEDIR, 'selfdrive/car/torque_data/substitute.toml')
TORQUE_NN_MODEL_PATH = os.path.join(BASEDIR, 'selfdrive/car/torque_data/lat_models')

def similarity(s1:str, s2:str) -> float:
  return SequenceMatcher(None, s1, s2).ratio()

class LatControlInputs(NamedTuple):
  lateral_acceleration: float
  roll_compensation: float
  vego: float
  aego: float


TorqueFromLateralAccelCallbackType = Callable[[LatControlInputs, car.CarParams.LateralTorqueTuning, float, float, bool, bool], float]


def get_torque_params(candidate):
  with open(TORQUE_SUBSTITUTE_PATH, 'rb') as f:
    sub = tomllib.load(f)
  if candidate in sub:
    candidate = sub[candidate]

  with open(TORQUE_PARAMS_PATH, 'rb') as f:
    params = tomllib.load(f)
  with open(TORQUE_OVERRIDE_PATH, 'rb') as f:
    override = tomllib.load(f)

  # Ensure no overlap
  if sum([candidate in x for x in [sub, params, override]]) > 1:
    raise RuntimeError(f'{candidate} is defined twice in torque config')

  if candidate in override:
    out = override[candidate]
  elif candidate in params:
    out = params[candidate]
  else:
    raise NotImplementedError(f"Did not find torque params for {candidate}")
  return {key: out[i] for i, key in enumerate(params['legend'])}


# Twilsonco's Lateral Neural Network Feedforward
class FluxModel:
  # dict used to rename activation functions whose names aren't valid python identifiers
  activation_function_names = {'σ': 'sigmoid'}
  def __init__(self, params_file, zero_bias=False):
    with open(params_file, "r") as f:
      params = load(f)

    self.input_size = params["input_size"]
    self.output_size = params["output_size"]
    self.input_mean = np.array(params["input_mean"], dtype=np.float32).T
    self.input_std = np.array(params["input_std"], dtype=np.float32).T
    self.layers = []

    for layer_params in params["layers"]:
      W = np.array(layer_params[next(key for key in layer_params.keys() if key.endswith('_W'))], dtype=np.float32).T
      b = np.array(layer_params[next(key for key in layer_params.keys() if key.endswith('_b'))], dtype=np.float32).T
      if zero_bias:
        b = np.zeros_like(b)
      activation = layer_params["activation"]
      for k, v in self.activation_function_names.items():
        activation = activation.replace(k, v)
      self.layers.append((W, b, activation))

    self.validate_layers()
    self.check_for_friction_override()

  # Begin activation functions.
  # These are called by name using the keys in the model json file
  def sigmoid(self, x):
    return 1 / (1 + np.exp(-x))

  def identity(self, x):
    return x
  # End activation functions

  def forward(self, x):
    for W, b, activation in self.layers:
      x = getattr(self, activation)(x.dot(W) + b)
    return x

  def evaluate(self, input_array):
    in_len = len(input_array)
    if in_len != self.input_size:
      # If the input is length 2-4, then it's a simplified evaluation.
      # In that case, need to add on zeros to fill out the input array to match the correct length.
      if 2 <= in_len:
        input_array = input_array + [0] * (self.input_size - in_len)
      else:
        raise ValueError(f"Input array length {len(input_array)} must be length 2 or greater")

    input_array = np.array(input_array, dtype=np.float32)

    # Rescale the input array using the input_mean and input_std
    input_array = (input_array - self.input_mean) / self.input_std

    output_array = self.forward(input_array)

    return float(output_array[0, 0])

  def validate_layers(self):
    for W, b, activation in self.layers:
      if not hasattr(self, activation):
        raise ValueError(f"Unknown activation: {activation}")

  def check_for_friction_override(self):
    y = self.evaluate([10.0, 0.0, 0.2])
    self.friction_override = (y < 0.1)

def get_nn_model_path(car, eps_firmware) -> Tuple[Union[str, None, float]]:
  def check_nn_path(check_model):
    model_path = None
    max_similarity = -1.0
    for f in os.listdir(TORQUE_NN_MODEL_PATH):
      if f.endswith(".json") and car in f:
        model = f.replace(".json", "").replace(f"{TORQUE_NN_MODEL_PATH}/", "")
        similarity_score = similarity(model, check_model)
        if similarity_score > max_similarity:
          max_similarity = similarity_score
          model_path = os.path.join(TORQUE_NN_MODEL_PATH, f)
    return model_path, max_similarity

  car1 = car.replace('_', ' ')
  car1 = car1.replace(' HEV', ' HYBRID')
  car = car1.replace('EV ', 'ELECTRIC ')
  print("########get_nn_model_path :", car, eps_firmware)
  if len(eps_firmware) > 3:
    eps_firmware = eps_firmware.replace("\\", "")
    check_model = f"{car} {eps_firmware}"
  else:
    check_model = car
  model_path, max_similarity = check_nn_path(check_model)
  if max_similarity < 0.9:
    check_model = car
    model_path, max_similarity = check_nn_path(check_model)
    if max_similarity < 0.9:
      model_path = None
  return model_path, max_similarity

def get_nn_model(car, eps_firmware) -> Tuple[Union[FluxModel, None, float]]:
  print("###########get_nn_model", car)
  model, similarity_score = get_nn_model_path(car, eps_firmware)
  if model is not None:
    model = FluxModel(model)
  return model, similarity_score

# generic car and radar interfaces

class CarInterfaceBase(ABC):
  def __init__(self, CP, CarController, CarState):
    self.CP = CP
    self.VM = VehicleModel(CP)
    eps_firmware = str(next((fw.fwVersion for fw in CP.carFw if fw.ecu == "eps"), ""))

    self.frame = 0
    self.steering_unpressed = 0
    self.low_speed_alert = False
    self.no_steer_warning = False
    self.silent_steer_warning = True
    self.v_ego_cluster_seen = False

    # tw: steer warning
    self.steer_warning = 0
    self.CS = CarState(CP)
    self.cp = self.CS.get_can_parser(CP)
    self.cp_cam = self.CS.get_cam_can_parser(CP)
    self.cp_adas = self.CS.get_adas_can_parser(CP)
    self.cp_body = self.CS.get_body_can_parser(CP)
    self.cp_loopback = self.CS.get_loopback_can_parser(CP)
    self.can_parsers = [self.cp, self.cp_cam, self.cp_adas, self.cp_body, self.cp_loopback]

    dbc_name = "" if self.cp is None else self.cp.dbc_name
    self.CC: CarControllerBase = CarController(dbc_name, CP, self.VM)
    self.params = Params()
    lateral_tune = True
    print("$$$$$$$$$$$ NNFF")
    nnff_supported = self.initialize_lat_torque_nn(CP.carFingerprint, eps_firmware)
    print("$$$$$$$$$$$ nnff_supported = ", nnff_supported)
    use_comma_nnff = self.check_comma_nn_ff_support(CP.carFingerprint)
    print("$$$$$$$$$$$ use_comma_nnff = ", use_comma_nnff)
    self.use_nnff = not use_comma_nnff and nnff_supported and lateral_tune and self.params.get_bool("NNFF")
    print("$$$$$$$$$$$ use_nnff = ", self.use_nnff)
    self.use_nnff_lite = not use_comma_nnff and not nnff_supported and lateral_tune and self.params.get_bool("NNFFLite")
    print("$$$$$$$$$$$ use_nnff_lite = ", self.use_nnff_lite)

    Params().put('LongitudinalPersonalityMax', "3")

  def get_ff_nn(self, x):
    return self.lat_torque_nn_model.evaluate(x)

  def check_comma_nn_ff_support(self, car):
    try:
      with open("../car/torque_data/neural_ff_weights.json", "r") as file:
        data = json.load(file)
      return car in data

    except FileNotFoundError:
      print("Failed to open neural_ff_weights file.")
      return False

  def initialize_lat_torque_nn(self, car, eps_firmware):
    self.lat_torque_nn_model, _ = get_nn_model(car, eps_firmware)
    return (self.lat_torque_nn_model is not None)

  def apply(self, c: car.CarControl, now_nanos: int) -> tuple[car.CarControl.Actuators, list[tuple[int, int, bytes, int]]]:
    return self.CC.update(c, self.CS, now_nanos)

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return ACCEL_MIN, ACCEL_MAX

  @classmethod
  def get_non_essential_params(cls, candidate: str):
    """
    Parameters essential to controlling the car may be incomplete or wrong without FW versions or fingerprints.
    """
    return cls.get_params(candidate, gen_empty_fingerprint(), list(), False, False)

  @classmethod
  def get_params(cls, candidate: str, fingerprint: dict[int, dict[int, int]], car_fw: list[car.CarParams.CarFw], experimental_long: bool, docs: bool):
    ret = CarInterfaceBase.get_std_params(candidate)

    platform = PLATFORMS[candidate]
    ret.mass = platform.config.specs.mass
    ret.wheelbase = platform.config.specs.wheelbase
    ret.steerRatio = platform.config.specs.steerRatio
    ret.centerToFront = ret.wheelbase * platform.config.specs.centerToFrontRatio
    ret.minEnableSpeed = platform.config.specs.minEnableSpeed
    ret.minSteerSpeed = platform.config.specs.minSteerSpeed
    ret.tireStiffnessFactor = platform.config.specs.tireStiffnessFactor
    ret.flags |= int(platform.config.flags)

    ret = cls._get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs)
    params = Params()
    if ret.steerControlType != car.CarParams.SteerControlType.angle and params.get_bool("NNFF"):
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      eps_firmware = str(next((fw.fwVersion for fw in car_fw if fw.ecu == "eps"), ""))
      model, similarity_score = get_nn_model_path(candidate, eps_firmware)
      if model is not None:
        params.put("NNFFModelName", candidate)

    # Vehicle mass is published curb weight plus assumed payload such as a human driver; notCars have no assumed payload
    if not ret.notCar:
      ret.mass = ret.mass + STD_CARGO_KG

    # Set params dependent on values set by the car interface
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront, ret.tireStiffnessFactor)

    return ret

  @staticmethod
  @abstractmethod
  def _get_params(ret: car.CarParams, candidate, fingerprint: dict[int, dict[int, int]],
                  car_fw: list[car.CarParams.CarFw], experimental_long: bool, docs: bool):
    raise NotImplementedError

  @staticmethod
  def init(CP, logcan, sendcan):
    pass

  @staticmethod
  def get_steer_feedforward_default(desired_angle, v_ego):
    # Proportional to realigning tire momentum: lateral acceleration.
    return desired_angle * (v_ego**2)

  def get_steer_feedforward_function(self):
    return self.get_steer_feedforward_default

  def torque_from_lateral_accel_linear(self, latcontrol_inputs: LatControlInputs, torque_params: car.CarParams.LateralTorqueTuning,
                                       lateral_accel_error: float, lateral_accel_deadzone: float, friction_compensation: bool, gravity_adjusted: bool) -> float:
    # The default is a linear relationship between torque and lateral acceleration (accounting for road roll and steering friction)
    friction = get_friction(lateral_accel_error, lateral_accel_deadzone, FRICTION_THRESHOLD, torque_params, friction_compensation)
    return (latcontrol_inputs.lateral_acceleration / float(torque_params.latAccelFactor)) + friction

  def torque_from_lateral_accel(self) -> TorqueFromLateralAccelCallbackType:
    return self.torque_from_lateral_accel_linear

  # returns a set of default params to avoid repetition in car specific params
  @staticmethod
  def get_std_params(candidate):
    ret = car.CarParams.new_message()
    ret.carFingerprint = candidate

    # Car docs fields
    ret.maxLateralAccel = get_torque_params(candidate)['MAX_LAT_ACCEL_MEASURED']
    ret.autoResumeSng = True  # describes whether car can resume from a stop automatically

    # standard ALC params
    ret.tireStiffnessFactor = 1.0
    ret.steerControlType = car.CarParams.SteerControlType.torque
    ret.minSteerSpeed = 0.
    ret.wheelSpeedFactor = 1.0

    ret.pcmCruise = True     # openpilot's state is tied to the PCM's cruise state on most cars
    ret.minEnableSpeed = -1. # enable is done by stock ACC, so ignore this
    ret.steerRatioRear = 0.  # no rear steering, at least on the listed cars aboveA
    ret.openpilotLongitudinalControl = False
    ret.stopAccel = -2.0
    ret.stoppingDecelRate = 0.8 # brake_travel/s while trying to stop
    ret.vEgoStopping = 0.5
    ret.vEgoStarting = 0.5
    ret.stoppingControl = True
    ret.longitudinalTuning.deadzoneBP = [0.]
    ret.longitudinalTuning.deadzoneV = [0.]
    ret.longitudinalTuning.kf = 1.
    ret.longitudinalTuning.kpBP = [0.]
    ret.longitudinalTuning.kpV = [1.]
    ret.longitudinalTuning.kiBP = [0.]
    ret.longitudinalTuning.kiV = [1.]
    # TODO estimate car specific lag, use .15s for now
    ret.longitudinalActuatorDelayLowerBound = 0.15
    ret.longitudinalActuatorDelayUpperBound = 0.15
    ret.steerLimitTimer = 1.0
    return ret

  @staticmethod
  def configure_torque_tune(candidate, tune, steering_angle_deadzone_deg=0.0, use_steering_angle=True):
    params = get_torque_params(candidate)

    tune.init('torque')
    tune.torque.useSteeringAngle = use_steering_angle
    tune.torque.kp = 1.0
    tune.torque.kf = 1.0
    tune.torque.ki = 0.1
    tune.torque.friction = params['FRICTION']
    tune.torque.latAccelFactor = params['LAT_ACCEL_FACTOR']
    tune.torque.latAccelOffset = 0.0
    tune.torque.steeringAngleDeadzoneDeg = steering_angle_deadzone_deg

  @abstractmethod
  def _update(self, c: car.CarControl) -> car.CarState:
    pass

  def update(self, c: car.CarControl, can_strings: list[bytes]) -> car.CarState:
    # parse can
    for cp in self.can_parsers:
      if cp is not None:
        cp.update_strings(can_strings)

    # get CarState
    ret = self._update(c)

    ret.canValid = all(cp.can_valid for cp in self.can_parsers if cp is not None)
    ret.canTimeout = any(cp.bus_timeout for cp in self.can_parsers if cp is not None)

    if ret.vEgoCluster == 0.0 and not self.v_ego_cluster_seen:
      ret.vEgoCluster = ret.vEgo
    else:
      self.v_ego_cluster_seen = True

    # Many cars apply hysteresis to the ego dash speed
    if self.CS is not None:
      ret.vEgoCluster = apply_hysteresis(ret.vEgoCluster, self.CS.out.vEgoCluster, self.CS.cluster_speed_hyst_gap)
      if abs(ret.vEgo) < self.CS.cluster_min_speed:
        ret.vEgoCluster = 0.0

    if ret.cruiseState.speedCluster == 0:
      ret.cruiseState.speedCluster = ret.cruiseState.speed

    # copy back for next iteration
    reader = ret.as_reader()
    if self.CS is not None:
      self.CS.out = reader

    return reader


  def create_common_events(self, cs_out, extra_gears=None, pcm_enable=True, allow_enable=True,
                           enable_buttons=(ButtonType.accelCruise, ButtonType.decelCruise)):
    events = Events()
    self.mute_door = Params().get_bool("MuteDoor")
    self.mute_seatbelt = Params().get_bool("MuteSeatbelt")

    if cs_out.doorOpen and not self.mute_door:
      events.add(EventName.doorOpen)
    if cs_out.seatbeltUnlatched and not self.mute_seatbelt:
      events.add(EventName.seatbeltNotLatched)
    if cs_out.gearShifter != GearShifter.drive and (extra_gears is None or
       cs_out.gearShifter not in extra_gears):
      events.add(EventName.wrongGear)
    if cs_out.gearShifter == GearShifter.reverse:
      events.add(EventName.reverseGear)
    if not cs_out.cruiseState.available:
      events.add(EventName.wrongCarMode)
    if cs_out.espDisabled:
      events.add(EventName.espDisabled)
    if cs_out.stockFcw:
      events.add(EventName.stockFcw)
    if cs_out.stockAeb:
      events.add(EventName.stockAeb)
    if cs_out.vEgo > MAX_CTRL_SPEED:
      events.add(EventName.speedTooHigh)
    if cs_out.cruiseState.nonAdaptive:
      events.add(EventName.wrongCruiseMode)
    if cs_out.brakeHoldActive and self.CP.openpilotLongitudinalControl:
      events.add(EventName.brakeHold)
    if cs_out.parkingBrake:
      events.add(EventName.parkBrake)
    if cs_out.accFaulted:
      events.add(EventName.accFaulted)
    if cs_out.steeringPressed:
      events.add(EventName.steerOverride)

    # Handle button presses
    for b in cs_out.buttonEvents:
      # Enable OP long on falling edge of enable buttons (defaults to accelCruise and decelCruise, overridable per-port)
      if not self.CP.pcmCruise and (b.type in enable_buttons and not b.pressed):
        events.add(EventName.buttonEnable)
        print("$$$$$$$$$$$$$$ EventName.buttonEnable")
      # Disable on rising and falling edge of cancel for both stock and OP long
      if b.type == ButtonType.cancel:
        events.add(EventName.buttonCancel)
        print("$$$$$$$$$$$$$$ EventName.buttonCancel")

    # Handle permanent and temporary steering faults
    # tw: steer warning
    self.steer_warning = self.steer_warning + 1 if cs_out.steerFaultTemporary else 0
    self.steering_unpressed = 0 if cs_out.steeringPressed else self.steering_unpressed + 1
    if cs_out.steerFaultPermanent: # 스티어폴트 선행.
      events.add(EventName.steerUnavailable)

    elif cs_out.steerFaultTemporary: # 일시오류 체크.
      if cs_out.steeringPressed and (not self.CS.out.steerFaultTemporary or self.no_steer_warning):
        self.no_steer_warning = True
      else:
        self.no_steer_warning = False

        # 핸들손올림이나 일시 스티어오류가 0.5초이상일때 경고하며, 운전자 개입은 안하는 것으로 한다.
        if self.steering_unpressed > int(0.5/DT_CTRL) and self.steer_warning > int(0.5/DT_CTRL):
          pass # events.add(EventName.steerTempUnavailable)
        else:
          events.add(EventName.steerTempUnavailableSilent)

    else:
      self.no_steer_warning = False
      self.silent_steer_warning = False

    # we engage when pcm is active (rising edge)
    # enabling can optionally be blocked by the car interface
    if pcm_enable:
      if cs_out.cruiseState.enabled and not self.CS.out.cruiseState.enabled and allow_enable:
        events.add(EventName.pcmEnable)
        print("###############  pcmEnabled")
      elif not cs_out.cruiseState.enabled:
        events.add(EventName.pcmDisable)

    return events


class RadarInterfaceBase(ABC):
  def __init__(self, CP):
    self.rcp = None
    self.pts = {}
    self.delay = 0
    self.radar_ts = CP.radarTimeStep
    self.frame = 0

  def update(self, can_strings):
    self.frame += 1
    if (self.frame % int(100 * self.radar_ts)) == 0:
      return car.RadarData.new_message()
    return None


class CarStateBase(ABC):
  def __init__(self, CP):
    self.CP = CP
    self.params = Params()
    self.car_fingerprint = CP.carFingerprint
    self.out = car.CarState.new_message()

    self.cruise_buttons = 0
    self.left_blinker_cnt = 0
    self.right_blinker_cnt = 0
    self.steering_pressed_cnt = 0
    self.left_blinker_prev = False
    self.right_blinker_prev = False
    self.cluster_speed_hyst_gap = 0.0
    self.cluster_min_speed = 0.0  # min speed before dropping to 0

    Q = [[0.0, 0.0], [0.0, 100.0]]
    R = 0.3
    A = [[1.0, DT_CTRL], [0.0, 1.0]]
    C = [[1.0, 0.0]]
    x0=[[0.0], [0.0]]
    K = get_kalman_gain(DT_CTRL, np.array(A), np.array(C), np.array(Q), R)
    self.v_ego_kf = KF1D(x0=x0, A=A, C=C[0], K=K)
    self.v_ego_clu_kf = KF1D(x0=x0, A=A, C=C[0], K=K)


  def update_speed_kf(self, v_ego_raw):
    if abs(v_ego_raw - self.v_ego_kf.x[0][0]) > 2.0:  # Prevent large accelerations when car starts at non zero speed
      self.v_ego_kf.set_x([[v_ego_raw], [0.0]])

    v_ego_x = self.v_ego_kf.update(v_ego_raw)
    return float(v_ego_x[0]), float(v_ego_x[1])
  
  def update_clu_speed_kf(self, v_ego_raw):
    if abs(v_ego_raw - self.v_ego_clu_kf.x[0][0]) > 2.0:  # Prevent large accelerations when car starts at non zero speed
      self.v_ego_clu_kf.set_x([[v_ego_raw], [0.0]])

    v_ego_x = self.v_ego_clu_kf.update(v_ego_raw)
    return float(v_ego_x[0]), float(v_ego_x[1])

  def get_wheel_speeds(self, fl, fr, rl, rr, unit=CV.KPH_TO_MS):
    factor = unit * self.CP.wheelSpeedFactor

    wheelSpeeds = car.CarState.WheelSpeeds.new_message()
    wheelSpeeds.fl = fl * factor
    wheelSpeeds.fr = fr * factor
    wheelSpeeds.rl = rl * factor
    wheelSpeeds.rr = rr * factor
    return wheelSpeeds

  def update_blinker_from_lamp(self, blinker_time: int, left_blinker_lamp: bool, right_blinker_lamp: bool):
    """Update blinkers from lights. Enable output when light was seen within the last `blinker_time`
    iterations"""
    # TODO: Handle case when switching direction. Now both blinkers can be on at the same time
    self.left_blinker_cnt = blinker_time if left_blinker_lamp else max(self.left_blinker_cnt - 1, 0)
    self.right_blinker_cnt = blinker_time if right_blinker_lamp else max(self.right_blinker_cnt - 1, 0)
    return self.left_blinker_cnt > 0, self.right_blinker_cnt > 0

  def update_steering_pressed(self, steering_pressed, steering_pressed_min_count):
    """Applies filtering on steering pressed for noisy driver torque signals."""
    self.steering_pressed_cnt += 1 if steering_pressed else -1
    self.steering_pressed_cnt = clip(self.steering_pressed_cnt, 0, steering_pressed_min_count * 2)
    return self.steering_pressed_cnt > steering_pressed_min_count

  def update_blinker_from_stalk(self, blinker_time: int, left_blinker_stalk: bool, right_blinker_stalk: bool):
    """Update blinkers from stalk position. When stalk is seen the blinker will be on for at least blinker_time,
    or until the stalk is turned off, whichever is longer. If the opposite stalk direction is seen the blinker
    is forced to the other side. On a rising edge of the stalk the timeout is reset."""

    if left_blinker_stalk:
      self.right_blinker_cnt = 0
      if not self.left_blinker_prev:
        self.left_blinker_cnt = blinker_time

    if right_blinker_stalk:
      self.left_blinker_cnt = 0
      if not self.right_blinker_prev:
        self.right_blinker_cnt = blinker_time

    self.left_blinker_cnt = max(self.left_blinker_cnt - 1, 0)
    self.right_blinker_cnt = max(self.right_blinker_cnt - 1, 0)

    self.left_blinker_prev = left_blinker_stalk
    self.right_blinker_prev = right_blinker_stalk

    return bool(left_blinker_stalk or self.left_blinker_cnt > 0), bool(right_blinker_stalk or self.right_blinker_cnt > 0)

  @staticmethod
  def parse_gear_shifter(gear: str | None) -> car.CarState.GearShifter:
    if gear is None:
      return GearShifter.unknown

    d: dict[str, car.CarState.GearShifter] = {
      'P': GearShifter.park, 'PARK': GearShifter.park,
      'R': GearShifter.reverse, 'REVERSE': GearShifter.reverse,
      'N': GearShifter.neutral, 'NEUTRAL': GearShifter.neutral,
      'E': GearShifter.eco, 'ECO': GearShifter.eco,
      'T': GearShifter.manumatic, 'MANUAL': GearShifter.manumatic,
      'D': GearShifter.drive, 'DRIVE': GearShifter.drive,
      'S': GearShifter.sport, 'SPORT': GearShifter.sport,
      'L': GearShifter.low, 'LOW': GearShifter.low,
      'B': GearShifter.brake, 'BRAKE': GearShifter.brake,
    }
    return d.get(gear.upper(), GearShifter.unknown)

  @staticmethod
  def get_can_parser(CP):
    return None

  @staticmethod
  def get_cam_can_parser(CP):
    return None

  @staticmethod
  def get_adas_can_parser(CP):
    return None

  @staticmethod
  def get_body_can_parser(CP):
    return None

  @staticmethod
  def get_loopback_can_parser(CP):
    return None


SendCan = tuple[int, int, bytes, int]


class CarControllerBase(ABC):
  def __init__(self, dbc_name: str, CP, VM):
    pass

  @abstractmethod
  def update(self, CC: car.CarControl.Actuators, CS: car.CarState, now_nanos: int) -> tuple[car.CarControl.Actuators, list[SendCan]]:
    pass


INTERFACE_ATTR_FILE = {
  "FINGERPRINTS": "fingerprints",
  "FW_VERSIONS": "fingerprints",
}

# interface-specific helpers

def get_interface_attr(attr: str, combine_brands: bool = False, ignore_none: bool = False) -> dict[str | StrEnum, Any]:
  # read all the folders in selfdrive/car and return a dict where:
  # - keys are all the car models or brand names
  # - values are attr values from all car folders
  result = {}
  for car_folder in sorted([x[0] for x in os.walk(BASEDIR + '/selfdrive/car')]):
    try:
      brand_name = car_folder.split('/')[-1]
      brand_values = __import__(f'openpilot.selfdrive.car.{brand_name}.{INTERFACE_ATTR_FILE.get(attr, "values")}', fromlist=[attr])
      if hasattr(brand_values, attr) or not ignore_none:
        attr_data = getattr(brand_values, attr, None)
      else:
        continue

      if combine_brands:
        if isinstance(attr_data, dict):
          for f, v in attr_data.items():
            result[f] = v
      else:
        result[brand_name] = attr_data
    except (ImportError, OSError):
      pass

  return result


class NanoFFModel:
  def __init__(self, weights_loc: str, platform: str):
    self.weights_loc = weights_loc
    self.platform = platform
    self.load_weights(platform)

  def load_weights(self, platform: str):
    with open(self.weights_loc) as fob:
      self.weights = {k: np.array(v) for k, v in json.load(fob)[platform].items()}

  def relu(self, x: np.ndarray):
    return np.maximum(0.0, x)

  def forward(self, x: np.ndarray):
    assert x.ndim == 1
    x = (x - self.weights['input_norm_mat'][:, 0]) / (self.weights['input_norm_mat'][:, 1] - self.weights['input_norm_mat'][:, 0])
    x = self.relu(np.dot(x, self.weights['w_1']) + self.weights['b_1'])
    x = self.relu(np.dot(x, self.weights['w_2']) + self.weights['b_2'])
    x = self.relu(np.dot(x, self.weights['w_3']) + self.weights['b_3'])
    x = np.dot(x, self.weights['w_4']) + self.weights['b_4']
    return x

  def predict(self, x: list[float], do_sample: bool = False):
    x = self.forward(np.array(x))
    if do_sample:
      pred = np.random.laplace(x[0], np.exp(x[1]) / self.weights['temperature'])
    else:
      pred = x[0]
    pred = pred * (self.weights['output_norm_mat'][1] - self.weights['output_norm_mat'][0]) + self.weights['output_norm_mat'][0]
    return pred
