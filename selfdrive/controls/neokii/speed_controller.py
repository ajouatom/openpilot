import random
import numpy as np

from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.params import Params
from openpilot.selfdrive.car.hyundai.values import Buttons
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MIN, V_CRUISE_MAX
from openpilot.selfdrive.controls.neokii.navi_controller import SpeedLimiter

SYNC_MARGIN = 3.
MIN_CURVE_SPEED = 32. * CV.KPH_TO_MS
TRAJECTORY_SIZE = 33

EventName = car.CarEvent.EventName
ButtonType = car.CarState.ButtonEvent.Type


class SpeedController:
  def __init__(self, CP, CI):
    self.CP = CP
    self.CI = CI

    self.long_control = CP.openpilotLongitudinalControl

    self.is_metric = Params().get_bool('IsMetric')

    self.speed_conv_to_ms = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS
    self.speed_conv_to_clu = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH

    self.min_set_speed_clu = self.kph_to_clu(V_CRUISE_MIN)  # TODO - neokii
    self.max_set_speed_clu = self.kph_to_clu(V_CRUISE_MAX)

    self.btn = Buttons.NONE
    self.prev_button = ButtonType.unknown
    self.button_count = 0
    self.long_pressed = False

    self.target_speed = 0.
    self.wait_timer = 0
    self.alive_timer = 0
    self.alive_index = 0
    self.wait_index = 0
    self.alive_count = 0
    self.max_speed_clu = 0.
    self.curve_speed_ms = 0.
    self.cruise_speed_kph = 0.
    self.real_set_speed_kph = 0.

    self.slowing_down = False
    self.slowing_down_alert = False
    self.slowing_down_sound_alert = False
    self.active_cam = False
    self.prev_cruise_enabled = False
    self.limited_lead = False

    self.wait_count_list, self.alive_count_list = CI.get_params_adjust_set_speed()
    random.shuffle(self.wait_count_list)
    random.shuffle(self.alive_count_list)


  def kph_to_clu(self, kph):
    return int(kph * CV.KPH_TO_MS * self.speed_conv_to_clu)

  def get_alive_count(self):
    count = self.alive_count_list[self.alive_index]
    self.alive_index += 1
    if self.alive_index >= len(self.alive_count_list):
      self.alive_index = 0
    return count


  def get_wait_count(self):
    count = self.wait_count_list[self.wait_index]
    self.wait_index += 1
    if self.wait_index >= len(self.wait_count_list):
      self.wait_index = 0
    return count


  def reset(self):
    self.btn = Buttons.NONE

    self.wait_timer = 0
    self.alive_timer = 0
    self.target_speed = 0.
    self.max_speed_clu = 0.
    self.curve_speed_ms = 0.

    self.slowing_down = False
    self.slowing_down_alert = False
    self.slowing_down_sound_alert = False


  def inject_events(self, CS, events):
    if CS.cruiseState.enabled:
      if self.slowing_down_sound_alert:
        self.slowing_down_sound_alert = False
        events.add(EventName.slowingDownSpeedSound)
      elif self.slowing_down_alert:
        events.add(EventName.slowingDownSpeed)


  def cal_max_speed(self, CS, sm, clu_speed, v_cruise_kph):
    # kph
    apply_limit_speed, road_limit_speed, left_dist, first_started, cam_type, max_speed_log = \
      SpeedLimiter.instance().get_max_speed(clu_speed, self.is_metric)

    self.cal_curve_speed(sm, CS.vEgo, sm.frame)
    if self.curve_speed_ms >= MIN_CURVE_SPEED:
      max_speed_clu = min(v_cruise_kph * CV.KPH_TO_MS, self.curve_speed_ms) * self.speed_conv_to_clu
    else:
      max_speed_clu = self.kph_to_clu(v_cruise_kph)

    self.active_cam = road_limit_speed > 0 and left_dist > 0

    if apply_limit_speed >= self.kph_to_clu(10):
      if first_started:
        self.max_speed_clu = clu_speed

      max_speed_clu = min(max_speed_clu, apply_limit_speed)

      if clu_speed > apply_limit_speed:
        if not self.slowing_down_alert and not self.slowing_down:
          self.slowing_down_sound_alert = True
          self.slowing_down = True
        self.slowing_down_alert = True
      else:
        self.slowing_down_alert = False
    else:
      self.slowing_down_alert = False
      self.slowing_down = False

    lead_speed = self.get_long_lead_speed(clu_speed, sm)
    if lead_speed >= self.min_set_speed_clu:
      if lead_speed < max_speed_clu:
        max_speed_clu = lead_speed
        if not self.limited_lead:
          self.max_speed_clu = clu_speed + 3.
          self.limited_lead = True
    else:
      self.limited_lead = False

    max_speed = int(round(max_speed_clu))
    if not self.long_control or self.max_speed_clu <= 0:
      self.max_speed_clu = max_speed
    else:
      kp = 0.01
      error = max_speed - self.max_speed_clu
      self.max_speed_clu = self.max_speed_clu + error * kp


  def get_lead(self, sm):
    radar = sm['radarState']
    if radar.leadOne.status:
      return radar.leadOne
    return None


  def get_long_lead_speed(self, clu_speed, sm):
    if self.long_control:
      lead = self.get_lead(sm)
      if lead is not None:
        d = lead.dRel - 5.
        if 0. < d < -lead.vRel * 11. * 2. and lead.vRel < -1.:
          t = d / lead.vRel
          accel = -(lead.vRel / t) * self.speed_conv_to_clu
          accel *= 1.5 #1.2

          if accel < 0.:
            target_speed = clu_speed + accel
            target_speed = max(target_speed, self.min_set_speed_clu)
            return target_speed
    return 0


  def cal_curve_speed(self, sm, v_ego, frame):
    if frame % 20 == 0:
      model = sm['modelV2']
      if len(model.position.x) == TRAJECTORY_SIZE and len(model.position.y) == TRAJECTORY_SIZE:
        x = model.position.x
        y = model.position.y
        dy = np.gradient(y, x)
        d2y = np.gradient(dy, x)
        curv = d2y / (1 + dy ** 2) ** 1.5

        start = int(interp(v_ego, [10., 27.], [10, TRAJECTORY_SIZE - 10]))
        curv = curv[start:min(start + 10, TRAJECTORY_SIZE)]
        a_y_max = 2.975 - v_ego * 0.0375  # ~1.85 @ 75mph, ~2.6 @ 25mph
        v_curvature = np.sqrt(a_y_max / np.clip(np.abs(curv), 1e-4, None))
        model_speed = np.mean(v_curvature) * 0.85

        if model_speed < v_ego:
          self.curve_speed_ms = float(max(model_speed, MIN_CURVE_SPEED))
        else:
          self.curve_speed_ms = 255.

        if np.isnan(self.curve_speed_ms):
          self.curve_speed_ms = 255.
      else:
        self.curve_speed_ms = 255.


  def cal_target_speed(self, CS, clu_speed, v_cruise_kph, cruise_btn_pressed):
    override_speed = -1
    if not self.long_control:
      if CS.gasPressed and not cruise_btn_pressed:
        if clu_speed + SYNC_MARGIN > self.kph_to_clu(v_cruise_kph):
          set_speed = clip(clu_speed + SYNC_MARGIN, self.min_set_speed_clu, self.max_set_speed_clu)
          v_cruise_kph = int(round(set_speed * self.speed_conv_to_ms * CV.MS_TO_KPH))
          override_speed = v_cruise_kph

      self.target_speed = self.kph_to_clu(v_cruise_kph)
      if self.max_speed_clu > self.min_set_speed_clu:
        self.target_speed = clip(self.target_speed, self.min_set_speed_clu, self.max_speed_clu)

    elif CS.cruiseState.enabled:
      if CS.gasPressed and not cruise_btn_pressed:
        if clu_speed + SYNC_MARGIN > self.kph_to_clu(v_cruise_kph):
          set_speed = clip(clu_speed + SYNC_MARGIN, self.min_set_speed_clu, self.max_set_speed_clu)
          self.target_speed = set_speed

    return override_speed


  def get_button(self, current_set_speed):
    if self.target_speed < self.min_set_speed_clu:
      return Buttons.NONE
    error = self.target_speed - current_set_speed
    if abs(error) < 0.9:
      return Buttons.NONE
    return Buttons.RES_ACCEL if error > 0 else Buttons.SET_DECEL


  def update_v_cruise(self, CS, sm, v_cruise_kph):  # called by controlds's state_transition
    self.real_set_speed_kph = v_cruise_kph
    if CS.cruiseState.enabled:
      clu_speed = CS.vEgoCluster * self.speed_conv_to_clu
      self.cal_max_speed(CS, sm, clu_speed, v_cruise_kph)
      self.cruise_speed_kph = float(clip(v_cruise_kph, V_CRUISE_MIN, self.max_speed_clu * self.speed_conv_to_ms * CV.MS_TO_KPH))
    else:
      self.reset()
    return v_cruise_kph


  def update_can(self, enabled, CS, can_sends):
    new_v_cruise_kph = -1

    clu_speed = CS.vEgoCluster * self.speed_conv_to_clu
    ascc_enabled = enabled and CS.cruiseState.enabled and 1 < CS.cruiseState.speed < 255 and not CS.brakePressed

    btn_pressed = self.CI.CS.cruise_buttons[-1] != Buttons.NONE

    if not self.long_control:
      if not ascc_enabled or CS.cruiseState.standstill or btn_pressed:
        self.reset()
        self.wait_timer = max(self.alive_count_list) + max(self.wait_count_list)
        return new_v_cruise_kph

    if not ascc_enabled:
      self.reset()

    override_speed = self.cal_target_speed(CS, clu_speed, self.real_set_speed_kph, btn_pressed)
    if override_speed > 0:
      new_v_cruise_kph = override_speed

    if self.wait_timer > 0:
      self.wait_timer -= 1
    elif ascc_enabled and CS.vEgo > 0.1:
      if self.alive_timer == 0:
        current_set_speed_clu = int(round(CS.cruiseState.speed * self.speed_conv_to_clu))
        self.btn = self.get_button(current_set_speed_clu)
        self.alive_count = 1

      if self.btn != Buttons.NONE:
        can = self.CI.create_buttons(self.btn)
        if can is not None:
          for _ in range(self.get_alive_count()):
            can_sends.append(can)

        self.alive_timer += 1

        if self.alive_timer >= self.alive_count:
          self.alive_timer = 0
          self.wait_timer = self.get_wait_count()
          self.btn = Buttons.NONE
      else:
        if self.long_control and self.target_speed >= self.min_set_speed_clu:
          self.target_speed = 0.
    else:
      if self.long_control:
        self.target_speed = 0.
    return new_v_cruise_kph