from collections import deque

import numpy as np

from opendbc.car import DT_CTRL


DRIVER_OVERRIDE_CONFIRM_FRAMES = 8
DRIVER_OVERRIDE_RELEASE_FRAMES = int(0.2 / DT_CTRL)
DRIVER_OVERRIDE_ERROR_MIN = 1.0
PRE_OVERRIDE_CONFIRM_FRAMES = 2
PRE_OVERRIDE_TORQUE_FLOOR_RATIO = 0.5
PRE_OVERRIDE_TORQUE_DELTA = 10.0
TORQUE_TRANSITION_DELTA = 20.0

TORQUE_FILTER_TAU = 0.12
OSCILLATION_BASELINE_TAU = 0.25
OSCILLATION_ENVELOPE_TAU = 0.12
OSCILLATION_WINDOW_FRAMES = int(0.5 / DT_CTRL)
OSCILLATION_MIN_TORQUE_FLIPS = 4
OSCILLATION_TRACKING_ENVELOPE_MIN = 0.4
OSCILLATION_EPS_ENVELOPE_MIN = 0.8
OSCILLATION_RECOVERY_MAX_TORQUE_FLIPS = 1
OSCILLATION_RECOVERY_TRACKING_ENVELOPE_MAX = 0.25
OSCILLATION_RECOVERY_EPS_ENVELOPE_MAX = 0.5
OSCILLATION_STABLE_FRAMES = int(0.3 / DT_CTRL)


class HyundaiAngleSteering:
  """Driver handoff and EPS oscillation protection for Hyundai angle control."""

  def __init__(self, max_torque: float, min_torque: float, steer_threshold: float):
    self.max_torque = float(max_torque)
    self.min_torque = float(min_torque)
    self.steer_threshold = max(float(steer_threshold), 1.0)
    self.reset()

  def reset(self):
    self.active = False
    self.max_torque_command = 0.0
    self.driver_override_active = False
    self.driver_override_frames = 0
    self.driver_release_frames = 0
    self.pre_override_frames = 0
    self.recapturing = False

    self.driver_torque_envelope = 0.0
    self.tracking_error_slow = 0.0
    self.eps_torque_slow = 0.0
    self.tracking_error_envelope = 0.0
    self.eps_torque_envelope = 0.0
    self.filters_initialized = False

    self.frame = 0
    self.strong_torque_sign = 0
    self.torque_flip_frames = deque()
    self.oscillation_active = False
    self.oscillation_stable_frames = 0

  @staticmethod
  def _low_pass(value: float, filtered: float, tau: float) -> float:
    alpha = DT_CTRL / (tau + DT_CTRL)
    return filtered + alpha * (value - filtered)

  @staticmethod
  def _tracking_window(v_ego: float) -> float:
    # Keep enough angle error for the EPS to produce full curve authority, while
    # preventing a released wheel from seeing a large stored target-angle step.
    return float(np.interp(v_ego, [0.0, 10.0, 25.0], [8.0, 5.0, 2.0]))

  def _update_filters(self, tracking_error: float, driver_torque: float, eps_torque: float):
    if not self.filters_initialized:
      self.tracking_error_slow = tracking_error
      self.eps_torque_slow = eps_torque
      self.driver_torque_envelope = abs(driver_torque)
      self.filters_initialized = True

    self.driver_torque_envelope = self._low_pass(abs(driver_torque), self.driver_torque_envelope, TORQUE_FILTER_TAU)
    self.tracking_error_slow = self._low_pass(tracking_error, self.tracking_error_slow, OSCILLATION_BASELINE_TAU)
    self.eps_torque_slow = self._low_pass(eps_torque, self.eps_torque_slow, OSCILLATION_BASELINE_TAU)

    tracking_error_ac = tracking_error - self.tracking_error_slow
    eps_torque_ac = eps_torque - self.eps_torque_slow
    self.tracking_error_envelope = self._low_pass(abs(tracking_error_ac), self.tracking_error_envelope,
                                                  OSCILLATION_ENVELOPE_TAU)
    self.eps_torque_envelope = self._low_pass(abs(eps_torque_ac), self.eps_torque_envelope,
                                              OSCILLATION_ENVELOPE_TAU)

  def _update_oscillation_detector(self, driver_torque: float) -> bool:
    strong_torque_sign = 1 if driver_torque > self.steer_threshold else -1 if driver_torque < -self.steer_threshold else 0
    if strong_torque_sign != 0:
      if self.strong_torque_sign != 0 and strong_torque_sign != self.strong_torque_sign:
        self.torque_flip_frames.append(self.frame)
      self.strong_torque_sign = strong_torque_sign

    while self.torque_flip_frames and self.torque_flip_frames[0] < self.frame - OSCILLATION_WINDOW_FRAMES:
      self.torque_flip_frames.popleft()

    oscillating = (len(self.torque_flip_frames) >= OSCILLATION_MIN_TORQUE_FLIPS and
                   self.tracking_error_envelope > OSCILLATION_TRACKING_ENVELOPE_MIN and
                   self.eps_torque_envelope > OSCILLATION_EPS_ENVELOPE_MIN)
    if oscillating:
      self.oscillation_active = True
      self.oscillation_stable_frames = 0
    elif self.oscillation_active:
      stable = (len(self.torque_flip_frames) <= OSCILLATION_RECOVERY_MAX_TORQUE_FLIPS and
                self.tracking_error_envelope < OSCILLATION_RECOVERY_TRACKING_ENVELOPE_MAX and
                self.eps_torque_envelope < OSCILLATION_RECOVERY_EPS_ENVELOPE_MAX)
      self.oscillation_stable_frames = self.oscillation_stable_frames + 1 if stable else 0
      if self.oscillation_stable_frames >= OSCILLATION_STABLE_FRAMES:
        self.oscillation_active = False
        self.oscillation_stable_frames = 0
    return self.oscillation_active

  def _safe_recapture_angle(self, limited_angle: float, steering_angle: float, v_ego: float) -> float:
    tracking_window = self._tracking_window(v_ego)
    return float(np.clip(limited_angle, steering_angle - tracking_window, steering_angle + tracking_window))

  def update(self, active: bool, desired_angle: float, limited_angle: float, steering_angle: float,
             driver_torque: float, eps_torque: float, steering_pressed: bool, v_ego: float) -> tuple[float, float]:
    if not active:
      self.reset()
      return float(steering_angle), 0.0

    if not self.active:
      self.active = True
      self.max_torque_command = self.min_torque
      self.recapturing = True

    tracking_error = float(limited_angle - steering_angle)
    desired_error = float(desired_angle - steering_angle)
    self._update_filters(tracking_error, driver_torque, eps_torque)
    oscillation_active = self._update_oscillation_detector(driver_torque)

    # Hyundai's column torque includes EPS/road reaction. A real override must
    # persist against the requested path; same-direction load is not driver intent.
    opposing_torque = (abs(desired_error) > DRIVER_OVERRIDE_ERROR_MIN and
                       driver_torque * desired_error < 0.0)

    if oscillation_active:
      self.driver_override_frames = 0
      self.pre_override_frames = 0
    else:
      override_candidate = steering_pressed and opposing_torque
      self.driver_override_frames = self.driver_override_frames + 1 if override_candidate else 0
      if self.driver_override_frames >= DRIVER_OVERRIDE_CONFIRM_FRAMES:
        self.driver_override_active = True
        self.driver_release_frames = 0
        self.recapturing = False

      pre_override_candidate = (not self.driver_override_active and opposing_torque and
                                abs(driver_torque) > self.steer_threshold * 0.7)
      self.pre_override_frames = self.pre_override_frames + 1 if pre_override_candidate else 0

    if oscillation_active:
      # A physical oscillation is not a hand on the wheel. Unload the EPS, then
      # re-enter through the bounded tracking window once the motion is stable.
      self.max_torque_command = max(self.min_torque, self.max_torque_command - TORQUE_TRANSITION_DELTA)
      self.recapturing = True
      command_angle = steering_angle
    elif self.driver_override_active:
      self.max_torque_command = max(self.min_torque, self.max_torque_command - TORQUE_TRANSITION_DELTA)
      command_angle = steering_angle

      torque_released = self.driver_torque_envelope < self.steer_threshold * 0.6
      self.driver_release_frames = self.driver_release_frames + 1 if not steering_pressed and torque_released else 0
      if self.driver_release_frames >= DRIVER_OVERRIDE_RELEASE_FRAMES:
        self.driver_override_active = False
        self.driver_override_frames = 0
        self.driver_release_frames = 0
        self.recapturing = True
    elif self.pre_override_frames >= PRE_OVERRIDE_CONFIRM_FRAMES:
      pre_override_floor = self.max_torque * PRE_OVERRIDE_TORQUE_FLOOR_RATIO
      self.max_torque_command = max(pre_override_floor, self.max_torque_command - PRE_OVERRIDE_TORQUE_DELTA)
      self.recapturing = True
      command_angle = self._safe_recapture_angle(limited_angle, steering_angle, v_ego)
    elif self.recapturing:
      # Restore full curve authority quickly. Safety comes from bounding target
      # angle error during recapture, not from leaving the EPS weak for seconds.
      self.max_torque_command = min(self.max_torque, self.max_torque_command + TORQUE_TRANSITION_DELTA)
      command_angle = self._safe_recapture_angle(limited_angle, steering_angle, v_ego)

      aligned = abs(desired_error) <= min(2.0, self._tracking_window(v_ego) * 0.5)
      if aligned and self.max_torque_command >= self.max_torque:
        self.recapturing = False
    else:
      self.max_torque_command = self.max_torque
      command_angle = limited_angle

    self.frame += 1
    return float(command_angle), float(self.max_torque_command)
