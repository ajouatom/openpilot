"""Separate DM2 timing and experimental response policy; stock mode stays stock."""
import math

from openpilot.common.realtime import DT_DMON
from openpilot.selfdrive.monitoring.policy import DriverMonitoring, AlertLevel, MonitoringPolicy


class DriverMonitoring2(DriverMonitoring):
  INTERACTION_TIMEOUTS = (15.0, 30.0, 45.0)

  def __init__(self, *args, experimental=False, **kwargs):
    super().__init__(*args, **kwargs)
    self.experimental = experimental
    self.stock_timeouts = {kind: self._timeouts(kind) for kind in ('VISION', 'WHEELTOUCH')}
    self.camera_available = True
    self.relax_pose = False
    self.wheel_factor = self.vision_factor = 1.0
    self.forward_score = 0.0
    self.forward_frames = 0
    self.forward_recovery = False
    self.last_input_event = -math.inf
    self.grace_started = -math.inf
    self.grace_expired = True
    self.now = 0.0
    self.input_credit_seconds = 0.0
    self.input_received = False
    self.timing_crossed_terminal = False
    self.parked_since = None
    self.parked_last_check = None
    self.parked_reset_done = False

  def update_parked_reset(self, now, eligible):
    """Clear accumulated DM state once per confirmed parking stop, never on an engage edge."""
    continuous = self.parked_last_check is not None and 0 <= now - self.parked_last_check <= 0.25
    self.parked_last_check = now
    if not eligible or not continuous:
      self.parked_since = None
      self.parked_reset_done = False
    if not eligible:
      return False
    if self.parked_since is None:
      self.parked_since = now
    if self.parked_reset_done or now - self.parked_since < 1.0:
      return False
    self.parked_reset_done = True
    self.too_distracted = False
    self.alert_3_cnt = self.cnt_since_alert_3 = self.no_response_cnt = self.lockout_time = 0
    self._reset_awareness()
    self.alert_level = AlertLevel.none
    self.timing_crossed_terminal = False
    self.grace_started = -math.inf
    self.grace_expired = True
    self.forward_frames = 0
    self.forward_recovery = False
    self.input_received = False
    self.input_credit_seconds = 0.0
    return True

  def _timeouts(self, kind):
    return tuple(getattr(self.settings, f'_{kind}_POLICY_ALERT_{i}_TIMEOUT') for i in (1, 2, 3))

  def _active_kind(self):
    return 'VISION' if self.active_policy == MonitoringPolicy.vision else 'WHEELTOUCH'

  def configure_context(self, now, camera_available, strict=False, clear=False):
    self.now = now
    self.input_credit_seconds = 0.0
    self.input_received = False
    empty_bonus = self.experimental and clear and not strict
    wheel_factor = 2.0 if empty_bonus else 1.0
    vision_factor = (4.0 if empty_bonus else 2.0) if self.experimental else 1.0
    custom_wheel = self.experimental or not camera_available
    wheel_base = self.INTERACTION_TIMEOUTS if custom_wheel else self.stock_timeouts['WHEELTOUCH']
    desired = {'VISION': tuple(v * vision_factor for v in self.stock_timeouts['VISION']),
               'WHEELTOUCH': tuple(v * wheel_factor for v in wheel_base)}
    source_changed = camera_available != self.camera_available
    old_kind = self._active_kind()
    old_budget = self._timeouts(old_kind)[2]
    changed = source_changed or any(desired[kind] != self._timeouts(kind) for kind in desired)
    self.camera_available = camera_available
    self.relax_pose = camera_available and self.experimental
    if source_changed:
      self.forward_score = 0.0
      self.forward_frames = 0
      self.active_policy = MonitoringPolicy.vision if camera_available else MonitoringPolicy.wheeltouch
    # Context alone cannot make an existing orange/red alert disappear. Keep
    # its active budget until a genuine response or normal attention recovery.
    kind = self._active_kind()
    if not source_changed and self.alert_level in (AlertLevel.two, AlertLevel.three) and desired[kind][2] > old_budget:
      desired[kind] = self._timeouts(kind)
    for policy, timeouts in desired.items():
      for i, value in enumerate(timeouts, 1):
        setattr(self.settings, f'_{policy}_POLICY_ALERT_{i}_TIMEOUT', value)
    self.wheel_factor = desired['WHEELTOUCH'][2] / wheel_base[2]
    self.vision_factor = desired['VISION'][2] / self.stock_timeouts['VISION'][2]
    if now - self.grace_started >= desired['WHEELTOUCH'][2] or (source_changed and self.alert_level in (AlertLevel.two, AlertLevel.three)):
      self.grace_expired = True
    if changed:
      first, second, budget = desired[kind]
      previous = self.awareness
      if not source_changed and previous > 0:
        self.awareness = 1 - (1 - previous) * old_budget / budget
      self.threshold_alert_1 = 1 - first / budget
      self.threshold_alert_2 = 1 - second / budget
      self.step_change = DT_DMON / budget
      if self.alert_level == AlertLevel.two:
        self.awareness = min(self.awareness, self.threshold_alert_2)
      elif self.alert_level == AlertLevel.three:
        self.awareness = min(self.awareness, 0)
      self.timing_crossed_terminal |= previous > 0 and self.awareness <= 0
      self.last_vision_awareness = self.last_wheeltouch_awareness = self.awareness

  @property
  def interaction_grace_remaining(self):
    if not self.experimental or not self.camera_available or self.grace_expired or self.alert_level == AlertLevel.three or self.too_distracted:
      return 0.0
    return max(0.0, self._timeouts('WHEELTOUCH')[2] - (self.now - self.grace_started))

  def _response_reset(self):
    if self.alert_level == AlertLevel.three or self.too_distracted:
      return False
    self._reset_awareness()
    self.alert_level = AlertLevel.none
    self.timing_crossed_terminal = False
    return True

  def record_interaction(self, event_time):
    if not (event_time > self.last_input_event and 0 <= self.now - event_time < 0.25):
      return
    self.last_input_event = event_time
    if self.camera_available and not self.experimental:
      return
    previous = self.awareness
    if self._response_reset():
      self.input_received = True
      self.input_credit_seconds = max(0.0, 1 - previous) * self._timeouts(self._active_kind())[2]
      if self.experimental:
        self.grace_started = event_time
        self.grace_expired = False

  def _get_distracted_types(self):
    fields = ('_POSE_PITCH_THRESHOLD', '_PITCH_NATURAL_THRESHOLD', '_POSE_YAW_THRESHOLD')
    previous = [getattr(self.settings, field) for field in fields]
    if self.relax_pose and self.alert_level not in (AlertLevel.two, AlertLevel.three):
      for field, value in zip(fields, previous, strict=True):
        setattr(self.settings, field, value * 1.2)
    try:
      super()._get_distracted_types()
    finally:
      for field, value in zip(fields, previous, strict=True):
        setattr(self.settings, field, value)

  def _update_states(self, driver_state, *args, **kwargs):
    super()._update_states(driver_state, *args, **kwargs)
    driver = driver_state.rightDriverData if self.wheel_on_right else driver_state.leftDriverData
    pitch_center = self.settings._PITCH_NATURAL_OFFSET
    yaw_center = self.settings._YAW_NATURAL_OFFSET
    if self.pose.calibrated:
      pitch_center = min(max(self.pose.pitch_offsetter.filtered_stat.mean(), self.settings._PITCH_MIN_OFFSET),
                         self.settings._PITCH_MAX_OFFSET)
      yaw_center = min(max(self.pose.yaw_offsetter.filtered_stat.mean(), self.settings._YAW_MIN_OFFSET), self.settings._YAW_MAX_OFFSET)
    # Evidence score, not a calibrated gaze or wakefulness probability.
    self.forward_score = max(0.0, min(driver.faceProb, driver.leftEyeProb, driver.rightEyeProb,
                                      1 - driver.leftBlinkProb, 1 - driver.rightBlinkProb,
                                      1 - driver.sleepProb, 1 - driver.phoneProb, 1 - driver.sunglassesProb,
                                      1 - abs(self.pose.pitch - pitch_center) / 1.5,
                                      1 - abs(self.pose.yaw - yaw_center) / 1.5))
    confident = self.experimental and self.forward_score >= 0.9 and self.pose.low_std and not self.driver_distracted
    self.forward_frames = min(self.forward_frames + 1, round(2 / DT_DMON)) if confident else 0

  def _update_events(self, driver_engaged, op_engaged, lowspeed, wrong_gear):
    self.forward_recovery = False
    custom_camera = self.camera_available and self.experimental
    grace = custom_camera and self.interaction_grace_remaining > 0
    if custom_camera:
      # Held stock steering/gas signals must not perpetually renew the grace.
      driver_engaged = self.input_received
      if self.forward_frames * DT_DMON >= 2:
        self.forward_recovery = self._response_reset()
      if grace:
        self._response_reset()
    previous_count = self.alert_3_cnt
    super()._update_events(driver_engaged, op_engaged, lowspeed, wrong_gear)
    if grace:
      # Evaluate detections throughout the explicitly requested grace, but start
      # the camera warning clock only once the interaction allowance expires.
      self._response_reset()
    if self.timing_crossed_terminal and self.alert_level == AlertLevel.three and self.alert_3_cnt == previous_count:
      self.alert_3_cnt += 1
      self.cnt_since_alert_3 = 0
    self.timing_crossed_terminal = False
    if not op_engaged and not self.always_on:
      self.grace_started = -math.inf

  def run_without_camera(self, response, enabled, lowspeed, wrong_gear):
    self.face_detected = False
    self.driver_distracted = False
    self.is_model_uncertain = True
    self.forward_recovery = False
    self._set_policy(MonitoringPolicy.wheeltouch)
    if response:
      self._response_reset()
    self._update_events(response, enabled, lowspeed, wrong_gear)

  def get_state_packet(self, valid=True):
    packet = super().get_state_packet(valid)
    state = packet.driverMonitoringState
    state.dm2ForwardAttentionScore = self.forward_score
    state.dm2ForwardRecovery = self.forward_recovery
    state.dm2InteractionCredit = self.input_credit_seconds
    state.dm2VisionTimeoutFactor = self.vision_factor
    state.dm2InteractionGraceRemaining = self.interaction_grace_remaining
    return packet
