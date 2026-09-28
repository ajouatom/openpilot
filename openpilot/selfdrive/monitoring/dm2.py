"""Experimental DM policy, isolated from the unchanged comma policy.

No experimental relaxation applies to eye closure, sleep, phone use,
orange/red alerts, or the ten seconds following a new moving vehicle.
"""
from openpilot.common.realtime import DT_DMON
from openpilot.selfdrive.monitoring.policy import DriverMonitoring, AlertLevel, MonitoringPolicy


class DriverMonitoring2(DriverMonitoring):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self.relax_pose = False
    self.wheel_factor = 1.0
    self.camera_available = True
    self.forward_score = 0.0
    self.forward_frames = 0
    self.protected_distraction = False
    self.forward_recovery = False
    self.last_input_credit = -1.0
    self.last_input_event = -1.0
    self.input_credit_seconds = 0.0

  def set_camera_available(self, available, factor=1.0):
    if available == self.camera_available:
      return
    self.camera_available = available
    self.forward_score = 0.0
    self.forward_frames = 0
    # Sensor-source transitions retain monitoring progress, including orange/red.
    # Do not restore the stock policy's previously saved (possibly fresh) budget.
    self.active_policy = MonitoringPolicy.vision if available else MonitoringPolicy.wheeltouch
    self.wheel_factor = 1.0 if available or self.alert_level in (AlertLevel.two, AlertLevel.three) else factor
    prefix = "_VISION_POLICY" if available else "_WHEELTOUCH_POLICY"
    budget = getattr(self.settings, prefix + "_ALERT_3_TIMEOUT")
    self.threshold_alert_1 = 1 - getattr(self.settings, prefix + "_ALERT_1_TIMEOUT") / budget
    self.threshold_alert_2 = 1 - getattr(self.settings, prefix + "_ALERT_2_TIMEOUT") / budget
    self.step_change = DT_DMON / (budget * self.wheel_factor)
    if self.alert_level == AlertLevel.two:
      self.awareness = min(self.awareness, self.threshold_alert_2)
    elif self.alert_level == AlertLevel.three:
      self.awareness = min(self.awareness, 0)
    self.last_vision_awareness = self.awareness
    self.last_wheeltouch_awareness = self.awareness

  def _get_distracted_types(self):
    fields = ("_POSE_PITCH_THRESHOLD", "_PITCH_NATURAL_THRESHOLD", "_POSE_YAW_THRESHOLD")
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
    # A conservative evidence score, not a calibrated gaze or wakefulness probability.
    self.forward_score = max(0.0, min(driver.faceProb, driver.leftEyeProb, driver.rightEyeProb,
                                      1 - driver.leftBlinkProb, 1 - driver.rightBlinkProb,
                                      1 - driver.sleepProb, 1 - driver.phoneProb, 1 - driver.sunglassesProb,
                                      1 - abs(self.pose.pitch - pitch_center) / 1.5,
                                      1 - abs(self.pose.yaw - yaw_center) / 1.5))
    confident = self.relax_pose and self.forward_score >= 0.9 and self.pose.low_std and not self.driver_distracted
    self.forward_frames = min(self.forward_frames + 1, round(2 / DT_DMON)) if confident else 0
    self.protected_distraction |= any(self.distracted_types[cause] for cause in ('eye', 'sleep', 'phone'))

  def _update_events(self, *args, **kwargs):
    self.forward_recovery = (self.camera_available and self.relax_pose and self.forward_frames * DT_DMON >= 2 and
                             not self.protected_distraction and self.alert_level not in (AlertLevel.two, AlertLevel.three))
    fields = ('_TIMEOUT_RECOVERY_FACTOR_MAX', '_TIMEOUT_RECOVERY_FACTOR_MIN')
    original = [getattr(self.settings, field) for field in fields]
    if self.forward_recovery:
      for field, value in zip(fields, original, strict=True):
        setattr(self.settings, field, value * 1.5)
    try:
      super()._update_events(*args, **kwargs)
    finally:
      for field, value in zip(fields, original, strict=True):
        setattr(self.settings, field, value)
    if self.awareness == 1:
      self.protected_distraction = False

  def get_state_packet(self, valid=True):
    packet = super().get_state_packet(valid)
    packet.driverMonitoringState.dm2ForwardAttentionScore = self.forward_score
    packet.driverMonitoringState.dm2ForwardRecovery = self.forward_recovery
    packet.driverMonitoringState.dm2InteractionCredit = self.input_credit_seconds
    return packet

  def credit_camera_interaction(self, now, event_time):
    self.input_credit_seconds = 0.0
    if not (event_time > self.last_input_event and 0 <= now - event_time < 0.25):
      return
    self.last_input_event = event_time
    if (not self.camera_available or not self.relax_pose or self.protected_distraction or
        any(self.distracted_types[cause] for cause in ('eye', 'sleep', 'phone')) or
        self.alert_level in (AlertLevel.two, AlertLevel.three) or now - self.last_input_credit < 1.0):
      return
    budget = (self.settings._VISION_POLICY_ALERT_3_TIMEOUT if self.active_policy == MonitoringPolicy.vision
              else self.settings._WHEELTOUCH_POLICY_ALERT_3_TIMEOUT)
    previous = self.awareness
    self.awareness = min(1.0, self.awareness + 2.0 / budget)
    self.input_credit_seconds = (self.awareness - previous) * budget
    self.driver_interacting = True
    self.last_input_credit = now

  def run_without_camera(self, response, enabled, lowspeed, wrong_gear, factor):
    self.face_detected = False
    self.driver_distracted = False
    self.is_model_uncertain = True
    self._set_policy(MonitoringPolicy.wheeltouch)
    # Awareness is a fraction of the budget. Convert through elapsed seconds so
    # changing context never resets time already spent without a response.
    budget = self.settings._WHEELTOUCH_POLICY_ALERT_3_TIMEOUT
    elapsed = (1.0 - self.awareness) * budget * self.wheel_factor
    if self.alert_level in (AlertLevel.two, AlertLevel.three):
      factor = min(factor, self.wheel_factor)
    factor = min(max(factor, 1.0), 2.4)
    self.wheel_factor = factor
    previous_awareness = self.awareness
    previous_count = self.alert_3_cnt
    self.awareness = min(self.awareness, 0.0) if self.awareness <= 0 else 1.0 - elapsed / (budget * factor)
    self.threshold_alert_1 = 1.0 - self.settings._WHEELTOUCH_POLICY_ALERT_1_TIMEOUT / budget
    self.threshold_alert_2 = 1.0 - self.settings._WHEELTOUCH_POLICY_ALERT_2_TIMEOUT / budget
    self.step_change = DT_DMON / (budget * factor)
    self._update_events(response, enabled, lowspeed, wrong_gear)
    if previous_awareness > 0 and self.alert_level == AlertLevel.three and self.alert_3_cnt == previous_count:
      self.alert_3_cnt += 1
      self.cnt_since_alert_3 = 0
