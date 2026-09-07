from openpilot.common.realtime import DT_CTRL
from openpilot.common.params import Params

class CarrotControls:
  def __init__(self, CP):
    self.CP = CP
    self.params = Params()
    self.lat_suspend_active = False
    self.lat_suspend_enter_t = 0.0
    self.lat_suspend_hold_t = 0.0

  def lat_suspend_control(self, CS, latActive, model_v2=None):
    suspend_angle = float(self.params.get_int("LatSuspendAngleDeg"))
    resume_angle  = 15
    delay_sec     = 1.0
    hold_sec      = 0.5

    blinker_on = bool(CS.leftBlinker or CS.rightBlinker)
    lane_probs = getattr(model_v2, "laneLineProbs", None) or []
    # laneLineProbs: [0: outer left, 1: inner left, 2: inner right, 3: outer right]
    lane_visible = False
    if len(lane_probs) >= 3:
      lane_visible = bool(lane_probs[1] >= 0.3 or lane_probs[2] >= 0.3)

    # 1) enter condition timer
    enter_cond = CS.steeringPressed and abs(CS.steeringAngleDeg) > suspend_angle
    # 커브길에서 차선이 30% 이상 인식되고 깜빡이를 켜지 않은 경우 수동 조향 중에도 자동조향 유지
    if lane_visible and not blinker_on:
      enter_cond = False

    if not self.lat_suspend_active:
      if enter_cond:
        self.lat_suspend_enter_t += DT_CTRL
        if self.lat_suspend_enter_t >= delay_sec:
          self.lat_suspend_active = True
          self.lat_suspend_hold_t = 0.0
      else:
        self.lat_suspend_enter_t = 0.0

    # 2) while suspended: enforce minimum hold time + hysteresis exit
    if self.lat_suspend_active:
      self.lat_suspend_hold_t += DT_CTRL

      # 수동 조향을 멈췄을 때:
      # - 스티어링 각도가 15도 미만으로 돌아왔거나
      # - 또는 차선이 인식되고 깜빡이가 꺼져 있다면 커브 중간이어도 즉시 복귀
      angle_cleared = abs(CS.steeringAngleDeg) < resume_angle
      lane_ready = lane_visible and not blinker_on
      exit_cond = (not CS.steeringPressed) and (angle_cleared or lane_ready)

      if (self.lat_suspend_hold_t >= hold_sec) and exit_cond:
        self.lat_suspend_active = False
        self.lat_suspend_enter_t = 0.0

    if self.lat_suspend_active:
      latActive = False
    return latActive