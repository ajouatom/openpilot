import unittest
from types import SimpleNamespace
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.carrot.carrot_controls import CarrotControls

class TestCarrotControls(unittest.TestCase):
  def setUp(self):
    self.params = Params()
    self.params.put_int("LatSuspendAngleDeg", 45)
    self.cc = CarrotControls(CP=SimpleNamespace())

  def _make_cs(self, angle=0.0, pressed=False, left_blinker=False, right_blinker=False):
    return SimpleNamespace(
      steeringAngleDeg=angle,
      steeringPressed=pressed,
      leftBlinker=left_blinker,
      rightBlinker=right_blinker,
    )

  def _make_model(self, left_prob=0.5, right_prob=0.5):
    return SimpleNamespace(laneLineProbs=[0.1, left_prob, right_prob, 0.1])

  def test_curve_with_lane_and_no_blinker_keeps_lat_active(self):
    # 차선 인식 30% 이상이고 깜빡이 OFF인 커브(각도 50도, suspend_angle 45도 초과)에서
    # 핸들에 힘이 들어가더라도(steeringPressed) suspend에 진입하지 않고 자동조향 유지
    cs = self._make_cs(angle=50.0, pressed=True)
    model = self._make_model(left_prob=0.4, right_prob=0.4)

    for _ in range(120): # 1.2초 동안 지속 (delay_sec 1.0초 초과)
      lat_active = self.cc.lat_suspend_control(cs, latActive=True, model_v2=model)
      self.assertTrue(lat_active)
    self.assertFalse(self.cc.lat_suspend_active)

  def test_blinker_on_allows_suspend(self):
    # 차선 변경을 위해 깜빡이를 켜고 핸들을 돌리면 suspend 진입
    cs = self._make_cs(angle=50.0, pressed=True, left_blinker=True)
    model = self._make_model(left_prob=0.4, right_prob=0.4)

    for _ in range(120): # 1.2초 동안 지속
      lat_active = self.cc.lat_suspend_control(cs, latActive=True, model_v2=model)

    self.assertTrue(self.cc.lat_suspend_active)
    self.assertFalse(lat_active)

  def test_suspended_resumes_in_curve_if_lane_ready(self):
    # 이미 suspend된 상태에서 손을 떼었을 때 (steeringPressed = False):
    # 각도가 30도(resume_angle 15도 초과)인 커브 구간이어도,
    # 차선이 인식되고 깜빡이가 꺼져 있으면 hold_sec(0.5초) 후 자동조향으로 즉시 복귀
    self.cc.lat_suspend_active = True
    self.cc.lat_suspend_hold_t = 0.0

    cs = self._make_cs(angle=30.0, pressed=False)
    model = self._make_model(left_prob=0.5, right_prob=0.5)

    for _ in range(60): # 0.6초 경과 (hold_sec 0.5초 초과)
      lat_active = self.cc.lat_suspend_control(cs, latActive=True, model_v2=model)

    self.assertFalse(self.cc.lat_suspend_active)
    self.assertTrue(lat_active)

  def test_suspended_resumes_on_straight_even_without_lane(self):
    # 차선이 인식되지 않는 상태에서는 직선(각도 < 15도)으로 복귀해야 재개
    self.cc.lat_suspend_active = True
    self.cc.lat_suspend_hold_t = 0.0

    # 커브 중(30도) 차선 없음 -> 복귀 안 됨
    cs_curve = self._make_cs(angle=30.0, pressed=False)
    model_no_lane = self._make_model(left_prob=0.1, right_prob=0.1)
    for _ in range(60):
      lat_active = self.cc.lat_suspend_control(cs_curve, latActive=True, model_v2=model_no_lane)
    self.assertTrue(self.cc.lat_suspend_active)
    self.assertFalse(lat_active)

    # 직선 진입(10도) -> 복귀됨
    cs_straight = self._make_cs(angle=10.0, pressed=False)
    for _ in range(60):
      lat_active = self.cc.lat_suspend_control(cs_straight, latActive=True, model_v2=model_no_lane)
    self.assertFalse(self.cc.lat_suspend_active)
    self.assertTrue(lat_active)

if __name__ == '__main__':
  unittest.main()
