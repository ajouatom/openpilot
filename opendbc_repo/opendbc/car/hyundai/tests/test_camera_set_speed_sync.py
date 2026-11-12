"""CameraSetSpeedSync: the camera's stock set speed follows openpilot's so a device drop-out keeps the speed."""
from opendbc.car.hyundai.hyundaicanfd import CameraSetSpeedSync

PRESS = 12  # inside the 10 < frame % 30 <= 16 pulse window
IDLE = 20


def test_presses_toward_the_target_only_inside_the_pulse_window():
  s = CameraSetSpeedSync()
  assert s.button(87.4, 80, PRESS) == 1  # + (RES_ACCEL)
  assert s.button(87.4, 81, IDLE) == 0
  assert s.button(70, 81, PRESS) == 2  # - (SET_DECEL) for a new target


def test_equal_or_unknown_camera_speed_sends_nothing():
  s = CameraSetSpeedSync()
  assert s.button(87, 87, PRESS) == 0
  assert s.button(87, 0, PRESS) == 0


def test_overshoot_stops_for_that_target_but_not_the_next():
  s = CameraSetSpeedSync()
  assert s.button(87, 85, PRESS) == 1
  assert s.button(87, 90, PRESS) == 0  # stepped past (5/10 km/h steps): do not toggle
  assert s.button(87, 90, PRESS) == 0
  assert s.button(95, 90, PRESS) == 1


def test_reaching_the_target_resets_the_direction():
  s = CameraSetSpeedSync()
  assert s.button(87, 86, PRESS) == 1
  assert s.button(87, 87, PRESS) == 0
  assert s.button(87, 88, PRESS) == 2  # camera moved later (e.g. driver): approach again, not an overshoot


def test_target_is_clamped_to_the_stock_range():
  s = CameraSetSpeedSync()
  assert s.button(20, 30, PRESS) == 0
  assert s.button(250, 180, PRESS) == 0


GAP_PRESS = 112  # inside the 10 < frame % 100 <= 16 window (one press per second)


def test_gap_presses_until_the_camera_matches_openpilot():
  from opendbc.car.hyundai.hyundaicanfd import CameraGapSync
  s = CameraGapSync()
  assert s.button(1, 4, GAP_PRESS) == 3  # GAP_DIST: 11-08/09 logs had openpilot 1 with the camera at 4
  assert s.button(1, 4, GAP_PRESS + 50) == 0  # outside the window
  assert s.button(1, 1, GAP_PRESS + 100) == 0
  assert s.button(1, 0, GAP_PRESS) == 0  # camera gap unknown


def test_gap_sync_gives_up_when_the_camera_does_not_follow():
  from opendbc.car.hyundai.hyundaicanfd import CameraGapSync
  s = CameraGapSync()
  sent = sum(s.button(2, 4, GAP_PRESS + 100 * i) == 3 for i in range(20))
  assert sent == CameraGapSync.MAX_PRESSES
  assert s.button(3, 4, GAP_PRESS + 3000) == 3  # a new target tries again
