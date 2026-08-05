from collections import deque
from types import SimpleNamespace

import pytest

from opendbc.car import DT_CTRL
from opendbc.car.hyundai.carcontroller import CANFD_JERK_ERROR_DELAY, CANFD_JERK_ERROR_FILTER_TIME, HyundaiJerk, LongCtrlState, \
                                                   calculate_canfd_jerk_limits
from opendbc.car.hyundai.values import HyundaiFlags
from openpilot.common.filter_simple import MyMovingAverage


def build_jerk_controller():
  controller = HyundaiJerk.__new__(HyundaiJerk)
  controller.jerk = 0.0
  controller.jerk_u = controller.jerk_l = 0.0
  controller.cb_upper = controller.cb_lower = 0.0
  controller.jerk_u_min = 0.5
  controller.accel_request_history = deque(maxlen=max(1, round(CANFD_JERK_ERROR_DELAY / DT_CTRL)))
  controller.jerk_error_filter = MyMovingAverage(max(1, round(CANFD_JERK_ERROR_FILTER_TIME / DT_CTRL)), 0.0)
  return controller


@pytest.mark.parametrize(
  "accel, expected_lower",
  [
    (0.0, 1.2),
    (-0.8, 1.2),
    (-1.2, 1.2),
    (-1.5, 1.7),
    (-2.0, 3.0),
    (-2.5, 3.3),
    (-3.2, 3.7),
  ],
)
def test_canfd_lower_jerk_tracks_decel_request(accel, expected_lower):
  upper, lower = calculate_canfd_jerk_limits(accel, jerk=0.0)

  assert upper == pytest.approx(1.0)
  assert lower == pytest.approx(expected_lower)


def test_canfd_lower_jerk_keeps_mpc_transient_authority():
  _, lower = calculate_canfd_jerk_limits(accel=-1.0, jerk=-1.0)

  assert lower == pytest.approx(4.0)


def test_canfd_lower_jerk_adds_bounded_tracking_error_trim():
  _, lower = calculate_canfd_jerk_limits(accel=-2.0, jerk=0.0, tracking_error=0.6)
  _, capped_lower = calculate_canfd_jerk_limits(accel=-3.2, jerk=-2.0, tracking_error=2.0)

  assert lower == pytest.approx(3.35)
  assert capped_lower == pytest.approx(5.0)


def test_canfd_upper_jerk_uses_stock_like_floor():
  upper_floor, _ = calculate_canfd_jerk_limits(accel=0.0, jerk=0.0)
  upper_dynamic, _ = calculate_canfd_jerk_limits(accel=0.0, jerk=0.8)

  assert upper_floor == pytest.approx(1.0)
  assert upper_dynamic == pytest.approx(1.6)


def test_canfd_sustained_tracking_error_adds_trim():
  controller = build_jerk_controller()
  CP = SimpleNamespace(flags=HyundaiFlags.CANFD)
  CS = SimpleNamespace(out=SimpleNamespace(aEgo=-1.4, brakePressed=False, gasPressed=False))
  actuators = SimpleNamespace(longControlState=LongCtrlState.pid, jerk=0.0)

  for _ in range(100):
    controller.make_jerk(CP, CS, accel=-2.0, actuators=actuators, hud_control=None)

  assert controller.jerk_l == pytest.approx(3.35)


def test_classic_can_jerk_limits_are_unchanged():
  controller = build_jerk_controller()
  CP = SimpleNamespace(flags=0)
  CS = SimpleNamespace(out=SimpleNamespace(aEgo=-1.4, brakePressed=False, gasPressed=False))
  actuators = SimpleNamespace(longControlState=LongCtrlState.pid, jerk=0.0)

  controller.make_jerk(CP, CS, accel=-2.0, actuators=actuators, hud_control=None)

  assert controller.jerk_u == pytest.approx(0.5)
  assert controller.jerk_l == pytest.approx(1.0)
