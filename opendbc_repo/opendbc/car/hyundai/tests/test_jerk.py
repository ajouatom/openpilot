from collections import deque
from types import SimpleNamespace

import pytest

from opendbc.car import DT_CTRL
from opendbc.car.hyundai.carcontroller import CANFD_JERK_ERROR_DELAY, CANFD_JERK_ERROR_FILTER_TIME, HyundaiJerk, LongCtrlState, \
                                                   calculate_canfd_jerk_limits
from opendbc.car.hyundai.hyundaicanfd import apply_accel_jerk_limit, create_acc_control_scc2
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
  "accel",
  [
    0.0,
    -0.8,
    -1.2,
    -1.5,
    -2.0,
    -2.5,
    -3.2,
  ],
)
def test_canfd_lower_jerk_does_not_use_accel_without_tracking_error(accel):
  upper, lower = calculate_canfd_jerk_limits(accel, jerk=0.0)

  assert upper == pytest.approx(1.0)
  assert lower == pytest.approx(1.0)


def test_canfd_lower_jerk_keeps_mpc_transient_authority():
  _, lower = calculate_canfd_jerk_limits(accel=-1.0, jerk=-1.0)

  assert lower == pytest.approx(4.0)


def test_canfd_lower_jerk_blends_feedforward_for_tracking_error():
  _, partial_lower = calculate_canfd_jerk_limits(accel=-2.0, jerk=0.0, tracking_error=0.5)
  _, full_lower = calculate_canfd_jerk_limits(accel=-2.0, jerk=0.0, tracking_error=0.75)
  _, capped_lower = calculate_canfd_jerk_limits(accel=-3.2, jerk=-2.0, tracking_error=2.0)

  assert partial_lower == pytest.approx(2.0)
  assert full_lower == pytest.approx(3.0)
  assert capped_lower == pytest.approx(5.0)


def test_canfd_positive_jerk_releases_feedforward_immediately():
  _, lower = calculate_canfd_jerk_limits(accel=-3.2, jerk=0.2, tracking_error=2.0)

  assert lower == pytest.approx(1.0)


def test_canfd_small_jerk_noise_keeps_feedforward():
  _, lower = calculate_canfd_jerk_limits(accel=-2.0, jerk=0.05, tracking_error=0.75)

  assert lower == pytest.approx(3.0)


def test_canfd_upper_jerk_uses_stock_like_floor():
  upper_floor, _ = calculate_canfd_jerk_limits(accel=0.0, jerk=0.0)
  upper_dynamic, _ = calculate_canfd_jerk_limits(accel=0.0, jerk=0.8)

  assert upper_floor == pytest.approx(1.0)
  assert upper_dynamic == pytest.approx(1.6)


def test_canfd_accel_value_uses_lower_jerk_while_decelerating():
  value = apply_accel_jerk_limit(a_raw=-4.0, a_value_last=0.0, jerk_u=1.0, jerk_l=5.0)

  assert value == pytest.approx(-0.1)


def test_canfd_accel_value_uses_upper_jerk_while_releasing_deceleration():
  value = apply_accel_jerk_limit(a_raw=0.0, a_value_last=-2.0, jerk_u=1.0, jerk_l=5.0)

  assert value == pytest.approx(-1.98)


def test_canfd_accel_value_reaches_raw_without_overshoot():
  value = 0.0
  for _ in range(50):
    value = apply_accel_jerk_limit(a_raw=-4.0, a_value_last=value, jerk_u=1.0, jerk_l=5.0)

  assert value == pytest.approx(-4.0)


def test_camera_scc_accel_value_keeps_ramping_from_previous_value():
  class FakePacker:
    @staticmethod
    def make_can_msg(name, bus, values):
      return name, bus, values.copy()

  CAN = SimpleNamespace(ECAN=0)
  CS = SimpleNamespace(
    scc_control={"ACC_ObjRelSpd": 0.0, "InfoDisplay": 0},
    softHoldActive=0,
    paddle_button_prev=0,
    out=SimpleNamespace(aEgo=0.0, vEgo=20.0),
  )
  hud_control = SimpleNamespace(leadDistanceBars=2, leadVisible=False)
  jerk = SimpleNamespace(carrot_cruise=0, jerk_u=1.0, jerk_l=5.0)

  first_msg, first_value = create_acc_control_scc2(
    FakePacker(), CAN, True, 0.0, -4.0, False, False, 100.0, hud_control, jerk, CS,
  )
  second_msg, second_value = create_acc_control_scc2(
    FakePacker(), CAN, True, first_value, -4.0, False, False, 100.0, hud_control, jerk, CS,
  )

  assert first_msg[2]["aReqRaw"] == pytest.approx(-4.0)
  assert first_msg[2]["aReqValue"] == pytest.approx(-0.1)
  assert second_msg[2]["aReqValue"] == pytest.approx(-0.2)
  assert second_value == pytest.approx(-0.2)


def test_canfd_sustained_tracking_error_adds_trim():
  controller = build_jerk_controller()
  CP = SimpleNamespace(flags=HyundaiFlags.CANFD)
  CS = SimpleNamespace(out=SimpleNamespace(aEgo=-1.4, brakePressed=False, gasPressed=False))
  actuators = SimpleNamespace(longControlState=LongCtrlState.pid, jerk=0.0)

  for _ in range(100):
    controller.make_jerk(CP, CS, accel=-2.0, actuators=actuators, hud_control=None)

  assert controller.jerk_l == pytest.approx(2.4)


def test_canfd_tracking_assist_releases_on_overshoot():
  controller = build_jerk_controller()
  CP = SimpleNamespace(flags=HyundaiFlags.CANFD)
  CS = SimpleNamespace(out=SimpleNamespace(aEgo=-1.0, brakePressed=False, gasPressed=False))
  actuators = SimpleNamespace(longControlState=LongCtrlState.pid, jerk=0.0)

  for _ in range(100):
    controller.make_jerk(CP, CS, accel=-2.0, actuators=actuators, hud_control=None)
  assert controller.jerk_l > 1.0

  CS.out.aEgo = -2.5
  controller.make_jerk(CP, CS, accel=-2.0, actuators=actuators, hud_control=None)

  assert controller.jerk_l == pytest.approx(1.0)


def test_canfd_tracking_assist_does_not_chatter_near_deadband():
  controller = build_jerk_controller()
  CP = SimpleNamespace(flags=HyundaiFlags.CANFD)
  CS = SimpleNamespace(out=SimpleNamespace(aEgo=-1.0, brakePressed=False, gasPressed=False))
  actuators = SimpleNamespace(longControlState=LongCtrlState.pid, jerk=0.0)

  for _ in range(100):
    controller.make_jerk(CP, CS, accel=-2.0, actuators=actuators, hud_control=None)
  assisted_lower = controller.jerk_l

  CS.out.aEgo = -1.8
  for _ in range(20):
    controller.make_jerk(CP, CS, accel=-2.0, actuators=actuators, hud_control=None)

  assert controller.jerk_l > 1.0
  assert controller.jerk_l < assisted_lower


def test_canfd_tracking_assist_releases_when_plan_unwinds():
  controller = build_jerk_controller()
  CP = SimpleNamespace(flags=HyundaiFlags.CANFD)
  CS = SimpleNamespace(out=SimpleNamespace(aEgo=-1.0, brakePressed=False, gasPressed=False))
  actuators = SimpleNamespace(longControlState=LongCtrlState.pid, jerk=0.0)

  for _ in range(100):
    controller.make_jerk(CP, CS, accel=-2.0, actuators=actuators, hud_control=None)
  assert controller.jerk_l > 1.0

  actuators.jerk = 0.2
  controller.make_jerk(CP, CS, accel=-2.0, actuators=actuators, hud_control=None)

  assert controller.jerk_l == pytest.approx(1.0)


def test_classic_can_jerk_limits_are_unchanged():
  controller = build_jerk_controller()
  CP = SimpleNamespace(flags=0)
  CS = SimpleNamespace(out=SimpleNamespace(aEgo=-1.4, brakePressed=False, gasPressed=False))
  actuators = SimpleNamespace(longControlState=LongCtrlState.pid, jerk=0.0)

  controller.make_jerk(CP, CS, accel=-2.0, actuators=actuators, hud_control=None)

  assert controller.jerk_u == pytest.approx(0.5)
  assert controller.jerk_l == pytest.approx(1.0)
