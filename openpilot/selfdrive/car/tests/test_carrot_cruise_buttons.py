import pytest

from openpilot.cereal import car
from openpilot.selfdrive.car.cruise import ButtonType, VCruiseCarrot


def make_cruise_helper(button_kph, cruise_button_mode, carrot_cruise_active, cruise_enabled,
                       cruise_speed_initialized=True, cruise_speed_at_brake=0):
  helper = VCruiseCarrot.__new__(VCruiseCarrot)
  helper._prepare_buttons = lambda CS, v_cruise_kph: (button_kph, ButtonType.accelCruise, False)
  helper._carrot_command = lambda v_cruise_kph, button_type, long_pressed: (v_cruise_kph, button_type, long_pressed)
  helper._update_cruise_state = lambda CS, CC, v_cruise_kph: v_cruise_kph
  helper._add_log = lambda log: None

  helper._paddle_decel_active = False
  helper.autoCruiseControl_cancel_timer = 0
  helper._cruise_cancel_state = False
  helper._lat_enabled = False
  helper._pause_auto_speed_up = True
  helper._soft_hold_active = 0
  helper._cruise_ready = False
  helper._v_cruise_kph_at_brake = cruise_speed_at_brake
  helper._cruise_speed_initialized = cruise_speed_initialized
  helper._cruise_button_mode = cruise_button_mode
  helper._cruise_speed_unit = 10
  helper._cruise_speed_min = 5
  helper._paddle_mode = 0
  helper.v_ego_kph_set = 70
  helper.carrot_cruise_active = carrot_cruise_active

  CS = car.CarState(cruiseState={"standstill": False})
  CC = car.CarControl(enabled=cruise_enabled)
  return helper, CS, CC


@pytest.mark.parametrize("cruise_button_mode, button_kph", [(0, 81), (2, 81)])
def test_accel_exits_carrot_cruise_without_increasing_speed(cruise_button_mode, button_kph):
  helper, CS, CC = make_cruise_helper(
    button_kph,
    cruise_button_mode,
    carrot_cruise_active=True,
    cruise_enabled=True,
    cruise_speed_initialized=False,
    cruise_speed_at_brake=90,
  )

  assert helper._update_cruise_buttons(CS, CC, 80) == 80
  assert not helper.carrot_cruise_active
  assert helper._v_cruise_kph_at_brake == 0


@pytest.mark.parametrize("cruise_button_mode, expected_speed", [(0, 81), (2, 90)])
@pytest.mark.parametrize("helper_state", ["resume_speed", "uninitialized"])
def test_accel_always_increases_speed_while_cruise_is_on(cruise_button_mode, expected_speed, helper_state):
  helper, CS, CC = make_cruise_helper(
    81,
    cruise_button_mode,
    carrot_cruise_active=False,
    cruise_enabled=True,
    cruise_speed_initialized=helper_state != "uninitialized",
    cruise_speed_at_brake=95 if helper_state == "resume_speed" else 0,
  )

  assert helper._update_cruise_buttons(CS, CC, 80) == expected_speed
  assert helper._cruise_speed_initialized
  assert helper._v_cruise_kph_at_brake == 0


@pytest.mark.parametrize("cruise_speed_at_brake, expected_speed", [(75, 80), (90, 90)])
def test_accel_restores_at_least_brake_speed_while_cruise_is_off(cruise_speed_at_brake, expected_speed):
  helper, CS, CC = make_cruise_helper(
    81,
    cruise_button_mode=0,
    carrot_cruise_active=False,
    cruise_enabled=False,
    cruise_speed_at_brake=cruise_speed_at_brake,
  )

  assert helper._update_cruise_buttons(CS, CC, 80) == expected_speed
  assert helper._v_cruise_kph_at_brake == 0


@pytest.mark.parametrize("cruise_button_mode", [0, 2])
def test_accel_keeps_initialized_speed_without_brake_snapshot_while_cruise_is_off(cruise_button_mode):
  helper, CS, CC = make_cruise_helper(81, cruise_button_mode, carrot_cruise_active=False, cruise_enabled=False)

  assert helper._update_cruise_buttons(CS, CC, 80) == 80
