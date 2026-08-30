from types import SimpleNamespace

import pytest

from openpilot.cereal import car
from openpilot.selfdrive.car.cruise import ButtonType, VCruiseCarrot, is_hold_interlock_active


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
  helper._cruise_available = True
  helper._hold_interlock_active = False
  helper._steering_interlock_active = False
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


def test_auto_hold_blocks_automatic_cruise_activation():
  helper = VCruiseCarrot.__new__(VCruiseCarrot)
  helper._cruise_available = True
  helper._hold_interlock_active = True
  helper._steering_interlock_active = False
  helper._activate_cruise = 0
  helper._add_log = lambda log: None

  helper._cruise_control(1, -1, "Cruise on (test)")

  assert helper._activate_cruise == 0


def test_large_steering_angle_blocks_automatic_cruise_activation():
  helper = VCruiseCarrot.__new__(VCruiseCarrot)
  helper._cruise_available = True
  helper._hold_interlock_active = False
  helper._steering_interlock_active = True
  helper._activate_cruise = 0
  helper._add_log = lambda log: None

  helper._cruise_control(1, -1, "Cruise on (speed)")

  assert helper._activate_cruise == 0


@pytest.mark.parametrize(("cruise_available", "expected_activate"), [(False, 0), (True, 1)])
def test_cruise_availability_gates_automatic_activation(cruise_available, expected_activate):
  helper = VCruiseCarrot.__new__(VCruiseCarrot)
  helper._cruise_available = cruise_available
  helper._hold_interlock_active = False
  helper._steering_interlock_active = False
  helper._cruise_cancel_state = False
  helper._cancel_timer = 0
  helper._activate_cruise = 0
  helper._soft_hold_active = 0
  helper.autoCruiseControl = 1
  helper.autoCruiseControl_cancel_timer = 0
  helper._add_log = lambda log: None

  helper._cruise_control(1, -1, "Cruise on (test)")

  assert helper._activate_cruise == expected_activate


def test_soft_hold_does_not_arm_when_cruise_is_unavailable():
  helper = VCruiseCarrot.__new__(VCruiseCarrot)
  helper.CP = SimpleNamespace(pcmCruise=False)
  helper.autoCruiseControl = 1
  helper.enabled_last = False
  helper._cruise_ready = False
  helper._paddle_decel_active = False
  helper._gas_pressed_count = -1
  helper._gas_pressed_count_last = 0
  helper._gas_pressed_value = 0
  helper._gas_tok_timer = 40
  helper._gas_tok = False
  helper._brake_pressed_count = 60
  helper._soft_hold_count = 60
  helper._soft_hold_active = 0
  helper.soft_hold_on_cancel = False
  helper._cruise_cancel_state = False
  helper.autoCruiseControl_cancel_timer = 0

  CS = SimpleNamespace(
    gasPressed=False,
    brakePressed=True,
    vEgo=0.0,
    gearShifter=car.CarState.GearShifter.drive,
    cruiseState=SimpleNamespace(available=False),
  )
  helper._prepare_brake_gas(CS, car.CarControl(enabled=False))

  assert helper._soft_hold_count == 0
  assert helper._soft_hold_active == 0


@pytest.mark.parametrize(("cancel_timer", "expected_count", "expected_active"), [
  (1, 0, 0),
  (0, 61, 1),
])
def test_post_shift_cancel_timer_gates_soft_hold(cancel_timer, expected_count, expected_active):
  helper = VCruiseCarrot.__new__(VCruiseCarrot)
  helper.CP = SimpleNamespace(pcmCruise=False)
  helper.autoCruiseControl = 1
  helper.enabled_last = False
  helper._cruise_ready = False
  helper._paddle_decel_active = False
  helper._gas_pressed_count = -1
  helper._gas_pressed_count_last = 0
  helper._gas_pressed_value = 0
  helper._gas_tok_timer = 40
  helper._gas_tok = False
  helper._brake_pressed_count = 60
  helper._soft_hold_count = 60
  helper._soft_hold_active = 0
  helper.soft_hold_on_cancel = False
  helper._cruise_cancel_state = False
  helper.autoCruiseControl_cancel_timer = cancel_timer

  CS = SimpleNamespace(
    gasPressed=False,
    brakePressed=True,
    vEgo=0.0,
    gearShifter=car.CarState.GearShifter.drive,
    cruiseState=SimpleNamespace(available=True),
  )
  helper._prepare_brake_gas(CS, car.CarControl(enabled=False))

  assert helper._soft_hold_count == expected_count
  assert helper._soft_hold_active == expected_active


@pytest.mark.parametrize(("soft_hold_on_cancel", "expected_count", "expected_active"), [
  (False, 0, 0),
  (True, 61, 1),
])
def test_cancel_state_soft_hold_policy(soft_hold_on_cancel, expected_count, expected_active):
  helper = VCruiseCarrot.__new__(VCruiseCarrot)
  helper.CP = SimpleNamespace(pcmCruise=False)
  helper.autoCruiseControl = 1
  helper.enabled_last = False
  helper._cruise_ready = False
  helper._paddle_decel_active = False
  helper._gas_pressed_count = -1
  helper._gas_pressed_count_last = 0
  helper._gas_pressed_value = 0
  helper._gas_tok_timer = 40
  helper._gas_tok = False
  helper._brake_pressed_count = 60
  helper._soft_hold_count = 60
  helper._soft_hold_active = 0
  helper.soft_hold_on_cancel = soft_hold_on_cancel
  helper._cruise_cancel_state = True
  helper.autoCruiseControl_cancel_timer = 0

  CS = SimpleNamespace(
    gasPressed=False,
    brakePressed=True,
    vEgo=0.0,
    gearShifter=car.CarState.GearShifter.drive,
    cruiseState=SimpleNamespace(available=True),
  )
  helper._prepare_brake_gas(CS, car.CarControl(enabled=False))

  assert helper._soft_hold_count == expected_count
  assert helper._soft_hold_active == expected_active


def test_soft_hold_on_cancel_keeps_cancel_state_while_engaging():
  helper = VCruiseCarrot.__new__(VCruiseCarrot)
  helper._cruise_available = True
  helper._hold_interlock_active = False
  helper._steering_interlock_active = False
  helper._cruise_cancel_state = True
  helper._cancel_timer = 0
  helper._activate_cruise = 0
  helper._soft_hold_active = 1
  helper.soft_hold_on_cancel = True
  helper.autoCruiseControl = 1
  helper.autoCruiseControl_cancel_timer = 0
  helper._add_log = lambda log: None

  helper._engage_soft_hold()

  assert helper._soft_hold_active == 2
  assert helper._cruise_cancel_state
  assert helper._activate_cruise == 1


@pytest.mark.parametrize(("cancel_state", "expected_activate"), [
  (False, 0),
  (True, -1),
])
def test_gas_releases_cancel_soft_hold_to_cruise_off(cancel_state, expected_activate):
  helper = VCruiseCarrot.__new__(VCruiseCarrot)
  helper._cruise_available = True
  helper._hold_interlock_active = False
  helper._steering_interlock_active = False
  helper._cruise_cancel_state = cancel_state
  helper._cancel_timer = 0
  helper._activate_cruise = 0
  helper._soft_hold_count = 0
  helper._soft_hold_active = 2
  helper.autoCruiseControl = 1
  helper.autoCruiseControl_cancel_timer = 0
  helper.disengage_on_accelerator = False
  helper._cruise_ready = False
  helper._paddle_decel_active = False
  helper.carrot_cruise_active = False
  helper._gas_pressed_count = -1
  helper._gas_pressed_count_last = 0
  helper._gas_pressed_value = 0
  helper._gas_tok_timer = 40
  helper._gas_tok = False
  helper._brake_pressed_count = -1
  helper._add_log = lambda log: None

  CS = SimpleNamespace(gasPressed=True, gas=0.2, brakePressed=False)
  helper._prepare_brake_gas(CS, car.CarControl(enabled=True))

  assert helper._soft_hold_active == 0
  assert helper._cruise_cancel_state is cancel_state
  assert helper._activate_cruise == expected_activate


@pytest.mark.parametrize(("brake_hold_active", "parking_brake", "active"), [
  (False, False, False),
  (True, False, True),
  (False, True, True),
  (True, True, True),
])
def test_cruise_hold_interlock_sources(brake_hold_active, parking_brake, active):
  CS = car.CarState(brakeHoldActive=brake_hold_active, parkingBrake=parking_brake)

  assert is_hold_interlock_active(CS) is active
