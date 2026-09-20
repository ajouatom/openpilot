from types import SimpleNamespace

import pytest

from openpilot.cereal import car
from openpilot.selfdrive.car.cruise import ButtonType, VCruiseCarrot, is_hold_interlock_active
from openpilot.selfdrive.carrot.cruise_gap import cruise_gap_levels
from openpilot.selfdrive.carrot.bluetooth.model import BLUETOOTH_CANCEL


def make_cruise_helper(button_kph, cruise_button_mode, carrot_cruise_active, cruise_enabled,
                       cruise_speed_initialized=True, cruise_speed_at_brake=0):
  helper = VCruiseCarrot.__new__(VCruiseCarrot)
  helper.bluetooth_commands = SimpleNamespace(read=lambda **kwargs: None)
  helper._prepare_buttons = lambda CS, v_cruise_kph, remote=None: (button_kph, ButtonType.accelCruise, False)
  helper._carrot_command = lambda v_cruise_kph, button_type, long_pressed: (v_cruise_kph, button_type, long_pressed)
  helper._update_cruise_state = lambda CS, CC, v_cruise_kph: v_cruise_kph
  helper._add_log = lambda log: None

  helper._paddle_decel_active = False
  helper._activate_cruise = 0
  helper._cancel_timer = 0
  helper.autoCruiseControl = 1
  helper.button_cnt = 0
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


@pytest.mark.parametrize(('action', 'expected'), [('accelCruise', 81), ('decelCruise', 79)])
def test_bluetooth_uses_cruise_button_steps(action, expected):
  helper, CS, CC = make_cruise_helper(81, 0, False, True)
  helper._prepare_buttons = VCruiseCarrot._prepare_buttons.__get__(helper)
  helper.button_cnt = 0
  helper.button_long_time = 50
  helper.long_pressed = False
  helper._cruise_speed_unit_basic = 1
  helper._cruise_button_long_delay = 50
  helper.is_metric = True
  helper.bluetooth_commands = SimpleNamespace(read=lambda allowed: action if allowed else None)
  CS.canValid = True
  CS.cruiseState.available = True
  CS.gearShifter = 'drive'
  CS.cruiseSpeedBigStep = True  # A physical stalk's big-step bit must not affect HID.
  assert helper._update_cruise_buttons(CS, CC, 80) == expected
  assert helper.button_cnt == 0
  assert len(CS.buttonEvents) == 0  # No fake CAN/CarState events.


@pytest.mark.parametrize('block', ['can', 'available', 'gear', 'button', 'held'])
def test_bluetooth_does_not_override_vehicle_button_or_gate(block):
  helper, CS, CC = make_cruise_helper(81, 0, False, True)
  helper.button_cnt = int(block == 'held')
  CS.canValid = block != 'can'
  CS.cruiseState.available = block != 'available'
  CS.gearShifter = 'park' if block == 'gear' else 'drive'
  if block == 'button':
    CS.buttonEvents = [{'type': 'cancel', 'pressed': True}]
  allowed_values = []
  helper.bluetooth_commands = SimpleNamespace(read=lambda allowed: allowed_values.append(allowed))
  helper._update_cruise_buttons(CS, CC, 80)
  assert allowed_values == [False]


def test_bluetooth_paddle_decel_uses_ready_path_independent_of_physical_paddle_mode():
  helper, CS, CC = make_cruise_helper(80, 0, False, True)
  helper.button_cnt = 0
  helper._prepare_buttons = lambda *args: (80, 0, False)
  helper.bluetooth_commands = SimpleNamespace(read=lambda allowed: 'paddleDecel' if allowed else None)
  CS.canValid = CS.cruiseState.available = True
  CS.gearShifter = 'drive'
  calls = []
  helper._cruise_control = lambda *args: calls.append(args[:2])
  helper._update_cruise_buttons(CS, CC, 80)
  assert calls == [(-2, -1)]
  assert helper._paddle_decel_active
  assert helper._paddle_mode == 0


def test_bluetooth_gap_cycles_even_with_pcm_gap():
  helper, CS, CC = make_cruise_helper(80, 0, False, True)
  helper.button_cnt = 0
  helper._prepare_buttons = lambda *args: (80, ButtonType.gapAdjustCruise, False)
  helper.bluetooth_commands = SimpleNamespace(read=lambda allowed: 'gapAdjustCruise' if allowed else None)
  CS.canValid = CS.cruiseState.available = True
  CS.gearShifter = 'drive'
  CS.pcmCruiseGap = 4
  helper.CP = SimpleNamespace(openpilotLongitudinalControl=True)
  values = {'LongitudinalPersonalityMax': 4, 'CruiseGapLevels': 4, 'LongitudinalPersonality': 3}
  helper.params = SimpleNamespace(get_int=values.__getitem__, put_int_nonblocking=values.__setitem__)
  helper._update_cruise_buttons(CS, CC, 80)
  assert values['LongitudinalPersonality'] != 3


@pytest.mark.parametrize('initial', [False, True])
def test_bluetooth_carrot_cruise_enters_existing_mode_without_toggling_or_speed_change(initial):
  helper, CS, CC = make_cruise_helper(80, 0, initial, True)
  helper.button_cnt = 0
  helper._prepare_buttons = lambda *args: (80, 0, False)
  helper.bluetooth_commands = SimpleNamespace(read=lambda allowed: 'carrotCruise' if allowed else None)
  CS.canValid = CS.cruiseState.available = True
  CS.gearShifter = 'drive'
  calls = []
  helper._cruise_control = lambda *args: calls.append(args)
  assert helper._update_cruise_buttons(CS, CC, 80) == 80
  assert helper.carrot_cruise_active
  assert not helper._paddle_decel_active
  assert calls == []


def make_remote_helper(action, enabled=False):
  helper, CS, CC = make_cruise_helper(80, 0, False, enabled)
  helper._prepare_buttons = VCruiseCarrot._prepare_buttons.__get__(helper)
  helper.button_long_time = helper._cruise_button_long_delay = 50
  helper.long_pressed = False
  helper._cruise_speed_unit_basic = 1
  helper.is_metric = True
  helper._cancel_button_mode = 0
  helper._lfa_button_mode = 0
  helper.useLaneLineSpeed = 50
  helper.useLaneLineSpeedApply = 0
  helper.bluetooth_commands = SimpleNamespace(read=lambda allowed: action if allowed else None)
  CS.canValid = CS.cruiseState.available = True
  CS.gearShifter = 'drive'
  return helper, CS, CC


@pytest.mark.parametrize('action', ['accelCruise', 'decelCruise', 'accelCruiseLong', 'decelCruiseLong'])
@pytest.mark.parametrize('auto_cruise', [0, 1])
def test_remote_cruise_buttons_request_engagement_independently_of_auto_cruise(action, auto_cruise):
  helper, CS, CC = make_remote_helper(action)
  helper.autoCruiseControl = auto_cruise
  helper._update_cruise_buttons(CS, CC, 80)
  assert helper._activate_cruise == 1
  assert helper._lat_enabled
  assert helper.button_cnt == 0
  assert not CS.buttonEvents


@pytest.mark.parametrize('block', ['brake', 'gas', 'hold', 'steering', 'gear', 'can', 'unavailable', 'physical_cancel', 'negative_request'])
@pytest.mark.parametrize('action', ['accelCruise', 'decelCruiseLong'])
def test_remote_engagement_preserves_interlocks_and_physical_priority(block, action):
  helper, CS, CC = make_remote_helper(action)
  CS.brakePressed = block == 'brake'
  CS.gasPressed = block == 'gas'
  helper._hold_interlock_active = block == 'hold'
  helper._steering_interlock_active = block == 'steering'
  if block == 'gear':
    CS.gearShifter = 'park'
  if block == 'can':
    CS.canValid = False
  if block == 'unavailable':
    CS.cruiseState.available = helper._cruise_available = False
  if block == 'physical_cancel':
    CS.buttonEvents = [{'type': 'cancel', 'pressed': True}]
  if block == 'negative_request':
    helper._activate_cruise = -1
  helper._update_cruise_buttons(CS, CC, 80)
  assert helper._activate_cruise <= 0


@pytest.mark.parametrize('block', [None, 'doorOpen', 'seatbeltUnlatched', 'accFaulted'])
def test_remote_engagement_reaches_existing_state_machine_without_bypassing_no_entry(block):
  from openpilot.selfdrive.car.car_specific import CarSpecificEvents
  from openpilot.selfdrive.selfdrived.state import StateMachine
  helper, CS, CC = make_remote_helper('accelCruise')
  helper.autoCruiseControl = 0
  if block:
    setattr(CS, block, True)
  helper._update_cruise_buttons(CS, CC, 80)
  CS.activateCruise = helper._activate_cruise  # card.py publishes this existing field.
  car_events = CarSpecificEvents(SimpleNamespace(pcmCruise=False, openpilotLongitudinalControl=True))
  events = car_events.create_common_events(CS, car.CarState(), pcm_enable=False)
  enabled, _ = StateMachine().update(events)
  assert enabled is (block is None)


@pytest.mark.parametrize('action,expected', [('accelCruiseLong', 90), ('decelCruiseLong', 80)])
def test_remote_long_speed_action_is_one_native_step_without_held_state(action, expected):
  helper, CS, CC = make_remote_helper(action, enabled=True)
  assert helper._update_cruise_buttons(CS, CC, 83) == expected
  assert helper.button_cnt == 0
  assert not helper.long_pressed
  assert helper._activate_cruise == 0
  helper.bluetooth_commands = SimpleNamespace(read=lambda **_: None)
  assert helper._update_cruise_buttons(CS, CC, expected) == expected


@pytest.mark.parametrize('action', ['accelCruise', 'decelCruise', 'accelCruiseLong', 'decelCruiseLong'])
def test_remote_repeat_cannot_reengage_disabled_cruise(action):
  helper, CS, CC = make_remote_helper(action, enabled=False)
  helper.bluetooth_commands.is_repeat = True
  assert helper._update_cruise_buttons(CS, CC, 80) == 80
  assert helper._activate_cruise == 0
  assert not helper._lat_enabled


def test_remote_gap_long_cycles_driving_mode_without_changing_gap():
  helper, CS, CC = make_remote_helper('gapAdjustCruiseLong', enabled=True)
  values = {'MyDrivingMode': 4, 'LongitudinalPersonality': 2}
  helper.params = SimpleNamespace(get_int=values.__getitem__, put_int_nonblocking=values.__setitem__)
  assert helper._update_cruise_buttons(CS, CC, 80) == 80
  assert values == {'MyDrivingMode': 1, 'LongitudinalPersonality': 2}


def test_remote_lfa_short_and_long_follow_existing_distinct_handlers():
  helper, CS, CC = make_remote_helper('lfaButton', enabled=True)
  helper._update_cruise_buttons(CS, CC, 80)
  assert helper._lat_enabled
  helper.bluetooth_commands = SimpleNamespace(read=lambda **_: 'lfaButtonLong')
  helper._update_cruise_buttons(CS, CC, 80)
  assert helper.useLaneLineSpeedApply == 50
  assert helper._lat_enabled


@pytest.mark.parametrize('action,lat_enabled', [('cancel', True), ('cancelLong', False)])
def test_remote_cancel_actions_disengage_even_with_auto_cruise_disabled(action, lat_enabled):
  helper, CS, CC = make_remote_helper(action, enabled=True)
  helper._lat_enabled = True
  helper.autoCruiseControl = 0
  helper.autoCruiseControl_cancel_timer = 2000
  helper._update_cruise_buttons(CS, CC, 80)
  assert helper._activate_cruise == BLUETOOTH_CANCEL
  assert helper._lat_enabled is lat_enabled
  assert helper._cruise_cancel_state


def test_remote_set_keeps_soft_hold_cancel_behavior():
  helper, CS, CC = make_remote_helper('decelCruise')
  helper._soft_hold_active = 2
  helper._update_cruise_buttons(CS, CC, 80)
  assert helper._activate_cruise == -1


@pytest.mark.parametrize('pcm', [False, True])
def test_remote_cancel_reaches_existing_disengagement_state_machine(pcm):
  from openpilot.cereal import log
  from openpilot.selfdrive.car.car_specific import CarSpecificEvents
  from openpilot.selfdrive.selfdrived.state import StateMachine
  helper, CS, CC = make_remote_helper('cancelLong', enabled=True)
  helper._update_cruise_buttons(CS, CC, 80)
  CS.activateCruise = helper._activate_cruise
  CS.cruiseState.enabled = pcm
  previous = car.CarState(cruiseState={'enabled': pcm})
  car_events = CarSpecificEvents(SimpleNamespace(pcmCruise=pcm, openpilotLongitudinalControl=not pcm))
  events = car_events.create_common_events(CS, previous, pcm_enable=pcm)
  state = StateMachine()
  state.state = log.SelfdriveState.OpenpilotState.enabled
  enabled, _ = state.update(events)
  assert not enabled


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


@pytest.mark.parametrize("maximum,requested,expected", [
  (4, 4, 4), (3, 4, 3), (4, 2, 2), (3, 2, 2), (4, 3, 3),
  (3, 0, 3), (4, 0, 4), (0, 4, 3), (4, 99, 4), (3, 1, 2),
])
def test_gap_cycle_limits(maximum, requested, expected):
  assert cruise_gap_levels(requested, maximum) == expected


@pytest.mark.parametrize("maximum,requested,pcm_gap,expected", [
  (4, 2, 0, [1, 0, 1, 0]),
  (4, 2, 4, [1, 0, 1, 0]),
  (4, 3, 4, [2, 1, 0, 2]),
  (3, 2, 3, [1, 0, 1, 0]),
  (4, 4, 0, [3, 2, 1, 0]),
  (3, 4, 0, [2, 1, 0, 2]),
])
def test_gap_button_cycles_selected_levels(maximum, requested, pcm_gap, expected):
  helper, CS, CC = make_cruise_helper(80, 0, False, True)
  helper._prepare_buttons = lambda CS, speed, remote=None: (speed, ButtonType.gapAdjustCruise, False)
  helper.CP = SimpleNamespace(openpilotLongitudinalControl=True)
  values = {"LongitudinalPersonalityMax": maximum, "CruiseGapLevels": requested, "LongitudinalPersonality": 0}
  helper.params = SimpleNamespace(get_int=values.__getitem__, put_int_nonblocking=values.__setitem__)
  CS.pcmCruiseGap = pcm_gap
  for personality in expected:
    assert helper._update_cruise_buttons(CS, CC, 80) == 80
    assert values["LongitudinalPersonality"] == personality
  assert values["CruiseGapLevels"] == requested


@pytest.mark.parametrize("openpilot_long,requested,pcm_gap,current,expected", [
  (True, 2, 4, 3, 1),  # Reducing from TF4 enters TF2 on the next press.
  (True, 4, 3, 0, 2),  # Default preserves the vehicle-reported gap.
  (False, 2, 4, 0, 3),  # Stock ACC owns its gap cycle.
  (False, 2, 0, 0, 3),
  (True, 4, 9, 0, 3),  # Invalid OEM values cannot create an invalid enum.
])
def test_gap_button_reduction_and_oem_gap(openpilot_long, requested, pcm_gap, current, expected):
  helper, CS, CC = make_cruise_helper(80, 0, False, True)
  helper._prepare_buttons = lambda CS, speed, remote=None: (speed, ButtonType.gapAdjustCruise, False)
  helper.CP = SimpleNamespace(openpilotLongitudinalControl=openpilot_long)
  values = {"LongitudinalPersonalityMax": 4, "CruiseGapLevels": requested, "LongitudinalPersonality": current}
  helper.params = SimpleNamespace(get_int=values.__getitem__, put_int_nonblocking=values.__setitem__)
  CS.pcmCruiseGap = pcm_gap
  helper._update_cruise_buttons(CS, CC, 80)
  assert values["LongitudinalPersonality"] == expected


def test_gap_long_press_still_changes_driving_mode():
  helper, CS, CC = make_cruise_helper(80, 0, False, True)
  helper._prepare_buttons = lambda CS, speed, remote=None: (speed, ButtonType.gapAdjustCruise, True)
  values = {"MyDrivingMode": 4, "LongitudinalPersonality": 1}
  helper.params = SimpleNamespace(get_int=values.__getitem__, put_int_nonblocking=values.__setitem__)
  helper._update_cruise_buttons(CS, CC, 80)
  assert values == {"MyDrivingMode": 1, "LongitudinalPersonality": 1}
