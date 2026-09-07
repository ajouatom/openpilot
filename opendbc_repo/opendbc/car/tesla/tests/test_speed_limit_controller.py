from types import SimpleNamespace

from opendbc.car.tesla.speed_limit_controller import TeslaSpeedLimitController, create_speed_wheel_frame
from opendbc.car.tesla.values import TeslaFlags


IDLE_TEMPLATE = bytes.fromhex("2955000000000080")


def fake_state(current_speed=20.0, target_speed=25.0, template_time=2_000_000_000):
  return SimpleNamespace(
    out=SimpleNamespace(cruiseState=SimpleNamespace(enabled=True, speedCluster=current_speed), brakePressed=False),
    tesla_speed_limit_target=target_speed,
    tesla_speed_limit_target_nanos=template_time,
    tesla_speed_limit_target_valid=True,
    tesla_speed_button_template=IDLE_TEMPLATE,
    tesla_speed_button_template_nanos=template_time,
    tesla_speed_units="KPH",
    tesla_manual_speed_adjustment_counter=0,
    tesla_speed_auto_resume_gesture_counter=0,
  )


def fake_control():
  return SimpleNamespace(enabled=True, cruiseControl=SimpleNamespace(cancel=False))


def test_speed_wheel_frame_only_changes_signed_tick():
  assert create_speed_wheel_frame(IDLE_TEMPLATE, 1) == bytes.fromhex("2955000100000080")
  assert create_speed_wheel_frame(IDLE_TEMPLATE, -1) == bytes.fromhex("2955003f00000080")


def test_controller_waits_for_stable_target_and_feedback():
  controller = TeslaSpeedLimitController(SimpleNamespace(flags=TeslaFlags.AUTO_SPEED_LIMIT))
  state = fake_state()

  assert controller.update(fake_control(), state, 2_000_000_000) == []
  sends = controller.update(fake_control(), state, 2_500_000_000)
  assert len(sends) == 1
  assert sends[0].address == 0x3C2
  assert sends[0].src == 1
  assert sends[0].dat == bytes.fromhex("2955000100000080")
  state.tesla_speed_limit_target_nanos = 2_900_000_000
  assert controller.update(fake_control(), state, 2_900_000_000) == []

  state.out.cruiseState.speedCluster += 1.0 / 3.6
  state.tesla_speed_limit_target_nanos = 3_100_000_000
  assert len(controller.update(fake_control(), state, 3_100_000_000)) == 1


def test_manual_adjustment_pauses_until_opposite_direction_gesture():
  controller = TeslaSpeedLimitController(SimpleNamespace(flags=TeslaFlags.AUTO_SPEED_LIMIT))
  state = fake_state(current_speed=25.0, target_speed=25.0)
  assert controller.update(fake_control(), state, 2_000_000_000) == []
  assert controller.update(fake_control(), state, 2_500_000_000) == []

  state.out.cruiseState.speedCluster = 26.0
  state.tesla_manual_speed_adjustment_counter = 1
  state.tesla_speed_limit_target_nanos = 2_600_000_000
  assert controller.update(fake_control(), state, 2_600_000_000) == []
  assert controller.manual_override_active

  state.tesla_speed_limit_target_valid = False
  assert controller.update(fake_control(), state, 2_650_000_000) == []
  state.tesla_speed_limit_target_valid = True
  state.tesla_speed_limit_target = 24.0
  state.tesla_speed_limit_target_nanos = 2_675_000_000
  assert controller.update(fake_control(), state, 2_675_000_000) == []
  assert controller.manual_override_active

  state.tesla_manual_speed_adjustment_counter = 2
  state.tesla_speed_auto_resume_gesture_counter = 1
  state.tesla_speed_limit_target_nanos = 3_175_000_000
  assert len(controller.update(fake_control(), state, 3_175_000_000)) == 1
  assert not controller.manual_override_active


def test_controller_quantizes_in_vehicle_display_units():
  controller = TeslaSpeedLimitController(SimpleNamespace(flags=TeslaFlags.AUTO_SPEED_LIMIT))
  mph = 0.44704
  state = fake_state(current_speed=60.0 * mph, target_speed=60.4 * mph)
  state.tesla_speed_units = "MPH"
  assert controller.update(fake_control(), state, 2_000_000_000) == []
  assert controller.update(fake_control(), state, 2_500_000_000) == []

  state.tesla_speed_limit_target = 60.6 * mph
  state.tesla_speed_limit_target_nanos = 2_600_000_000
  assert controller.update(fake_control(), state, 2_600_000_000) == []
  assert len(controller.update(fake_control(), state, 3_100_000_000)) == 1
