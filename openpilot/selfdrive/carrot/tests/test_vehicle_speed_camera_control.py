from types import SimpleNamespace

import pytest

from openpilot.selfdrive.carrot.carrot_serv import CarrotServ


def _serv(mode):
  serv = CarrotServ.__new__(CarrotServ)
  serv.vehicleSpeedCameraControlMode = mode
  serv.gas_override_speed = 0
  serv.gas_pressed_state = False
  serv.source_last = "none"
  return serv


def _car_state(*, gas=False, brake=False, speed_limit=50, distance=300, v_ego=20):
  return SimpleNamespace(
    gasPressed=gas,
    brakePressed=brake,
    speedLimit=speed_limit,
    speedLimitDistance=distance,
    vEgo=v_ego,
  )


@pytest.mark.parametrize(("mode", "gas_pressed", "expected"), (
  (0, False, False),
  (0, True, False),
  (1, False, True),
  (1, True, True),
  (2, False, True),
  (2, True, True),
  (3, False, True),
  (3, True, False),
))
def test_vehicle_speed_camera_mode_controls_candidate(mode, gas_pressed, expected):
  serv = _serv(mode)
  assert serv._vehicle_speed_camera_enabled(_car_state(gas=gas_pressed)) is expected


def test_gas_floor_tracks_peak_speed_and_remains_after_release():
  serv = _serv(2)
  serv.source_last = "hda"
  CS = _car_state(gas=True)

  desired_speed, source = serv._apply_speed_source_gas_floor(CS, 60, "hda", 80, False)
  assert (desired_speed, source, serv.gas_override_speed) == (80, "gas", 80)

  desired_speed, source = serv._apply_speed_source_gas_floor(CS, 60, "hda", 85, False)
  assert (desired_speed, source, serv.gas_override_speed) == (85, "gas", 85)

  CS.gasPressed = False
  desired_speed, source = serv._apply_speed_source_gas_floor(CS, 60, "hda", 82, False)
  assert (desired_speed, source, serv.gas_override_speed) == (85, "gas", 85)


@pytest.mark.parametrize("mode", (0, 1, 3))
def test_non_floor_modes_do_not_raise_vehicle_camera_target(mode):
  serv = _serv(mode)
  serv.gas_override_speed = 90

  desired_speed, source = serv._apply_speed_source_gas_floor(
    _car_state(gas=True), 60, "hda", 80, False,
  )

  assert (desired_speed, source, serv.gas_override_speed) == (60, "hda", 0)


@pytest.mark.parametrize("mode", (0, 1, 2, 3))
def test_road_limit_keeps_accelerator_speed_floor_after_release(mode):
  serv = _serv(mode)
  serv.source_last = "road"
  CS = _car_state(gas=True)

  desired_speed, source = serv._apply_speed_source_gas_floor(CS, 52, "road", 79, False)
  assert (desired_speed, source, serv.gas_override_speed) == (79, "gas", 79)

  CS.gasPressed = False
  desired_speed, source = serv._apply_speed_source_gas_floor(CS, 52, "road", 79, False)
  assert (desired_speed, source, serv.gas_override_speed) == (79, "gas", 79)


def test_road_limit_change_resets_accelerator_speed_floor():
  serv = _serv(1)
  serv.source_last = "road"
  serv.gas_override_speed = 79

  desired_speed, source = serv._apply_speed_source_gas_floor(
    _car_state(), 52, "road", 79, True,
  )

  assert (desired_speed, source, serv.gas_override_speed) == (52, "road", 0)


@pytest.mark.parametrize("source", ("cam", "section", "police"))
def test_enforced_navigation_sources_reject_accelerator_speed_floor(source):
  serv = _serv(2)
  serv.source_last = source
  serv.gas_override_speed = 80

  desired_speed, returned_source = serv._apply_speed_source_gas_floor(
    _car_state(gas=True), 30, source, 80, False,
  )

  assert (desired_speed, returned_source, serv.gas_override_speed) == (30, source, 0)


@pytest.mark.parametrize(("brake_pressed", "road_limit_changed"), (
  (True, False),
  (False, True),
))
def test_gas_floor_resets_on_brake_or_camera_speed_change(brake_pressed, road_limit_changed):
  serv = _serv(2)
  serv.source_last = "hda"
  serv.gas_override_speed = 85

  desired_speed, source = serv._apply_speed_source_gas_floor(
    _car_state(brake=brake_pressed), 60, "hda", 80, road_limit_changed,
  )

  assert (desired_speed, source, serv.gas_override_speed) == (60, "hda", 0)
