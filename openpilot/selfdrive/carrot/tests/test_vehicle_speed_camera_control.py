from types import SimpleNamespace

import pytest

from openpilot.selfdrive.carrot.carrot_serv import CarrotServ


def _serv(mode):
  serv = CarrotServ.__new__(CarrotServ)
  serv.vehicleSpeedCameraControlMode = mode
  serv.vehicleNaviCanControl = True
  serv.vehicleNaviSchoolZoneControl = False
  serv.autoNaviSpeedSafetyFactor = 1.05
  serv.autoNaviSpeedBumpSpeed = 25
  serv.autoCurveSpeedLowerLimit = 30
  serv.autoNaviSpeedCtrlEnd = 6
  serv.autoNaviSpeedDecelRate = 2.0
  serv.gas_override_speed = 0
  serv.gas_pressed_state = False
  serv.speed_event_gas_pressed = False
  serv.source_last = "none"
  serv.school_zone_gas_override_started_at = None
  serv.school_zone_suppressed = False
  return serv


def _car_state(*, gas=False, brake=False, speed_limit=50, distance=300, v_ego=20):
  return SimpleNamespace(
    gasPressed=gas,
    brakePressed=brake,
    speedLimit=speed_limit,
    speedLimitDistance=distance,
    speedBumpDistance=0,
    schoolZoneActive=False,
    vehicleNaviActive=False,
    vehicleNaviSectionActive=False,
    vehicleNaviSpeed=0,
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


def test_gas_floor_tracks_override_peak_during_decel_and_remains_after_release():
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


def test_gas_floor_does_not_arm_before_camera_deceleration_starts():
  serv = _serv(2)
  serv.source_last = "hda"
  CS = _car_state(gas=True)

  desired_speed, source = serv._apply_speed_source_gas_floor(CS, 90, "hda", 80, False)
  assert (desired_speed, source, serv.gas_override_speed) == (90, "hda", 0)

  desired_speed, source = serv._apply_speed_source_gas_floor(CS, 60, "hda", 80, False)
  assert (desired_speed, source, serv.gas_override_speed) == (60, "hda", 0)

  CS.gasPressed = False
  desired_speed, source = serv._apply_speed_source_gas_floor(CS, 60, "hda", 80, False)
  assert (desired_speed, source, serv.gas_override_speed) == (60, "hda", 0)

  CS.gasPressed = True
  desired_speed, source = serv._apply_speed_source_gas_floor(CS, 60, "hda", 80, False)
  assert (desired_speed, source, serv.gas_override_speed) == (80, "gas", 80)


def test_gas_floor_can_latch_when_camera_deceleration_first_becomes_source():
  serv = _serv(2)

  desired_speed, source = serv._apply_speed_source_gas_floor(
    _car_state(gas=True), 60, "hda", 80, False,
  )

  assert (desired_speed, source, serv.gas_override_speed) == (80, "gas", 80)


@pytest.mark.parametrize("mode", (0, 1, 3))
def test_non_floor_modes_do_not_raise_vehicle_camera_target(mode):
  serv = _serv(mode)
  serv.gas_override_speed = 90

  desired_speed, source = serv._apply_speed_source_gas_floor(
    _car_state(gas=True), 60, "hda", 80, False,
  )

  assert (desired_speed, source, serv.gas_override_speed) == (60, "hda", 0)


def test_vehicle_navi_exact_distance_feeds_countdown():
  serv = _serv(1)
  serv.autoNaviCountDownMode = 1
  serv.xSpdType = -1
  serv.xSpdDist = 0
  CS = _car_state(distance=1997)
  CS.vehicleNaviActive = True

  assert serv._speed_countdown_distance(CS) == 1997


def test_vehicle_navi_bump_countdown_follows_countdown_mode():
  serv = _serv(1)
  serv.xSpdType = -1
  serv.xSpdDist = 0
  CS = _car_state(distance=0)
  CS.vehicleNaviActive = True
  CS.speedBumpDistance = 120

  serv.autoNaviCountDownMode = 1
  assert serv._speed_countdown_distance(CS) == 0
  serv.autoNaviCountDownMode = 2
  assert serv._speed_countdown_distance(CS) == 120


def test_countdown_idle_reset_rearms_same_second_for_next_camera():
  serv = _serv(1)
  serv.left_sec = 5
  serv.max_left_sec = 6
  serv.carrot_left_sec = 5
  serv.sdi_inform = True

  serv._update_countdown_alert(100, "none", 50)
  assert (serv.left_sec, serv.carrot_left_sec) == (100, 100)

  serv._update_countdown_alert(5, "hda", 50)
  assert (serv.left_sec, serv.carrot_left_sec) == (5, 5)


def test_countdown_distance_jump_publishes_idle_before_next_event():
  left_sec, distance, rearmed = CarrotServ._countdown_channel(85, 5, 1, 10)
  assert (left_sec, distance, rearmed) == (100, 85, True)

  left_sec, distance, rearmed = CarrotServ._countdown_channel(84, distance, left_sec, 10)
  assert (left_sec, distance, rearmed) == (7, 84, False)


def test_countdown_distance_jump_resets_to_new_time_before_announcement_window():
  left_sec, distance, rearmed = CarrotServ._countdown_channel(400, 100, 15, 10)
  assert (left_sec, distance, rearmed) == (39, 400, False)


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


@pytest.mark.parametrize(("mode", "gas_pressed", "expected"), (
  (0, False, False),
  (1, False, True),
  (1, True, True),
  (2, False, True),
  (2, True, True),
  (3, False, True),
  (3, True, False),
))
def test_school_control_follows_vehicle_camera_mode(mode, gas_pressed, expected):
  serv = _serv(mode)
  CS = _car_state()
  CS.schoolZoneActive = True
  CS.gasPressed = gas_pressed
  serv.vehicleNaviSchoolZoneControl = True

  assert serv._vehicle_school_zone_enabled(CS) is expected
  assert serv._vehicle_school_zone_speed(CS) == (30 if expected else 250)


def test_school_mode_two_uses_accelerator_speed_floor():
  serv = _serv(2)
  serv.vehicleNaviSchoolZoneControl = True
  serv.source_last = "school"
  CS = _car_state(gas=True)
  CS.schoolZoneActive = True

  desired_speed, source = serv._apply_speed_source_gas_floor(CS, 30, "school", 48, False)

  assert (desired_speed, source, serv.gas_override_speed) == (48, "gas", 48)


@pytest.mark.parametrize("mode", (0, 1, 2, 3))
def test_vehicle_bump_always_uses_accelerator_speed_floor(mode):
  serv = _serv(mode)
  serv.source_last = "hda_bump"

  desired_speed, source = serv._apply_speed_source_gas_floor(
    _car_state(gas=True), 22, "hda_bump", 35, False,
  )

  assert (desired_speed, source, serv.gas_override_speed) == (35, "gas", 35)


def test_vehicle_bump_override_requires_new_pedal_input_during_decel():
  serv = _serv(1)
  CS = _car_state(gas=True)

  serv._apply_speed_source_gas_floor(CS, 90, "cam", 34, False)
  assert serv._apply_speed_source_gas_floor(CS, 22, "hda_bump", 34, False)[:2] == (22, "hda_bump")

  CS.gasPressed = False
  assert serv._apply_speed_source_gas_floor(CS, 22, "hda_bump", 34, False)[:2] == (22, "hda_bump")

  CS.gasPressed = True
  assert serv._apply_speed_source_gas_floor(CS, 22, "hda_bump", 35, False)[:2] == (35, "gas")
  assert serv._apply_speed_source_gas_floor(CS, 22, "hda_bump", 40, False)[:2] == (40, "gas")


@pytest.mark.parametrize(("mode", "source"), ((2, "hda"), (1, "hda_bump"), (1, "bump")))
def test_speed_event_gas_floor_resets_when_event_source_ends(mode, source):
  serv = _serv(mode)
  serv.source_last = source
  CS = _car_state(gas=True)

  assert serv._apply_speed_source_gas_floor(CS, 22, source, 35, False)[:2] == (35, "gas")

  CS.gasPressed = False
  desired_speed, returned_source = serv._apply_speed_source_gas_floor(CS, 200, "road", 35, False)
  assert (desired_speed, returned_source, serv.gas_override_speed) == (200, "road", 0)


def test_legacy_bump_override_requires_new_pedal_input_during_decel():
  serv = _serv(1)
  serv.source_last = "bump"
  CS = _car_state(gas=True)

  assert serv._apply_speed_source_gas_floor(CS, 40, "bump", 35, False)[:2] == (40, "bump")
  assert serv._apply_speed_source_gas_floor(CS, 22, "bump", 35, False)[:2] == (22, "bump")

  CS.gasPressed = False
  serv._apply_speed_source_gas_floor(CS, 22, "bump", 35, False)
  CS.gasPressed = True
  assert serv._apply_speed_source_gas_floor(CS, 22, "bump", 35, False)[:2] == (35, "gas")
  assert serv._apply_speed_source_gas_floor(CS, 22, "bump", 40, False)[:2] == (40, "gas")


def test_school_gas_override_suppresses_zone_after_three_seconds(monkeypatch):
  serv = _serv(2)
  serv.vehicleNaviSchoolZoneControl = True
  serv.source_last = "school"
  CS = _car_state(gas=True)
  CS.schoolZoneActive = True
  now = [100.0]
  monkeypatch.setattr(CarrotServ._update_school_zone_gas_override.__globals__["time"], "monotonic", lambda: now[0])

  assert serv._apply_speed_source_gas_floor(CS, 30, "school", 48, False)[:2] == (48, "gas")
  now[0] += 2.9
  serv._apply_speed_source_gas_floor(CS, 30, "school", 48, False)
  assert serv._vehicle_school_zone_enabled(CS)

  now[0] += 0.2
  serv._apply_speed_source_gas_floor(CS, 30, "school", 48, False)
  assert not serv._vehicle_school_zone_enabled(CS)
  assert not serv._vehicle_speed_camera_enabled(CS)

  CS.schoolZoneActive = False
  assert not serv._vehicle_school_zone_enabled(CS)
  CS.schoolZoneActive = True
  assert serv._vehicle_school_zone_enabled(CS)


@pytest.mark.parametrize(("mode", "gas_pressed", "expected"), (
  (0, False, False),
  (1, False, True),
  (1, True, True),
  (2, True, True),
  (3, False, True),
  (3, True, False),
))
def test_vehicle_section_zone_holds_speed_cap(mode, gas_pressed, expected):
  serv = _serv(mode)
  CS = _car_state(gas=gas_pressed)
  CS.vehicleNaviActive = True
  CS.vehicleNaviSectionActive = True
  CS.vehicleNaviSpeed = 100

  assert serv._vehicle_section_zone_enabled(CS) is expected
  if expected:
    assert CS.vehicleNaviSpeed * serv.autoNaviSpeedSafetyFactor == pytest.approx(105.0)


def test_vehicle_navigation_display_is_independent_of_cruise_state():
  serv = _serv(1)
  CS = _car_state()
  CS.vehicleNaviActive = True
  CS.vehicleNaviSectionActive = True
  CS.vehicleNaviSpeed = 100

  assert serv._vehicle_navigation_display(CS) == (True, 105, True)


def test_vehicle_section_mode_two_uses_accelerator_speed_floor():
  serv = _serv(2)
  serv.source_last = "hda_section"
  CS = _car_state(gas=True)

  desired_speed, source = serv._apply_speed_source_gas_floor(CS, 105, "hda_section", 115, False)

  assert (desired_speed, source, serv.gas_override_speed) == (115, "gas", 115)


@pytest.mark.parametrize(("x_spd_type", "vehicle_camera", "vehicle_bump", "suppressed"), (
  (1, True, False, True),
  (1, False, True, False),
  (22, True, False, False),
  (22, False, True, True),
  (22, True, True, True),
  (100, True, False, False),
  (101, True, False, False),
  (1, False, False, False),
))
def test_vehicle_can_source_suppresses_only_same_legacy_sdi_type(x_spd_type, vehicle_camera, vehicle_bump, suppressed):
  assert CarrotServ._legacy_sdi_suppressed(x_spd_type, vehicle_camera, vehicle_bump) is suppressed


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
