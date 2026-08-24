from types import SimpleNamespace

import pytest

from opendbc.car.hyundai.carstate import CarState, VEHICLE_NAVI_SCHOOL_ZONE_MAX_DISTANCE, VEHICLE_SPEED_CAMERA_PARAM_UPDATE_FRAMES


class FakeParams:
  def __init__(self, value=60, vehicle_navi=False, school_zone=False):
    self.value = value
    self.vehicle_navi = vehicle_navi
    self.school_zone = school_zone
    self.read_count = 0

  def get_int(self, key):
    assert key == "VehicleSpeedCameraDistanceTime"
    self.read_count += 1
    return self.value

  def get_bool(self, key):
    if key == "VehicleNaviCanControl":
      return self.vehicle_navi
    assert key == "VehicleNaviSchoolZoneControl"
    return self.school_zone


def _car_state(distance_time_tenths=60):
  state = CarState.__new__(CarState)
  state.op_params = FakeParams(distance_time_tenths)
  state.totalDistance = 0.0
  state.speedLimitDistance = 0.0
  state.vehicleSpeedCameraDistanceTime = CarState._vehicle_speed_camera_distance_time(distance_time_tenths)
  state.vehicleNaviCanControl = False
  state.vehicleNaviSchoolZoneControl = False
  state.vehicleSpeedCameraParamsCounter = 0
  state.vehicleNaviEvents = []
  state.vehicleNaviSegmentTimestamp = 0
  state.vehicleNaviProfileTimestamp = 0
  state.vehicleNaviRouteResetTimestamp = 0
  state.vehicleNaviCameraTarget = None
  state.vehicleNaviSchoolZoneActive = False
  state.vehicleNaviSchoolZoneStartDistance = 0.0
  state.vehicleNaviSchoolZoneUsesCameraStatus = False
  state.navi_segment_4b9 = None
  state.navi_profile_4be = None
  return state


@pytest.mark.parametrize("gas_pressed", (False, True))
def test_vehicle_speed_camera_distance_is_independent_of_accelerator(gas_pressed):
  state = _car_state()
  ret = SimpleNamespace(vEgo=10.0, speedLimit=50.0, gasPressed=gas_pressed)

  state.update_speed_limit(ret, speed_limit_cam=True)

  assert ret.speedLimitDistance == pytest.approx(300.0)


def test_vehicle_speed_camera_distance_uses_tenths_of_a_second():
  state = _car_state(distance_time_tenths=62)
  ret = SimpleNamespace(vEgo=10.0, speedLimit=50.0, gasPressed=False)

  state.update_speed_limit(ret, speed_limit_cam=True)

  assert ret.speedLimitDistance == pytest.approx(310.0)


@pytest.mark.parametrize(("raw_value", "expected"), (
  (0, 1.0),
  (10, 1.0),
  (62, 6.2),
  (200, 20.0),
  (999, 20.0),
))
def test_vehicle_speed_camera_distance_time_is_clamped_and_scaled(raw_value, expected):
  assert CarState._vehicle_speed_camera_distance_time(raw_value) == pytest.approx(expected)


def test_vehicle_speed_camera_distance_time_reloads_every_second():
  state = _car_state()
  state.op_params.value = 62
  state.vehicleSpeedCameraParamsCounter = VEHICLE_SPEED_CAMERA_PARAM_UPDATE_FRAMES - 1
  ret = SimpleNamespace(vEgo=10.0, speedLimit=50.0, gasPressed=False)

  state.update_speed_limit(ret, speed_limit_cam=True)

  assert state.op_params.read_count == 1
  assert state.vehicleSpeedCameraDistanceTime == pytest.approx(6.2)
  assert ret.speedLimitDistance == pytest.approx(310.0)


def test_vehicle_speed_camera_params_are_not_read_every_frame():
  state = _car_state()
  ret = SimpleNamespace(vEgo=10.0, speedLimit=50.0, gasPressed=False)

  for _ in range(VEHICLE_SPEED_CAMERA_PARAM_UPDATE_FRAMES - 1):
    state.update_speed_limit(ret, speed_limit_cam=True)

  assert state.op_params.read_count == 0


def test_vehicle_navigation_toggles_reload_every_second_without_restart():
  state = _car_state()
  state.op_params.vehicle_navi = True
  state.op_params.school_zone = True
  state.vehicleSpeedCameraParamsCounter = VEHICLE_SPEED_CAMERA_PARAM_UPDATE_FRAMES - 1

  state._update_vehicle_speed_camera_params()

  assert state.vehicleNaviCanControl
  assert state.vehicleNaviSchoolZoneControl

  state.op_params.vehicle_navi = False
  state.op_params.school_zone = False
  state.vehicleNaviEvents = [{"type": "camera", "speed": 50, "kind": 1, "target": 100.0}]
  state.vehicleNaviSchoolZoneActive = True
  state.vehicleSpeedCameraParamsCounter = VEHICLE_SPEED_CAMERA_PARAM_UPDATE_FRAMES - 1

  state._update_vehicle_speed_camera_params()

  assert not state.vehicleNaviCanControl
  assert not state.vehicleNaviSchoolZoneControl
  assert state.vehicleNaviEvents == []
  assert not state.vehicleNaviSchoolZoneActive


@pytest.mark.parametrize(("speed_limit", "camera", "expected"), (
  (0.0, True, 0.0),
  (50.0, False, 0.0),
))
def test_vehicle_speed_camera_distance_requires_valid_camera_speed(speed_limit, camera, expected):
  state = _car_state()
  state.speedLimitDistance = 300.0
  ret = SimpleNamespace(vEgo=10.0, speedLimit=speed_limit, gasPressed=False)

  state.update_speed_limit(ret, speed_limit_cam=camera)

  assert ret.speedLimitDistance == pytest.approx(expected)


@pytest.mark.parametrize(("value", "expected"), (
  (0xB1, ("camera", 50, 1)),
  (0x71, ("camera", 30, 1)),
  (0xD1, ("camera", 60, 1)),
  (0xF2, ("camera", 70, 2)),
  (0x06, ("bump", 0, 6)),
  (0x07, None),
  (0xffffffff, None),
))
def test_vehicle_navi_profile_classification(value, expected):
  profile = {"profile_type": 16, "value": value, "offset": 890}
  assert CarState._classify_vehicle_navi_profile(profile) == expected


@pytest.mark.parametrize(("value", "expected"), (
  (0x77, ("speed_limit_zone", 30, 7)),
  (0xB7, ("speed_limit_zone", 50, 7)),
))
def test_vehicle_navi_speed_limit_zone_classification(value, expected):
  profile = {"profile_type": 16, "value": value, "offset": 0}
  assert CarState._classify_vehicle_navi_profile(profile) == expected


def test_vehicle_navi_profile_decodes_labeled_speed_bump_frame():
  raw = bytes.fromhex("060000007ae30804")
  value = int.from_bytes(raw, "little")
  profile = CarState._decode_vehicle_navi_profile({
    "PROLONG_VALUE": value & 0xffffffff,
    "PROLONG_OFFSET": (value >> 32) & 0x1fff,
    "PROLONG_CYCLIC_COUNTER": (value >> 45) & 0x3,
    "PROLONG_UPDATE": (value >> 47) & 0x1,
    "PROLONG_PROFILE_TYPE": (value >> 54) & 0x1f,
  })

  assert profile == {"value": 6, "offset": 890, "counter": 3, "update": 1, "profile_type": 16}
  assert CarState._classify_vehicle_navi_profile(profile) == ("bump", 0, 6)


def test_vehicle_navi_route_recalculation_clears_events():
  state = _car_state()
  state.vehicleNaviCanControl = True
  state.navi_profile_4be = {
    "PROLONG_VALUE": 6,
    "PROLONG_OFFSET": 890,
    "PROLONG_CYCLIC_COUNTER": 3,
    "PROLONG_UPDATE": 1,
    "PROLONG_PROFILE_TYPE": 16,
  }
  cp = SimpleNamespace(ts_nanos={"NEW_MSG_4BE": {"PROLONG_VALUE": 1}})
  ret = SimpleNamespace(speedLimit=0.0, speedBumpDistance=0.0, schoolZoneActive=False)

  assert not state._update_vehicle_navi_events(cp, ret, False)
  assert ret.speedBumpDistance == pytest.approx(890.0)

  raw = bytes.fromhex("0000b8063110fdff")
  state.navi_segment_4b9 = {f"BYTE_{i + 1}": byte for i, byte in enumerate(raw)}
  cp.ts_nanos["NEW_MSG_4B9"] = {"BYTE_1": 2}
  state._update_vehicle_navi_events(cp, ret, False)

  assert state.vehicleNaviEvents == []
  assert ret.speedBumpDistance == 0.0


def test_vehicle_navi_school_zone_follows_vehicle_camera_status():
  state = _car_state()
  state.vehicleNaviSchoolZoneControl = True
  state.navi_profile_4be = {
    "PROLONG_VALUE": 0x77,
    "PROLONG_OFFSET": 0,
    "PROLONG_CYCLIC_COUNTER": 3,
    "PROLONG_UPDATE": 1,
    "PROLONG_PROFILE_TYPE": 16,
  }
  cp = SimpleNamespace(ts_nanos={"NEW_MSG_4BE": {"PROLONG_VALUE": 1}})
  ret = SimpleNamespace(speedLimit=0.0, speedBumpDistance=0.0, schoolZoneActive=False)

  ret.speedLimit = 30.0
  assert not state._update_vehicle_navi_events(cp, ret, True)
  assert ret.schoolZoneActive
  assert ret.speedLimit == 30
  assert state.vehicleNaviSchoolZoneUsesCameraStatus

  # 00000d87--35927b43c4: 0x77 is followed by an invalid profile, not 0xB7.
  state.navi_profile_4be.update({
    "PROLONG_VALUE": 0xffffffff,
    "PROLONG_OFFSET": 8191,
    "PROLONG_PROFILE_TYPE": 31,
  })
  cp.ts_nanos["NEW_MSG_4BE"]["PROLONG_VALUE"] = 2
  ret.speedLimit = 30.0
  assert not state._update_vehicle_navi_events(cp, ret, True)
  assert ret.schoolZoneActive
  assert ret.speedLimit == 30

  ret.speedLimit = 0.0
  assert not state._update_vehicle_navi_events(cp, ret, False)
  assert not ret.schoolZoneActive
  assert not state.vehicleNaviSchoolZoneActive


def test_vehicle_navi_school_zone_explicit_speed_change_clears_cap():
  state = _car_state()
  state.vehicleNaviSchoolZoneControl = True
  state.vehicleNaviSchoolZoneActive = True
  state.navi_profile_4be = {
    "PROLONG_VALUE": 0xB7,
    "PROLONG_OFFSET": 0,
    "PROLONG_CYCLIC_COUNTER": 3,
    "PROLONG_UPDATE": 1,
    "PROLONG_PROFILE_TYPE": 16,
  }
  cp = SimpleNamespace(ts_nanos={"NEW_MSG_4BE": {"PROLONG_VALUE": 1}})
  ret = SimpleNamespace(speedLimit=50.0, speedBumpDistance=0.0, schoolZoneActive=False)

  assert not state._update_vehicle_navi_events(cp, ret, False)
  assert not ret.schoolZoneActive
  assert not state.vehicleNaviSchoolZoneActive


def test_vehicle_navi_route_recalculation_clears_school_zone():
  state = _car_state()
  state.vehicleNaviSchoolZoneControl = True
  state.vehicleNaviSchoolZoneActive = True
  state.vehicleNaviSchoolZoneStartDistance = 10.0
  raw = bytes.fromhex("0000b8063110fdff")
  state.navi_segment_4b9 = {f"BYTE_{i + 1}": byte for i, byte in enumerate(raw)}
  cp = SimpleNamespace(ts_nanos={"NEW_MSG_4B9": {"BYTE_1": 2}})
  ret = SimpleNamespace(speedLimit=30.0, speedBumpDistance=0.0, schoolZoneActive=False)

  assert not state._update_vehicle_navi_events(cp, ret, True)
  assert not ret.schoolZoneActive
  assert not state.vehicleNaviSchoolZoneActive


def test_vehicle_navi_school_zone_distance_fail_safe():
  state = _car_state()
  state.vehicleNaviSchoolZoneControl = True
  state.vehicleNaviSchoolZoneActive = True
  state.vehicleNaviSchoolZoneStartDistance = 10.0
  state.totalDistance = 10.0 + VEHICLE_NAVI_SCHOOL_ZONE_MAX_DISTANCE
  cp = SimpleNamespace(ts_nanos={})
  ret = SimpleNamespace(speedLimit=0.0, speedBumpDistance=0.0, schoolZoneActive=False)

  assert not state._update_vehicle_navi_events(cp, ret, False)
  assert not ret.schoolZoneActive
  assert not state.vehicleNaviSchoolZoneActive


def test_vehicle_navi_exact_camera_distance_replaces_virtual_distance():
  state = _car_state()
  state.vehicleNaviCanControl = True
  state.op_params.vehicle_navi = True
  state.vehicleNaviCameraTarget = 300.0
  ret = SimpleNamespace(vEgo=10.0, speedLimit=50.0, gasPressed=False)

  state.update_speed_limit(ret, speed_limit_cam=True)

  assert ret.speedLimitDistance == pytest.approx(300.0 - 10.0 * 0.01)
