import math
from types import SimpleNamespace

import pytest

from opendbc.can import CANParser
from opendbc.car.hyundai.carstate import (
  CarState, VEHICLE_NAVI_POSITION_TIMEOUT_NS, VEHICLE_NAVI_SCHOOL_ZONE_MAX_DISTANCE,
  VEHICLE_SPEED_CAMERA_PARAM_UPDATE_FRAMES,
)


class FakeParams:
  def __init__(self, value=60, vehicle_navi=False, school_zone=False):
    self.value = value
    self.vehicle_navi = vehicle_navi
    self.school_zone = school_zone
    self.read_count = 0

  def get_int(self, key):
    if key == "VehicleSpeedCameraDistanceTime":
      self.read_count += 1
      return self.value
    return {
      "VehicleNaviCurveSpeedFactor": 100,
      "AutoCurveSpeedLowerLimit": 30,
      "AutoNaviSpeedDecelRate": 120,
      "AutoNaviSpeedCtrlEnd": 7,
    }[key]

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
  state.vehicleNaviCurveSpeedFactor = 1.0
  state.vehicleNaviCurveLowerLimit = 30.0
  state.vehicleNaviCurveDecelRate = 1.2
  state.vehicleNaviCurveControlEnd = 7.0
  state.vehicleSpeedCameraParamsCounter = 0
  state.vehicleNaviEvents = []
  state.vehicleNaviCurves = []
  state.vehicleNaviSegmentTimestamp = 0
  state.vehicleNaviCurveTimestamp = 0
  state.vehicleNaviProfileTimestamp = 0
  state.vehicleNaviAvailable = False
  state.vehicleNaviRouteResetTimestamp = 0
  state.vehicleNaviCurveRouteActive = False
  state.vehicleNaviCameraTarget = None
  state.vehicleNaviSpeedZoneActive = False
  state.vehicleNaviSpeedZoneSpeed = 0.0
  state.vehicleNaviSchoolZoneActive = False
  state.vehicleNaviSchoolZoneStartDistance = 0.0
  state.vehicleNaviSchoolZoneUsesCameraStatus = False
  state.navi_segment_4b9 = None
  state.navi_position_4b4 = None
  state.navi_profile_4ba = None
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


def test_vehicle_navi_availability_only_requires_receiving_0x4be():
  state = _car_state()
  cp = SimpleNamespace(ts_nanos={"NEW_MSG_4BE": {"PROLONG_VALUE": 1}})
  ret = SimpleNamespace()

  assert not state._update_vehicle_navi_events(cp, ret, False)
  assert ret.vehicleNaviAvailable

  cp.ts_nanos["NEW_MSG_4BE"]["PROLONG_VALUE"] = 0
  assert not state._update_vehicle_navi_events(cp, ret, False)
  assert ret.vehicleNaviAvailable
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
  (0x150, ("camera", 100, 0)),
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
  (0x157, ("speed_limit_zone", 100, 7)),
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


def test_vehicle_navi_section_log_frames_hold_cap_until_camera_status_ends():
  state = _car_state()
  state.vehicleNaviCanControl = True
  state.navi_profile_4be = {
    "PROLONG_VALUE": 0x157,
    "PROLONG_OFFSET": 0,
    "PROLONG_CYCLIC_COUNTER": 3,
    "PROLONG_UPDATE": 1,
    "PROLONG_PROFILE_TYPE": 16,
  }
  cp = SimpleNamespace(ts_nanos={"NEW_MSG_4BE": {"PROLONG_VALUE": 1}})
  ret = SimpleNamespace(speedLimit=100.0, speedBumpDistance=0.0, schoolZoneActive=False)

  assert not state._update_vehicle_navi_events(cp, ret, True)
  assert state.vehicleNaviSpeedZoneActive
  assert ret.vehicleNaviActive
  assert ret.vehicleNaviSectionActive
  assert ret.vehicleNaviSpeed == 100

  # The valid burst is followed by 0xff; it must not drop the section latch.
  state.navi_profile_4be.update({
    "PROLONG_VALUE": 0xffffffff,
    "PROLONG_OFFSET": 8191,
    "PROLONG_PROFILE_TYPE": 31,
  })
  cp.ts_nanos["NEW_MSG_4BE"]["PROLONG_VALUE"] = 2
  assert not state._update_vehicle_navi_events(cp, ret, True)
  assert ret.vehicleNaviSectionActive

  assert not state._update_vehicle_navi_events(cp, ret, False)
  assert not state.vehicleNaviSpeedZoneActive
  assert not ret.vehicleNaviActive
  assert not ret.vehicleNaviSectionActive


def test_vehicle_navi_range_average_holds_section_until_zero():
  state = _car_state()
  state.vehicleNaviCanControl = True
  state.navi_position_4b4 = {"POS_RANGE_AVG_SPEED": 103}
  cp = SimpleNamespace(ts_nanos={"NEW_MSG_4B4": {"POS_RANGE_AVG_SPEED": 1_000_000_000}},
                       _last_update_nanos=1_100_000_000)
  ret = SimpleNamespace(speedLimit=100.0, speedBumpDistance=0.0, schoolZoneActive=False)

  assert not state._update_vehicle_navi_events(cp, ret, True)
  assert state.vehicleNaviSpeedZoneActive
  assert ret.vehicleNaviActive
  assert ret.vehicleNaviSectionActive
  assert ret.vehicleNaviSpeed == 100

  state.navi_position_4b4["POS_RANGE_AVG_SPEED"] = 0
  cp.ts_nanos["NEW_MSG_4B4"]["POS_RANGE_AVG_SPEED"] = 1_200_000_000
  cp._last_update_nanos = 1_200_000_000
  assert not state._update_vehicle_navi_events(cp, ret, True)
  assert not state.vehicleNaviSpeedZoneActive
  assert not ret.vehicleNaviActive
  assert not ret.vehicleNaviSectionActive


def test_vehicle_navi_range_average_dbc_decodes_logged_frame():
  parser = CANParser("hyundai_canfd_generated", [("NEW_MSG_4B4", math.nan)], 0)
  parser.update([1_000_000_000, [(0x4B4, bytes.fromhex("0d0085cbb18948a5"), 0)]])

  assert parser.vl["NEW_MSG_4B4"]["POS_OFFSET"] == 13
  assert parser.vl["NEW_MSG_4B4"]["POS_RANGE_AVG_SPEED"] == 92


@pytest.mark.parametrize(("raw_value", "expected"), (
  (511, 0.0),
  (512, 0.00001),
  (599, 0.00112),
  (656, 0.00260),
  (819, 0.01792),
  (1023, None),
))
def test_adasis_v2_curvature_decoder(raw_value, expected):
  decoded = CarState._decode_adasis_curvature(raw_value)
  if expected is None:
    assert decoded is None
  else:
    assert decoded == pytest.approx(expected)


def test_vehicle_navi_curve_dbc_decodes_logged_frame():
  parser = CANParser("hyundai_canfd_generated", [("NEW_MSG_4BA", math.nan)], 0)
  parser.update([1_000_000_000, [(0x4BA, bytes.fromhex("3901f902ae04206c"), 0)]])

  values = parser.vl["NEW_MSG_4BA"]
  assert values["PROSHORT_OFFSET"] == 313
  assert values["PROSHORT_DISTANCE"] == 5
  assert values["PROSHORT_VALUE_0"] == 599
  assert values["PROSHORT_VALUE_1"] == 0
  assert values["PROSHORT_PROFILE_TYPE"] == 1


def test_vehicle_navi_curve_profile_publishes_reference_speed_and_distance():
  state = _car_state()
  state.vehicleNaviCurveRouteActive = True
  state.navi_profile_4ba = {
    "PROSHORT_OFFSET": 313,
    "PROSHORT_VALUE_0": 599,
    "PROSHORT_PROFILE_TYPE": 1,
  }
  cp = SimpleNamespace(ts_nanos={"NEW_MSG_4BA": {"PROSHORT_VALUE_0": 1}})
  ret = SimpleNamespace(speedLimit=0.0, speedBumpDistance=0.0, schoolZoneActive=False)

  assert not state._update_vehicle_navi_events(cp, ret, False)
  assert ret.vehicleNaviCurveDistance == pytest.approx(313.0)
  assert ret.vehicleNaviCurveCurvature == pytest.approx(0.00112)
  assert ret.vehicleNaviCurveSpeed == pytest.approx(math.sqrt(1.9 / 0.00112) * 3.6)
  assert ret.vehicleNaviCurveRouteActive


def test_vehicle_navi_route_recalculation_clears_curve_profile():
  state = _car_state()
  state.vehicleNaviCurves = [{"target": 300.0, "curvature": 0.01, "speed": 50.0}]
  raw = bytes.fromhex("0000b8063110fdff")
  state.navi_segment_4b9 = {f"BYTE_{i + 1}": byte for i, byte in enumerate(raw)}
  cp = SimpleNamespace(ts_nanos={"NEW_MSG_4B9": {"BYTE_1": 2}})
  ret = SimpleNamespace(speedLimit=0.0, speedBumpDistance=0.0, schoolZoneActive=False)

  assert not state._update_vehicle_navi_events(cp, ret, False)
  assert state.vehicleNaviCurves == []
  assert ret.vehicleNaviCurveSpeed == 0.0


def test_vehicle_navi_curve_control_requires_calculated_route():
  state = _car_state()
  raw = (1 << 22).to_bytes(8, "little")
  state.navi_segment_4b9 = {f"BYTE_{i + 1}": byte for i, byte in enumerate(raw)}
  cp = SimpleNamespace(ts_nanos={"NEW_MSG_4B9": {"BYTE_1": 1}})
  ret = SimpleNamespace(speedLimit=0.0, speedBumpDistance=0.0, schoolZoneActive=False)

  assert not state._update_vehicle_navi_events(cp, ret, False)
  assert state.vehicleNaviCurveRouteActive
  assert ret.vehicleNaviCurveRouteActive

  state.navi_segment_4b9 = {f"BYTE_{i + 1}": 0 for i in range(8)}
  cp.ts_nanos["NEW_MSG_4B9"]["BYTE_1"] = 2
  assert not state._update_vehicle_navi_events(cp, ret, False)
  assert not state.vehicleNaviCurveRouteActive
  assert state.vehicleNaviCurves == []


def test_vehicle_navi_stale_range_average_releases_section():
  state = _car_state()
  state.vehicleNaviCanControl = True
  state.navi_position_4b4 = {"POS_RANGE_AVG_SPEED": 103}
  cp = SimpleNamespace(ts_nanos={"NEW_MSG_4B4": {"POS_RANGE_AVG_SPEED": 1_000_000_000}},
                       _last_update_nanos=1_000_000_000)
  ret = SimpleNamespace(speedLimit=100.0, speedBumpDistance=0.0, schoolZoneActive=False)

  assert not state._update_vehicle_navi_events(cp, ret, True)
  cp._last_update_nanos += VEHICLE_NAVI_POSITION_TIMEOUT_NS + 1
  assert not state._update_vehicle_navi_events(cp, ret, True)
  assert not state.vehicleNaviSpeedZoneActive
  assert not ret.vehicleNaviSectionActive


def test_vehicle_navi_range_average_does_not_keep_regular_cap_in_school_zone():
  state = _car_state()
  state.vehicleNaviCanControl = True
  state.vehicleNaviSpeedZoneActive = True
  state.vehicleNaviSpeedZoneSpeed = 100
  state.navi_position_4b4 = {"POS_RANGE_AVG_SPEED": 25}
  cp = SimpleNamespace(ts_nanos={"NEW_MSG_4B4": {"POS_RANGE_AVG_SPEED": 1_000_000_000}},
                       _last_update_nanos=1_000_000_000)
  ret = SimpleNamespace(speedLimit=30.0, speedBumpDistance=0.0, schoolZoneActive=False)

  assert not state._update_vehicle_navi_events(cp, ret, True)
  assert not state.vehicleNaviSpeedZoneActive
  assert not ret.vehicleNaviSectionActive


def test_vehicle_navi_section_end_camera_uses_logged_1997_meter_offset():
  state = _car_state()
  state.vehicleNaviCanControl = True
  state.navi_profile_4be = {
    "PROLONG_VALUE": 0x150,
    "PROLONG_OFFSET": 1997,
    "PROLONG_CYCLIC_COUNTER": 3,
    "PROLONG_UPDATE": 1,
    "PROLONG_PROFILE_TYPE": 16,
  }
  cp = SimpleNamespace(ts_nanos={"NEW_MSG_4BE": {"PROLONG_VALUE": 1}})
  ret = SimpleNamespace(speedLimit=100.0, speedBumpDistance=0.0, schoolZoneActive=False)

  assert state._update_vehicle_navi_events(cp, ret, True)
  assert ret.vehicleNaviActive
  assert ret.vehicleNaviSpeed == 100
  assert state.vehicleNaviCameraTarget == pytest.approx(1997.0)

  speed_ret = SimpleNamespace(vEgo=10.0, speedLimit=ret.speedLimit, gasPressed=False)
  state.update_speed_limit(speed_ret, speed_limit_cam=True)
  assert speed_ret.speedLimitDistance == pytest.approx(1997.0 - 10.0 * 0.01)
