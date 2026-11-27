import math
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import pytest

from opendbc.can import CANParser
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.hyundai.carstate import (
  CANFD_HDA_INFO_MSG, CANFD_NAVI_PROFILE_MSG, CANFD_NAVI_STATUS_MSG, CarState,
  VEHICLE_NAVI_POSITION_TIMEOUT_NS, VEHICLE_NAVI_ROUTE_TIMEOUT_NS, VEHICLE_NAVI_SCHOOL_ZONE_MAX_DISTANCE,
  VEHICLE_SPEED_CAMERA_PARAM_UPDATE_FRAMES, is_canfd_navi_camera_active,
)
from opendbc.car.hyundai.navi_speedcam import SpeedcamPolicy
from opendbc.car.hyundai.section_average import SectionAverage
from opendbc.car.hyundai.values import CAR, Buttons, HyundaiFlags

ButtonType = structs.CarState.ButtonEvent.Type


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
    if key == "VehicleNaviCanControl":
      return int(self.vehicle_navi)
    if key == "AutoNaviSpeedCtrlMode":
      return 2
    if key == "AutoNaviSpeedSafetyFactor":
      return 107
    if key == "CruiseButtonLongDelay":
      return 40
    if key in ("AutoNaviSpeedBumpSpeed", "AutoNaviSpeedBumpTime", "AutoNaviSpeedBumpEndDistance", "AutoNaviSpeedDecelRate"):
      return {"AutoNaviSpeedBumpSpeed": 35, "AutoNaviSpeedBumpTime": 1,
              "AutoNaviSpeedBumpEndDistance": 200, "AutoNaviSpeedDecelRate": 120}[key]
    raise KeyError(key)

  def get_bool(self, key):
    if key in ("VehicleNaviDecelCancel", "VehicleNaviSectionAvgControl"):
      return False
    if key in ("VehicleNaviDecelCancelBox", "VehicleNaviDecelCancelMobileZone", "VehicleNaviDecelCancelBump",
               "VehicleNaviDecelCancelUnknown",
               "VehicleNaviDecelCancelEarlyWarning"):
      return True
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
  state.vehicleNaviProfile4b8Timestamp = 0
  state.vehicleNaviAvailable = False
  state.vehicleNaviRouteResetTimestamp = 0
  state.vehicleNaviRouteState = 0
  state.vehicleNaviRoutePathIndex = None
  state.vehicleNaviRoadClass = 7
  state.vehicleNaviCameraTarget = None
  state.vehicleNaviCameraStatusEvent = None
  state.vehicleNaviCameraStatusSpeed = 0.0
  state.vehicleNaviCameraStatusTarget = None
  state.vehicleNaviSpeedZoneActive = False
  state.vehicleNaviSpeedZoneSpeed = 0.0
  state.vehicleNaviSchoolZoneActive = False
  state.vehicleNaviSchoolZoneStartDistance = 0.0
  state.vehicleNaviSchoolZoneUsesCameraStatus = False
  state.vehicleNaviZoneControlSupported = True
  state.canfd_wrapped_navi = False
  state.canfd_ev4_navi = False
  state.navi_segment_4b5 = None
  state.navi_position_4de = None
  state.vehicleNaviSegment4b5Timestamp = 0
  state.vehicleNaviSegment4b5Active = False
  state.vehicleNaviPosition4deSeen = False
  state.vehicleNaviSegment4b5Seen = False
  state.vehicleNaviSegment4b9Seen = False
  state.vehicleNaviGv80HorizonTimestamp = 0
  state.vehicleNaviRoadHorizon = []
  state.vehicleNaviGv80SteerHistory = deque(maxlen=300)
  state.vehicleNaviGv80SectionWarningSpeed = 0.0
  state.vehicleNaviGv80SectionWarningStartDistance = 0.0
  state.pv5_section_start_prev = False
  state.pv5_completed_camera_speed = 0
  state.is_metric = True
  state.navi_profile_msg = "NEW_MSG_4BE"
  state.hda_info_msg = "HDA_INFO_4A3"
  state.navi_segment_msg = "NEW_MSG_4B9"
  state.navi_segment_4b9 = None
  state.navi_position_4b4 = None
  state.navi_profile_4be = None
  state.navi_profile_4b8 = None
  state.hda_info_4a3 = None
  state.speedcam_policy = SpeedcamPolicy()
  state.speedcam_accel_swallow = False
  state.speedcam_engaged = False
  state.speedcam_gas_count = 0
  state.speedcam_gas_tok_consumed = False
  state.speedcam_input_consumed = False
  state.speedcam_v_cruise = 0.0
  state.speedcam_decel_count = 0
  state.speedcam_decel_hold = False
  state.speedcam_decel_long_fired = False
  state.speedcam_time = 0.0
  state.section_average = SectionAverage()
  state._read_speedcam_params()
  return state


@pytest.mark.parametrize("gas_pressed", (False, True))
def test_vehicle_speed_camera_distance_is_independent_of_accelerator(gas_pressed):
  state = _car_state()
  ret = SimpleNamespace(vEgo=10.0, speedLimit=50.0, gasPressed=gas_pressed)

  state.update_speed_limit(ret, speed_limit_cam=True)

  assert ret.speedLimitDistance == pytest.approx(300.0)


@pytest.mark.parametrize("road_class", (1, 2))
def test_vehicle_30_kph_camera_control_is_blocked_on_controlled_access_road(road_class):
  state = _car_state()
  state.vehicleNaviRoadClass = road_class
  ret = SimpleNamespace(vEgo=20.0, speedLimit=30.0, gasPressed=False)

  state.update_speed_limit(ret, speed_limit_cam=True)

  assert ret.speedLimit == 30.0
  assert ret.speedLimitDistance == 0.0


def test_vehicle_regular_camera_control_remains_active_on_expressway():
  state = _car_state()
  state.vehicleNaviRoadClass = 2
  ret = SimpleNamespace(vEgo=20.0, speedLimit=60.0, gasPressed=False)

  state.update_speed_limit(ret, speed_limit_cam=True)

  assert ret.speedLimitDistance == pytest.approx(360.0)


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
  cp = SimpleNamespace(dat={}, ts_nanos={"NEW_MSG_4BE": {"PROLONG_VALUE": 1}})
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
    "PROLONG_PATH_INDEX": (value >> 48) & 0x3f,
    "PROLONG_PROFILE_TYPE": (value >> 54) & 0x1f,
  })

  assert profile == {"value": 6, "offset": 890, "counter": 3, "update": 1, "path_index": 8, "profile_type": 16}
  assert CarState._classify_vehicle_navi_profile(profile) == ("bump", 0, 6)


def test_vehicle_navi_segment_decodes_functional_road_class():
  raw = (1 << 24) | (1 << 22) | 123
  segment = CarState._decode_vehicle_navi_segment({
    f"BYTE_{i + 1}": byte for i, byte in enumerate(raw.to_bytes(8, "little"))
  })

  assert segment == {"offset": 123, "path_index": 0, "calculated_route": 1, "functional_road_class": 1}


# 00000f07--e42270f2b5--2 reported the false bump as route state 0 with
# segment/profile path 8. Modes 2 and 3 must reject that combination.
@pytest.mark.parametrize(("mode", "value", "route_state", "segment_path", "profile_path", "expected"), (
  (1, 6, 0, 8, 8, True),
  (2, 0xB1, 0, 8, 8, True),
  (2, 6, 0, 8, 8, False),
  (2, 6, 1, 8, 8, True),
  (2, 6, 1, 8, 9, False),
  (3, 0xB1, 0, 8, 8, False),
  (3, 0xB1, 1, 8, 8, True),
  (3, 6, 1, 8, 8, True),
))
def test_vehicle_navi_control_mode_filters_profiles_by_calculated_route(
    mode, value, route_state, segment_path, profile_path, expected,
):
  state = _car_state()
  state.vehicleNaviCanControl = mode
  segment_raw = (segment_path << 13) | (route_state << 22) | (6 << 24)
  state.navi_segment_4b9 = {
    f"BYTE_{i + 1}": byte for i, byte in enumerate(segment_raw.to_bytes(8, "little"))
  }
  state.navi_profile_4be = {
    "PROLONG_VALUE": value,
    "PROLONG_OFFSET": 300,
    "PROLONG_CYCLIC_COUNTER": 3,
    "PROLONG_UPDATE": 1,
    "PROLONG_PATH_INDEX": profile_path,
    "PROLONG_PROFILE_TYPE": 16,
  }
  cp = SimpleNamespace(dat={}, ts_nanos={
    "NEW_MSG_4B9": {"BYTE_1": 1},
    "NEW_MSG_4BE": {"PROLONG_VALUE": 2},
  })
  ret = SimpleNamespace(speedLimit=0.0, speedBumpDistance=0.0, schoolZoneActive=False)

  state._update_vehicle_navi_events(cp, ret, False)

  assert bool(state.vehicleNaviEvents) is expected
  if value == 6:
    assert (ret.speedBumpDistance > 0) is expected
  else:
    assert (ret.vehicleNaviSpeed > 0) is expected


@pytest.mark.parametrize(("mode", "remaining_types"), (
  (2, {"camera"}),
  (3, set()),
))
def test_vehicle_navi_leaving_calculated_route_clears_route_filtered_events(mode, remaining_types):
  state = _car_state()
  state.vehicleNaviCanControl = mode
  state.vehicleNaviRouteState = 1
  state.vehicleNaviRoutePathIndex = 8
  state.vehicleNaviEvents = [
    {"type": "camera", "speed": 50, "kind": 1, "target": 300.0},
    {"type": "bump", "speed": 0, "kind": 6, "target": 200.0},
  ]
  segment_raw = (8 << 13) | (0 << 22) | (6 << 24)
  state.navi_segment_4b9 = {
    f"BYTE_{i + 1}": byte for i, byte in enumerate(segment_raw.to_bytes(8, "little"))
  }
  cp = SimpleNamespace(dat={}, ts_nanos={"NEW_MSG_4B9": {"BYTE_1": 1}})
  ret = SimpleNamespace(speedLimit=0.0, speedBumpDistance=0.0, schoolZoneActive=False)

  state._update_vehicle_navi_events(cp, ret, False)

  assert {event["type"] for event in state.vehicleNaviEvents} == remaining_types


def test_vehicle_navi_stale_calculated_route_rejects_route_filtered_bump():
  state = _car_state()
  state.vehicleNaviCanControl = 2
  state.vehicleNaviRouteState = 1
  state.vehicleNaviRoutePathIndex = 8
  state.vehicleNaviSegmentTimestamp = 1_000_000_000
  state.vehicleNaviEvents = [{"type": "bump", "speed": 0, "kind": 6, "target": 200.0}]
  cp = SimpleNamespace(dat={}, ts_nanos={}, _last_update_nanos=1_000_000_000 + VEHICLE_NAVI_ROUTE_TIMEOUT_NS + 1)
  ret = SimpleNamespace(speedLimit=0.0, speedBumpDistance=0.0, schoolZoneActive=False)

  state._update_vehicle_navi_events(cp, ret, False)

  assert state.vehicleNaviRouteState == 0
  assert state.vehicleNaviRoutePathIndex is None
  assert state.vehicleNaviEvents == []
  assert ret.speedBumpDistance == 0


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
  cp = SimpleNamespace(dat={}, ts_nanos={"NEW_MSG_4BE": {"PROLONG_VALUE": 1}})
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
  cp = SimpleNamespace(dat={}, ts_nanos={"NEW_MSG_4BE": {"PROLONG_VALUE": 1}})
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


def test_vehicle_navi_unconfirmed_30_zone_does_not_activate_school_zone():
  state = _car_state()
  state.vehicleNaviSchoolZoneControl = True
  state.navi_profile_4be = {
    "PROLONG_VALUE": 0x77,
    "PROLONG_OFFSET": 0,
    "PROLONG_CYCLIC_COUNTER": 3,
    "PROLONG_UPDATE": 1,
    "PROLONG_PROFILE_TYPE": 16,
  }
  cp = SimpleNamespace(dat={}, ts_nanos={"NEW_MSG_4BE": {"PROLONG_VALUE": 1}})
  ret = SimpleNamespace(speedLimit=0.0, speedBumpDistance=0.0, schoolZoneActive=False)

  # 00000166--a0a89dc8d2--7: 0x77 occurred on a non-school road while
  # 0x4A3 remained at speed 0 / MapSource 1 for the entire segment.
  assert not state._update_vehicle_navi_events(cp, ret, False)
  assert not ret.schoolZoneActive
  assert not state.vehicleNaviSchoolZoneActive


@pytest.mark.parametrize("link_class", (1, 2, 3))
def test_vehicle_navi_school_zone_is_blocked_by_controlled_access_link_class(link_class):
  state = _car_state()
  state.vehicleNaviSchoolZoneControl = True
  state.hda_info_4a3 = {"LinkClass": link_class}
  state.navi_profile_4be = {
    "PROLONG_VALUE": 0x77,
    "PROLONG_OFFSET": 0,
    "PROLONG_CYCLIC_COUNTER": 3,
    "PROLONG_UPDATE": 1,
    "PROLONG_PROFILE_TYPE": 16,
  }
  cp = SimpleNamespace(dat={}, ts_nanos={"NEW_MSG_4BE": {"PROLONG_VALUE": 1}})
  ret = SimpleNamespace(speedLimit=30.0, speedBumpDistance=0.0, schoolZoneActive=False)

  assert not state._update_vehicle_navi_events(cp, ret, True)
  assert not ret.schoolZoneActive
  assert not state.vehicleNaviSchoolZoneActive


@pytest.mark.parametrize("road_class", (1, 2))
def test_vehicle_navi_school_zone_is_cleared_by_controlled_access_road_class(road_class):
  state = _car_state()
  state.vehicleNaviSchoolZoneControl = True
  state.vehicleNaviSchoolZoneActive = True
  raw = road_class << 24
  state.navi_segment_4b9 = {f"BYTE_{i + 1}": byte for i, byte in enumerate(raw.to_bytes(8, "little"))}
  cp = SimpleNamespace(dat={}, ts_nanos={"NEW_MSG_4B9": {"BYTE_1": 1}})
  ret = SimpleNamespace(speedLimit=30.0, speedBumpDistance=0.0, schoolZoneActive=False)

  assert not state._update_vehicle_navi_events(cp, ret, True)
  assert state.vehicleNaviRoadClass == road_class
  assert not ret.schoolZoneActive
  assert not state.vehicleNaviSchoolZoneActive


@pytest.mark.parametrize("road_class", (1, 2))
def test_vehicle_navi_speed_bump_is_blocked_on_controlled_access_road(road_class):
  state = _car_state()
  state.vehicleNaviCanControl = True
  state.vehicleNaviRoadClass = road_class
  state.navi_profile_4be = {
    "PROLONG_VALUE": 6,
    "PROLONG_OFFSET": 300,
    "PROLONG_CYCLIC_COUNTER": 3,
    "PROLONG_UPDATE": 1,
    "PROLONG_PROFILE_TYPE": 16,
  }
  cp = SimpleNamespace(dat={}, ts_nanos={"NEW_MSG_4BE": {"PROLONG_VALUE": 1}})
  ret = SimpleNamespace(speedLimit=100.0, speedBumpDistance=0.0, schoolZoneActive=False)

  assert not state._update_vehicle_navi_events(cp, ret, False)
  assert ret.speedBumpDistance == 0.0
  assert state.vehicleNaviEvents == []


def test_vehicle_navi_pending_speed_bump_is_cleared_on_expressway_entry():
  state = _car_state()
  state.vehicleNaviCanControl = True
  state.vehicleNaviEvents = [{"type": "bump", "speed": 0, "kind": 6, "target": 300.0}]
  state.vehicleNaviRoadClass = 2
  cp = SimpleNamespace(dat={}, ts_nanos={})
  ret = SimpleNamespace(speedLimit=100.0, speedBumpDistance=0.0, schoolZoneActive=False)

  assert not state._update_vehicle_navi_events(cp, ret, False)
  assert ret.speedBumpDistance == 0.0
  assert state.vehicleNaviEvents == []


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
  cp = SimpleNamespace(dat={}, ts_nanos={"NEW_MSG_4BE": {"PROLONG_VALUE": 1}})
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
  cp = SimpleNamespace(dat={}, ts_nanos={"NEW_MSG_4B9": {"BYTE_1": 2}})
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
  cp = SimpleNamespace(dat={}, ts_nanos={})
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


def test_vehicle_navi_preview_remains_available_before_camera_status():
  state = _car_state()
  state.vehicleNaviCanControl = True
  camera = {"type": "camera", "speed": 50, "kind": 1, "target": 500.0}
  state.vehicleNaviEvents = [camera]
  cp = SimpleNamespace(dat={}, ts_nanos={})
  ret = SimpleNamespace(speedLimit=0.0, speedBumpDistance=0.0, schoolZoneActive=False)

  assert state._update_vehicle_navi_events(cp, ret, False)
  assert state.vehicleNaviCameraTarget == pytest.approx(500.0)
  assert ret.speedLimit == 50


def test_vehicle_navi_camera_status_end_retires_confirmed_event_immediately():
  state = _car_state()
  state.vehicleNaviCanControl = True
  current_camera = {"type": "camera", "speed": 60, "kind": 1, "target": 40.0}
  next_camera = {"type": "camera", "speed": 50, "kind": 1, "target": 510.0}
  state.vehicleNaviEvents = [current_camera, next_camera]
  cp = SimpleNamespace(dat={}, ts_nanos={})
  ret = SimpleNamespace(speedLimit=60.0, speedBumpDistance=0.0, schoolZoneActive=False)

  assert state._update_vehicle_navi_events(cp, ret, True)
  assert state.vehicleNaviCameraStatusEvent is current_camera
  assert state.vehicleNaviCameraTarget == pytest.approx(40.0)

  # The vehicle camera status ends with 40 m still left in the 0x4BE offset.
  # Retire only the confirmed camera; keep the next preview for early braking.
  ret.speedLimit = 0.0
  assert state._update_vehicle_navi_events(cp, ret, False)
  assert current_camera not in state.vehicleNaviEvents
  assert next_camera in state.vehicleNaviEvents
  assert state.vehicleNaviCameraStatusEvent is None
  assert state.vehicleNaviCameraTarget == pytest.approx(510.0)
  assert ret.speedLimit == 50


def test_vehicle_navi_camera_status_does_not_select_mismatched_future_camera():
  state = _car_state()
  state.vehicleNaviCanControl = True
  state.vehicleNaviEvents = [{"type": "camera", "speed": 50, "kind": 1, "target": 500.0}]
  cp = SimpleNamespace(dat={}, ts_nanos={})
  ret = SimpleNamespace(speedLimit=60.0, speedBumpDistance=0.0, schoolZoneActive=False)

  assert not state._update_vehicle_navi_events(cp, ret, True)
  assert state.vehicleNaviCameraTarget is None
  assert ret.speedLimit == 60.0


@pytest.mark.parametrize("future_distance", (1138.0, 1027.0, 1702.0))
def test_vehicle_navi_current_warning_rejects_distant_same_speed_preview(future_distance):
  state = _car_state(75)
  state.vehicleNaviCanControl = True
  camera = {"type": "camera", "speed": 50, "kind": 1, "target": future_distance}
  state.vehicleNaviEvents = [camera]
  cp = SimpleNamespace(dat={}, ts_nanos={})
  ret = SimpleNamespace(speedLimit=0.0, vEgo=0.0)

  # A previously selected preview must not leak into the current warning.
  assert state._update_vehicle_navi_events(cp, ret, False)
  state.update_speed_limit(ret, True, distance_time_changed=False)
  assert ret.speedLimitDistance == future_distance
  ret.speedLimit = 50.0
  assert not state._update_vehicle_navi_events(cp, ret, True)
  state.update_speed_limit(ret, True, distance_time_changed=False)
  assert state.vehicleNaviCameraStatusEvent is None
  assert ret.speedLimitDistance == 375.0

  # Travel does not move the association window toward a future camera.
  state.totalDistance = future_distance - 100.0
  assert not state._update_vehicle_navi_events(cp, ret, True)
  state.update_speed_limit(ret, True, distance_time_changed=False)
  assert ret.speedLimitDistance == 1.0

  # Ending an unmatched warning preserves the future preview.
  ret.speedLimit = 0.0
  assert state._update_vehicle_navi_events(cp, ret, False)
  state.update_speed_limit(ret, True, distance_time_changed=False)
  assert state.vehicleNaviCameraStatusEvent is None
  assert camera in state.vehicleNaviEvents
  assert ret.speedLimitDistance == 100.0


def test_vehicle_navi_late_profile_cannot_extend_current_warning_distance():
  state = _car_state(75)
  state.vehicleNaviCanControl = True
  cp = SimpleNamespace(dat={}, ts_nanos={})
  ret = SimpleNamespace(speedLimit=50.0, vEgo=0.0)
  assert not state._update_vehicle_navi_events(cp, ret, True)
  state.update_speed_limit(ret, True, distance_time_changed=False)
  state.totalDistance = 50.0
  state._add_vehicle_navi_event("camera", 50, 1, 1996.0)
  assert not state._update_vehicle_navi_events(cp, ret, True)
  state.update_speed_limit(ret, True, distance_time_changed=False)
  assert ret.speedLimitDistance == 325.0

  # A delayed nearby profile can still replace the estimate (including the
  # known 30-40 m offset beyond the physical camera).
  state._add_vehicle_navi_event("camera", 50, 1, 355.0)
  assert state._update_vehicle_navi_events(cp, ret, True)
  state.update_speed_limit(ret, True, distance_time_changed=False)
  assert ret.speedLimitDistance == 355.0


def test_vehicle_navi_warning_keeps_cap_after_profile_endpoint():
  state = _car_state()
  state.vehicleNaviCanControl = True
  state._add_vehicle_navi_event("camera", 50, 1, 20.0)
  cp = SimpleNamespace(dat={}, ts_nanos={})
  ret = SimpleNamespace(speedLimit=50.0, vEgo=0.0)
  assert state._update_vehicle_navi_events(cp, ret, True)
  state.totalDistance = 60.0
  assert state._update_vehicle_navi_events(cp, ret, True)
  state.update_speed_limit(ret, True, distance_time_changed=False)
  assert ret.speedLimitDistance == 1.0
  ret.speedLimit = 0.0
  assert not state._update_vehicle_navi_events(cp, ret, False)
  state.update_speed_limit(ret, False, distance_time_changed=False)
  assert ret.speedLimitDistance == 0.0


def test_vehicle_navi_warning_speed_change_and_restart_reset_fallback():
  state = _car_state(75)
  cp = SimpleNamespace(dat={}, ts_nanos={})
  ret = SimpleNamespace(speedLimit=50.0, vEgo=0.0)
  state._update_vehicle_navi_events(cp, ret, True)
  state.totalDistance = 100.0
  ret.speedLimit = 30.0
  state._update_vehicle_navi_events(cp, ret, True)
  state.update_speed_limit(ret, True, distance_time_changed=False)
  assert ret.speedLimitDistance == 225.0
  ret.speedLimit = 0.0
  state._update_vehicle_navi_events(cp, ret, False)
  ret.speedLimit = 30.0
  state.totalDistance = 200.0
  state._update_vehicle_navi_events(cp, ret, True)
  state.update_speed_limit(ret, True, distance_time_changed=False)
  assert ret.speedLimitDistance == 225.0


def test_vehicle_navi_live_distance_setting_updates_current_warning():
  state = _car_state(75)
  ret = SimpleNamespace(speedLimit=50.0, vEgo=0.0)
  state._update_vehicle_navi_events(SimpleNamespace(dat={}, ts_nanos={}), ret, True)
  state.totalDistance = 100.0
  state.op_params.value = 62
  state.vehicleSpeedCameraParamsCounter = VEHICLE_SPEED_CAMERA_PARAM_UPDATE_FRAMES - 1
  state.update_speed_limit(ret, True)
  assert ret.speedLimitDistance == 310.0


def test_vehicle_navi_section_log_frames_hold_cap_until_camera_status_ends():
  state = _car_state()
  state.vehicleNaviCanControl = True
  state.speedcamSectionAvgControl = True
  state.navi_profile_4be = {
    "PROLONG_VALUE": 0x157,
    "PROLONG_OFFSET": 0,
    "PROLONG_CYCLIC_COUNTER": 3,
    "PROLONG_UPDATE": 1,
    "PROLONG_PROFILE_TYPE": 16,
  }
  cp = SimpleNamespace(dat={}, ts_nanos={"NEW_MSG_4BE": {"PROLONG_VALUE": 1}})
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
  state.speedcamSectionAvgControl = True
  state.navi_position_4b4 = {"POS_RANGE_AVG_SPEED": 103}
  cp = SimpleNamespace(dat={}, ts_nanos={"NEW_MSG_4B4": {"POS_RANGE_AVG_SPEED": 1_000_000_000}},
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


def test_pv5_canfd_navi_dbc_decodes_logged_frames():
  parser = CANParser("hyundai_canfd_generated", [(CANFD_HDA_INFO_MSG, math.nan),
                                                   (CANFD_NAVI_PROFILE_MSG, math.nan)], 0)
  parser.update([1_000_000_000, [
    (0x364, bytes.fromhex("4fe828000000000040329a1100000100"), 0),
    (0x093, bytes.fromhex("5e0f24ffffffffffb0000000cce70814ffffffffffffffff"), 0),
  ]])

  hda_info = parser.vl[CANFD_HDA_INFO_MSG]
  assert hda_info["SPEED_LIMIT"] == 50
  assert hda_info["CountryCode"] == 410
  assert hda_info["MapSource"] == 2

  profile = CarState._decode_vehicle_navi_profile(parser.vl[CANFD_NAVI_PROFILE_MSG])
  assert profile == {"value": 0xB0, "offset": 1996, "counter": 3, "update": 1, "path_index": 8, "profile_type": 16}
  assert CarState._classify_vehicle_navi_profile(profile) == ("camera", 50, 0)

  parser.update([1_010_000_000, [
    (0x093, bytes.fromhex("c27c44ffffffffff060000007ae30814ffffffffffffffff"), 0),
  ]])
  bump = CarState._decode_vehicle_navi_profile(parser.vl[CANFD_NAVI_PROFILE_MSG])
  assert bump == {"value": 6, "offset": 890, "counter": 3, "update": 1, "path_index": 8, "profile_type": 16}
  assert CarState._classify_vehicle_navi_profile(bump) == ("bump", 0, 6)


def test_pv5_canfd_navi_status_decodes_logged_bit_transition():
  parser = CANParser("hyundai_canfd_generated", [(CANFD_NAVI_STATUS_MSG, math.nan)], 1)

  parser.update([1_000_000_000, [(0x380, bytes.fromhex("d0af0b4001000030000000323210f1000000000000000000"), 1)]])
  assert parser.vl[CANFD_NAVI_STATUS_MSG]["SPEED_LIMIT"] == 50
  assert is_canfd_navi_camera_active(parser.vl[CANFD_NAVI_STATUS_MSG])

  parser.update([1_010_000_000, [(0x380, bytes.fromhex("3e38110401000000280000323210f1000000000000000000"), 1)]])
  assert not is_canfd_navi_camera_active(parser.vl[CANFD_NAVI_STATUS_MSG])


def test_pv5_canfd_navi_messages_are_registered_on_their_logged_buses():
  cp = SimpleNamespace(carFingerprint=CAR.KIA_PV5, flags=HyundaiFlags.CANFD | HyundaiFlags.EV,
                       extFlags=0, safetyConfigs=[None])
  parsers = CarState.__new__(CarState).get_can_parsers_canfd(cp)

  assert CANFD_HDA_INFO_MSG in parsers[Bus.pt].vl
  assert CANFD_NAVI_PROFILE_MSG in parsers[Bus.pt].vl
  assert CANFD_NAVI_STATUS_MSG in parsers[Bus.alt].vl
  assert "NEW_MSG_4B4" not in parsers[Bus.pt].vl


def test_pv5_canfd_navi_profile_uses_wrapped_message_timestamp():
  state = _car_state()
  state.vehicleNaviCanControl = True
  state.navi_profile_msg = CANFD_NAVI_PROFILE_MSG
  state.navi_profile_4be = {
    "PROLONG_VALUE": 0xB0,
    "PROLONG_OFFSET": 1996,
    "PROLONG_CYCLIC_COUNTER": 3,
    "PROLONG_UPDATE": 1,
    "PROLONG_PROFILE_TYPE": 16,
  }
  cp = SimpleNamespace(dat={}, ts_nanos={CANFD_NAVI_PROFILE_MSG: {"PROLONG_VALUE": 1}})
  ret = SimpleNamespace(speedLimit=0.0, speedBumpDistance=0.0, schoolZoneActive=False)

  assert state._update_vehicle_navi_events(cp, ret, False)
  assert ret.vehicleNaviAvailable
  assert ret.vehicleNaviActive
  assert ret.vehicleNaviSpeed == 50
  assert state.vehicleNaviCameraTarget == pytest.approx(1996.0)


def test_pv5_canfd_navi_ignores_unverified_speed_limit_zones():
  state = _car_state()
  state.vehicleNaviCanControl = True
  state.vehicleNaviSchoolZoneControl = True
  state.vehicleNaviZoneControlSupported = False
  state.navi_profile_4be = {
    "PROLONG_VALUE": 0xB7,
    "PROLONG_OFFSET": 0,
    "PROLONG_CYCLIC_COUNTER": 3,
    "PROLONG_UPDATE": 1,
    "PROLONG_PROFILE_TYPE": 16,
  }
  cp = SimpleNamespace(dat={}, ts_nanos={"NEW_MSG_4BE": {"PROLONG_VALUE": 1}})
  ret = SimpleNamespace(speedLimit=50.0, speedBumpDistance=0.0, schoolZoneActive=False)

  assert not state._update_vehicle_navi_events(cp, ret, True)
  assert not state.vehicleNaviSpeedZoneActive
  assert not state.vehicleNaviSchoolZoneActive
  assert not ret.vehicleNaviActive


# Raw 2026-09-07 PV5 frames: section approach, announcement, and early exit.
PV5_SECTION_HDA = bytes.fromhex("5ef6be000000000041509a1100000100")
PV5_SECTION_LOW = bytes.fromhex("80a5d30001000000000000325010f1000000000000000000")
PV5_SECTION_HIGH = bytes.fromhex("b99cdd0001000000000010325010f1000000000000000000")
PV5_SECTION_EXIT_HDA = bytes.fromhex("ec0d040000000000521e9a0940000100")
PV5_SECTION_EXIT = bytes.fromhex("ba962c0001000000000000320010f1000000000000000000")


def _pv5_section_state():
  state = _car_state()
  state.canfd_wrapped_navi = True
  state.vehicleNaviZoneControlSupported = False
  state.vehicleNaviCanControl = True
  state.speedcamSectionAvgControl = True
  state.navi_profile_msg = CANFD_NAVI_PROFILE_MSG
  cp = CANParser("hyundai_canfd_generated", [(CANFD_HDA_INFO_MSG, math.nan)], 0)
  alt = CANParser("hyundai_canfd_generated", [(CANFD_NAVI_STATUS_MSG, math.nan)], 1)
  state.hda_info_4a3 = cp.vl[CANFD_HDA_INFO_MSG]
  state.navi_status_380 = alt.vl[CANFD_NAVI_STATUS_MSG]
  return state, cp, alt


def _pv5_section_step(state, cp, alt, t, status=PV5_SECTION_LOW, hda=PV5_SECTION_HDA):
  cp.update([int(t * 1e9), [] if hda is None else [(0x364, hda, 0)]])
  alt.update([int(t * 1e9), [] if status is None else [(0x380, status, 1)]])
  ret = SimpleNamespace(speedLimit=80.0)
  state._update_vehicle_navi_events(cp, ret, False, alt)
  return ret


# Recorded rear-camera warning and its end, 2026-09-21 PV5 segment 32.
PV5_REAR_HDA = bytes.fromhex("c8f096000000000040329a1100000100")
PV5_REAR_END_HDA = bytes.fromhex("bbf1a0000000000040329a0900000100")
PV5_REAR_STATUS = bytes.fromhex("3af78e0001000000000000323210f1000000000000000000")
PV5_FRONT_APPROACH = bytes.fromhex("d0af0b4001000030000000323210f1000000000000000000")
PV5_CAMERA_STATUS_04 = bytes.fromhex("3e38110401000000280000323210f1000000000000000000")


def _pv5_camera_step(state, cp, alt, t, status=PV5_REAR_STATUS, hda=PV5_REAR_HDA):
  cp.update([int(t * 1e9), [] if hda is None else [(0x364, hda, 0)]])
  alt.update([int(t * 1e9), [] if status is None else [(0x380, status, 1)]])
  warning = state._update_pv5_camera_warning(cp, alt)
  ret = SimpleNamespace(vEgo=0.0, speedLimit=state.hda_info_4a3["SPEED_LIMIT"])
  camera = state._update_vehicle_navi_events(cp, ret, warning, alt)
  state.update_speed_limit(ret, warning or camera, distance_time_changed=False)
  return warning, ret


def test_pv5_zero_status_camera_uses_own_distance_and_releases_on_map_end():
  state, cp, alt = _pv5_section_state()
  state._add_vehicle_navi_event("camera", 50, 0, 1000)
  warning, ret = _pv5_camera_step(state, cp, alt, 1)
  assert warning
  assert ret.speedLimitDistance == 300
  assert state.vehicleNaviCameraStatusEvent is None  # Future preview is not this warning.
  state.totalDistance = 350
  warning, ret = _pv5_camera_step(state, cp, alt, 1.1)
  assert warning and ret.speedLimitDistance == 1  # No full-distance restart.
  warning, ret = _pv5_camera_step(state, cp, alt, 1.2, hda=PV5_REAR_END_HDA)
  assert not warning
  assert ret.speedLimitDistance == 0
  assert not ret.vehicleNaviActive and not state.vehicleNaviEvents
  assert state.vehicleNaviCameraStatusTarget is None


# Follow-up log: the notification ends with the current 30 km/h event still
# about 139 m ahead, while MapSource=2 remains asserted. It is not a pass.
PV5_PULSE_HDA = bytes.fromhex("0a43640000000000401e9a1100000100")
PV5_PULSE_HIGH = bytes.fromhex("1f20064001000040000000341e10f1000000000000000000")
PV5_PULSE_LOW = bytes.fromhex("1b43600001000000000002341e10f1000000000000000000")


def test_pv5_current_camera_survives_pulse_end_then_waits_for_next_warning():
  state, cp, alt = _pv5_section_state()
  state._add_vehicle_navi_event("camera", 30, 0, 189.4)
  current = state.vehicleNaviEvents[0]
  state._add_vehicle_navi_event("camera", 50, 0, 1377.4)
  warning, ret = _pv5_camera_step(state, cp, alt, 1, PV5_PULSE_HIGH, PV5_PULSE_HDA)
  assert warning and ret.speedLimit == 30
  assert state.vehicleNaviCameraStatusEvent is current
  state.totalDistance = 50.3
  warning, ret = _pv5_camera_step(state, cp, alt, 1.1, PV5_PULSE_LOW, PV5_PULSE_HDA)
  assert warning and ret.speedLimit == 30
  assert ret.speedLimitDistance == pytest.approx(139.1)
  assert state.vehicleNaviCameraStatusEvent is current
  assert len(state.vehicleNaviEvents) == 2
  # Stationary time and repeated announcements cannot finish/restart it.
  for i in range(12, 80):
    _, ret = _pv5_camera_step(state, cp, alt, i / 10,
                             PV5_PULSE_HIGH if i % 2 else PV5_PULSE_LOW, PV5_PULSE_HDA)
    assert ret.speedLimitDistance == pytest.approx(139.1)
  state.totalDistance = 189.4
  warning, ret = _pv5_camera_step(state, cp, alt, 8, PV5_PULSE_LOW, PV5_PULSE_HDA)
  assert not warning  # The completed event's unchanged map warning is consumed.
  assert ret.speedLimitDistance == 0 and not ret.vehicleNaviActive
  assert state.vehicleNaviEvents == [{"type": "camera", "speed": 50, "kind": 0, "target": 1377.4}]
  assert not _pv5_camera_step(state, cp, alt, 8.1, PV5_PULSE_HIGH, PV5_PULSE_HDA)[0]


@pytest.mark.parametrize("status", [PV5_REAR_STATUS, PV5_CAMERA_STATUS_04, PV5_FRONT_APPROACH])
@pytest.mark.parametrize("hda", [PV5_REAR_END_HDA, PV5_REAR_HDA])
def test_pv5_map_end_or_limit_change_cancels_current_and_queued_cameras(status, hda):
  state, cp, alt = _pv5_section_state()
  state._add_vehicle_navi_event("camera", 30, 0, 180)
  state._add_vehicle_navi_event("camera", 50, 0, 400)
  _pv5_camera_step(state, cp, alt, 1, PV5_PULSE_HIGH, PV5_PULSE_HDA)
  state.totalDistance = 100
  warning, ret = _pv5_camera_step(state, cp, alt, 1.1, status, hda)
  assert not state.vehicleNaviEvents and state.vehicleNaviCameraStatusEvent is None
  assert not ret.vehicleNaviActive
  # A changed active warning gets its own virtual distance, never the old queue.
  assert ret.speedLimitDistance == (300 if warning else 0)
  _, ret = _pv5_camera_step(state, cp, alt, 1.2, status, PV5_REAR_HDA)
  assert ret.speedLimit == 50 and ret.speedLimitDistance == 300
  assert state.vehicleNaviCameraStatusEvent is None


def test_pv5_completed_warning_does_not_capture_next_same_speed_camera():
  state, cp, alt = _pv5_section_state()
  state._add_vehicle_navi_event("camera", 50, 0, 150)
  state._add_vehicle_navi_event("camera", 50, 0, 300)
  _pv5_camera_step(state, cp, alt, 1)
  state.totalDistance = 150
  warning, ret = _pv5_camera_step(state, cp, alt, 1.1)
  assert not warning and ret.speedLimitDistance == 0
  assert not ret.vehicleNaviActive
  assert state.vehicleNaviCameraStatusEvent is None
  assert not _pv5_camera_step(state, cp, alt, 1.2, hda=PV5_REAR_END_HDA)[0]
  assert _pv5_camera_step(state, cp, alt, 1.3)[0]
  assert state.vehicleNaviCameraStatusEvent is None
  assert not state.vehicleNaviEvents  # The cancelled queue cannot return with a new warning.


@pytest.mark.parametrize("status_byte", [0, 4, 1, 2, 64, 128])
def test_pv5_map_warning_is_not_finished_by_notification_byte(status_byte):
  state, cp, alt = _pv5_section_state()
  status = bytearray(PV5_REAR_STATUS)
  status[3] = status_byte
  assert _pv5_camera_step(state, cp, alt, 1, bytes(status))[0]
  assert _pv5_camera_step(state, cp, alt, 1.1)[0]
  assert not _pv5_camera_step(state, cp, alt, 1.2, hda=PV5_REAR_END_HDA)[0]


@pytest.mark.parametrize("missing", ["status", "hda", "both"])
def test_pv5_completed_distance_stays_consumed_through_signal_loss(missing):
  state, cp, alt = _pv5_section_state()
  state._add_vehicle_navi_event("camera", 50, 0, 200)
  _pv5_camera_step(state, cp, alt, 1)
  state.totalDistance = 200
  assert not _pv5_camera_step(state, cp, alt, 2.2,
                             None if missing in ("status", "both") else PV5_REAR_STATUS,
                             None if missing in ("hda", "both") else PV5_REAR_HDA)[0]
  assert not _pv5_camera_step(state, cp, alt, 2.3)[0]
  assert not _pv5_camera_step(state, cp, alt, 2.4, hda=PV5_REAR_END_HDA)[0]
  assert _pv5_camera_step(state, cp, alt, 2.5)[0]


@pytest.mark.parametrize("missing", ["status", "hda", "both"])
def test_pv5_map_fallback_requires_fresh_messages(missing):
  state, cp, alt = _pv5_section_state()
  assert _pv5_camera_step(state, cp, alt, 1)[0]
  warning, ret = _pv5_camera_step(state, cp, alt, 2.1,
                                 None if missing in ("status", "both") else PV5_REAR_STATUS,
                                 None if missing in ("hda", "both") else PV5_REAR_HDA)
  assert not warning and ret.speedLimitDistance == 0


def test_pv5_explicit_queue_clear_does_not_retain_current_camera():
  state, cp, alt = _pv5_section_state()
  state._add_vehicle_navi_event("camera", 50, 0, 200)
  _pv5_camera_step(state, cp, alt, 1)
  state._clear_vehicle_navi_events()
  _, ret = _pv5_camera_step(state, cp, alt, 1.1, hda=PV5_REAR_END_HDA)
  assert ret.speedLimitDistance == 0 and state.vehicleNaviCameraStatusEvent is None


def test_pv5_notification_alone_does_not_create_a_virtual_camera():
  state, cp, alt = _pv5_section_state()
  warning, ret = _pv5_camera_step(state, cp, alt, 1, PV5_FRONT_APPROACH, PV5_REAR_END_HDA)
  assert not warning and ret.speedLimitDistance == 0


@pytest.mark.parametrize("missing", ["status", "hda", "both"])
def test_pv5_signal_loss_cancels_current_and_queued_camera_distances(missing):
  state, cp, alt = _pv5_section_state()
  state._add_vehicle_navi_event("camera", 50, 0, 200)
  state._add_vehicle_navi_event("camera", 30, 0, 500)
  _pv5_camera_step(state, cp, alt, 1)
  state.totalDistance = 60
  warning, ret = _pv5_camera_step(state, cp, alt, 2.2,
                                 None if missing in ("status", "both") else PV5_REAR_STATUS,
                                 None if missing in ("hda", "both") else PV5_REAR_HDA)
  assert not warning and ret.speedLimitDistance == 0 and not ret.vehicleNaviActive
  assert not state.vehicleNaviEvents and state.vehicleNaviCameraStatusEvent is None
  warning, ret = _pv5_camera_step(state, cp, alt, 2.3)
  assert warning and ret.speedLimitDistance == 300
  assert state.vehicleNaviCameraStatusEvent is None  # No stale distance after recovery.


@pytest.mark.parametrize("value,expected_bump", [(0xB0, 0), (6, 200)])
def test_pv5_cancellation_consumes_cached_camera_profile_without_dropping_bumps(value, expected_bump):
  state, cp, alt = _pv5_section_state()
  state.navi_profile_msg = CANFD_NAVI_PROFILE_MSG
  state._add_vehicle_navi_event("camera", 50, 0, 100)
  _pv5_camera_step(state, cp, alt, 1)
  # The same decoder is used by the real parser; a cached profile may have
  # arrived with the cancellation frame before the event loop processes it.
  state.navi_profile_4be = {"PROLONG_VALUE": value, "PROLONG_OFFSET": 200, "PROLONG_PROFILE_TYPE": 16}
  cp.ts_nanos[CANFD_NAVI_PROFILE_MSG] = {"PROLONG_VALUE": 1_100_000_000}
  warning, ret = _pv5_camera_step(state, cp, alt, 1.1, hda=PV5_REAR_END_HDA)
  assert not warning and ret.speedLimitDistance == 0
  assert ret.speedBumpDistance == expected_bump
  assert all(event["type"] != "camera" for event in state.vehicleNaviEvents)
  # A returning warning cannot resurrect the already cached camera distance.
  _, ret = _pv5_camera_step(state, cp, alt, 1.2)
  assert ret.speedLimitDistance == 300 and state.vehicleNaviCameraStatusEvent is None
  # A newly received matching profile can supply the new warning's distance.
  state.navi_profile_4be["PROLONG_VALUE"] = 0xB0
  cp.ts_nanos[CANFD_NAVI_PROFILE_MSG] = {"PROLONG_VALUE": 1_300_000_000}
  _, ret = _pv5_camera_step(state, cp, alt, 1.3)
  assert ret.speedLimitDistance == 200 and state.vehicleNaviCameraStatusEvent is not None


@pytest.mark.parametrize("raw_limit", [0, 255])
def test_pv5_invalid_limit_cancels_camera_queue(raw_limit):
  state, cp, alt = _pv5_section_state()
  state._add_vehicle_navi_event("camera", 50, 0, 200)
  _pv5_camera_step(state, cp, alt, 1)
  hda = bytearray(PV5_REAR_HDA)
  hda[9] = raw_limit
  warning, ret = _pv5_camera_step(state, cp, alt, 1.1, hda=bytes(hda))
  assert not warning and ret.speedLimitDistance == 0 and not state.vehicleNaviEvents


def test_pv5_section_latches_through_alert_end_and_repeated_alert_then_exits():
  state, cp, alt = _pv5_section_state()
  assert not _pv5_section_step(state, cp, alt, 1).vehicleNaviSectionActive
  ret = _pv5_section_step(state, cp, alt, 1.1, PV5_SECTION_HIGH)
  assert ret.vehicleNaviAvailable and ret.vehicleNaviActive and ret.vehicleNaviSectionActive
  assert ret.vehicleNaviSpeed == 80
  assert state.navi_status_380["SECTION_ALERT"] == 1
  assert not is_canfd_navi_camera_active(state.navi_status_380)
  # Fresh messages throughout a section, including another five-second alert.
  for i in range(12, 200):
    status = PV5_SECTION_HIGH if 100 <= i < 150 else PV5_SECTION_LOW
    ret = _pv5_section_step(state, cp, alt, i / 10, status)
    assert ret.vehicleNaviSectionActive and ret.vehicleNaviSpeed == 80
  ret = _pv5_section_step(state, cp, alt, 20, PV5_SECTION_EXIT, PV5_SECTION_EXIT_HDA)
  assert not ret.vehicleNaviSectionActive and not ret.vehicleNaviActive
  # The old limit returning after exit is insufficient to start another zone.
  assert not _pv5_section_step(state, cp, alt, 20.1).vehicleNaviSectionActive


@pytest.mark.parametrize("status,hda", [(None, PV5_SECTION_HDA), (PV5_SECTION_HIGH, None), (None, None)])
def test_pv5_section_releases_on_message_loss_and_requires_new_edge(status, hda):
  state, cp, alt = _pv5_section_state()
  _pv5_section_step(state, cp, alt, 1)
  assert _pv5_section_step(state, cp, alt, 1.1, PV5_SECTION_HIGH).vehicleNaviSectionActive
  assert not _pv5_section_step(state, cp, alt, 2.2, status, hda).vehicleNaviSectionActive
  assert not _pv5_section_step(state, cp, alt, 2.3, PV5_SECTION_HIGH).vehicleNaviSectionActive
  _pv5_section_step(state, cp, alt, 2.4)
  assert _pv5_section_step(state, cp, alt, 2.5, PV5_SECTION_HIGH).vehicleNaviSectionActive


def test_pv5_section_setting_disabled_and_reenabled_requires_new_alert():
  state, cp, alt = _pv5_section_state()
  _pv5_section_step(state, cp, alt, 1)
  assert _pv5_section_step(state, cp, alt, 1.1, PV5_SECTION_HIGH).vehicleNaviSectionActive
  state.vehicleNaviCanControl = False
  assert not _pv5_section_step(state, cp, alt, 1.2, PV5_SECTION_HIGH).vehicleNaviSectionActive
  state.vehicleNaviCanControl = True
  assert not _pv5_section_step(state, cp, alt, 1.3, PV5_SECTION_HIGH).vehicleNaviSectionActive
  _pv5_section_step(state, cp, alt, 1.4)
  assert _pv5_section_step(state, cp, alt, 1.5, PV5_SECTION_HIGH).vehicleNaviSectionActive


@pytest.mark.parametrize("speed,map_source", [(0, 2), (30, 2), (255, 2), (80, 1), (50, 2)])
def test_pv5_section_rejects_invalid_or_disagreeing_limits(speed, map_source):
  state, cp, alt = _pv5_section_state()
  hda = bytearray(PV5_SECTION_HDA)
  hda[9] = speed
  hda[11] = (hda[11] & ~0x38) | (map_source << 3)
  _pv5_section_step(state, cp, alt, 1)
  assert not _pv5_section_step(state, cp, alt, 1.1, PV5_SECTION_HIGH, bytes(hda)).vehicleNaviSectionActive


def test_pv5_section_does_not_start_from_spot_camera_or_other_alert():
  state, cp, alt = _pv5_section_state()
  for i, (camera, alert) in enumerate([(0, 0), (0x40, 0), (4, 0), (0, 2)]):
    status = bytearray(PV5_SECTION_LOW)
    status[3], status[10] = camera, alert
    assert not _pv5_section_step(state, cp, alt, 1 + i / 10, bytes(status)).vehicleNaviSectionActive


def test_pv5_section_imperial_limit_remains_stable():
  state, cp, alt = _pv5_section_state()
  state.is_metric = False
  _pv5_section_step(state, cp, alt, 1)
  ret = _pv5_section_step(state, cp, alt, 1.1, PV5_SECTION_HIGH)
  assert ret.vehicleNaviSpeed == pytest.approx(80 * 1.609344)
  assert _pv5_section_step(state, cp, alt, 1.2).vehicleNaviSectionActive


def test_vehicle_navi_stale_range_average_releases_section():
  state = _car_state()
  state.vehicleNaviCanControl = True
  state.navi_position_4b4 = {"POS_RANGE_AVG_SPEED": 103}
  cp = SimpleNamespace(dat={}, ts_nanos={"NEW_MSG_4B4": {"POS_RANGE_AVG_SPEED": 1_000_000_000}},
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
  cp = SimpleNamespace(dat={}, ts_nanos={"NEW_MSG_4B4": {"POS_RANGE_AVG_SPEED": 1_000_000_000}},
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
  cp = SimpleNamespace(dat={}, ts_nanos={"NEW_MSG_4BE": {"PROLONG_VALUE": 1}})
  ret = SimpleNamespace(speedLimit=100.0, speedBumpDistance=0.0, schoolZoneActive=False)

  # The section-end profile is still useful as a future preview, but must
  # not move an independently active camera warning almost 2 km away.
  assert not state._update_vehicle_navi_events(cp, ret, True)
  assert state.vehicleNaviCameraTarget is None
  ret.speedLimit = 0.0
  assert state._update_vehicle_navi_events(cp, ret, False)
  assert ret.vehicleNaviActive
  assert ret.vehicleNaviSpeed == 100
  assert state.vehicleNaviCameraTarget == pytest.approx(1997.0)

  speed_ret = SimpleNamespace(vEgo=10.0, speedLimit=ret.speedLimit, gasPressed=False)
  state.update_speed_limit(speed_ret, speed_limit_cam=True)
  assert speed_ret.speedLimitDistance == pytest.approx(1997.0 - 10.0 * 0.01)


def _speedcam_state():
  state = _car_state()
  state.vehicleNaviCanControl = 1
  state.speedcam_engaged = True
  return state


def test_speedcam_gas_tap_matches_cruise_timer():
  state = _speedcam_state()
  assert not any(state._update_speedcam_gas_tok(True) for _ in range(39))
  assert state._update_speedcam_gas_tok(False)              # 39 pressed frames: a tap
  assert not any(state._update_speedcam_gas_tok(True) for _ in range(40))
  assert not state._update_speedcam_gas_tok(False)          # 40 frames: a hold
  assert not state._update_speedcam_gas_tok(False)


def test_gas_tap_skips_box_and_is_withheld_from_cruise():
  state = _speedcam_state()
  state.speedcamSkipBox = True
  state.speedcam_policy.add_preview(0xD2, 366, 0.0)
  ret = SimpleNamespace(vEgo=16.0, speedLimit=60.0, gasPressed=False)
  assert not state._apply_speedcam_policy(ret, True, True, 60.0, False, True)
  assert state.speedcam_gas_tok_consumed and not state.speedcam_accel_swallow
  assert not state._apply_speedcam_policy(ret, True, True, 60.0, False, False)
  assert not state.speedcam_gas_tok_consumed                # only the tap frame is reported


def test_skip_inputs_keep_their_roles_while_disengaged():
  state = _speedcam_state()
  state.speedcam_engaged = False
  state.speedcamSkipBox = True
  state.speedcam_policy.add_preview(0xD2, 366, 0.0)
  ret = SimpleNamespace(vEgo=16.0, speedLimit=60.0, gasPressed=False)
  assert state._apply_speedcam_policy(ret, True, True, 60.0, True, True)
  assert not state.speedcam_accel_swallow and not state.speedcam_gas_tok_consumed


def test_stock_mobile_zone_warning_is_ignored_below_navi_mode_three():
  state = _speedcam_state()
  state.speedcam_policy.add_preview(0xD3, 361, 0.0)        # kind 3 mobile zone at the 60 km/h lead
  ret = SimpleNamespace(vEgo=16.0, speedLimit=60.0, gasPressed=False)

  speed_limit_cam = state._apply_speedcam_policy(ret, True, True, 60.0, False)
  state.update_speed_limit(ret, speed_limit_cam)
  assert not speed_limit_cam
  assert ret.speedLimitDistance == 0.0

  state.speedcamMobileZoneDecel = True                      # AutoNaviSpeedCtrlMode 3
  assert state._apply_speedcam_policy(ret, True, True, 60.0, False)


def test_box_camera_skip_consumes_one_accel_press():
  state = _speedcam_state()
  state.speedcamSkipBox = True
  state.speedcam_policy.add_preview(0xD2, 366, 0.0)        # kind 2 box at the 60 km/h lead
  ret = SimpleNamespace(vEgo=16.0, speedLimit=60.0, gasPressed=False)

  assert state._apply_speedcam_policy(ret, True, True, 60.0, False)
  assert not state._apply_speedcam_policy(ret, True, True, 60.0, True)
  assert state.speedcam_accel_swallow

  press = [SimpleNamespace(type=ButtonType.accelCruise, pressed=True), SimpleNamespace(type=ButtonType.gapAdjustCruise, pressed=True)]
  assert [event.type for event in state._swallow_speedcam_accel(press)] == [ButtonType.gapAdjustCruise]
  release = [SimpleNamespace(type=ButtonType.accelCruise, pressed=False)]
  assert state._swallow_speedcam_accel(release) == []
  assert not state.speedcam_accel_swallow
  assert state._swallow_speedcam_accel(press) == press


def test_speedcam_button_filter_keeps_capnp_button_events_intact():
  # Regression: `ret.buttonEvents = f(ret.buttonEvents)` on a capnp builder clears the
  # source list first, so every button became unknown/released on the car.
  state = _speedcam_state()
  ret = structs.CarState()
  ret.buttonEvents = state._swallow_speedcam_accel([
    structs.CarState.ButtonEvent(type=ButtonType.accelCruise, pressed=True),
    structs.CarState.ButtonEvent(type=ButtonType.paddleLeft, pressed=False)])
  assert [(str(b.type), b.pressed) for b in ret.buttonEvents] == [("accelCruise", True), ("paddleLeft", False)]

  source = (Path(__file__).resolve().parents[1] / "carstate.py").read_text(encoding="utf-8")
  assert "_swallow_speedcam_accel(ret.buttonEvents)" not in source


def test_speedcam_skip_needs_driving_speed_and_leaves_fixed_cameras():
  state = _speedcam_state()
  state.speedcamSkipBox = True
  state.speedcam_policy.add_preview(0xD2, 366, 0.0)
  slow = SimpleNamespace(vEgo=2.0, speedLimit=60.0, gasPressed=False)
  assert state._apply_speedcam_policy(slow, True, True, 60.0, True)
  assert not state.speedcam_accel_swallow

  fixed = _speedcam_state()
  fixed.speedcamSkipBox = fixed.speedcamSkipMobileZone = True
  fixed.speedcam_policy.add_preview(0xD0, 366, 0.0)        # kind 0 fixed camera
  ret = SimpleNamespace(vEgo=16.0, speedLimit=60.0, gasPressed=False)
  assert fixed._apply_speedcam_policy(ret, True, True, 60.0, True)
  assert not fixed.speedcam_accel_swallow


def test_speedcam_policy_leaves_pv5_and_disabled_navi_untouched():
  state = _speedcam_state()
  state.canfd_wrapped_navi = True
  state.speedcam_policy.add_preview(0xD3, 361, 0.0)
  ret = SimpleNamespace(vEgo=16.0, speedLimit=60.0, gasPressed=False)
  assert state._apply_speedcam_policy(ret, True, True, 60.0, False)

  state = _car_state()                                      # VehicleNaviCanControl 0
  state.speedcam_policy.add_preview(0xD3, 361, 0.0)
  assert state._apply_speedcam_policy(ret, True, True, 60.0, False)


def _section_state():
  state = _speedcam_state()
  state.speedcamSectionAvgControl = True
  state.speedcam_v_cruise = 130.0
  state.cruise_buttons = deque([0])
  return state


CLU_RATIO = 0.968  # vEgo / cluster speed on this car (1/1.033)
TRUE_OVER_VEGO = 1.0074  # GPS / vEgo, largest drive median (2026-09-30)


def _section_ret(v_ego=16.0):
  return SimpleNamespace(vEgo=v_ego, vehicleNaviSectionActive=True, vehicleNaviSpeed=80.0, speedLimitDistance=1.0,
                         vCluRatio=CLU_RATIO)


def test_section_stays_capped_until_unlocked():
  state = _section_state()
  ret = _section_ret()
  state._apply_section_average(ret, False, False, False)
  assert not state.section_average.unlocked
  assert (ret.vehicleNaviSpeed, ret.speedLimitDistance) == (80.0, 1.0)   # existing cap and virtual distance

  state.speedcam_v_cruise = 85.0                            # set speed not above 80 x 1.07
  state._apply_section_average(ret, True, False, False)
  assert not state.section_average.unlocked and not state.speedcam_accel_swallow


def test_section_unlock_by_plus_or_gas_tap_spends_banked_time():
  state = _section_state()
  state.vehicleNaviEvents = [{"type": "camera", "speed": 80, "kind": 0, "target": 900.0},
                             {"type": "camera", "speed": 80, "kind": 0, "target": 2500.0}]
  state.totalDistance = 1000.0
  ret = _section_ret()
  state._apply_section_average(ret, True, False, False)
  assert state.section_average.unlocked and state.speedcam_accel_swallow
  assert ret.vehicleNaviSpeed == 80.0                       # no banked time yet
  assert ret.speedLimitDistance == pytest.approx(1500.0)    # only the end camera ahead, no virtual distance

  state.speedcam_time += 60.0                               # a slow minute: banked time
  ret = _section_ret()
  state._apply_section_average(ret, False, False, False)
  assert ret.vehicleNaviSpeed > 80.0

  gas = _section_state()
  gas._apply_section_average(_section_ret(), False, True, False)
  assert gas.section_average.unlocked and gas.speedcam_gas_tok_consumed


def test_section_long_minus_locks_again_and_camera_skip_wins_the_input():
  state = _section_state()
  state._apply_section_average(_section_ret(), True, False, False)
  assert state.section_average.unlocked
  state.speedcamLongPressFrames = 40
  state.cruise_buttons = deque([Buttons.SET_DECEL])
  assert not any(state._update_speedcam_decel_long_press() for _ in range(40))
  assert state._update_speedcam_decel_long_press()          # 41st held frame = cruise.py long press
  ret = _section_ret()
  state._apply_section_average(ret, False, False, True)
  assert not state.section_average.unlocked
  assert (ret.vehicleNaviSpeed, ret.speedLimitDistance) == (80.0, 1.0)

  skipped = _section_state()
  skipped.speedcam_input_consumed = True                    # this press already skipped a camera
  skipped._apply_section_average(_section_ret(), True, False, False)
  assert not skipped.section_average.unlocked


def _decel(pressed):
  return structs.CarState.ButtonEvent(type=ButtonType.decelCruise, pressed=pressed)


def _decel_types(events):
  return [(str(event.type), event.pressed) for event in events if event.type == ButtonType.decelCruise]


def test_section_long_minus_relock_never_reaches_cruise():
  # A long cruise - that relocks an unlocked section must not also step the set speed down by 10.
  state = _section_state()
  state._apply_section_average(_section_ret(), True, False, False)
  assert state.section_average.unlocked
  state.speedcamLongPressFrames = 40
  state.cruise_buttons = deque([Buttons.SET_DECEL])
  paddle = structs.CarState.ButtonEvent(type=ButtonType.paddleLeft, pressed=True)

  assert _decel_types(state._hold_speedcam_decel([_decel(True), paddle], False)) == []
  forwarded = []
  for _ in range(80):                                       # hold past the long press and one repeat
    long_press = state._update_speedcam_decel_long_press()
    state._apply_section_average(_section_ret(), False, False, long_press)
    forwarded += state._hold_speedcam_decel([], long_press)
  assert not state.section_average.unlocked                 # relocked
  assert forwarded == []
  assert state._hold_speedcam_decel([_decel(False)], False) == []   # release swallowed too
  assert not state.speedcam_decel_hold


def test_section_short_minus_is_handed_to_cruise_at_release():
  state = _section_state()
  state._apply_section_average(_section_ret(), True, False, False)
  state.speedcamLongPressFrames = 40
  state.cruise_buttons = deque([Buttons.SET_DECEL])

  assert state._hold_speedcam_decel([_decel(True)], False) == []
  for _ in range(10):
    assert state._hold_speedcam_decel([], state._update_speedcam_decel_long_press()) == []
  ret = structs.CarState()
  ret.buttonEvents = state._hold_speedcam_decel([_decel(False)], False)
  # cruise.py handles a same-frame press + release as one short press (as for remote buttons)
  assert _decel_types(ret.buttonEvents) == [("decelCruise", True), ("decelCruise", False)]
  assert state.section_average.unlocked and not state.speedcam_decel_hold


def test_minus_passes_through_while_section_locked():
  state = _section_state()
  assert not state.section_average.unlocked
  press, release = [_decel(True)], [_decel(False)]
  assert state._hold_speedcam_decel(press, False) == press
  assert state._hold_speedcam_decel([], True) == []
  assert state._hold_speedcam_decel(release, False) == release


def test_section_allowance_is_inverted_through_factor_and_cluster_ratio():
  state = _section_state()
  state._apply_section_average(_section_ret(), True, False, False)       # unlock
  state.speedcam_time += 60.0                                            # bank a minute
  ret = _section_ret()
  state._apply_section_average(ret, False, False, False)
  allowance = state.section_average.allowance_kph(state.speedcam_time, state.totalDistance)
  assert ret.vehicleNaviSpeed == pytest.approx(allowance / 1.015 / CLU_RATIO / 1.07)
  # What the car then drives (carrot_serv x factor, planner x vCluRatio) is the allowance in vEgo units.
  assert ret.vehicleNaviSpeed * 1.07 * CLU_RATIO == pytest.approx(allowance / 1.015)


def test_section_true_average_stays_at_or_below_the_limit_end_to_end():
  # Congestion banks time; the driver unlocks with the set speed at 130 and the car follows the
  # section target the way carrot_serv (x factor, cluster speed) and the planner (x vCluRatio) do.
  # Its true speed is vEgo x TRUE_OVER_VEGO. While banked time is being spent the true running
  # average must stay at or below the limit.
  state = _section_state()
  state.speedcam_v_cruise = 130.0
  dt = DT_CTRL * 10
  true_distance = 0.0

  def step(v_ego, unlock=False):
    nonlocal true_distance
    ret = _section_ret(v_ego)
    state.speedcam_time += dt - DT_CTRL                    # _apply_section_average adds DT_CTRL itself
    state._apply_section_average(ret, unlock, False, False)
    state.totalDistance += v_ego * dt
    true_distance += v_ego * TRUE_OVER_VEGO * dt
    return ret

  for _ in range(int(120 / dt)):
    step(40 / 3.6 / TRUE_OVER_VEGO)                        # two minutes at a true 40 km/h
  ret = step(40 / 3.6 / TRUE_OVER_VEGO, unlock=True)
  assert state.section_average.unlocked

  spent_steps = 0
  while state.section_average.allowance_kph(state.speedcam_time, state.totalDistance) > 0.0:
    v_ego = min(130.0, ret.vehicleNaviSpeed * 1.07) * CLU_RATIO / 3.6
    ret = step(v_ego)
    elapsed = state.speedcam_time - state.section_average.start_time
    assert true_distance / elapsed * 3.6 <= 80.0 + 0.1
    spent_steps += 1
    assert spent_steps < int(1200 / dt)
  assert spent_steps * dt > 30                             # the bank was actually used above the limit


def _bump_state(cancel=True):
  state = _speedcam_state()
  state.speedcamCancelBump = cancel
  state.speedcam_v_cruise = 60.0
  state.vehicleNaviEvents = [{"type": "bump", "speed": 0, "kind": 6, "target": 80.0},
                             {"type": "bump", "speed": 0, "kind": 6, "target": 300.0}]
  return state


def _bump_ret():
  return SimpleNamespace(vEgo=16.0, speedBumpDistance=80.0)


def test_bump_decel_cancel_releases_only_the_decelerating_bump():
  state = _bump_state()
  assert state._speedcam_bump_target_kph(80.0) < 60.0        # 35 km/h bump 80 m ahead: slowing below the set 60
  ret = _bump_ret()
  state._apply_speedcam_bump_cancel(ret, True, False)
  assert ret.speedBumpDistance == pytest.approx(300.0)      # the next bump still decelerates
  assert state.speedcam_input_consumed and state.speedcam_accel_swallow
  assert state.vehicleNaviEvents[0]["cancelled"]

  ret = _bump_ret()                                         # following frames keep the cancel
  state.speedcam_input_consumed = state.speedcam_accel_swallow = False
  state._apply_speedcam_bump_cancel(ret, False, False)
  assert ret.speedBumpDistance == pytest.approx(300.0)


def test_bump_cancel_needs_actual_deceleration_and_its_toggles():
  far = _bump_state()
  far.vehicleNaviEvents = [{"type": "bump", "speed": 0, "kind": 6, "target": 400.0}]
  assert far._speedcam_bump_target_kph(400.0) > 60.0        # not slowing yet
  ret = SimpleNamespace(vEgo=16.0, speedBumpDistance=400.0)
  far._apply_speedcam_bump_cancel(ret, True, False)
  assert ret.speedBumpDistance == pytest.approx(400.0) and not far.speedcam_input_consumed

  off = _bump_state(cancel=False)                           # master or bump toggle off
  ret = _bump_ret()
  off._apply_speedcam_bump_cancel(ret, True, True)
  assert ret.speedBumpDistance == pytest.approx(80.0) and not off.speedcam_input_consumed

  disengaged = _bump_state()
  disengaged.speedcam_engaged = False
  ret = _bump_ret()
  disengaged._apply_speedcam_bump_cancel(ret, False, True)
  assert ret.speedBumpDistance == pytest.approx(80.0) and not disengaged.speedcam_gas_tok_consumed


def test_bump_gas_tap_cancel_and_camera_skip_win_the_input():
  state = _bump_state()
  ret = _bump_ret()
  state._apply_speedcam_bump_cancel(ret, False, True)
  assert ret.speedBumpDistance == pytest.approx(300.0) and state.speedcam_gas_tok_consumed

  taken = _bump_state()
  taken.speedcam_input_consumed = True                      # this press already skipped a camera
  ret = _bump_ret()
  taken._apply_speedcam_bump_cancel(ret, True, False)
  assert ret.speedBumpDistance == pytest.approx(80.0)


def test_bump_cancel_survives_a_repeated_announcement():
  state = _bump_state()
  state._apply_speedcam_bump_cancel(_bump_ret(), True, False)
  state._add_vehicle_navi_event("bump", 0, 6, 85.0)         # same bump announced again, 5 m further
  assert len(state.vehicleNaviEvents) == 2 and state.vehicleNaviEvents[0]["cancelled"]


def test_decel_cancel_master_gates_every_kind():
  state = _car_state()
  assert not (state.speedcamSkipBox or state.speedcamSkipMobileZone or state.speedcamCancelBump
              or state.speedcamSkipUnknown)
  state.op_params.get_bool = lambda key: True
  state._read_speedcam_params()
  assert state.speedcamSkipBox and state.speedcamSkipMobileZone and state.speedcamCancelBump
  assert state.speedcamSkipUnknown and state.speedcam_policy.early_lead


def test_fixed_camera_always_decelerates():
  state = _speedcam_state()
  state.speedcamSkipBox = state.speedcamSkipMobileZone = state.speedcamSkipUnknown = True
  state.speedcam_policy.add_preview(0xD0, 366, 0.0)        # kind 0 fixed @60
  ret = SimpleNamespace(vEgo=16.0, speedLimit=60.0, gasPressed=False, vehicleNaviSectionActive=False)
  assert state._apply_speedcam_policy(ret, True, True, 60.0, True)
  assert not state.speedcam_accel_swallow                 # + keeps raising the set speed


def test_gv80_horizon_trim_keeps_nearest_boundaries():
  # 호라이즌이 32개를 넘어도 가까운 등급 경계가 남아 있어야 한다. 먼 미래만
  # 남기면 간선도로(등급 5) 진입 경계가 사라져 현재 등급을 6으로 오판한다.
  state = _car_state()
  state._gv80_update_road_horizon(0.0, 6)                  # 출발 지점: 생활도로
  for i in range(1, 41):                                   # 100m 간격 전방 구간들
    state._gv80_update_road_horizon(i * 100.0, 5 if i < 40 else 6)
  assert len(state.vehicleNaviRoadHorizon) <= 32
  state.totalDistance = 150.0                              # 100m 경계를 지난 지점
  assert state._gv80_current_road_class() == 5


def test_gv80_horizon_small_offset_segment_keeps_current_cover():
  # 오프셋이 15m 이하인 새 구간이 와도, 그 구간이 시작되기 전까지는 현재
  # 위치를 커버하는 항목이 남아 현재 등급이 불명(None)이 되면 안 된다.
  state = _car_state()
  state.totalDistance = 990.0
  state._gv80_update_road_horizon(990.0, 6)
  state.totalDistance = 1000.0
  state._gv80_update_road_horizon(1002.0, 5)               # 2m 앞에서 시작하는 새 구간
  assert state._gv80_current_road_class() == 6
  assert state._gv80_bump_output_allowed()
  state.totalDistance = 1003.0                             # 새 구간에 진입한 뒤
  assert state._gv80_current_road_class() == 5
  assert not state._gv80_bump_output_allowed()


def test_gv80_horizon_chain_of_small_offset_segments_keeps_current_cover():
  # 15m 이하 간격의 구간이 연속으로 들어와도 현재 등급이 중간에 불명(None)이
  # 되면 안 된다. 실차에서는 이 연쇄 삭제로 실물 방지턱 접근 구간 전체가
  # 블랙아웃된 사례가 있었다 (2026-11-23 route 00000008).
  state = _car_state()
  state.totalDistance = 1000.0
  state._gv80_update_road_horizon(1000.0, 6)
  for total_distance, offset in ((1005.0, 10.0), (1011.0, 16.0), (1022.0, 27.0)):
    state.totalDistance = total_distance
    state._gv80_update_road_horizon(total_distance + offset, 6)
    assert state._gv80_current_road_class() == 6
    assert state._gv80_bump_output_allowed()


def _make_real_canfd_navi(monkeypatch, fingerprint):
  from unittest.mock import Mock
  from opendbc.car.hyundai import carstate, hyundaicanfd

  params = Mock()
  params.get_int.side_effect = lambda key: 1 if key == "VehicleNaviCanControl" else 0
  params.get_bool.side_effect = lambda key: key == "ControlsReady"
  params.get.return_value = "{0: {}, 1: {}, 2: {}}"
  monkeypatch.setattr(carstate, "Params", lambda: params)
  monkeypatch.setattr(hyundaicanfd, "Params", lambda: params)
  cp_cfg = structs.CarParams(carFingerprint=fingerprint, flags=int(HyundaiFlags.CANFD), safetyConfigs=[{}])
  state = CarState(cp_cfg)
  state.is_metric = True
  state.vehicleNaviCanControl = 1
  parsers = state.get_can_parsers(cp_cfg)
  for parser in parsers.values():
    parser.controls_ready = True
  return state, parsers


def test_vehicle_navi_4de_dbc_decodes_logged_frames():
  parser = CANParser("hyundai_canfd_generated", [("NEW_MSG_4DE", math.nan)], 0)

  # GV80 route 00000381 s17 (110km/h 구간단속 본구간)
  parser.update([1_000_000_000, [(0x4DE, bytes.fromhex("0500b98a108ac895"), 0)]])
  assert parser.vl["NEW_MSG_4DE"]["POS_OFFSET"] == 5
  assert parser.vl["NEW_MSG_4DE"]["POS_PATH_INDEX"] == 8
  assert parser.vl["NEW_MSG_4DE"]["POS_CYCLIC_COUNTER"] == 3
  assert parser.vl["NEW_MSG_4DE"]["POS_RANGE_AVG_SPEED"] == 85

  # route 0000000a s15 (구간단속 외 평시 주행)
  parser.update([1_100_000_000, [(0x4DE, bytes.fromhex("0200014070730430"), 0)]])
  assert parser.vl["NEW_MSG_4DE"]["POS_OFFSET"] == 2
  assert parser.vl["NEW_MSG_4DE"]["POS_PATH_INDEX"] == 8
  assert parser.vl["NEW_MSG_4DE"]["POS_CYCLIC_COUNTER"] == 0
  assert parser.vl["NEW_MSG_4DE"]["POS_RANGE_AVG_SPEED"] == 0

  # route 00000001 s00 (초기화 센티넬 프레임)
  parser.update([1_200_000_000, [(0x4DE, bytes.fromhex("ff1fe0ffffffffff"), 0)]])
  assert parser.vl["NEW_MSG_4DE"]["POS_OFFSET"] == 8191
  assert parser.vl["NEW_MSG_4DE"]["POS_PATH_INDEX"] == 0
  assert parser.vl["NEW_MSG_4DE"]["POS_CYCLIC_COUNTER"] == 0
  assert parser.vl["NEW_MSG_4DE"]["POS_RANGE_AVG_SPEED"] == 511


def test_vehicle_navi_4de_activates_and_ends_section_on_any_canfd_model(monkeypatch):
  state, parsers = _make_real_canfd_navi(monkeypatch, CAR.HYUNDAI_IONIQ_5)
  state.speedcamSectionAvgControl = True
  pt = parsers[Bus.pt]
  bus = pt.bus

  # 1) 초기 수신·등록
  pt.update([1_000_000_000, [(0x4DE, bytes.fromhex("0500b98a108ac895"), bus)]])
  state.monitor_fingerprint(parsers, True)
  state.controls_ready_count = 122
  pt.update([1_100_000_000, [(0x4DE, bytes.fromhex("0500b98a108ac895"), bus)]])
  state.monitor_fingerprint(parsers, True)
  assert state.navi_position_4de is pt.vl["NEW_MSG_4DE"]

  ret = SimpleNamespace(speedLimit=110.0, speedBumpDistance=0.0, schoolZoneActive=False)
  state._update_vehicle_navi_events(pt, ret, True)
  assert ret.vehicleNaviSectionActive
  assert ret.vehicleNaviSpeed == 110.0

  # 2) 출구 통과 시 0x4A3 경고가 아직 켜져 있어도 0x4DE avg=0이면 즉시 종료
  pt.update([1_200_000_000, [(0x4DE, bytes.fromhex("0200014070730430"), bus)]])
  ret = SimpleNamespace(speedLimit=110.0, speedBumpDistance=0.0, schoolZoneActive=False)
  state._update_vehicle_navi_events(pt, ret, True)
  assert not state.vehicleNaviSpeedZoneActive
  assert not ret.vehicleNaviSectionActive


def test_vehicle_navi_4de_sentinel_frame_does_not_activate_section(monkeypatch):
  state, parsers = _make_real_canfd_navi(monkeypatch, CAR.HYUNDAI_IONIQ_5)
  pt = parsers[Bus.pt]
  bus = pt.bus
  state.controls_ready_count = 122
  frames = [
    (0x4DE, bytes.fromhex("ff1fe0ffffffffff"), bus),
    (0x4B4, bytes.fromhex("0d0085cbb18948a5"), bus),
  ]
  pt.update([1_000_000_000, frames])
  state.monitor_fingerprint(parsers, True)
  pt.update([1_100_000_000, frames])

  ret = SimpleNamespace(speedLimit=100.0, speedBumpDistance=0.0, schoolZoneActive=False)
  state._update_vehicle_navi_events(pt, ret, True)
  assert state.vehicleNaviPosition4deSeen
  assert not state.vehicleNaviSpeedZoneActive
  assert not ret.vehicleNaviSectionActive


@pytest.mark.parametrize("size", (4, 12, 24))
def test_vehicle_navi_4de_other_length_is_ignored(size):
  state = _car_state()
  state.vehicleNaviCanControl = True
  state.speedcamSectionAvgControl = True
  state.navi_position_4de = {"POS_OFFSET": 5, "POS_PATH_INDEX": 8, "POS_RANGE_AVG_SPEED": 85}
  state.navi_position_4b4 = {"POS_RANGE_AVG_SPEED": 92}
  payload = bytes.fromhex("0500b98a108ac895").ljust(size, b"\x00")[:size]
  cp = SimpleNamespace(
    dat={0x4DE: payload, 0x4B4: bytes.fromhex("0d0085cbb18948a5")},
    ts_nanos={
      "NEW_MSG_4DE": {"POS_RANGE_AVG_SPEED": 1_000_000_000},
      "NEW_MSG_4B4": {"POS_RANGE_AVG_SPEED": 1_000_000_000},
    },
    _last_update_nanos=1_050_000_000,
  )
  ret = SimpleNamespace(speedLimit=100.0, speedBumpDistance=0.0, schoolZoneActive=False)
  state._update_vehicle_navi_events(cp, ret, True)
  assert not state.vehicleNaviPosition4deSeen
  assert ret.vehicleNaviSectionActive  # 타 차종 0x4B4 경로 불변


def test_vehicle_navi_4de_priority_and_fallback():
  state = _car_state()
  state.vehicleNaviCanControl = True
  state.speedcamSectionAvgControl = True
  state.navi_position_4de = {"POS_OFFSET": 5, "POS_PATH_INDEX": 8, "POS_RANGE_AVG_SPEED": 0}
  cp = SimpleNamespace(
    dat={0x4DE: bytes.fromhex("0200014070730430")},
    ts_nanos={"NEW_MSG_4DE": {"POS_RANGE_AVG_SPEED": 1_000_000_000}},
    _last_update_nanos=1_050_000_000,
  )

  # 1) 0x4DE avg=0이 살아 있는 동안은 1200m 이상 경고가 이어져도 폴백이 구간을 켜지 않는다.
  for dist in (0.0, 600.0, 1200.0):
    state.totalDistance = dist
    ret = SimpleNamespace(speedLimit=100.0, speedBumpDistance=0.0, schoolZoneActive=False)
    state._update_vehicle_navi_events(cp, ret, True)
    assert not ret.vehicleNaviSectionActive

  # 2) 0x4DE가 타임아웃(>1s)되면 예비 1000m 폴백이 동작한다.
  cp._last_update_nanos = 2_500_000_000
  state.totalDistance = 1300.0
  ret = SimpleNamespace(speedLimit=100.0, speedBumpDistance=0.0, schoolZoneActive=False)
  state._update_vehicle_navi_events(cp, ret, True)
  assert not ret.vehicleNaviSectionActive  # 폴백 거리 측정 시작점 재설정

  state.totalDistance = 2350.0
  cp._last_update_nanos = 3_500_000_000
  ret = SimpleNamespace(speedLimit=100.0, speedBumpDistance=0.0, schoolZoneActive=False)
  state._update_vehicle_navi_events(cp, ret, True)
  assert ret.vehicleNaviSectionActive
  assert ret.vehicleNaviSpeed == 100.0

  # 3) 다시 0x4DE avg=0이 수신되면 즉시 종료되고 폴백 타이머도 초기화된다.
  cp.ts_nanos["NEW_MSG_4DE"]["POS_RANGE_AVG_SPEED"] = 4_000_000_000
  cp._last_update_nanos = 4_050_000_000
  ret = SimpleNamespace(speedLimit=100.0, speedBumpDistance=0.0, schoolZoneActive=False)
  state._update_vehicle_navi_events(cp, ret, True)
  assert not ret.vehicleNaviSectionActive
  assert state.vehicleNaviGv80SectionWarningSpeed == 0.0

  cp._last_update_nanos = 5_500_000_000
  state.totalDistance = 2500.0
  ret = SimpleNamespace(speedLimit=100.0, speedBumpDistance=0.0, schoolZoneActive=False)
  state._update_vehicle_navi_events(cp, ret, True)
  assert not ret.vehicleNaviSectionActive


def test_vehicle_navi_fallback_warning_speed_resets_on_mode_off_and_mode3_clear():
  state = _car_state()
  state.vehicleNaviCanControl = 1
  state.op_params.vehicle_navi = 1
  state.vehicleNaviGv80SectionWarningSpeed = 100.0
  state.vehicleNaviGv80SectionWarningStartDistance = 500.0

  # CanControl 1->0 전환 시 초기화
  state.op_params.vehicle_navi = 0
  state.vehicleSpeedCameraParamsCounter = VEHICLE_SPEED_CAMERA_PARAM_UPDATE_FRAMES - 1
  state._update_vehicle_speed_camera_params()
  assert state.vehicleNaviCanControl == 0
  assert state.vehicleNaviGv80SectionWarningSpeed == 0.0
  assert state.vehicleNaviGv80SectionWarningStartDistance == 0.0

  # 모드 3 리셋 시 초기화
  state.vehicleNaviCanControl = 3
  state.vehicleNaviGv80SectionWarningSpeed = 110.0
  state.vehicleNaviGv80SectionWarningStartDistance = 900.0
  state._clear_vehicle_navi_route_filtered_events()
  assert state.vehicleNaviGv80SectionWarningSpeed == 0.0
  assert state.vehicleNaviGv80SectionWarningStartDistance == 0.0


def test_alt_legacy_navi_signal_gating_opens_without_gv80_fingerprint(monkeypatch):
  # A) 비GV80 차종(IONIQ 5)에서도 0x4B5+0x4DE+0x4B8 신호가 오면 게이팅·호라이즌·리셋이 열린다.
  state, parsers = _make_real_canfd_navi(monkeypatch, CAR.HYUNDAI_IONIQ_5)
  pt = parsers[Bus.pt]
  bus = pt.bus
  state.controls_ready_count = 122

  # 0x4B5: offset=0, path=8, act=1(0x08), frc=5(간선) -> byte4 = 0x08 | 5 = 0x0D
  seg_frc5 = (8 << 13) | (0x0D << 32)
  bump_frame = (0x4B8, bytes.fromhex("060000003d410804"), bus)     # 방지턱 317m, path 8
  cam_frame = (0x4B8, bytes.fromhex("b10000002c010804"), bus)      # 무경고 단독 카메라 50km/h @ 300m, path 8
  init_frames = [
    (0x4DE, bytes.fromhex("0200014070730430"), bus),
    (0x4B5, seg_frc5.to_bytes(8, "little"), bus),
    bump_frame,
  ]
  pt.update([1_000_000_000, init_frames])
  state.monitor_fingerprint(parsers, True)
  pt.update([1_100_000_000, init_frames])
  ret = SimpleNamespace(speedLimit=0.0, speedBumpDistance=0.0, schoolZoneActive=False)
  assert not state._update_vehicle_navi_events(pt, ret, False)
  assert ret.speedBumpDistance == 0.0  # 등급 5에서는 방지턱 차단

  pt.update([1_150_000_000, [cam_frame]])
  ret = SimpleNamespace(speedLimit=0.0, speedBumpDistance=0.0, schoolZoneActive=False)
  assert not state._update_vehicle_navi_events(pt, ret, False)  # 무경고 카메라 차단
  assert ret.speedBumpDistance == 0.0

  # 등급 6(생활도로)으로 갱신되면 방지턱 출력 허용
  seg_frc6 = (8 << 13) | (0x0E << 32)
  pt.update([1_200_000_000, [(0x4B5, seg_frc6.to_bytes(8, "little"), bus)]])
  ret = SimpleNamespace(speedLimit=0.0, speedBumpDistance=0.0, schoolZoneActive=False)
  state._update_vehicle_navi_events(pt, ret, False)
  assert ret.speedBumpDistance == 317.0

  # 0x4B5 안내 OFF(1->0) 전이 시 큐 리셋
  seg_off = (8 << 13) | (0x06 << 32)
  pt.update([1_300_000_000, [(0x4B5, seg_off.to_bytes(8, "little"), bus)]])
  ret = SimpleNamespace(speedLimit=0.0, speedBumpDistance=0.0, schoolZoneActive=False)
  state._update_vehicle_navi_events(pt, ret, False)
  assert state.vehicleNaviEvents == []
  assert ret.speedBumpDistance == 0.0

  # B) 표준 0x4B9 차종에서는 0x4B4 구간·단독 카메라·방지턱·종류7 래치 동작이 그대로 유지된다.
  std_state, std_parsers = _make_real_canfd_navi(monkeypatch, CAR.HYUNDAI_IONIQ_5)
  std_state.speedcamSectionAvgControl = True
  std_pt = std_parsers[Bus.pt]
  std_bus = std_pt.bus
  std_state.controls_ready_count = 122
  seg_4b9 = (8 << 13) | (1 << 22) | (3 << 24)
  std_frames = [
    (0x4B9, seg_4b9.to_bytes(8, "little"), std_bus),
    (0x4B4, bytes.fromhex("0d0085cbb18948a5"), std_bus),
    (0x4B8, bytes.fromhex("060000003d410804"), std_bus),
  ]
  std_pt.update([1_000_000_000, std_frames])
  std_state.monitor_fingerprint(std_parsers, True)
  std_pt.update([1_100_000_000, std_frames])
  std_ret = SimpleNamespace(speedLimit=100.0, speedBumpDistance=0.0, schoolZoneActive=False)
  std_state._update_vehicle_navi_events(std_pt, std_ret, True)
  assert std_ret.vehicleNaviSectionActive
  assert std_ret.speedBumpDistance == 317.0


def _make_4b8_bump(offset, path=8):
  raw = 6 | (int(offset) << 32) | (int(path) << 48) | (16 << 54)
  return raw.to_bytes(8, "little")


def _make_4b4_horizon(offset, path_index, road_class):
  raw = (int(offset) & 0x1fff) | ((int(path_index) & 0x3f) << 13) | ((int(road_class) & 0x7) << 24)
  return raw.to_bytes(8, "little")


def test_gv80_frc5_bump_allows_near_or_reconfirmed_low_speed_only(monkeypatch):
  state, parsers = _make_real_canfd_navi(monkeypatch, CAR.HYUNDAI_IONIQ_5)
  pt = parsers[Bus.pt]
  bus = pt.bus
  state.controls_ready_count = 122

  # FRC 5, 450m 원거리 단발 예고 -> 차단
  seg_frc5 = ((8 << 13) | (0x0D << 32)).to_bytes(8, "little")
  init_frames = [
    (0x4DE, bytes.fromhex("0200014070730430"), bus),
    (0x4B5, seg_frc5, bus),
    (0x4B8, _make_4b8_bump(450), bus),
  ]
  pt.update([1_000_000_000, init_frames])
  state.monitor_fingerprint(parsers, True)
  pt.update([1_100_000_000, init_frames])
  ret = SimpleNamespace(vEgo=30.0 / 3.6, steeringAngleDeg=0.0, speedLimit=0.0, speedBumpDistance=0.0, schoolZoneActive=False)
  state._update_vehicle_navi_events(pt, ret, False)
  assert ret.speedBumpDistance == 0.0

  # 접근하며 200m로 재확인(min_offset=200m) -> 저속(30 km/h)에서는 출력 허용
  state.totalDistance = 250.0
  pt.update([1_200_000_000, [(0x4B8, _make_4b8_bump(200), bus)]])
  ret = SimpleNamespace(vEgo=30.0 / 3.6, steeringAngleDeg=0.0, speedLimit=0.0, speedBumpDistance=0.0, schoolZoneActive=False)
  state._update_vehicle_navi_events(pt, ret, False)
  assert ret.speedBumpDistance == pytest.approx(200.0)

  # 같은 FRC 5 지점이어도 고속(70 km/h > 55 km/h)에서는 차단
  ret_fast = SimpleNamespace(vEgo=70.0 / 3.6, steeringAngleDeg=0.0, speedLimit=0.0, speedBumpDistance=0.0, schoolZoneActive=False)
  state._update_vehicle_navi_events(pt, ret_fast, False)
  assert ret_fast.speedBumpDistance == 0.0

  # FRC 4(큰 도로)로 바뀌면 저속·근거리라도 영구 차단
  seg_frc4 = ((8 << 13) | (0x0C << 32)).to_bytes(8, "little")
  pt.update([1_300_000_000, [(0x4B5, seg_frc4, bus)]])
  ret_frc4 = SimpleNamespace(vEgo=30.0 / 3.6, steeringAngleDeg=0.0, speedLimit=0.0, speedBumpDistance=0.0, schoolZoneActive=False)
  state._update_vehicle_navi_events(pt, ret_frc4, False)
  assert ret_frc4.speedBumpDistance == 0.0


def test_gv80_reset_preserves_bumps_on_straight_drive_and_drops_on_major_road():
  state = _car_state()
  state.vehicleNaviCanControl = 1
  state.vehicleNaviPosition4deSeen = True
  state.vehicleNaviRoadHorizon = [(0.0, 5)]
  state._add_vehicle_navi_event("bump", 0, 6, 168.0)
  state._add_vehicle_navi_event("bump", 0, 6, 211.0)
  state.vehicleNaviGv80SteerHistory.extend([2.0, 8.8, 0.5])

  # 1) 직진 중(max |steer| = 8.8° < 60°) 0x4B4 pi=0 리셋(rc=5) -> 큐 보존
  cp = SimpleNamespace(
    dat={0x4B4: _make_4b4_horizon(0, 0, 5)},
    ts_nanos={"NEW_MSG_4B4": {"POS_OFFSET": 1_000_000_000}},
    _last_update_nanos=1_000_000_000,
  )
  ret = SimpleNamespace(vEgo=20.0 / 3.6, steeringAngleDeg=0.5, speedLimit=0.0, speedBumpDistance=0.0, schoolZoneActive=False)
  state._update_vehicle_navi_events(cp, ret, False)
  assert [e["target"] for e in state.vehicleNaviEvents] == [168.0, 211.0]
  assert ret.speedBumpDistance == pytest.approx(168.0)

  # 2) 직진이라도 큰 도로(FRC 4) 진입 리셋이면 120m 초과 삭제
  cp_major = SimpleNamespace(
    dat={0x4B4: _make_4b4_horizon(0, 0, 4)},
    ts_nanos={"NEW_MSG_4B4": {"POS_OFFSET": 2_000_000_000}},
    _last_update_nanos=2_000_000_000,
  )
  ret_major = SimpleNamespace(vEgo=20.0 / 3.6, steeringAngleDeg=0.5, speedLimit=0.0, speedBumpDistance=0.0, schoolZoneActive=False)
  state._update_vehicle_navi_events(cp_major, ret_major, False)
  assert state.vehicleNaviEvents == []
  assert ret_major.speedBumpDistance == 0.0


def test_gv80_reset_on_turn_keeps_chained_pair_and_drops_far():
  state = _car_state()
  state.vehicleNaviCanControl = 1
  state.vehicleNaviPosition4deSeen = True
  state.vehicleNaviRoadHorizon = [(0.0, 6)]
  # 37m 간격 쌍(97m, 134m) + 180m 상한 초과(195m) + 먼 잔여물(300m)
  for off in (97.0, 134.0, 195.0, 300.0):
    state._add_vehicle_navi_event("bump", 0, 6, off)
  state.vehicleNaviGv80SteerHistory.extend([10.0, 150.0, 80.0])

  cp = SimpleNamespace(
    dat={0x4B4: _make_4b4_horizon(0, 0, 6)},
    ts_nanos={"NEW_MSG_4B4": {"POS_OFFSET": 1_000_000_000}},
    _last_update_nanos=1_000_000_000,
  )
  ret = SimpleNamespace(vEgo=5.0 / 3.6, steeringAngleDeg=80.0, speedLimit=0.0, speedBumpDistance=0.0, schoolZoneActive=False)
  state._update_vehicle_navi_events(cp, ret, False)
  assert [e["target"] for e in state.vehicleNaviEvents] == [97.0, 134.0]
  assert ret.speedBumpDistance == pytest.approx(97.0)

  # 첫째 통과 후 둘째(134m)로 정상 릴레이
  state.totalDistance = 98.0
  ret2 = SimpleNamespace(vEgo=5.0 / 3.6, steeringAngleDeg=0.0, speedLimit=0.0, speedBumpDistance=0.0, schoolZoneActive=False)
  state._update_vehicle_navi_events(cp, ret2, False)
  assert ret2.speedBumpDistance == pytest.approx(36.0)


@pytest.mark.parametrize("mode,bit35_on,expected_dist", [
  (1, False, 200.0),
  (2, False, 0.0),
  (2, True, 200.0),
  (3, False, 0.0),
  (3, True, 200.0),
])
def test_gv80_frc5_bump_respects_all_can_control_modes(monkeypatch, mode, bit35_on, expected_dist):
  state, parsers = _make_real_canfd_navi(monkeypatch, CAR.HYUNDAI_IONIQ_5)
  state.vehicleNaviCanControl = mode
  pt = parsers[Bus.pt]
  bus = pt.bus
  state.controls_ready_count = 122

  byte4 = (0x08 | 5) if bit35_on else 5
  seg_frc5 = ((8 << 13) | (byte4 << 32)).to_bytes(8, "little")
  frames = [
    (0x4DE, bytes.fromhex("0200014070730430"), bus),
    (0x4B5, seg_frc5, bus),
    (0x4B8, _make_4b8_bump(200, path=8), bus),
  ]
  pt.update([1_000_000_000, frames])
  state.monitor_fingerprint(parsers, True)
  pt.update([1_100_000_000, frames])
  ret = SimpleNamespace(vEgo=25.0 / 3.6, steeringAngleDeg=0.0, speedLimit=0.0, speedBumpDistance=0.0, schoolZoneActive=False)
  state._update_vehicle_navi_events(pt, ret, False)
  assert ret.speedBumpDistance == pytest.approx(expected_dist)


def test_section_control_off_clears_section_and_cam_after_entry_pass():
  state = _car_state()
  state.vehicleNaviCanControl = True
  state.speedcamSectionAvgControl = False
  state.navi_position_4de = {"POS_OFFSET": 6, "POS_PATH_INDEX": 8, "POS_RANGE_AVG_SPEED": 0}
  cp = SimpleNamespace(
    dat={0x4DE: bytes.fromhex("06000140808ac8ad")},
    ts_nanos={"NEW_MSG_4DE": {"POS_RANGE_AVG_SPEED": 1_000_000_000}},
    _last_update_nanos=1_050_000_000,
  )

  # 1) 구간 진입 전: 진입 카메라 접근 중에는 정상 감속(가상거리 660m)
  state.totalDistance = 0.0
  ret = SimpleNamespace(vEgo=30.0, speedLimit=110.0, speedBumpDistance=0.0, schoolZoneActive=False)
  state._update_vehicle_navi_events(cp, ret, True)
  state.update_speed_limit(ret, True, distance_time_changed=False)
  assert not ret.vehicleNaviSectionActive
  assert ret.speedLimitDistance > 600.0

  # 2) 진입 카메라 통과 후 구간 진입(0x4DE avg=115, 0x4A3 경고는 계속 ON):
  #    설정 OFF이므로 구간 제어도 꺼지고, 1.0m 가상거리 cam 고정도 즉시 해제된다.
  state.totalDistance = 700.0
  state.navi_position_4de["POS_RANGE_AVG_SPEED"] = 115
  cp.ts_nanos["NEW_MSG_4DE"]["POS_RANGE_AVG_SPEED"] = 2_000_000_000
  cp._last_update_nanos = 2_050_000_000
  ret2 = SimpleNamespace(vEgo=30.0, speedLimit=110.0, speedBumpDistance=0.0, schoolZoneActive=False)
  state._update_vehicle_navi_events(cp, ret2, True)
  state.update_speed_limit(ret2, True, distance_time_changed=False)
  assert state.vehicleNaviSpeedZoneActive
  assert not ret2.vehicleNaviSectionActive
  assert not ret2.vehicleNaviActive
  assert ret2.speedLimitDistance == 0.0


def test_section_control_off_decelerates_only_for_upcoming_camera_then_clears():
  state = _car_state()
  state.vehicleNaviCanControl = True
  state.speedcamSectionAvgControl = False
  state.navi_position_4de = {"POS_OFFSET": 6, "POS_PATH_INDEX": 8, "POS_RANGE_AVG_SPEED": 110}
  cp = SimpleNamespace(
    dat={0x4DE: bytes.fromhex("0600d94d008cc8a5")},
    ts_nanos={"NEW_MSG_4DE": {"POS_RANGE_AVG_SPEED": 1_000_000_000}},
    _last_update_nanos=1_050_000_000,
  )

  # 구간 내부에서 전방 300m 종료 카메라가 발견된 경우: 발견 시점부터 통과 시점까지만 cam 감속
  state.totalDistance = 5000.0
  state.vehicleNaviEvents = [{"type": "camera", "speed": 110, "kind": 1, "target": 5300.0}]
  ret = SimpleNamespace(vEgo=30.0, speedLimit=110.0, speedBumpDistance=0.0, schoolZoneActive=False)
  state._update_vehicle_navi_events(cp, ret, True)
  state.update_speed_limit(ret, True, distance_time_changed=False)
  assert not ret.vehicleNaviSectionActive
  assert ret.vehicleNaviActive
  assert ret.speedLimitDistance == pytest.approx(300.0 - 30.0 * DT_CTRL)

  # 종료 카메라를 지나간 직후: 0x4A3 경고와 0x4DE가 남아 있어도 즉시 0.0으로 꺼진다.
  state.totalDistance = 5305.0
  cp.ts_nanos["NEW_MSG_4DE"]["POS_RANGE_AVG_SPEED"] = 2_000_000_000
  cp._last_update_nanos = 2_050_000_000
  ret2 = SimpleNamespace(vEgo=30.0, speedLimit=110.0, speedBumpDistance=0.0, schoolZoneActive=False)
  state._update_vehicle_navi_events(cp, ret2, True)
  state.update_speed_limit(ret2, True, distance_time_changed=False)
  assert not ret2.vehicleNaviSectionActive
  assert not ret2.vehicleNaviActive
  assert ret2.speedLimitDistance == 0.0


def test_section_control_toggle_mid_section_immediately_updates_state():
  state = _car_state()
  state.vehicleNaviCanControl = True
  state.speedcamSectionAvgControl = True
  state.navi_position_4de = {"POS_OFFSET": 6, "POS_PATH_INDEX": 8, "POS_RANGE_AVG_SPEED": 115}
  cp = SimpleNamespace(
    dat={0x4DE: bytes.fromhex("0600794ec08ac8ad")},
    ts_nanos={"NEW_MSG_4DE": {"POS_RANGE_AVG_SPEED": 1_000_000_000}},
    _last_update_nanos=1_050_000_000,
  )

  # ON 상태: 구간 제어 활성
  ret_on = SimpleNamespace(vEgo=30.0, speedLimit=110.0, speedBumpDistance=0.0, schoolZoneActive=False)
  state._update_vehicle_navi_events(cp, ret_on, True)
  assert ret_on.vehicleNaviSectionActive
  assert ret_on.vehicleNaviSpeed == 110.0

  # 구간 중 OFF 전환: 즉시 구간 제어 및 cam 거리 해제
  state.speedcamSectionAvgControl = False
  ret_off = SimpleNamespace(vEgo=30.0, speedLimit=110.0, speedBumpDistance=0.0, schoolZoneActive=False)
  state._update_vehicle_navi_events(cp, ret_off, True)
  state.update_speed_limit(ret_off, True, distance_time_changed=False)
  assert not ret_off.vehicleNaviSectionActive
  assert not ret_off.vehicleNaviActive
  assert ret_off.speedLimitDistance == 0.0

  # 다시 ON 전환: 구간 내부이므로 즉시 구간 제어 재활성화
  state.speedcamSectionAvgControl = True
  ret_on2 = SimpleNamespace(vEgo=30.0, speedLimit=110.0, speedBumpDistance=0.0, schoolZoneActive=False)
  state._update_vehicle_navi_events(cp, ret_on2, True)
  assert ret_on2.vehicleNaviSectionActive
  assert ret_on2.vehicleNaviSpeed == 110.0
