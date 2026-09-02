import math
from types import SimpleNamespace

import pytest

from opendbc.can import CANParser
from opendbc.car import Bus
from opendbc.car.hyundai.carstate import (
  CANFD_HDA_INFO_MSG, CANFD_NAVI_PROFILE_MSG, CANFD_NAVI_STATUS_MSG, CarState,
  VEHICLE_NAVI_POSITION_TIMEOUT_NS, VEHICLE_NAVI_SCHOOL_ZONE_MAX_DISTANCE,
  VEHICLE_SPEED_CAMERA_PARAM_UPDATE_FRAMES, is_canfd_navi_camera_active,
)
from opendbc.car.hyundai.values import CAR, HyundaiFlags


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
    raise KeyError(key)

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
  state.vehicleNaviAvailable = False
  state.vehicleNaviRouteResetTimestamp = 0
  state.vehicleNaviRoadClass = 7
  state.vehicleNaviCameraTarget = None
  state.vehicleNaviCameraStatusEvent = None
  state.vehicleNaviSpeedZoneActive = False
  state.vehicleNaviSpeedZoneSpeed = 0.0
  state.vehicleNaviSchoolZoneActive = False
  state.vehicleNaviSchoolZoneStartDistance = 0.0
  state.vehicleNaviSchoolZoneUsesCameraStatus = False
  state.vehicleNaviZoneControlSupported = True
  state.navi_profile_msg = "NEW_MSG_4BE"
  state.navi_segment_4b9 = None
  state.navi_position_4b4 = None
  state.navi_profile_4be = None
  state.hda_info_4a3 = None
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


def test_vehicle_navi_segment_decodes_functional_road_class():
  raw = (1 << 24) | (1 << 22) | 123
  segment = CarState._decode_vehicle_navi_segment({
    f"BYTE_{i + 1}": byte for i, byte in enumerate(raw.to_bytes(8, "little"))
  })

  assert segment == {"offset": 123, "path_index": 0, "calculated_route": 1, "functional_road_class": 1}


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
  cp = SimpleNamespace(ts_nanos={"NEW_MSG_4BE": {"PROLONG_VALUE": 1}})
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
  cp = SimpleNamespace(ts_nanos={"NEW_MSG_4BE": {"PROLONG_VALUE": 1}})
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
  cp = SimpleNamespace(ts_nanos={"NEW_MSG_4B9": {"BYTE_1": 1}})
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
  cp = SimpleNamespace(ts_nanos={"NEW_MSG_4BE": {"PROLONG_VALUE": 1}})
  ret = SimpleNamespace(speedLimit=100.0, speedBumpDistance=0.0, schoolZoneActive=False)

  assert not state._update_vehicle_navi_events(cp, ret, False)
  assert ret.speedBumpDistance == 0.0
  assert state.vehicleNaviEvents == []


def test_vehicle_navi_pending_speed_bump_is_cleared_on_expressway_entry():
  state = _car_state()
  state.vehicleNaviCanControl = True
  state.vehicleNaviEvents = [{"type": "bump", "speed": 0, "kind": 6, "target": 300.0}]
  state.vehicleNaviRoadClass = 2
  cp = SimpleNamespace(ts_nanos={})
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


def test_vehicle_navi_preview_remains_available_before_camera_status():
  state = _car_state()
  state.vehicleNaviCanControl = True
  camera = {"type": "camera", "speed": 50, "kind": 1, "target": 500.0}
  state.vehicleNaviEvents = [camera]
  cp = SimpleNamespace(ts_nanos={})
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
  cp = SimpleNamespace(ts_nanos={})
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
  cp = SimpleNamespace(ts_nanos={})
  ret = SimpleNamespace(speedLimit=60.0, speedBumpDistance=0.0, schoolZoneActive=False)

  assert not state._update_vehicle_navi_events(cp, ret, True)
  assert state.vehicleNaviCameraTarget is None
  assert ret.speedLimit == 60.0


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
  assert profile == {"value": 0xB0, "offset": 1996, "counter": 3, "update": 1, "profile_type": 16}
  assert CarState._classify_vehicle_navi_profile(profile) == ("camera", 50, 0)

  parser.update([1_010_000_000, [
    (0x093, bytes.fromhex("c27c44ffffffffff060000007ae30814ffffffffffffffff"), 0),
  ]])
  bump = CarState._decode_vehicle_navi_profile(parser.vl[CANFD_NAVI_PROFILE_MSG])
  assert bump == {"value": 6, "offset": 890, "counter": 3, "update": 1, "profile_type": 16}
  assert CarState._classify_vehicle_navi_profile(bump) == ("bump", 0, 6)


def test_pv5_canfd_navi_status_uses_logged_camera_pass_transition():
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
  cp = SimpleNamespace(ts_nanos={CANFD_NAVI_PROFILE_MSG: {"PROLONG_VALUE": 1}})
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
  cp = SimpleNamespace(ts_nanos={"NEW_MSG_4BE": {"PROLONG_VALUE": 1}})
  ret = SimpleNamespace(speedLimit=50.0, speedBumpDistance=0.0, schoolZoneActive=False)

  assert not state._update_vehicle_navi_events(cp, ret, True)
  assert not state.vehicleNaviSpeedZoneActive
  assert not state.vehicleNaviSchoolZoneActive
  assert not ret.vehicleNaviActive


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
