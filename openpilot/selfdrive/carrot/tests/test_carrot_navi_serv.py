import pytest

from types import SimpleNamespace

from openpilot.common.constants import CV
from openpilot.selfdrive.carrot.carrot_serv import CarrotServ


def _meta(sequence, present=True):
  return {"present": present, "sequence": sequence}


class _SubMaster:
  def __init__(self, data):
    self.data = data
    self.alive = {"carrotNavi": True}
    self.valid = {"carrotNavi": True}
    self.updated = {"carrotNavi": True}

  def __getitem__(self, service):
    assert service == "carrotNavi"
    return self.data


class _MemoryParams:
  def __init__(self):
    self.writes = []
    self.removed = []

  def put_nonblocking(self, key, value):
    self.writes.append((key, value))

  def remove(self, key):
    self.removed.append(key)


def _serv():
  serv = CarrotServ.__new__(CarrotServ)
  serv.carrot_navi_session_id = ""
  serv.carrot_navi_speed_sequence = -1
  serv.carrot_navi_current_sequence = -1
  serv.carrot_navi_next_sequence = -1
  serv.carrot_navi_vehicle_sequence = -1
  serv.carrot_navi_route_sequence = -1
  serv.carrot_navi_active = False
  serv.carrot_navi_has_control = False
  serv.carrot_navi_road_limit_valid = False
  serv.carrot_navi_off_route = False
  serv.carrot_navi_traffic_active = False
  serv.carrot_navi_control = None
  serv.active_count = 0
  serv.active_sdi_count = 0
  serv.active_sdi_count_max = 200
  serv.carrotIndex = 0
  serv.nRoadLimitSpeed_counter = 0
  serv.active_kisa_count = 0
  serv.autoNaviSpeedCtrlMode = 3
  serv.vehicleNaviCanControl = True
  serv.autoNaviSpeedSafetyFactor = 1.0
  serv.autoNaviSpeedBumpSpeed = 20
  serv.is_metric = True
  serv.roadcate = 8
  serv.nRoadLimitSpeed = 30
  serv.params_memory = _MemoryParams()

  defaults = {
    "nSdiType": -1,
    "nSdiSpeedLimit": 0,
    "nSdiSection": -1,
    "nSdiDist": 0,
    "nSdiBlockType": -1,
    "nSdiBlockSpeed": 0,
    "nSdiBlockDist": 0,
    "nSdiPlusType": -1,
    "nSdiPlusSpeedLimit": 0,
    "nSdiPlusDist": 0,
    "nSdiPlusBlockType": -1,
    "nSdiPlusBlockSpeed": 0,
    "nSdiPlusBlockDist": 0,
    "xSpdType": -1,
    "xSpdLimit": 0,
    "xSpdDist": 0,
    "nTBTDist": 0,
    "nTBTTurnType": -1,
    "nTBTDistNext": 0,
    "nTBTTurnTypeNext": -1,
    "nTBTNextRoadWidth": 0,
    "szTBTMainText": "",
    "szNearDirName": "",
    "szFarDirName": "",
    "szTBTMainTextNext": "",
    "xTurnInfo": -1,
    "xDistToTurn": 0,
    "xTurnInfoNext": -1,
    "xDistToTurnNext": 0,
    "navType": "invalid",
    "navModifier": "",
    "navTypeNext": "invalid",
    "navModifierNext": "",
    "nGoPosDist": 0,
    "nGoPosTime": 0,
    "vpPosPointLatNavi": 0.0,
    "vpPosPointLonNavi": 0.0,
    "nPosAngle": 0.0,
    "nPosSpeed": 0.0,
    "szPosRoadName": "",
    "last_update_gps_time_navi": 0.0,
    "last_calculate_gps_time": 0.0,
  }
  for name, value in defaults.items():
    setattr(serv, name, value)
  return serv


def _message():
  return {
    "schemaVersion": 1,
    "connected": True,
    "sessionId": "session",
    "speed": {
      "meta": _meta(1),
      "roadLimitValid": True,
      "roadLimitKph": 80,
      "sdiPresent": True,
      "sdiType": 1,
      "sdiDistanceM": 420,
      "sdiSpeedLimitKph": 60,
    },
    "guidanceCurrent": {"meta": _meta(2), "distanceM": 131, "turnType": 117},
    "guidanceNext": {"meta": _meta(3), "distanceM": 3339, "turnType": 104},
    "laneCurrent": {"meta": _meta(1, present=False)},
    "navigationStatus": {"meta": _meta(1, present=False)},
  }


def test_applies_new_navi_control_without_resetting_distance_on_heartbeat():
  serv = _serv()
  sm = _SubMaster(_message())

  assert serv._update_carrot_navi(sm)
  assert (serv.nRoadLimitSpeed, serv.xSpdType, serv.xSpdLimit, serv.xSpdDist) == (80, 1, 60, 420)
  assert (serv.xTurnInfo, serv.xDistToTurn) == (4, 131)
  assert (serv.xTurnInfoNext, serv.xDistToTurnNext) == (4, 3470)

  serv.xSpdDist = 400
  serv.xDistToTurn = 100
  serv.xDistToTurnNext = 3400
  assert serv._update_carrot_navi(sm)
  assert (serv.xSpdDist, serv.xDistToTurn, serv.xDistToTurnNext) == (400, 100, 3400)


def test_disconnect_clears_7714_control_state():
  serv = _serv()
  data = _message()
  sm = _SubMaster(data)
  assert serv._update_carrot_navi(sm)

  data["connected"] = False
  assert not serv._update_carrot_navi(sm)
  assert not serv.carrot_navi_active
  assert serv.active_count == 0
  assert serv.xSpdType == -1
  assert serv.xTurnInfo == -1


def test_active_section_uses_existing_section_speed_control():
  serv = _serv()
  data = _message()
  data["speed"] = {
    "meta": _meta(4),
    "roadLimitValid": True,
    "roadLimitKph": 100,
    "sectionPresent": True,
    "sectionActive": True,
    "sectionSpeedLimitKph": 80,
    "sectionRemainingDistanceM": 2345.6,
  }

  assert serv._update_carrot_navi(_SubMaster(data))
  assert serv.xSpdType == 4
  assert serv.xSpdLimit == 80
  assert serv.xSpdDist == 2346


def test_applies_7714_vehicle_route_traffic_and_secondary_sdi():
  serv = _serv()
  data = _message()
  data.update({
    "vehicle": {
      "meta": _meta(10),
      "latitude": 37.5,
      "longitude": 127.1,
      "headingDeg": 92.0,
      "speedKph": 42.5,
      "roadName": "Test road",
    },
    "speed": {
      "meta": _meta(11),
      "roadLimitValid": True,
      "roadLimitKph": 50,
      "sdiPresent": True,
      "sdiType": 1,
      "sdiDistanceM": 420,
      "sdiSpeedLimitKph": 50,
      "sdiBlockType": 2,
      "sdiBlockSpeedKph": 40,
      "sdiBlockDistanceM": 390,
      "secondarySdiPresent": True,
      "secondarySdiType": 22,
      "secondarySdiDistanceM": 93,
    },
    "route": {
      "meta": _meta(12),
      "remainingDistanceM": 12500,
      "remainingTimeSec": 1320,
      "polyline": [{"latitude": 37.5, "longitude": 127.1}],
    },
    "trafficSignal": {
      "meta": _meta(13),
      "visible": True,
      "distanceM": 145,
      "source": "ssinf",
      "redValid": True,
      "redOn": True,
      "redRemainSec": 18,
    },
    "navigationStatus": {"meta": _meta(14), "guidanceActive": True},
  })

  sm = _SubMaster(data)
  assert serv._update_carrot_navi(sm)
  assert (serv.vpPosPointLatNavi, serv.vpPosPointLonNavi, serv.nPosAngle) == (37.5, 127.1, 92.0)
  assert serv.szPosRoadName == "Test road"
  assert serv.last_update_gps_time_navi > 0
  assert (serv.nGoPosDist, serv.nGoPosTime) == (12500, 1320)
  assert (serv.nSdiBlockType, serv.nSdiBlockSpeed, serv.nSdiBlockDist) == (2, 40, 390)
  assert (serv.nSdiPlusType, serv.nSdiPlusDist) == (22, 93)
  assert serv.params_memory.writes[-1][0] == "TrafficLight"
  assert '"lamp": "red"' in serv.params_memory.writes[-1][1]

  first_gps_update = serv.last_update_gps_time_navi
  assert serv._update_carrot_navi(sm)
  assert serv.last_update_gps_time_navi >= first_gps_update
  assert len(serv.params_memory.writes) == 2

  data["connected"] = False
  assert not serv._update_carrot_navi(sm)
  assert serv.last_update_gps_time_navi == 0
  assert serv.szPosRoadName == ""
  assert (serv.nGoPosDist, serv.nGoPosTime) == (0, 0)
  assert serv.params_memory.removed[-1] == "TrafficLight"


def test_legacy_7713_navigation_update_path_remains_operational():
  serv = _serv()
  legacy = {
    "nRoadLimitSpeed": 50,
    "nSdiType": 1,
    "nSdiSpeedLimit": 40,
    "nSdiSection": 0,
    "nSdiDist": 240,
    "nSdiBlockType": 2,
    "nSdiBlockSpeed": 40,
    "nSdiBlockDist": 210,
    "nSdiPlusType": 22,
    "nSdiPlusSpeedLimit": 20,
    "nSdiPlusDist": 80,
    "nTBTDist": 310,
    "nTBTTurnType": 12,
    "szTBTMainText": "Legacy left",
    "szNearDirName": "Legacy near",
    "szFarDirName": "Legacy far",
    "nTBTDistNext": 940,
    "nTBTTurnTypeNext": 13,
    "nGoPosDist": 8700,
    "nGoPosTime": 720,
    "szPosRoadName": "Legacy road",
    "vpPosPointLat": 37.4,
    "vpPosPointLon": 126.9,
    "nPosAngle": 183.0,
    "nPosSpeed": 31.0,
    "roadcate": 6,
  }
  for _ in range(7):
    serv.update(legacy)

  assert serv.nRoadLimitSpeed == 50
  assert (serv.nSdiType, serv.nSdiDist, serv.nSdiPlusType) == (1, 240, 22)
  assert (serv.nTBTDist, serv.nTBTTurnType, serv.nTBTDistNext) == (310, 12, 940)
  assert (serv.nGoPosDist, serv.nGoPosTime) == (8700, 720)
  assert (serv.szPosRoadName, serv.vpPosPointLatNavi, serv.vpPosPointLonNavi) == ("Legacy road", 37.4, 126.9)


@pytest.mark.parametrize(
  ("is_metric", "report_id", "road_limit", "alert_distance", "expected_type", "expected_limit_kph", "expected_distance_m"),
  (
    (True, "camera", 50, "300 m", 101, 52.5, 300),
    (False, "police", 25, "1000 ft", 100, 25 * CV.MPH_TO_KPH * 1.05, 304),
  ),
)
def test_waze_alert_uses_navi_speed_limit_ratio(is_metric, report_id, road_limit, alert_distance,
                                                expected_type, expected_limit_kph, expected_distance_m):
  serv = _serv()
  serv.is_metric = is_metric
  serv.autoNaviSpeedSafetyFactor = 1.05

  serv.update_kisa({
    "kisawazeroadspdlimit": road_limit,
    "kisawazereportid": report_id,
    "kisawazealertdist": alert_distance,
  })

  assert serv.xSpdType == expected_type
  assert serv.xSpdLimit == pytest.approx(expected_limit_kph)
  assert serv.xSpdDist == expected_distance_m


def test_waze_alert_without_road_limit_has_no_speed_target():
  serv = _serv()
  serv.nRoadLimitSpeed = 0
  serv.autoNaviSpeedSafetyFactor = 1.05

  serv.update_kisa({
    "kisawazereportid": "camera",
    "kisawazealertdist": "200 m",
  })

  assert serv.xSpdType == 101
  assert serv.xSpdLimit == 0
  assert serv.xSpdDist == 200


def test_vehicle_navi_speed_bump_requires_both_settings_and_distance():
  serv = _serv()
  car_state = SimpleNamespace(speedBumpDistance=120.0)

  assert serv._vehicle_speed_bump_enabled(car_state)

  serv.vehicleNaviCanControl = False
  assert not serv._vehicle_speed_bump_enabled(car_state)

  serv.vehicleNaviCanControl = True
  serv.autoNaviSpeedCtrlMode = 1
  assert not serv._vehicle_speed_bump_enabled(car_state)

  serv.autoNaviSpeedCtrlMode = 2
  car_state.speedBumpDistance = 0.0
  assert not serv._vehicle_speed_bump_enabled(car_state)
