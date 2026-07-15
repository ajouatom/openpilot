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


def _serv():
  serv = CarrotServ.__new__(CarrotServ)
  serv.carrot_navi_session_id = ""
  serv.carrot_navi_speed_sequence = -1
  serv.carrot_navi_current_sequence = -1
  serv.carrot_navi_next_sequence = -1
  serv.carrot_navi_active = False
  serv.carrot_navi_has_control = False
  serv.carrot_navi_road_limit_valid = False
  serv.carrot_navi_off_route = False
  serv.active_count = 0
  serv.active_sdi_count = 0
  serv.active_sdi_count_max = 200
  serv.autoNaviSpeedCtrlMode = 3
  serv.autoNaviSpeedSafetyFactor = 1.0
  serv.autoNaviSpeedBumpSpeed = 20
  serv.roadcate = 8
  serv.nRoadLimitSpeed = 30

  defaults = {
    "nSdiType": -1,
    "nSdiSpeedLimit": 0,
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


def test_disconnect_clears_new_navi_control_immediately():
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
