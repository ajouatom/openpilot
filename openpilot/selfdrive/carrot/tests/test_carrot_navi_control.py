from openpilot.selfdrive.carrot.carrot_navi_control import parse_carrot_navi_control


def _meta(sequence=1, present=True):
  return {"present": present, "sequence": sequence}


def _message(**overrides):
  message = {
    "schemaVersion": 1,
    "connected": True,
    "sessionId": "session",
    "speed": {"meta": _meta()},
    "guidanceCurrent": {"meta": _meta(present=False)},
    "guidanceNext": {"meta": _meta(present=False)},
    "laneCurrent": {"meta": _meta(present=False)},
    "navigationStatus": {"meta": _meta(present=False)},
  }
  message.update(overrides)
  return message


def test_parses_speed_sdi_and_guidance_for_control():
  control = parse_carrot_navi_control(_message(
    speed={
      "meta": _meta(7),
      "roadLimitValid": True,
      "roadLimitKph": 80,
      "sdiPresent": True,
      "sdiType": 1,
      "sdiDistanceM": 420,
      "sdiSpeedLimitKph": 60,
    },
    guidanceCurrent={
      "meta": _meta(8),
      "distanceM": 131,
      "turnType": 117,
      "mainText": "Anseong",
    },
    guidanceNext={
      "meta": _meta(9),
      "distanceM": 3339,
      "turnType": 104,
    },
  ))

  assert control is not None
  assert control.speed.sequence == 7
  assert control.speed.road_limit_kph == 80
  assert control.speed.sdi_type == 1
  assert control.speed.sdi_distance_m == 420
  assert control.speed.sdi_speed_limit_kph == 60
  assert control.current.turn_type == 117
  assert control.current.distance_m == 131
  assert control.next.turn_type == 104


def test_valid_section_control_takes_only_active_route_data():
  control = parse_carrot_navi_control(_message(speed={
    "meta": _meta(2),
    "sectionPresent": True,
    "sectionActive": True,
    "sectionSpeedLimitKph": 80,
    "sectionRemainingDistanceM": 2345.6,
  }))

  assert control is not None
  assert control.speed.section_active
  assert control.speed.section_speed_limit_kph == 80
  assert control.speed.section_remaining_distance_m == 2346

  suspended = parse_carrot_navi_control(_message(speed={
    "meta": _meta(3),
    "sectionPresent": True,
    "sectionActive": True,
    "sectionSpeedLimitKph": 80,
    "sectionRemainingDistanceM": 1000,
    "sectionSuspended": True,
  }))
  assert suspended is not None and not suspended.speed.section_active


def test_off_route_suppresses_distance_based_control():
  control = parse_carrot_navi_control(_message(
    speed={
      "meta": _meta(4),
      "roadLimitValid": True,
      "roadLimitKph": 50,
      "sdiPresent": True,
      "sdiType": 1,
      "sdiDistanceM": 100,
      "sdiSpeedLimitKph": 30,
    },
    guidanceCurrent={"meta": _meta(4), "distanceM": 90, "turnType": 12},
    navigationStatus={"meta": _meta(4), "offRoute": True},
  ))

  assert control is not None
  assert control.speed.road_limit_kph == 50
  assert not control.speed.sdi_present
  assert not control.current.present


def test_disconnected_or_wrong_schema_is_rejected():
  assert parse_carrot_navi_control(_message(connected=False)) is None
  assert parse_carrot_navi_control(_message(schemaVersion=2)) is None
