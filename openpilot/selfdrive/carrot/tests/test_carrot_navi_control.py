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


def test_control_rejects_encoded_or_invalid_road_limit():
  encoded = parse_carrot_navi_control(_message(speed={
    "meta": _meta(5),
    "roadLimitValid": True,
    "roadLimitKph": 1020,
  }))
  assert encoded is not None
  assert encoded.speed.road_limit_kph is None

  invalid = parse_carrot_navi_control(_message(speed={
    "meta": _meta(6),
    "roadLimitValid": True,
    "roadLimitKph": 300,
  }))
  assert invalid is not None
  assert invalid.speed.road_limit_kph is None


def test_disconnected_or_wrong_schema_is_rejected():
  assert parse_carrot_navi_control(_message(connected=False)) is None
  assert parse_carrot_navi_control(_message(schemaVersion=2)) is None


def test_parses_vehicle_route_traffic_and_secondary_sdi():
  control = parse_carrot_navi_control(_message(
    vehicle={
      "meta": _meta(10),
      "latitude": 37.5,
      "longitude": 127.1,
      "headingDeg": 361.0,
      "speedKph": 42.5,
      "roadName": "Test road",
    },
    speed={
      "meta": _meta(11),
      "sdiPresent": True,
      "sdiType": 1,
      "sdiDistanceM": 420,
      "sdiSpeedLimitKph": 50,
      "sdiSectionType": 7,
      "sdiBlockType": 2,
      "sdiBlockSpeedKph": 40,
      "sdiBlockDistanceM": 390,
      "secondarySdiPresent": True,
      "secondarySdiType": 22,
      "secondarySdiDistanceM": 93,
    },
    route={
      "meta": _meta(12),
      "remainingDistanceM": 12500,
      "remainingTimeSec": 1320,
      "polyline": [
        {"latitude": 37.5, "longitude": 127.1},
        {"latitude": 37.6, "longitude": 127.2},
      ],
    },
    trafficSignal={
      "meta": _meta(13),
      "visible": True,
      "distanceM": 145,
      "source": "ssinf",
      "redValid": True,
      "redOn": True,
      "redRemainSec": 18,
    },
    laneCurrent={"meta": _meta(15), "roadCategory": 6},
    navigationStatus={"meta": _meta(14), "guidanceActive": True},
  ))

  assert control is not None
  assert control.vehicle.present
  assert (control.vehicle.latitude, control.vehicle.longitude, control.vehicle.heading_deg) == (37.5, 127.1, 1.0)
  assert control.vehicle.road_name == "Test road"
  assert control.speed.sdi_block_distance_m == 390
  assert control.speed.secondary_sdi_present
  assert (control.speed.secondary_sdi_type, control.speed.secondary_sdi_distance_m) == (22, 93)
  assert control.route.remaining_distance_m == 12500
  assert control.route.polyline == ((37.5, 127.1), (37.6, 127.2))
  assert (control.traffic.lamp, control.traffic.remain_sec) == ("red", 18)
  assert control.road_category == 6
  assert control.guidance_active
