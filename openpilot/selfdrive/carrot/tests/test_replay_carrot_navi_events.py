from types import SimpleNamespace

from openpilot.selfdrive.carrot.server.features.dashcam.replay_events import (
  REPLAY_EVENT_INDEX_VERSION,
  ReplayEventIndexer,
)


def ns(**values):
  return SimpleNamespace(**values)


def meta(present=True):
  return ns(present=present)


def guidance(distance_m, turn_type=12, title="우회전", road_name="테스트로", *, present=True):
  return ns(
    meta=meta(present),
    distanceM=distance_m,
    turnType=turn_type,
    mainText=title,
    roadName=road_name,
    pointValid=True,
    latitude=37.1 + turn_type / 10000,
    longitude=127.1,
  )


def navi_state(
  *,
  generation=1,
  current_distance=250,
  current_turn=12,
  next_present=True,
  off_route=False,
  lane_available=(1, 1, 0),
  sdi_present=True,
  section_active=True,
  signal_green=True,
  crossroad_visible=True,
  session_id="route-a",
):
  return ns(
    connected=True,
    sessionId=session_id,
    generation=generation,
    navigationStatus=ns(guidanceActive=True, routePresent=True, offRoute=off_route),
    guidanceCurrent=guidance(current_distance, current_turn),
    guidanceNext=guidance(600, 13, "다음 좌회전", "다음로", present=next_present),
    laneCurrent=ns(
      meta=meta(), visible=True, count=3, currentLane=2, turnCode=1,
      available=list(lane_available), distanceM=current_distance,
    ),
    speed=ns(
      roadLimitValid=True, roadLimitKph=60,
      sdiPresent=sdi_present, sdiType=1, sdiSpeedLimitKph=60, sdiDistanceM=180,
      sdiSectionType=0, sdiBlockType=0,
      secondarySdiPresent=False, secondarySdiType=0, secondarySdiSpeedLimitKph=0,
      secondarySdiDistanceM=0, secondarySdiSectionType=0, secondarySdiBlockType=0,
      sectionPresent=section_active, sectionActive=section_active,
      sectionSpeedLimitKph=80, sectionAverageKph=72.5, sectionRemainingDistanceM=1400.0,
      sectionSuspended=False, sectionOffRoute=False,
    ),
    trafficSignal=ns(
      visible=True, distanceM=90,
      redValid=True, redOn=not signal_green,
      leftValid=False, leftOn=False,
      greenValid=True, greenOn=signal_green,
      rightValid=False, rightOn=False,
      uturnValid=False, uturnOn=False,
    ),
    crossroad=ns(visible=crossroad_visible, distanceM=120, imageCode=7),
  )


def ingest(indexer, mono_ns, value):
  event = ns(valid=True, logMonoTime=mono_ns, carrotNavi=value)
  indexer.ingest(event, "carrotNavi")


def event_types(indexer, base_ns, duration_ms=10_000):
  return [event["type"] for event in indexer.finalize(base_ns, duration_ms)]


def test_initial_carrot_navi_snapshot_emits_semantic_replay_markers_once():
  assert REPLAY_EVENT_INDEX_VERSION == 2
  base_ns = 10_000_000_000
  indexer = ReplayEventIndexer()
  ingest(indexer, base_ns, navi_state())
  ingest(indexer, base_ns + 500_000_000, navi_state(current_distance=240))

  assert event_types(indexer, base_ns) == [
    "navigation_active",
    "navigation_maneuver_current",
    "navigation_approach",
    "navigation_maneuver_next",
    "lane_guidance_shown",
    "speed_alert_shown",
    "section_control_started",
    "traffic_signal_shown",
    "crossroad_guidance_shown",
  ]


def test_carrot_navi_distance_bands_and_state_changes_are_distinct_events():
  base_ns = 20_000_000_000
  indexer = ReplayEventIndexer()
  ingest(indexer, base_ns, navi_state(current_distance=320))
  ingest(indexer, base_ns + 500_000_000, navi_state(current_distance=295))
  ingest(indexer, base_ns + 1_000_000_000, navi_state(current_distance=195))
  changed = navi_state(
    generation=2,
    current_distance=180,
    current_turn=14,
    next_present=False,
    off_route=True,
    lane_available=(0, 1, 1),
    sdi_present=False,
    section_active=False,
    signal_green=False,
    crossroad_visible=False,
    session_id="route-b",
  )
  changed.speed.roadLimitKph = 50
  ingest(indexer, base_ns + 1_500_000_000, changed)

  events = indexer.finalize(base_ns, 10_000)
  types = [event["type"] for event in events]
  approach_thresholds = [
    event["params"]["thresholdM"]
    for event in events
    if event["type"] == "navigation_approach"
  ]

  assert approach_thresholds == [300, 200, 200]
  for expected in (
    "navigation_session_changed",
    "navigation_off_route",
    "navigation_maneuver_current",
    "navigation_maneuver_next_cleared",
    "lane_guidance_changed",
    "speed_alert_cleared",
    "road_speed_limit_changed",
    "section_control_ended",
    "traffic_signal_changed",
    "crossroad_guidance_hidden",
  ):
    assert expected in types

  current = next(
    event for event in events
    if event["type"] == "navigation_maneuver_current" and event["params"]["turnType"] == 14
  )
  assert current["params"]["distanceM"] == 180
  assert current["sourceTitle"] == "우회전"


def test_carrot_navi_generation_update_is_not_a_route_session_change():
  base_ns = 25_000_000_000
  indexer = ReplayEventIndexer()
  ingest(indexer, base_ns, navi_state(generation=1, session_id="route-a"))
  ingest(indexer, base_ns + 500_000_000, navi_state(generation=2, session_id="route-a"))

  events = indexer.finalize(base_ns, 10_000)
  assert not any(event["type"] == "navigation_session_changed" for event in events)
  assert all(
    event["sourceTag"] == "CarrotNavi"
    for event in events
    if event["type"].startswith(("navigation_", "lane_guidance_", "speed_alert_"))
  )


def test_carrot_navi_connection_and_route_end_transitions_are_indexed():
  base_ns = 30_000_000_000
  indexer = ReplayEventIndexer()
  first = navi_state()
  ingest(indexer, base_ns, first)

  disconnected = navi_state()
  disconnected.connected = False
  ingest(indexer, base_ns + 500_000_000, disconnected)

  types = event_types(indexer, base_ns)
  assert "navi_disconnected" in types
  assert "navigation_ended" in types
  assert "navigation_maneuver_current_cleared" in types
  assert "lane_guidance_hidden" in types
  assert "traffic_signal_hidden" in types


def test_rich_carrot_navi_events_replace_only_the_legacy_navigation_marker():
  base_ns = 40_000_000_000
  indexer = ReplayEventIndexer()
  legacy = ns(
    valid=True,
    logMonoTime=base_ns,
    carrotMan=ns(xTurnInfo=12, xDistToTurn=250, szTBTMainText="우회전", szPosRoadName="테스트로"),
  )
  indexer.ingest(legacy, "carrotMan")
  ingest(indexer, base_ns + 100_000_000, navi_state())

  types = event_types(indexer, base_ns)
  assert "navigation_maneuver" not in types
  assert "navigation_maneuver_current" in types


def test_disconnected_carrot_navi_does_not_hide_usable_legacy_navigation():
  base_ns = 50_000_000_000
  indexer = ReplayEventIndexer()
  legacy = ns(
    valid=True,
    logMonoTime=base_ns,
    carrotMan=ns(xTurnInfo=12, xDistToTurn=250, szTBTMainText="우회전", szPosRoadName="테스트로"),
  )
  indexer.ingest(legacy, "carrotMan")
  disconnected = navi_state()
  disconnected.connected = False
  ingest(indexer, base_ns + 100_000_000, disconnected)

  assert "navigation_maneuver" in event_types(indexer, base_ns)
