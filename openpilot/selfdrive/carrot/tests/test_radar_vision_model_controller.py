from dataclasses import replace
from types import SimpleNamespace

from openpilot.selfdrive.carrot.radar.radar_lead_model import RadarLeadDecision, VisionLeadContext
from openpilot.selfdrive.carrot.radar.radar_lead_runtime import RadarLeadRuntimeResult
from openpilot.selfdrive.carrot.radar.radar_trajectory_model import (
  RadarTrajectoryRuntimeResult,
  TrajectoryCutinDecision,
)
from openpilot.selfdrive.carrot.radar.tools.radar_lead_simulator import (
  Candidate,
  Selection,
  cutin_continuity_series,
  validation_review_events,
)
from openpilot.selfdrive.carrot.radar.radar_vision_model_controller import VisionModelRadarController, VisionRadarMatcher
from openpilot.selfdrive.carrot.tests.test_radar_lead_controller import prediction


def vision_model(d_rel: float, y_rel: float, v_lead: float, probability: float = 0.95):
  return SimpleNamespace(leadsV3=[SimpleNamespace(
    prob=probability,
    x=[d_rel + 1.52],
    y=[-y_rel],
    v=[v_lead],
    xStd=[1.0],
    yStd=[0.5],
    vStd=[1.0],
  )])


def test_laplacian_match_requires_all_sanity_gates() -> None:
  matcher = VisionRadarMatcher()
  sane = prediction(40, 0.2, 0.1, 0.1)
  vision = VisionLeadContext(0.95, 12.0, 0.2, 19.0, 0.0, 1.0, 0.5, 1.0)
  assert matcher.match_context(vision, (sane,), 20.0) is not None

  bad_distance = VisionLeadContext(0.95, 35.0, 0.2, 19.0, 0.0, 1.0, 0.5, 1.0)
  assert matcher.match_context(bad_distance, (sane,), 20.0) is None

  bad_lateral = VisionLeadContext(0.95, 12.0, 3.0, 19.0, 0.0, 1.0, 0.5, 1.0)
  assert matcher.match_context(bad_lateral, (sane,), 20.0) is None

  bad_velocity = VisionLeadContext(0.95, 12.0, 0.2, 10.0, 0.0, 1.0, 0.5, 1.0)
  assert matcher.match_context(bad_velocity, (sane,), 40.0) is None

  low_probability = VisionLeadContext(0.34, 12.0, 0.2, 19.0, 0.0, 1.0, 0.5, 1.0)
  assert matcher.match_context(low_probability, (sane,), 20.0) is None


def test_low_probability_hold_keeps_only_previous_sane_radar_alias() -> None:
  matcher = VisionRadarMatcher()
  selected = prediction(40, 0.2, 0.1, 0.1)
  other = prediction(41, 0.2, 0.1, 0.1)
  high = VisionLeadContext(0.95, 12.0, 0.2, 19.0, 0.0, 1.0, 0.5, 1.0)
  low = VisionLeadContext(0.43, 12.0, 0.2, 19.0, 0.0, 1.0, 0.5, 1.0)

  first = matcher.match_context(high, (selected, other), 20.0)
  assert first is not None and first.prediction.features.radar_object.front_track_id == 40
  held = matcher.match_context(low, (selected, other), 20.0)
  assert held is not None and held.prediction.features.radar_object.front_track_id == 40

  for _ in range(9):
    assert matcher.match_context(low, (selected, other), 20.0) is not None
  assert matcher.match_context(low, (selected, other), 20.0) is None


def test_previous_match_bridges_small_distance_gate_jitter() -> None:
  selected = prediction(40, 0.2, 0.1, 0.1)
  initial = VisionLeadContext(0.95, 12.0, 0.2, 19.0, 0.0, 1.0, 0.5, 1.0)
  boundary_jitter = VisionLeadContext(0.95, 17.5, 0.2, 19.0, 0.0, 1.0, 0.5, 1.0)

  fresh_matcher = VisionRadarMatcher()
  assert fresh_matcher.match_context(boundary_jitter, (selected,), 20.0) is None

  held_matcher = VisionRadarMatcher()
  assert held_matcher.match_context(initial, (selected,), 20.0) is not None
  held = held_matcher.match_context(boundary_jitter, (selected,), 20.0)
  assert held is not None and held.prediction.features.radar_object.front_track_id == 40


def test_long_range_off_path_target_does_not_match_vision() -> None:
  matcher = VisionRadarMatcher()
  ghost = prediction(46, -11.6, 0.1, 0.1, d_rel=64.0, v_lead=16.2)
  ghost = replace(ghost, features=replace(
    ghost.features,
    d_path=-4.65,
    d_path_future=-5.47,
    in_lane_prob=0.0,
    radar_object=replace(ghost.features.radar_object, front_d_rel=64.0, front_v_rel=-0.8),
  ))
  vision = VisionLeadContext(0.54, 81.0, -11.0, 16.0, 0.0, 10.0, 2.0, 3.0)

  assert matcher.match_context(vision, (ghost,), 17.0) is None


def test_fresh_close_off_path_target_does_not_match_uncertain_vision_range() -> None:
  matcher = VisionRadarMatcher()
  adjacent = prediction(55, -2.2, 0.1, 0.1, d_rel=38.15, v_lead=13.18)
  adjacent = replace(adjacent, features=replace(
    adjacent.features,
    track_age=23,
    d_path=-2.16,
    d_path_future=-2.1,
    in_lane_prob=0.0,
    radar_object=replace(
      adjacent.features.radar_object,
      front_d_rel=38.15,
      front_v_rel=-0.53,
    ),
  ))
  vision = VisionLeadContext(0.71, 47.97, -0.23, 13.70, 0.0, 16.47, 0.58, 2.32)

  assert matcher.match_context(vision, (adjacent,), 13.71) is None


def test_large_vision_velocity_std_does_not_match_stationary_front_reflection() -> None:
  matcher = VisionRadarMatcher()
  reflection = prediction(61, 1.0, 0.0, 0.0, d_rel=61.3, v_lead=0.7)
  reflection = replace(reflection, features=replace(
    reflection.features,
    track_age=23,
    d_path=1.04,
    in_lane_prob=0.7,
    radar_object=replace(
      reflection.features.radar_object,
      front_d_rel=61.3,
      front_v_rel=-14.0,
    ),
  ))
  vision = VisionLeadContext(0.86, 81.9, -0.3, 10.3, 0.0, 11.6, 0.5, 4.0)

  assert matcher.match_context(vision, (reflection,), 14.7) is None


def test_slow_front_track_can_match_stationary_vision_with_corner_corroboration() -> None:
  matcher = VisionRadarMatcher()
  corner = prediction(1012, 7.6, 0.0, 0.0, front=False, d_rel=85.1, v_lead=2.4)
  corner = replace(corner, features=replace(
    corner.features,
    track_age=14,
    d_path=0.35,
    d_path_future=-0.67,
    in_lane_prob=0.77,
  ))
  front = prediction(60, 7.3, 0.0, 0.0, d_rel=84.7, v_lead=0.2)
  front = replace(front, features=replace(
    front.features,
    track_age=18,
    d_path=0.12,
    in_lane_prob=0.92,
    radar_object=replace(front.features.radar_object, front_d_rel=84.7, front_v_rel=-21.1),
  ))
  vision = VisionLeadContext(0.66, 91.5, 8.3, 17.7, 0.0, 11.3, 2.0, 3.2)

  match = matcher.match_context(vision, (front,), 21.3, (corner,))
  assert match is not None
  assert match.prediction.features.radar_object.front_track_id == 60


def test_stationary_vision_accepts_position_matched_corner_despite_sensor_speed_disagreement() -> None:
  matcher = VisionRadarMatcher()
  front = prediction(60, 0.2, 0.0, 0.0, d_rel=54.0, v_lead=13.0)
  front = replace(front, features=replace(
    front.features,
    track_age=18,
    d_path=0.10,
    in_lane_prob=0.94,
    radar_object=replace(front.features.radar_object, front_d_rel=54.0, front_v_rel=-5.0),
  ))
  corner = prediction(1012, 0.3, 0.0, 0.0, front=False, d_rel=54.8, v_lead=1.0)
  corner = replace(corner, features=replace(
    corner.features,
    track_age=14,
    d_path=0.18,
    in_lane_prob=0.86,
  ))
  vision = VisionLeadContext(0.90, 55.0, 0.2, 17.5, 0.0, 3.0, 0.8, 1.5)

  match = matcher.match_context(vision, (front,), 18.0, (corner,))

  assert match is not None
  assert match.prediction is front


def test_stationary_vision_does_not_use_adjacent_moving_corner_as_corroboration() -> None:
  matcher = VisionRadarMatcher()
  front = prediction(60, 0.0, 0.0, 0.0, d_rel=54.0, v_lead=0.5)
  front = replace(front, features=replace(
    front.features,
    track_age=18,
    d_path=0.10,
    in_lane_prob=0.94,
    radar_object=replace(front.features.radar_object, front_d_rel=54.0, front_v_rel=-17.5),
  ))
  adjacent = prediction(1012, 1.5, 0.0, 0.0, front=False, d_rel=54.8, v_lead=17.0)
  adjacent = replace(adjacent, features=replace(
    adjacent.features,
    track_age=14,
    d_path=0.18,
    in_lane_prob=0.86,
  ))
  vision = VisionLeadContext(0.90, 55.0, 0.0, 17.5, 0.0, 3.0, 0.8, 1.5)

  assert matcher.match_context(vision, (front,), 18.0, (adjacent,)) is None


def test_stationary_vision_does_not_use_distant_moving_corner_as_corroboration() -> None:
  matcher = VisionRadarMatcher()
  front = prediction(60, 0.0, 0.0, 0.0, d_rel=54.0, v_lead=0.5)
  front = replace(front, features=replace(
    front.features,
    track_age=18,
    d_path=0.10,
    in_lane_prob=0.94,
    radar_object=replace(front.features.radar_object, front_d_rel=54.0, front_v_rel=-17.5),
  ))
  distant = prediction(1012, 0.2, 0.0, 0.0, front=False, d_rel=58.0, v_lead=17.0)
  distant = replace(distant, features=replace(
    distant.features,
    track_age=14,
    d_path=0.18,
    in_lane_prob=0.86,
  ))
  vision = VisionLeadContext(0.90, 55.0, 0.0, 17.5, 0.0, 3.0, 0.8, 1.5)

  assert matcher.match_context(vision, (front,), 18.0, (distant,)) is None


def test_stationary_front_reflection_without_corner_corroboration_stays_rejected() -> None:
  matcher = VisionRadarMatcher()
  front = prediction(60, 7.3, 0.0, 0.0, d_rel=84.7, v_lead=0.2)
  front = replace(front, features=replace(
    front.features,
    track_age=18,
    d_path=0.12,
    in_lane_prob=0.92,
    radar_object=replace(front.features.radar_object, front_d_rel=84.7, front_v_rel=-21.1),
  ))
  vision = VisionLeadContext(0.66, 91.5, 8.3, 17.7, 0.0, 11.3, 2.0, 3.2)

  assert matcher.match_context(vision, (front,), 21.3) is None


def test_high_probability_vision_can_replace_farther_previous_match() -> None:
  matcher = VisionRadarMatcher()
  farther = prediction(43, 0.41, 0.1, 0.1, d_rel=29.1, v_lead=9.2)
  closer = prediction(34, -0.07, 0.1, 0.1, d_rel=19.7, v_lead=8.5)
  farther = replace(farther, features=replace(
    farther.features,
    radar_object=replace(farther.features.radar_object, front_d_rel=29.1, front_v_rel=-0.9),
  ))
  closer = replace(closer, features=replace(
    closer.features,
    radar_object=replace(closer.features.radar_object, front_d_rel=19.7, front_v_rel=-1.5),
  ))

  initial = VisionLeadContext(0.99, 29.1, 0.41, 9.2, 0.0, 2.37, 0.25, 1.2)
  assert matcher.match_context(initial, (farther,), 10.0).prediction is farther

  motorcycle_scene = VisionLeadContext(0.993, 25.6, -0.075, 10.48, 0.0, 2.37, 0.25, 1.2)
  match = matcher.match_context(motorcycle_scene, (farther, closer), 10.0)
  assert match is not None
  assert match.prediction is closer


def test_stable_in_lane_closer_second_match_wins_for_small_target() -> None:
  matcher = VisionRadarMatcher()
  farther = prediction(43, 0.10, 0.1, 0.1, d_rel=29.4, v_lead=9.13)
  closer = prediction(34, -0.14, 0.1, 0.1, d_rel=20.3, v_lead=8.69)
  farther = replace(farther, features=replace(
    farther.features,
    radar_object=replace(farther.features.radar_object, front_d_rel=29.4, front_v_rel=-0.73),
  ))
  closer = replace(closer, features=replace(
    closer.features,
    radar_object=replace(closer.features.radar_object, front_d_rel=20.3, front_v_rel=-1.17),
  ))
  # The closer motorcycle is just outside the normal 25% distance gate, but
  # is a stable, sane second match more than halfway to the vision estimate.
  vision = VisionLeadContext(0.994, 27.94, -0.053, 10.16, 0.0, 2.54, 0.26, 1.22)

  match = matcher.match_context(vision, (farther, closer), 9.9)
  assert match is not None
  assert match.prediction is closer


def test_corner_backed_previous_match_resists_closer_second_match_jitter() -> None:
  matcher = VisionRadarMatcher()
  farther = prediction(33, -10.0, 1.0, 0.0, d_rel=84.5, v_lead=17.3)
  closer = prediction(52, -6.4, 1.0, 0.0, d_rel=62.7, v_lead=22.9)
  farther = replace(farther, features=replace(
    farther.features,
    aliases=("front:33", "corner:1007"),
    d_path=0.5,
    radar_object=replace(
      farther.features.radar_object,
      front_d_rel=84.5,
      front_v_rel=-2.7,
      corner_track_id=1007,
      corner_d_rel=84.5,
      corner_y_rel=-10.0,
      corner_v_rel=-2.7,
    ),
  ))
  closer = replace(closer, features=replace(
    closer.features,
    d_path=-0.7,
    radar_object=replace(closer.features.radar_object, front_d_rel=62.7, front_v_rel=2.9),
  ))

  initial = VisionLeadContext(0.90, 80.1, -8.9, 19.0, 0.0, 5.0, 1.0, 3.0)
  assert matcher.match_context(initial, (farther,), 20.0).prediction is farther

  jitter = VisionLeadContext(0.94, 76.3, -8.3, 18.7, 0.0, 5.0, 1.0, 3.0)
  match = matcher.match_context(jitter, (farther, closer), 20.0)
  assert match is not None
  assert match.prediction is farther


def test_corner_backed_previous_match_bridges_lateral_vision_jitter() -> None:
  matcher = VisionRadarMatcher()
  target = prediction(52, -5.7, 1.0, 0.0, d_rel=60.0, v_lead=23.4)
  target = replace(target, features=replace(
    target.features,
    aliases=("front:52", "corner:1000"),
    d_path=-1.0,
    radar_object=replace(
      target.features.radar_object,
      front_d_rel=60.0,
      front_v_rel=3.4,
      corner_track_id=1000,
      corner_d_rel=60.0,
      corner_y_rel=-5.7,
      corner_v_rel=3.4,
    ),
  ))

  initial = VisionLeadContext(0.96, 70.0, -7.0, 20.0, 0.0, 7.0, 1.2, 3.0)
  assert matcher.match_context(initial, (target,), 20.0) is not None

  lateral_jitter = VisionLeadContext(0.95, 75.8, -8.0, 19.5, 0.0, 6.6, 1.3, 3.0)
  match = matcher.match_context(lateral_jitter, (target,), 20.0)
  assert match is not None
  assert match.prediction is target


def test_scc_can_supply_vision_matched_lead_one() -> None:
  matcher = VisionRadarMatcher()
  scc = prediction(0, 0.1, 0.1, 0.1, front=False, scc=True, d_rel=20.0, v_lead=18.0)
  vision = VisionLeadContext(0.95, 20.0, 0.1, 18.0, 0.0, 1.0, 0.5, 1.0)
  match = matcher.match_context(vision, (scc,), 20.0)
  assert match is not None
  assert match.prediction.features.radar_object.scc_track_id == 0


def test_controller_trajectory_ignores_candidates_behind_lead_one() -> None:
  front = prediction(40, 0.2, 0.95, 0.1, d_rel=20.0, v_lead=19.0)
  front = replace(front, features=replace(
    front.features,
    radar_object=replace(front.features.radar_object, front_d_rel=20.0),
  ))
  near = SimpleNamespace(
    source="frontRadar", track_id=41, probability=0.95,
    point=SimpleNamespace(d_rel=15.0),
  )
  far = SimpleNamespace(
    source="frontRadar", track_id=42, probability=0.99,
    point=SimpleNamespace(d_rel=30.0),
  )
  trajectory_decision = TrajectoryCutinDecision((near, far), (), (near, far))
  trajectory_result = RadarTrajectoryRuntimeResult(
    True,
    {},
    (near, far),
    trajectory_decision,
    front_decision=trajectory_decision,
    corner_decision=trajectory_decision,
  )

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(
        True,
        RadarLeadDecision((), (), ()),
        (front,),
        0.1,
        front_predictions=(front,),
        trajectory=trajectory_result,
      )

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  output = controller.update(0.0, 20.0, (), vision_model(20.0, 0.2, 19.0))

  assert output.lead_one is not None
  assert controller.last_trajectory_result.predictions == (near, far)
  assert controller.last_trajectory_result.decision.confirmed == (near,)


def test_controller_uses_trajectory_probability_as_final_lead_two() -> None:
  primary = prediction(40, 0.2, 0.95, 0.1, d_rel=20.0, v_lead=19.0)
  primary = replace(primary, features=replace(
    primary.features,
    radar_object=replace(primary.features.radar_object, front_d_rel=20.0),
  ))
  entering = prediction(41, 2.8, 0.1, 0.1, d_rel=15.0, v_lead=18.0)
  trajectory_entry = SimpleNamespace(
    source="frontRadar",
    track_id=41,
    probability=0.96,
    point=SimpleNamespace(d_rel=15.0),
  )
  trajectory_decision = TrajectoryCutinDecision(
    (trajectory_entry,), (), (trajectory_entry,),
  )
  trajectory_result = RadarTrajectoryRuntimeResult(
    True,
    {},
    (trajectory_entry,),
    trajectory_decision,
    front_decision=trajectory_decision,
  )

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(
        True,
        RadarLeadDecision((), ()),
        (primary, entering),
        0.1,
        front_predictions=(primary, entering),
        trajectory=trajectory_result,
      )

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  output = controller.update(0.0, 20.0, (), vision_model(20.0, 0.2, 19.0))

  assert output.lead_one is not None
  assert output.lead_two is not None
  assert output.lead_two["radarTrackId"] == 41
  assert output.lead_two["modelProb"] == 0.96
  assert not output.lead_two_tentative


def test_controller_selects_nearest_cutin_then_keeps_that_lead_two_identity() -> None:
  near = prediction(41, 0.2, 0.1, 0.1, d_rel=15.0, v_lead=18.0)
  far = prediction(42, -2.8, 0.1, 0.1, d_rel=25.0, v_lead=18.0)
  state = {"near": near, "far": far}

  class Runtime:
    def update(self, *_args):
      near_prediction = state["near"]
      far_prediction = state["far"]
      trajectory_near = SimpleNamespace(
        source="frontRadar", track_id=41, probability=0.95,
        point=SimpleNamespace(d_rel=near_prediction.features.radar_object.d_rel),
      )
      trajectory_far = SimpleNamespace(
        source="frontRadar", track_id=42, probability=0.99,
        point=SimpleNamespace(d_rel=far_prediction.features.radar_object.d_rel),
      )
      # Deliberately put the farther/higher-probability object first. The
      # controller must use physical range and then retain the chosen identity.
      trajectory_decision = TrajectoryCutinDecision(
        (trajectory_far, trajectory_near), (), (trajectory_far, trajectory_near),
      )
      trajectory_result = RadarTrajectoryRuntimeResult(
        True,
        {},
        (trajectory_far, trajectory_near),
        trajectory_decision,
        front_decision=trajectory_decision,
      )
      return RadarLeadRuntimeResult(
        True,
        RadarLeadDecision((), ()),
        (near_prediction, far_prediction),
        0.1,
        front_predictions=(near_prediction, far_prediction),
        trajectory=trajectory_result,
      )

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  first = controller.update(0.0, 20.0, (), vision_model(60.0, 0.0, 19.0))
  assert first.lead_two is not None
  assert first.lead_two["radarTrackId"] == 41

  state["near"] = prediction(41, 0.2, 0.1, 0.1, d_rel=22.0, v_lead=18.0)
  state["far"] = prediction(42, -2.8, 0.1, 0.1, d_rel=10.0, v_lead=18.0)
  held = controller.update(0.1, 20.0, (), vision_model(60.0, 0.0, 19.0))
  assert held.lead_two is not None
  assert held.lead_two["radarTrackId"] == 41
  assert held.lead_cutin is not None
  assert held.lead_cutin["radarTrackId"] == 41
  assert [lead["radarTrackId"] for lead in held.leads_cutin] == [41, 42]


def test_controller_does_not_duplicate_primary_external_as_lead_two() -> None:
  shared = prediction(40, 0.2, 0.95, 0.1, external_prob=0.95)

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(
        True,
        RadarLeadDecision((shared,), (), (shared,)),
        (shared,),
        0.1,
      )

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  output = controller.update(0.0, 20.0, (), vision_model(12.0, 0.2, 19.0))
  assert output.lead_one is not None and output.lead_one["radarTrackId"] == 40
  assert output.lead_two is None


def test_controller_suppresses_external_duplicate_of_primary_by_geometry() -> None:
  primary = prediction(40, 0.0, 0.99, 0.1, d_rel=19.5, v_lead=12.0)
  primary = replace(primary, features=replace(
    primary.features,
    radar_object=replace(primary.features.radar_object, front_d_rel=19.5),
  ))
  duplicate = prediction(1080, -1.0, 0.1, 0.1, 1.0, front=False, d_rel=16.8, v_lead=12.0)

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(
        True,
        RadarLeadDecision((), (), (duplicate,)),
        (primary, duplicate),
        0.1,
        front_predictions=(primary,),
        corner_predictions=(duplicate,),
      )

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  output = controller.update(0.0, 20.0, (), vision_model(19.5, 0.0, 19.0))

  assert output.lead_one is not None
  assert output.lead_two is None
  assert output.lead_external is None


def test_controller_uses_matched_front_values_for_confirmed_corner_cutin() -> None:
  front = prediction(48, 1.0, 0.1, 0.1, d_rel=23.0, v_lead=12.0)
  front = replace(front, features=replace(
    front.features,
    radar_object=replace(
      front.features.radar_object,
      front_d_rel=23.0,
      front_v_rel=-8.0,
      a_lead=-1.2,
    ),
  ))
  corner = prediction(1048, 1.1, 0.1, 0.99, front=False, d_rel=22.0, v_lead=12.2)
  trajectory_entry = SimpleNamespace(
    source="corner235",
    track_id=1048,
    probability=0.99,
    point=SimpleNamespace(d_rel=22.0),
  )
  front_trajectory_entry = SimpleNamespace(
    source="frontRadar",
    track_id=48,
    probability=0.98,
    point=SimpleNamespace(d_rel=23.0),
  )
  trajectory_decision = TrajectoryCutinDecision(
    (front_trajectory_entry, trajectory_entry), (),
    (front_trajectory_entry, trajectory_entry),
  )
  trajectory_result = RadarTrajectoryRuntimeResult(
    True,
    {},
    (front_trajectory_entry, trajectory_entry),
    trajectory_decision,
    front_decision=TrajectoryCutinDecision(
      (front_trajectory_entry,), (), (front_trajectory_entry,),
    ),
    corner_decision=TrajectoryCutinDecision(
      (trajectory_entry,), (), (trajectory_entry,),
    ),
  )

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(
        True,
        RadarLeadDecision((), ()),
        (front, corner),
        0.1,
        front_predictions=(front,),
        corner_predictions=(corner,),
        trajectory=trajectory_result,
      )

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  output = controller.update(0.0, 20.0, (), vision_model(50.0, 0.0, 19.0))

  assert output.lead_two is not None
  assert not output.lead_two_tentative
  assert output.lead_two["radarTrackId"] == 48
  assert output.lead_two["dRel"] == 23.0
  assert output.lead_two["vLead"] == 12.0
  assert output.lead_two["aLead"] == -1.2
  assert len(output.leads_cutin) == 1


def test_controller_does_not_fallback_to_legacy_cutin_head() -> None:
  legacy_cutin = prediction(49, 2.2, 0.1, 0.99, d_rel=15.0, v_lead=18.0)

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(
        True,
        RadarLeadDecision((), (legacy_cutin,)),
        (legacy_cutin,),
        0.1,
      )

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  output = controller.update(0.0, 20.0, (), vision_model(50.0, 0.0, 19.0))

  assert output.lead_two is None
  assert output.leads_cutin == ()


def test_controller_never_uses_raw_vision_as_lead_one() -> None:
  far = prediction(40, 0.2, 0.1, 0.1)

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(True, RadarLeadDecision((), ()), (far,), 0.1)

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  output = controller.update(0.0, 20.0, (), vision_model(50.0, 0.2, 19.0))
  assert output.lead_one is None


def test_controller_uses_unmatched_in_lane_model_lead_as_stealth_lead_two() -> None:
  candidate = prediction(40, 0.2, 0.95, 0.1)
  candidate = replace(candidate, features=replace(
    candidate.features, d_path=0.1, in_lane_prob=0.9, track_age=12,
  ))

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(True, RadarLeadDecision((candidate,), ()), (candidate,), 0.1)

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  output = controller.update(0.0, 20.0, (), vision_model(50.0, 0.0, 19.0))
  assert output.lead_one is None
  assert output.lead_two is not None and output.lead_two["radarTrackId"] == 40
  assert output.lead_two["radar"]


def test_controller_rejects_out_of_lane_model_lead_as_stealth_lead_two() -> None:
  candidate = prediction(40, 2.0, 0.95, 0.1)
  candidate = replace(candidate, features=replace(
    candidate.features, d_path=1.2, in_lane_prob=0.2, track_age=12,
  ))

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(True, RadarLeadDecision((candidate,), ()), (candidate,), 0.1)

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  output = controller.update(0.0, 20.0, (), vision_model(50.0, 0.0, 19.0))
  assert output.lead_one is None
  assert output.lead_two is None


def test_controller_rejects_stationary_front_external_as_lead_two() -> None:
  ghost = prediction(40, 0.2, 0.0, 0.0, external_prob=1.0, v_lead=0.0, d_rel=22.0)

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(True, RadarLeadDecision((), (), (ghost,)), (ghost,), 0.1)

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  output = controller.update(0.0, 17.0, (), vision_model(75.0, 0.0, 18.0))

  assert output.lead_two is None
  assert output.lead_external is None


def test_controller_rejects_long_range_unmatched_external_as_lead_two() -> None:
  ghost = prediction(51, 0.2, 0.0, 0.0, external_prob=0.99, d_rel=89.0)
  ghost = replace(ghost, features=replace(
    ghost.features, d_path=0.1, d_path_future=0.2, in_lane_prob=0.9, track_age=12,
  ))

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(True, RadarLeadDecision((), (), (ghost,)), (ghost,), 0.1)

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  output = controller.update(0.0, 20.0, (), vision_model(40.0, 0.0, 18.0))

  assert output.lead_two is None
  assert output.lead_external is None


def test_controller_rejects_external_track_moving_out_of_lane() -> None:
  ghost = prediction(51, -1.0, 0.0, 0.0, external_prob=0.99, d_rel=35.0)
  ghost = replace(ghost, features=replace(
    ghost.features,
    d_path=-1.0,
    d_path_future=-2.4,
    in_lane_prob=0.45,
    track_age=12,
    radar_object=replace(ghost.features.radar_object, corner_track_id=1000),
  ))

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(True, RadarLeadDecision((), (), (ghost,)), (ghost,), 0.1)

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  output = controller.update(0.0, 20.0, (), vision_model(60.0, 0.0, 18.0))

  assert output.lead_two is None
  assert output.lead_external is None


def test_controller_rejects_stationary_unmatched_front_stealth_lead_two() -> None:
  ghost = prediction(40, 0.2, 1.0, 0.0, v_lead=0.0, d_rel=22.0)
  ghost = replace(ghost, features=replace(
    ghost.features, d_path=0.1, in_lane_prob=0.9, track_age=12,
  ))

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(True, RadarLeadDecision((ghost,), ()), (ghost,), 0.1)

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  output = controller.update(0.0, 17.0, (), vision_model(75.0, 0.0, 18.0))

  assert output.lead_two is None


def test_controller_rejects_distant_unmatched_stealth_lead_two() -> None:
  ghost = prediction(62, 0.1, 1.0, 0.0, v_lead=20.0, d_rel=57.0)
  ghost = replace(ghost, features=replace(
    ghost.features, d_path=0.1, d_path_future=0.1, in_lane_prob=0.9, track_age=12,
  ))

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(True, RadarLeadDecision((ghost,), ()), (ghost,), 0.1)

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  output = controller.update(0.0, 20.0, (), vision_model(80.0, 0.0, 18.0))

  assert output.lead_two is None


def test_controller_holds_same_physically_sane_stealth_lead_briefly() -> None:
  candidate = prediction(40, 0.2, 0.95, 0.1)
  candidate = replace(candidate, features=replace(
    candidate.features, d_path=0.1, in_lane_prob=0.9, track_age=12,
  ))

  class Runtime:
    active = True

    def update(self, *_args):
      decision = RadarLeadDecision((candidate,), ()) if self.active else RadarLeadDecision((), ())
      return RadarLeadRuntimeResult(True, decision, (candidate,), 0.1)

  controller = VisionModelRadarController()
  runtime = Runtime()
  controller.runtime = runtime
  assert controller.update(0.0, 20.0, (), vision_model(50.0, 0.0, 19.0)).lead_two is not None

  runtime.active = False
  assert controller.update(0.3, 20.0, (), vision_model(50.0, 0.0, 19.0)).lead_two is not None
  assert controller.update(0.6, 20.0, (), vision_model(50.0, 0.0, 19.0)).lead_two is None


def test_controller_moves_recent_primary_to_lead_two_during_brief_vision_mismatch() -> None:
  candidate = prediction(40, 0.2, 0.95, 0.1)
  candidate = replace(candidate, features=replace(
    candidate.features, d_path=1.5, in_lane_prob=0.1, track_age=12,
  ))

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(True, RadarLeadDecision((), ()), (candidate,), 0.1)

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  matched = controller.update(0.0, 20.0, (), vision_model(12.0, 0.2, 19.0))
  assert matched.lead_one is not None and matched.lead_one["radarTrackId"] == 40

  held = controller.update(0.5, 20.0, (), vision_model(40.0, 0.0, 19.0))
  assert held.lead_one is None
  assert held.lead_two is not None and held.lead_two["radarTrackId"] == 40
  assert controller.update(0.8, 20.0, (), vision_model(40.0, 0.0, 19.0)).lead_two is None


def test_front_path_exit_stops_only_stale_primary_hold() -> None:
  candidate = prediction(40, 0.2, 0.95, 0.1)
  candidate = replace(candidate, features=replace(
    candidate.features, d_path=1.5, in_lane_prob=0.1, track_age=12,
  ))
  path_exit = SimpleNamespace(source="frontRadar", track_id=40)
  trajectory_decision = TrajectoryCutinDecision(
    (path_exit,), (), (), (path_exit,),
  )
  trajectory_result = RadarTrajectoryRuntimeResult(
    True,
    {},
    (path_exit,),
    trajectory_decision,
    front_decision=trajectory_decision,
  )

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(
        True,
        RadarLeadDecision((), ()),
        (candidate,),
        0.1,
        trajectory=trajectory_result,
      )

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  matched = controller.update(0.0, 20.0, (), vision_model(12.0, 0.2, 19.0))
  assert matched.lead_one is not None and matched.lead_two is None

  released = controller.update(0.5, 20.0, (), vision_model(40.0, 0.0, 19.0))
  assert released.lead_one is None
  assert released.lead_two is None


def test_controller_keeps_displaced_primary_as_lead_two_during_vision_target_switch() -> None:
  near = prediction(40, 0.2, 0.95, 0.1)
  near = replace(near, features=replace(near.features, d_path=1.5, in_lane_prob=0.1, track_age=12))
  far = prediction(41, 0.2, 0.95, 0.1, d_rel=30.0)
  far = replace(far, features=replace(
    far.features,
    radar_object=replace(far.features.radar_object, front_d_rel=30.0),
    d_path=0.1,
    in_lane_prob=0.9,
    track_age=12,
  ))

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(True, RadarLeadDecision((), ()), (near, far), 0.1)

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  first = controller.update(0.0, 20.0, (), vision_model(12.0, 0.2, 19.0))
  assert first.lead_one is not None and first.lead_one["radarTrackId"] == 40

  switched = controller.update(0.1, 20.0, (), vision_model(30.0, 0.2, 19.0))
  assert switched.lead_one is not None and switched.lead_one["radarTrackId"] == 41
  assert switched.lead_two is not None and switched.lead_two["radarTrackId"] == 40


def test_review_pauses_only_when_lead_one_becomes_vision_only() -> None:
  frames = [SimpleNamespace(time_s=index * 0.1) for index in range(5)]
  selections = (
    Selection(Candidate(40, 1.0, "radar"), None),
    Selection(Candidate(-1, 1.0, "vision fallback"), None),
    Selection(Candidate(-1, 1.0, "vision fallback"), None),
    Selection(Candidate(41, 1.0, "radar"), None),
    Selection(Candidate(-1, 1.0, "vision fallback"), None),
  )

  class Selector:
    def select(self, _frame, frame_index):
      return selections[frame_index]

  events = validation_review_events(frames, Selector(), None)
  assert events == {1: ("leadOne VISION",), 4: ("leadOne VISION",)}


def test_review_pauses_when_high_probability_vision_has_no_sane_radar_match() -> None:
  frames = [
    SimpleNamespace(time_s=0.0, model_leads=(SimpleNamespace(probability=0.9),)),
    SimpleNamespace(time_s=0.1, model_leads=(SimpleNamespace(probability=0.9),)),
    SimpleNamespace(time_s=0.2, model_leads=(SimpleNamespace(probability=0.9),)),
  ]
  selections = (
    Selection(Candidate(40, 1.0, "radar"), None),
    Selection(None, None),
    Selection(None, None),
  )

  class Selector:
    def select(self, _frame, frame_index):
      return selections[frame_index]

  events = validation_review_events(frames, Selector(), None)
  assert events == {1: ("leadOne LOST (VISION)",)}


def test_review_distinguishes_initial_vision_unmatched_from_lead_one_loss() -> None:
  frames = [
    SimpleNamespace(time_s=0.0, model_leads=(SimpleNamespace(probability=0.9),)),
    SimpleNamespace(time_s=0.1, model_leads=(SimpleNamespace(probability=0.9),)),
  ]
  selections = (Selection(None, None), Selection(None, None))

  class Selector:
    def select(self, _frame, frame_index):
      return selections[frame_index]

  events = validation_review_events(frames, Selector(), None)
  assert events == {0: ("VISION UNMATCHED",)}


def test_review_pauses_when_lead_two_only_is_lost_with_vision_present() -> None:
  frames = [
    SimpleNamespace(time_s=0.0, model_leads=(SimpleNamespace(probability=0.9),)),
    SimpleNamespace(time_s=0.1, model_leads=(SimpleNamespace(probability=0.9),)),
    SimpleNamespace(time_s=0.2, model_leads=(SimpleNamespace(probability=0.9),)),
  ]
  selections = (
    Selection(None, Candidate(40, 0.9, "MLP active cutin")),
    Selection(None, None),
    Selection(None, None),
  )

  class Selector:
    def select(self, _frame, frame_index):
      return selections[frame_index]

  events = validation_review_events(frames, Selector(), None)
  assert events == {
    0: ("CUT-IN id 40", "VISION UNMATCHED"),
    1: ("leadTwo LOST (VISION)",),
  }


def test_review_does_not_pause_when_lead_one_replaces_lead_two_only() -> None:
  frames = [
    SimpleNamespace(time_s=0.0, model_leads=(SimpleNamespace(probability=0.9),)),
    SimpleNamespace(time_s=0.1, model_leads=(SimpleNamespace(probability=0.9),)),
  ]
  selections = (
    Selection(None, Candidate(40, 0.9, "MLP active cutin")),
    Selection(Candidate(40, 0.9, "radar"), None),
  )

  class Selector:
    def select(self, _frame, frame_index):
      return selections[frame_index]

  events = validation_review_events(frames, Selector(), None)
  assert events == {0: ("CUT-IN id 40", "VISION UNMATCHED")}


def test_cutin_continuity_bridges_short_lead_two_drop() -> None:
  frames = [SimpleNamespace(time_s=index * 0.1) for index in range(5)]
  cutin = Selection(None, Candidate(40, 0.9, "MLP active cutin"))
  selections = (cutin, cutin, Selection(None, None), cutin, cutin)

  class Selector:
    def select(self, _frame, frame_index):
      return selections[frame_index]

  continuity = cutin_continuity_series(frames, Selector(), bridge_gap_s=1.0, retain_s=0.0)
  assert continuity[2] is not None
  assert continuity[2].matched_frames == 2
  assert continuity[2].episode_frames == 3
  assert continuity[2].drop_runs == 1
  assert continuity[2].current_drop_s > 0.0
  assert continuity[4] is not None
  assert continuity[4].matched_frames == 4
  assert continuity[4].episode_frames == 5
  assert continuity[4].current_drop_s == 0.0
