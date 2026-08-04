from dataclasses import dataclass, replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.carrot.radar_motion.predictor import (
  CUT_IN_CURRENT_SCOPE_HALF_WIDTH_M,
  FRONT_CUT_IN_MIN_DREL_M,
  IMMEDIATE_LANE_SCOPE_HALF_WIDTH_M,
  RadarMotionDecisionTracker,
  RadarMotionHistorySample,
  RadarMotionPrediction,
  RadarMotionPredictor,
  cutin_probability_at,
  model_path_point_at_s,
  model_path_y,
  prediction_sample_at,
  project_to_model_path,
  radar_motion_sensitivity,
  radar_target_velocity_in_ego_frame,
  visible_motion_points,
)
from openpilot.selfdrive.carrot.radar_motion.lead_selection import (
  DPathLeadCandidate,
  DPathLeadTwoTracker,
  cutin_can_compete_with_primary,
  dpath_control_max_d_rel,
  front_cutin_motion_supported,
  lead_duplicates_primary,
  select_dpath_lead_two,
)
from openpilot.selfdrive.carrot.radar_motion.controller import (
  DPathRadarController,
)
from openpilot.selfdrive.carrot.radar_motion.predictor import RadarMotionCutIn
from openpilot.selfdrive.carrot.radar_motion.primary import (
  FrontRadarKinematicAssociator,
  RadarPointSnapshot,
  VisionLead,
  VisionRadarMatcher,
  apply_vision_bracket_cutin_support,
  lead_from_radar_point,
  prefer_front_radar_kinematics,
  select_primary_radar_points,
  snapshot_radar_points,
)


@dataclass(frozen=True)
class Point:
  track_id: int
  d_rel: float
  y_rel: float
  v_rel: float = 0.0
  a_rel: float = 0.0
  measured: bool = True
  source: str = "frontRadar"
  v_lead: float | None = None
  a_lead: float | None = None
  yv_rel: float = 0.0
  j_lead: float = 0.0


STRAIGHT_PATH = ((0.0, 0.0), (100.0, 0.0))


def model_with_lead(
  d_rel: float,
  y_rel: float,
  velocity: float,
  probability: float = 0.9,
) -> SimpleNamespace:
  return SimpleNamespace(
    position=SimpleNamespace(x=(0.0, 100.0), y=(0.0, 0.0)),
    leadsV3=(SimpleNamespace(
      prob=probability,
      x=(d_rel + 1.52,),
      y=(-y_rel,),
      v=(velocity,),
      xStd=(2.0,),
      yStd=(0.6,),
      vStd=(1.5,),
    ),),
  )


class FixedPredictor:
  def __init__(self, prediction: SimpleNamespace) -> None:
    self.prediction = prediction

  def update(self, *args, **kwargs):
    key = self.prediction.source, self.prediction.track_id
    return {key: self.prediction}


class EmptyPredictor:
  def update(self, *args, **kwargs):
    return {}


class FixedDecisionTracker:
  def __init__(self, prediction: SimpleNamespace) -> None:
    self.prediction = prediction

  def update(self, *args, **kwargs):
    return SimpleNamespace(
      confirmed=(RadarMotionCutIn(self.prediction, 0.8),),
    )


class EmptyDecisionTracker:
  def update(self, *args, **kwargs):
    return SimpleNamespace(confirmed=())


def update_series(
  predictor: RadarMotionPredictor,
  lateral_values: tuple[float, ...],
  *,
  source: str = "frontRadar",
  track_id: int = 10,
) -> object:
  prediction = None
  for index, y_rel in enumerate(lateral_values):
    values = predictor.update(
      index * 0.1,
      (Point(track_id, 30.0, y_rel, source=source),),
      STRAIGHT_PATH,
      v_ego=10.0,
    )
    prediction = values[(source, track_id)]
  assert prediction is not None
  return prediction


def test_dpath_uses_same_frame_model_path_without_lane_or_yaw_correction() -> None:
  path = ((0.0, 0.0), (100.0, 4.0))
  predictor = RadarMotionPredictor()

  prediction = predictor.update(
    0.0,
    (Point(10, 50.0, -0.5),),
    path,
    v_ego=10.0,
  )[("frontRadar", 10)]

  assert model_path_y(path, 50.0) == pytest.approx(-2.0)
  assert prediction.d_path == pytest.approx(150.0 / (10016.0 ** 0.5))
  assert prediction.current_path_occupancy


def test_radar_velocity_removes_ego_rotation_without_changing_dpath() -> None:
  target_vx, target_vy = radar_target_velocity_in_ego_frame(
    v_lead=10.06,
    yv_rel=-0.20,
    d_rel=10.0,
    y_rel=3.0,
    yaw_rate_rad_s=0.02,
  )

  assert target_vx == pytest.approx(10.0)
  assert target_vy == pytest.approx(0.0)

  predictor = RadarMotionPredictor()
  prediction = None
  for index in range(15):
    prediction = predictor.update(
      index * 0.1,
      (
        Point(
          1010,
          10.0,
          3.0 - index * 0.05,
          source="corner235",
          v_lead=10.06,
          yv_rel=-0.20,
        ),
      ),
      STRAIGHT_PATH,
      v_ego=10.0,
      yaw_rate_rad_s=0.02,
    )[("corner235", 1010)]
  assert prediction is not None

  assert prediction.d_path == pytest.approx(2.30)
  assert prediction.reported_normal_speed == pytest.approx(0.0, abs=1e-6)
  assert prediction.cut_in_probability == pytest.approx(0.0)


def test_new_cutin_must_currently_be_within_adjacent_lane_scope() -> None:
  predictor = RadarMotionPredictor()
  prediction = None
  lateral_values = (4.00, 3.70, 3.40, 3.10)
  for index, y_rel in enumerate(lateral_values):
    prediction = predictor.update(
      index * 0.1,
      (
        Point(
          1010,
          30.0,
          y_rel,
          source="corner235",
          v_lead=10.0,
          yv_rel=-3.0,
        ),
      ),
      STRAIGHT_PATH,
      v_ego=10.0,
    )[("corner235", 1010)]
  assert prediction is not None

  assert CUT_IN_CURRENT_SCOPE_HALF_WIDTH_M == pytest.approx(3.0)
  assert abs(prediction.d_path) > CUT_IN_CURRENT_SCOPE_HALF_WIDTH_M
  assert prediction.predicted_path_overlap_s >= 0.5
  assert not prediction.cut_in_detection_allowed
  assert prediction.cut_in_probability == 0.0
  assert prediction.reason == "outside adjacent-lane CUT-IN scope"


@pytest.mark.parametrize(
  ("reported_lateral_speed", "expected"),
  ((-0.5, True), (0.5, False)),
)
def test_corner_lane_boundary_straddle_requires_directional_radar_agreement(
  reported_lateral_speed: float,
  expected: bool,
) -> None:
  predictor = RadarMotionPredictor()
  prediction = None
  for index, y_rel in enumerate((
    3.00, 2.95, 2.90, 2.85, 2.80, 2.75,
    2.70, 2.65, 2.60, 2.55, 2.50, 2.45,
  )):
    prediction = predictor.update(
      index * 0.1,
      (
        Point(
          1010,
          20.0,
          y_rel,
          source="corner235",
          v_lead=10.0,
          yv_rel=reported_lateral_speed,
        ),
      ),
      STRAIGHT_PATH,
      v_ego=10.0,
    )[("corner235", 1010)]
  assert prediction is not None

  assert not prediction.current_path_occupancy
  assert prediction.lane_boundary_directional_entry is expected
  if expected:
    assert prediction.cut_in_probability >= 0.30
    assert prediction.reason == "lane-boundary directional entry"
    assert RadarMotionDecisionTracker(threshold=0.30).update(
      1.1, (prediction,),
    ).confirmed
  else:
    assert prediction.cut_in_probability < 0.30


def test_projection_uses_path_arc_distance_normal_distance_and_tangent() -> None:
  path = ((0.0, 0.0), (10.0, 0.0), (20.0, -10.0))
  offset = 2.0 / (2.0 ** 0.5)
  point = (15.0 - offset, 5.0 + offset)

  projection = project_to_model_path(path, *point)
  reconstructed = model_path_point_at_s(
    path,
    projection.path_s,
    projection.d_path,
  )

  assert projection.path_s == pytest.approx(10.0 + 5.0 * 2.0 ** 0.5)
  assert projection.d_path == pytest.approx(2.0)
  assert projection.tangent_x == pytest.approx(1.0 / 2.0 ** 0.5)
  assert projection.tangent_y == pytest.approx(1.0 / 2.0 ** 0.5)
  assert reconstructed == pytest.approx(point)


def test_projection_does_not_extend_a_reversing_terminal_path_segment() -> None:
  path = ((0.0, 0.0), (10.0, 0.0), (20.0, 0.0), (19.95, -0.02))

  projection = project_to_model_path(path, 5.0, 6.0)

  assert projection.center_x == pytest.approx(5.0)
  assert projection.center_y == pytest.approx(0.0)
  assert projection.d_path == pytest.approx(6.0)


def test_point_outside_the_measured_path_polyline_scope_is_not_predicted() -> None:
  predictor = RadarMotionPredictor()
  path = ((0.0, 0.0), (10.0, 0.0), (20.0, 0.0), (19.95, -0.02))

  assert predictor.update(
    0.0,
    (Point(10, 5.0, 6.0),),
    path,
    v_ego=10.0,
  ) == {}


def test_unmeasured_points_never_create_or_extend_history() -> None:
  predictor = RadarMotionPredictor()

  assert predictor.update(
    0.0, (Point(10, 30.0, 2.0, measured=False),), STRAIGHT_PATH,
    v_ego=10.0,
  ) == {}
  first = predictor.update(
    0.1, (Point(10, 30.0, 2.0),), STRAIGHT_PATH, v_ego=10.0,
  )[("frontRadar", 10)]
  assert predictor.update(
    0.2, (Point(10, 30.0, 1.8, measured=False),), STRAIGHT_PATH,
    v_ego=10.0,
  ) == {}
  resumed = predictor.update(
    0.3, (Point(10, 30.0, 1.6),), STRAIGHT_PATH, v_ego=10.0,
  )[("frontRadar", 10)]

  assert first.history_count == 1
  assert resumed.history_count == 2


def test_points_beyond_immediate_left_right_lanes_do_not_create_or_extend_history() -> None:
  predictor = RadarMotionPredictor()
  inside_y = IMMEDIATE_LANE_SCOPE_HALF_WIDTH_M - 0.01
  outside_y = IMMEDIATE_LANE_SCOPE_HALF_WIDTH_M + 0.01

  first = predictor.update(
    0.0, (Point(10, 30.0, inside_y),), STRAIGHT_PATH, v_ego=10.0,
  )[("frontRadar", 10)]
  assert predictor.update(
    0.1, (Point(10, 30.0, outside_y),), STRAIGHT_PATH, v_ego=10.0,
  ) == {}
  assert predictor.update(
    0.2, (Point(11, 30.0, outside_y),), STRAIGHT_PATH, v_ego=10.0,
  ) == {}
  resumed = predictor.update(
    0.2, (Point(10, 30.0, inside_y - 0.1),), STRAIGHT_PATH, v_ego=10.0,
  )[("frontRadar", 10)]

  assert first.history_count == 1
  assert resumed.history_count == 1


def test_points_outside_motion_longitudinal_range_are_not_predicted() -> None:
  predictor = RadarMotionPredictor()

  assert predictor.update(
    0.0,
    (Point(10, 130.0, -8.0, source="corner235", v_lead=20.0),),
    ((0.0, 0.0), (150.0, 10.0)),
    v_ego=20.0,
  ) == {}


def test_adjacent_vehicle_hides_farther_tracks_on_same_side_but_not_close_points() -> None:
  points = (
    Point(1, 3.0, 3.0),
    Point(2, 10.0, 3.0),
    Point(3, 25.0, 3.0),
    Point(4, 12.0, -3.0),
    Point(5, 30.0, -3.0),
    Point(6, 35.0, 0.0),
  )

  visible = visible_motion_points(points, STRAIGHT_PATH)
  protected = visible_motion_points(
    points,
    STRAIGHT_PATH,
    protected_identities=(("frontRadar", 3),),
  )

  assert {point.track_id for point in visible} == {1, 2, 4, 6}
  assert {point.track_id for point in protected} == {1, 2, 3, 4, 6}


def test_path_boundary_point_is_not_hidden_by_nearer_adjacent_return() -> None:
  points = (
    Point(1018, 9.1, -3.1),
    Point(1010, 21.1, -1.89),
  )

  visible = visible_motion_points(points, STRAIGHT_PATH)

  assert {point.track_id for point in visible} == {1010, 1018}


def test_adjacent_vehicle_ahead_of_nearest_is_visible_when_closer_than_lead_one() -> None:
  points = (
    Point(1, 10.0, 3.0, source="corner235", v_lead=10.0),
    Point(2, 25.0, 3.0, source="corner235", v_lead=10.0),
    Point(3, 35.0, 3.0, source="corner235", v_lead=10.0),
  )
  predictor = RadarMotionPredictor()

  predictions = predictor.update(
    0.0,
    points,
    STRAIGHT_PATH,
    v_ego=10.0,
    lead_one_d_rel=30.0,
  )

  assert set(predictions) == {
    ("corner235", 1),
    ("corner235", 2),
  }


def test_front_cutin_detection_starts_at_five_metres_without_dropping_occupancy() -> None:
  below = RadarMotionPredictor()
  at_limit = RadarMotionPredictor()
  below_prediction = None
  at_limit_prediction = None
  for index in range(15):
    y_rel = 3.2 - 0.1 * index
    below_prediction = below.update(
      index * 0.1,
      (
        Point(
          10,
          FRONT_CUT_IN_MIN_DREL_M - 0.1,
          y_rel,
          v_lead=10.0,
        ),
      ),
      STRAIGHT_PATH,
      v_ego=10.0,
    )[("frontRadar", 10)]
    at_limit_prediction = at_limit.update(
      index * 0.1,
      (
        Point(
          10,
          FRONT_CUT_IN_MIN_DREL_M,
          y_rel,
          v_lead=10.0,
        ),
      ),
      STRAIGHT_PATH,
      v_ego=10.0,
    )[("frontRadar", 10)]

  assert below_prediction is not None
  assert at_limit_prediction is not None
  assert not below_prediction.cut_in_detection_allowed
  assert below_prediction.cut_in_probability == 0.0
  assert below_prediction.path_entry_probability == 0.0
  assert at_limit_prediction.cut_in_detection_allowed

  occupied = below.update(
    1.5,
    (
      Point(
        10,
        FRONT_CUT_IN_MIN_DREL_M - 0.1,
        1.0,
        v_lead=10.0,
      ),
    ),
    STRAIGHT_PATH,
    v_ego=10.0,
  )[("frontRadar", 10)]
  assert occupied.current_path_occupancy


def test_front_cutin_pending_state_is_cleared_below_five_metres() -> None:
  prediction = update_series(
    RadarMotionPredictor(),
    (3.0, 2.9, 2.8, 2.7, 2.6, 2.5),
  )
  pending = replace(
    prediction,
    cut_in_detection_allowed=True,
    current_path_occupancy=False,
    cut_in_probability=1.0,
    path_entry_probability=1.0,
  )
  below_limit = replace(
    pending,
    cut_in_detection_allowed=False,
    current_path_occupancy=True,
  )
  tracker = RadarMotionDecisionTracker()

  assert not tracker.update(0.0, (pending,)).confirmed
  assert not tracker.update(0.3, (below_limit,)).confirmed


def test_front_cutin_needs_stronger_path_relative_motion_than_corner() -> None:
  assert not front_cutin_motion_supported("frontRadar", 0.50)
  assert front_cutin_motion_supported("frontRadar", 0.80)
  assert front_cutin_motion_supported(
    "frontRadar",
    -0.15,
    d_rel=12.0,
    d_path=2.5,
    d_path_rate_short=-0.25,
    predicted_path_overlap_s=1.0,
    directional_inward_displacement_m=0.25,
    directional_consistency=0.80,
    directional_inward_sample_ratio=0.70,
  )
  assert not front_cutin_motion_supported(
    "frontRadar",
    -0.15,
    d_rel=12.0,
    d_path=2.5,
    d_path_rate_short=-0.25,
    predicted_path_overlap_s=1.0,
    directional_inward_displacement_m=0.25,
    directional_consistency=0.40,
    directional_inward_sample_ratio=0.70,
  )
  assert front_cutin_motion_supported(
    "frontRadar",
    -0.30,
    d_rel=5.1,
    d_path=1.7,
    d_path_rate_short=-0.8,
    reported_normal_speed=-0.3,
    current_path_occupancy=True,
  )
  assert not front_cutin_motion_supported(
    "frontRadar",
    -0.30,
    d_rel=5.1,
    d_path=1.7,
    d_path_rate_short=-0.8,
    reported_normal_speed=0.0,
    current_path_occupancy=True,
  )
  assert front_cutin_motion_supported("corner235", 0.10)


def test_carrot_radar_cutin_sensitivity_uses_measured_motion_dwell() -> None:
  corner = radar_motion_sensitivity(3, "corner")
  front = radar_motion_sensitivity(3, "front")

  assert corner.cut_in_enabled
  assert corner.cut_in_threshold == pytest.approx(0.30)
  assert front.cut_in_threshold == pytest.approx(0.67)
  assert [
    radar_motion_sensitivity(level, "front").confirmation_s
    for level in range(6)
  ] == pytest.approx((0.0, 0.50, 0.40, 0.35, 0.25, 0.20))
  assert front.directional_min_consistency == pytest.approx(0.75)
  assert not radar_motion_sensitivity(0, "corner").cut_in_enabled
  assert radar_motion_sensitivity(-1, "corner").level == 0
  assert radar_motion_sensitivity(99, "front").level == 5


def test_higher_cutin_sensitivity_accepts_less_directional_consistency() -> None:
  common = {
    "d_rel": 52.6,
    "d_path": -2.03,
    "d_path_rate_short": 0.54,
    "predicted_path_overlap_s": 4.5,
    "directional_inward_displacement_m": 0.57,
    "directional_consistency": 0.712,
    "directional_inward_sample_ratio": 0.80,
  }

  assert not front_cutin_motion_supported(
    "frontRadar",
    0.36,
    minimum_directional_consistency=(
      radar_motion_sensitivity(3, "front").directional_min_consistency
    ),
    **common,
  )
  assert front_cutin_motion_supported(
    "frontRadar",
    0.36,
    minimum_directional_consistency=(
      radar_motion_sensitivity(4, "front").directional_min_consistency
    ),
    **common,
  )


def test_tracked_front_out_to_in_crossing_can_start_inside_five_metres() -> None:
  predictor = RadarMotionPredictor()
  prediction = None
  for index, y_rel in enumerate((
    2.8, 2.7, 2.6, 2.5, 2.4, 2.3, 2.2,
    2.1, 2.0, 1.9, 1.8, 1.7, 1.6,
  )):
    prediction = predictor.update(
      index * 0.1,
      (
        Point(
          59,
          3.2,
          y_rel,
          v_lead=10.0,
          yv_rel=-1.0,
        ),
      ),
      STRAIGHT_PATH,
      v_ego=10.0,
    )[("frontRadar", 59)]
  assert prediction is not None

  assert prediction.current_path_occupancy
  assert prediction.path_entry_age_s is not None
  assert prediction.front_tracked_close_entry
  assert prediction.cut_in_detection_allowed
  assert front_cutin_motion_supported(
    prediction.source,
    prediction.d_path_rate_long,
    tracked_close_entry=prediction.front_tracked_close_entry,
  )


def test_front_point_born_in_path_cannot_use_tracked_close_exception() -> None:
  predictor = RadarMotionPredictor()
  prediction = None
  for index, y_rel in enumerate((
    1.7, 1.6, 1.5, 1.4, 1.3, 1.2, 1.1, 1.0, 0.9,
  )):
    prediction = predictor.update(
      index * 0.1,
      (
        Point(
          59,
          3.2,
          y_rel,
          v_lead=10.0,
          yv_rel=-1.0,
        ),
      ),
      STRAIGHT_PATH,
      v_ego=10.0,
    )[("frontRadar", 59)]
  assert prediction is not None

  assert prediction.current_path_occupancy
  assert prediction.path_entry_age_s is None
  assert not prediction.front_tracked_close_entry
  assert not prediction.cut_in_detection_allowed


def test_production_controller_selects_tracked_close_front_crossing_as_lead_two() -> None:
  controller = DPathRadarController(prefer_corner_radar=False)
  output = None
  for index, y_rel in enumerate((
    2.8, 2.7, 2.6, 2.5, 2.4, 2.3, 2.2,
    2.1, 2.0, 1.9, 1.8, 1.7, 1.6,
  )):
    output = controller.update(
      time_s=index * 0.1,
      v_ego=10.0,
      radar_points=(
        Point(
          59,
          3.2,
          y_rel,
          v_lead=10.0,
          yv_rel=-1.0,
        ),
      ),
      model=model_with_lead(30.0, 0.0, 10.0, probability=0.0),
    )
  assert output is not None

  assert output.lead_one is None
  assert output.lead_two is not None
  assert output.lead_two["radarTrackId"] == 59


def test_existing_front_lead_two_can_remain_sticky_inside_five_metres() -> None:
  tracker = DPathLeadTwoTracker()
  outside_lead = {
    "status": True,
    "radar": True,
    "radarTrackId": 20,
    "dRel": FRONT_CUT_IN_MIN_DREL_M,
    "yRel": 1.0,
    "vRel": -1.0,
    "vLat": 0.0,
    "vLead": 10.0,
  }
  outside = DPathLeadCandidate(
    outside_lead,
    "frontRadar",
    20,
    7,
    True,
    True,
  )
  assert tracker.update(
    0.0, None, (outside,), 10.0,
  ).lead_two is outside_lead

  inside_lead = dict(
    outside_lead,
    dRel=FRONT_CUT_IN_MIN_DREL_M - 0.1,
    vLead=0.0,
  )
  inside = DPathLeadCandidate(
    inside_lead,
    "frontRadar",
    20,
    7,
    True,
    False,
  )
  assert tracker.update(
    0.1, None, (inside,), 10.0,
  ).lead_two is inside_lead


def test_corner_lead_uses_mutually_matched_front_longitudinal_kinematics() -> None:
  corner = RadarPointSnapshot(
    1019, "corner235", 62.057, 2.977, -1.760, 0.050, -0.2,
    16.711, 0.787, -0.186, True,
  )
  front = RadarPointSnapshot(
    35, "frontRadar", 66.166, 2.973, -1.460, 0.350, -0.1,
    17.011, 0.404, 0.016, True,
  )
  unrelated = RadarPointSnapshot(
    52, "frontRadar", 56.578, -1.305, -2.840, -0.2, 0.0,
    15.631, -0.201, -0.213, True,
  )

  fused = prefer_front_radar_kinematics(
    corner, (corner, front, unrelated),
  )

  assert (fused.track_id, fused.source) == (1019, "corner235")
  assert fused.y_rel == corner.y_rel
  assert fused.yv_rel == corner.yv_rel
  assert fused.d_rel == front.d_rel
  assert fused.v_rel == front.v_rel
  assert fused.a_rel == front.a_rel
  assert fused.v_lead == front.v_lead
  assert fused.a_lead == front.a_lead
  assert fused.j_lead == front.j_lead
  assert fused.kinematics_source == front.source
  assert fused.kinematics_track_id == front.track_id
  assert lead_duplicates_primary(
    lead_from_radar_point(fused, 0.0, 0.0, 0.0),
    lead_from_radar_point(front, 0.0, 0.0, 0.0),
  )
  assert lead_duplicates_primary(
    {
      **lead_from_radar_point(fused, 0.0, 0.0, 0.0),
      "yRel": 1.46,
    },
    {
      **lead_from_radar_point(front, 0.0, 0.0, 0.0),
      "yRel": 0.0,
    },
  )


def test_established_corner_front_match_survives_long_reflection_offset() -> None:
  associator = FrontRadarKinematicAssociator()
  corner = RadarPointSnapshot(
    1002, "corner235", 57.0, -0.10, -1.0, 0.0, 0.0,
    17.0, 0.0, 0.0, True,
  )
  front = RadarPointSnapshot(
    60, "frontRadar", 61.9, -0.20, -0.9, 0.0, 0.0,
    17.1, 0.0, 0.0, True,
  )
  initial_matches = associator.update((corner, front))
  assert initial_matches[("corner235", 1002)] is front

  moved_corner = replace(corner, d_rel=47.8, y_rel=-3.00, v_lead=18.2)
  moved_front = replace(front, d_rel=57.0, y_rel=-3.45, v_lead=18.2)
  assert (
    prefer_front_radar_kinematics(moved_corner, (moved_corner, moved_front))
    is moved_corner
  )

  held_matches = associator.update((moved_corner, moved_front))
  fused = prefer_front_radar_kinematics(
    moved_corner,
    (moved_corner, moved_front),
    held_matches,
  )
  assert fused.d_rel == moved_front.d_rel
  assert fused.kinematics_track_id == moved_front.track_id
  assert lead_duplicates_primary(
    lead_from_radar_point(fused, 0.0, 0.0, 0.0),
    lead_from_radar_point(moved_front, 0.0, 0.0, 0.0),
  )

  competing_corner = replace(
    moved_corner, track_id=1024, d_rel=56.5, y_rel=-3.40,
  )
  competing_matches = associator.update(
    (moved_corner, competing_corner, moved_front),
  )
  assert ("corner235", 1002) not in competing_matches
  assert competing_matches[("corner235", 1024)] is moved_front
  restored_matches = associator.update((moved_corner, moved_front))
  assert restored_matches[("corner235", 1002)] is moved_front

  assert associator.update(()) == {}
  assert associator.update((moved_corner, moved_front)) == {}


def test_same_primary_row_needs_an_actual_projected_path_entry() -> None:
  primary = {
    "status": True,
    "dRel": 28.5,
  }
  same_row = {
    "status": True,
    "dRel": 27.5,
  }
  well_ahead = {
    "status": True,
    "dRel": 19.0,
  }

  assert not cutin_can_compete_with_primary(
    same_row,
    primary,
    projected_path_entry=False,
  )
  assert cutin_can_compete_with_primary(
    same_row,
    primary,
    projected_path_entry=True,
  )
  assert cutin_can_compete_with_primary(
    well_ahead,
    primary,
    projected_path_entry=False,
  )


def test_cutin_must_still_be_ahead_of_primary_at_path_entry_time() -> None:
  primary = {
    "status": True,
    "dRel": 20.0,
    "vRel": 0.0,
  }
  currently_closer_but_passing_primary = {
    "status": True,
    "dRel": 15.0,
    "vRel": 3.0,
  }
  remains_ahead = {
    "status": True,
    "dRel": 15.0,
    "vRel": 0.0,
  }

  assert not cutin_can_compete_with_primary(
    currently_closer_but_passing_primary,
    primary,
    projected_path_entry=True,
    entry_horizon_s=2.0,
  )
  assert cutin_can_compete_with_primary(
    remains_ahead,
    primary,
    projected_path_entry=True,
    entry_horizon_s=2.0,
  )


def test_controller_filters_same_row_proximity_without_projected_entry() -> None:
  prediction = SimpleNamespace(
    source="corner235",
    track_id=1005,
    continuity_id=1,
    d_path=2.5,
    d_path_rate_long=-0.2,
    path_entry_probability=0.8,
    current_path_occupancy=False,
    time_to_entry_s=None,
  )

  def output_for(value: SimpleNamespace):
    controller = DPathRadarController(prefer_corner_radar=True)
    controller.motion_predictor = FixedPredictor(value)
    controller.motion_decisions = FixedDecisionTracker(value)
    return controller.update(
      time_s=1.0,
      v_ego=10.0,
      radar_points=(
        Point(10, 30.0, 0.0, source="frontRadar"),
        Point(1005, 25.0, 2.5, source="corner235"),
      ),
      model=model_with_lead(30.0, 0.0, 10.0),
    )

  filtered = output_for(prediction)
  projected = output_for(SimpleNamespace(
    **{**vars(prediction), "time_to_entry_s": 1.5},
  ))

  assert filtered.lead_one is not None
  assert filtered.lead_two is None
  assert projected.lead_two is not None
  assert projected.lead_two["radarTrackId"] == 1005


def test_occluded_point_keeps_history_but_is_not_exposed_until_visible() -> None:
  predictor = RadarMotionPredictor()
  prediction = None
  for index in range(13):
    values = predictor.update(
      index * 0.1,
      (
          Point(
            1,
            6.0 if index < 12 else 4.0,
            -3.1,
            source="corner235",
            v_lead=10.0,
          ),
        Point(
            2,
            25.0,
            -3.05 if index < 12 else -1.8,
          source="corner235",
          v_lead=10.0,
          yv_rel=1.0,
        ),
      ),
      STRAIGHT_PATH,
      v_ego=10.0,
    )
    if index < 12:
      assert ("corner235", 2) not in values
    else:
      prediction = values[("corner235", 2)]

  assert prediction is not None
  assert prediction.history_count == 13
  assert prediction.current_path_occupancy
  assert prediction.path_entry_age_s == pytest.approx(0.0)


def test_strong_cutin_motion_exposes_point_behind_adjacent_return() -> None:
  predictor = RadarMotionPredictor()
  prediction = None
  for index in range(15):
    values = predictor.update(
      index * 0.1,
      (
        Point(
          1018,
          9.0,
          -3.1,
          source="corner235",
          v_lead=10.0,
        ),
        Point(
          1010,
          24.0,
          -3.4 + index * 0.1,
          source="corner235",
          v_lead=10.0,
          yv_rel=1.0,
        ),
      ),
      STRAIGHT_PATH,
      v_ego=10.0,
    )
    prediction = values.get(("corner235", 1010), prediction)

  assert prediction is not None
  assert prediction.directional_inward_displacement_m > 0.7
  assert prediction.cut_in_probability > 0.5


def test_near_zero_vlead_is_position_only_and_clears_motion_history() -> None:
  predictor = RadarMotionPredictor()
  first = predictor.update(
    0.0,
    (Point(10, 30.0, 2.0, v_lead=10.0),),
    STRAIGHT_PATH,
    v_ego=10.0,
  )[("frontRadar", 10)]
  assert predictor.update(
    0.1,
    (Point(10, 30.0, 2.0, v_lead=2.4),),
    STRAIGHT_PATH,
    v_ego=10.0,
  ) == {}
  resumed = predictor.update(
    0.2,
    (Point(10, 30.0, 2.0, v_lead=10.0),),
    STRAIGHT_PATH,
    v_ego=10.0,
  )[("frontRadar", 10)]

  assert first.history_count == 1
  assert resumed.history_count == 1
  assert resumed.continuity_id != first.continuity_id


def test_point_above_position_only_speed_builds_motion_history() -> None:
  predictor = RadarMotionPredictor()
  prediction = predictor.update(
    0.0,
    (Point(10, 30.0, 2.0, v_lead=2.6),),
    STRAIGHT_PATH,
    v_ego=10.0,
  )[("frontRadar", 10)]

  assert prediction.history_count == 1


def test_short_gap_keeps_continuity_but_reused_track_id_resets_it() -> None:
  predictor = RadarMotionPredictor()
  first = predictor.update(
    0.0, (Point(10, 30.0, 3.0),), STRAIGHT_PATH, v_ego=10.0,
  )[("frontRadar", 10)]
  predictor.update(0.1, (), STRAIGHT_PATH, v_ego=10.0)
  resumed = predictor.update(
    0.2, (Point(10, 30.0, 2.8),), STRAIGHT_PATH, v_ego=10.0,
  )[("frontRadar", 10)]
  reused = predictor.update(
    0.3, (Point(10, 45.0, -4.0, v_rel=8.0),), STRAIGHT_PATH,
    v_ego=10.0,
  )[("frontRadar", 10)]

  assert resumed.continuity_id == first.continuity_id
  assert resumed.history_count == 2
  assert reused.continuity_id != first.continuity_id
  assert reused.history_count == 1


def test_front_and_corner_histories_are_independent_for_same_track_id() -> None:
  predictor = RadarMotionPredictor()
  for index in range(4):
    predictions = predictor.update(
      index * 0.1,
      (
        Point(10, 30.0, 3.0 - index * 0.2, source="frontRadar"),
        Point(10, 28.0, -3.0 + index * 0.1, source="corner235"),
      ),
      STRAIGHT_PATH,
      v_ego=10.0,
    )

  front = predictions[("frontRadar", 10)]
  corner = predictions[("corner235", 10)]
  assert front.sensor == "front"
  assert corner.sensor == "corner"
  assert front.continuity_id != corner.continuity_id
  assert front.d_path_rate_short < corner.d_path_rate_short


def test_vehicle_width_marks_existing_path_overlap_as_current_in() -> None:
  prediction = update_series(RadarMotionPredictor(), (1.7, 1.7, 1.7, 1.7))

  assert prediction.current_path_occupancy
  assert prediction.cut_in_probability == 0.0


def test_physical_cutin_and_cutout_probabilities_are_separate() -> None:
  cutin = update_series(RadarMotionPredictor(), (3.0, 2.8, 2.6, 2.4))
  cutout = update_series(RadarMotionPredictor(), (0.2, 0.5, 0.8, 1.1))

  assert not cutin.current_path_occupancy
  assert cutin.cut_in_probability > 0.5
  assert cutin.cut_out_probability == 0.0
  assert cutin_probability_at(cutin, 0.5) > 0.5
  assert cutout.current_path_occupancy
  assert cutout.cut_in_probability == 0.0
  assert cutout.cut_out_probability > 0.5


def test_future_drel_and_dpath_share_each_prediction_horizon() -> None:
  predictor = RadarMotionPredictor()
  prediction = None
  for index in range(4):
    prediction = predictor.update(
      index * 0.1,
      (Point(10, 30.0 - index * 0.2, 3.0 - index * 0.1, v_rel=-2.0),),
      STRAIGHT_PATH,
      v_ego=10.0,
    )[("frontRadar", 10)]
  assert prediction is not None

  sample = prediction_sample_at(prediction, 1.0)
  assert sample.horizon_s == 1.0
  assert sample.d_rel == pytest.approx(27.4)
  assert sample.d_path < prediction.d_path
  assert sample.y_rel == pytest.approx(sample.d_path)


def test_consistent_one_way_front_history_influences_future_trajectory() -> None:
  consistent = update_series(
    RadarMotionPredictor(),
    (3.00, 2.98, 2.95, 2.92, 2.88, 2.84, 2.80, 2.75, 2.69, 2.63),
  )
  noisy = update_series(
    RadarMotionPredictor(),
    (3.00, 2.94, 3.00, 2.94, 3.00, 2.94, 3.00, 2.94, 3.00, 2.94),
  )

  consistent_effective_rate = (
    consistent.d_path - prediction_sample_at(consistent, 5.0).d_path
  ) / 5.0
  noisy_effective_rate = (
    noisy.d_path - prediction_sample_at(noisy, 5.0).d_path
  ) / 5.0

  assert consistent.directional_inward_displacement_m >= 0.20
  assert consistent.directional_consistency >= 0.75
  assert consistent.directional_inward_sample_ratio >= 0.65
  assert consistent_effective_rate > -consistent.d_path_rate_long
  assert noisy.directional_inward_displacement_m < 0.20
  assert noisy.directional_consistency < 0.75
  assert noisy_effective_rate == pytest.approx(-noisy.d_path_rate_long)


def test_close_directional_corner_entry_survives_long_vehicle_point_passing_ego() -> None:
  predictor = RadarMotionPredictor()
  prediction = None
  for index in range(9):
    prediction = predictor.update(
      index * 0.1,
      (
        Point(
          1003,
          6.5 - index * 0.3,
          2.83 - index * 0.03,
          source="corner235",
          v_rel=-3.0,
          v_lead=7.0,
          yv_rel=-0.3,
        ),
      ),
      STRAIGHT_PATH,
      v_ego=10.0,
    )[("corner235", 1003)]
  assert prediction is not None

  assert prediction.predicted_path_overlap_s == 0.0
  assert prediction_sample_at(prediction, 1.5).d_rel < 0.0
  assert prediction.directional_inward_displacement_m >= 0.20
  assert prediction.directional_consistency >= 0.90
  assert prediction.near_side_directional_entry
  assert prediction.cut_in_probability >= 0.30
  assert prediction.reason == "near-side directional entry"
  decision = RadarMotionDecisionTracker(threshold=0.30).update(
    0.8, (prediction,),
  )
  assert decision.confirmed
  assert decision.confirmed[0].prediction.track_id == 1003


@pytest.mark.parametrize(
  ("d_rel", "lateral_values"),
  (
    (4.1, (2.59,) * 9),
    (4.1, (2.59, 2.65, 2.59, 2.65, 2.59, 2.65, 2.59, 2.65, 2.59)),
    (5.5, (2.83, 2.80, 2.77, 2.74, 2.71, 2.68, 2.65, 2.62, 2.59)),
  ),
)
def test_near_side_entry_rejects_parallel_noise_and_points_beyond_close_range(
  d_rel: float,
  lateral_values: tuple[float, ...],
) -> None:
  predictor = RadarMotionPredictor()
  prediction = None
  for index, y_rel in enumerate(lateral_values):
    prediction = predictor.update(
      index * 0.1,
      (
        Point(
          1003,
          d_rel,
          y_rel,
          source="corner235",
          v_lead=10.0,
          yv_rel=-0.3,
        ),
      ),
      STRAIGHT_PATH,
      v_ego=10.0,
    )[("corner235", 1003)]
  assert prediction is not None

  assert not prediction.near_side_directional_entry


def test_dpath_change_without_target_path_progress_is_not_extrapolated_as_cutin() -> None:
  predictor = RadarMotionPredictor()
  prediction = None
  for index in range(6):
    prediction = predictor.update(
      index * 0.1,
      (
        Point(
          10,
          30.0 - index,
          3.0 - index * 0.1,
          v_rel=-10.0,
          v_lead=5.0,
        ),
      ),
      STRAIGHT_PATH,
      v_ego=10.0,
    )[("frontRadar", 10)]
  assert prediction is not None

  assert prediction.cut_in_probability == 0.0
  assert prediction.d_path_rate_long == 0.0
  assert prediction_sample_at(prediction, 2.0).d_path == pytest.approx(
    prediction.d_path,
  )


def test_path_vector_history_stabilizes_ego_motion_for_straight_vehicle() -> None:
  predictor = RadarMotionPredictor()
  prediction = None
  for index in range(5):
    prediction = predictor.update(
      index * 0.1,
      (Point(10, 30.0, 3.0, v_rel=0.0, v_lead=10.0),),
      STRAIGHT_PATH,
      v_ego=10.0,
    )[("frontRadar", 10)]
  assert prediction is not None

  assert prediction.history[0].path_x == pytest.approx(26.0)
  assert prediction.history[-1].path_x == pytest.approx(30.0)
  assert prediction.history[0].actual_x == pytest.approx(26.0)
  assert prediction.history[-1].actual_x == pytest.approx(30.0)
  assert prediction.history[0].actual_y == pytest.approx(3.0)
  assert prediction.history[-1].actual_y == pytest.approx(3.0)
  sample = prediction_sample_at(prediction, 1.0)
  assert sample.path_x == pytest.approx(40.0)
  assert sample.d_rel == pytest.approx(30.0)
  assert prediction.path_speed == pytest.approx(10.0)
  assert prediction.vector_heading_deg == pytest.approx(0.0)


def test_future_path_y_uses_target_progress_not_future_relative_distance() -> None:
  path = ((0.0, 0.0), (100.0, 10.0))
  predictor = RadarMotionPredictor()
  prediction = None
  for index in range(5):
    prediction = predictor.update(
      index * 0.1,
      (Point(10, 30.0, 0.0, v_rel=0.0, v_lead=10.0),),
      path,
      v_ego=10.0,
    )[("frontRadar", 10)]
  assert prediction is not None

  sample = prediction_sample_at(prediction, 1.0)
  current_projection = project_to_model_path(path, 30.0, 0.0)
  expected_path_s = current_projection.path_s + prediction.path_speed
  _, expected_y = model_path_point_at_s(
    path,
    expected_path_s,
    current_projection.d_path,
  )
  assert sample.path_x == pytest.approx(expected_path_s)
  assert sample.d_rel == pytest.approx(expected_path_s - 10.0)
  assert sample.d_path == pytest.approx(current_projection.d_path)
  assert sample.y_rel == pytest.approx(expected_y)


def test_corner_position_drift_requires_reported_lateral_motion_consistency() -> None:
  inconsistent = RadarMotionPredictor()
  consistent = RadarMotionPredictor()
  inconsistent_prediction = None
  consistent_prediction = None
  for index in range(15):
    y_rel = 4.0 - index * 0.1
    inconsistent_prediction = inconsistent.update(
      index * 0.1,
      (
        Point(
          1010,
          25.0,
          y_rel,
          source="corner235",
          v_lead=10.0,
          yv_rel=0.0,
        ),
      ),
      STRAIGHT_PATH,
      v_ego=10.0,
    )[("corner235", 1010)]
    consistent_prediction = consistent.update(
      index * 0.1,
      (
        Point(
          1010,
          25.0,
          y_rel,
          source="corner235",
          v_lead=10.0,
          yv_rel=-1.0,
        ),
      ),
      STRAIGHT_PATH,
      v_ego=10.0,
    )[("corner235", 1010)]

  assert inconsistent_prediction is not None
  assert consistent_prediction is not None
  assert inconsistent_prediction.motion_consistency < 0.5
  assert inconsistent_prediction.cut_in_probability < 0.5
  assert consistent_prediction.motion_consistency > 0.9
  assert consistent_prediction.cut_in_probability > 0.5


def test_strong_dpath_history_survives_curved_path_normal_velocity_mismatch() -> None:
  path = ((0.0, 0.0), (100.0, -10.0))
  predictor = RadarMotionPredictor()
  prediction = None
  for index in range(15):
    prediction = predictor.update(
      index * 0.1,
      (
        Point(
          1010,
          25.0,
          -1.4 + index * 0.1,
          source="corner235",
          v_lead=10.0,
          yv_rel=0.4,
        ),
      ),
      path,
      v_ego=10.0,
    )[("corner235", 1010)]

  assert prediction is not None
  assert prediction.d_path_rate_long > 0.9
  assert prediction.reported_normal_speed < -0.5
  assert prediction.directional_inward_displacement_m > 0.7
  assert prediction.motion_consistency > 0.9
  assert prediction.predicted_path_overlap_start_s is not None
  assert prediction.cut_in_probability > 0.5


def test_stronger_same_direction_corner_velocity_supports_position_trend() -> None:
  predictor = RadarMotionPredictor()
  prediction = None
  for index in range(15):
    prediction = predictor.update(
      index * 0.1,
      (
        Point(
          1010,
          25.0,
          4.0 - index * 0.1,
          source="corner235",
          v_lead=10.0,
          yv_rel=-2.0,
        ),
      ),
      STRAIGHT_PATH,
      v_ego=10.0,
    )[("corner235", 1010)]

  assert prediction is not None
  assert prediction.reported_normal_speed < prediction.d_path_rate_long
  assert prediction.normal_speed_disagreement == pytest.approx(0.0)
  assert prediction.motion_consistency > 0.9
  assert prediction.cut_in_probability > 0.5


def test_long_horizon_corner_entry_requires_measured_inward_displacement() -> None:
  predictor = RadarMotionPredictor()
  prediction = None
  for index in range(15):
    prediction = predictor.update(
      index * 0.1,
      (
        Point(
          1010,
          20.0,
          3.0 - index * 0.02,
          source="corner235",
          v_lead=10.0,
          yv_rel=-0.2,
        ),
      ),
      STRAIGHT_PATH,
      v_ego=10.0,
    )[("corner235", 1010)]

  assert prediction is not None
  assert prediction.predicted_path_overlap_start_s == pytest.approx(4.0)
  assert prediction.predicted_path_overlap_s >= 0.5
  assert prediction.directional_inward_displacement_m < 0.30
  assert prediction.cut_in_probability == pytest.approx(0.0)
  assert prediction.reason == "long-horizon corner direction unconfirmed"


def test_nearer_horizon_corner_entry_keeps_existing_physical_gate() -> None:
  predictor = RadarMotionPredictor()
  prediction = None
  for index in range(15):
    prediction = predictor.update(
      index * 0.1,
      (
        Point(
          1010,
          20.0,
          2.8 - index * 0.03,
          source="corner235",
          v_lead=10.0,
          yv_rel=-0.3,
        ),
      ),
      STRAIGHT_PATH,
      v_ego=10.0,
    )[("corner235", 1010)]

  assert prediction is not None
  assert prediction.predicted_path_overlap_start_s < 3.5
  assert prediction.cut_in_probability > 0.5


def test_corner_cutin_confidence_drops_when_recent_lateral_motion_stops() -> None:
  predictor = RadarMotionPredictor()
  prediction = None
  for index in range(15):
    prediction = predictor.update(
      index * 0.1,
      (
        Point(
          1010,
          25.0,
          4.0 - index * 0.1,
          source="corner235",
          v_lead=10.0,
          yv_rel=-1.0,
        ),
      ),
      STRAIGHT_PATH,
      v_ego=10.0,
    )[("corner235", 1010)]

  for index in range(15, 17):
    prediction = predictor.update(
      index * 0.1,
      (
        Point(
          1010,
          25.0,
          2.6,
          source="corner235",
          v_lead=10.0,
          yv_rel=0.0,
        ),
      ),
      STRAIGHT_PATH,
      v_ego=10.0,
    )[("corner235", 1010)]

  assert prediction.d_path_rate_long < -0.8
  assert prediction.reported_normal_speed == pytest.approx(0.0)
  assert prediction.motion_consistency < 0.1
  assert prediction.cut_in_probability < 0.1


def test_recent_position_stall_does_not_extend_long_cutin_trend() -> None:
  predictor = RadarMotionPredictor()
  prediction = None
  lateral_values = (
    3.2, 3.16, 3.12, 3.08, 3.04, 3.0, 2.96, 2.92,
    2.88, 2.84, 2.80, 2.80, 2.80, 2.80, 2.80,
  )
  for index, y_rel in enumerate(lateral_values):
    prediction = predictor.update(
      index * 0.1,
      (
        Point(
          1010,
          25.0,
          y_rel,
          source="corner235",
          v_lead=10.0,
          yv_rel=-0.4,
        ),
      ),
      STRAIGHT_PATH,
      v_ego=10.0,
    )[("corner235", 1010)]

  assert prediction is not None
  assert prediction.d_path_rate_long < -0.2
  assert abs(prediction.d_path_rate_short) < 0.1
  assert prediction.recent_motion_support < 0.5
  assert prediction.cut_in_probability < 0.26


def test_short_long_rate_disagreement_increases_future_uncertainty() -> None:
  steady = update_series(RadarMotionPredictor(), (3.0, 2.9, 2.8, 2.7, 2.6, 2.5))
  turning = update_series(RadarMotionPredictor(), (3.0, 2.95, 2.9, 2.7, 2.35, 1.9))

  assert abs(turning.d_path_rate_short - turning.d_path_rate_long) > abs(
    steady.d_path_rate_short - steady.d_path_rate_long
  )
  assert prediction_sample_at(turning, 2.0).lateral_sigma > prediction_sample_at(
    steady, 2.0,
  ).lateral_sigma


def test_shared_decision_tracker_confirms_sustained_physical_cutin() -> None:
  predictor = RadarMotionPredictor()
  decision_tracker = RadarMotionDecisionTracker()
  confirmed_at = None

  for index in range(12):
    time_s = index * 0.1
    prediction = predictor.update(
      time_s,
      (
        Point(
          10,
          30.0,
          3.0 - 0.12 * index,
          v_lead=10.0,
        ),
      ),
      STRAIGHT_PATH,
      v_ego=10.0,
    )[("frontRadar", 10)]
    decision = decision_tracker.update(time_s, (prediction,))
    if decision.confirmed and confirmed_at is None:
      confirmed_at = time_s

  assert confirmed_at is not None
  # This front track already has the strict 0.8-second one-way direction
  # evidence, so it receives at most one radar-frame credit. With 10 Hz test
  # samples, the first observed confirmation moves from 0.7 s to 0.6 s.
  assert confirmed_at == pytest.approx(0.6)
  assert decision.confirmed[0].prediction.track_id == 10


def test_path_proximity_without_continuous_overlap_is_not_a_cutin() -> None:
  predictor = RadarMotionPredictor()
  prediction = None
  for index in range(15):
    prediction = predictor.update(
      index * 0.1,
      (
        Point(
          10,
          30.0,
          3.2 - 0.2 * index * 0.1,
          v_lead=10.0,
        ),
      ),
      STRAIGHT_PATH,
      v_ego=10.0,
    )[("frontRadar", 10)]
  assert prediction is not None

  # Uncertainty can approach the corridor, but no two consecutive future
  # samples physically overlap it.
  assert prediction.time_to_entry_s is None
  assert min(abs(sample.d_path) for sample in prediction.samples) > 1.68
  assert prediction.predicted_path_overlap_s == 0.0
  assert prediction.cut_in_probability == 0.0
  assert cutin_probability_at(prediction, 5.0) == 0.0

  sensitive = RadarMotionDecisionTracker(threshold=0.20)
  conservative = RadarMotionDecisionTracker(threshold=0.50)
  for index in range(4):
    sensitive_result = sensitive.update(index * 0.1, (prediction,))
    conservative_result = conservative.update(index * 0.1, (prediction,))

  assert not sensitive_result.confirmed
  assert not conservative_result.confirmed


def test_future_overlap_does_not_replace_measured_motion_dwell() -> None:
  prediction = update_series(
    RadarMotionPredictor(),
    (3.0, 2.8, 2.6, 2.4),
  )
  assert prediction.samples[-1].horizon_s == pytest.approx(5.0)
  assert prediction.predicted_path_overlap_s >= 0.5

  long_overlap = replace(
    prediction,
    cut_in_probability=0.8,
    path_entry_probability=0.8,
    predicted_path_overlap_s=1.5,
  )
  short_overlap = replace(
    long_overlap,
    predicted_path_overlap_s=0.5,
  )
  long_tracker = RadarMotionDecisionTracker(
    threshold=0.5,
    confirmation_s=0.35,
  )
  short_tracker = RadarMotionDecisionTracker(
    threshold=0.5,
    confirmation_s=0.35,
  )

  assert not long_tracker.update(0.0, (long_overlap,)).confirmed
  assert not short_tracker.update(0.0, (short_overlap,)).confirmed
  assert not long_tracker.update(0.34, (long_overlap,)).confirmed
  assert not short_tracker.update(0.34, (short_overlap,)).confirmed
  assert long_tracker.update(0.35, (long_overlap,)).confirmed
  assert short_tracker.update(0.35, (short_overlap,)).confirmed


def test_normal_confirmation_does_not_change_at_25_metres() -> None:
  prediction = update_series(
    RadarMotionPredictor(),
    (3.0, 2.8, 2.6, 2.4),
  )
  common = {
    "cut_in_probability": 0.8,
    "path_entry_probability": 0.8,
    "predicted_path_overlap_s": 1.5,
    "current_path_occupancy": False,
  }
  near = replace(
    prediction,
    history=prediction.history[:-1] + (
      replace(prediction.history[-1], d_rel=10.0),
    ),
    **common,
  )
  far = replace(
    prediction,
    history=prediction.history[:-1] + (
      replace(prediction.history[-1], d_rel=40.0),
    ),
    **common,
  )
  near_tracker = RadarMotionDecisionTracker(
    threshold=0.5,
    confirmation_s=0.35,
  )
  far_tracker = RadarMotionDecisionTracker(
    threshold=0.5,
    confirmation_s=0.35,
  )

  assert not near_tracker.update(0.0, (near,)).confirmed
  assert not far_tracker.update(0.0, (far,)).confirmed
  assert near_tracker.update(0.35, (near,)).confirmed
  assert far_tracker.update(0.35, (far,)).confirmed


@pytest.mark.parametrize(
  ("sensitivity", "confirmation_s"),
  ((1, 0.50), (2, 0.40), (3, 0.35), (4, 0.25), (5, 0.20)),
)
def test_sensitivity_controls_measured_confirmation_time(
  sensitivity: int,
  confirmation_s: float,
) -> None:
  prediction = replace(
    update_series(
      RadarMotionPredictor(),
      (3.0, 2.8, 2.6, 2.4),
    ),
    cut_in_probability=0.8,
    path_entry_probability=0.8,
    predicted_path_overlap_s=1.5,
    current_path_occupancy=False,
  )
  policy = radar_motion_sensitivity(sensitivity, "front")
  tracker = RadarMotionDecisionTracker(
    threshold=0.5,
    confirmation_s=policy.confirmation_s,
  )

  assert policy.confirmation_s == pytest.approx(confirmation_s)
  assert not tracker.update(0.0, (prediction,)).confirmed
  assert not tracker.update(
    confirmation_s - 0.01,
    (prediction,),
  ).confirmed
  assert tracker.update(confirmation_s, (prediction,)).confirmed


def test_strong_front_direction_history_gets_one_frame_confirmation_credit() -> None:
  prediction = replace(
    update_series(
      RadarMotionPredictor(),
      (3.0, 2.8, 2.6, 2.4),
    ),
    cut_in_probability=0.8,
    path_entry_probability=0.8,
    predicted_path_overlap_s=1.5,
    current_path_occupancy=False,
    directional_inward_displacement_m=0.55,
    directional_consistency=0.85,
    directional_inward_sample_ratio=0.85,
    d_path_rate_short=-0.50,
    d_path_rate_long=-0.30,
    motion_consistency=1.0,
    recent_motion_support=1.0,
  )
  tracker = RadarMotionDecisionTracker(
    threshold=0.5,
    confirmation_s=0.35,
  )

  assert not tracker.update(0.0, (prediction,)).confirmed
  assert not tracker.update(0.28, (prediction,)).confirmed
  assert tracker.update(0.29, (prediction,)).confirmed


def test_small_front_drift_does_not_get_confirmation_credit() -> None:
  prediction = replace(
    update_series(
      RadarMotionPredictor(),
      (3.0, 2.8, 2.6, 2.4),
    ),
    cut_in_probability=0.8,
    path_entry_probability=0.8,
    predicted_path_overlap_s=1.5,
    current_path_occupancy=False,
    directional_inward_displacement_m=0.20,
    directional_consistency=0.95,
    directional_inward_sample_ratio=0.85,
    d_path_rate_short=-0.50,
    d_path_rate_long=-0.30,
    motion_consistency=1.0,
    recent_motion_support=1.0,
  )
  tracker = RadarMotionDecisionTracker(
    threshold=0.5,
    confirmation_s=0.35,
  )

  assert not tracker.update(0.0, (prediction,)).confirmed
  assert not tracker.update(0.34, (prediction,)).confirmed
  assert tracker.update(0.35, (prediction,)).confirmed


def test_close_consistent_corner_entry_uses_short_confirmation() -> None:
  prediction = update_series(
    RadarMotionPredictor(),
    (2.7, 2.65, 2.6, 2.55, 2.5, 2.45),
    source="corner235",
    track_id=1071,
  )
  history = prediction.history[:-1] + (
    replace(prediction.history[-1], d_rel=3.0),
  )
  urgent = replace(
    prediction,
    d_path=2.15,
    d_path_rate_short=-0.48,
    d_path_rate_long=-0.15,
    motion_consistency=1.0,
    recent_motion_support=1.0,
    current_path_occupancy=False,
    cut_in_probability=0.6,
    path_entry_probability=0.6,
    history=history,
  )
  not_urgent = replace(
    urgent,
    d_path_rate_long=-0.05,
  )
  urgent_tracker = RadarMotionDecisionTracker(threshold=0.3)
  normal_tracker = RadarMotionDecisionTracker(threshold=0.3)

  assert not urgent_tracker.update(0.0, (urgent,)).confirmed
  assert not normal_tracker.update(0.0, (not_urgent,)).confirmed
  assert urgent_tracker.update(0.11, (urgent,)).confirmed
  assert not normal_tracker.update(0.11, (not_urgent,)).confirmed
  assert normal_tracker.update(0.35, (not_urgent,)).confirmed


def test_slow_adjacent_drift_below_path_uncertainty_is_not_a_cutin() -> None:
  predictor = RadarMotionPredictor()
  prediction = None
  for index in range(16):
    prediction = predictor.update(
      index * 0.1,
      (
        Point(
          1010,
          25.0,
          2.4 - 0.007 * index,
          source="corner235",
          v_lead=10.0,
          yv_rel=-0.07,
        ),
      ),
      STRAIGHT_PATH,
      v_ego=10.0,
    )[("corner235", 1010)]

  assert prediction is not None
  assert prediction.d_path_rate_long == pytest.approx(-0.07)
  assert prediction.predicted_path_overlap_s == 0.0
  assert prediction.cut_in_probability < 0.5


def test_dpath_lead_two_is_selected_after_and_ahead_of_primary() -> None:
  primary = {
    "status": True,
    "radar": True,
    "radarTrackId": 10,
    "dRel": 35.0,
    "yRel": 0.1,
  }
  candidates = (
    {
      "status": True,
      "radar": True,
      "radarTrackId": 10,
      "dRel": 34.0,
      "yRel": 0.2,
      "vLead": 10.0,
    },
    {
      "status": True,
      "radar": True,
      "radarTrackId": 20,
      "dRel": 22.0,
      "yRel": 2.0,
      "vLead": 10.0,
    },
    {
      "status": True,
      "radar": True,
      "radarTrackId": 30,
      "dRel": 45.0,
      "yRel": -2.0,
      "vLead": 10.0,
    },
  )

  selection = select_dpath_lead_two(primary, candidates, v_ego=20.0)

  assert dpath_control_max_d_rel(20.0) == pytest.approx(80.0)
  assert [lead["radarTrackId"] for lead in selection.cutins] == [20]
  assert selection.lead_two["radarTrackId"] == 20


def test_dpath_lead_two_is_limited_to_80m_and_strictly_closer_than_primary() -> None:
  primary = {
    "status": True,
    "radar": True,
    "radarTrackId": 10,
    "dRel": 80.0,
    "yRel": 0.0,
  }
  candidates = tuple(
    {
      "status": True,
      "radar": True,
      "radarTrackId": track_id,
      "dRel": d_rel,
      "yRel": 2.0,
      "vLead": 20.0,
    }
    for track_id, d_rel in ((20, 79.9), (30, 80.0), (40, 80.1))
  )

  selection = select_dpath_lead_two(primary, candidates, v_ego=40.0)
  without_primary = select_dpath_lead_two(None, candidates, v_ego=40.0)

  assert dpath_control_max_d_rel(40.0) == pytest.approx(80.0)
  assert [lead["radarTrackId"] for lead in selection.cutins] == [20]
  assert [
    lead["radarTrackId"] for lead in without_primary.cutins
  ] == [20, 30]


def test_confirmed_lead_two_is_retained_while_same_continuity_is_in_path() -> None:
  tracker = DPathLeadTwoTracker()
  detected_lead = {
    "status": True,
    "radar": True,
    "radarTrackId": 20,
    "dRel": 22.0,
    "yRel": 2.0,
    "vLead": 10.0,
  }
  detected = DPathLeadCandidate(
    detected_lead, "corner235", 20, 7, False, True,
  )

  assert tracker.update(0.0, None, (detected,), 20.0).lead_two is detected_lead

  assert tracker.update(0.1, None, (), 20.0).lead_two is None
  assert tracker.active_identity == detected.identity

  held_lead = dict(detected_lead, dRel=21.0, yRel=1.0, vLead=0.0)
  held = DPathLeadCandidate(
    held_lead, "corner235", 20, 7, True, False,
  )
  assert tracker.update(0.2, None, (held,), 20.0).lead_two is held_lead

  reused = DPathLeadCandidate(
    dict(held_lead), "corner235", 20, 8, True, False,
  )
  assert tracker.update(0.3, None, (reused,), 20.0).lead_two is None
  assert tracker.active_identity is None


def test_unconfirmed_current_path_motion_does_not_become_lead_two() -> None:
  tracker = DPathLeadTwoTracker()
  path_lead = {
    "status": True,
    "radar": True,
    "radarTrackId": 20,
    "dRel": 22.0,
    "yRel": 1.2,
    "vLead": 10.0,
  }
  path_candidate = DPathLeadCandidate(
    path_lead, "corner235", 20, 7, True, False,
  )
  selection = tracker.update(0.0, None, (path_candidate,), 20.0)

  assert selection.lead_two is None
  assert selection.cutins == ()


def test_unconfirmed_current_path_motion_stays_out_behind_primary() -> None:
  tracker = DPathLeadTwoTracker()
  primary = {
    "status": True,
    "radar": True,
    "radarTrackId": 10,
    "dRel": 25.0,
    "yRel": 0.0,
  }
  path_lead = {
    "status": True,
    "radar": True,
    "radarTrackId": 20,
    "dRel": 40.0,
    "yRel": 0.2,
    "vLead": 10.0,
  }
  path_candidate = DPathLeadCandidate(
    path_lead, "frontRadar", 20, 1, True, False,
  )

  selection = tracker.update(0.0, primary, (path_candidate,), 20.0)

  assert selection.lead_two is None
  assert selection.cutins == ()


def test_confirmed_cutin_behind_primary_is_not_lead_two() -> None:
  tracker = DPathLeadTwoTracker()
  primary = {
    "status": True,
    "radar": True,
    "radarTrackId": 10,
    "dRel": 25.0,
    "yRel": 0.0,
  }
  cutin_lead = {
    "status": True,
    "radar": True,
    "radarTrackId": 20,
    "dRel": 40.0,
    "yRel": 2.0,
    "vLead": 10.0,
  }
  cutin = DPathLeadCandidate(
    cutin_lead, "frontRadar", 20, 1, True, True,
  )

  selection = tracker.update(0.0, primary, (cutin,), 20.0)

  assert selection.lead_two is None
  assert selection.cutins == ()


def test_primary_matcher_uses_model_lead_zero_and_front_scc_only() -> None:
  matcher = VisionRadarMatcher()
  points = snapshot_radar_points(
    (
      Point(10, 30.0, 0.2, v_rel=2.0, source="frontRadar"),
      Point(20, 30.0, 0.2, v_rel=2.0, source="corner235"),
      Point(30, 22.0, 0.2, v_rel=2.0, source="frontRadar"),
    ),
    v_ego=10.0,
  )

  match = matcher.match(
    model_with_lead(30.0, 0.2, 12.0),
    points,
    STRAIGHT_PATH,
  )

  assert match is not None
  assert match.point.track_id == 10
  assert match.point.source == "frontRadar"
  assert match.d_path == pytest.approx(0.2)


def test_primary_matcher_uses_vision_longitudinal_uncertainty() -> None:
  matcher = VisionRadarMatcher()
  points = snapshot_radar_points(
    (
      Point(
        32,
        26.78,
        0.0,
        source="frontRadar",
        v_lead=14.60,
      ),
      Point(
        56,
        5.84,
        -2.59,
        source="frontRadar",
        v_lead=11.02,
      ),
    ),
    v_ego=10.0,
  )
  model = model_with_lead(18.54, 0.0, 10.54, probability=0.993)
  model.leadsV3[0].xStd = (3.60,)

  match = matcher.match(model, points, STRAIGHT_PATH)

  assert match is not None
  assert match.point.track_id == 32


def test_vision_bracket_supports_only_persistent_mutually_matched_cutin() -> None:
  corner, front = snapshot_radar_points(
    (
      Point(
        1003,
        3.95,
        -2.59,
        source="corner235",
        v_lead=11.41,
      ),
      Point(
        56,
        5.84,
        -2.59,
        source="frontRadar",
        v_lead=11.02,
      ),
    ),
    v_ego=10.0,
  )
  prediction = RadarMotionPrediction(
    track_id=1003,
    source="corner235",
    sensor="corner",
    continuity_id=1,
    d_path=-2.59,
    d_path_rate_short=0.246,
    d_path_rate_long=0.193,
    d_path_curvature=0.0,
    path_speed_short=11.0,
    path_speed_long=11.0,
    path_speed=11.0,
    vector_heading_deg=0.8,
    reported_heading_deg=0.8,
    reported_normal_speed=0.2,
    normal_speed_disagreement=0.0,
    motion_consistency=0.99,
    recent_motion_support=0.99,
    uncertainty=0.4,
    lane_half_width=1.8,
    current_path_occupancy=False,
    cut_in_detection_allowed=True,
    cut_in_probability=0.21,
    cut_out_probability=0.0,
    path_entry_probability=0.21,
    path_entry_age_s=None,
    samples=(),
    history=(
      RadarMotionHistorySample(
        0.0, 1.10, 4.2, 4.2, 4.2, -2.91, -2.91,
      ),
      RadarMotionHistorySample(
        1.1, 0.0, 3.95, 3.95, 3.95, -2.59, -2.59,
      ),
    ),
    history_count=12,
    time_to_entry_s=None,
    reason="outside path corridor",
    predicted_path_overlap_s=1.0,
    predicted_path_overlap_start_s=2.0,
  )
  vision = VisionLead(
    probability=0.993,
    d_rel=18.54,
    y_rel=0.0,
    velocity=10.54,
    x_std=3.60,
    y_std=0.60,
    v_std=1.50,
  )
  lead_one = {"status": True, "dRel": 26.78}

  supported = apply_vision_bracket_cutin_support(
    prediction,
    corner,
    (corner, front),
    vision,
    lead_one,
  )
  unbracketed = apply_vision_bracket_cutin_support(
    prediction,
    corner,
    (corner, front),
    replace(vision, d_rel=4.0),
    lead_one,
  )
  no_continuous_overlap = apply_vision_bracket_cutin_support(
    replace(
      prediction,
      predicted_path_overlap_s=0.0,
      predicted_path_overlap_start_s=None,
    ),
    corner,
    (corner, front),
    vision,
    lead_one,
  )

  assert supported.cut_in_probability == pytest.approx(0.9732393)
  assert supported.path_entry_probability == pytest.approx(0.9732393)
  assert supported.reason == "vision-bracketed physical CUT-IN"
  assert unbracketed is prediction
  assert no_continuous_overlap.cut_in_probability == pytest.approx(0.21)


def test_primary_matcher_rejects_low_score_fresh_distant_side_match() -> None:
  matcher = VisionRadarMatcher()
  point = snapshot_radar_points(
    (
      Point(
        34,
        75.63,
        -2.55,
        v_rel=2.51,
        source="frontRadar",
      ),
    ),
    v_ego=26.49,
  )[0]
  model = SimpleNamespace(
    position=SimpleNamespace(x=(0.0, 120.0), y=(0.0, 0.0)),
    leadsV3=(SimpleNamespace(
      prob=0.78,
      x=(96.46,),
      y=(1.15,),
      v=(25.73,),
      xStd=(12.64,),
      yStd=(0.59,),
      vStd=(2.16,),
    ),),
  )

  match = matcher.match(model, (point,), STRAIGHT_PATH)

  assert match is None


def test_stationary_radar_is_confirmed_once_then_retained_without_vision() -> None:
  matcher = VisionRadarMatcher()
  match = None
  for index in range(7):
    time_s = index * 0.05
    point = snapshot_radar_points(
      (
        Point(
          1009,
          100.0 - 10.0 * time_s,
          0.1,
          v_rel=-10.0,
          source="corner235",
        ),
      ),
      v_ego=10.0,
    )[0]
    match = matcher.match(
      model_with_lead(
        point.d_rel,
        point.y_rel,
        7.0,
        probability=0.45 if index == 0 else 0.0,
      ),
      (),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(point,),
      prefer_corner_stationary=True,
    )

  assert match is not None
  assert match.point.track_id == 1009

  retained_point = snapshot_radar_points(
    (
      Point(
        1009,
        96.5,
        0.1,
        v_rel=-10.0,
        source="corner235",
      ),
    ),
    v_ego=10.0,
  )[0]
  retained = matcher.match(
    model_with_lead(40.0, 0.0, 10.0, probability=0.0),
    (),
    STRAIGHT_PATH,
    time_s=0.35,
    stationary_points=(retained_point,),
    prefer_corner_stationary=True,
  )

  assert retained is not None
  assert retained.point.track_id == 1009


def test_stationary_front_radar_rejects_low_confidence_vision_seed() -> None:
  matcher = VisionRadarMatcher()
  match = None
  for index in range(12):
    time_s = index * 0.05
    point = snapshot_radar_points(
      (
        Point(
          55,
          40.0 - 10.0 * time_s,
          0.1,
          v_rel=-10.0,
          source="frontRadar",
        ),
      ),
      v_ego=10.0,
    )[0]
    match = matcher.match(
      model_with_lead(
        point.d_rel,
        point.y_rel,
        0.0,
        probability=0.15,
      ),
      (),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(point,),
    )

  assert match is None
  assert matcher.stationary_identity is None


def test_stationary_front_radar_rejects_two_frame_vision_spike() -> None:
  matcher = VisionRadarMatcher()
  match = None
  for index in range(12):
    time_s = index * 0.05
    point = snapshot_radar_points(
      (
        Point(
          50,
          70.0 - 10.0 * time_s,
          0.1,
          v_rel=-10.0,
          source="frontRadar",
        ),
      ),
      v_ego=10.0,
    )[0]
    match = matcher.match(
      model_with_lead(
        point.d_rel,
        point.y_rel,
        0.0,
        probability=0.45 if index in (1, 4) else 0.15,
      ),
      (),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(point,),
    )

  assert match is None
  assert matcher.stationary_identity is None


def test_stationary_radar_rejects_fast_vision_speed_mismatch() -> None:
  matcher = VisionRadarMatcher()
  match = None
  for index in range(7):
    time_s = index * 0.05
    point = snapshot_radar_points(
      (
        Point(
          37,
          95.0 - 26.4 * time_s,
          1.7,
          v_rel=-26.8,
          source="frontRadar",
        ),
      ),
      v_ego=26.4,
    )[0]
    match = matcher.match(
      model_with_lead(
        point.d_rel + 15.0,
        0.5,
        26.4,
        probability=0.60,
      ),
      (),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(point,),
    )

  assert match is None
  assert matcher.stationary_identity is None


def test_central_corner_stationary_becomes_lead_without_vision() -> None:
  matcher = VisionRadarMatcher()
  match = None
  for index in range(20):
    time_s = index * 0.05
    point = snapshot_radar_points(
      (
        Point(
          1009,
          100.0 - 10.0 * time_s,
          0.1,
          v_rel=-10.0,
          source="corner235",
        ),
      ),
      v_ego=10.0,
    )[0]
    match = matcher.match(
      model_with_lead(
        point.d_rel,
        point.y_rel,
        0.0,
        probability=0.0,
      ),
      (),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(point,),
      prefer_corner_stationary=True,
    )

  assert match is not None
  assert match.point.source == "corner235"
  assert match.point.track_id == 1009
  assert matcher.stationary_identity == ("corner235", 1009)


def test_radar_only_stationary_corner_requires_half_second_confirmation() -> None:
  matcher = VisionRadarMatcher()
  for index in range(10):
    time_s = index * 0.05
    point = snapshot_radar_points(
      (
        Point(
          1009,
          60.0 - 10.0 * time_s,
          0.1,
          v_rel=-10.0,
          source="corner235",
        ),
      ),
      v_ego=10.0,
    )[0]
    assert matcher.match(
      model_with_lead(
        point.d_rel, point.y_rel, 0.0, probability=0.0,
      ),
      (),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(point,),
      prefer_corner_stationary=True,
    ) is None

  time_s = 0.50
  point = snapshot_radar_points(
    (
      Point(
        1009,
        60.0 - 10.0 * time_s,
        0.1,
        v_rel=-10.0,
        source="corner235",
      ),
    ),
    v_ego=10.0,
  )[0]
  match = matcher.match(
    model_with_lead(
      point.d_rel, point.y_rel, 0.0, probability=0.0,
    ),
    (),
    STRAIGHT_PATH,
    time_s=time_s,
    stationary_points=(point,),
    prefer_corner_stationary=True,
  )

  assert match is not None
  assert match.point.track_id == 1009


def test_radar_only_stationary_pending_resets_after_center_support_loss() -> None:
  matcher = VisionRadarMatcher()
  for index in range(20):
    time_s = index * 0.05
    y_rel = 0.8 if index == 9 else 0.1
    point = snapshot_radar_points(
      (
        Point(
          1009,
          60.0 - 10.0 * time_s,
          y_rel,
          v_rel=-10.0,
          source="corner235",
        ),
      ),
      v_ego=10.0,
    )[0]
    match = matcher.match(
      model_with_lead(
        point.d_rel, point.y_rel, 0.0, probability=0.0,
      ),
      (),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(point,),
      prefer_corner_stationary=True,
    )
    assert match is None

  assert matcher.stationary_identity is None


def test_cross_source_stationary_support_cannot_bypass_center_gate() -> None:
  matcher = VisionRadarMatcher()
  for index in range(20):
    time_s = index * 0.05
    d_rel = 60.0 - 10.0 * time_s
    points = snapshot_radar_points(
      (
        Point(
          35,
          d_rel,
          1.0,
          v_rel=-10.0,
          source="frontRadar",
        ),
        Point(
          1009,
          d_rel + 0.5,
          1.0,
          v_rel=-10.0,
          source="corner235",
        ),
      ),
      v_ego=10.0,
    )
    match = matcher.match(
      model_with_lead(d_rel, 1.0, 0.0, probability=0.0),
      (),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=points,
    )
    assert match is None

  assert matcher.stationary_identity is None


def test_radar_only_stationary_projects_only_relevant_sources(
  monkeypatch: pytest.MonkeyPatch,
) -> None:
  matcher = VisionRadarMatcher()
  points = snapshot_radar_points(
    (
      *(
        Point(
          track_id,
          10.0 + track_id,
          5.0,
          v_rel=-10.0,
          source="frontRadar",
        )
        for track_id in range(1, 25)
      ),
      Point(35, 82.0, 0.2, v_rel=-10.0, source="frontRadar"),
      Point(1009, 80.0, 0.1, v_rel=-10.0, source="corner235"),
    ),
    v_ego=10.0,
  )
  projected: list[tuple[float, float]] = []

  def counted_projection(path, x, y):
    projected.append((x, y))
    return project_to_model_path(path, x, y)

  monkeypatch.setattr(
    "openpilot.selfdrive.carrot.radar_motion.primary.project_to_model_path",
    counted_projection,
  )
  matcher.match(
    model_with_lead(80.0, 0.1, 0.0, probability=0.0),
    (),
    STRAIGHT_PATH,
    time_s=0.0,
    stationary_points=points,
  )

  assert projected == [
    (points[-1].d_rel, points[-1].y_rel),
    (points[-2].d_rel, points[-2].y_rel),
  ]


def test_radar_only_stationary_hold_uses_narrow_path_gate() -> None:
  matcher = VisionRadarMatcher()
  match = None
  for index in range(11):
    time_s = index * 0.05
    point = snapshot_radar_points(
      (
        Point(
          1009,
          60.0 - 10.0 * time_s,
          0.1,
          v_rel=-10.0,
          source="corner235",
        ),
      ),
      v_ego=10.0,
    )[0]
    match = matcher.match(
      model_with_lead(
        point.d_rel, point.y_rel, 0.0, probability=0.0,
      ),
      (),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(point,),
      prefer_corner_stationary=True,
    )

  assert match is not None

  held_point = snapshot_radar_points(
    (
      Point(
        1009,
        54.5,
        1.0,
        v_rel=-10.0,
        source="corner235",
      ),
    ),
    v_ego=10.0,
  )[0]
  held = matcher.match(
    model_with_lead(
      held_point.d_rel, held_point.y_rel, 0.0, probability=0.0,
    ),
    (),
    STRAIGHT_PATH,
    time_s=0.55,
    stationary_points=(held_point,),
    prefer_corner_stationary=True,
  )
  released_point = replace(held_point, d_rel=54.0, y_rel=1.3)
  released = matcher.match(
    model_with_lead(
      released_point.d_rel,
      released_point.y_rel,
      0.0,
      probability=0.0,
    ),
    (),
    STRAIGHT_PATH,
    time_s=0.60,
    stationary_points=(released_point,),
    prefer_corner_stationary=True,
  )

  assert held is not None
  assert released is None
  assert matcher.stationary_identity is None


def test_adjacent_stationary_corner_stays_out_without_vision() -> None:
  matcher = VisionRadarMatcher()
  match = None
  for index in range(20):
    time_s = index * 0.05
    point = snapshot_radar_points(
      (
        Point(
          1009,
          100.0 - 10.0 * time_s,
          1.2,
          v_rel=-10.0,
          source="corner235",
        ),
      ),
      v_ego=10.0,
    )[0]
    match = matcher.match(
      model_with_lead(
        point.d_rel,
        point.y_rel,
        0.0,
        probability=0.0,
      ),
      (),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(point,),
      prefer_corner_stationary=True,
    )

  assert match is None
  assert matcher.stationary_identity is None


def test_stationary_match_prefers_continuous_corner_object() -> None:
  matcher = VisionRadarMatcher()
  match = None
  for index in range(7):
    time_s = index * 0.05
    d_rel = 100.0 - 10.0 * time_s
    points = snapshot_radar_points(
      (
        Point(35, d_rel + 3.0, 0.2, v_rel=-10.0),
        Point(
          1009,
          d_rel,
          0.3,
          v_rel=-10.0,
          source="corner235",
        ),
      ),
      v_ego=10.0,
    )
    match = matcher.match(
      model_with_lead(d_rel + 1.0, 0.2, 6.0, probability=0.45),
      (points[0],),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=points,
      prefer_corner_stationary=True,
    )

  assert match is not None
  assert match.point.source == "corner235"
  assert match.point.track_id == 1009


def test_confirmed_stationary_lead_tolerates_brief_model_path_outlier() -> None:
  matcher = VisionRadarMatcher()
  match = None
  for index in range(7):
    time_s = index * 0.05
    d_rel = 30.0 - 10.0 * time_s
    point = snapshot_radar_points(
      (
        Point(
          35,
          d_rel,
          0.0,
          v_rel=-10.0,
          source="frontRadar",
        ),
      ),
      v_ego=10.0,
    )[0]
    match = matcher.match(
      model_with_lead(d_rel, 0.0, 0.0, probability=0.89),
      (point,),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(point,),
      prefer_primary_stationary=True,
    )

  assert match is not None
  shifted_path = ((0.0, 5.0), (100.0, 5.0))
  for time_s in (0.35, 0.40):
    d_rel = 30.0 - 10.0 * time_s
    point = snapshot_radar_points(
      (
        Point(
          35,
          d_rel,
          0.0,
          v_rel=-10.0,
          source="frontRadar",
        ),
      ),
      v_ego=10.0,
    )[0]
    match = matcher.match(
      model_with_lead(d_rel, 0.0, 0.0, probability=0.89),
      (point,),
      shifted_path,
      time_s=time_s,
      stationary_points=(point,),
      prefer_primary_stationary=True,
    )
    assert match is not None
    assert match.point.track_id == 35

  time_s = 0.60
  d_rel = 30.0 - 10.0 * time_s
  point = snapshot_radar_points(
    (
      Point(
        35,
        d_rel,
        0.0,
        v_rel=-10.0,
        source="frontRadar",
      ),
    ),
    v_ego=10.0,
  )[0]
  released = matcher.match(
    model_with_lead(d_rel, 0.0, 0.0, probability=0.89),
    (point,),
    shifted_path,
    time_s=time_s,
    stationary_points=(point,),
    prefer_primary_stationary=True,
  )
  assert released is None


def test_confident_vision_lead_elsewhere_releases_stationary_hold() -> None:
  matcher = VisionRadarMatcher()
  for index in range(7):
    time_s = index * 0.05
    stationary = snapshot_radar_points(
      (
        Point(
          1009,
          100.0 - 10.0 * time_s,
          0.0,
          v_rel=-10.0,
          source="corner235",
        ),
      ),
      v_ego=10.0,
    )[0]
    matcher.match(
      model_with_lead(stationary.d_rel, 0.0, 5.0, probability=0.45),
      (),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(stationary,),
      prefer_corner_stationary=True,
    )

  moving = snapshot_radar_points(
    (Point(10, 30.0, 0.0, v_rel=2.0),),
    v_ego=10.0,
  )[0]
  released = matcher.match(
    model_with_lead(30.0, 0.0, 12.0, probability=0.9),
    (moving,),
    STRAIGHT_PATH,
    time_s=0.35,
    stationary_points=(moving,),
    prefer_corner_stationary=True,
  )

  assert released is not None
  assert released.point.track_id == 10
  assert matcher.stationary_identity is None


def test_controller_publishes_vision_seeded_continuous_corner_stationary_lead() -> None:
  controller = DPathRadarController(prefer_corner_radar=True)
  output = None
  for index in range(7):
    time_s = index * 0.05
    output = controller.update(
      time_s=time_s,
      v_ego=10.0,
      radar_points=(
        Point(
          1009,
          100.0 - 10.0 * time_s,
          0.1,
          v_rel=-10.0,
          source="corner235",
        ),
      ),
      model=model_with_lead(
        100.0 - 10.0 * time_s,
        0.1,
        7.0,
        probability=0.45 if index == 0 else 0.0,
      ),
    )

  assert output is not None
  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == 1009
  assert output.lead_one["vLead"] == pytest.approx(0.0)


def test_controller_prefers_front_for_vision_supported_stationary_lead_one() -> None:
  controller = DPathRadarController(prefer_corner_radar=True)
  output = None
  for index in range(7):
    time_s = index * 0.05
    front_d_rel = 30.0 - 10.0 * time_s
    output = controller.update(
      time_s=time_s,
      v_ego=10.0,
      radar_points=(
        Point(
          35,
          front_d_rel,
          0.1,
          v_rel=-10.0,
          source="frontRadar",
        ),
        Point(
          1009,
          front_d_rel - 0.8,
          0.1,
          v_rel=-10.0,
          source="corner235",
        ),
        Point(
          0,
          front_d_rel - 0.2,
          0.0,
          v_rel=-10.0,
          source="scc",
        ),
      ),
      model=model_with_lead(
        front_d_rel,
        0.1,
        0.0,
        probability=0.99,
      ),
    )

  assert output is not None
  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == 35
  assert output.lead_one["dRel"] == pytest.approx(front_d_rel)


def test_vision_only_mode_uses_vision_when_no_radar_is_available() -> None:
  output = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=-2,
  ).update(
    time_s=1.0,
    v_ego=10.0,
    radar_points=(),
    model=model_with_lead(
      30.0, 0.2, 12.0, probability=0.90,
    ),
  )

  assert output.lead_one is not None
  assert output.lead_one["status"]
  assert not output.lead_one["radar"]
  assert output.lead_one["radarTrackId"] == -1
  assert output.lead_one["dRel"] == pytest.approx(30.0)
  assert output.lead_one["vLead"] == pytest.approx(12.0)


def test_radar_mode_suppresses_central_vision_without_raw_corroboration() -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
    cut_in_sensitivity=0,
  )

  near = controller.update(
    time_s=1.0,
    v_ego=10.0,
    radar_points=(),
    model=model_with_lead(
      50.0, 0.0, 12.0, probability=0.90,
    ),
  )
  far = controller.update(
    time_s=1.05,
    v_ego=10.0,
    radar_points=(),
    model=model_with_lead(
      50.01, 0.0, 12.0, probability=0.90,
    ),
  )

  assert near.lead_one is None
  assert far.lead_one is None


def test_far_high_probability_vision_confirms_moving_radar_range() -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
    cut_in_sensitivity=0,
  )
  output = None
  for index, probability in enumerate(
    (0.94, 0.85, 0.93, 0.95, 0.94, 0.94),
  ):
    output = controller.update(
      time_s=index * 0.05,
      v_ego=20.0,
      radar_points=(Point(
        56,
        112.6 - index * 0.1,
        0.9,
        v_rel=-12.0,
        source="frontRadar",
      ),),
      model=model_with_lead(
        88.5 - index * 0.1,
        -0.3,
        16.0,
        probability=probability,
      ),
    )
    if index < 5:
      assert output.lead_one is None

  assert output is not None
  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == 56
  assert output.lead_one["radar"]
  assert output.lead_one["dRel"] == pytest.approx(112.1)
  assert output.lead_one["dRel"] != pytest.approx(88.0)


def test_far_loose_radar_match_requires_high_probability_seed() -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
    cut_in_sensitivity=0,
  )
  output = None
  for index in range(8):
    output = controller.update(
      time_s=index * 0.05,
      v_ego=20.0,
      radar_points=(Point(
        56,
        112.6 - index * 0.1,
        0.9,
        v_rel=-12.0,
        source="frontRadar",
      ),),
      model=model_with_lead(
        88.5 - index * 0.1,
        -0.3,
        16.0,
        probability=0.89,
      ),
    )

  assert output is not None
  assert output.lead_one is None


def test_radar_only_moving_corner_rejects_reported_velocity_conflict() -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
    cut_in_sensitivity=0,
  )
  output = None
  d_rel = 70.0
  for index in range(14):
    d_rel += 0.35 if index < 7 else -0.1
    output = controller.update(
      time_s=index * 0.05,
      v_ego=20.0,
      radar_points=(Point(
        1002,
        d_rel,
        0.1,
        v_rel=-2.0,
        source="corner235",
      ),),
      model=model_with_lead(
        90.0, 0.0, 20.0, probability=0.0,
      ),
    )
    assert output.lead_one is None

  assert output is not None


def test_rejected_corner_identity_can_reacquire_after_physical_break() -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
    cut_in_sensitivity=0,
  )
  for index in range(7):
    output = controller.update(
      time_s=index * 0.05,
      v_ego=20.0,
      radar_points=(Point(
        1002,
        70.0 + index * 0.35,
        0.1,
        v_rel=-2.0,
        source="corner235",
      ),),
      model=model_with_lead(
        90.0, 0.0, 20.0, probability=0.0,
      ),
    )
    assert output.lead_one is None

  output = None
  for index in range(7):
    output = controller.update(
      time_s=0.70 + index * 0.05,
      v_ego=20.0,
      radar_points=(Point(
        1002,
        60.0 - index * 0.1,
        0.1,
        v_rel=-2.0,
        source="corner235",
      ),),
      model=model_with_lead(
        90.0, 0.0, 20.0, probability=0.0,
      ),
    )

  assert output is not None
  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == 1002


def test_radar_only_moving_corner_accepts_consistent_range_rate() -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
    cut_in_sensitivity=0,
  )
  output = None
  for index in range(7):
    output = controller.update(
      time_s=index * 0.05,
      v_ego=20.0,
      radar_points=(Point(
        1002,
        70.0 - index * 0.1,
        0.1,
        v_rel=-2.0,
        source="corner235",
      ),),
      model=model_with_lead(
        90.0, 0.0, 20.0, probability=0.0,
      ),
    )

  assert output is not None
  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == 1002


def test_vision_match_miss_recovers_unmatched_raw_corner() -> None:
  output = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
  ).update(
    time_s=1.0,
    v_ego=10.0,
    radar_points=(
      Point(
        1009, 30.0, 0.1,
        v_rel=2.0, source="corner235",
      ),
    ),
    model=model_with_lead(
      30.0, 0.1, 12.0, probability=0.90,
    ),
  )

  assert output.lead_one is not None
  assert output.lead_one["radar"]
  assert output.lead_one["radarTrackId"] == 1009


def test_vision_match_miss_recovers_close_moving_front_radar() -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
  )
  model = model_with_lead(
    12.944, -0.070, 9.491, probability=0.996,
  )
  model.leadsV3[0].xStd = (2.496,)
  output = controller.update(
    time_s=23.04,
    v_ego=13.514,
    radar_points=(
      Point(
        56, 5.12, -1.87,
        v_rel=10.55 - 13.514,
        source="frontRadar",
      ),
      Point(
        32, 26.92, -0.04,
        v_rel=14.03 - 13.514,
        source="frontRadar",
      ),
    ),
    model=model,
  )

  assert output.lead_one is not None
  assert output.lead_one["radar"]
  assert output.lead_one["radarTrackId"] == 56
  assert output.lead_one["dRel"] == pytest.approx(5.12)


def test_stationary_front_rejects_tunnel_vision_speed_contradiction() -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
  )
  output = None
  for index in range(8):
    vision_d_rel = 45.02 - 1.84 * index * 0.05
    output = controller.update(
      time_s=index * 0.05,
      v_ego=17.44,
      radar_points=(
        Point(
          40,
          vision_d_rel - 12.60,
          0.34,
          v_rel=-17.62,
          source="frontRadar",
        ),
      ),
      model=model_with_lead(
        vision_d_rel,
        0.06,
        15.60,
        probability=0.82,
      ),
    )

  assert output is not None
  assert output.lead_one is None


def test_unconfirmed_stationary_front_cannot_enable_vision_fallback() -> None:
  output = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
  ).update(
    time_s=2.89,
    v_ego=17.46,
    radar_points=(
      Point(
        40,
        33.97,
        0.34,
        v_rel=-17.65,
        source="frontRadar",
      ),
    ),
    model=model_with_lead(
      26.06,
      0.01,
      9.14,
      probability=0.969,
    ),
  )

  assert output.lead_one is None


def test_radar_mode_keeps_sustained_vision_side_cutin_without_radar() -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
    cut_in_sensitivity=0,
  )
  output = None
  for index, d_path in enumerate(
    (2.00, 1.94, 1.88, 1.80, 1.70, 1.60, 1.50),
  ):
    output = controller.update(
      time_s=index * 0.05,
      v_ego=10.0,
      radar_points=(),
      model=model_with_lead(
        30.0, d_path, 9.0, probability=0.90,
      ),
    )
    if index < 5:
      assert output.lead_one is None

  assert output is not None
  assert output.lead_one is not None
  assert not output.lead_one["radar"]
  assert output.lead_one["radarTrackId"] == -1
  held = controller.update(
    time_s=0.35,
    v_ego=10.0,
    radar_points=(),
    model=model_with_lead(
      30.0, 1.45, 9.0, probability=0.36,
    ),
  )
  assert held.lead_one is not None
  assert not held.lead_one["radar"]


def test_near_vision_fallback_rejects_adjacent_raw_reflection() -> None:
  output = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
  ).update(
    time_s=1.0,
    v_ego=10.0,
    radar_points=(
      Point(
        35, 30.0, 2.5,
        v_rel=2.0, source="frontRadar",
      ),
    ),
    model=model_with_lead(
      30.0, 0.0, 12.0, probability=0.90,
    ),
  )

  assert output.lead_one is None


def test_radar_mode_suppresses_static_adjacent_vision_with_raw_radar() -> None:
  output = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
  ).update(
    time_s=1.0,
    v_ego=10.0,
    radar_points=(
      Point(
        1009, 30.0, 3.0,
        v_rel=2.0, source="corner235",
      ),
    ),
    model=model_with_lead(
      30.0, 3.0, 12.0, probability=0.90,
    ),
  )

  assert output.lead_one is None


def test_controller_prefers_front_over_better_scoring_scc_match() -> None:
  output = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=2,
  ).update(
    time_s=1.0,
    v_ego=10.0,
    radar_points=(
      Point(
        35, 31.0, 0.1,
        v_rel=-6.0, source="frontRadar",
      ),
      Point(
        0, 30.0, 0.0,
        v_rel=-6.0, source="scc",
      ),
    ),
    model=model_with_lead(
      30.0, 0.0, 4.0, probability=0.90,
    ),
  )

  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == 35


def test_controller_suppresses_vision_when_radar_distance_error_is_large() -> None:
  output = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
  ).update(
    time_s=1.0,
    v_ego=10.0,
    radar_points=(
      Point(
        35, 97.0, 0.0,
        v_rel=2.0, source="frontRadar",
      ),
    ),
    model=model_with_lead(
      80.0, 0.0, 12.0, probability=0.90,
    ),
  )

  assert output.lead_one is None


def test_controller_vision_only_acquires_at_point_four_and_holds_briefly() -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=-2,
  )

  rejected = controller.update(
    time_s=0.0,
    v_ego=10.0,
    radar_points=(),
    model=model_with_lead(
      30.0, 0.2, 12.0, probability=0.399,
    ),
  )
  acquired = controller.update(
    time_s=0.05,
    v_ego=10.0,
    radar_points=(),
    model=model_with_lead(
      30.0, 0.2, 12.0, probability=0.40,
    ),
  )
  held = controller.update(
    time_s=0.10,
    v_ego=10.0,
    radar_points=(),
    model=model_with_lead(
      30.0, 0.2, 12.0, probability=0.36,
    ),
  )

  assert rejected.lead_one is None
  assert acquired.lead_one is not None
  assert held.lead_one is not None
  assert held.lead_one["modelProb"] == pytest.approx(0.36)

  for index in range(9):
    held = controller.update(
      time_s=0.15 + index * 0.05,
      v_ego=10.0,
      radar_points=(),
      model=model_with_lead(
        30.0, 0.2, 12.0, probability=0.36,
      ),
    )
    assert held.lead_one is not None

  released = controller.update(
    time_s=0.60,
    v_ego=10.0,
    radar_points=(),
    model=model_with_lead(
      30.0, 0.2, 12.0, probability=0.36,
    ),
  )
  assert released.lead_one is None


def test_corner_support_strengthens_front_stationary_sticky_and_handoff() -> None:
  controller = DPathRadarController(prefer_corner_radar=True)
  output = None
  for index in range(7):
    time_s = index * 0.05
    d_rel = 30.0 - 10.0 * time_s
    output = controller.update(
      time_s=time_s,
      v_ego=10.0,
      radar_points=(
        Point(
          35, d_rel, 0.1,
          v_rel=-10.0, source="frontRadar",
        ),
        Point(
          1009, d_rel - 0.5, 0.1,
          v_rel=-10.0, source="corner235",
        ),
      ),
      model=model_with_lead(
        d_rel, 0.1, 0.0,
        probability=0.40 if index == 0 else 0.0,
      ),
    )

  assert output is not None
  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == 1009

  corner_handoff = controller.update(
    time_s=0.35,
    v_ego=10.0,
    radar_points=(
      Point(
        1009, 26.0, 0.1,
        v_rel=-10.0, source="corner235",
      ),
    ),
    model=model_with_lead(
      26.5, 0.1, 0.0, probability=0.0,
    ),
  )
  front_return = controller.update(
    time_s=0.40,
    v_ego=10.0,
    radar_points=(
      Point(
        35, 26.0, 0.1,
        v_rel=-10.0, source="frontRadar",
      ),
    ),
    model=model_with_lead(
      26.0, 0.1, 0.0, probability=0.0,
    ),
  )

  assert corner_handoff.lead_one is not None
  assert corner_handoff.lead_one["radarTrackId"] == 1009
  assert front_return.lead_one is not None
  assert front_return.lead_one["radarTrackId"] == 35


def test_stationary_sticky_releases_to_vision_on_large_distance_error() -> None:
  controller = DPathRadarController(prefer_corner_radar=True)
  output = None
  for index in range(7):
    time_s = index * 0.05
    d_rel = 30.0 - 10.0 * time_s
    output = controller.update(
      time_s=time_s,
      v_ego=10.0,
      radar_points=(
        Point(
          35, d_rel, 0.1,
          v_rel=-10.0, source="frontRadar",
        ),
        Point(
          1009, d_rel - 0.5, 0.1,
          v_rel=-10.0, source="corner235",
        ),
      ),
      model=model_with_lead(
        d_rel, 0.1, 0.0, probability=0.90,
      ),
    )

  assert output is not None
  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == 35

  released = controller.update(
    time_s=0.35,
    v_ego=10.0,
    radar_points=(
      Point(
        35, 26.5, 0.1,
        v_rel=-10.0, source="frontRadar",
      ),
      Point(
        1009, 26.0, 0.1,
        v_rel=-10.0, source="corner235",
      ),
    ),
    model=model_with_lead(
      45.0, 0.1, 0.0, probability=0.90,
    ),
  )

  assert released.lead_one is None


def test_controller_stationary_mismatch_falls_back_front_corner_scc() -> None:
  cases = (
    (
      (
        Point(
          35, 70.9, 0.1,
          v_rel=2.65 - 16.82, source="frontRadar",
        ),
        Point(
          1020, 95.8, 0.0,
          v_rel=-3.0 - 16.82, source="corner235",
        ),
        Point(
          0, 68.6, 0.0,
          v_rel=2.65 - 16.82, source="scc",
        ),
      ),
      1,
      35,
      "frontRadar",
    ),
    (
      (
        Point(
          1020, 78.0, 0.0,
          v_rel=-3.0 - 16.82, source="corner235",
        ),
        Point(
          0, 68.6, 0.0,
          v_rel=2.65 - 16.82, source="scc",
        ),
      ),
      1,
      1020,
      "corner235",
    ),
    (
      (
        Point(
          0, 68.6, 0.0,
          v_rel=2.65 - 16.82, source="scc",
        ),
      ),
      -1,
      0,
      "scc",
    ),
    (
      (
        Point(
          0, 68.6, 0.0,
          v_rel=2.65 - 16.82, source="scc",
        ),
      ),
      0,
      0,
      "scc",
    ),
    (
      (
        Point(
          0, 68.6, 0.0,
          v_rel=2.65 - 16.82, source="scc",
        ),
      ),
      1,
      None,
      None,
    ),
    (
      (
        Point(
          0, 68.6, 0.0,
          v_rel=2.65 - 16.82, source="scc",
        ),
      ),
      2,
      0,
      "scc",
    ),
    (
      (
        Point(
          0, 68.6, 0.0,
          v_rel=2.65 - 16.82, source="scc",
        ),
      ),
      -2,
      None,
      None,
    ),
  )

  for (
    points,
    enable_radar_tracks,
    expected_track_id,
    expected_source,
  ) in cases:
    controller = DPathRadarController(
      prefer_corner_radar=True,
      enable_radar_tracks=enable_radar_tracks,
    )
    output = None
    for index in range(7):
      output = controller.update(
        time_s=index * 0.05,
        v_ego=16.82,
        radar_points=points,
        model=model_with_lead(
          75.56, -0.32, 14.20, probability=0.951,
        ),
      )

    assert output is not None
    if expected_track_id is None:
      if enable_radar_tracks <= -2:
        assert output.lead_one is not None
        assert not output.lead_one["radar"]
        assert output.lead_one["radarTrackId"] == -1
      else:
        assert output.lead_one is None
      continue
    assert output.lead_one is not None
    assert output.lead_one["radar"]
    assert output.lead_one["radarTrackId"] == expected_track_id
    selected = next(
      point for point in points
      if point.track_id == expected_track_id
    )
    assert selected.source == expected_source


def test_distant_corner_without_correspondence_suppresses_vision() -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
  )
  output = None
  for index in range(7):
    output = controller.update(
      time_s=index * 0.05,
      v_ego=16.82,
      radar_points=(
        Point(
          1020, 95.8, 0.0,
          v_rel=-3.0 - 16.82, source="corner235",
        ),
      ),
      model=model_with_lead(
        75.56, -0.32, 14.20, probability=0.951,
      ),
    )

  assert output is not None
  assert output.lead_one is None


def test_in_path_moving_radar_fallback_prefers_front_then_corner_scc() -> None:
  cases = (
    (
      (
        Point(
          1009, 30.0, 0.1,
          v_rel=-5.0, source="corner235",
        ),
        Point(
          35, 30.2, 0.1,
          v_rel=-5.0, source="frontRadar",
        ),
        Point(
          0, 30.1, 0.1,
          v_rel=-5.0, source="scc",
        ),
      ),
      1,
      35,
    ),
    (
      (
        Point(
          1009, 30.0, 0.1,
          v_rel=-5.0, source="corner235",
        ),
        Point(
          0, 30.1, 0.1,
          v_rel=-5.0, source="scc",
        ),
      ),
      1,
      1009,
    ),
    (
      (
        Point(
          35, 30.2, 0.1,
          v_rel=-2.0, source="frontRadar",
        ),
        Point(
          0, 30.1, 0.1,
          v_rel=-2.0, source="scc",
        ),
      ),
      1,
      35,
    ),
    (
      (
        Point(
          0, 30.1, 0.1,
          v_rel=-2.0, source="scc",
        ),
      ),
      -1,
      0,
    ),
    (
      (
        Point(
          0, 30.1, 0.1,
          v_rel=-2.0, source="scc",
        ),
      ),
      0,
      0,
    ),
    (
      (
        Point(
          0, 30.1, 0.1,
          v_rel=-2.0, source="scc",
        ),
      ),
      1,
      None,
    ),
    (
      (
        Point(
          0, 30.1, 0.1,
          v_rel=-7.0, source="scc",
        ),
      ),
      2,
      0,
    ),
    (
      (
        Point(
          0, 30.1, 0.1,
          v_rel=-2.0, source="scc",
        ),
      ),
      2,
      None,
    ),
    (
      (
        Point(
          0, 30.1, 0.1,
          v_rel=-2.0, source="scc",
        ),
      ),
      -2,
      None,
    ),
  )

  for points, enable_radar_tracks, expected_track_id in cases:
    controller = DPathRadarController(
      prefer_corner_radar=True,
      enable_radar_tracks=enable_radar_tracks,
    )
    output = None
    # Moving candidates confirm at 0.25 s; the permitted SCC case in the
    # stationary speed band now uses the stricter 0.50 s radar-only dwell.
    for index in range(12):
      time_s = index * 0.05
      moving_points = tuple(
        replace(
          point,
          d_rel=point.d_rel + point.v_rel * time_s,
        )
        for point in points
      )
      output = controller.update(
        time_s=time_s,
        v_ego=10.0,
        radar_points=moving_points,
        model=model_with_lead(
          30.0, 0.0, 0.0, probability=0.0,
        ),
      )
      if index < 5:
        assert output.lead_one is None

    assert output is not None
    if expected_track_id is None:
      assert output.lead_one is None
      continue
    assert output.lead_one is not None
    assert output.lead_one["radar"]
    assert output.lead_one["radarTrackId"] == expected_track_id


def test_vision_range_outlier_falls_back_to_moving_front_radar() -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
  )
  output = None
  for index in range(7):
    time_s = index * 0.05
    output = controller.update(
      time_s=time_s,
      v_ego=35.0,
      radar_points=(
        Point(
          39, 50.2 - 5.5 * time_s, -0.05,
          v_rel=-5.5, source="frontRadar",
        ),
        Point(
          1005, 48.7 - 5.5 * time_s, -0.40,
          v_rel=-5.5, source="corner235",
        ),
      ),
      model=model_with_lead(
        81.0, 0.02, 32.0, probability=0.893,
      ),
    )
    if index < 5:
      assert output.lead_one is None

  assert output is not None
  assert output.lead_one is not None
  assert output.lead_one["radar"]
  assert output.lead_one["radarTrackId"] == 39


@pytest.mark.parametrize(
  ("v_lead", "expected_track_id"),
  ((2.4, None), (4.0, None), (4.1, 60)),
)
def test_moving_fallback_does_not_overlap_stationary_speed_band(
  v_lead: float,
  expected_track_id: int | None,
) -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
  )
  output = None
  v_ego = 26.7
  v_rel = v_lead - v_ego
  for index in range(7):
    time_s = index * 0.05
    output = controller.update(
      time_s=time_s,
      v_ego=v_ego,
      radar_points=(
        Point(
          60, 20.0 + v_rel * time_s, -0.05,
          v_rel=v_rel, source="frontRadar",
        ),
      ),
      model=model_with_lead(
        120.0, 0.0, 26.0, probability=0.15,
      ),
    )

  assert output is not None
  if expected_track_id is None:
    assert output.lead_one is None
  else:
    assert output.lead_one is not None
    assert output.lead_one["radarTrackId"] == expected_track_id


def test_normal_match_primes_moving_radar_fallback_for_range_jump() -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
  )
  output = None
  for index in range(6):
    time_s = index * 0.05
    radar_d_rel = 50.0 - 5.5 * time_s
    output = controller.update(
      time_s=time_s,
      v_ego=35.0,
      radar_points=(
        Point(
          39, radar_d_rel, -0.05,
          v_rel=-5.5, source="frontRadar",
        ),
      ),
      model=model_with_lead(
        radar_d_rel, 0.0, 29.5, probability=0.90,
      ),
    )
    assert output.lead_one is not None
    assert output.lead_one["radarTrackId"] == 39

  assert output is not None
  time_s = 0.30
  output = controller.update(
    time_s=time_s,
    v_ego=35.0,
    radar_points=(
      Point(
        39, 50.0 - 5.5 * time_s, -0.05,
        v_rel=-5.5, source="frontRadar",
      ),
    ),
    model=model_with_lead(
      81.0, 0.0, 32.0, probability=0.90,
    ),
  )

  assert output.lead_one is not None
  assert output.lead_one["radar"]
  assert output.lead_one["radarTrackId"] == 39


def test_confirmed_closer_moving_radar_overrides_farther_vision_match() -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
  )
  output = None
  for index in range(7):
    time_s = index * 0.05
    output = controller.update(
      time_s=time_s,
      v_ego=35.0,
      radar_points=(
        Point(
          39, 84.4 - 5.5 * time_s, 0.1,
          v_rel=-5.5, source="frontRadar",
        ),
        Point(
          45, 95.3 - 5.5 * time_s, -0.1,
          v_rel=-5.5, source="frontRadar",
        ),
        Point(
          1005, 82.5 - 5.5 * time_s, -0.4,
          v_rel=-5.5, source="corner235",
        ),
        Point(
          0, 84.8 - 5.5 * time_s, 0.0,
          v_rel=-5.5, source="scc",
        ),
      ),
      model=model_with_lead(
        107.2, 0.0, 29.5, probability=0.715,
      ),
    )
    assert output.lead_one is not None
    if index < 5:
      assert output.lead_one["radarTrackId"] == 45

  assert output is not None
  assert output.lead_one is not None
  assert output.lead_one["radar"]
  assert output.lead_one["radarTrackId"] == 39
  assert output.lead_one["modelProb"] == pytest.approx(0.0)


def test_confirmed_closer_moving_radar_replaces_held_farther_identity() -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
  )
  v_ego = 20.0
  v_rel = -2.0
  for index in range(7):
    time_s = index * 0.05
    far_d_rel = 35.0 + v_rel * time_s
    output = controller.update(
      time_s=time_s,
      v_ego=v_ego,
      radar_points=(Point(
        43, far_d_rel, 0.1,
        v_rel=v_rel, source="frontRadar",
      ),),
      model=model_with_lead(
        far_d_rel, 0.1, v_ego + v_rel, probability=0.99,
      ),
    )
    assert output.lead_one is not None
    assert output.lead_one["radarTrackId"] == 43

  output = None
  for index in range(7, 14):
    time_s = index * 0.05
    far_d_rel = 35.0 + v_rel * time_s
    output = controller.update(
      time_s=time_s,
      v_ego=v_ego,
      radar_points=(
        Point(
          34, far_d_rel - 9.0, 0.0,
          v_rel=v_rel, source="frontRadar",
        ),
        Point(
          43, far_d_rel, 0.1,
          v_rel=v_rel, source="frontRadar",
        ),
      ),
      model=model_with_lead(
        far_d_rel, 0.1, v_ego + v_rel, probability=0.99,
      ),
    )
    assert output.lead_one is not None
    if index < 12:
      assert output.lead_one["radarTrackId"] == 43

  assert output is not None
  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == 34
  assert output.lead_one["modelProb"] == pytest.approx(0.0)


def test_off_center_moving_radar_does_not_replace_held_lead_one() -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
  )
  v_ego = 20.0
  v_rel = -2.0
  output = None
  for index in range(14):
    time_s = index * 0.05
    far_d_rel = 35.0 + v_rel * time_s
    points = [Point(
      43, far_d_rel, 0.1,
      v_rel=v_rel, source="frontRadar",
    )]
    if index >= 7:
      points.insert(0, Point(
        34, far_d_rel - 9.0, 0.8,
        v_rel=v_rel, source="frontRadar",
      ))
    output = controller.update(
      time_s=time_s,
      v_ego=v_ego,
      radar_points=tuple(points),
      model=model_with_lead(
        far_d_rel, 0.1, v_ego + v_rel, probability=0.99,
      ),
    )

  assert output is not None
  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == 43


def test_closer_moving_radar_does_not_override_track_zero() -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
  )
  v_ego = 20.0
  v_rel = -2.0
  output = None
  for index in range(14):
    time_s = index * 0.05
    far_d_rel = 35.0 + v_rel * time_s
    points = [Point(
      0, far_d_rel, 0.0,
      v_rel=v_rel, source="frontRadar",
    )]
    if index >= 7:
      points.insert(0, Point(
        34, far_d_rel - 9.0, 0.0,
        v_rel=v_rel, source="frontRadar",
      ))
    output = controller.update(
      time_s=time_s,
      v_ego=v_ego,
      radar_points=tuple(points),
      model=model_with_lead(
        far_d_rel, 0.0, v_ego + v_rel, probability=0.99,
      ),
    )

  assert output is not None
  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == 0


def test_moving_corner_born_in_path_waits_for_lead_one_not_lead_two() -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
  )
  output = None
  for index in range(8):
    output = controller.update(
      time_s=index * 0.05,
      v_ego=10.0,
      radar_points=(
        Point(
          1009, 30.0, 0.1,
          v_rel=0.0, source="corner235",
        ),
      ),
      model=model_with_lead(
        30.0, 0.0, 0.0, probability=0.0,
      ),
    )
    assert output.lead_two is None

  assert output is not None
  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == 1009


def test_no_vision_adjacent_moving_corner_does_not_become_lead_one() -> None:
  controller = DPathRadarController(prefer_corner_radar=True)
  output = None
  for index in range(10):
    time_s = index * 0.05
    output = controller.update(
      time_s=time_s,
      v_ego=10.0,
      radar_points=(
        Point(
          1009,
          30.0 - 2.0 * time_s,
          1.3,
          v_rel=-2.0,
          source="corner235",
        ),
      ),
      model=model_with_lead(
        30.0, 0.0, 0.0, probability=0.0,
      ),
    )

  assert output is not None
  assert output.lead_one is None


def test_controller_accepts_radar_jitter_and_suppresses_stale_near_vision() -> None:
  point = Point(35, 30.0, 0.0, source="frontRadar")
  accepted = DPathRadarController(prefer_corner_radar=False).update(
    time_s=1.0,
    v_ego=10.0,
    radar_points=(point,),
    model=model_with_lead(30.0, 0.0, 10.0),
    radar_to_model_time_s=-0.11,
  )
  rejected = DPathRadarController(prefer_corner_radar=False).update(
    time_s=1.0,
    v_ego=10.0,
    radar_points=(point,),
    model=model_with_lead(30.0, 0.0, 10.0),
    radar_to_model_time_s=-0.16,
  )

  assert accepted.lead_one is not None
  assert accepted.lead_one["radarTrackId"] == 35
  assert rejected.lead_one is None


def test_controller_uses_sensor_specific_production_thresholds() -> None:
  corner = DPathRadarController(prefer_corner_radar=True)
  front = DPathRadarController(prefer_corner_radar=False)

  assert corner.motion_decisions.threshold == pytest.approx(0.30)
  assert front.motion_decisions.threshold == pytest.approx(0.67)


def test_controller_disables_new_lead_two_at_zero_sensitivity() -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    cut_in_sensitivity=0,
  )
  prediction = SimpleNamespace(
    source="corner235",
    track_id=1005,
    continuity_id=1,
    d_path=1.0,
    d_path_rate_long=-0.5,
    cut_out_probability=0.0,
    path_entry_probability=1.0,
    current_path_occupancy=True,
    reason="current path overlap",
    path_entry_age_s=0.0,
    time_to_entry_s=0.0,
  )
  controller.motion_predictor = FixedPredictor(prediction)
  controller.motion_decisions = FixedDecisionTracker(prediction)

  output = controller.update(
    time_s=1.0,
    v_ego=10.0,
    radar_points=(
      Point(10, 30.0, 0.0, source="frontRadar"),
      Point(1005, 20.0, 1.0, source="corner235"),
    ),
    model=model_with_lead(30.0, 0.0, 10.0),
  )

  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == 10
  assert output.lead_two is None
  assert output.leads_cutin == ()


def test_controller_matches_radard_lead_dynamics_and_raw_jerk() -> None:
  controller = DPathRadarController(prefer_corner_radar=False)
  hard_motion = Point(
    10,
    30.0,
    0.0,
    a_lead=1.0,
    j_lead=0.75,
  )
  hard = controller.update(
    time_s=1.0,
    v_ego=10.0,
    radar_points=(hard_motion,),
    model=model_with_lead(30.0, 0.0, 10.0),
    radar_reaction_factor=0.5,
  )

  assert hard.lead_one is not None
  assert hard.lead_one["aLeadTau"] == pytest.approx(1.35)
  assert hard.lead_one["jLead"] == pytest.approx(0.75)
  assert hard.leads_center[0]["aLeadTau"] == pytest.approx(1.35)
  assert hard.leads_center[0]["jLead"] == pytest.approx(0.75)

  quiet = controller.update(
    time_s=1.05,
    v_ego=10.0,
    radar_points=(replace(hard_motion, a_lead=0.0, j_lead=0.0),),
    model=model_with_lead(30.0, 0.0, 10.0),
    radar_reaction_factor=0.5,
  )

  assert quiet.lead_one is not None
  assert quiet.lead_one["aLeadTau"] == pytest.approx(0.75)
  assert quiet.lead_one["jLead"] == pytest.approx(0.0)


def test_corner_lead_two_uses_matched_front_dynamics() -> None:
  controller = DPathRadarController(prefer_corner_radar=True)
  prediction = SimpleNamespace(
    source="corner235",
    track_id=1019,
    continuity_id=1,
    d_path=2.0,
    d_path_rate_long=-0.5,
    cut_out_probability=0.0,
    path_entry_probability=0.8,
    current_path_occupancy=False,
    reason="confirmed physical CUT-IN",
    path_entry_age_s=0.0,
    time_to_entry_s=0.5,
  )
  controller.motion_predictor = FixedPredictor(prediction)
  controller.motion_decisions = FixedDecisionTracker(prediction)

  output = controller.update(
    time_s=1.0,
    v_ego=10.0,
    radar_points=(
      Point(
        35,
        20.0,
        2.0,
        source="frontRadar",
        a_lead=1.0,
        j_lead=0.75,
      ),
      Point(
        1019,
        20.0,
        2.0,
        source="corner235",
        a_lead=0.0,
        j_lead=0.0,
      ),
    ),
    model=model_with_lead(30.0, 0.0, 10.0, probability=0.0),
    radar_reaction_factor=0.5,
  )

  assert output.lead_two is not None
  assert output.lead_two["radarTrackId"] == 1019
  assert output.lead_two["aLead"] == pytest.approx(1.0)
  assert output.lead_two["jLead"] == pytest.approx(0.75)
  assert output.lead_two["aLeadTau"] == pytest.approx(1.35)


def test_cut_out_probability_does_not_remove_or_filter_control_leads() -> None:
  controller = DPathRadarController(prefer_corner_radar=True)
  exiting_primary = SimpleNamespace(
    source="corner235",
    track_id=1005,
    continuity_id=1,
    d_path=1.0,
    d_path_rate_long=0.8,
    cut_out_probability=0.91,
    cut_in_probability=0.0,
    path_entry_probability=0.0,
    current_path_occupancy=True,
    reason="tracked current path",
    path_entry_age_s=None,
    time_to_entry_s=None,
  )
  next_lead = SimpleNamespace(
    source="corner235",
    track_id=1006,
    continuity_id=2,
    d_path=0.2,
    d_path_rate_long=0.0,
    cut_out_probability=0.99,
    cut_in_probability=0.0,
    path_entry_probability=0.0,
    current_path_occupancy=True,
    reason="tracked current path",
    path_entry_age_s=None,
    time_to_entry_s=None,
  )
  controller.motion_predictor = SimpleNamespace(
    update=lambda *args, **kwargs: {
      ("corner235", 1005): exiting_primary,
      ("corner235", 1006): next_lead,
    },
  )
  controller.motion_decisions = FixedDecisionTracker(next_lead)

  output = controller.update(
    time_s=1.0,
    v_ego=10.0,
    radar_points=(
      Point(10, 30.0, 0.0, v_rel=2.0, source="frontRadar"),
      Point(1005, 30.5, 1.0, v_rel=2.0, source="corner235"),
      Point(1006, 20.0, 0.2, v_rel=1.0, source="corner235"),
    ),
    model=model_with_lead(30.0, 0.0, 12.0),
  )

  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == 10
  assert output.lead_two is not None
  assert output.lead_two["radarTrackId"] == 1006
  assert controller.primary_matcher.last_identity is not None


def test_primary_input_policy_matches_removed_model_radard() -> None:
  points = snapshot_radar_points(
    (
      Point(10, 30.0, 0.0, source="frontRadar"),
      Point(0, 30.0, 0.0, source="scc"),
      Point(1, 30.0, 0.0, v_rel=10.0, source="scc"),
      Point(1005, 20.0, 2.0, source="corner235"),
    ),
    v_ego=0.0,
  )

  assert [point.track_id for point in select_primary_radar_points(points, -2)] == []
  assert [point.track_id for point in select_primary_radar_points(points, 0)] == [0, 1]
  assert [point.track_id for point in select_primary_radar_points(points, 1)] == [10]
  assert [point.track_id for point in select_primary_radar_points(points, 2)] == [10, 0]


def test_independent_controller_calculates_lead_one_before_motion_lead_two() -> None:
  controller = DPathRadarController(prefer_corner_radar=True)
  prediction = SimpleNamespace(
    source="corner235",
    track_id=1005,
    continuity_id=1,
    d_path=1.0,
    d_path_rate_long=0.0,
    current_path_occupancy=False,
  )

  controller.motion_predictor = FixedPredictor(prediction)
  controller.motion_decisions = FixedDecisionTracker(prediction)
  output = controller.update(
    time_s=1.0,
    v_ego=10.0,
    radar_points=(
      Point(10, 30.0, 0.1, v_rel=2.0, source="frontRadar"),
      Point(1005, 20.0, 2.0, v_rel=0.0, source="corner235"),
    ),
    model=model_with_lead(30.0, 0.1, 12.0),
  )

  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == 10
  assert output.lead_two is not None
  assert output.lead_two["radarTrackId"] == 1005
  assert output.lead_two["dRel"] < output.lead_one["dRel"]
  assert output.lead_left is not None
  assert output.lead_left["radarTrackId"] == 1005


def test_independent_controller_retains_confirmed_lead_two_until_path_exit() -> None:
  controller = DPathRadarController(prefer_corner_radar=True)
  prediction = SimpleNamespace(
    source="corner235",
    track_id=1005,
    continuity_id=1,
    d_path=2.0,
    d_path_rate_long=0.0,
    path_entry_probability=0.8,
    current_path_occupancy=False,
  )
  controller.motion_predictor = FixedPredictor(prediction)
  controller.motion_decisions = FixedDecisionTracker(prediction)
  points = (
    Point(1005, 20.0, 2.0, source="corner235"),
  )

  detected = controller.update(
    time_s=1.0,
    v_ego=10.0,
    radar_points=points,
    model=model_with_lead(30.0, 0.0, 10.0, probability=0.0),
  )
  assert detected.lead_two is not None

  controller.motion_predictor = EmptyPredictor()
  controller.motion_decisions = EmptyDecisionTracker()
  retained = controller.update(
    time_s=1.1,
    v_ego=10.0,
    radar_points=(
      Point(1005, 20.0, 2.0, source="corner235", v_lead=0.0),
    ),
    model=model_with_lead(30.0, 0.0, 10.0, probability=0.0),
  )
  assert retained.lead_two is not None
  assert retained.lead_two["radarTrackId"] == 1005

  occluded = controller.update(
    time_s=1.15,
    v_ego=10.0,
    radar_points=(
      Point(1006, 6.0, 2.5, source="corner235"),
      Point(1005, 19.5, 2.0, source="corner235", v_lead=0.0),
    ),
    model=model_with_lead(30.0, 0.0, 10.0, probability=0.0),
  )
  assert occluded.lead_two is not None
  assert occluded.lead_two["radarTrackId"] == 1005

  controller.motion_predictor = FixedPredictor(prediction)
  prediction.d_path_rate_long = 1.0
  exited = controller.update(
    time_s=1.2,
    v_ego=10.0,
    radar_points=points,
    model=model_with_lead(30.0, 0.0, 10.0, probability=0.0),
  )
  assert exited.lead_two is None


def test_confirmed_cutin_falls_back_to_lead_two_when_lead_one_disappears() -> None:
  controller = DPathRadarController(prefer_corner_radar=True)
  prediction = SimpleNamespace(
    source="corner235",
    track_id=1005,
    continuity_id=1,
    d_path=1.0,
    d_path_rate_long=0.0,
    current_path_occupancy=False,
  )
  controller.motion_predictor = FixedPredictor(prediction)
  controller.motion_decisions = FixedDecisionTracker(prediction)
  points = (
    Point(10, 20.0, 1.0, v_rel=0.0, source="frontRadar"),
    Point(1005, 19.5, 1.1, v_rel=0.0, source="corner235"),
  )

  matched = controller.update(
    time_s=1.0,
    v_ego=10.0,
    radar_points=points,
    model=model_with_lead(20.0, 1.0, 10.0),
  )
  held = controller.update(
    time_s=1.1,
    v_ego=10.0,
    radar_points=points,
    model=model_with_lead(20.0, 1.0, 10.0, probability=0.0),
  )

  assert matched.lead_one is not None
  assert matched.lead_two is None
  assert held.lead_one is None
  assert held.lead_two is not None
  assert held.lead_two["radarTrackId"] == 1005


def test_production_dpath_mode_is_independent_of_conventional_radard() -> None:
  radard = Path(__file__).resolve().parents[2] / "controls" / "radard.py"
  dpath_radard = (
    Path(__file__).resolve().parents[1] / "radar" / "radard_dpath.py"
  )
  validation_replay = (
    Path(__file__).resolve().parents[1]
    / "radar"
    / "tools"
    / "radar_validation_replay.py"
  )
  process_config = (
    Path(__file__).resolve().parents[3]
    / "system"
    / "manager"
    / "process_config.py"
  )
  conventional_source = radard.read_text(encoding="utf-8")
  dpath_source = dpath_radard.read_text(encoding="utf-8")
  validation_source = validation_replay.read_text(encoding="utf-8")
  manager_source = process_config.read_text(encoding="utf-8")

  assert "RadarLeadModelMode" not in conventional_source
  assert "RadarMotionMode" not in conventional_source
  assert "CarrotRadarMode" not in conventional_source
  assert "RadarMotionPredictor" not in conventional_source
  assert "from openpilot.selfdrive.controls.radard" not in dpath_source
  assert 'getattr(sm["modelV2"], "timestampEof", 0)' in dpath_source
  assert 'params.get_int(\n        "CarrotRadarCutInSensitivity",' in dpath_source
  assert 'self.params.get_float("RadarReactionFactor") * 0.01' in dpath_source
  for field in (
    "leadOne",
    "leadTwo",
    "leadLeft",
    "leadRight",
    "leadsLeft",
    "leadsCenter",
    "leadsRight",
    "leadsCutIn",
    "leadsLeft2",
    "leadsRight2",
  ):
    assert f"self.radar_state.{field} =" in dpath_source
  assert "max_measurement_age_s=VALIDATION_CORNER_MAX_MEASUREMENT_AGE_S" in validation_source
  assert '"radard", "openpilot.selfdrive.controls.radard", conventional_radard' in manager_source
  assert '"radard_dpath", "openpilot.selfdrive.carrot.radar.radard_dpath", dpath_radard' in manager_source
