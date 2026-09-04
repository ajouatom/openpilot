from dataclasses import dataclass, replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.cereal import car
from openpilot.selfdrive.carrot.radar_motion.predictor import (
  CornerCutInPredecelTracker,
  CUT_IN_CURRENT_SCOPE_HALF_WIDTH_M,
  FRONT_CUT_IN_MIN_DREL_M,
  IMMEDIATE_LANE_SCOPE_HALF_WIDTH_M,
  RadarMotionDecisionTracker,
  RadarMotionHistorySample,
  RadarMotionPrediction,
  RadarMotionPredictor,
  corner_cutin_predecel_score,
  cutin_probability_at,
  model_path_point_at_s,
  model_path_y,
  prediction_sample_at,
  project_to_model_path,
  radar_motion_sensitivity,
  radar_target_velocity_in_ego_frame,
  turning_corner_path_entry_allowed,
  visible_motion_points,
)
from openpilot.selfdrive.carrot.radar_motion.lead_selection import (
  DPathLeadCandidate,
  DPathLeadTwoTracker,
  DPathStationaryPrimaryHandoffTracker,
  cutin_can_compete_with_primary,
  dpath_control_max_d_rel,
  front_cutin_motion_supported,
  lead_duplicates_primary,
  select_dpath_lead_two,
)
from openpilot.selfdrive.carrot.radar_motion.controller import (
  DPathRadarController,
  stationary_shadow_corner_supported,
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
  snapshot_live_radar_points,
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
  trackState: int = 0


STRAIGHT_PATH = ((0.0, 0.0), (100.0, 0.0))


def test_turning_corner_path_entry_rejects_far_lateral_projection_alias():
  assert not turning_corner_path_entry_allowed(
    "corner235", 8.05, 1.28, -0.37,
  )


@pytest.mark.parametrize(
  "source,y_rel,d_path,yaw_rate,cross_sensor_confirmed",
  (
    ("corner235", 8.05, 1.28, -0.19, False),
    ("corner235", 4.99, 1.28, -0.37, False),
    ("corner235", 8.05, 1.28, -0.37, True),
    ("frontRadar", 8.05, 1.28, -0.37, False),
  ),
)
def test_turning_corner_path_entry_preserves_supported_cases(
  source, y_rel, d_path, yaw_rate, cross_sensor_confirmed,
):
  assert turning_corner_path_entry_allowed(
    source,
    y_rel,
    d_path,
    yaw_rate,
    cross_sensor_confirmed=cross_sensor_confirmed,
  )


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


def test_lane_boundary_entry_rejects_sub_35cm_quantized_drift() -> None:
  predictor = RadarMotionPredictor()
  prediction = None
  lateral_values = (3.0,) * 18 + (
    2.98, 2.96, 2.94, 2.92, 2.90, 2.88, 2.86, 2.84,
    2.82, 2.80, 2.78, 2.76, 2.74, 2.72, 2.69,
  )
  for index, y_rel in enumerate(lateral_values):
    prediction = predictor.update(
      index * 0.05,
      (
        Point(
          2516,
          15.0 - index * 0.2,
          y_rel,
          source="corner235",
          v_rel=-4.0,
          v_lead=16.0,
          yv_rel=-0.2,
        ),
      ),
      STRAIGHT_PATH,
      v_ego=20.0,
    )[("corner235", 2516)]
  assert prediction is not None

  assert 0.20 <= prediction.directional_inward_displacement_m < 0.35
  assert prediction.directional_consistency >= 0.90
  assert prediction.d_path_rate_short < -0.25
  assert prediction.reported_normal_speed <= -0.20
  assert not prediction.lane_boundary_directional_entry
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


def test_requested_predictions_keep_unselected_front_histories_ready() -> None:
  reference = RadarMotionPredictor()
  selective = RadarMotionPredictor()
  selected_identity = ("frontRadar", 10)
  waiting_identity = ("frontRadar", 20)

  for index in range(12):
    points = (
      Point(10, 30.0 - index * 0.2, 0.2, v_rel=-2.0),
      Point(20, 42.0 - index * 0.2, 2.8 - index * 0.04, v_rel=-2.0),
    )
    reference_values = reference.update(
      index * 0.05, points, STRAIGHT_PATH, v_ego=20.0,
    )
    requested = (
      (waiting_identity,)
      if index == 11
      else (selected_identity,)
    )
    selective_values = selective.update(
      index * 0.05,
      points,
      STRAIGHT_PATH,
      v_ego=20.0,
      prediction_identities=requested,
    )

    assert set(selective_values) == set(requested)
    identity = requested[0]
    assert selective_values[identity] == reference_values[identity]

  assert selective_values[waiting_identity].history_count == 12


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


def test_cutin_confirmation_survives_raw_id_handoff() -> None:
  base = update_series(
    RadarMotionPredictor(),
    (3.0, 2.9, 2.8, 2.7, 2.6, 2.5),
    source="corner235",
    track_id=202,
  )
  first = replace(
    base,
    track_id=202,
    continuity_id=77,
    current_path_occupancy=False,
    cut_in_probability=1.0,
    path_entry_probability=1.0,
  )
  handed_off = replace(first, track_id=203)
  tracker = RadarMotionDecisionTracker(
    threshold=0.30,
    confirmation_s=0.35,
  )

  assert not tracker.update(0.0, (first,)).confirmed
  assert not tracker.update(0.2, (handed_off,)).confirmed
  decision = tracker.update(0.4, (handed_off,))

  assert decision.confirmed
  assert decision.confirmed[0].prediction.track_id == 203


def test_front_cutin_needs_stronger_path_relative_motion_than_corner() -> None:
  assert not front_cutin_motion_supported("frontRadar", 0.50)
  assert not front_cutin_motion_supported("frontRadar", 0.80)
  assert front_cutin_motion_supported(
    "frontRadar",
    0.80,
    d_path=-2.1,
    directional_consistency=0.80,
    directional_inward_sample_ratio=0.70,
  )
  assert not front_cutin_motion_supported(
    "frontRadar",
    0.80,
    d_path=-2.1,
    directional_consistency=0.47,
    directional_inward_sample_ratio=0.73,
    minimum_directional_consistency=0.71,
  )
  assert front_cutin_motion_supported(
    "frontRadar",
    -0.15,
    d_rel=12.0,
    d_path=2.1,
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
    d_path=2.1,
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


def test_front_forecast_only_cutin_waits_until_measured_near_path() -> None:
  common = {
    "d_rel": 30.0,
    "d_path_rate_short": 1.72,
    "predicted_path_overlap_s": 2.5,
    "directional_inward_displacement_m": 0.90,
    "directional_consistency": 0.89,
    "directional_inward_sample_ratio": 0.80,
  }

  assert not front_cutin_motion_supported(
    "frontRadar",
    0.83,
    d_path=-2.37,
    **common,
  )
  assert front_cutin_motion_supported(
    "frontRadar",
    0.83,
    d_path=-2.18,
    **common,
  )


def test_close_cross_sensor_cutin_accepts_sustained_moderate_motion_only() -> None:
  common = {
    "d_rel": 5.8,
    "d_path": -2.15,
    "d_path_rate_short": 0.34,
    "predicted_path_overlap_s": 2.0,
    "directional_inward_displacement_m": 0.44,
    "directional_consistency": 0.57,
    "directional_inward_sample_ratio": 0.53,
  }

  assert not front_cutin_motion_supported(
    "frontRadar", 0.30, **common,
  )
  assert front_cutin_motion_supported(
    "frontRadar", 0.30, cross_sensor_confirmed=True, **common,
  )
  assert not front_cutin_motion_supported(
    "frontRadar",
    0.30,
    cross_sensor_confirmed=True,
    directional_inward_displacement_m=0.10,
    **{
      key: value for key, value in common.items()
      if key != "directional_inward_displacement_m"
    },
  )
  assert not front_cutin_motion_supported(
    "frontRadar",
    0.30,
    cross_sensor_confirmed=True,
    d_rel=12.1,
    **{key: value for key, value in common.items() if key != "d_rel"},
  )


def test_far_corner_cutin_needs_strong_lateral_motion_not_closing_alone() -> None:
  common = {
    "d_rel": 20.0,
    "d_path": -2.68,
    "d_path_rate_short": 0.59,
    "current_path_occupancy": False,
  }

  assert not front_cutin_motion_supported(
    "corner180",
    0.43,
    v_rel=-2.20,
    **common,
  )
  assert front_cutin_motion_supported(
    "corner235",
    0.74,
    v_rel=-1.90,
    **common,
  )
  assert not front_cutin_motion_supported(
    "corner235",
    0.56,
    v_rel=-6.50,
    **common,
  )
  assert front_cutin_motion_supported(
    "corner235",
    0.56,
    v_rel=-6.50,
    corner_directional_entry=True,
    **common,
  )
  assert front_cutin_motion_supported(
    "corner180",
    0.43,
    d_path=-2.18,
    v_rel=-2.20,
    d_rel=20.0,
    current_path_occupancy=False,
  )


def test_distant_corner_path_overlap_needs_sustained_entry_history() -> None:
  common = {
    "d_rel": 64.5,
    "d_path": -1.42,
    "d_path_rate_short": 0.45,
    "current_path_occupancy": True,
    "directional_consistency": 0.90,
    "directional_inward_sample_ratio": 0.80,
  }

  assert not front_cutin_motion_supported(
    "corner235",
    0.22,
    directional_inward_displacement_m=0.22,
    **common,
  )
  assert front_cutin_motion_supported(
    "corner235",
    0.22,
    directional_inward_displacement_m=0.40,
    **common,
  )
  assert front_cutin_motion_supported(
    "corner235",
    0.05,
    d_rel=20.0,
    d_path=-1.42,
    current_path_occupancy=True,
  )


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
  controller = DPathRadarController(prefer_corner_radar=True)
  trajectory_detected = False
  output = None
  for index in range(30):
    time_s = index * 0.05
    side_y = 3.40 - 0.05 * index
    output = controller.update(
      time_s=time_s,
      v_ego=10.0,
      radar_points=(
        Point(10, 30.0, 0.0, v_lead=10.0, source="frontRadar"),
        Point(20, 27.5, side_y, v_rel=-7.0, v_lead=3.0),
        Point(
          1005, 27.4, side_y + 0.1, v_rel=-7.0, v_lead=3.0,
          yv_rel=-1.0, source="corner235", trackState=2,
        ),
      ),
      model=model_with_lead(30.0, 0.0, 10.0),
    )
    trajectory_detected |= any(
      estimate.confirmed_cutin
      for estimate in controller.trajectory_cutin.last_estimates
    )

  assert output is not None
  assert trajectory_detected
  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == 10
  assert output.lead_two is None
  assert output.lead_cutin_risk is None


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


def test_explicit_low_speed_identity_keeps_measured_motion_history() -> None:
  predictor = RadarMotionPredictor()
  identity = ("frontRadar", 49)
  prediction = None

  for index in range(6):
    values = predictor.update(
      index * 0.1,
      (Point(
        49,
        10.0 - 0.78 * index,
        -2.8 + 0.10 * index,
        v_rel=-7.8,
        v_lead=2.2,
      ),),
      STRAIGHT_PATH,
      v_ego=10.0,
      prediction_identities=(identity,),
      allow_low_speed_identities=(identity,),
    )
    prediction = values[identity]

  assert prediction is not None
  assert prediction.history_count == 6
  assert prediction.directional_inward_displacement_m > 0.4


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


def test_corner_raw_id_handoff_preserves_physical_motion_history() -> None:
  predictor = RadarMotionPredictor()
  first = None
  for index in range(5):
    first = predictor.update(
      index * 0.1,
      (Point(
        202,
        12.0,
        3.0 - index * 0.1,
        source="corner235",
        v_lead=10.0,
        yv_rel=-1.0,
      ),),
      STRAIGHT_PATH,
      v_ego=10.0,
    )[("corner235", 202)]

  assert first is not None
  handed_off = predictor.update(
    0.5,
    (Point(
      203,
      12.0,
      2.5,
      source="corner235",
      v_lead=10.0,
      yv_rel=-1.0,
    ),),
    STRAIGHT_PATH,
    v_ego=10.0,
  )[("corner235", 203)]

  assert handed_off.continuity_id == first.continuity_id
  assert handed_off.history_count == first.history_count + 1
  assert handed_off.directional_inward_displacement_m > 0.4


def test_corner_raw_id_handoff_does_not_merge_a_still_visible_target() -> None:
  predictor = RadarMotionPredictor()
  first = predictor.update(
    0.0,
    (Point(202, 12.0, 3.0, source="corner235", v_lead=10.0),),
    STRAIGHT_PATH,
    v_ego=10.0,
  )[("corner235", 202)]
  predictions = predictor.update(
    0.1,
    (
      Point(202, 12.0, 2.9, source="corner235", v_lead=10.0),
      Point(203, 12.2, 2.8, source="corner235", v_lead=10.0),
    ),
    STRAIGHT_PATH,
    v_ego=10.0,
  )

  assert predictions[("corner235", 202)].continuity_id == first.continuity_id
  assert predictions[("corner235", 203)].continuity_id != first.continuity_id


def test_corner_object_slot_swap_preserves_both_physical_tracks() -> None:
  predictor = RadarMotionPredictor()
  first = predictor.update(
    0.0,
    (
      Point(201, 20.0, -3.0, source="corner235", v_lead=10.0),
      Point(202, 12.0, 3.0, source="corner235", v_lead=10.0),
    ),
    STRAIGHT_PATH,
    v_ego=10.0,
  )
  swapped = predictor.update(
    0.1,
    (
      Point(201, 12.0, 2.9, source="corner235", v_lead=10.0),
      Point(202, 20.0, -2.9, source="corner235", v_lead=10.0),
    ),
    STRAIGHT_PATH,
    v_ego=10.0,
  )

  assert (
    swapped[("corner235", 201)].continuity_id
    == first[("corner235", 202)].continuity_id
  )
  assert (
    swapped[("corner235", 202)].continuity_id
    == first[("corner235", 201)].continuity_id
  )


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


def test_corner_position_history_override_holds_through_small_directional_jitter() -> None:
  predictor = RadarMotionPredictor()
  prediction = None
  lateral_values = tuple(3.5 - index * 0.08 for index in range(11))
  for index, y_rel in enumerate(lateral_values):
    prediction = predictor.update(
      index * 0.05,
      (
        Point(
          1010,
          25.0,
          y_rel,
          source="corner235",
          v_lead=10.0,
          yv_rel=-0.2,
        ),
      ),
      STRAIGHT_PATH,
      v_ego=10.0,
    )[("corner235", 1010)]

  assert prediction is not None
  assert prediction.directional_consistency >= 0.95
  assert prediction.motion_consistency > 0.9
  assert prediction.cut_in_probability > 0.5

  jittered = predictor.update(
    len(lateral_values) * 0.05,
    (
      Point(
        1010,
        25.0,
        lateral_values[-1] + 0.03,
        source="corner235",
        v_lead=10.0,
        yv_rel=-0.2,
      ),
    ),
    STRAIGHT_PATH,
    v_ego=10.0,
  )[("corner235", 1010)]

  assert 0.90 <= jittered.directional_consistency < 0.95
  assert jittered.directional_inward_displacement_m >= 0.35
  assert jittered.motion_consistency > 0.9
  assert jittered.cut_in_probability > 0.5


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


def test_stationary_shadow_may_be_farther_than_cutting_out_primary() -> None:
  primary = {
    "status": True,
    "radar": True,
    "radarTrackId": 44,
    "dRel": 50.0,
    "yRel": -1.0,
    "vLead": 10.0,
  }
  stopped = {
    "status": True,
    "radar": True,
    "radarTrackId": 53,
    "dRel": 60.0,
    "yRel": 0.0,
    "dPath": 0.0,
    "vLead": 0.0,
  }

  normal = select_dpath_lead_two(primary, (stopped,), v_ego=16.0)
  shadow = select_dpath_lead_two(
    primary,
    (stopped,),
    v_ego=16.0,
    allow_stopped_track_ids=frozenset((53,)),
    allow_farther_track_ids=frozenset((53,)),
  )

  assert normal.lead_two is None
  assert shadow.lead_two is stopped


def test_controller_only_runs_front_cut_out_prediction_for_lead_one(
  monkeypatch: pytest.MonkeyPatch,
) -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
    cut_in_sensitivity=3,
  )
  original_prediction = RadarMotionPredictor._prediction
  predicted_track_ids = []

  def counted_prediction(self, state, track_id, *args, **kwargs):
    if self is controller.primary_cut_out_predictor:
      predicted_track_ids.append(track_id)
    return original_prediction(self, state, track_id, *args, **kwargs)

  monkeypatch.setattr(
    RadarMotionPredictor, "_prediction", counted_prediction,
  )
  for index in range(10):
    output = controller.update(
      time_s=index * 0.05,
      v_ego=20.0,
      radar_points=(
        Point(44, 50.0, 0.0, v_rel=-10.0),
        Point(45, 58.0, 0.5, v_rel=-9.0),
        Point(46, 66.0, -0.5, v_rel=-8.0),
        Point(47, 74.0, 1.5, v_rel=-7.0),
      ),
      model=model_with_lead(
        50.0, 0.0, 10.0, probability=0.99,
      ),
    )
    assert output.lead_one is not None
    assert output.lead_one["radarTrackId"] == 44

  assert predicted_track_ids == [44] * 10
  assert set(controller.primary_cut_out_predictor._states["front"]) == {
    ("frontRadar", 44),
    ("frontRadar", 45),
    ("frontRadar", 46),
    ("frontRadar", 47),
  }


def test_controller_confirms_stationary_shadow_behind_cutting_out_lead() -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
    cut_in_sensitivity=3,
  )
  controller.primary_cut_out_predictor = FixedPredictor(SimpleNamespace(
    source="frontRadar",
    track_id=44,
    cut_out_probability=0.90,
  ))

  output = None
  for index in range(12):
    corner = (
      Point(
        1053, 66.0 - index * 0.8, 0.4,
        v_rel=-15.0, v_lead=1.0, source="corner235",
      ),
    ) if index <= 6 else ()
    output = controller.update(
      time_s=index * 0.05,
      v_ego=16.0,
      radar_points=(
        Point(
          44, 50.0, -1.0, v_rel=-6.0, v_lead=10.0,
          trackState=2,
        ),
        Point(
          53, 60.0 - index * 0.8, 0.0,
          v_rel=-16.0, v_lead=0.0, trackState=2,
        ),
      ) + corner,
      model=model_with_lead(50.0, -1.0, 10.0, probability=0.99),
    )
    assert output.lead_one is not None
    assert output.lead_one["radarTrackId"] == 44
    if index < 5:
      assert output.lead_two is None

  assert output is not None
  assert output.lead_two is not None
  assert output.lead_two["radarTrackId"] == 53
  assert output.lead_two["dRel"] > output.lead_one["dRel"]
  assert output.leads_cutin == ()


def test_controller_rejects_front_only_stationary_shadow() -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
    cut_in_sensitivity=3,
  )
  controller.primary_cut_out_predictor = FixedPredictor(SimpleNamespace(
    source="frontRadar",
    track_id=44,
    cut_out_probability=0.90,
  ))

  output = None
  for index in range(8):
    output = controller.update(
      time_s=index * 0.05,
      v_ego=16.0,
      radar_points=(
        Point(
          44, 50.0, -1.0, v_rel=-6.0, v_lead=10.0,
          trackState=2,
        ),
        Point(
          53, 60.0 - index * 0.8, 0.0,
          v_rel=-16.0, v_lead=0.0, trackState=2,
        ),
      ),
      model=model_with_lead(50.0, -1.0, 10.0, probability=0.99),
    )

  assert output is not None
  assert output.lead_two is None


def test_stationary_shadow_rejects_nearby_moving_corner_return() -> None:
  path = STRAIGHT_PATH
  front = snapshot_radar_points((
    Point(
      34, 54.0, -0.5, v_rel=-24.0, v_lead=0.0,
      trackState=2,
    ),
  ), v_ego=24.0)[0]
  points = snapshot_radar_points((
    Point(
      34, 54.0, -0.5, v_rel=-24.0, v_lead=0.0,
      trackState=2,
    ),
    Point(
      1000, 56.6, -0.05, v_rel=-0.1, v_lead=23.9,
      source="corner235",
    ),
  ), v_ego=24.0)

  assert not stationary_shadow_corner_supported(front, points, path)


def test_stationary_corner_primary_handoff_becomes_and_remains_lead_two() -> None:
  handoff = DPathStationaryPrimaryHandoffTracker()
  lead_two = DPathLeadTwoTracker()
  corner_lead = {
    "status": True,
    "radar": True,
    "radarTrackId": 1039,
    "dRel": 45.0,
    "yRel": 0.2,
    "dPath": 0.2,
    "vRel": -19.0,
    "vLead": 1.0,
    "modelProb": 0.85,
  }
  corner = DPathLeadCandidate(
    corner_lead, "corner235", 1039, 0, True, False,
  )

  assert handoff.update(0.0, corner_lead, (corner,), None) is None

  farther_primary = {
    "status": True,
    "radar": True,
    "radarTrackId": 55,
    "dRel": 64.0,
    "yRel": 0.0,
    "vRel": -10.0,
    "vLead": 10.0,
    "modelProb": 0.9,
  }
  moved_lead = dict(corner_lead, dRel=39.3)
  moved = replace(corner, lead=moved_lead)
  acquired = handoff.update(0.3, farther_primary, (moved,), None)

  assert acquired is not None
  assert acquired.confirmed_stationary_shadow
  assert lead_two.update(
    0.3, farther_primary, (acquired,), 20.0,
  ).lead_two is acquired.lead

  retained = None
  for index in range(4, 13):
    time_s = index * 0.1
    retained_candidate = replace(
      corner,
      lead=dict(corner_lead, dRel=45.0 - 19.0 * time_s),
    )
    retained = handoff.update(
      time_s,
      farther_primary,
      (retained_candidate,),
      lead_two.active_identity,
    )
    assert retained is retained_candidate
    assert not retained.confirmed_stationary_shadow
    assert lead_two.update(
      time_s, farther_primary, (retained,), 20.0,
    ).lead_two is retained.lead

  assert retained is not None


def test_stationary_corner_handoff_cannot_duplicate_primary() -> None:
  primary = {
    "status": True,
    "radar": True,
    "radarTrackId": 55,
    "dRel": 38.0,
    "yRel": 1.0,
    "vRel": -7.7,
    "vLead": -0.2,
    "modelProb": 0.93,
  }
  corner_lead = {
    "status": True,
    "radar": True,
    "radarTrackId": 5961,
    "dRel": 35.8,
    "yRel": 0.6,
    "dPath": -0.2,
    "vRel": -7.5,
    "vLead": 0.0,
    "modelProb": 0.93,
  }
  corner = DPathLeadCandidate(
    lead=corner_lead,
    source="corner180",
    track_id=5961,
    continuity_id=0,
    retainable=True,
    confirmed_cutin=False,
    confirmed_stationary_shadow=True,
  )

  selection = DPathLeadTwoTracker().update(
    0.3, primary, (corner,), v_ego=7.7,
  )

  assert lead_duplicates_primary(corner_lead, primary)
  assert selection.lead_two is None


def test_stationary_corner_primary_handoff_rejects_weak_or_reused_identity() -> None:
  base_lead = {
    "status": True,
    "radar": True,
    "radarTrackId": 1039,
    "dRel": 45.0,
    "yRel": 0.2,
    "dPath": 0.2,
    "vRel": -19.0,
    "vLead": 1.0,
    "modelProb": 0.39,
  }
  corner = DPathLeadCandidate(
    base_lead, "corner235", 1039, 0, True, False,
  )
  farther_primary = {
    "status": True,
    "radar": True,
    "radarTrackId": 55,
    "dRel": 80.0,
    "vLead": 10.0,
    "modelProb": 0.9,
  }

  weak = DPathStationaryPrimaryHandoffTracker()
  assert weak.update(0.0, base_lead, (corner,), None) is None
  assert weak.update(0.1, farther_primary, (corner,), None) is None

  reused = DPathStationaryPrimaryHandoffTracker()
  strong_lead = dict(base_lead, modelProb=0.85)
  strong = replace(corner, lead=strong_lead)
  assert reused.update(0.0, strong_lead, (strong,), None) is None
  jumped = replace(
    corner,
    lead=dict(strong_lead, dRel=70.0),
  )
  assert reused.update(0.1, farther_primary, (jumped,), None) is None


@pytest.mark.parametrize(
  ("track_state", "cut_out_probability"),
  ((0, 0.90), (1, 0.90), (2, 0.69)),
)
def test_controller_rejects_unconfirmed_stationary_shadow(
  track_state: int,
  cut_out_probability: float,
) -> None:
  controller = DPathRadarController(enable_radar_tracks=1)
  controller.primary_cut_out_predictor = FixedPredictor(SimpleNamespace(
    source="frontRadar",
    track_id=44,
    cut_out_probability=cut_out_probability,
  ))

  output = None
  for index in range(8):
    output = controller.update(
      time_s=index * 0.05,
      v_ego=16.0,
      radar_points=(
        Point(
          44, 50.0, -1.0, v_rel=-6.0, v_lead=10.0,
          trackState=2,
        ),
        Point(
          53, 60.0 - index * 0.8, 0.0,
          v_rel=-16.0, v_lead=0.0, trackState=track_state,
        ),
      ),
      model=model_with_lead(50.0, -1.0, 10.0, probability=0.99),
    )

  assert output is not None
  assert output.lead_two is None


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


def test_confirmed_lead_two_survives_raw_id_handoff_of_same_physical_track() -> None:
  tracker = DPathLeadTwoTracker()
  first_lead = {
    "status": True,
    "radar": True,
    "radarTrackId": 202,
    "dRel": 12.0,
    "yRel": 2.5,
    "vRel": 0.0,
    "vLat": -1.0,
    "vLead": 10.0,
  }
  first = DPathLeadCandidate(
    first_lead, "corner235", 202, 77, True, True,
  )
  assert tracker.update(0.0, None, (first,), 10.0).lead_two is first_lead

  next_lead = dict(first_lead, radarTrackId=203, yRel=2.4)
  handed_off = DPathLeadCandidate(
    next_lead, "corner235", 203, 77, True, False,
  )
  assert tracker.update(0.1, None, (handed_off,), 10.0).lead_two is next_lead
  assert tracker.active_identity == handed_off.identity


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


def test_controller_publishes_strong_corner_cutin_without_delaying_lead_two() -> None:
  controller = DPathRadarController(prefer_corner_radar=True)
  model = SimpleNamespace(
    position=SimpleNamespace(x=(0.0, 100.0), y=(0.0, 0.0)),
    leadsV3=(),
    velocity=SimpleNamespace(x=(30.0,)),
  )
  first_risk = None
  for index in range(18):
    output = controller.update(
      index * 0.05,
      30.0,
      (Point(
        2091,
        25.0 - 0.325 * index,
        3.60 - 0.05 * index,
        v_rel=-6.5,
        source="corner235",
        v_lead=23.5,
        yv_rel=-1.0,
        trackState=2,
      ),),
      model,
    )
    if output.lead_cutin_risk is not None and first_risk is None:
      first_risk = index * 0.05, output

  assert first_risk is not None
  risk_time_s, output = first_risk
  assert risk_time_s <= 0.65
  assert output.lead_two is not None
  assert output.lead_cutin_risk["radarTrackId"] == 2091
  assert output.lead_cutin_risk["score"] > 0.75
  assert output.lead_cutin_risk["vRel"] == pytest.approx(-6.5)


def test_close_low_speed_cutin_predecel_uses_moving_cross_sensor_target() -> None:
  prediction = SimpleNamespace(
    sensor="corner",
    source="corner180",
    current_path_occupancy=False,
    history_count=40,
    d_path=-2.40,
    d_path_rate_short=0.36,
    d_path_rate_long=0.41,
    reported_normal_speed=0.26,
    directional_inward_displacement_m=0.34,
    directional_consistency=0.96,
    directional_inward_sample_ratio=0.75,
    motion_consistency=0.83,
    recent_motion_support=0.87,
  )

  score = corner_cutin_predecel_score(
    prediction,
    3.10,
    -2.00,
    v_ego=7.69,
    cross_sensor_confirmed=True,
  )

  assert score >= 0.20
  assert corner_cutin_predecel_score(
    prediction,
    3.10,
    -2.00,
    v_ego=7.69,
    cross_sensor_confirmed=False,
  ) == 0.0


@pytest.mark.parametrize(
  ("v_ego", "v_rel", "prediction_overrides"),
  (
    (15.0, -2.0, {}),
    (7.69, -7.69, {}),
    (7.69, -2.0, {"reported_normal_speed": 0.0}),
    (7.69, -2.0, {"directional_consistency": 0.70}),
  ),
)
def test_close_low_speed_cutin_predecel_rejects_unsafe_shortcuts(
  v_ego: float,
  v_rel: float,
  prediction_overrides: dict[str, float],
) -> None:
  values = {
    "sensor": "corner",
    "source": "corner180",
    "current_path_occupancy": False,
    "history_count": 40,
    "d_path": -2.40,
    "d_path_rate_short": 0.36,
    "d_path_rate_long": 0.41,
    "reported_normal_speed": 0.26,
    "directional_inward_displacement_m": 0.34,
    "directional_consistency": 0.96,
    "directional_inward_sample_ratio": 0.75,
    "motion_consistency": 0.83,
    "recent_motion_support": 0.87,
  }
  values.update(prediction_overrides)

  assert corner_cutin_predecel_score(
    SimpleNamespace(**values),
    3.10,
    v_rel,
    v_ego=v_ego,
    cross_sensor_confirmed=True,
  ) == 0.0


def test_controller_selects_cross_sensor_slow_close_cutin() -> None:
  controller = DPathRadarController(prefer_corner_radar=True)
  output = None
  selected_pickup = []

  for index in range(30):
    time_s = index * 0.1
    primary_d_rel = 25.0 - 0.05 * index
    pickup_d_rel = 13.0 - 0.12 * index
    pickup_y_rel = -2.9 + 0.04 * index
    output = controller.update(
      time_s,
      10.0,
      (
        Point(
          35,
          primary_d_rel,
          0.0,
          v_rel=-4.0,
          v_lead=6.0,
        ),
        Point(
          49,
          pickup_d_rel,
          pickup_y_rel,
          v_rel=-7.8,
          v_lead=2.2,
        ),
        Point(
          16687,
          pickup_d_rel - 0.25,
          pickup_y_rel - 0.15,
          v_rel=-7.7,
          v_lead=2.3,
          source="corner235",
          trackState=2,
        ),
      ),
      model_with_lead(primary_d_rel, 0.0, 6.0),
    )
    if output.lead_two is not None:
      selected_pickup.append((time_s, output.lead_two))

  assert output is not None
  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == 35
  assert selected_pickup
  assert selected_pickup[0][0] <= 2.1
  # Lateral identity stays with corner radar while longitudinal dynamics come
  # from the mutually matched front-radar track.
  assert selected_pickup[0][1]["radarTrackId"] == 16687
  assert selected_pickup[0][1]["vLead"] == pytest.approx(2.2)


def test_controller_trajectory_cutin_adds_early_risk_and_lead_two() -> None:
  controller = DPathRadarController(prefer_corner_radar=True)
  first_risk_s = None
  first_lead_two_s = None

  for index in range(36):
    time_s = index * 0.05
    target_d_rel = 8.0 - 1.7 * time_s
    target_y_rel = -3.30 + 0.70 * time_s
    output = controller.update(
      time_s=time_s,
      v_ego=7.9,
      radar_points=(
        Point(
          35, 25.0, 0.0, v_rel=0.0, source="frontRadar",
        ),
        Point(
          45, target_d_rel, target_y_rel,
          v_rel=-1.7, source="frontRadar",
        ),
        Point(
          3504, target_d_rel - 0.1, target_y_rel - 0.1,
          v_rel=-1.6, yv_rel=0.70, source="corner235", trackState=2,
        ),
      ),
      model=model_with_lead(25.0, 0.0, 7.9),
    )
    if output.lead_cutin_risk is not None and first_risk_s is None:
      first_risk_s = time_s
    if output.lead_two is not None and first_lead_two_s is None:
      first_lead_two_s = time_s

  assert first_risk_s is not None
  assert first_lead_two_s is not None
  assert first_risk_s <= first_lead_two_s
  assert first_risk_s <= 0.85
  assert first_lead_two_s <= 1.46
  assert output.lead_two is not None
  assert output.lead_two["radarTrackId"] == 3504


def test_corner_cutin_predecel_requires_continuous_confirmation() -> None:
  tracker = CornerCutInPredecelTracker(confirmation_s=0.10, hold_s=0.20)
  candidate = RadarMotionCutIn(SimpleNamespace(
    source="corner235",
    track_id=2091,
    continuity_id=1,
  ), 0.9)

  assert tracker.update(0.00, (candidate,)) is None
  assert tracker.update(0.05, ()) is None
  assert tracker.update(0.11, (candidate,)) is None
  assert tracker.update(0.16, (candidate,)) is None
  assert tracker.update(0.22, (candidate,)) is not None
  assert tracker.update(0.35, ()) is not None
  assert tracker.update(0.45, ()) is None


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


def test_stationary_front_rejects_opposite_side_uncertain_vision_match() -> None:
  matcher = VisionRadarMatcher()
  for index in range(8):
    time_s = index * 0.05
    point = snapshot_radar_points(
      (
        Point(
          45,
          69.8 - 9.54 * time_s,
          1.32,
          v_rel=-9.54,
          source="frontRadar",
        ),
      ),
      v_ego=9.05,
    )[0]
    model = model_with_lead(
      62.35 - 4.13 * time_s,
      -2.42,
      4.92,
      probability=0.72,
    )
    model.leadsV3[0].xStd = (10.8,)
    model.leadsV3[0].yStd = (1.7,)
    model.leadsV3[0].vStd = (3.25,)

    match = matcher.match(
      model,
      (point,),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(point,),
      prefer_primary_stationary=True,
    )

    assert match is None

  assert matcher.stationary_identity is None


def test_stationary_front_rejects_offset_moving_vision_median_reflection() -> None:
  matcher = VisionRadarMatcher()
  for index in range(8):
    time_s = index * 0.05
    point = snapshot_radar_points(
      (
        Point(
          37,
          30.8 - 10.0 * time_s,
          0.8,
          v_rel=-10.0,
          source="frontRadar",
        ),
      ),
      v_ego=10.0,
    )[0]
    match = matcher.match(
      model_with_lead(
        33.4 - 8.0 * time_s,
        -0.5,
        8.0,
        probability=0.88,
      ),
      (point,),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(point,),
      prefer_primary_stationary=True,
      yaw_rate_rad_s=0.04,
    )

    assert match is None

  assert matcher.stationary_identity is None


def test_stationary_front_preserves_cross_sensor_support_during_turn() -> None:
  matcher = VisionRadarMatcher()
  match = None
  for index in range(7):
    time_s = index * 0.05
    d_rel = 30.8 - 10.0 * time_s
    front, corner = snapshot_radar_points(
      (
        Point(
          37,
          d_rel,
          0.8,
          v_rel=-10.0,
          source="frontRadar",
        ),
        Point(
          1037,
          d_rel + 0.2,
          0.8,
          v_rel=-10.0,
          source="corner235",
        ),
      ),
      v_ego=10.0,
    )
    match = matcher.match(
      model_with_lead(
        d_rel + 2.6,
        -0.5,
        8.0,
        probability=0.88,
      ),
      (front,),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(front, corner),
      prefer_primary_stationary=True,
      yaw_rate_rad_s=0.04,
    )

  assert match is not None
  assert match.point.track_id == 37


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


def test_stationary_front_hands_off_to_persistent_closer_vision_match() -> None:
  matcher = VisionRadarMatcher()
  match = None
  for index in range(7):
    time_s = index * 0.05
    far = snapshot_radar_points(
      (Point(59, 50.0 - 10.0 * time_s, 0.1, v_rel=-10.0),),
      v_ego=10.0,
    )[0]
    match = matcher.match(
      model_with_lead(far.d_rel, far.y_rel, 0.0, probability=1.0),
      (far,),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(far,),
      prefer_primary_stationary=True,
    )

  assert match is not None
  assert match.point.track_id == 59

  selected_ids = []
  for index in range(7, 14):
    time_s = index * 0.05
    far = Point(59, 50.0 - 10.0 * time_s, 0.1, v_rel=-10.0)
    closer = Point(46, far.d_rel - 2.0, 0.15, v_rel=-10.0)
    points = snapshot_radar_points((far, closer), v_ego=10.0)
    match = matcher.match(
      model_with_lead(
        points[1].d_rel, points[1].y_rel, 0.0, probability=1.0,
      ),
      points,
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=points,
      prefer_primary_stationary=True,
    )
    assert match is not None
    selected_ids.append(match.point.track_id)

  assert selected_ids[:5] == [59] * 5
  assert selected_ids[-1] == 46
  assert matcher.stationary_identity == ("frontRadar", 46)


def test_stationary_front_hands_off_to_offset_closer_vision_range() -> None:
  matcher = VisionRadarMatcher()
  match = None
  for index in range(7):
    time_s = index * 0.05
    held = snapshot_radar_points(
      (Point(55, 4.55, -0.30, v_rel=0.0),),
      v_ego=0.0,
    )[0]
    match = matcher.match(
      model_with_lead(4.55, -0.30, 0.0, probability=1.0),
      (held,),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(held,),
      prefer_primary_stationary=True,
    )

  assert match is not None
  assert match.point.track_id == 55

  selected_ids = []
  for index in range(7, 14):
    time_s = index * 0.05
    points = snapshot_radar_points((
      Point(55, 4.55, -0.30, v_rel=0.0),
      Point(35, 2.40, 0.75, v_rel=0.0),
    ), v_ego=0.0)
    model = model_with_lead(2.80, 0.0, 0.0, probability=1.0)
    model.leadsV3[0].xStd = (0.44,)
    model.leadsV3[0].yStd = (0.22,)
    model.leadsV3[0].vStd = (0.07,)
    match = matcher.match(
      model,
      points,
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=points,
      prefer_primary_stationary=True,
    )
    assert match is not None
    selected_ids.append(match.point.track_id)

  assert selected_ids[:5] == [55] * 5
  assert selected_ids[-1] == 35
  assert matcher.stationary_identity == ("frontRadar", 35)


def test_stationary_front_rejects_offset_challenger_without_range_gain() -> None:
  matcher = VisionRadarMatcher()
  match = None
  for index in range(7):
    time_s = index * 0.05
    held = snapshot_radar_points(
      (Point(55, 4.55, -0.30, v_rel=0.0),),
      v_ego=0.0,
    )[0]
    match = matcher.match(
      model_with_lead(4.55, -0.30, 0.0, probability=1.0),
      (held,),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(held,),
      prefer_primary_stationary=True,
    )

  assert match is not None
  assert match.point.track_id == 55

  for index in range(7, 15):
    time_s = index * 0.05
    points = snapshot_radar_points((
      Point(55, 4.55, -0.30, v_rel=0.0),
      Point(35, 2.40, 0.75, v_rel=0.0),
    ), v_ego=0.0)
    match = matcher.match(
      model_with_lead(4.10, 0.0, 0.0, probability=1.0),
      points,
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=points,
      prefer_primary_stationary=True,
    )
    assert match is not None
    assert match.point.track_id == 55

  assert matcher.stationary_identity == ("frontRadar", 55)


def test_stationary_front_ignores_transient_closer_vision_match() -> None:
  matcher = VisionRadarMatcher()
  match = None
  for index in range(7):
    time_s = index * 0.05
    far = snapshot_radar_points(
      (Point(59, 50.0 - 10.0 * time_s, 0.1, v_rel=-10.0),),
      v_ego=10.0,
    )[0]
    match = matcher.match(
      model_with_lead(far.d_rel, far.y_rel, 0.0, probability=1.0),
      (far,),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(far,),
      prefer_primary_stationary=True,
    )

  assert match is not None
  assert match.point.track_id == 59

  for index in range(7, 11):
    time_s = index * 0.05
    far = Point(59, 50.0 - 10.0 * time_s, 0.1, v_rel=-10.0)
    closer = Point(46, far.d_rel - 2.0, 0.15, v_rel=-10.0)
    points = snapshot_radar_points((far, closer), v_ego=10.0)
    match = matcher.match(
      model_with_lead(
        points[1].d_rel, points[1].y_rel, 0.0, probability=1.0,
      ),
      points,
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=points,
      prefer_primary_stationary=True,
    )
    assert match is not None
    assert match.point.track_id == 59

  time_s = 0.55
  far = snapshot_radar_points(
    (Point(59, 50.0 - 10.0 * time_s, 0.1, v_rel=-10.0),),
    v_ego=10.0,
  )[0]
  retained = matcher.match(
    model_with_lead(far.d_rel, far.y_rel, 0.0, probability=1.0),
    (far,),
    STRAIGHT_PATH,
    time_s=time_s,
    stationary_points=(far,),
    prefer_primary_stationary=True,
  )

  assert retained is not None
  assert retained.point.track_id == 59
  assert matcher.stationary_identity == ("frontRadar", 59)


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


def test_stationary_front_corner_pair_survives_visual_range_outlier() -> None:
  matcher = VisionRadarMatcher()
  match = None
  for index in range(8):
    time_s = index * 0.05
    d_rel = 100.0 - 17.0 * time_s
    points = snapshot_radar_points(
      (
        Point(
          45,
          d_rel,
          0.2,
          v_rel=-17.0,
          source="frontRadar",
        ),
        Point(
          1004,
          d_rel - 1.0,
          0.5,
          v_rel=-15.0,
          source="corner235",
        ),
      ),
      v_ego=20.0,
    )
    match = matcher.match(
      model_with_lead(
        d_rel + 20.0,
        0.3,
        14.0,
        probability=0.50 if index == 0 else 0.10,
      ),
      (),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=points,
      prefer_primary_stationary=True,
    )

  assert match is not None
  assert match.point.source == "frontRadar"
  assert match.point.track_id == 45
  assert matcher.stationary_identity == ("frontRadar", 45)


def test_corner_supported_stationary_front_releases_after_path_departure() -> None:
  matcher = VisionRadarMatcher()
  match = None
  for index in range(7):
    time_s = index * 0.05
    d_rel = 40.0 - 10.0 * time_s
    front, corner = snapshot_radar_points(
      (
        Point(
          48, d_rel, 0.1, v_rel=-10.0, yv_rel=0.0,
          source="frontRadar", trackState=2,
        ),
        Point(
          1280, d_rel - 0.8, 0.3, v_rel=-10.0,
          source="corner235",
        ),
      ),
      v_ego=10.0,
    )
    match = matcher.match(
      model_with_lead(d_rel, 0.1, 0.0, probability=0.90),
      (front,),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(front, corner),
      prefer_primary_stationary=True,
    )

  assert match is not None
  assert match.point.track_id == 48

  matches = []
  for index, y_rel in enumerate(
    (
      0.4, 0.7, 1.0, 1.3, 1.6, 1.9, 2.1, 2.3,
      2.5, 2.7, 2.8, 2.8, 2.8, 2.8,
    ),
    start=7,
  ):
    time_s = index * 0.05
    d_rel = 40.0 - 10.0 * time_s
    front, corner = snapshot_radar_points(
      (
        Point(
          48, d_rel, y_rel, v_rel=-10.0, yv_rel=4.0,
          source="frontRadar", trackState=2,
        ),
        Point(
          1280, d_rel - 0.8, y_rel + 0.2, v_rel=-10.0,
          source="corner235",
        ),
      ),
      v_ego=10.0,
    )
    matches.append(matcher.match(
      model_with_lead(
        d_rel + 15.0, -3.0, 8.0, probability=0.08,
      ),
      (front,),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(front, corner),
      prefer_primary_stationary=True,
    ))

  assert matches[0] is not None
  assert all(match is not None for match in matches[:11])
  assert any(match is None for match in matches[11:])
  assert matches[-1] is None
  assert matcher.stationary_identity is None


def test_corner_supported_stationary_front_tolerates_brief_path_departure() -> None:
  matcher = VisionRadarMatcher()
  match = None
  for index in range(7):
    time_s = index * 0.05
    d_rel = 40.0 - 10.0 * time_s
    front, corner = snapshot_radar_points(
      (
        Point(
          48, d_rel, 0.1, v_rel=-10.0,
          source="frontRadar", trackState=2,
        ),
        Point(
          1280, d_rel - 0.8, 0.3, v_rel=-10.0,
          source="corner235",
        ),
      ),
      v_ego=10.0,
    )
    match = matcher.match(
      model_with_lead(d_rel, 0.1, 0.0, probability=0.90),
      (front,),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(front, corner),
      prefer_primary_stationary=True,
    )

  assert match is not None
  for index, y_rel in enumerate(
    (0.5, 0.9, 1.3, 1.7, 2.1, 2.4, 2.3, 1.8),
    start=7,
  ):
    time_s = index * 0.05
    d_rel = 40.0 - 10.0 * time_s
    front, corner = snapshot_radar_points(
      (
        Point(
          48, d_rel, y_rel, v_rel=-10.0, yv_rel=4.0,
          source="frontRadar", trackState=2,
        ),
        Point(
          1280, d_rel - 0.8, y_rel + 0.2, v_rel=-10.0,
          source="corner235",
        ),
      ),
      v_ego=10.0,
    )
    match = matcher.match(
      model_with_lead(
        d_rel + 15.0, -3.0, 8.0, probability=0.08,
      ),
      (front,),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(front, corner),
      prefer_primary_stationary=True,
    )
    assert match is not None

  assert matcher.stationary_identity == ("frontRadar", 48)
  assert matcher._stationary_front_departure_since_s is None


def test_stationary_visual_range_outlier_needs_matching_corner() -> None:
  matcher = VisionRadarMatcher()
  for index in range(8):
    time_s = index * 0.05
    d_rel = 100.0 - 17.0 * time_s
    point = snapshot_radar_points(
      (
        Point(
          45,
          d_rel,
          0.2,
          v_rel=-17.0,
          source="frontRadar",
        ),
      ),
      v_ego=20.0,
    )[0]
    match = matcher.match(
      model_with_lead(
        d_rel + 20.0,
        0.3,
        14.0,
        probability=0.50,
      ),
      (),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(point,),
      prefer_primary_stationary=True,
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


def test_stationary_front_position_lock_recovers_model_speed_error() -> None:
  matcher = VisionRadarMatcher()
  match = None
  for index in range(18):
    time_s = index * 0.05
    point = snapshot_radar_points(
      (
        Point(
          43,
          125.0 - 23.4 * time_s,
          -0.6,
          v_rel=-23.8,
          source="frontRadar",
          trackState=2,
        ),
      ),
      v_ego=23.4,
    )[0]
    match = matcher.match(
      model_with_lead(
        point.d_rel - 1.0,
        -0.3,
        15.9,
        probability=0.75,
      ),
      (),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(point,),
      prefer_primary_stationary=True,
    )

  assert match is not None
  assert match.point.source == "frontRadar"
  assert match.point.track_id == 43


@pytest.mark.parametrize(
  "track_state,distance_error,yaw_rate",
  (
    (1, 1.0, 0.0),
    (2, 5.5, 0.0),
    (2, 1.0, 0.021),
  ),
)
def test_stationary_front_position_lock_rejects_weak_geometry(
  track_state: int,
  distance_error: float,
  yaw_rate: float,
) -> None:
  matcher = VisionRadarMatcher()
  match = None
  for index in range(18):
    time_s = index * 0.05
    point = snapshot_radar_points(
      (
        Point(
          43,
          125.0 - 23.4 * time_s,
          -0.6,
          v_rel=-23.8,
          source="frontRadar",
          trackState=track_state,
        ),
      ),
      v_ego=23.4,
    )[0]
    match = matcher.match(
      model_with_lead(
        point.d_rel - distance_error,
        -0.3,
        15.9,
        probability=0.75,
      ),
      (),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(point,),
      prefer_primary_stationary=True,
      yaw_rate_rad_s=yaw_rate,
    )

  assert match is None
  assert matcher.stationary_identity is None


def test_stationary_corner_cannot_bypass_confirmation_via_vision_recovery() -> None:
  matcher = VisionRadarMatcher()
  v_ego = 22.2
  for index in range(5):
    time_s = index * 0.05
    point = snapshot_radar_points(
      (
        Point(
          1078,
          79.0 - 20.0 * time_s,
          0.8,
          v_rel=1.5 - v_ego,
          source="corner235",
        ),
      ),
      v_ego=v_ego,
    )[0]
    match = matcher.match(
      model_with_lead(
        point.d_rel + 8.5,
        -0.5,
        21.3,
        probability=0.65,
      ),
      (),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(point,),
      prefer_primary_stationary=True,
    )
    assert match is None

  assert matcher.stationary_identity is None


def test_moving_corner_vision_recovery_requires_observed_age() -> None:
  matcher = VisionRadarMatcher()
  v_ego = 22.2
  match = None
  for index in range(6):
    time_s = index * 0.05
    point = snapshot_radar_points(
      (
        Point(
          1077,
          87.0 - 15.0 * time_s,
          -1.3,
          v_rel=7.2 - v_ego,
          source="corner235",
        ),
      ),
      v_ego=v_ego,
    )[0]
    match = matcher.match(
      model_with_lead(
        point.d_rel + 0.5,
        -0.5,
        22.0,
        probability=0.85,
      ),
      (),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(point,),
      prefer_primary_stationary=True,
    )
    if index < 5:
      assert match is None

  assert match is not None
  assert match.point.track_id == 1077


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
          80.0 - 10.0 * time_s,
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
        80.0 - 10.0 * time_s,
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


def test_close_corner_only_stationary_reflection_cannot_seed_lead() -> None:
  matcher = VisionRadarMatcher()
  for index in range(30):
    time_s = index * 0.05
    point = snapshot_radar_points(
      (
        Point(
          2294,
          55.0 - 12.0 * time_s,
          max(0.0, 1.2 - 0.6 * time_s),
          v_rel=-12.0,
          source="corner235",
        ),
      ),
      v_ego=13.0,
    )[0]
    match = matcher.match(
      model_with_lead(
        95.0, 2.0, 13.0, probability=0.2,
      ),
      (),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(point,),
      prefer_corner_stationary=True,
    )
    assert match is None

  assert matcher.stationary_identity is None


def test_radar_only_stationary_corner_range_jumps_restart_confirmation() -> None:
  matcher = VisionRadarMatcher()
  ranges_m = (
    23.20, 21.55, 20.75, 18.30, 18.30, 15.95, 17.35, 16.55,
    14.90, 13.35, 11.70, 10.50, 8.80, 8.75, 6.30, 9.25,
    6.80, 6.80, 3.55, 1.05,
  )
  lateral_m = (
    -0.70, -0.65, -0.60, -0.50, -0.50, -0.40, -0.40, -0.40,
    -0.35, -0.30, -0.25, -0.20, -0.10, -0.15, -0.10, -0.10,
    -0.05, -0.05, -0.05, 0.0,
  )

  for index, (d_rel, y_rel) in enumerate(zip(
    ranges_m, lateral_m, strict=True,
  )):
    point = snapshot_radar_points((Point(
      1910,
      d_rel,
      y_rel,
      v_rel=-29.8,
      v_lead=-0.4,
      source="corner235",
    ),), v_ego=29.4)[0]
    match = matcher.match(
      model_with_lead(115.0, 8.5, 30.0, probability=0.002),
      (),
      STRAIGHT_PATH,
      time_s=index * 0.05,
      stationary_points=(point,),
      prefer_corner_stationary=True,
    )

    assert match is None

  assert matcher.stationary_identity is None


def test_radar_only_stationary_pending_resets_after_center_support_loss() -> None:
  matcher = VisionRadarMatcher()
  for index in range(20):
    time_s = index * 0.05
    y_rel = 0.8 if index == 9 else 0.1
    point = snapshot_radar_points(
      (
        Point(
          1009,
          80.0 - 10.0 * time_s,
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
          80.0 - 10.0 * time_s,
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

  held = None
  held_point = None
  for index in range(1, 22):
    time_s = 0.5 + index * 0.05
    held_point = snapshot_radar_points(
      (
        Point(
          1009,
          75.0 - 10.0 * (time_s - 0.5),
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
      time_s=time_s,
      stationary_points=(held_point,),
      prefer_corner_stationary=True,
    )
    assert held is not None

  assert held_point is not None
  released_point = replace(held_point, d_rel=64.0, y_rel=1.3)
  released = matcher.match(
    model_with_lead(
      released_point.d_rel,
      released_point.y_rel,
      0.0,
      probability=0.0,
    ),
    (),
    STRAIGHT_PATH,
    time_s=1.60,
    stationary_points=(released_point,),
    prefer_corner_stationary=True,
  )

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


def test_controller_does_not_publish_corner_stationary_as_lead_one() -> None:
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
  assert output.lead_one is None
  assert any(
    lead["radarTrackId"] == 1009
    for lead in output.leads_center
  )


def test_controller_turn_rejects_weak_vision_stationary_corner_seed() -> None:
  controller = DPathRadarController(prefer_corner_radar=True)
  output = None
  for index in range(12):
    time_s = index * 0.05
    output = controller.update(
      time_s=time_s,
      v_ego=6.0,
      radar_points=(Point(
        2734,
        14.0 - 6.0 * time_s,
        -0.5,
        v_rel=-6.0,
        source="corner235",
      ),),
      model=model_with_lead(
        14.0 - 6.0 * time_s,
        -0.5,
        0.0,
        probability=0.65,
      ),
      yaw_rate_rad_s=0.20,
    )

  assert output is not None
  assert (
    output.lead_one is None
    or output.lead_one["radarTrackId"] != 2734
  )
  assert controller.primary_matcher.stationary_identity is None


def test_controller_turn_uses_vision_instead_of_unmatched_corner_as_l1() -> None:
  controller = DPathRadarController(prefer_corner_radar=True)
  output = None
  for index in range(7):
    time_s = index * 0.05
    output = controller.update(
      time_s=time_s,
      v_ego=6.0,
      radar_points=(Point(
        1009,
        14.0 - 6.0 * time_s,
        0.1,
        v_rel=-6.0,
        source="corner235",
      ),),
      model=model_with_lead(
        14.0 - 6.0 * time_s,
        0.1,
        0.0,
        probability=0.85,
      ),
      yaw_rate_rad_s=0.20,
    )

  assert output is not None
  assert output.lead_one is not None
  assert not output.lead_one["radar"]


def test_controller_turn_uses_vision_instead_of_corner_only_l1() -> None:
  controller = DPathRadarController(prefer_corner_radar=True)
  output = None
  for index in range(7):
    time_s = index * 0.05
    output = controller.update(
      time_s=time_s,
      v_ego=6.0,
      radar_points=(Point(
        1009,
        20.0 - 6.0 * time_s,
        0.1,
        v_rel=-6.0,
        source="corner235",
      ),),
      model=model_with_lead(
        20.0 - 6.0 * time_s,
        0.1,
        0.0,
        probability=0.45,
      ),
    )

  assert output is not None
  assert output.lead_one is not None
  assert not output.lead_one["radar"]

  retained = controller.update(
    time_s=0.35,
    v_ego=6.0,
    radar_points=(Point(
      1009,
      17.9,
      0.2,
      v_rel=-6.0,
      source="corner235",
    ),),
    model=model_with_lead(17.9, 0.2, 0.0, probability=0.0),
    yaw_rate_rad_s=0.20,
  )

  assert retained.lead_one is None


def test_weak_vision_accelerates_only_tight_front_corner_stationary_pair() -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
    cut_in_sensitivity=0,
  )
  output = None
  for index in range(8):
    front_d_rel = 90.0 - index * 0.6
    output = controller.update(
      time_s=index * 0.05,
      v_ego=12.0,
      radar_points=(
        Point(
          60,
          front_d_rel,
          0.45,
          v_rel=-12.0,
          source="frontRadar",
        ),
        Point(
          1003,
          front_d_rel + 4.5,
          0.20,
          v_rel=-12.0,
          source="corner235",
        ),
      ),
      model=model_with_lead(
        front_d_rel - 2.0,
        0.20,
        0.0,
        probability=0.23 if index == 0 else 0.05,
      ),
    )
    if index < 7:
      assert output.lead_one is None

  assert output is not None
  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == 60
  assert output.lead_one["dRel"] == pytest.approx(85.8)


@pytest.mark.parametrize(
  ("weak_probability", "include_corner", "lateral_m"),
  (
    (0.19, True, 0.20),
    (0.23, False, 0.20),
    (0.23, True, 1.20),
  ),
)
def test_weak_vision_cannot_promote_without_tight_physical_pair(
  weak_probability: float,
  include_corner: bool,
  lateral_m: float,
) -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
    cut_in_sensitivity=0,
  )
  output = None
  for index in range(8):
    front_d_rel = 90.0 - index * 0.6
    radar_points = [Point(
      60,
      front_d_rel,
      lateral_m,
      v_rel=-12.0,
      source="frontRadar",
    )]
    if include_corner:
      radar_points.append(Point(
        1003,
        front_d_rel + 4.5,
        lateral_m,
        v_rel=-12.0,
        source="corner235",
      ))
    output = controller.update(
      time_s=index * 0.05,
      v_ego=12.0,
      radar_points=tuple(radar_points),
      model=model_with_lead(
        front_d_rel - 2.0,
        lateral_m,
        0.0,
        probability=weak_probability if index == 0 else 0.05,
      ),
    )

  assert output is not None
  assert output.lead_one is None


@pytest.mark.parametrize(
  ("source", "y_rel", "expected_track_id"),
  (
    ("frontRadar", 1.5, None),
    ("frontRadar", 0.5, 34),
    ("corner235", 1.5, 1005),
  ),
)
def test_weak_vision_releases_only_offset_front_stationary_hold(
  source: str,
  y_rel: float,
  expected_track_id: int | None,
) -> None:
  matcher = VisionRadarMatcher()
  track_id = 1005 if source.startswith("corner") else 34
  match = None
  for index in range(7):
    time_s = index * 0.05
    point = snapshot_radar_points((Point(
      track_id,
      37.0,
      y_rel,
      v_rel=0.0,
      source=source,
    ),), v_ego=0.0)[0]
    match = matcher.match(
      model_with_lead(
        30.6,
        0.0,
        0.2,
        probability=0.42,
      ),
      (point,),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(point,),
      prefer_primary_stationary=True,
    )

  assert match is not None
  assert match.point.track_id == track_id

  point = snapshot_radar_points((Point(
    track_id,
    36.9,
    y_rel,
    v_rel=-0.1,
    source=source,
  ),), v_ego=0.1)[0]
  match = matcher.match(
    model_with_lead(
      55.0,
      0.0,
      10.0,
      probability=0.01,
    ),
    (point,),
    STRAIGHT_PATH,
    time_s=0.35,
    stationary_points=(point,),
    prefer_primary_stationary=True,
  )

  assert (None if match is None else match.point.track_id) == expected_track_id


def test_held_stationary_corner_ignores_bounded_velocity_outlier() -> None:
  matcher = VisionRadarMatcher()
  match = None
  v_leads = (0.0, 0.0, -4.5, -5.4, -6.2, -7.1, -5.7)
  for index, v_lead in enumerate(v_leads):
    time_s = index * 0.05
    d_rel = 122.0 - index * 2.0
    point = snapshot_radar_points((Point(
      1005,
      d_rel,
      -0.6,
      v_rel=v_lead - 23.0,
      source="corner235",
    ),), v_ego=23.0)[0]
    match = matcher.match(
      model_with_lead(
        d_rel - 8.0,
        -1.0,
        18.0,
        probability=0.51,
      ),
      (),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(point,),
      prefer_corner_stationary=True,
    )
    if index < 5:
      assert match is None
    else:
      assert match is not None
      assert match.point.track_id == 1005
      assert match.point.v_lead == pytest.approx(v_lead)

  assert matcher.stationary_identity == ("corner235", 1005)


def test_held_stationary_corner_hands_off_to_continuous_new_slot() -> None:
  matcher = VisionRadarMatcher()
  v_leads = (0.0, 0.0, -4.5, -5.4, -6.2, -7.1, -5.7)
  for index, v_lead in enumerate(v_leads):
    time_s = index * 0.05
    d_rel = 122.0 - index * 2.0
    point = snapshot_radar_points((Point(
      1005,
      d_rel,
      -0.6,
      v_rel=v_lead - 23.0,
      source="corner235",
    ),), v_ego=23.0)[0]
    match = matcher.match(
      model_with_lead(d_rel - 8.0, -1.0, 18.0, probability=0.70),
      (),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(point,),
      prefer_corner_stationary=True,
    )

  assert match is not None
  replacement = snapshot_radar_points((Point(
    1010,
    107.8,
    -0.55,
    v_rel=-29.0,
    source="corner235",
  ),), v_ego=23.0)[0]
  match = matcher.match(
    model_with_lead(100.0, -1.0, 17.0, probability=0.90),
    (),
    STRAIGHT_PATH,
    time_s=0.35,
    stationary_points=(replacement,),
    prefer_corner_stationary=True,
  )

  assert match is not None
  assert match.point.track_id == 1010
  assert matcher.stationary_identity == ("corner235", 1010)


def test_held_stationary_corner_bridges_one_missing_measurement() -> None:
  matcher = VisionRadarMatcher()
  match = None
  v_leads = (0.0, 0.0, -4.5, -5.4, -6.2, -7.1, -5.7)
  for index, v_lead in enumerate(v_leads):
    time_s = index * 0.05
    d_rel = 122.0 - index * 2.0
    point = snapshot_radar_points((Point(
      1005,
      d_rel,
      -0.6,
      v_rel=v_lead - 23.0,
      source="corner235",
    ),), v_ego=23.0)[0]
    match = matcher.match(
      model_with_lead(d_rel - 8.0, -1.0, 18.0, probability=0.70),
      (),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(point,),
      prefer_corner_stationary=True,
    )

  assert match is not None
  held = matcher.match(
    model_with_lead(101.0, -1.0, 17.0, probability=0.90),
    (),
    STRAIGHT_PATH,
    time_s=0.35,
    stationary_points=(),
    prefer_corner_stationary=True,
  )

  assert held is not None
  assert held.point.track_id == 1005
  assert not held.point.measured


def test_corner_supported_front_stationary_tolerates_vision_range_noise() -> None:
  matcher = VisionRadarMatcher()
  match = None
  for index in range(7):
    time_s = index * 0.05
    d_rel = 80.0 - index * 0.5
    front = snapshot_radar_points((Point(
      41,
      d_rel,
      0.1,
      v_rel=-10.0,
      source="frontRadar",
    ),), v_ego=10.0)[0]
    corner = snapshot_radar_points((Point(
      1005,
      d_rel + 0.2,
      0.1,
      v_rel=-10.0,
      source="corner235",
    ),), v_ego=10.0)[0]
    match = matcher.match(
      model_with_lead(d_rel, 0.1, 0.0, probability=0.90),
      (front,),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(front, corner),
      prefer_primary_stationary=True,
    )

  assert match is not None
  assert match.point.track_id == 41
  front = replace(front, d_rel=76.5)
  held = matcher.match(
    model_with_lead(100.0, 0.1, 0.0, probability=0.90),
    (front,),
    STRAIGHT_PATH,
    time_s=0.35,
    stationary_points=(front,),
    prefer_primary_stationary=True,
  )

  assert held is not None
  assert held.point.track_id == 41


def test_fresh_corner_velocity_outlier_cannot_seed_stationary_lead() -> None:
  matcher = VisionRadarMatcher()
  match = None
  for index in range(8):
    time_s = index * 0.05
    d_rel = 110.0 - index * 1.5
    point = snapshot_radar_points((Point(
      1005,
      d_rel,
      -0.5,
      v_rel=-28.0,
      source="corner235",
    ),), v_ego=23.0)[0]
    match = matcher.match(
      model_with_lead(
        d_rel - 8.0,
        -1.0,
        18.0,
        probability=0.90,
      ),
      (),
      STRAIGHT_PATH,
      time_s=time_s,
      stationary_points=(point,),
      prefer_corner_stationary=True,
    )

  assert match is None
  assert matcher.stationary_identity is None


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


@pytest.mark.parametrize("enable_radar_tracks", (-1, 0))
def test_scc_mode_uses_vision_when_scc_object_is_missing(
  enable_radar_tracks: int,
) -> None:
  output = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=enable_radar_tracks,
  ).update(
    time_s=1.0,
    v_ego=2.1,
    radar_points=(),
    model=model_with_lead(
      6.8, 0.0, 0.1, probability=0.998,
    ),
  )

  assert output.lead_one is not None
  assert output.lead_one["status"]
  assert not output.lead_one["radar"]
  assert output.lead_one["radarTrackId"] == -1
  assert output.lead_one["dRel"] == pytest.approx(6.8)
  assert output.lead_one["vLead"] == pytest.approx(0.1)


def test_unconditional_scc_mode_uses_scc_without_vision_match() -> None:
  output = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=-1,
  ).update(
    time_s=1.0,
    v_ego=10.0,
    radar_points=(Point(
      0, 80.0, 0.0, v_rel=2.0, source="scc",
    ),),
    model=model_with_lead(
      30.0, 0.0, 12.0, probability=0.99,
    ),
  )

  assert output.lead_one is not None
  assert output.lead_one["radar"]
  assert output.lead_one["radarTrackId"] == 0
  assert output.lead_one["dRel"] == pytest.approx(80.0)
  assert output.lead_one["yRel"] == pytest.approx(0.0)
  assert output.lead_one["dPath"] == pytest.approx(0.0)


def test_stock_scc_mode_uses_vision_while_scc_object_conflicts() -> None:
  output = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=0,
  ).update(
    time_s=1.0,
    v_ego=10.0,
    radar_points=(Point(
      0, 80.0, 0.0, v_rel=2.0, source="scc",
    ),),
    model=model_with_lead(
      30.0, 0.0, 12.0, probability=0.99,
    ),
  )

  assert output.lead_one is not None
  assert not output.lead_one["radar"]
  assert output.lead_one["dRel"] == pytest.approx(30.0)


@pytest.mark.parametrize(
  "enable_radar_tracks,scc_y_rel",
  ((-1, -4.0), (-1, 4.0), (3, -4.0), (3, 4.0)),
)
def test_always_scc_mode_ignores_scc_lateral_position(
  enable_radar_tracks: int,
  scc_y_rel: float,
) -> None:
  output = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=enable_radar_tracks,
  ).update(
    time_s=1.0,
    v_ego=10.0,
    radar_points=(Point(
      0, 30.0, scc_y_rel, v_rel=2.0, source="scc",
    ),),
    model=model_with_lead(
      30.0, -1.0, 12.0, probability=0.99,
    ),
  )

  assert output.lead_one is not None
  assert output.lead_one["radar"]
  assert output.lead_one["radarTrackId"] == 0
  assert output.lead_one["dRel"] == pytest.approx(30.0)
  assert output.lead_one["vLead"] == pytest.approx(12.0)
  assert output.lead_one["yRel"] == pytest.approx(0.0)
  assert output.lead_one["dPath"] == pytest.approx(0.0)


def test_stock_scc_mode_uses_vision_for_lateral_conflict() -> None:
  output = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=0,
  ).update(
    time_s=1.0,
    v_ego=10.0,
    radar_points=(Point(
      0, 30.0, 4.0, v_rel=2.0, source="scc",
    ),),
    model=model_with_lead(
      30.0, -1.0, 12.0, probability=0.99,
    ),
  )

  assert output.lead_one is not None
  assert not output.lead_one["radar"]
  assert output.lead_one["yRel"] == pytest.approx(-1.0)


def test_mode_three_uses_unmatched_scc_before_vision() -> None:
  output = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=3,
  ).update(
    time_s=1.0,
    v_ego=10.0,
    radar_points=(Point(
      0, 80.0, 4.0, v_rel=2.0, source="scc",
    ),),
    model=model_with_lead(
      30.0, -1.0, 12.0, probability=0.99,
    ),
  )

  assert output.lead_one is not None
  assert output.lead_one["radar"]
  assert output.lead_one["radarTrackId"] == 0
  assert output.lead_one["dRel"] == pytest.approx(80.0)
  assert output.lead_one["yRel"] == pytest.approx(0.0)
  assert output.lead_one["dPath"] == pytest.approx(0.0)


def test_mode_three_uses_vision_when_scc_and_front_are_missing() -> None:
  output = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=3,
  ).update(
    time_s=1.0,
    v_ego=10.0,
    radar_points=(),
    model=model_with_lead(
      30.0, 0.0, 12.0, probability=0.99,
    ),
  )

  assert output.lead_one is not None
  assert not output.lead_one["radar"]
  assert output.lead_one["dRel"] == pytest.approx(30.0)


@pytest.mark.parametrize("enable_radar_tracks", (-1, 0, 1, 2, 3))
def test_radar_mode_does_not_create_low_probability_vision_fallback(
  enable_radar_tracks: int,
) -> None:
  output = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=enable_radar_tracks,
  ).update(
    time_s=1.0,
    v_ego=10.0,
    radar_points=(),
    model=model_with_lead(
      30.0, 0.0, 12.0, probability=0.39,
    ),
  )

  assert output.lead_one is None


def test_radar_mode_uses_central_vision_without_raw_corroboration() -> None:
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

  assert near.lead_one is not None
  assert far.lead_one is not None
  assert not near.lead_one["radar"]
  assert not far.lead_one["radar"]


def test_far_high_probability_vision_wins_when_radar_stays_much_farther() -> None:
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
      assert output.lead_one is not None
      assert not output.lead_one["radar"]

  assert output is not None
  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == -1
  assert not output.lead_one["radar"]
  assert output.lead_one["dRel"] == pytest.approx(88.0)


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
  assert output.lead_one is not None
  assert not output.lead_one["radar"]


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


def test_corner_identity_cannot_reacquire_as_primary_after_physical_break() -> None:
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
        78.0 - index * 0.1,
        0.1,
        v_rel=-2.0,
        source="corner235",
      ),),
      model=model_with_lead(
        90.0, 0.0, 20.0, probability=0.0,
      ),
    )

  assert output is not None
  assert output.lead_one is None


def test_radar_only_moving_corner_never_owns_lead_one() -> None:
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
        75.0 - index * 0.1,
        0.1,
        v_rel=-2.0,
        source="corner235",
      ),),
      model=model_with_lead(
        90.0, 0.0, 20.0, probability=0.0,
      ),
    )

  assert output is not None
  assert output.lead_one is None


def test_close_born_corner_only_moving_reflection_cannot_seed_lead() -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
    cut_in_sensitivity=0,
  )
  output = None
  for index in range(14):
    time_s = index * 0.05
    output = controller.update(
      time_s=time_s,
      v_ego=19.3 + index * 0.05,
      radar_points=(Point(
        2333,
        41.2 - 8.2 * time_s,
        -0.95,
        v_rel=-8.2,
        source="corner235",
      ),),
      model=model_with_lead(
        110.0, 0.2, 19.0, probability=0.08,
      ),
    )
    assert output.lead_one is None

  assert output is not None


def test_radar_only_moving_far_corner_rejects_tunnel_fixture() -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
    cut_in_sensitivity=0,
  )
  output = None
  # Reproduce the tunnel-fixture signature: no usable vision, a centered
  # corner-only return near 94 m, and initially plausible range kinematics
  # that contradict the reported vRel after roughly half a second.
  d_rels = (
    96.65, 96.15, 95.05, 94.50, 94.55, 94.25, 93.60,
    93.60, 92.65, 91.45, 90.80, 89.45, 88.35, 89.60,
    89.00, 88.90, 88.60, 88.00, 89.15, 88.65,
  )
  v_rels = (
    -8.85, -8.85, -9.85, -9.85, -11.10, -11.10, -11.10,
    -11.10, -10.75, -11.50, -11.50, -12.90, -12.95, -10.95,
    -10.95, -10.60, -10.60, -10.60, -8.90, -8.80,
  )
  for index, (d_rel, v_rel) in enumerate(zip(d_rels, v_rels, strict=True)):
    time_s = index * 0.05
    output = controller.update(
      time_s=time_s,
      v_ego=29.6,
      radar_points=(Point(
        2809,
        d_rel,
        -0.55,
        v_rel=v_rel,
        source="corner235",
      ),),
      model=model_with_lead(
        118.0, 0.2, 27.5, probability=0.02,
      ),
    )
    assert output.lead_one is None

  assert output is not None


def test_radar_only_moving_far_corner_stays_out_of_lead_one() -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
    cut_in_sensitivity=0,
  )
  output = None
  for index in range(22):
    time_s = index * 0.05
    output = controller.update(
      time_s=time_s,
      v_ego=20.0,
      radar_points=(Point(
        1002,
        95.0 - 2.0 * time_s,
        0.1,
        v_rel=-2.0,
        source="corner235",
      ),),
      model=model_with_lead(
        120.0, 0.0, 20.0, probability=0.0,
      ),
    )
    if time_s < 1.0:
      assert output.lead_one is None

  assert output is not None
  assert output.lead_one is None


def test_radar_only_moving_far_corner_with_front_support_uses_front() -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
    cut_in_sensitivity=0,
  )
  output = None
  for index in range(7):
    time_s = index * 0.05
    d_rel = 95.0 - 2.0 * time_s
    output = controller.update(
      time_s=time_s,
      v_ego=20.0,
      radar_points=(
        Point(
          1002, d_rel, 0.1,
          v_rel=-2.0, source="corner235",
        ),
        Point(
          52, d_rel + 0.2, 0.1,
          v_rel=-2.0, source="frontRadar",
        ),
      ),
      model=model_with_lead(
        100.0, 0.0, 20.0, probability=0.0,
      ),
    )

  assert output is not None
  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == 52


@pytest.mark.parametrize("track_state", (0, 2, 3))
def test_radar_only_moving_front_accepts_unknown_or_confirmed_state(
  track_state: int,
) -> None:
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
        52,
        40.0 - index * 0.1,
        0.1,
        v_rel=-2.0,
        source="frontRadar",
        trackState=track_state,
      ),),
      model=model_with_lead(
        90.0, 0.0, 20.0, probability=0.0,
      ),
    )

  assert output is not None
  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == 52


def test_radar_only_moving_front_requires_longer_tentative_confirmation() -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
    cut_in_sensitivity=0,
  )
  for index in range(12):
    output = controller.update(
      time_s=index * 0.05,
      v_ego=16.0,
      radar_points=(Point(
        39,
        20.0 - index * 0.45,
        -0.1,
        v_rel=-9.0,
        source="frontRadar",
        trackState=1,
      ),),
      model=model_with_lead(
        115.0, -0.3, 19.0, probability=0.02,
      ),
    )
    assert output.lead_one is None

  for index in range(12, 18):
    output = controller.update(
      time_s=index * 0.05,
      v_ego=16.0,
      radar_points=(Point(
        39,
        20.0 - index * 0.45,
        -0.1,
        v_rel=-9.0,
        source="frontRadar",
        trackState=1,
      ),),
      model=model_with_lead(
        115.0, -0.3, 19.0, probability=0.02,
      ),
    )

  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == 39


def test_tentative_native_track_remains_available_to_vision_match() -> None:
  output = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=1,
    cut_in_sensitivity=0,
  ).update(
    time_s=1.0,
    v_ego=16.0,
    radar_points=(Point(
      39,
      20.0,
      -0.1,
      v_rel=-9.0,
      source="frontRadar",
      trackState=1,
    ),),
    model=model_with_lead(
      20.0, -0.1, 7.0, probability=0.95,
    ),
  )

  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == 39


def test_vision_match_miss_uses_vision_not_unmatched_raw_corner() -> None:
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
  assert not output.lead_one["radar"]
  assert output.lead_one["radarTrackId"] == -1


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


def test_stationary_front_speed_contradiction_falls_back_to_vision() -> None:
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
  assert output.lead_one is not None
  assert not output.lead_one["radar"]


def test_unconfirmed_stationary_front_uses_central_vision_fallback() -> None:
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

  assert output.lead_one is not None
  assert not output.lead_one["radar"]


def test_radar_mode_keeps_vision_only_side_cutin_out_of_lead_one() -> None:
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
  assert output.lead_one is None
  held = controller.update(
    time_s=0.35,
    v_ego=10.0,
    radar_points=(),
    model=model_with_lead(
      30.0, 1.45, 9.0, probability=0.36,
    ),
  )
  assert held.lead_one is None


def test_near_vision_fallback_ignores_adjacent_raw_reflection() -> None:
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

  assert output.lead_one is not None
  assert not output.lead_one["radar"]


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


def test_mode_two_uses_vision_when_only_fast_scc_matches_nearer_vision() -> None:
  model = model_with_lead(
    14.7, 0.0, 5.04, probability=1.0,
  )
  model.leadsV3[0].xStd = (6.0,)
  output = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=2,
  ).update(
    time_s=39.6,
    v_ego=3.4,
    radar_points=(
      Point(
        52, 28.6, 0.2,
        v_rel=1.55, source="frontRadar",
      ),
      Point(
        0, 14.3, -0.2,
        v_rel=1.64, source="scc",
      ),
    ),
    model=model,
  )

  assert output.lead_one is not None
  assert not output.lead_one["radar"]
  assert output.lead_one["radarTrackId"] == -1
  assert output.lead_one["dRel"] == pytest.approx(14.7)


def test_controller_uses_vision_when_radar_distance_error_is_large() -> None:
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

  assert output.lead_one is not None
  assert not output.lead_one["radar"]


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


def test_corner_support_strengthens_front_without_owning_lead_one() -> None:
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
  assert output.lead_one["radarTrackId"] == 35

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
  assert corner_handoff.lead_one["radarTrackId"] == 35
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

  assert released.lead_one is not None
  assert not released.lead_one["radar"]


def test_controller_stationary_mismatch_uses_configured_primary_only() -> None:
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
      assert output.lead_one is not None
      assert not output.lead_one["radar"]
      assert output.lead_one["radarTrackId"] == -1
      continue
    assert output.lead_one is not None
    assert output.lead_one["radar"]
    assert output.lead_one["radarTrackId"] == expected_track_id
    selected = next(
      point for point in points
      if point.track_id == expected_track_id
    )
    assert selected.source == expected_source


def test_distant_corner_without_correspondence_uses_vision() -> None:
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
  assert output.lead_one is not None
  assert not output.lead_one["radar"]


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
      None,
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
      if index < 5 and enable_radar_tracks != -1:
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
      assert output.lead_one is not None
      assert not output.lead_one["radar"]

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


def test_tentative_closer_moving_radar_requires_longer_confirmation() -> None:
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

  for index in range(7, 22):
    time_s = index * 0.05
    far_d_rel = 35.0 + v_rel * time_s
    output = controller.update(
      time_s=time_s,
      v_ego=v_ego,
      radar_points=(
        Point(
          35, far_d_rel - 9.0, 0.0,
          v_rel=v_rel, source="frontRadar", trackState=1,
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
    assert output.lead_one["radarTrackId"] == 43

  time_s = 22 * 0.05
  far_d_rel = 35.0 + v_rel * time_s
  output = controller.update(
    time_s=time_s,
    v_ego=v_ego,
    radar_points=(
      Point(
        35, far_d_rel - 9.0, 0.0,
        v_rel=v_rel, source="frontRadar", trackState=1,
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
  assert output.lead_one["radarTrackId"] == 35
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


def test_moving_corner_born_in_path_does_not_become_control_lead() -> None:
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
    assert output.lead_one is None
    assert output.lead_two is None

  assert output is not None


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


def test_controller_accepts_radar_jitter_and_falls_back_on_stale_radar() -> None:
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
  assert rejected.lead_one is not None
  assert not rejected.lead_one["radar"]


def test_controller_latches_requested_motion_sensor() -> None:
  corner = DPathRadarController(prefer_corner_radar=True)
  front = DPathRadarController(prefer_corner_radar=False)

  assert corner.motion_sensor == "corner"
  assert front.motion_sensor == "front"

  front.update(
    time_s=0.0,
    v_ego=10.0,
    radar_points=(Point(1005, 20.0, 3.0, source="corner235"),),
    model=model_with_lead(30.0, 0.0, 10.0, probability=0.0),
  )
  assert front.motion_sensor == "corner"


def test_controller_disables_new_lead_two_at_zero_sensitivity() -> None:
  controller = DPathRadarController(
    prefer_corner_radar=True,
    cut_in_sensitivity=0,
  )
  enabled_controller = DPathRadarController(prefer_corner_radar=True)
  output = None
  enabled_output = None
  for index in range(30):
    target_d_rel = 13.0 - 0.12 * index
    target_y_rel = -2.9 + 0.04 * index
    output = controller.update(
      time_s=index * 0.1,
      v_ego=10.0,
      radar_points=(
        Point(10, 30.0, 0.0, source="frontRadar"),
        Point(
          49, target_d_rel, target_y_rel,
          v_rel=-7.8, v_lead=2.2,
        ),
        Point(
          1005, target_d_rel - 0.25, target_y_rel - 0.15,
          v_rel=-7.7, v_lead=2.3, yv_rel=0.4,
          source="corner235", trackState=2,
        ),
      ),
      model=model_with_lead(30.0, 0.0, 10.0),
    )
    enabled_output = enabled_controller.update(
      time_s=index * 0.1,
      v_ego=10.0,
      radar_points=(
        Point(10, 30.0, 0.0, source="frontRadar"),
        Point(
          49, target_d_rel, target_y_rel,
          v_rel=-7.8, v_lead=2.2,
        ),
        Point(
          1005, target_d_rel - 0.25, target_y_rel - 0.15,
          v_rel=-7.7, v_lead=2.3, yv_rel=0.4,
          source="corner235", trackState=2,
        ),
      ),
      model=model_with_lead(30.0, 0.0, 10.0),
    )

  assert output is not None
  assert enabled_output is not None
  assert enabled_output.leads_cutin
  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == 10
  assert output.lead_two is None
  assert output.leads_cutin == ()


def test_controller_uses_fixed_lead_dynamics_and_raw_jerk() -> None:
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
  )

  assert quiet.lead_one is not None
  assert quiet.lead_one["aLeadTau"] == pytest.approx(1.5)
  assert quiet.lead_one["jLead"] == pytest.approx(0.0)


def test_corner_lead_two_uses_matched_front_dynamics() -> None:
  controller = DPathRadarController(prefer_corner_radar=True)
  selected: list[dict] = []
  first_selected_index = None
  for index in range(40):
    time_s = index * 0.05
    target_d_rel = 7.0 - 0.05 * index
    target_y_rel = -2.60 + 0.04 * index
    corner_track_id = 1019 if index % 2 == 0 else 1020
    output = controller.update(
      time_s=time_s,
      v_ego=10.0,
      radar_points=(
        Point(10, 25.0, 0.0, v_rel=-4.0, v_lead=6.0),
        Point(
          35, target_d_rel, target_y_rel, v_rel=-1.0, v_lead=9.0,
          a_lead=1.0, j_lead=0.75,
        ),
        Point(
          corner_track_id, target_d_rel - 0.1, target_y_rel - 0.1,
          v_rel=-1.0, v_lead=9.0, yv_rel=0.8,
          source="corner235", a_lead=0.0, j_lead=0.0, trackState=2,
        ),
      ),
      model=model_with_lead(25.0, 0.0, 6.0),
    )
    if output.lead_two is not None:
      if first_selected_index is None:
        first_selected_index = index
      selected.append(output.lead_two)
    elif first_selected_index is not None:
      pytest.fail("leadTwo dropped when the corner slot changed")

  assert first_selected_index is not None
  assert {lead["radarTrackId"] for lead in selected} == {1019, 1020}
  assert all(lead["aLead"] == pytest.approx(1.0) for lead in selected)
  assert all(lead["jLead"] == pytest.approx(0.75) for lead in selected)


def test_corner_lateral_jitter_does_not_remove_primary_or_add_control_lead() -> None:
  controller = DPathRadarController(prefer_corner_radar=True)
  output = None
  for index in range(40):
    jitter = 0.30 if index % 2 == 0 else -0.30
    output = controller.update(
      time_s=index * 0.05,
      v_ego=10.0,
      radar_points=(
        Point(10, 30.0, 0.0, v_rel=2.0, v_lead=12.0),
        Point(
          1005, 20.0, 2.8 + jitter, v_rel=0.0, v_lead=10.0,
          yv_rel=(-12.0 if index % 2 == 0 else 12.0),
          source="corner235", trackState=2,
        ),
      ),
      model=model_with_lead(30.0, 0.0, 12.0),
    )

  assert output is not None
  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == 10
  assert output.lead_two is None
  assert output.lead_cutin_risk is None
  assert output.leads_cutin == ()
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
  assert [point.track_id for point in select_primary_radar_points(points, -1)] == [0, 1]
  assert [point.track_id for point in select_primary_radar_points(points, 0)] == [0, 1]
  assert [point.track_id for point in select_primary_radar_points(points, 1)] == [10]
  assert [point.track_id for point in select_primary_radar_points(points, 2)] == [10, 0]
  assert [point.track_id for point in select_primary_radar_points(points, 3)] == [10, 0, 1]


def test_live_radar_snapshot_matches_generic_capnp_adapter() -> None:
  radar_data = car.RadarData.new_message()
  capnp_points = radar_data.init("points", 6)
  specs = (
    (52, "frontRadar"),
    (0, "scc"),
    (1005, "corner235"),
    (1241, "corner180"),
    (1301, "corner430"),
    (205, "frontRadar"),
  )
  for index, (track_id, source) in enumerate(specs):
    point = capnp_points[index]
    point.trackId = track_id
    point.radarSource = source
    point.dRel = 10.0 + index
    point.yRel = -3.0 + index
    point.vRel = -2.0 + index * 0.25
    point.aRel = -0.5 + index * 0.1
    point.yvRel = 0.2 - index * 0.05
    point.vLead = 8.0
    point.aLead = -0.4 + index * 0.05
    point.jLead = -1.0 + index * 0.2
    point.measured = index != 4
    point.trackState = index % 3

  generic = snapshot_radar_points(
    capnp_points, v_ego=12.5, time_delta_s=0.075,
  )
  production = snapshot_live_radar_points(
    capnp_points, v_ego=12.5, time_delta_s=0.075,
  )

  assert production == generic
  assert [point.source for point in production] == [
    "frontRadar", "scc", "corner235", "corner180", "frontRadar",
  ]

  generic_controller = DPathRadarController(
    front_radar_measurement_delay_s=0.02,
    corner_radar_measurement_delay_s=0.05,
  )
  production_controller = DPathRadarController(
    front_radar_measurement_delay_s=0.02,
    corner_radar_measurement_delay_s=0.05,
    production_live_tracks=True,
  )
  assert production_controller._points_at_model_time(
    capnp_points, 12.5, 0.015,
  ) == generic_controller._points_at_model_time(
    capnp_points, 12.5, 0.015,
  )


def test_high_front_track_id_remains_primary_on_non_hyundai_radar() -> None:
  radar_data = car.RadarData.new_message()
  point = radar_data.init("points", 1)[0]
  point.trackId = 380
  point.radarSource = "frontRadar"
  point.dRel = 35.0
  point.yRel = 0.1
  point.vRel = -6.0
  point.measured = True

  controller = DPathRadarController(
    enable_radar_tracks=1,
    production_live_tracks=True,
  )
  output = controller.update(
    time_s=1.0,
    v_ego=28.0,
    radar_points=radar_data.points,
    model=model_with_lead(35.0, 0.1, 22.0),
  )

  assert controller.motion_sensor == "front"
  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == 380


def test_front_radar_measurement_delay_projects_fresh_points() -> None:
  controller = DPathRadarController(
    front_radar_measurement_delay_s=0.8,
  )
  points = controller._points_at_model_time(
    (Point(10, 30.0, 0.0, v_rel=-2.0),),
    v_ego=20.0,
    radar_to_model_time_s=-0.05,
  )

  assert len(points) == 1
  assert points[0].d_rel == pytest.approx(28.5)


def test_stale_radar_publication_is_rejected_before_delay_projection() -> None:
  controller = DPathRadarController(
    front_radar_measurement_delay_s=0.8,
  )

  assert controller._points_at_model_time(
    (Point(10, 30.0, 0.0, v_rel=-2.0),),
    v_ego=20.0,
    radar_to_model_time_s=-0.8,
  ) == ()


def test_mode_three_uses_scc_at_any_speed_when_front_omits_lead() -> None:
  output = DPathRadarController(
    enable_radar_tracks=3,
  ).update(
    time_s=16.0,
    v_ego=9.2,
    radar_points=(
      Point(
        52, 11.3, -3.39,
        v_rel=-5.16, source="frontRadar",
      ),
      Point(
        0, 6.6, 0.0,
        v_rel=1.5, source="scc",
      ),
    ),
    model=model_with_lead(
      6.86, -0.2, 10.55, probability=1.0,
    ),
  )

  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == 0
  assert output.lead_one["dRel"] == pytest.approx(6.6)


def test_mode_two_does_not_use_uncorroborated_fast_scc_radar_only() -> None:
  controller = DPathRadarController(enable_radar_tracks=2)
  output = None
  for index in range(20):
    output = controller.update(
      time_s=index * 0.05,
      v_ego=9.2,
      radar_points=(
        Point(0, 30.0, 0.0, v_rel=1.5, source="scc"),
      ),
      model=model_with_lead(
        80.0, 0.0, 20.0, probability=0.0,
      ),
    )

  assert output is not None
  assert output.lead_one is None


def test_option_two_publishes_corroborated_low_speed_scc_as_lead_two() -> None:
  model = SimpleNamespace(
    position=SimpleNamespace(x=(0.0, 120.0), y=(0.0, -9.6)),
    leadsV3=(SimpleNamespace(
      prob=0.20,
      x=(111.52,),
      y=(-8.0,),
      v=(4.5,),
      xStd=(8.0,),
      yStd=(1.5,),
      vStd=(2.0,),
    ),),
  )
  points = (
    Point(0, 105.0, 0.0, v_rel=-15.5, source="scc"),
    Point(45, 103.5, 8.0, v_rel=-15.6, source="frontRadar"),
    Point(1618, 104.0, 7.5, v_rel=-14.0, source="corner235"),
  )
  controller = DPathRadarController(
    prefer_corner_radar=True,
    enable_radar_tracks=2,
  )
  output = None
  for index in range(5):
    output = controller.update(
      time_s=index * 0.05,
      v_ego=20.0,
      radar_points=points,
      model=model,
    )

    if index < 15:
      assert output.lead_one is None

  assert output is not None
  assert output.lead_one is None
  assert output.lead_two is not None
  assert output.lead_two["radarTrackId"] == 45
  assert output.lead_two["dRel"] == pytest.approx(103.5)


def test_option_one_does_not_publish_low_speed_scc_lead_two() -> None:
  controller = DPathRadarController(enable_radar_tracks=1)
  output = None
  for index in range(5):
    output = controller.update(
      time_s=index * 0.05,
      v_ego=20.0,
      radar_points=(
        Point(0, 105.0, 0.0, v_rel=-15.5, source="scc"),
      ),
      model=model_with_lead(
        111.0, 0.0, 4.5, probability=0.20,
      ),
    )

  assert output is not None
  assert output.lead_two is None


def test_option_two_suppresses_scc_lead_two_duplicate_of_primary() -> None:
  controller = DPathRadarController(enable_radar_tracks=2)
  output = None
  points = (
    Point(0, 51.0, 0.0, v_rel=-15.5, source="scc"),
    Point(45, 50.0, 0.1, v_rel=-15.4, source="frontRadar"),
  )
  for index in range(5):
    output = controller.update(
      time_s=index * 0.05,
      v_ego=20.0,
      radar_points=points,
      model=model_with_lead(
        50.0, 0.1, 4.6, probability=0.90,
      ),
    )

  assert output is not None
  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == 45
  assert output.lead_two is None


def test_independent_controller_calculates_lead_one_before_motion_lead_two() -> None:
  controller = DPathRadarController(prefer_corner_radar=True)
  output = None
  for index in range(32):
    target_d_rel = 12.0 - 0.10 * index
    target_y_rel = -3.0 + 0.05 * index
    output = controller.update(
      time_s=index * 0.05,
      v_ego=10.0,
      radar_points=(
        Point(10, 30.0, 0.1, v_rel=2.0, v_lead=12.0),
        Point(
          49, target_d_rel, target_y_rel,
          v_rel=-2.0, v_lead=8.0,
        ),
        Point(
          1005, target_d_rel - 0.1, target_y_rel - 0.1,
          v_rel=-2.0, v_lead=8.0, yv_rel=1.0,
          source="corner235", trackState=2,
        ),
      ),
      model=model_with_lead(30.0, 0.1, 12.0),
    )

  assert output is not None
  assert output.lead_one is not None
  assert output.lead_one["radarTrackId"] == 10
  assert output.lead_two is not None
  assert output.lead_two["radarTrackId"] == 1005
  assert output.lead_two["dRel"] < output.lead_one["dRel"]


def test_independent_controller_retains_confirmed_lead_two_until_path_exit() -> None:
  controller = DPathRadarController(prefer_corner_radar=True)
  detected = None
  for index in range(32):
    target_d_rel = 12.0 - 0.10 * index
    target_y_rel = -3.0 + 0.05 * index
    detected = controller.update(
      time_s=index * 0.05,
      v_ego=10.0,
      radar_points=(
        Point(49, target_d_rel, target_y_rel, v_rel=-2.0, v_lead=8.0),
        Point(
          1005, target_d_rel - 0.1, target_y_rel - 0.1,
          v_rel=-2.0, v_lead=8.0, yv_rel=1.0,
          source="corner235", trackState=2,
        ),
      ),
      model=model_with_lead(30.0, 0.0, 10.0, probability=0.0),
    )

  assert detected is not None
  assert detected.lead_two is not None

  retained = None
  for offset in range(1, 7):
    retained = controller.update(
      time_s=1.55 + 0.05 * offset,
      v_ego=10.0,
      radar_points=(
        Point(49, 8.8, -1.45, v_rel=0.0, v_lead=10.0),
        Point(
          1005, 8.7, -1.50, v_rel=0.0, v_lead=10.0,
          source="corner235", trackState=2,
        ),
      ),
      model=model_with_lead(30.0, 0.0, 10.0, probability=0.0),
    )

  assert retained is not None
  assert retained.lead_two is not None
  assert retained.lead_two["radarTrackId"] == 1005

  exited = retained
  for offset in range(1, 31):
    target_y_rel = -1.50 - 0.12 * offset
    exited = controller.update(
      time_s=1.85 + 0.05 * offset,
      v_ego=10.0,
      radar_points=(
        Point(49, 8.8, target_y_rel, v_rel=0.0, v_lead=10.0),
        Point(
          1005, 8.7, target_y_rel - 0.05, v_rel=0.0, v_lead=10.0,
          yv_rel=-2.4, source="corner235", trackState=2,
        ),
      ),
      model=model_with_lead(30.0, 0.0, 10.0, probability=0.0),
    )
  assert exited.lead_two is None


def test_confirmed_cutin_falls_back_to_lead_two_when_lead_one_disappears() -> None:
  controller = DPathRadarController(prefer_corner_radar=True)
  matched = None
  for index in range(20):
    target_y_rel = -2.50 + 0.05 * index
    points = (
      Point(49, 7.0, target_y_rel, v_rel=0.0, v_lead=10.0),
      Point(
        1005, 6.9, target_y_rel - 0.05, v_rel=0.0, v_lead=10.0,
        yv_rel=1.0, source="corner235", trackState=2,
      ),
    )
    matched = controller.update(
      time_s=index * 0.05,
      v_ego=10.0,
      radar_points=points,
      model=model_with_lead(7.0, target_y_rel, 10.0),
    )

  assert matched is not None
  held = matched
  for offset in range(1, 5):
    target_y_rel = -1.50 + 0.03 * offset
    points = (
      Point(49, 7.0, target_y_rel, v_rel=0.0, v_lead=10.0),
      Point(
        1005, 6.9, target_y_rel - 0.05, v_rel=0.0, v_lead=10.0,
        yv_rel=0.6, source="corner235", trackState=2,
      ),
    )
    held = controller.update(
      time_s=0.95 + 0.05 * offset,
      v_ego=10.0,
      radar_points=points,
      model=model_with_lead(
        7.0, target_y_rel, 10.0, probability=0.0,
      ),
    )

  assert matched.lead_one is not None
  assert matched.lead_two is None
  assert held.lead_one is None
  assert held.lead_two is not None
  assert held.lead_two["radarTrackId"] == 1005


def test_production_radar_is_fixed_to_carrot() -> None:
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
  dpath_source = dpath_radard.read_text(encoding="utf-8")
  validation_source = validation_replay.read_text(encoding="utf-8")
  manager_source = process_config.read_text(encoding="utf-8")

  assert not radard.exists()
  assert "from openpilot.selfdrive.controls.radard" not in dpath_source
  assert 'getattr(sm["modelV2"], "timestampEof", 0)' in dpath_source
  assert "CarrotRadarCutInSensitivity" not in dpath_source
  assert "PRODUCTION_CUT_IN_SENSITIVITY = 3" in dpath_source
  assert "cut_in_sensitivity=PRODUCTION_CUT_IN_SENSITIVITY" in dpath_source
  assert "production_live_tracks=True" in dpath_source
  assert "RadarReactionFactor" not in dpath_source
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
  assert '"radard", "openpilot.selfdrive.carrot.radar.radard_dpath", only_onroad' in manager_source
  assert 'PythonProcess("radard_dpath"' not in manager_source
  assert "CarrotRadarMode" not in manager_source
