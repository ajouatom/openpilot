from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.carrot.radar_motion.predictor import (
  IMMEDIATE_LANE_SCOPE_HALF_WIDTH_M,
  RadarMotionDecisionTracker,
  RadarMotionPredictor,
  cutin_probability_at,
  model_path_point_at_s,
  model_path_y,
  prediction_sample_at,
  project_to_model_path,
  visible_motion_points,
)
from openpilot.selfdrive.carrot.radar_motion.lead_selection import (
  dpath_control_max_d_rel,
  select_dpath_lead_two,
)
from openpilot.selfdrive.carrot.radar_motion.controller import (
  DPathRadarController,
)
from openpilot.selfdrive.carrot.radar_motion.predictor import RadarMotionCutIn
from openpilot.selfdrive.carrot.radar_motion.primary import (
  VisionRadarMatcher,
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


class FixedDecisionTracker:
  def __init__(self, prediction: SimpleNamespace) -> None:
    self.prediction = prediction

  def update(self, *args, **kwargs):
    return SimpleNamespace(
      confirmed=(RadarMotionCutIn(self.prediction, 0.8),),
    )


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

  assert {point.track_id for point in visible} == {1, 2, 4, 6}


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
          -3.0,
          source="corner235",
          v_lead=10.0,
        ),
        Point(
          2,
          25.0,
          -3.0 + index * 0.1,
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
  assert prediction.path_entry_probability > 0.5


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
    (Point(10, 30.0, 2.0, v_lead=0.5),),
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
  assert confirmed_at >= 0.8
  assert decision.confirmed[0].prediction.track_id == 10


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

  assert dpath_control_max_d_rel(20.0) == pytest.approx(50.0)
  assert [lead["radarTrackId"] for lead in selection.cutins] == [20]
  assert selection.lead_two["radarTrackId"] == 20


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
    d_path=1.0,
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


def test_recent_primary_is_not_republished_as_motion_lead_two() -> None:
  controller = DPathRadarController(prefer_corner_radar=True)
  prediction = SimpleNamespace(
    source="corner235",
    track_id=1005,
    d_path=1.0,
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
  assert held.lead_two is None


def test_production_dpath_mode_is_independent_of_conventional_radard() -> None:
  radard = Path(__file__).resolve().parents[2] / "controls" / "radard.py"
  dpath_radard = (
    Path(__file__).resolve().parents[1] / "radar" / "radard_dpath.py"
  )
  process_config = (
    Path(__file__).resolve().parents[3]
    / "system"
    / "manager"
    / "process_config.py"
  )
  conventional_source = radard.read_text(encoding="utf-8")
  dpath_source = dpath_radard.read_text(encoding="utf-8")
  manager_source = process_config.read_text(encoding="utf-8")

  assert "RadarLeadModelMode" not in conventional_source
  assert "RadarMotionMode" not in conventional_source
  assert "RadarMotionPredictor" not in conventional_source
  assert "from openpilot.selfdrive.controls.radard" not in dpath_source
  assert '"radard", "openpilot.selfdrive.controls.radard", conventional_radard' in manager_source
  assert '"radard_dpath", "openpilot.selfdrive.carrot.radar.radard_dpath", dpath_radard' in manager_source
