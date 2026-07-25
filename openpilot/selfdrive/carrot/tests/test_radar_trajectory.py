from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.carrot.radar.radar_trajectory import (
  RadarTrajectoryAnalyzer,
  TRAJECTORY_MODEL_FEATURE_NAMES,
  estimated_yaw_rate_rad_s,
  trajectory_entry_probability,
  trajectory_feature_row,
  trajectory_is_review_candidate,
  trajectory_model_feature_row,
)
from openpilot.selfdrive.carrot.radar.radar_trajectory_model import (
  DEFAULT_CORNER_MODEL_PATH,
  DEFAULT_FRONT_MODEL_PATH,
  RadarTrajectoryModel,
  RadarTrajectoryDecisionFilter,
  TrajectoryCutinDecision,
  TrajectoryCutinPrediction,
  trajectory_decision_ahead_of_primary,
)


PATH = ((0.0, 0.0), (100.0, 0.0))
LANE_LINES = (
  ((0.0, 3.6), (100.0, 3.6)),
  ((0.0, 1.8), (100.0, 1.8)),
  ((0.0, -1.8), (100.0, -1.8)),
  ((0.0, -3.6), (100.0, -3.6)),
)
LANE_PROBS = (0.8, 0.95, 0.95, 0.8)


def point(
  track_id: int,
  d_rel: float,
  y_rel: float,
  source: str = "corner235",
  v_rel: float = 0.0,
  yv_rel: float = 0.0,
  measured: bool = True,
) -> SimpleNamespace:
  return SimpleNamespace(
    track_id=track_id,
    d_rel=d_rel,
    y_rel=y_rel,
    source=source,
    v_rel=v_rel,
    yv_rel=yv_rel,
    measured=measured,
  )


def update_sequence(
  y_values: tuple[float, ...],
  source: str = "corner235",
  path_y_stds: tuple[tuple[float, float], ...] = ((0.0, 0.15), (100.0, 0.15)),
  steering_angle_deg: float = 0.0,
  steering_rate_deg_s: float = 0.0,
  yaw_rate_rad_s: float = 0.0,
  yaw_rate_estimated: bool = False,
):
  analyzer = RadarTrajectoryAnalyzer()
  result = None
  for index, y_rel in enumerate(y_values):
    result = analyzer.update(
      index * 0.25,
      (point(10, 25.0, y_rel, source),),
      PATH,
      LANE_LINES,
      LANE_PROBS,
      path_y_stds,
      (0.2, 0.2, 0.2, 0.2),
      steering_angle_deg,
      steering_rate_deg_s,
      yaw_rate_rad_s,
      yaw_rate_estimated,
    )
  assert result is not None
  return result[(source, 10)]


def test_inward_history_predicts_lane_entry() -> None:
  trajectory = update_sequence((4.5, 4.0, 3.5, 3.0))

  assert trajectory.d_path_rate < -1.0
  assert trajectory.time_to_entry_s is not None
  assert trajectory.time_to_entry_s < 0.5
  assert trajectory_entry_probability(trajectory, 1.0) > 0.60
  assert trajectory_is_review_candidate(trajectory, 25.0, 1.0)


def test_outward_history_is_not_review_candidate() -> None:
  trajectory = update_sequence((3.0, 3.4, 3.8, 4.2))

  assert trajectory.d_path_rate > 0.0
  assert trajectory.reason == "not inward"
  assert not trajectory_is_review_candidate(trajectory, 25.0, 2.0)


def test_front_and_corner_tracks_keep_independent_history() -> None:
  analyzer = RadarTrajectoryAnalyzer()
  output = {}
  for index in range(4):
    output = analyzer.update(
      index * 0.25,
      (
        point(10, 25.0, 4.0 - index * 0.4, "frontRadar"),
        point(10, 25.0, -4.0 + index * 0.2, "corner235"),
      ),
      PATH,
      LANE_LINES,
      LANE_PROBS,
    )

  assert output[("frontRadar", 10)].d_path_rate < -1.0
  assert output[("corner235", 10)].d_path_rate > 0.0


def test_path_uncertainty_widens_future_distribution() -> None:
  precise = update_sequence(
    (4.5, 4.0, 3.5, 3.0),
    path_y_stds=((0.0, 0.1), (100.0, 0.1)),
  )
  uncertain = update_sequence(
    (4.5, 4.0, 3.5, 3.0),
    path_y_stds=((0.0, 1.5), (100.0, 1.5)),
  )

  assert uncertain.path_sigma > precise.path_sigma
  assert uncertain.samples[-1].lateral_sigma > precise.samples[-1].lateral_sigma


def test_low_lane_probability_uses_position_path_fallback() -> None:
  analyzer = RadarTrajectoryAnalyzer()
  result = analyzer.update(
    0.0,
    (point(10, 25.0, 4.0),),
    PATH,
    LANE_LINES,
    (0.8, 0.1, 0.1, 0.8),
  )
  trajectory = result[("corner235", 10)]

  assert not trajectory.lane_reliable
  assert trajectory.lane_probability == 0.1
  assert trajectory.reason == "lane-low / path fallback"


def test_model_feature_row_contains_only_current_and_past_observations() -> None:
  trajectory = update_sequence(
    (4.5, 4.0, 3.5, 3.0),
    steering_angle_deg=18.0,
    steering_rate_deg_s=25.0,
    yaw_rate_rad_s=0.08,
    yaw_rate_estimated=True,
  )
  row = trajectory_feature_row(trajectory)
  model_row = trajectory_model_feature_row(
    trajectory,
    point(10, 25.0, 3.0),
    20.0,
  )

  assert row["history_valid_0p00"] == 1.0
  assert row["history_d_path_0p50"] > row["history_d_path_0p00"]
  assert not any(name.startswith("future_") for name in model_row)
  assert tuple(model_row) == TRAJECTORY_MODEL_FEATURE_NAMES
  assert row["steering_angle_deg"] == 18.0
  assert row["steering_rate_deg_s"] == 25.0
  assert row["yaw_rate_rad_s"] == 0.08
  assert row["yaw_rate_estimated"] == 1.0
  assert row["ego_rotation_lateral_speed"] == 2.0
  assert 0.0 < row["turn_ambiguity"] < 1.0


def test_unmeasured_front_slots_are_not_input_history() -> None:
  analyzer = RadarTrajectoryAnalyzer()
  empty = analyzer.update(
    0.0,
    (point(10, 0.0, 0.0, "frontRadar", measured=False),),
    PATH,
    LANE_LINES,
    LANE_PROBS,
  )
  measured = analyzer.update(
    0.1,
    (point(10, 25.0, 3.0, "frontRadar"),),
    PATH,
    LANE_LINES,
    LANE_PROBS,
  )

  assert empty == {}
  assert measured[("frontRadar", 10)].history_count == 1


def test_reused_track_id_starts_new_physical_episode() -> None:
  analyzer = RadarTrajectoryAnalyzer()
  first = analyzer.update(
    0.0, (point(10, 25.0, 3.0),), PATH, LANE_LINES, LANE_PROBS,
  )[("corner235", 10)]
  reused = analyzer.update(
    0.1, (point(10, 70.0, -5.0),), PATH, LANE_LINES, LANE_PROBS,
  )[("corner235", 10)]

  assert reused.continuity_id == first.continuity_id + 1
  assert reused.history_count == 1


def test_short_dropout_keeps_history_only_when_motion_is_continuous() -> None:
  analyzer = RadarTrajectoryAnalyzer()
  first = analyzer.update(
    0.0, (point(10, 25.0, 3.0, v_rel=-2.0),), PATH, LANE_LINES, LANE_PROBS,
  )[("corner235", 10)]
  analyzer.update(0.1, (), PATH, LANE_LINES, LANE_PROBS)
  resumed = analyzer.update(
    0.2, (point(10, 24.6, 2.9, v_rel=-2.0),), PATH, LANE_LINES, LANE_PROBS,
  )[("corner235", 10)]

  assert resumed.continuity_id == first.continuity_id
  assert resumed.history_count == 2


def test_steering_yaw_fallback_uses_vehicle_geometry() -> None:
  yaw_rate = estimated_yaw_rate_rad_s(
    v_ego=25.0,
    steering_angle_deg=14.0,
    steer_ratio=14.0,
    wheelbase=2.8,
  )

  assert 0.14 < yaw_rate < 0.17


def test_probability_filter_has_only_direct_threshold() -> None:
  trajectory = update_sequence((4.5, 4.0, 3.5, 3.0))
  high = TrajectoryCutinPrediction(
    10, "corner235", 0.71, (0.4, 0.71, 0.6, 0.5), trajectory, point(10, 25.0, 3.0),
  )
  low = TrajectoryCutinPrediction(
    11, "corner235", 0.60, (0.6, 0.4, 0.3, 0.2), trajectory, point(11, 25.0, 3.0),
  )

  decision = RadarTrajectoryDecisionFilter(0.70).update(0.0, (high, low))

  assert decision.confirmed == (high,)
  assert decision.tentative == ()


def test_probability_filter_reports_path_exit_separately_from_cutin() -> None:
  trajectory = update_sequence((0.2, 0.3, 0.5, 0.8))
  exiting = TrajectoryCutinPrediction(
    10, "frontRadar", 0.0, (0.9, 0.5, 0.2, 0.05),
    trajectory, point(10, 25.0, 0.8, "frontRadar"),
    path_exit_probability=0.95,
    current_path_occupancy=True,
  )

  decision = RadarTrajectoryDecisionFilter(0.90, 0.90).update(0.0, (exiting,))

  assert decision.confirmed == ()
  assert decision.exiting == (exiting,)


def test_model_entry_probability_ignores_horizons_after_vehicle_passes_ego() -> None:
  analyzer = RadarTrajectoryAnalyzer()
  trajectory = None
  current_point = None
  for index, (d_rel, y_rel) in enumerate(((3.03, 3.40), (2.69, 3.30), (2.35, 3.20))):
    current_point = point(
      58, d_rel, y_rel, "frontRadar", v_rel=-1.36, yv_rel=-0.08,
    )
    trajectory = analyzer.update(
      index * 0.25,
      (current_point,),
      PATH,
      LANE_LINES,
      LANE_PROBS,
    )[("frontRadar", 58)]
  assert trajectory is not None
  assert current_point is not None

  model = object.__new__(RadarTrajectoryModel)
  model.source = "front"
  model.probabilities = lambda _matrix: np.asarray(
    ((0.80, 0.88, 0.93, 0.95),), dtype=np.float32,
  )
  prediction = model.predict(
    {("frontRadar", 58): trajectory},
    (current_point,),
    v_ego=9.1,
  )[0]

  assert prediction.horizon_probabilities == pytest.approx((0.80, 0.88, 0.93, 0.95))
  assert prediction.forward_horizon_relevant == (True, True, False, False)
  assert prediction.probability == pytest.approx(0.88)


def test_model_entry_probability_is_zero_when_all_horizons_are_behind_ego() -> None:
  analyzer = RadarTrajectoryAnalyzer()
  trajectory = None
  current_point = None
  for index, d_rel in enumerate((3.0, 2.0, 1.0)):
    current_point = point(
      1167, d_rel, -7.3, "corner235", v_rel=-8.75, yv_rel=2.86,
    )
    trajectory = analyzer.update(
      index * 0.1,
      (current_point,),
      PATH,
      LANE_LINES,
      LANE_PROBS,
    )[("corner235", 1167)]
  assert trajectory is not None
  assert current_point is not None

  model = object.__new__(RadarTrajectoryModel)
  model.source = "corner"
  model.probabilities = lambda _matrix: np.asarray(
    ((0.93, 0.96, 0.96, 0.96),), dtype=np.float32,
  )
  prediction = model.predict(
    {("corner235", 1167): trajectory},
    (current_point,),
    v_ego=9.36,
  )[0]

  assert prediction.forward_horizon_relevant == (False, False, False, False)
  assert prediction.probability == 0.0


def test_primary_lead_distance_blocks_only_far_detection_not_raw_score() -> None:
  trajectory = update_sequence((4.5, 4.0, 3.5, 3.0))
  near = TrajectoryCutinPrediction(
    10, "corner235", 0.99, (0.8, 0.9, 0.99, 0.95), trajectory, point(10, 20.0, 3.0),
  )
  far = TrajectoryCutinPrediction(
    11, "corner235", 1.0, (1.0, 1.0, 1.0, 1.0), trajectory, point(11, 35.0, 3.0),
  )
  raw = TrajectoryCutinDecision((near, far), (), (near, far))

  filtered = trajectory_decision_ahead_of_primary(raw, 30.0)

  assert filtered.predictions == (near, far)
  assert filtered.confirmed == (near,)


def test_production_path_occupancy_artifacts_load_with_separate_sources() -> None:
  front = RadarTrajectoryModel(DEFAULT_FRONT_MODEL_PATH, "front")
  corner = RadarTrajectoryModel(DEFAULT_CORNER_MODEL_PATH, "corner")
  matrix = np.zeros((1, len(TRAJECTORY_MODEL_FEATURE_NAMES)), dtype=np.float32)

  assert front.source == "front"
  assert corner.source == "corner"
  assert 0.0 < front.threshold <= 1.0
  assert 0.0 < front.exit_threshold <= 1.0
  assert front.probabilities(matrix).shape == (1, 4)
  assert corner.probabilities(matrix).shape == (1, 4)
  assert np.load(DEFAULT_FRONT_MODEL_PATH, allow_pickle=False)["manual_training_rows"].item() == 0
  assert np.load(DEFAULT_CORNER_MODEL_PATH, allow_pickle=False)["manual_training_rows"].item() == 0
