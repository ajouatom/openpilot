import json
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.carrot.radar.radar_trajectory import TRAJECTORY_MODEL_FEATURE_NAMES
from openpilot.selfdrive.carrot.radar.tools.radar_lead_simulator import RadarPoint
from openpilot.selfdrive.carrot.radar.tools.radar_trajectory_train import (
  Dataset,
  ENTRY_LATERAL_WEIGHT,
  ENTRY_LONGITUDINAL_WEIGHT,
  EXIT_LATERAL_WEIGHT,
  EXIT_LONGITUDINAL_WEIGHT,
  _downsample_rows,
  _frame_rows,
  _future_targets,
  _group_folds,
  _load_log_rows,
  _model_targets,
  _outside_targets,
  _position_probabilities,
  _residual_model_targets,
  _save_log_rows,
  _segment_log_group,
  _training_head_weights,
  _track_observations,
  _weighted_quantile,
  evaluation_segments,
)
from openpilot.selfdrive.carrot.radar.tools.radar_trajectory_compare import (
  ComparisonRow,
  _path_occupancy_selected,
  _summary,
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
  d_rel: float,
  y_rel: float,
  *,
  measured: bool = True,
  v_rel: float = 0.0,
  yv_rel: float = 0.0,
) -> RadarPoint:
  return RadarPoint(
    track_id=10,
    d_rel=d_rel,
    y_rel=y_rel,
    v_rel=v_rel,
    a_rel=0.0,
    yv_rel=yv_rel,
    v_lead=20.0 + v_rel,
    measured=measured,
    source="frontRadar",
  )


def frame(time_s: float, points: tuple[RadarPoint, ...]) -> SimpleNamespace:
  return SimpleNamespace(
    mono_time_s=time_s,
    time_s=time_s,
    v_ego=20.0,
    points=points,
    path=PATH,
    lane_lines=LANE_LINES,
    lane_probs=LANE_PROBS,
    path_y_stds=(),
    lane_stds=(),
    steering_angle_deg=0.0,
    steering_rate_deg_s=0.0,
    yaw_rate_rad_s=0.0,
    yaw_rate_estimated=False,
  )


def test_future_target_uses_measured_same_episode_position() -> None:
  frames = (
    frame(0.0, (point(25.0, 4.0),)),
    frame(0.25, (point(25.0, 3.2),)),
    frame(0.5, (point(25.0, 2.5),)),
    frame(0.75, (point(25.0, 1.8),)),
    frame(1.0, (point(25.0, 1.0),)),
  )
  by_frame, episodes = _track_observations(frames)
  current = by_frame[(0, "frontRadar", 10)]
  positions, lane_half_widths, valid = _future_targets(
    frames,
    current,
    episodes[("frontRadar", 10, current.episode_id)],
  )

  assert valid[:2] == [True, True]
  assert positions[:2] == [25.0, 25.0]
  assert positions[4:6] == [2.5, 1.0]
  assert lane_half_widths[:2] == [1.8, 1.8]
  assert valid[2:] == [False, False]


def test_unmeasured_future_slot_is_not_ground_truth() -> None:
  frames = (
    frame(0.0, (point(25.0, 4.0),)),
    frame(0.5, (point(0.0, 0.0, measured=False),)),
  )
  by_frame, episodes = _track_observations(frames)
  current = by_frame[(0, "frontRadar", 10)]
  positions, lane_half_widths, valid = _future_targets(
    frames,
    current,
    episodes[("frontRadar", 10, current.episode_id)],
  )

  assert not valid[0]
  assert positions[0] == 0.0
  assert lane_half_widths[0] == 0.0


def test_currently_occupied_vehicle_keeps_unconditional_future_position_target() -> None:
  frames = (
    frame(0.0, (point(25.0, 0.2),)),
    frame(0.25, (point(25.0, 0.3),)),
    frame(0.5, (point(25.0, 0.4),)),
  )
  by_frame, episodes = _track_observations(frames)
  current = by_frame[(0, "frontRadar", 10)]
  positions, _, valid = _future_targets(
    frames,
    current,
    episodes[("frontRadar", 10, current.episode_id)],
  )

  assert valid[0]
  assert positions[0] == 25.0
  assert positions[4] == 0.4


def test_future_path_exit_keeps_measured_outside_position_target() -> None:
  frames = (
    frame(0.0, (point(25.0, 0.2, yv_rel=5.0),)),
    frame(0.25, (point(25.0, 1.45, yv_rel=5.0),)),
    frame(0.5, (point(25.0, 2.8, yv_rel=5.0),)),
  )
  by_frame, episodes = _track_observations(frames)
  current = by_frame[(0, "frontRadar", 10)]
  positions, _, valid = _future_targets(
    frames,
    current,
    episodes[("frontRadar", 10, current.episode_id)],
  )

  assert valid[0]
  assert positions[4] == 2.8


def test_training_exposes_x_y_targets_with_per_horizon_validity() -> None:
  dataset = Dataset(
    features=np.zeros((1, 2), dtype=np.float32),
    labels=np.asarray(((10.0, 9.0, 8.0, 7.0, 1.0, 2.0, 3.0, 4.0),), dtype=np.float32),
    lane_half_widths=np.asarray(((1.8, 1.8, 1.8, 1.8),), dtype=np.float32),
    valid=np.asarray(((True, True, False, True),)),
    current_occupancy=np.asarray((True,)),
    sample_ids=np.asarray(("sample",)),
    log_groups=np.asarray(("log",)),
  )

  targets, valid = _model_targets(dataset)

  assert targets.tolist() == [[10.0, 9.0, 8.0, 7.0, 1.0, 2.0, 3.0, 4.0]]
  assert valid.tolist() == [[True, True, False, True, True, True, False, True]]


def test_training_learns_residual_over_past_only_kinematic_projection() -> None:
  features = np.zeros((1, len(TRAJECTORY_MODEL_FEATURE_NAMES)), dtype=np.float32)
  features[0, TRAJECTORY_MODEL_FEATURE_NAMES.index("d_rel")] = 20.0
  features[0, TRAJECTORY_MODEL_FEATURE_NAMES.index("v_rel")] = -2.0
  features[0, TRAJECTORY_MODEL_FEATURE_NAMES.index("d_path")] = 3.0
  features[0, TRAJECTORY_MODEL_FEATURE_NAMES.index("d_path_rate")] = -1.0
  dataset = Dataset(
    features=features,
    labels=np.asarray(((19.5, 18.0, 17.0, 16.0, 2.4, 2.0, 1.4, 1.0),), dtype=np.float32),
    lane_half_widths=np.full((1, 4), 1.8, dtype=np.float32),
    valid=np.ones((1, 4), dtype=np.bool_),
    current_occupancy=np.zeros(1, dtype=np.bool_),
    sample_ids=np.asarray(("sample",)),
    log_groups=np.asarray(("log",)),
  )

  residuals, valid = _residual_model_targets(dataset)

  assert valid.all()
  np.testing.assert_allclose(
    residuals,
    np.asarray(((
      0.5, 0.0, 0.0, 0.0,
      -0.1, 0.0, -0.1, 0.0,
    ),), dtype=np.float32),
    atol=1e-6,
  )


def test_training_weights_emphasize_self_supervised_path_transitions() -> None:
  dataset = Dataset(
    features=np.zeros((3, 1), dtype=np.float32),
    labels=np.asarray((
      (20.0, 20.0, 20.0, 20.0, 3.0, 2.5, 1.7, 1.0),
      (20.0, 20.0, 20.0, 20.0, 0.0, 1.0, 2.0, 3.0),
      (20.0, 20.0, 20.0, 20.0, 3.0, 3.0, 3.0, 3.0),
    ), dtype=np.float32),
    lane_half_widths=np.full((3, 4), 1.8, dtype=np.float32),
    valid=np.ones((3, 4), dtype=np.bool_),
    current_occupancy=np.asarray((False, True, False)),
    sample_ids=np.asarray(("entry", "exit", "steady")),
    log_groups=np.asarray(("a", "b", "c")),
  )

  weights = _training_head_weights(dataset)

  np.testing.assert_allclose(weights[0, :4], ENTRY_LONGITUDINAL_WEIGHT)
  np.testing.assert_allclose(weights[0, 4:], ENTRY_LATERAL_WEIGHT)
  np.testing.assert_allclose(weights[1, :4], EXIT_LONGITUDINAL_WEIGHT)
  np.testing.assert_allclose(weights[1, 4:], EXIT_LATERAL_WEIGHT)
  np.testing.assert_allclose(weights[2], 1.0)


def test_weighted_quantile_gives_transition_rows_more_calibration_influence() -> None:
  values = np.asarray((1.0, 2.0, 8.0), dtype=np.float32)
  weights = np.asarray((1.0, 1.0, 4.0), dtype=np.float32)

  assert _weighted_quantile(values, weights, 0.6827) == 8.0


def test_group_folds_keep_every_segment_in_exactly_one_fold() -> None:
  group_codes = np.repeat(np.arange(6, dtype=np.int32), 2)
  future_y = np.where(group_codes[:, None] % 2 == 0, 0.0, 3.0)
  dataset = Dataset(
    features=np.zeros((len(group_codes), 2), dtype=np.float32),
    labels=np.concatenate((
      np.full((len(group_codes), 4), 20.0, dtype=np.float32),
      np.repeat(future_y, 4, axis=1).astype(np.float32),
    ), axis=1),
    lane_half_widths=np.full((len(group_codes), 4), 1.8, dtype=np.float32),
    valid=np.ones((len(group_codes), 4), dtype=np.bool_),
    current_occupancy=np.zeros(len(group_codes), dtype=np.bool_),
    sample_ids=np.empty(0, dtype=np.str_),
    log_groups=group_codes,
  )

  folds = _group_folds(dataset, seed=7, fold_count=3)

  assert len(folds) == 3
  assert np.all(np.sum(np.stack(folds), axis=0) == 1)
  for group_code in np.unique(group_codes):
    memberships = {
      fold_index
      for fold_index, mask in enumerate(folds)
      if np.any(mask & (group_codes == group_code))
    }
    assert len(memberships) == 1


def test_downsampling_does_not_treat_behind_ego_as_path_occupied() -> None:
  behind = (
    [0.0],
    [-1.0, -2.0, -3.0, -4.0, 0.0, 0.0, 0.0, 0.0],
    [1.8, 1.8, 1.8, 1.8],
    [True, True, True, True],
    False,
    "behind",
    "log",
  )
  actual_entry = (
    [0.0],
    [10.0, 9.0, 8.0, 7.0, 3.0, 1.0, 0.5, 0.0],
    [1.8, 1.8, 1.8, 1.8],
    [True, True, True, True],
    False,
    "entry",
    "log",
  )

  sampled = _downsample_rows([behind, actual_entry])

  assert sampled[0][5] == "entry"
  assert sampled[1][5] == "behind"


def test_position_probabilities_use_x_and_y_uncertainty() -> None:
  means = np.asarray(((
    0.5, 10.0, -10.0, 10.0,
    0.0, 0.0, 0.0, 3.0,
  ),), dtype=np.float32)
  stds = np.ones_like(means)
  widths = np.asarray(((1.8, 1.8, 1.8, 1.8),), dtype=np.float32)

  probabilities = _position_probabilities(means, stds, widths)

  assert probabilities[0, 0] == pytest.approx(0.464, abs=0.002)
  assert probabilities[0, 1] > 0.92
  assert probabilities[0, 2] < 1e-20
  assert probabilities[0, 7] > 0.88
  assert probabilities[0, 0] + probabilities[0, 4] == pytest.approx(0.5, abs=0.002)


def test_behind_ego_is_neither_future_inside_nor_future_outside() -> None:
  dataset = Dataset(
    features=np.zeros((1, 1), dtype=np.float32),
    labels=np.asarray(((-1.0, 10.0, 10.0, 10.0, 0.0, 0.0, 3.0, 3.0),), dtype=np.float32),
    lane_half_widths=np.asarray(((1.8, 1.8, 1.8, 1.8),), dtype=np.float32),
    valid=np.asarray(((True, True, True, True),)),
    current_occupancy=np.asarray((True,)),
    sample_ids=np.asarray(("sample",)),
    log_groups=np.asarray(("log",)),
  )

  assert not _outside_targets(dataset)[0, 0]
  assert _outside_targets(dataset)[0, 2]


def test_future_measurements_change_only_targets_not_current_model_input() -> None:
  shared_past = (
    frame(0.0, (point(25.0, 4.0),)),
    frame(0.25, (point(25.0, 3.8),)),
    frame(0.5, (point(25.0, 3.6),)),
  )
  entering = shared_past + (
    frame(0.75, (point(25.0, 2.7),)),
    frame(1.0, (point(25.0, 1.7),)),
  )
  staying_out = shared_past + (
    frame(0.75, (point(25.0, 3.8),)),
    frame(1.0, (point(25.0, 4.0),)),
  )

  entering_rows = _frame_rows(entering, "entering")["front"]
  staying_rows = _frame_rows(staying_out, "staying")["front"]
  entering_current = next(row for row in entering_rows if row[5].endswith(":0.50"))
  staying_current = next(row for row in staying_rows if row[5].endswith(":0.50"))

  assert entering_current[0] == staying_current[0]
  assert entering_current[1][4] == 1.7
  assert staying_current[1][4] == 4.0


def test_reused_id_cannot_supply_future_ground_truth() -> None:
  frames = (
    frame(0.0, (point(25.0, 4.0),)),
    frame(0.25, (point(25.0, 3.8),)),
    frame(0.5, (point(80.0, -5.0),)),
  )
  by_frame, episodes = _track_observations(frames)
  current = by_frame[(0, "frontRadar", 10)]
  reused = by_frame[(2, "frontRadar", 10)]
  _, _, valid = _future_targets(
    frames,
    current,
    episodes[("frontRadar", 10, current.episode_id)],
  )

  assert reused.episode_id != current.episode_id
  assert not valid[0]


def test_brief_physically_continuous_dropout_keeps_episode() -> None:
  frames = (
    frame(0.0, (point(25.0, 4.0, v_rel=-1.0),)),
    frame(0.1, ()),
    frame(0.2, (point(24.8, 3.9, v_rel=-1.0),)),
  )
  by_frame, _ = _track_observations(frames)

  assert by_frame[(0, "frontRadar", 10)].episode_id == by_frame[(2, "frontRadar", 10)].episode_id


def test_incremental_log_cache_round_trip_and_fingerprint_guard(tmp_path) -> None:
  cache_path = tmp_path / "log.npz"
  fingerprint = {"size": 123, "mtime_ns": 456}
  rows = {
    "front": [(
      [1.0, 2.0],
      [20.0, 18.0, 16.0, 14.0, 3.0, 2.0, 1.0, 0.0],
      [1.8, 1.8, 1.8, 1.8],
      [True, True, False, False],
      False,
      "sample",
      "log",
    )],
  }

  _save_log_rows(cache_path, "vehicle/log/rlog.zst", fingerprint, rows)

  loaded = _load_log_rows(cache_path, "vehicle/log/rlog.zst", fingerprint)
  assert loaded is not None
  assert loaded["front"][0][:2] == rows["front"][0][:2]
  assert loaded["front"][0][2] == pytest.approx(rows["front"][0][2])
  assert loaded["front"][0][3:] == rows["front"][0][3:]
  assert _load_log_rows(cache_path, "vehicle/log/rlog.zst", {"size": 124, "mtime_ns": 456}) is None


def test_every_rlog_variant_in_segment_uses_same_cross_validation_group() -> None:
  assert _segment_log_group("vehicle/route--0/rlog.zst") == "vehicle/route--0"
  assert _segment_log_group("vehicle/route--0/rlog.1.zst") == "vehicle/route--0"


def test_manual_label_holds_out_every_rlog_variant_in_segment(tmp_path) -> None:
  labels_path = tmp_path / "labels.json"
  cases_path = tmp_path / "cases.json"
  labels_path.write_text(json.dumps({
    "labels": [{
      "id": "held-out",
      "vehicle_folder": "vehicle",
      "log": "route--0/rlog.1.zst",
      "time_s": 1.0,
      "track_id": 10,
      "source": "front",
      "expected": "detect",
      "human_verified": True,
    }],
  }), encoding="utf-8")
  cases_path.write_text(json.dumps({
    "cases": [{
      "id": "additional-held-out",
      "vehicle_folder": "vehicle",
      "log": "other-route--0/rlog.zst",
      "expected": "clear",
      "human_verified": True,
    }],
  }), encoding="utf-8")

  segments = evaluation_segments(labels_path, tmp_path, cases_path)

  assert (tmp_path / "vehicle" / "route--0").resolve() in segments
  assert (tmp_path / "vehicle" / "route--0" / "rlog.zst").parent.resolve() in segments
  assert (tmp_path / "vehicle" / "other-route--0").resolve() in segments


def test_carrot_wip_comparison_summary_is_source_separated() -> None:
  rows = (
    ComparisonRow("f-tp", "front", "detect", "detect", True, 0.9, 3),
    ComparisonRow("f-fp", "front", "clear", "detect", True, 0.8, 2),
    ComparisonRow("c-tn", "corner", "clear", "clear", False, 0.1, 4),
  )

  values = _summary(rows)

  assert values["front"]["tp"] == 1
  assert values["front"]["fp"] == 1
  assert values["front"]["precision"] == 0.5
  assert values["corner"]["tn"] == 1
  assert values["corner"]["fp"] == 0


def test_path_occupancy_comparison_counts_only_final_lead_two() -> None:
  assert _path_occupancy_selected(SimpleNamespace(stage="SELECTED"))
  assert not _path_occupancy_selected(SimpleNamespace(stage="OUTPUT"))
  assert not _path_occupancy_selected(SimpleNamespace(stage="DECISION"))
