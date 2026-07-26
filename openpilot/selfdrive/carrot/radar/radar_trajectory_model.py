#!/usr/bin/env python3
"""Past-only future path-occupancy model shared by replay and radard."""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any

import numpy as np

from openpilot.selfdrive.carrot.radar.radar_trajectory import (
  RadarTrajectory,
  RadarTrajectoryAnalyzer,
  TARGET_HORIZONS_S,
  TRAJECTORY_MODEL_FEATURE_NAMES,
  forward_probability,
  path_center_occupied,
  path_occupancy_probability,
  radar_point_measured,
  radar_point_source,
  radar_point_track_id,
  radar_point_value,
  trajectory_model_feature_row,
)


MODEL_VERSION = 9
DEFAULT_FRONT_MODEL_PATH = Path(__file__).resolve().parent / "models" / "radar_path_occupancy_front.npz"
DEFAULT_CORNER_MODEL_PATH = Path(__file__).resolve().parent / "models" / "radar_path_occupancy_corner.npz"
MIN_FORWARD_ENTRY_DREL_M = 0.5
DECISION_HYSTERESIS = 0.05
TRACK_STATE_HOLD_S = 0.35
MEASURED_EXIT_CONFIRM_S = 0.25
MEASURED_EXIT_DISPLAY_HOLD_S = 1.0
CONTROL_RELEVANCE_MIN_DREL_M = 20.0
CONTROL_RELEVANCE_BUFFER_M = 10.0
POSITION_TARGET_NAMES = tuple(
  f"{axis}_{horizon_s:.1f}"
  for axis in ("x", "y")
  for horizon_s in TARGET_HORIZONS_S
)
OUTPUT_HEAD_NAMES = (
  *(f"mean_{name}" for name in POSITION_TARGET_NAMES),
  *(f"log_std_{name}" for name in POSITION_TARGET_NAMES),
)


def kinematic_position_baseline(matrix: np.ndarray) -> np.ndarray:
  """Project current longitudinal and path-relative lateral motion to each head."""
  values = np.asarray(matrix, dtype=np.float32)
  if values.ndim != 2 or values.shape[1] != len(TRAJECTORY_MODEL_FEATURE_NAMES):
    raise ValueError("trajectory baseline feature schema mismatch")
  d_rel = values[:, TRAJECTORY_MODEL_FEATURE_NAMES.index("d_rel")]
  v_rel = values[:, TRAJECTORY_MODEL_FEATURE_NAMES.index("v_rel")]
  d_path = values[:, TRAJECTORY_MODEL_FEATURE_NAMES.index("d_path")]
  d_path_rate = values[:, TRAJECTORY_MODEL_FEATURE_NAMES.index("d_path_rate")]
  horizons = np.asarray(TARGET_HORIZONS_S, dtype=np.float32)[None, :]
  future_x = d_rel[:, None] + v_rel[:, None] * horizons
  future_y = d_path[:, None] + d_path_rate[:, None] * horizons
  return np.concatenate((future_x, future_y), axis=1).astype(np.float32)


@dataclass(frozen=True)
class TrajectoryCutinPrediction:
  track_id: int
  source: str
  probability: float
  horizon_probabilities: tuple[float, ...]
  trajectory: RadarTrajectory
  point: object
  horizon_out_probabilities: tuple[float, ...] = ()
  path_exit_probability: float = 0.0
  current_path_occupancy: bool = False
  forward_horizon_relevant: tuple[bool, ...] = ()
  horizon_x: tuple[float, ...] = ()
  horizon_y: tuple[float, ...] = ()
  horizon_x_stds: tuple[float, ...] = ()
  horizon_y_stds: tuple[float, ...] = ()

  @property
  def path_in_probability(self) -> float:
    """Probability of occupying the ego path now or at a relevant future head."""
    return self.probability

  @property
  def path_out_probability(self) -> float:
    """Probability of being outside the ego path now or at a future head."""
    return self.path_exit_probability

  def probability_at(self, horizon_s: float) -> float:
    """Return IN probability derived from the learned future y distribution."""
    index = min(
      range(len(TARGET_HORIZONS_S)),
      key=lambda value: abs(TARGET_HORIZONS_S[value] - horizon_s),
    )
    return self.horizon_probabilities[index]

  def out_probability_at(self, horizon_s: float) -> float:
    """Return independently learned future OUT probability."""
    index = min(
      range(len(TARGET_HORIZONS_S)),
      key=lambda value: abs(TARGET_HORIZONS_S[value] - horizon_s),
    )
    if self.horizon_out_probabilities:
      return self.horizon_out_probabilities[index]
    return self.path_out_probability


@dataclass(frozen=True)
class TrajectoryCutinDecision:
  predictions: tuple[TrajectoryCutinPrediction, ...]
  tentative: tuple[TrajectoryCutinPrediction, ...]
  confirmed: tuple[TrajectoryCutinPrediction, ...]
  exiting: tuple[TrajectoryCutinPrediction, ...] = ()


EMPTY_DECISION = TrajectoryCutinDecision((), (), ())


def source_trajectory_decision(
  front: TrajectoryCutinDecision,
  corner: TrajectoryCutinDecision,
  corner_radar_enabled: bool,
) -> TrajectoryCutinDecision:
  """Assign corner-equipped entry to corner radar and retain front path exits."""
  if not corner_radar_enabled:
    return front
  return TrajectoryCutinDecision(
    predictions=corner.predictions,
    tentative=corner.tentative,
    confirmed=corner.confirmed,
    exiting=front.exiting + corner.exiting,
  )


def trajectory_decision_ahead_of_primary(
  decision: TrajectoryCutinDecision,
  primary_d_rel: float | None,
  control_max_d_rel: float | None = None,
) -> TrajectoryCutinDecision:
  """Keep raw scores visible while limiting actual leadTwo control relevance."""
  limits = tuple(
    value
    for value in (primary_d_rel, control_max_d_rel)
    if value is not None and math.isfinite(value)
  )
  if not limits:
    return decision
  maximum_d_rel = min(limits)

  def relevant(prediction: TrajectoryCutinPrediction) -> bool:
    return radar_point_value(prediction.point, "d_rel", "dRel") <= maximum_d_rel

  return TrajectoryCutinDecision(
    decision.predictions,
    tuple(prediction for prediction in decision.tentative if relevant(prediction)),
    tuple(prediction for prediction in decision.confirmed if relevant(prediction)),
    decision.exiting,
  )


def trajectory_control_max_d_rel(v_ego: float) -> float:
  """Bound control to what ego can reach over the model's prediction horizon."""
  return max(
    CONTROL_RELEVANCE_MIN_DREL_M,
    max(0.0, float(v_ego)) * max(TARGET_HORIZONS_S) + CONTROL_RELEVANCE_BUFFER_M,
  )


class RadarTrajectoryModel:
  """Small NumPy MLP predicting future x/y Gaussian distributions."""

  def __init__(self, path: Path, expected_source: str) -> None:
    artifact = np.load(path, allow_pickle=False)
    version = int(artifact["model_version"].reshape(-1)[0])
    source = str(artifact["sensor_mode"].reshape(-1)[0])
    feature_names = tuple(str(value) for value in artifact["feature_names"].tolist())
    target_horizons = tuple(float(value) for value in artifact["target_horizons_s"].tolist())
    output_names = tuple(str(value) for value in artifact["output_head_names"].tolist())
    manual_training_rows = int(artifact["manual_training_rows"].reshape(-1)[0])
    if version != MODEL_VERSION:
      raise ValueError(f"unsupported trajectory model version {version}")
    if source != expected_source:
      raise ValueError(f"expected {expected_source} model, got {source}")
    if feature_names != TRAJECTORY_MODEL_FEATURE_NAMES:
      raise ValueError("trajectory model feature schema mismatch")
    if target_horizons != TARGET_HORIZONS_S:
      raise ValueError("trajectory target horizon schema mismatch")
    if output_names != OUTPUT_HEAD_NAMES:
      raise ValueError("trajectory x/y distribution output schema mismatch")
    if manual_training_rows != 0:
      raise ValueError("manual labels are forbidden in the trajectory model")

    self.source = source
    self.threshold = float(artifact["entry_threshold"].reshape(-1)[0])
    self.exit_threshold = float(artifact["exit_threshold"].reshape(-1)[0])
    self.feature_mean = artifact["feature_mean"].astype(np.float32)
    self.feature_std = artifact["feature_std"].astype(np.float32)
    self.target_mean = artifact["target_mean"].astype(np.float32)
    self.target_std = artifact["target_std"].astype(np.float32)
    self.sigma_calibration = artifact["sigma_calibration"].astype(np.float32)
    self.w1 = artifact["w1"].astype(np.float32)
    self.b1 = artifact["b1"].astype(np.float32)
    self.w2 = artifact["w2"].astype(np.float32)
    self.b2 = artifact["b2"].astype(np.float32)
    self.w3 = artifact["w3"].astype(np.float32)
    self.b3 = artifact["b3"].astype(np.float32)
    feature_count = len(TRAJECTORY_MODEL_FEATURE_NAMES)
    target_count = len(POSITION_TARGET_NAMES)
    output_count = len(OUTPUT_HEAD_NAMES)
    if (
      self.feature_mean.shape != (feature_count,)
      or self.feature_std.shape != (feature_count,)
      or np.any(~np.isfinite(self.feature_mean))
      or np.any(~np.isfinite(self.feature_std))
      or np.any(self.feature_std <= 0.0)
    ):
      raise ValueError("trajectory feature normalization schema mismatch")
    if (
      self.target_mean.shape != (target_count,)
      or self.target_std.shape != (target_count,)
      or self.sigma_calibration.shape != (target_count,)
      or np.any(~np.isfinite(self.target_mean))
      or np.any(~np.isfinite(self.target_std))
      or np.any(~np.isfinite(self.sigma_calibration))
      or np.any(self.target_std <= 0.0)
      or np.any(self.sigma_calibration <= 0.0)
    ):
      raise ValueError("trajectory target distribution schema mismatch")
    weights_and_biases = (self.w1, self.b1, self.w2, self.b2, self.w3, self.b3)
    if any(np.any(~np.isfinite(value)) for value in weights_and_biases):
      raise ValueError("trajectory model contains non-finite parameters")
    if (
      self.w1.ndim != 2
      or self.w1.shape[0] != feature_count
      or self.b1.shape != (self.w1.shape[1],)
      or self.w2.ndim != 2
      or self.w2.shape[0] != self.w1.shape[1]
      or self.b2.shape != (self.w2.shape[1],)
      or self.w3.shape != (self.w2.shape[1], output_count)
      or self.b3.shape != (output_count,)
    ):
      raise ValueError("trajectory model output schema mismatch")
    if not 0.0 <= self.threshold <= 1.0 or not 0.0 <= self.exit_threshold <= 1.0:
      raise ValueError("trajectory model probability threshold out of range")

  def distributions(self, matrix: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    values = matrix.astype(np.float32)
    normalized = (values - self.feature_mean) / self.feature_std
    hidden1 = np.maximum(normalized @ self.w1 + self.b1, 0.0)
    hidden2 = np.maximum(hidden1 @ self.w2 + self.b2, 0.0)
    output = hidden2 @ self.w3 + self.b3
    target_count = len(POSITION_TARGET_NAMES)
    residual_means = output[:, :target_count] * self.target_std + self.target_mean
    means = kinematic_position_baseline(values) + residual_means
    log_stds = np.clip(output[:, target_count:], -4.0, 3.0)
    stds = np.exp(log_stds) * self.target_std * self.sigma_calibration
    return means, np.maximum(stds, 0.05)

  def predict(
    self,
    trajectories: dict[tuple[str, int], RadarTrajectory],
    points: Iterable[object],
    v_ego: float,
  ) -> tuple[TrajectoryCutinPrediction, ...]:
    candidates: list[tuple[object, RadarTrajectory]] = []
    rows: list[list[float]] = []
    for point in points:
      if not radar_point_measured(point):
        continue
      source = radar_point_source(point)
      if (self.source == "corner") != source.startswith("corner"):
        continue
      if self.source == "front" and source not in ("frontRadar", "scc"):
        continue
      track_id = radar_point_track_id(point)
      trajectory = trajectories.get((source, track_id))
      if trajectory is None or trajectory.history_count < 3:
        continue
      row = trajectory_model_feature_row(trajectory, point, v_ego)
      rows.append([row[name] for name in TRAJECTORY_MODEL_FEATURE_NAMES])
      candidates.append((point, trajectory))
    if not rows:
      return ()
    position_means, position_stds = self.distributions(
      np.asarray(rows, dtype=np.float32),
    )
    result = []
    horizon_count = len(TARGET_HORIZONS_S)
    for (point, trajectory), means, stds in zip(
      candidates, position_means, position_stds, strict=True,
    ):
      x_values = tuple(float(value) for value in means[:horizon_count])
      y_values = tuple(float(value) for value in means[horizon_count:])
      x_stds = tuple(float(value) for value in stds[:horizon_count])
      y_stds = tuple(float(value) for value in stds[horizon_count:])
      if not all(math.isfinite(value) for value in (
        *x_values, *y_values, *x_stds, *y_stds,
      )):
        continue
      samples = tuple(
        min(
          trajectory.samples,
          key=lambda sample: abs(sample.horizon_s - horizon_s),
        )
        for horizon_s in TARGET_HORIZONS_S
      )
      ahead_values = tuple(
        forward_probability(x_value, x_std, MIN_FORWARD_ENTRY_DREL_M)
        for x_value, x_std in zip(x_values, x_stds, strict=True)
      )
      lane_in_values = tuple(
        path_occupancy_probability(y_value, y_std, sample.lane_half_width)
        for y_value, y_std, sample in zip(
          y_values, y_stds, samples, strict=True,
        )
      )
      in_values = tuple(
        ahead * lane_in
        for ahead, lane_in in zip(ahead_values, lane_in_values, strict=True)
      )
      out_values = tuple(
        ahead * (1.0 - lane_in)
        for ahead, lane_in in zip(ahead_values, lane_in_values, strict=True)
      )
      forward_horizon_relevant = tuple(
        x_value > MIN_FORWARD_ENTRY_DREL_M
        for x_value in x_values
      )
      current_path_occupancy = path_center_occupied(
        trajectory.d_path, trajectory.lane_half_width,
      )
      future_path_in_probability = max(in_values, default=0.0)
      path_in_probability = max(
        float(current_path_occupancy),
        future_path_in_probability,
      )
      path_out_probability = max(
        float(not current_path_occupancy),
        max(out_values, default=0.0),
      )
      result.append(TrajectoryCutinPrediction(
        track_id=radar_point_track_id(point),
        source=radar_point_source(point),
        probability=path_in_probability,
        horizon_probabilities=in_values,
        trajectory=trajectory,
        point=point,
        horizon_out_probabilities=out_values,
        path_exit_probability=path_out_probability,
        current_path_occupancy=current_path_occupancy,
        forward_horizon_relevant=forward_horizon_relevant,
        horizon_x=x_values,
        horizon_y=y_values,
        horizon_x_stds=x_stds,
        horizon_y_stds=y_stds,
      ))
    return tuple(result)


class RadarTrajectoryDecisionFilter:
  """Latch thresholded IN until measured exit; rank active candidates by range."""

  def __init__(
    self,
    threshold: float,
    exit_threshold: float | None = None,
    hysteresis: float = DECISION_HYSTERESIS,
    state_hold_s: float = TRACK_STATE_HOLD_S,
    measured_exit_confirm_s: float = MEASURED_EXIT_CONFIRM_S,
  ) -> None:
    self.threshold = float(threshold)
    self.exit_threshold = float(threshold if exit_threshold is None else exit_threshold)
    self.hysteresis = max(0.0, float(hysteresis))
    self.state_hold_s = max(0.0, float(state_hold_s))
    self.measured_exit_confirm_s = max(0.0, float(measured_exit_confirm_s))
    self._active_entry: set[tuple[str, int, int]] = set()
    self._latched_inside: set[tuple[str, int, int]] = set()
    self._active_exit: set[tuple[str, int, int]] = set()
    self._previous_occupancy: dict[tuple[str, int, int], bool] = {}
    self._last_seen: dict[tuple[str, int, int], float] = {}
    self._outside_since: dict[tuple[str, int, int], float] = {}
    self._measured_exit_time: dict[tuple[str, int, int], float] = {}

  @staticmethod
  def _identity(prediction: TrajectoryCutinPrediction) -> tuple[str, int, int]:
    return (
      prediction.source,
      prediction.track_id,
      prediction.trajectory.continuity_id,
    )

  def update(
    self,
    time_s: float,
    predictions: Iterable[TrajectoryCutinPrediction],
  ) -> TrajectoryCutinDecision:
    predictions = tuple(predictions)
    entry_release = max(0.0, self.threshold - self.hysteresis)
    exit_release = max(0.0, self.exit_threshold - self.hysteresis)
    for prediction in predictions:
      identity = self._identity(prediction)
      self._last_seen[identity] = time_s
      previous = self._previous_occupancy.get(identity)
      self._previous_occupancy[identity] = prediction.current_path_occupancy

      if identity not in self._active_entry:
        if prediction.path_in_probability >= self.threshold:
          self._active_entry.add(identity)
      elif identity not in self._latched_inside and prediction.path_in_probability < entry_release:
        self._active_entry.discard(identity)

      if identity in self._active_entry and prediction.current_path_occupancy:
        self._latched_inside.add(identity)
        self._outside_since.pop(identity, None)
      elif identity in self._latched_inside and not prediction.current_path_occupancy:
        outside_since = self._outside_since.setdefault(identity, time_s)
        if time_s - outside_since >= self.measured_exit_confirm_s:
          self._active_entry.discard(identity)
          self._latched_inside.discard(identity)
          self._outside_since.pop(identity, None)
          self._measured_exit_time[identity] = time_s

      exit_threshold = exit_release if identity in self._active_exit else self.exit_threshold
      if (
        prediction.current_path_occupancy
        and prediction.path_out_probability >= exit_threshold
      ):
        self._active_exit.add(identity)
      else:
        self._active_exit.discard(identity)

      if previous is True and not prediction.current_path_occupancy:
        self._measured_exit_time[identity] = time_s

    expired = {
      identity
      for identity, last_seen in self._last_seen.items()
      if time_s - last_seen > self.state_hold_s
    }
    for identity in expired:
      self._active_entry.discard(identity)
      self._latched_inside.discard(identity)
      self._active_exit.discard(identity)
      self._previous_occupancy.pop(identity, None)
      self._last_seen.pop(identity, None)
      self._outside_since.pop(identity, None)
      self._measured_exit_time.pop(identity, None)

    self._measured_exit_time = {
      identity: transition_time
      for identity, transition_time in self._measured_exit_time.items()
      if time_s - transition_time <= MEASURED_EXIT_DISPLAY_HOLD_S
    }
    confirmed = [
      prediction
      for prediction in predictions
      if self._identity(prediction) in self._active_entry
    ]
    exiting = [
      prediction
      for prediction in predictions
      if (
        self._identity(prediction) in self._active_exit
        or self._identity(prediction) in self._measured_exit_time
        or self._identity(prediction) in self._outside_since
      )
    ]
    tentative: list[TrajectoryCutinPrediction] = []
    confirmed.sort(key=lambda item: (
      radar_point_value(item.point, "d_rel", "dRel"),
      -item.path_in_probability,
    ))
    exiting.sort(key=lambda item: (
      radar_point_value(item.point, "d_rel", "dRel"),
      -item.path_out_probability,
    ))
    return TrajectoryCutinDecision(predictions, tuple(tentative), tuple(confirmed), tuple(exiting))


@dataclass(frozen=True)
class RadarTrajectoryRuntimeResult:
  available: bool
  trajectories: dict[tuple[str, int], RadarTrajectory]
  predictions: tuple[TrajectoryCutinPrediction, ...]
  decision: TrajectoryCutinDecision
  front_predictions: tuple[TrajectoryCutinPrediction, ...] = ()
  corner_predictions: tuple[TrajectoryCutinPrediction, ...] = ()
  front_decision: TrajectoryCutinDecision = EMPTY_DECISION
  corner_decision: TrajectoryCutinDecision = EMPTY_DECISION
  error: str = ""


class RadarTrajectoryRuntime:
  """The single feature, model, and post-processing path used by replay and device."""

  def __init__(
    self,
    *,
    front_model_path: Path = DEFAULT_FRONT_MODEL_PATH,
    corner_model_path: Path = DEFAULT_CORNER_MODEL_PATH,
    corner_radar_enabled: bool = False,
  ) -> None:
    self.front_model_path = Path(front_model_path)
    self.corner_model_path = Path(corner_model_path)
    self.corner_radar_enabled = bool(corner_radar_enabled)
    self.analyzer = RadarTrajectoryAnalyzer()
    self.front_model: RadarTrajectoryModel | None = None
    self.corner_model: RadarTrajectoryModel | None = None
    self.front_filter: RadarTrajectoryDecisionFilter | None = None
    self.corner_filter: RadarTrajectoryDecisionFilter | None = None
    self.load_error = ""

  def _load(self) -> bool:
    if self.front_model is not None and self.corner_model is not None:
      return True
    try:
      self.front_model = RadarTrajectoryModel(self.front_model_path, "front")
      self.corner_model = RadarTrajectoryModel(self.corner_model_path, "corner")
      self.front_filter = RadarTrajectoryDecisionFilter(
        self.front_model.threshold, self.front_model.exit_threshold,
      )
      self.corner_filter = RadarTrajectoryDecisionFilter(
        self.corner_model.threshold, self.corner_model.exit_threshold,
      )
      self.load_error = ""
      return True
    except Exception as exc:
      self.load_error = f"{type(exc).__name__}: {exc}"
      return False

  def update(
    self,
    time_s: float,
    v_ego: float,
    points: Sequence[Any],
    path: Sequence[tuple[float, float]],
    lane_lines: Sequence[Sequence[tuple[float, float]]],
    lane_probs: Sequence[float],
    path_y_stds: Sequence[tuple[float, float]] = (),
    lane_stds: Sequence[float] = (),
    steering_angle_deg: float = 0.0,
    steering_rate_deg_s: float = 0.0,
    yaw_rate_rad_s: float = 0.0,
    yaw_rate_estimated: bool = False,
  ) -> RadarTrajectoryRuntimeResult:
    if not self._load():
      return RadarTrajectoryRuntimeResult(False, {}, (), EMPTY_DECISION, error=self.load_error)
    try:
      trajectories = self.analyzer.update(
        time_s,
        points,
        path,
        lane_lines,
        lane_probs,
        path_y_stds,
        lane_stds,
        steering_angle_deg,
        steering_rate_deg_s,
        yaw_rate_rad_s,
        yaw_rate_estimated,
      )
      assert self.front_model is not None and self.corner_model is not None
      assert self.front_filter is not None and self.corner_filter is not None
      front_predictions = self.front_model.predict(trajectories, points, v_ego)
      corner_predictions = self.corner_model.predict(trajectories, points, v_ego)
      front_decision = self.front_filter.update(time_s, front_predictions)
      corner_decision = self.corner_filter.update(time_s, corner_predictions)
      decision = source_trajectory_decision(
        front_decision, corner_decision, self.corner_radar_enabled,
      )
      return RadarTrajectoryRuntimeResult(
        True,
        trajectories,
        front_predictions + corner_predictions,
        decision,
        front_predictions,
        corner_predictions,
        front_decision,
        corner_decision,
      )
    except Exception as exc:
      return RadarTrajectoryRuntimeResult(
        False, {}, (), EMPTY_DECISION, error=f"{type(exc).__name__}: {exc}",
      )
