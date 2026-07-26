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
  VEHICLE_HALF_WIDTH_M,
  radar_point_measured,
  radar_point_source,
  radar_point_track_id,
  radar_point_value,
  trajectory_model_feature_row,
)


MODEL_VERSION = 3
DEFAULT_FRONT_MODEL_PATH = Path(__file__).resolve().parent / "models" / "radar_path_occupancy_front.npz"
DEFAULT_CORNER_MODEL_PATH = Path(__file__).resolve().parent / "models" / "radar_path_occupancy_corner.npz"
MIN_FORWARD_ENTRY_DREL_M = 0.5
DECISION_HYSTERESIS = 0.05
MEASURED_TRANSITION_HOLD_S = 1.0


@dataclass(frozen=True)
class TrajectoryCutinPrediction:
  track_id: int
  source: str
  probability: float
  horizon_probabilities: tuple[float, ...]
  trajectory: RadarTrajectory
  point: object
  path_exit_probability: float = 0.0
  current_path_occupancy: bool = False
  forward_horizon_relevant: tuple[bool, ...] = ()

  @property
  def path_in_probability(self) -> float:
    """Probability of occupying the ego path now or at a relevant future head."""
    return self.probability

  @property
  def path_out_probability(self) -> float:
    """Probability of being outside the ego path now or at a future head."""
    return self.path_exit_probability

  def probability_at(self, horizon_s: float) -> float:
    """Return future path-occupancy probability at the nearest model horizon."""
    index = min(
      range(len(TARGET_HORIZONS_S)),
      key=lambda value: abs(TARGET_HORIZONS_S[value] - horizon_s),
    )
    return self.horizon_probabilities[index]


@dataclass(frozen=True)
class TrajectoryCutinDecision:
  predictions: tuple[TrajectoryCutinPrediction, ...]
  tentative: tuple[TrajectoryCutinPrediction, ...]
  confirmed: tuple[TrajectoryCutinPrediction, ...]
  exiting: tuple[TrajectoryCutinPrediction, ...] = ()


EMPTY_DECISION = TrajectoryCutinDecision((), (), ())


def trajectory_decision_ahead_of_primary(
  decision: TrajectoryCutinDecision,
  primary_d_rel: float | None,
) -> TrajectoryCutinDecision:
  """Keep raw scores visible, but do not detect farther than leadOne."""
  if primary_d_rel is None or not math.isfinite(primary_d_rel):
    return decision

  def relevant(prediction: TrajectoryCutinPrediction) -> bool:
    return radar_point_value(prediction.point, "d_rel", "dRel") <= primary_d_rel

  return TrajectoryCutinDecision(
    decision.predictions,
    tuple(prediction for prediction in decision.tentative if relevant(prediction)),
    tuple(prediction for prediction in decision.confirmed if relevant(prediction)),
    decision.exiting,
  )


class RadarTrajectoryModel:
  """Small NumPy MLP with one calibrated probability per future horizon."""

  def __init__(self, path: Path, expected_source: str) -> None:
    artifact = np.load(path, allow_pickle=False)
    version = int(artifact["model_version"].reshape(-1)[0])
    source = str(artifact["sensor_mode"].reshape(-1)[0])
    feature_names = tuple(str(value) for value in artifact["feature_names"].tolist())
    target_horizons = tuple(float(value) for value in artifact["target_horizons_s"].tolist())
    manual_training_rows = int(artifact["manual_training_rows"].reshape(-1)[0])
    if version != MODEL_VERSION:
      raise ValueError(f"unsupported trajectory model version {version}")
    if source != expected_source:
      raise ValueError(f"expected {expected_source} model, got {source}")
    if feature_names != TRAJECTORY_MODEL_FEATURE_NAMES:
      raise ValueError("trajectory model feature schema mismatch")
    if target_horizons != TARGET_HORIZONS_S:
      raise ValueError("trajectory target horizon schema mismatch")
    if manual_training_rows != 0:
      raise ValueError("manual labels are forbidden in the trajectory model")

    self.source = source
    self.threshold = float(artifact["entry_threshold"].reshape(-1)[0])
    self.exit_threshold = float(artifact["exit_threshold"].reshape(-1)[0])
    self.feature_mean = artifact["feature_mean"].astype(np.float32)
    self.feature_std = artifact["feature_std"].astype(np.float32)
    self.w1 = artifact["w1"].astype(np.float32)
    self.b1 = artifact["b1"].astype(np.float32)
    self.w2 = artifact["w2"].astype(np.float32)
    self.b2 = artifact["b2"].astype(np.float32)
    self.w3 = artifact["w3"].astype(np.float32)
    self.b3 = artifact["b3"].astype(np.float32)
    if self.w3.shape[1] != len(TARGET_HORIZONS_S) or self.b3.shape != (len(TARGET_HORIZONS_S),):
      raise ValueError("trajectory model output schema mismatch")

  def probabilities(self, matrix: np.ndarray) -> np.ndarray:
    normalized = (matrix.astype(np.float32) - self.feature_mean) / self.feature_std
    hidden1 = np.maximum(normalized @ self.w1 + self.b1, 0.0)
    hidden2 = np.maximum(hidden1 @ self.w2 + self.b2, 0.0)
    logits = hidden2 @ self.w3 + self.b3
    return 1.0 / (1.0 + np.exp(-np.clip(logits, -30.0, 30.0)))

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
    probabilities = self.probabilities(np.asarray(rows, dtype=np.float32))
    result = []
    for (point, trajectory), horizon_values in zip(candidates, probabilities, strict=True):
      values = tuple(float(value) for value in horizon_values)
      if not all(math.isfinite(value) for value in values):
        continue
      forward_horizon_relevant = tuple(
        min(
          trajectory.samples,
          key=lambda sample: abs(sample.horizon_s - horizon_s),
        ).d_rel > MIN_FORWARD_ENTRY_DREL_M
        for horizon_s in TARGET_HORIZONS_S
      )
      time_to_entry_s = trajectory.time_to_entry_s
      if time_to_entry_s is not None:
        entry_d_rel = (
          radar_point_value(point, "d_rel", "dRel")
          + radar_point_value(point, "v_rel", "vRel") * time_to_entry_s
        )
        if entry_d_rel <= MIN_FORWARD_ENTRY_DREL_M:
          forward_horizon_relevant = tuple(False for _ in TARGET_HORIZONS_S)
      current_path_occupancy = (
        abs(trajectory.d_path)
        <= trajectory.lane_half_width + VEHICLE_HALF_WIDTH_M
      )
      future_path_in_probability = max(
        (
          value
          for value, forward_relevant in zip(
            values, forward_horizon_relevant, strict=True,
          )
          if forward_relevant
        ),
        default=0.0,
      )
      path_in_probability = max(
        float(current_path_occupancy),
        future_path_in_probability,
      )
      path_out_probability = max(
        float(not current_path_occupancy),
        max((1.0 - value for value in values), default=0.0),
      )
      result.append(TrajectoryCutinPrediction(
        track_id=radar_point_track_id(point),
        source=radar_point_source(point),
        probability=path_in_probability,
        horizon_probabilities=values,
        trajectory=trajectory,
        point=point,
        path_exit_probability=path_out_probability,
        current_path_occupancy=current_path_occupancy,
        forward_horizon_relevant=forward_horizon_relevant,
      ))
    return tuple(result)


class RadarTrajectoryDecisionFilter:
  """Current-state gate plus probability hysteresis, with no scene exceptions."""

  def __init__(
    self,
    threshold: float,
    exit_threshold: float | None = None,
    hysteresis: float = DECISION_HYSTERESIS,
    transition_hold_s: float = MEASURED_TRANSITION_HOLD_S,
  ) -> None:
    self.threshold = float(threshold)
    self.exit_threshold = float(threshold if exit_threshold is None else exit_threshold)
    self.hysteresis = max(0.0, float(hysteresis))
    self.transition_hold_s = max(0.0, float(transition_hold_s))
    self._active_entry: set[tuple[str, int, int]] = set()
    self._active_exit: set[tuple[str, int, int]] = set()
    self._previous_occupancy: dict[tuple[str, int, int], bool] = {}
    self._measured_entry_time: dict[tuple[str, int, int], float] = {}
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
    seen = {self._identity(prediction) for prediction in predictions}
    for prediction in predictions:
      identity = self._identity(prediction)
      previous = self._previous_occupancy.get(identity)
      if previous is False and prediction.current_path_occupancy:
        self._measured_entry_time[identity] = time_s
      elif previous is True and not prediction.current_path_occupancy:
        self._measured_exit_time[identity] = time_s
      self._previous_occupancy[identity] = prediction.current_path_occupancy

    self._previous_occupancy = {
      identity: occupied
      for identity, occupied in self._previous_occupancy.items()
      if identity in seen
    }
    self._measured_entry_time = {
      identity: transition_time
      for identity, transition_time in self._measured_entry_time.items()
      if identity in seen and time_s - transition_time <= self.transition_hold_s
    }
    self._measured_exit_time = {
      identity: transition_time
      for identity, transition_time in self._measured_exit_time.items()
      if identity in seen and time_s - transition_time <= self.transition_hold_s
    }
    confirmed = [
      prediction for prediction in predictions
      if (
        self._identity(prediction) in self._measured_entry_time
        or (
          not prediction.current_path_occupancy
          and prediction.path_in_probability >= (
            entry_release
            if self._identity(prediction) in self._active_entry
            else self.threshold
          )
        )
      )
    ]
    exiting = [
      prediction for prediction in predictions
      if (
        self._identity(prediction) in self._measured_exit_time
        or (
          prediction.current_path_occupancy
          and prediction.path_out_probability >= (
            exit_release
            if self._identity(prediction) in self._active_exit
            else self.exit_threshold
          )
        )
      )
    ]
    self._active_entry = {self._identity(prediction) for prediction in confirmed}
    self._active_exit = {self._identity(prediction) for prediction in exiting}
    tentative: list[TrajectoryCutinPrediction] = []
    confirmed.sort(key=lambda item: (-item.path_in_probability, item.trajectory.time_to_entry_s or 0.0))
    exiting.sort(key=lambda item: -item.path_out_probability)
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
      decision = corner_decision if self.corner_radar_enabled else front_decision
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
