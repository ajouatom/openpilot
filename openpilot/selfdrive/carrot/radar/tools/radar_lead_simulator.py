#!/usr/bin/env python3
"""Offline radar lead-selection workbench for openpilot route logs.

This intentionally lives outside radard.  It aligns recorded radar inputs with
the recorded radarState output and runs a replaceable candidate selector over
the same frame.  The initial selector is only a transparent heuristic baseline;
it is the seam where a trained model can be added later.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import gzip
import json
import math
import os
import shutil
import sys
import threading
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, replace
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Protocol

REPO_ROOT = Path(__file__).resolve().parents[5]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

try:
  from openpilot.selfdrive.carrot.radar.radar_object import RadarObject
  from openpilot.selfdrive.carrot.radar.radar_object_fusion import RadarObjectFusion
except ModuleNotFoundError:
  from radar_object import RadarObject
  from radar_object_fusion import RadarObjectFusion

try:
  from openpilot.selfdrive.carrot.radar.radar_lead_model import (
    CUTIN_ACTIVE_CURRENT_MIN,
    CUTIN_TEMPORAL_THRESHOLD_MAX,
    RadarLeadContext,
    RadarLeadDecisionFilter,
    RadarLeadFeatureBuilder,
    RadarLeadModel,
    VisionLeadContext,
  )
except ModuleNotFoundError:
  from radar_lead_model import (
    CUTIN_ACTIVE_CURRENT_MIN,
    CUTIN_TEMPORAL_THRESHOLD_MAX,
    RadarLeadContext,
    RadarLeadDecisionFilter,
    RadarLeadFeatureBuilder,
    RadarLeadModel,
    VisionLeadContext,
  )

try:
  from openpilot.selfdrive.carrot.radar.radar_lead_controller import RadarLeadModelController
except ModuleNotFoundError:
  from radar_lead_controller import RadarLeadModelController

try:
  from openpilot.selfdrive.carrot.radar.radar_lead_runtime import RadarLeadRuntime
except ModuleNotFoundError:
  from radar_lead_runtime import RadarLeadRuntime

try:
  from openpilot.selfdrive.carrot.radar.radar_trajectory import (
    RadarTrajectory,
    RadarTrajectoryAnalyzer,
    TARGET_HORIZONS_S,
    ego_yaw_rate_rad_s,
    forward_probability,
    path_relative_state,
    trajectory_entry_probability,
    trajectory_sample_at,
    trajectory_is_review_candidate,
  )
except ModuleNotFoundError:
  from radar_trajectory import (
    RadarTrajectory,
    RadarTrajectoryAnalyzer,
    TARGET_HORIZONS_S,
    ego_yaw_rate_rad_s,
    forward_probability,
    path_relative_state,
    trajectory_entry_probability,
    trajectory_is_review_candidate,
    trajectory_sample_at,
  )

try:
  from openpilot.selfdrive.carrot.radar.radar_sensor_objects import (
    match_corner_to_front_identity,
  )
except ModuleNotFoundError:
  from radar_sensor_objects import match_corner_to_front_identity

try:
  from openpilot.selfdrive.carrot.radar.radar_vision_model_controller import (
    PRIMARY_STEALTH_HOLD_S,
    STEALTH_LEAD_HOLD_S,
    VisionModelRadarController,
    VisionRadarMatcher,
    cutin_is_ahead_of_primary,
  )
except ModuleNotFoundError:
  from radar_vision_model_controller import (
    PRIMARY_STEALTH_HOLD_S,
    STEALTH_LEAD_HOLD_S,
    VisionModelRadarController,
    VisionRadarMatcher,
    cutin_is_ahead_of_primary,
  )


RADAR_TO_CAMERA = 1.52
DEFAULT_FORWARD_RANGE_M = 100.0
RADAR_ROOT = Path(__file__).resolve().parents[1]
CARROT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_VALIDATION_CASES = CARROT_ROOT / "cluster" / "cutin_validation_cases.json"
DEFAULT_MULTITASK_MODEL = RADAR_ROOT / "models" / "radar_lead_multitask.npz"
DEFAULT_FRONT_MODEL = RADAR_ROOT / "models" / "radar_lead_front.npz"
DEFAULT_CORNER_MODEL = RADAR_ROOT / "models" / "radar_lead_corner.npz"
DEFAULT_REVIEW_PROBABILITY = 0.5
REVIEW_POINT_TEXT_MAX_DREL_M = 35.0
REVIEW_SETTINGS_ENV = "CARROT_RADAR_REVIEW_SETTINGS"
CURRENT_RADARD_CORNER_CENTER_MIN_AGE = 5
CURRENT_RADARD_CORNER_CENTER_UNMATCHED_MAX_DREL = 45.0
VALIDATION_EXPECTED_LABELS = ("detect", "clear", "stationary")


def _source_mode_name(source: str) -> str:
  return "corner" if source.startswith("corner") else "front"


@dataclass(frozen=True)
class RadarPoint:
  track_id: int
  d_rel: float
  y_rel: float
  v_rel: float
  a_rel: float
  yv_rel: float
  v_lead: float
  measured: bool
  source: str
  a_lead: float = 0.0
  j_lead: float = 0.0


@dataclass(frozen=True)
class ModelLead:
  probability: float
  x: float
  y: float
  v: float
  a: float
  x_std: float
  y_std: float
  v_std: float


@dataclass(frozen=True)
class RecordedLead:
  status: bool
  radar: bool
  track_id: int
  d_rel: float
  y_rel: float
  v_rel: float
  v_lead: float
  d_path: float
  model_prob: float
  score: float


@dataclass(frozen=True)
class RadarFrame:
  mono_time_s: float
  time_s: float
  input_age_s: float
  model_age_s: float
  v_ego: float
  points: tuple[RadarPoint, ...]
  path: tuple[tuple[float, float], ...]
  lane_lines: tuple[tuple[tuple[float, float], ...], ...]
  lane_probs: tuple[float, ...]
  model_leads: tuple[ModelLead, ...]
  recorded_one: RecordedLead
  recorded_two: RecordedLead
  video_time_s: float | None = None
  path_y_stds: tuple[tuple[float, float], ...] = ()
  lane_stds: tuple[float, ...] = ()
  steering_angle_deg: float = 0.0
  steering_rate_deg_s: float = 0.0
  yaw_rate_rad_s: float = 0.0
  yaw_rate_estimated: bool = False
  yaw_rate_source: str = "unknown"
  steer_ratio: float = 14.0
  wheelbase: float = 2.8


@dataclass(frozen=True)
class Candidate:
  track_id: int
  score: float
  reason: str
  decision_threshold: float = 0.0
  d_rel: float | None = None
  y_rel: float | None = None
  track_aliases: tuple[int, ...] = ()
  base_score: float | None = None
  temporal_score: float | None = None
  horizon_scores: tuple[float, ...] = ()
  out_horizon_scores: tuple[float, ...] = ()
  path_exit_score: float = 0.0
  path_exit_threshold: float = 0.0
  current_path_occupancy: bool = False
  stage: str = ""
  detail: str = ""
  source: str = ""
  horizon_x: tuple[float, ...] = ()
  horizon_y: tuple[float, ...] = ()
  horizon_x_stds: tuple[float, ...] = ()
  horizon_y_stds: tuple[float, ...] = ()

  @property
  def eligible(self) -> bool:
    return self.score >= self.decision_threshold

  @property
  def path_in_score(self) -> float:
    return self.score

  @property
  def path_out_score(self) -> float:
    return self.path_exit_score


def candidate_track_ids(candidate: Candidate | None) -> frozenset[int]:
  if candidate is None:
    return frozenset()
  return frozenset((candidate.track_id, *candidate.track_aliases))


def candidate_matches_targets(candidate: Candidate | None, targets: set[int]) -> bool:
  return candidate is not None and (not targets or bool(candidate_track_ids(candidate) & targets))


@dataclass(frozen=True)
class Selection:
  lead_one: Candidate | None
  lead_two: Candidate | None
  front_candidates: tuple[Candidate, ...] = ()
  corner_candidates: tuple[Candidate, ...] = ()
  cutin_diagnostics: tuple[Candidate, ...] = ()
  decision_cutin_candidates: tuple[Candidate, ...] = ()
  active_cutin_candidates: tuple[Candidate, ...] = ()
  external_candidates: tuple[Candidate, ...] = ()
  active_external_candidates: tuple[Candidate, ...] = ()
  lead_two_tentative: bool | None = None


@dataclass(frozen=True)
class ValidationReview:
  case_id: str
  expected: str
  source: str
  start_s: float
  end_s: float
  scene: str
  target_track_ids: tuple[int, ...] = ()
  forbidden_lead_two_ids: tuple[int, ...] = ()
  validation_stage: str = "output"
  human_verified: bool = False


@dataclass(frozen=True)
class CutinContinuity:
  track_id: int
  matched_frames: int
  episode_frames: int
  drop_runs: int
  max_drop_s: float
  current_drop_s: float
  active: bool


@dataclass(frozen=True)
class LeadPlotValue:
  track_id: int
  d_rel: float


@dataclass(frozen=True)
class LeadComparisonFrame:
  radard_one: LeadPlotValue | None
  radard_two: LeadPlotValue | None
  model_one: LeadPlotValue | None
  model_two: LeadPlotValue | None


@dataclass(frozen=True)
class CutinStageFrame:
  model: Candidate | None
  path_out: Candidate | None
  decision: Candidate | None
  output: Candidate | None
  selected: Candidate | None


@dataclass
class _TrajectoryReviewEpisode:
  sensor: str
  last_time_s: float
  d_rel: float
  y_rel: float
  v_rel: float
  yv_rel: float
  track_keys: set[tuple[str, int]]

  def update(self, time_s: float, key: tuple[str, int], point: RadarPoint) -> None:
    self.last_time_s = time_s
    self.d_rel = point.d_rel
    self.y_rel = point.y_rel
    self.v_rel = point.v_rel
    self.yv_rel = point.yv_rel
    self.track_keys.add(key)


def _trajectory_episode_cost(
  episode: _TrajectoryReviewEpisode,
  time_s: float,
  key: tuple[str, int],
  point: RadarPoint,
) -> float | None:
  source = key[0]
  sensor = "corner" if source.startswith("corner") else "front"
  age_s = time_s - episode.last_time_s
  if sensor != episode.sensor or age_s < -1e-3 or age_s > 2.0:
    return None
  if key in episode.track_keys:
    return -1.0
  predicted_d = episode.d_rel + episode.v_rel * age_s
  predicted_y = episode.y_rel + episode.yv_rel * age_s
  d_error = abs(point.d_rel - predicted_d)
  y_error = abs(point.y_rel - predicted_y)
  v_error = abs(point.v_rel - episode.v_rel)
  d_limit = min(6.0, max(2.0, 0.06 * max(point.d_rel, episode.d_rel)))
  if d_error > d_limit or y_error > 1.15 or v_error > 4.0:
    return None
  return d_error / d_limit + y_error / 1.15 + v_error / 4.0


def radar_trajectory_series(
  frames: Iterable[RadarFrame],
) -> tuple[dict[tuple[str, int], RadarTrajectory], ...]:
  analyzer = RadarTrajectoryAnalyzer()
  values: list[dict[tuple[str, int], RadarTrajectory]] = []
  for frame in frames:
    values.append(analyzer.update(
      frame.time_s,
      frame.points,
      frame.path,
      frame.lane_lines,
      frame.lane_probs,
      frame.path_y_stds,
      frame.lane_stds,
      frame.steering_angle_deg,
      frame.steering_rate_deg_s,
      frame.yaw_rate_rad_s,
      frame.yaw_rate_estimated,
    ))
  return tuple(values)


def trajectory_review_events(
  frames: Sequence[RadarFrame],
  trajectories: Sequence[dict[tuple[str, int], RadarTrajectory]],
  sources: Iterable[str],
  horizon_s: float = 1.0,
  probability_threshold: float = 0.60,
) -> dict[int, tuple[str, ...]]:
  requested = {source.lower() for source in sources}
  want_corner = any("corner" in source for source in requested)
  want_front = any("front" in source for source in requested)
  events: dict[int, tuple[str, ...]] = {}
  episodes: list[_TrajectoryReviewEpisode] = []
  for index, (frame, frame_trajectories) in enumerate(zip(frames, trajectories, strict=True)):
    has_corner = any(source.startswith("corner") for source, _ in frame_trajectories)
    use_corner = want_corner and has_corner
    eligible_points: dict[tuple[str, int], RadarPoint] = {}
    current: dict[tuple[str, int], RadarTrajectory] = {}
    for key, trajectory in frame_trajectories.items():
      source, track_id = key
      source_matches = (
        source.startswith("corner")
        if use_corner
        else source == "frontRadar" and (want_front or not want_corner)
      )
      point = next(
        (item for item in frame.points if item.source == source and item.track_id == track_id),
        None,
      )
      if source_matches and point is not None:
        eligible_points[key] = point
      if (
        source_matches
        and point is not None
        and trajectory_is_review_candidate(
          trajectory, point.d_rel, horizon_s, probability_threshold,
        )
      ):
        current[key] = trajectory

    episodes = [
      episode for episode in episodes
      if frame.time_s - episode.last_time_s <= 2.0
    ]
    point_episode: dict[tuple[str, int], _TrajectoryReviewEpisode] = {}
    associations = sorted(
      (
        (cost, episode_index, key)
        for episode_index, episode in enumerate(episodes)
        for key, point in eligible_points.items()
        if (cost := _trajectory_episode_cost(episode, frame.time_s, key, point)) is not None
      ),
      key=lambda item: item[0],
    )
    used_episodes: set[int] = set()
    used_points: set[tuple[str, int]] = set()
    for _, episode_index, key in associations:
      if episode_index in used_episodes or key in used_points:
        continue
      episode = episodes[episode_index]
      episode.update(frame.time_s, key, eligible_points[key])
      point_episode[key] = episode
      used_episodes.add(episode_index)
      used_points.add(key)

    labels: list[str] = []
    for key, trajectory in sorted(current.items()):
      source, track_id = key
      point = eligible_points[key]
      episode = point_episode.get(key)
      if episode is None:
        episode = next(
          (
            candidate for candidate in episodes
            if _trajectory_episode_cost(candidate, frame.time_s, key, point) is not None
          ),
          None,
        )
      if episode is not None:
        episode.update(frame.time_s, key, point)
        continue
      sensor_name = "corner" if source.startswith("corner") else "front"
      episodes.append(_TrajectoryReviewEpisode(
        sensor_name, frame.time_s, point.d_rel, point.y_rel, point.v_rel, point.yv_rel, {key},
      ))
      sensor_label = sensor_name.upper()
      tte = trajectory.time_to_entry_s if trajectory.time_to_entry_s is not None else math.inf
      probability = trajectory_entry_probability(trajectory, horizon_s)
      labels.append(
        f"TRAJECTORY {sensor_label} id {track_id} p{probability:.2f} t{tte:.2f}s"
      )
    if labels:
      events[index] = tuple(labels)
  return events


def trajectory_model_review_events(
  frames: Sequence[RadarFrame],
  selector: LeadSelector,
  sources: Iterable[str],
  probability_threshold: float,
) -> dict[int, tuple[str, ...]]:
  """Create one pause event per physical object from the displayed model probability."""
  requested = {source.lower() for source in sources}
  want_corner = any("corner" in source for source in requested)
  want_front = any("front" in source for source in requested)
  events: dict[int, tuple[str, ...]] = {}
  episodes: list[_TrajectoryReviewEpisode] = []

  def mode_for(candidate: Candidate) -> str | None:
    reason = candidate.reason.lower()
    if "trajectory corner" in reason:
      return "corner"
    if "trajectory front" in reason:
      return "front"
    return None

  def source_matches(point: RadarPoint, mode: str) -> bool:
    return point.source.startswith("corner") if mode == "corner" else point.source in ("frontRadar", "scc")

  for index, frame in enumerate(frames):
    selection = selector.select(frame, index)
    eligible: dict[tuple[str, int], tuple[RadarPoint, Candidate, str]] = {}
    for candidate in selection.cutin_diagnostics:
      if candidate.stage == "BLOCK-LEAD":
        continue
      mode = mode_for(candidate)
      if mode is None:
        continue
      if mode == "corner" and not want_corner:
        continue
      if mode == "front" and not (want_front or not want_corner):
        continue
      ids = candidate_track_ids(candidate)
      matching_points = [
        point for point in frame.points
        if point.measured and point.track_id in ids and source_matches(point, mode)
      ]
      if not matching_points:
        continue
      point = min(
        matching_points,
        key=lambda value: (
          abs(value.d_rel - candidate.d_rel) if candidate.d_rel is not None else 0.0,
          abs(value.y_rel - candidate.y_rel) if candidate.y_rel is not None else 0.0,
        ),
      )
      eligible[(point.source, point.track_id)] = (point, candidate, mode)

    episodes = [
      episode for episode in episodes
      if frame.time_s - episode.last_time_s <= 2.0
    ]
    point_episode: dict[tuple[str, int], _TrajectoryReviewEpisode] = {}
    associations = sorted(
      (
        (cost, episode_index, key)
        for episode_index, episode in enumerate(episodes)
        for key, (point, _, _) in eligible.items()
        if (cost := _trajectory_episode_cost(episode, frame.time_s, key, point)) is not None
      ),
      key=lambda item: item[0],
    )
    used_episodes: set[int] = set()
    used_points: set[tuple[str, int]] = set()
    for _, episode_index, key in associations:
      if episode_index in used_episodes or key in used_points:
        continue
      episode = episodes[episode_index]
      point, _, _ = eligible[key]
      episode.update(frame.time_s, key, point)
      point_episode[key] = episode
      used_episodes.add(episode_index)
      used_points.add(key)

    labels: list[str] = []
    for key, (point, candidate, mode) in sorted(eligible.items()):
      if candidate.score < probability_threshold:
        continue
      episode = point_episode.get(key)
      if episode is None:
        episode = next(
          (
            value for value in episodes
            if _trajectory_episode_cost(value, frame.time_s, key, point) is not None
          ),
          None,
        )
      if episode is not None:
        episode.update(frame.time_s, key, point)
        continue
      episodes.append(_TrajectoryReviewEpisode(
        mode, frame.time_s, point.d_rel, point.y_rel, point.v_rel, point.yv_rel, {key},
      ))
      raw_in = (
        " I " + "/".join(f"{value:.2f}" for value in candidate.horizon_scores)
        if candidate.horizon_scores else ""
      )
      raw_out = (
        " O " + "/".join(f"{value:.2f}" for value in candidate.out_horizon_scores)
        if candidate.out_horizon_scores else ""
      )
      labels.append(
        f"TRAJECTORY MODEL {mode.upper()} id {point.track_id} "
        + f"IN{candidate.score:.2f} OUT{candidate.path_exit_score:.2f}{raw_in}{raw_out}"
      )
    if labels:
      events[index] = tuple(labels)
  return events


def default_review_settings_path() -> Path:
  override = os.environ.get(REVIEW_SETTINGS_ENV)
  if override:
    return Path(override)
  config_root = os.environ.get("LOCALAPPDATA") or os.environ.get("XDG_CONFIG_HOME")
  base = Path(config_root) if config_root else Path.home() / ".config"
  return base / "carrotpilot" / "radar_lead_review.json"


def load_review_probability(
  path: Path | None = None,
  default: float = DEFAULT_REVIEW_PROBABILITY,
) -> float:
  settings_path = path or default_review_settings_path()
  try:
    payload = json.loads(settings_path.read_text(encoding="utf-8"))
    value = float(payload["probability"])
  except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
    return default
  return min(max(value, 0.0), 1.0) if math.isfinite(value) else default


def save_review_probability(probability: float, path: Path | None = None) -> bool:
  settings_path = path or default_review_settings_path()
  value = min(max(float(probability), 0.0), 1.0)
  try:
    settings_path.parent.mkdir(parents=True, exist_ok=True)
    payload: dict[str, Any] = {}
    if settings_path.is_file():
      try:
        loaded = json.loads(settings_path.read_text(encoding="utf-8"))
        if isinstance(loaded, dict):
          payload.update(loaded)
      except (OSError, json.JSONDecodeError):
        pass
    payload["probability"] = round(value, 4)
    temporary = settings_path.with_name(settings_path.name + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    temporary.replace(settings_path)
    return True
  except OSError:
    return False


def _candidate_plot_value(frame: RadarFrame, candidate: Candidate | None) -> LeadPlotValue | None:
  if candidate is None:
    return None
  distance = candidate.d_rel
  if distance is None:
    point = next((point for point in frame.points if point.track_id == candidate.track_id), None)
    distance = point.d_rel if point is not None else None
  if distance is None or not math.isfinite(distance) or distance <= 0.0:
    return None
  return LeadPlotValue(candidate.track_id, float(distance))


def _highest_score(candidates: Iterable[Candidate]) -> Candidate | None:
  return max(candidates, key=lambda candidate: candidate.score, default=None)


def cutin_stage_series(
  frames: Iterable[RadarFrame],
  selector: LeadSelector,
) -> tuple[CutinStageFrame, ...]:
  values: list[CutinStageFrame] = []
  for index, frame in enumerate(frames):
    selection = selector.select(frame, index)
    diagnostics = selection.cutin_diagnostics or selection.corner_candidates
    outside = tuple(
      candidate for candidate in diagnostics
      if not candidate.current_path_occupancy
    )
    inside = tuple(
      candidate for candidate in diagnostics
      if candidate.current_path_occupancy
    )
    selected = (
      selection.lead_two
      if selection.lead_two is not None and "cutin" in selection.lead_two.reason.lower()
      else None
    )
    values.append(CutinStageFrame(
      model=_highest_score(outside),
      path_out=max(inside, key=lambda candidate: candidate.path_out_score, default=None),
      decision=_highest_score(selection.decision_cutin_candidates),
      output=_highest_score(selection.active_cutin_candidates),
      selected=selected,
    ))
  return tuple(values)


def preferred_radar_points(frame: RadarFrame, review_source: str | None) -> tuple[RadarPoint, ...]:
  if review_source is None:
    return frame.points
  corner = tuple(point for point in frame.points if point.source.startswith("corner"))
  front = tuple(point for point in frame.points if point.source == "frontRadar")
  scc = tuple(point for point in frame.points if point.source == "scc")
  normalized_source = review_source.lower()
  if "front" in normalized_source and "corner" in normalized_source:
    return front + corner if front or corner else scc
  if "corner" in normalized_source and corner:
    return corner
  if front:
    return front
  if scc:
    return scc
  return corner


def lead_comparison_series(
  frames: list[RadarFrame], radard_selector: LeadSelector | None, model_selector: LeadSelector,
) -> tuple[LeadComparisonFrame, ...]:
  """Return model and optional current-radard control-facing distance histories."""
  result = []
  for index, frame in enumerate(frames):
    radard = radard_selector.select(frame, index) if radard_selector is not None else Selection(None, None)
    model = model_selector.select(frame, index)
    result.append(LeadComparisonFrame(
      radard_one=_candidate_plot_value(frame, radard.lead_one),
      radard_two=_candidate_plot_value(frame, radard.lead_two),
      model_one=_candidate_plot_value(frame, model.lead_one),
      model_two=_candidate_plot_value(frame, model.lead_two),
    ))
  return tuple(result)


def cutin_continuity_series(
  frames: list[RadarFrame], selector: LeadSelector, bridge_gap_s: float = 1.0, retain_s: float = 2.0,
) -> tuple[CutinContinuity | None, ...]:
  times = [frame.time_s for frame in frames]
  active_ids: list[int | None] = []
  indices_by_id: dict[int, list[int]] = {}
  for index, frame in enumerate(frames):
    lead_two = selector.select(frame, index).lead_two
    track_id = (
      lead_two.track_id
      if lead_two is not None and lead_two.reason.endswith(
        ("active cutin", "tentative cutin", "confirmed cutin"),
      )
      else None
    )
    active_ids.append(track_id)
    if track_id is not None:
      indices_by_id.setdefault(track_id, []).append(index)

  snapshots: list[CutinContinuity | None] = [None] * len(frames)
  for track_id, active_indices in indices_by_id.items():
    episodes: list[list[int]] = []
    for index in active_indices:
      if not episodes or frames[index].time_s - frames[episodes[-1][-1]].time_s > bridge_gap_s:
        episodes.append([index])
      else:
        episodes[-1].append(index)

    for episode in episodes:
      start, end = episode[0], episode[-1]
      matched = 0
      drop_runs = 0
      drop_started_at: float | None = None
      max_drop_s = 0.0
      for index in range(start, end + 1):
        active = active_ids[index] == track_id
        matched += int(active)
        if not active:
          if drop_started_at is None:
            drop_started_at = frames[index].time_s
            drop_runs += 1
          frame_dt = frames[index].time_s - frames[index - 1].time_s if index > 0 else 0.05
          current_drop_s = frames[index].time_s - drop_started_at + max(frame_dt, 0.0)
          max_drop_s = max(max_drop_s, current_drop_s)
        else:
          drop_started_at = None
          current_drop_s = 0.0
        snapshot = CutinContinuity(
          track_id, matched, index - start + 1, drop_runs, max_drop_s, current_drop_s, active,
        )
        if snapshots[index] is None or active:
          snapshots[index] = snapshot

      final_snapshot = snapshots[end]
      if final_snapshot is not None:
        retain_end = bisect.bisect_right(times, frames[end].time_s + retain_s)
        for index in range(end + 1, retain_end):
          if snapshots[index] is None:
            snapshots[index] = final_snapshot
  return tuple(snapshots)


def validation_review_events(
  frames: list[RadarFrame], selector: LeadSelector, review: ValidationReview | None,
) -> dict[int, tuple[str, ...]]:
  events_by_frame: dict[int, tuple[str, ...]] = {}
  previous_cutins: set[int] = set()
  stationary_event_seen = False
  previous_vision_lead = False
  previous_vision_unmatched = False
  previous_lead_one = False
  previous_lead_two_only = False
  previous_forbidden_lead_two: int | None = None
  last_cutin_event_time: dict[int, float] = {}
  for index, frame in enumerate(frames):
    selection = selector.select(frame, index)
    events: list[str] = []
    if review is not None and review.expected == "stationary":
      if stationary_event_seen:
        continue
      targets = set(review.target_track_ids)
      for role, candidate in (("leadOne", selection.lead_one), ("leadTwo", selection.lead_two)):
        if candidate is None or (targets and candidate.track_id not in targets):
          continue
        stopped_points = [
          point for point in frame.points
          if point.track_id == candidate.track_id
          and 0.8 < point.d_rel < 130.0
          and abs(point.v_lead) * 3.6 < 3.0
        ]
        if stopped_points:
          distance = min(point.d_rel for point in stopped_points)
          events.append(f"STATIONARY {role} id {candidate.track_id} {distance:.0f}m")
          stationary_event_seen = True
          break
      if events:
        events_by_frame[index] = tuple(events)
      continue

    primary_track_id = selection.lead_one.track_id if selection.lead_one is not None else None
    current_cutins = set()
    if (
      selection.lead_two is not None
      and selection.lead_two.track_id != primary_track_id
      and selection.lead_two.reason.endswith(
        ("active cutin", "tentative cutin", "confirmed cutin"),
      )
      and (
        selection.lead_two.score >= CUTIN_ACTIVE_CURRENT_MIN
        or selection.lead_two.track_id in previous_cutins
      )
    ):
      current_cutins.add(selection.lead_two.track_id)
    new_cutins = {
      track_id for track_id in current_cutins - previous_cutins
      if frame.time_s - last_cutin_event_time.get(track_id, -math.inf) >= 1.0
    }
    if new_cutins:
      for track_id in sorted(new_cutins):
        lead_two_matches = (
          selection.lead_two is not None
          and track_id in candidate_track_ids(selection.lead_two)
        )
        if lead_two_matches and selection.lead_two_tentative is not None:
          state = "TENTATIVE" if selection.lead_two_tentative else "CONFIRMED"
          events.append(f"CUT-IN {state} id {track_id}")
        else:
          events.append(f"CUT-IN id {track_id}")
      last_cutin_event_time.update((track_id, frame.time_s) for track_id in new_cutins)
    previous_cutins = current_cutins

    forbidden_lead_two = None
    if review is not None and selection.lead_two is not None:
      forbidden_ids = set(review.forbidden_lead_two_ids)
      if selection.lead_two.track_id in forbidden_ids:
        forbidden_lead_two = selection.lead_two.track_id
        if forbidden_lead_two != previous_forbidden_lead_two:
          events.append(f"FALSE leadTwo id {forbidden_lead_two}")
    previous_forbidden_lead_two = forbidden_lead_two

    # A maintained cut-in validation case must pause only for the decision it
    # is labeling. Lead continuity diagnostics remain visible in the UI, but
    # must not interrupt CUT-IN review playback.
    if review is not None:
      if events:
        events_by_frame[index] = tuple(events)
      continue

    current_vision_lead = selection.lead_one is not None and selection.lead_one.track_id < 0
    if current_vision_lead and not previous_vision_lead:
      events.append("leadOne VISION")
    previous_vision_lead = current_vision_lead

    model_leads = getattr(frame, "model_leads", ())
    vision_expected = bool(model_leads and model_leads[0].probability > 0.5)
    current_lead_one = selection.lead_one is not None
    current_lead_two = selection.lead_two is not None
    if vision_expected and previous_lead_two_only and not current_lead_one and not current_lead_two:
      events.append("leadTwo LOST (VISION)")
    current_vision_unmatched = vision_expected and not current_lead_one
    if current_vision_unmatched and previous_lead_one:
      events.append("leadOne LOST (VISION)")
    elif current_vision_unmatched and not previous_vision_unmatched:
      events.append("VISION UNMATCHED")
    previous_vision_unmatched = current_vision_unmatched
    previous_lead_one = current_lead_one
    previous_lead_two_only = current_lead_two and not current_lead_one

    if events:
      events_by_frame[index] = tuple(events)
  return events_by_frame


def validation_log_review_events(
  frames: list[RadarFrame],
  selector: LeadSelector,
  reviews: Sequence[ValidationReview],
) -> dict[int, tuple[str, ...]]:
  if not reviews:
    return {}

  merged: dict[int, list[str]] = {}

  def merge_events(values: dict[int, tuple[str, ...]], prefix: str = "") -> None:
    for frame_index, frame_events in values.items():
      bucket = merged.setdefault(frame_index, [])
      for event in frame_events:
        value = prefix + event
        if value not in bucket:
          bucket.append(value)

  generic = ValidationReview(
    "__log__", "detect", reviews[0].source, 0.0, frames[-1].time_s, "full log",
  )
  merge_events(validation_review_events(frames, selector, generic))
  for review in reviews:
    start_index = bisect.bisect_left([frame.time_s for frame in frames], review.start_s)
    end_index = bisect.bisect_right([frame.time_s for frame in frames], review.end_s)
    targets = set(review.target_track_ids)
    if review.expected == "stationary":
      for frame_index in range(start_index, end_index):
        frame = frames[frame_index]
        selection = selector.select(frame, frame_index)
        found = False
        for role, candidate in (("leadOne", selection.lead_one), ("leadTwo", selection.lead_two)):
          if candidate is None or (targets and not candidate_matches_targets(candidate, targets)):
            continue
          point = next(
            (
              item for item in frame.points
              if item.track_id in candidate_track_ids(candidate)
              and 0.8 < item.d_rel < 130.0
              and abs(item.v_lead) * 3.6 < 3.0
            ),
            None,
          )
          if point is not None:
            merge_events({
              frame_index: (
                f"{review.case_id}: STATIONARY {role} id {candidate.track_id} {point.d_rel:.0f}m",
              ),
            })
            found = True
            break
        if found:
          break

    forbidden = set(review.forbidden_lead_two_ids)
    previous_forbidden: int | None = None
    for frame_index in range(start_index, end_index):
      selection = selector.select(frames[frame_index], frame_index)
      current = (
        selection.lead_two.track_id
        if selection.lead_two is not None and selection.lead_two.track_id in forbidden
        else None
      )
      if current is not None and current != previous_forbidden:
        merge_events({
          frame_index: (f"{review.case_id}: FALSE leadTwo id {current}",),
        })
      previous_forbidden = current

  return {index: tuple(values) for index, values in merged.items()}


def qcamera_path_for_log(log_path: Path) -> Path:
  return log_path.parent / "qcamera.ts"


def play_review_alert(events: tuple[str, ...]) -> None:
  """Play an audible lead/cut-in cue without blocking the replay renderer."""
  def play() -> None:
    if sys.platform == "win32":
      try:
        import winsound
        if any("CUT-IN" in event or "TRAJECTORY" in event for event in events):
          winsound.Beep(1400, 140)
          winsound.Beep(1900, 180)
        else:
          winsound.Beep(1050, 130)
          winsound.Beep(1350, 150)
        return
      except (ImportError, RuntimeError):
        pass
    print("\a", end="", flush=True)

  threading.Thread(target=play, name="radar-review-alert", daemon=True).start()


LABEL_ROLES = ("leadOne", "leadTwo", "cutin")
HISTORY_FRAME_OFFSETS = (0, 1, 2, 4, 8)
MLP_CANDIDATE_FLOOR = 0.15


class ManualLabels:
  """Sparse manual overrides. Missing means use the recorded weak label; None means no lead."""

  def __init__(self, values: dict[int, dict[str, int | None]] | None = None) -> None:
    self.values = values or {}

  @classmethod
  def load(cls, path: Path, frame_count: int) -> ManualLabels:
    if not path.is_file():
      return cls()
    with path.open("r", encoding="utf-8") as source:
      payload = json.load(source)
    values: dict[int, dict[str, int | None]] = {}
    for entry in payload.get("labels", []):
      frame_index = int(entry.get("frame", -1))
      if not 0 <= frame_index < frame_count:
        continue
      overrides: dict[str, int | None] = {}
      for role in LABEL_ROLES:
        if role in entry:
          value = entry[role]
          overrides[role] = None if value is None else int(value)
      if overrides:
        values[frame_index] = overrides
    return cls(values)

  def get(self, frame_index: int, role: str) -> tuple[bool, int | None]:
    frame_labels = self.values.get(frame_index, {})
    return (role in frame_labels, frame_labels.get(role))

  def set(self, frame_index: int, role: str, track_id: int | None) -> None:
    if role not in LABEL_ROLES:
      raise ValueError(f"unknown label role: {role}")
    self.values.setdefault(frame_index, {})[role] = track_id

  def clear(self, frame_index: int, role: str) -> None:
    frame_labels = self.values.get(frame_index)
    if frame_labels is None:
      return
    frame_labels.pop(role, None)
    if not frame_labels:
      self.values.pop(frame_index, None)

  def fill(self, first: int, last: int, role: str, track_id: int | None) -> None:
    for frame_index in range(min(first, last), max(first, last) + 1):
      self.set(frame_index, role, track_id)

  def count(self) -> int:
    return sum(len(frame_labels) for frame_labels in self.values.values())

  def save(self, path: Path, log_path: Path, frames: list[RadarFrame]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    entries = []
    for frame_index, frame_labels in sorted(self.values.items()):
      entry: dict[str, Any] = {
        "frame": frame_index,
        "time_s": round(frames[frame_index].time_s, 6),
      }
      entry.update(frame_labels)
      entries.append(entry)
    payload = {
      "version": 1,
      "log": str(log_path),
      "frame_count": len(frames),
      "labels": entries,
    }
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as output:
      json.dump(payload, output, indent=2, ensure_ascii=True)
      output.write("\n")
    temporary.replace(path)


class LeadSelector(Protocol):
  name: str

  def select(self, frame: RadarFrame, frame_index: int | None = None) -> Selection:
    ...


class SimpleLeadSelector:
  """Readable starting point, not a replacement for radard or a trained model."""

  name = "simple-v0"

  @staticmethod
  def _usable_points(frame: RadarFrame) -> list[RadarPoint]:
    return [
      point for point in frame.points
      if 0.75 < point.d_rel < 160.0 and abs(point.y_rel) < 12.0
    ]

  @staticmethod
  def _absolute_speed(point: RadarPoint, v_ego: float) -> float:
    derived = v_ego + point.v_rel
    if abs(point.v_lead) < 0.01 and abs(derived) > 1.0:
      return derived
    return point.v_lead

  def _vision_candidate(self, frame: RadarFrame, points: Iterable[RadarPoint]) -> Candidate | None:
    if not frame.model_leads or frame.model_leads[0].probability <= 0.4:
      return None

    lead = frame.model_leads[0]
    target_d = lead.x - RADAR_TO_CAMERA
    target_y = -lead.y
    if target_d <= 0.0:
      return None

    distance_gate = max(7.0, target_d * 0.35)
    candidates: list[Candidate] = []
    for point in points:
      d_error = abs(point.d_rel - target_d)
      y_error = abs(point.y_rel - target_y)
      if d_error > distance_gate or y_error > 4.0:
        continue

      d_scale = max(2.5, lead.x_std, target_d * 0.08)
      y_scale = max(0.7, lead.y_std)
      v_scale = max(3.0, lead.v_std)
      v_error = abs(self._absolute_speed(point, frame.v_ego) - lead.v)
      source_penalty = 0.25 if point.source.startswith("corner") else 0.0
      score = d_error / d_scale + y_error / y_scale + v_error / v_scale + source_penalty
      candidates.append(Candidate(point.track_id, score, "model lead match"))

    return min(candidates, key=lambda candidate: candidate.score, default=None)

  @staticmethod
  def _path_y(frame: RadarFrame, distance: float) -> float:
    return model_line_y(frame.path, distance)

  def _secondary_candidate(
    self,
    frame: RadarFrame,
    points: Iterable[RadarPoint],
    excluded_track_id: int | None,
  ) -> Candidate | None:
    candidates: list[Candidate] = []
    for point in points:
      if point.track_id == excluded_track_id or point.d_rel > 100.0:
        continue
      d_path = point.y_rel - self._path_y(frame, point.d_rel)
      if abs(d_path) > 2.1:
        continue
      source_penalty = 0.0 if point.source.startswith("corner") else 0.20
      score = abs(d_path) / 0.8 + point.d_rel / 80.0 + source_penalty
      candidates.append(Candidate(point.track_id, score, "path corridor"))

    best = min(candidates, key=lambda candidate: candidate.score, default=None)
    return best if best is not None and best.score <= 2.8 else None

  def select(self, frame: RadarFrame, frame_index: int | None = None) -> Selection:
    points = self._usable_points(frame)
    lead_one = self._vision_candidate(frame, points)
    lead_two = self._secondary_candidate(
      frame,
      points,
      lead_one.track_id if lead_one is not None else None,
    )
    return Selection(lead_one, lead_two)


@dataclass
class TeacherTrackState:
  age: int = 0
  selected_count: int = 0
  stopped_count: int = 0
  d_path: float = 0.0
  in_lane_prob: float = 0.0
  lane_half_width: float = 1.8
  d_rel: float | None = None
  y_rel: float | None = None
  v_lead: float | None = None


class CurrentRadardTeacher:
  """Pure-Python source port of current radard lead selection for offline labels."""

  name = "current-radard-source-v2-can-corner-front-cutin"

  def __init__(self, frames: list[RadarFrame], cutin_ids: list[set[int]] | None = None) -> None:
    self.frames = frames
    self.cutin_ids = cutin_ids or [set() for _ in frames]
    if len(self.cutin_ids) != len(frames):
      raise ValueError("cutin ID frames must align with radar frames")
    self.selections = self._precompute()

  @staticmethod
  def _is_corner(point: RadarPoint) -> bool:
    return point.source.startswith("corner")

  @staticmethod
  def _laplacian(value: float, mean: float, scale: float) -> float:
    scale = max(abs(scale), 1e-4)
    return math.exp(-abs(value - mean) / scale) / (2.0 * scale)

  @staticmethod
  def _lane_state(frame: RadarFrame, point: RadarPoint) -> tuple[float, float, float]:
    if len(frame.lane_lines) < 3 or not frame.lane_lines[1] or not frame.lane_lines[2]:
      return point.y_rel - model_line_y(frame.path, point.d_rel), 0.0, 1.8
    left_y = model_line_y(frame.lane_lines[1], point.d_rel)
    right_y = model_line_y(frame.lane_lines[2], point.d_rel)
    center_y = 0.5 * (left_y + right_y)
    lane_half_width = max(0.1, 0.5 * abs(right_y - left_y))
    d_path = point.y_rel - center_y
    in_lane_prob = max(0.0, 1.0 - abs(d_path) / lane_half_width)
    return d_path, in_lane_prob, lane_half_width

  def _vision_match(
    self,
    frame: RadarFrame,
    points: list[RadarPoint],
    states: dict[int, TeacherTrackState],
    lead_prob: float,
  ) -> RadarPoint | None:
    if not frame.model_leads or lead_prob <= 0.4 or not points:
      return None
    lead = frame.model_leads[0]
    vision_d = lead.x - RADAR_TO_CAMERA
    min_d = max(vision_d * 0.80, 1.0)
    max_d = max(vision_d * 1.25, 5.0)
    min_d_wide = 1.5
    max_d_wide = max(vision_d * 1.45, 5.0)
    vel_tol = max(lead.v * (0.3 + clamp((lead_prob - 0.8) / 0.18, 0.0, 1.0) * 0.2), 5.0)
    vel_guard = max(vel_tol * 3.0, 20.0)

    def velocity_sane(point: RadarPoint) -> bool:
      delta = abs(point.v_lead - lead.v)
      if delta < vel_tol:
        return True
      return point.v_lead > 3.0 and delta <= vel_guard and states[point.track_id].in_lane_prob >= 0.25

    scored: list[tuple[float, float, RadarPoint]] = []
    for point in points:
      score = (
        self._laplacian(point.d_rel, vision_d, lead.x_std)
        * self._laplacian(point.y_rel, -lead.y, lead.y_std)
        * self._laplacian(point.v_lead, lead.v, lead.v_std)
      )
      wide_score = (
        self._laplacian(point.d_rel, vision_d, lead.x_std)
        * self._laplacian(point.y_rel, -lead.y, lead.y_std * 2.0)
        * self._laplacian(point.v_lead, lead.v, lead.v_std)
      )
      scored.append((score, wide_score, point))
    scored = [
      item for item in scored
      if abs(item[2].y_rel + lead.y) < 2.0 or abs(states[item[2].track_id].d_path) < 2.4
    ]
    if not scored:
      return None
    scored.sort(key=lambda item: item[0], reverse=True)
    first_score, _, first = scored[0]
    if first_score < 1e-4:
      return None
    second = scored[1][2] if len(scored) > 1 else None
    extra_score, extra = max((wide, point) for _, wide, point in scored)

    def distance_sane(point: RadarPoint, wide: bool = False) -> bool:
      return (min_d_wide < point.d_rel < max_d_wide) if wide else (min_d < point.d_rel < max_d)

    def lateral_sane(point: RadarPoint, wide: bool = False) -> bool:
      return abs(point.y_rel + lead.y) < (4.0 if wide else 2.0)

    best: RadarPoint | None = None
    if distance_sane(first) and velocity_sane(first):
      if (
        second is not None and velocity_sane(second) and states[second.track_id].in_lane_prob > 0.3
        and states[second.track_id].age > 5 and vision_d * 0.5 < second.d_rel < first.d_rel
      ):
        best = second
      elif lateral_sane(first) and lead_prob > 0.5:
        best = first
      elif lateral_sane(first) and lead_prob > 0.4 and states[first.track_id].selected_count > 0:
        best = first
      elif lead_prob > 0.6 and abs(states[first.track_id].d_path) < 2.4:
        best = first
    if best is None and distance_sane(first) and lateral_sane(first, wide=True):
      if second is not None and distance_sane(second) and lateral_sane(second) and velocity_sane(second):
        best = second
      elif states[first.track_id].selected_count > 0:
        best = first
      else:
        states[first.track_id].stopped_count += 2
        if states[first.track_id].stopped_count > 20:
          best = first
    if best is None and vision_d < 90.0 and lead_prob > 0.65:
      if extra_score > first_score and distance_sane(extra, True) and velocity_sane(extra) and lateral_sane(extra, True):
        best = extra
      elif distance_sane(first, True) and velocity_sane(first) and lateral_sane(first, True):
        best = first
      elif second is not None and distance_sane(second, True) and velocity_sane(second) and lateral_sane(second, True):
        best = second

    if best is not None:
      for point in points:
        state = states[point.track_id]
        if point.track_id == best.track_id:
          state.selected_count = min(state.selected_count + 1, 40)
        else:
          state.selected_count = 0
          state.stopped_count = max(0, state.stopped_count - 1)
    return best

  @classmethod
  def _center_candidate(
    cls,
    point: RadarPoint,
    state: TeacherTrackState,
    front_points: list[RadarPoint],
  ) -> bool:
    in_lane_min = 0.45 if point.d_rel > 60.0 else 0.30
    d_path_limit = 0.9 if point.d_rel > 60.0 else 1.2
    if state.in_lane_prob <= in_lane_min or abs(state.d_path) >= d_path_limit:
      return False
    if point.d_rel > 100.0:
      return False
    if cls._is_corner(point) and state.age < CURRENT_RADARD_CORNER_CENTER_MIN_AGE:
      return False
    if cls._is_corner(point) and point.d_rel > CURRENT_RADARD_CORNER_CENTER_UNMATCHED_MAX_DREL:
      matched_front = any(
        abs(front.d_rel - point.d_rel) <= 3.0 and abs(front.v_rel - point.v_rel) <= 2.0
        for front in front_points
      )
      if not matched_front:
        return False
    radar_only_limit = 0.75 if point.d_rel > 80.0 else (0.9 if point.d_rel > 60.0 else 1.1)
    return state.age > 3 and point.d_rel > 0.8 and point.v_lead > 2.0 and abs(state.d_path) < radar_only_limit

  @staticmethod
  def _stopped_candidate(point: RadarPoint, state: TeacherTrackState) -> bool:
    if not point.source.startswith("corner") or state.age < 7 or not 5.0 < point.d_rel < 120.0:
      return False
    if abs(point.v_lead) >= 1.8 or abs(point.yv_rel) >= 0.8:
      return False
    in_lane_min = 0.5 if point.d_rel > 60.0 else 0.35
    d_path_limit = 0.75 if point.d_rel > 60.0 else 1.0
    return state.in_lane_prob > in_lane_min and abs(state.d_path) < d_path_limit

  def _precompute(self) -> tuple[Selection, ...]:
    states: dict[int, TeacherTrackState] = {}
    lead_prob_filter = 0.0
    selections: list[Selection] = []
    for frame_index, frame in enumerate(self.frames):
      current_ids = {point.track_id for point in frame.points}
      for track_id in tuple(states):
        if track_id not in current_ids:
          states.pop(track_id)
      for point in frame.points:
        state = states.setdefault(point.track_id, TeacherTrackState())
        discontinuous = (
          state.d_rel is not None and (
            abs(point.d_rel - state.d_rel) > 5.0
            or abs(point.y_rel - state.y_rel) > 2.0
            or abs(point.v_lead - state.v_lead) > 7.0
          )
        )
        state.age = 1 if point.measured and discontinuous else state.age + 1 if point.measured else 0
        state.d_rel = point.d_rel
        state.y_rel = point.y_rel
        state.v_lead = point.v_lead
        state.d_path, state.in_lane_prob, state.lane_half_width = self._lane_state(frame, point)

      raw_prob = frame.model_leads[0].probability if frame.model_leads else 0.0
      lead_prob_filter = raw_prob if raw_prob > lead_prob_filter else lead_prob_filter + 0.2 * (raw_prob - lead_prob_filter)
      front_points = [
        point for point in frame.points
        if point.measured and states[point.track_id].age > 2 and not self._is_corner(point)
        and point.track_id != 0
      ]
      lead_one_point = self._vision_match(frame, front_points, states, lead_prob_filter)
      lead_one_id = lead_one_point.track_id if lead_one_point is not None else None

      external: RadarPoint | None = None
      cutin_points = [
        point for point in frame.points
        if point.track_id in self.cutin_ids[frame_index] and point.track_id != lead_one_id
      ]
      if cutin_points:
        external = min(cutin_points, key=lambda point: point.d_rel)
      else:
        stopped = [
          point for point in frame.points
          if point.track_id != lead_one_id and self._stopped_candidate(point, states[point.track_id])
        ]
        if stopped:
          external = min(stopped, key=lambda point: point.d_rel)
        elif len(frame.lane_probs) > 2 and frame.lane_probs[1] > 0.5 and frame.lane_probs[2] > 0.5:
          center = [
            point for point in frame.points
            if point.track_id != lead_one_id
            and self._center_candidate(point, states[point.track_id], front_points)
          ]
          if center:
            external = min(center, key=lambda point: point.d_rel)

      selections.append(Selection(
        Candidate(lead_one_id, 1.0, "current radard vision match") if lead_one_id is not None else None,
        Candidate(external.track_id, 1.0, "current radard external") if external is not None else None,
      ))
    return tuple(selections)

  def select(self, frame: RadarFrame, frame_index: int | None = None) -> Selection:
    if frame_index is None:
      raise ValueError("current radard teacher requires a frame index")
    return self.selections[frame_index]


def _finite(value: Any, default: float = 0.0) -> float:
  try:
    parsed = float(value)
  except (TypeError, ValueError):
    return default
  return parsed if math.isfinite(parsed) else default


def clamp(value: float, lower: float, upper: float) -> float:
  return max(lower, min(upper, value))


def model_line_y(points: tuple[tuple[float, float], ...], distance: float) -> float:
  """Interpolate model coordinates and return radar's positive-left convention."""
  if not points:
    return 0.0
  if len(points) == 1:
    return -points[0][1]
  xs = [point[0] for point in points]
  index = bisect.bisect_left(xs, distance)
  if index <= 0:
    return -points[0][1]
  if index >= len(points):
    return -points[-1][1]
  x0, y0 = points[index - 1]
  x1, y1 = points[index]
  ratio = 0.0 if x1 == x0 else (distance - x0) / (x1 - x0)
  return -(y0 + (y1 - y0) * ratio)


def trajectory_history_display_y(frame: RadarFrame, sample: Any) -> float:
  """Reproject an old path-relative observation onto the current path."""
  center_y, _, _, _, _ = path_relative_state(
    float(sample.d_rel),
    0.0,
    frame.path,
    frame.lane_lines,
    frame.lane_probs,
  )
  return center_y + float(sample.d_path)


def _route_replay_module() -> Any:
  script_path = Path(__file__).resolve()
  repo_root = script_path.parents[5]
  cluster_dir = script_path.parents[2] / "cluster"
  if str(repo_root) not in sys.path:
    sys.path.insert(0, str(repo_root))
  if str(cluster_dir) not in sys.path:
    sys.path.insert(0, str(cluster_dir))
  import cluster_route_replay
  return cluster_route_replay


def _copy_track_points(points: Iterable[Any]) -> tuple[RadarPoint, ...]:
  copied: list[RadarPoint] = []
  for point in points:
    track_id = int(point.trackId)
    source = str(point.radarSource)
    if source == "frontRadar":
      if 200 <= track_id < 220:
        source = "corner235"
      elif 240 <= track_id < 250:
        source = "corner180"
    copied.append(RadarPoint(
      track_id=track_id,
      d_rel=_finite(point.dRel),
      y_rel=_finite(point.yRel),
      v_rel=_finite(point.vRel),
      a_rel=_finite(point.aRel),
      yv_rel=_finite(point.yvRel),
      v_lead=_finite(point.vLead),
      measured=bool(point.measured),
      source=source,
      a_lead=_finite(getattr(point, "aLead", 0.0)),
      j_lead=_finite(getattr(point, "jLead", 0.0)),
    ))
  return tuple(copied)


def _copy_points(message: Any) -> tuple[RadarPoint, ...]:
  return _copy_track_points(message.points)


def _xy_points(data: Any) -> tuple[tuple[float, float], ...]:
  xs = data.x
  ys = data.y
  return tuple((_finite(xs[index]), _finite(ys[index])) for index in range(min(len(xs), len(ys))))


def _x_values(xs: Any, values: Any) -> tuple[tuple[float, float], ...]:
  return tuple(
    (_finite(xs[index]), max(0.0, _finite(values[index])))
    for index in range(min(len(xs), len(values)))
  )


def _copy_model(message: Any) -> tuple[
  tuple[tuple[float, float], ...],
  tuple[tuple[tuple[float, float], ...], ...],
  tuple[float, ...],
  tuple[ModelLead, ...],
  tuple[tuple[float, float], ...],
  tuple[float, ...],
]:
  path = _xy_points(message.position)
  lane_lines = tuple(_xy_points(line) for line in message.laneLines)
  lane_probs = tuple(_finite(probability) for probability in message.laneLineProbs)
  path_y_stds = _x_values(message.position.x, getattr(message.position, "yStd", ()))
  lane_stds = tuple(max(0.0, _finite(value)) for value in getattr(message, "laneLineStds", ()))
  leads: list[ModelLead] = []
  for lead in message.leadsV3:
    if not lead.x or not lead.y or not lead.v or not lead.a:
      continue
    leads.append(ModelLead(
      probability=_finite(lead.prob),
      x=_finite(lead.x[0]),
      y=_finite(lead.y[0]),
      v=_finite(lead.v[0]),
      a=_finite(lead.a[0]),
      x_std=_finite(lead.xStd[0], 1.0) if lead.xStd else 1.0,
      y_std=_finite(lead.yStd[0], 1.0) if lead.yStd else 1.0,
      v_std=_finite(lead.vStd[0], 1.0) if lead.vStd else 1.0,
    ))
  return path, lane_lines, lane_probs, tuple(leads), path_y_stds, lane_stds


def _copy_recorded_lead(lead: Any) -> RecordedLead:
  return RecordedLead(
    status=bool(lead.status),
    radar=bool(lead.radar),
    track_id=int(lead.radarTrackId),
    d_rel=_finite(lead.dRel),
    y_rel=_finite(lead.yRel),
    v_rel=_finite(lead.vRel),
    v_lead=_finite(lead.vLead),
    d_path=_finite(lead.dPath),
    model_prob=_finite(lead.modelProb),
    score=_finite(lead.score),
  )


def aligned_video_time_s(qcamera_start_eof_ns: int, model_eof_ns: int) -> float | None:
  if qcamera_start_eof_ns <= 0 or model_eof_ns < qcamera_start_eof_ns:
    return None
  return (model_eof_ns - qcamera_start_eof_ns) / 1e9


def load_frames(log_path: Path) -> list[RadarFrame]:
  route_replay = _route_replay_module()
  schema = route_replay.load_openpilot_log_schema()
  data = route_replay.read_log_bytes(log_path)

  latest_points: tuple[RadarPoint, ...] | None = None
  latest_points_ns = 0
  latest_v_ego = 0.0
  latest_steering_angle_deg = 0.0
  latest_steering_rate_deg_s = 0.0
  latest_live_pose: Any | None = None
  latest_live_pose_ns = 0
  latest_steer_ratio = 14.0
  latest_wheelbase = 2.8
  latest_path: tuple[tuple[float, float], ...] = ()
  latest_lanes: tuple[tuple[tuple[float, float], ...], ...] = ()
  latest_lane_probs: tuple[float, ...] = ()
  latest_model_leads: tuple[ModelLead, ...] = ()
  latest_path_y_stds: tuple[tuple[float, float], ...] = ()
  latest_lane_stds: tuple[float, ...] = ()
  latest_model_ns = 0
  latest_model_eof_ns = 0
  qcamera_start_eof_ns = 0
  absolute_frames: list[tuple[int, RadarFrame]] = []
  corner_tracker = route_replay.StableCornerObjectTracker()

  for event in schema.Event.read_multiple_bytes(data):
    try:
      which = event.which()
    except Exception:
      continue
    event_ns = int(event.logMonoTime)
    if which == "qRoadEncodeIdx" and qcamera_start_eof_ns == 0:
      qcamera_start_eof_ns = int(event.qRoadEncodeIdx.timestampEof)
    elif which == "liveTracks":
      event_t = event_ns / 1e9
      recorded_points = tuple(event.liveTracks.points)
      reconstructed = corner_tracker.live_tracks_at(event_t, latest_v_ego)
      merged = route_replay.merge_recorded_and_reconstructed_tracks(
        recorded_points,
        reconstructed,
        raw_corner_only=True,
      )
      latest_points = _copy_track_points(merged)
      latest_points_ns = event_ns
    elif which == "can":
      event_t = event_ns / 1e9
      for can_message in event.can:
        if int(can_message.src) != route_replay.RAW_CORNER_RADAR_BUS or int(can_message.src) >= 0x80:
          continue
        for obj in route_replay.decode_raw_corner_objects(event_t, int(can_message.address), bytes(can_message.dat)):
          if route_replay.raw_corner_object_is_valid(obj):
            corner_tracker.update(obj)
    elif which == "carParams":
      steer_ratio = _finite(event.carParams.steerRatio)
      wheelbase = _finite(event.carParams.wheelbase)
      latest_steer_ratio = steer_ratio if 5.0 <= steer_ratio <= 30.0 else 14.0
      latest_wheelbase = wheelbase if 1.8 <= wheelbase <= 4.5 else 2.8
    elif which == "carState":
      latest_v_ego = _finite(event.carState.vEgo)
      latest_steering_angle_deg = _finite(event.carState.steeringAngleDeg)
      latest_steering_rate_deg_s = _finite(event.carState.steeringRateDeg)
    elif which == "livePose":
      angular_velocity = event.livePose.angularVelocityDevice
      latest_live_pose = SimpleNamespace(
        angularVelocityDevice=SimpleNamespace(
          valid=bool(angular_velocity.valid),
          z=_finite(angular_velocity.z, math.nan),
        ),
        inputsOK=bool(event.livePose.inputsOK),
        sensorsOK=bool(event.livePose.sensorsOK),
      )
      latest_live_pose_ns = event_ns
    elif which == "modelV2":
      (
        latest_path,
        latest_lanes,
        latest_lane_probs,
        latest_model_leads,
        latest_path_y_stds,
        latest_lane_stds,
      ) = _copy_model(event.modelV2)
      latest_model_ns = event_ns
      latest_model_eof_ns = int(event.modelV2.timestampEof)
    elif which == "radarState" and latest_points is not None:
      radar_state = event.radarState
      live_pose_age_s = (
        max(0.0, (event_ns - latest_live_pose_ns) / 1e9)
        if latest_live_pose_ns
        else math.inf
      )
      yaw_rate_rad_s, yaw_rate_estimated, yaw_rate_source = ego_yaw_rate_rad_s(
        latest_v_ego,
        latest_steering_angle_deg,
        latest_live_pose,
        live_pose_age_s,
        latest_steer_ratio,
        latest_wheelbase,
      )
      absolute_frames.append((event_ns, RadarFrame(
        mono_time_s=event_ns / 1e9,
        time_s=0.0,
        input_age_s=max(0.0, (event_ns - latest_points_ns) / 1e9),
        model_age_s=max(0.0, (event_ns - latest_model_ns) / 1e9) if latest_model_ns else math.inf,
        v_ego=latest_v_ego,
        points=latest_points,
        path=latest_path,
        lane_lines=latest_lanes,
        lane_probs=latest_lane_probs,
        model_leads=latest_model_leads,
        recorded_one=_copy_recorded_lead(radar_state.leadOne),
        recorded_two=_copy_recorded_lead(radar_state.leadTwo),
        video_time_s=aligned_video_time_s(qcamera_start_eof_ns, latest_model_eof_ns),
        path_y_stds=latest_path_y_stds,
        lane_stds=latest_lane_stds,
        steering_angle_deg=latest_steering_angle_deg,
        steering_rate_deg_s=latest_steering_rate_deg_s,
        yaw_rate_rad_s=yaw_rate_rad_s,
        yaw_rate_estimated=yaw_rate_estimated,
        yaw_rate_source=yaw_rate_source,
        steer_ratio=latest_steer_ratio,
        wheelbase=latest_wheelbase,
      )))

  if not absolute_frames:
    raise RuntimeError("no aligned liveTracks/radarState frames were found in this log")

  origin_ns = absolute_frames[0][0]
  return [
    RadarFrame(
      mono_time_s=frame.mono_time_s,
      time_s=(event_ns - origin_ns) / 1e9,
      input_age_s=frame.input_age_s,
      model_age_s=frame.model_age_s,
      v_ego=frame.v_ego,
      points=frame.points,
      path=frame.path,
      lane_lines=frame.lane_lines,
      lane_probs=frame.lane_probs,
      model_leads=frame.model_leads,
      recorded_one=frame.recorded_one,
      recorded_two=frame.recorded_two,
      video_time_s=frame.video_time_s,
      path_y_stds=frame.path_y_stds,
      lane_stds=frame.lane_stds,
      steering_angle_deg=frame.steering_angle_deg,
      steering_rate_deg_s=frame.steering_rate_deg_s,
      yaw_rate_rad_s=frame.yaw_rate_rad_s,
      yaw_rate_estimated=frame.yaw_rate_estimated,
      yaw_rate_source=frame.yaw_rate_source,
      steer_ratio=frame.steer_ratio,
      wheelbase=frame.wheelbase,
    )
    for event_ns, frame in absolute_frames
  ]


def front_only_frames(frames: list[RadarFrame]) -> tuple[list[RadarFrame], int]:
  """Remove corner-radar inputs so replay follows the production front-only path."""
  filtered: list[RadarFrame] = []
  removed = 0
  for frame in frames:
    points = tuple(point for point in frame.points if not point.source.startswith("corner"))
    removed += len(frame.points) - len(points)
    filtered.append(replace(frame, points=points))
  return filtered, removed


def current_cutin_track_ids(
  log_path: Path,
  frames: list[RadarFrame],
  radar_sources: tuple[str, ...] = ("corner", "front"),
) -> list[set[int]]:
  """Align current corner/front cut-in replay results to radar frames."""
  script_path = Path(__file__).resolve()
  cluster_dir = script_path.parents[2] / "cluster"
  if str(cluster_dir) not in sys.path:
    sys.path.insert(0, str(cluster_dir))
  from cluster_route_replay import RouteLogParser, load_openpilot_log_schema

  aligned: list[set[int]] = [set() for _ in frames]
  for radar_source in radar_sources:
    route_frames = RouteLogParser(
      reconstruct_corner_live_tracks=True,
      cutin_radar_source=radar_source,
    ).parse_file(log_path, load_openpilot_log_schema())
    if not route_frames:
      continue
    route_times = [frame.t for frame in route_frames]
    for frame_index, radar_frame in enumerate(frames):
      index = bisect.bisect_left(route_times, radar_frame.mono_time_s)
      nearest_indices = [candidate for candidate in (index - 1, index) if 0 <= candidate < len(route_frames)]
      nearest = min(nearest_indices, key=lambda candidate: abs(route_times[candidate] - radar_frame.mono_time_s))
      route_frame = route_frames[nearest]
      available_ids = {point.track_id for point in radar_frame.points}
      aligned[frame_index].update(
        int(vehicle.radar_track_id)
        for vehicle in route_frame.detected_vehicles
        if vehicle.cut_in
        and vehicle.source.startswith("cutinReplay")
        and vehicle.radar_track_id is not None
        and int(vehicle.radar_track_id) in available_ids
      )
  return aligned


def candidate_track_id(candidate: Candidate | None) -> int | None:
  return candidate.track_id if candidate is not None else None


def recorded_track_id(lead: RecordedLead) -> int | None:
  return lead.track_id if lead.status and lead.radar and lead.track_id >= 0 else None


def resolved_recorded_track_id(frame: RadarFrame, lead: RecordedLead) -> int | None:
  """Map a recorded liveTracks slot ID to the stable raw-CAN track when needed."""
  track_id = recorded_track_id(lead)
  if track_id is None:
    return None
  if any(point.track_id == track_id for point in frame.points):
    return track_id
  nearby = [
    point for point in frame.points
    if abs(point.d_rel - lead.d_rel) <= 2.0
    and abs(point.y_rel - lead.y_rel) <= 1.0
    and abs(point.v_rel - lead.v_rel) <= 3.0
  ]
  if not nearby:
    return track_id
  best = min(
    nearby,
    key=lambda point: (
      abs(point.d_rel - lead.d_rel)
      + 2.0 * abs(point.y_rel - lead.y_rel)
      + 0.5 * abs(point.v_rel - lead.v_rel)
    ),
  )
  return best.track_id


def comparison_summary(frames: list[RadarFrame], selector: LeadSelector) -> dict[str, int]:
  result = {
    "frames": len(frames),
    "recorded_one_radar": 0,
    "lead_one_matches": 0,
    "recorded_two_radar": 0,
    "lead_two_matches": 0,
    "selected_exact": 0,
    "selected_true_positive": 0,
    "selected_false_positive": 0,
    "selected_false_negative": 0,
  }
  for frame_index, frame in enumerate(frames):
    selected = selector.select(frame, frame_index)
    recorded_one = resolved_recorded_track_id(frame, frame.recorded_one)
    recorded_two = resolved_recorded_track_id(frame, frame.recorded_two)
    recorded_ids = {track_id for track_id in (recorded_one, recorded_two) if track_id is not None}
    selected_ids = {
      track_id for track_id in (
        candidate_track_id(selected.lead_one), candidate_track_id(selected.lead_two)
      ) if track_id is not None
    }
    result["selected_exact"] += int(selected_ids == recorded_ids)
    result["selected_true_positive"] += len(selected_ids & recorded_ids)
    result["selected_false_positive"] += len(selected_ids - recorded_ids)
    result["selected_false_negative"] += len(recorded_ids - selected_ids)
    if recorded_one is not None:
      result["recorded_one_radar"] += 1
      result["lead_one_matches"] += int(recorded_one == candidate_track_id(selected.lead_one))
    if recorded_two is not None:
      result["recorded_two_radar"] += 1
      result["lead_two_matches"] += int(recorded_two == candidate_track_id(selected.lead_two))
  return result


def manual_comparison_summary(
  frames: list[RadarFrame],
  selector: LeadSelector,
  labels: ManualLabels,
) -> tuple[int, int]:
  matches = 0
  total = 0
  for frame_index, frame in enumerate(frames):
    selected = selector.select(frame, frame_index)
    for role, candidate in (("leadOne", selected.lead_one), ("leadTwo", selected.lead_two)):
      is_manual, target_track_id = labels.get(frame_index, role)
      if not is_manual:
        continue
      matches += int(candidate_track_id(candidate) == target_track_id)
      total += 1
  return matches, total


def export_csv(path: Path, frames: list[RadarFrame], selector: LeadSelector) -> None:
  with path.open("w", newline="", encoding="utf-8") as output:
    writer = csv.writer(output)
    writer.writerow([
      "time_s", "v_ego", "point_count",
      "recorded_one_id", "candidate_one_id", "candidate_one_score",
      "recorded_two_id", "candidate_two_id", "candidate_two_score",
    ])
    for frame_index, frame in enumerate(frames):
      selected = selector.select(frame, frame_index)
      writer.writerow([
        f"{frame.time_s:.3f}", f"{frame.v_ego:.3f}", len(frame.points),
        resolved_recorded_track_id(frame, frame.recorded_one), candidate_track_id(selected.lead_one),
        "" if selected.lead_one is None else f"{selected.lead_one.score:.4f}",
        resolved_recorded_track_id(frame, frame.recorded_two), candidate_track_id(selected.lead_two),
        "" if selected.lead_two is None else f"{selected.lead_two.score:.4f}",
      ])


def _training_target(
  frame_index: int,
  frame: RadarFrame,
  role: str,
  labels: ManualLabels,
  manual_only: bool,
) -> tuple[bool, int | None, str]:
  is_manual, manual_track_id = labels.get(frame_index, role)
  if is_manual:
    return True, manual_track_id, "manual"
  if manual_only:
    return False, None, ""
  recorded = frame.recorded_one if role == "leadOne" else frame.recorded_two
  track_id = resolved_recorded_track_id(frame, recorded)
  # A recorded vision-only or absent lead means none of the radar candidates
  # was selected. Preserve that as an all-negative group.
  return True, track_id, "recorded"


def _combined_training_target(
  frame_index: int,
  frame: RadarFrame,
  labels: ManualLabels,
  manual_only: bool,
  teacher_selection: Selection | None = None,
) -> tuple[bool, frozenset[int], str]:
  targets: set[int] = set()
  sources: set[str] = set()
  has_label = False
  teacher_candidates = {
    "leadOne": teacher_selection.lead_one if teacher_selection is not None else None,
    "leadTwo": teacher_selection.lead_two if teacher_selection is not None else None,
  }
  for role in LABEL_ROLES:
    is_manual, manual_track_id = labels.get(frame_index, role)
    if teacher_selection is not None and not is_manual and not manual_only:
      has_label = True
      sources.add("teacher")
      track_id = candidate_track_id(teacher_candidates[role])
      if track_id is not None:
        targets.add(track_id)
      continue
    role_has_label, track_id, source = _training_target(frame_index, frame, role, labels, manual_only)
    if not role_has_label:
      continue
    has_label = True
    sources.add(source)
    if track_id is not None:
      targets.add(track_id)
  label_source = "manual" if "manual" in sources else ("teacher" if "teacher" in sources else "recorded")
  return has_label, frozenset(targets), label_source


def _point_for_track(frame: RadarFrame, track_id: int) -> RadarPoint | None:
  return next((point for point in frame.points if point.track_id == track_id), None)


def _model_feature_names() -> tuple[str, ...]:
  names = [
    "v_ego", "point_count",
    "source_front", "source_scc", "source_corner235", "source_corner180",
    "measured", "d_rel", "y_rel", "v_rel", "a_rel", "yv_rel", "v_lead", "path_y", "d_path",
  ]
  for lane_index in range(4):
    names.extend((f"lane{lane_index}_y", f"lane{lane_index}_prob"))
  for lead_index in range(2):
    names.extend((
      f"model{lead_index}_prob", f"model{lead_index}_d", f"model{lead_index}_y",
      f"model{lead_index}_v", f"model{lead_index}_a", f"model{lead_index}_x_std",
      f"model{lead_index}_y_std", f"model{lead_index}_v_std",
    ))
  for offset in HISTORY_FRAME_OFFSETS:
    prefix = f"h{offset}"
    names.extend((
      f"{prefix}_present", f"{prefix}_d_rel", f"{prefix}_y_rel", f"{prefix}_v_rel",
      f"{prefix}_a_rel", f"{prefix}_yv_rel", f"{prefix}_v_lead",
    ))
  return tuple(names)


MODEL_FEATURE_NAMES = _model_feature_names()
DATASET_METADATA_COLUMNS = (
  "frame", "time_s", "role", "label_source", "target_track_ids", "track_id", "is_positive",
)


def candidate_feature_values(
  frames: list[RadarFrame],
  frame_index: int,
  point: RadarPoint,
) -> dict[str, float]:
  frame = frames[frame_index]
  usable_count = sum(0.75 < candidate.d_rel < 160.0 and abs(candidate.y_rel) < 12.0 for candidate in frame.points)
  path_y = model_line_y(frame.path, point.d_rel)
  values = {
    "v_ego": frame.v_ego,
    "point_count": float(usable_count),
    "source_front": float(point.source == "frontRadar"),
    "source_scc": float(point.source == "scc"),
    "source_corner235": float(point.source == "corner235"),
    "source_corner180": float(point.source == "corner180"),
    "measured": float(point.measured),
    "d_rel": point.d_rel,
    "y_rel": point.y_rel,
    "v_rel": point.v_rel,
    "a_rel": point.a_rel,
    "yv_rel": point.yv_rel,
    "v_lead": point.v_lead,
    "path_y": path_y,
    "d_path": point.y_rel - path_y,
  }
  for lane_index in range(4):
    lane = frame.lane_lines[lane_index] if lane_index < len(frame.lane_lines) else ()
    probability = frame.lane_probs[lane_index] if lane_index < len(frame.lane_probs) else 0.0
    values[f"lane{lane_index}_y"] = model_line_y(lane, point.d_rel) if lane else 0.0
    values[f"lane{lane_index}_prob"] = probability
  for lead_index in range(2):
    model_lead = frame.model_leads[lead_index] if lead_index < len(frame.model_leads) else None
    model_values = (
      model_lead.probability, model_lead.x - RADAR_TO_CAMERA, -model_lead.y, model_lead.v,
      model_lead.a, model_lead.x_std, model_lead.y_std, model_lead.v_std,
    ) if model_lead is not None else (0.0,) * 8
    for name, value in zip(("prob", "d", "y", "v", "a", "x_std", "y_std", "v_std"), model_values, strict=True):
      values[f"model{lead_index}_{name}"] = value
  for offset in HISTORY_FRAME_OFFSETS:
    history_frame = frames[max(0, frame_index - offset)]
    history_point = _point_for_track(history_frame, point.track_id)
    prefix = f"h{offset}"
    values[f"{prefix}_present"] = float(history_point is not None)
    history_values = (
      history_point.d_rel, history_point.y_rel, history_point.v_rel, history_point.a_rel,
      history_point.yv_rel, history_point.v_lead,
    ) if history_point is not None else (0.0,) * 6
    for name, value in zip(("d_rel", "y_rel", "v_rel", "a_rel", "yv_rel", "v_lead"), history_values, strict=True):
      values[f"{prefix}_{name}"] = value
  return values


def _dataset_header() -> list[str]:
  return [*DATASET_METADATA_COLUMNS, *MODEL_FEATURE_NAMES]


def export_training_dataset(
  path: Path,
  frames: list[RadarFrame],
  labels: ManualLabels,
  manual_only: bool = False,
  teacher: LeadSelector | None = None,
) -> dict[str, int]:
  """Write candidate-ranking rows with current context and 0.4 seconds of track history."""
  path.parent.mkdir(parents=True, exist_ok=True)
  stats = {
    "groups": 0, "manual_groups": 0, "recorded_groups": 0, "teacher_groups": 0, "none_groups": 0,
    "rows": 0, "positives": 0, "skipped": 0, "duplicate_groups_skipped": 0,
  }
  opener = gzip.open if path.suffix.lower() == ".gz" else open
  with opener(path, "wt", newline="", encoding="utf-8") as output:
    writer = csv.DictWriter(output, fieldnames=_dataset_header())
    writer.writeheader()
    for frame_index, frame in enumerate(frames):
      candidates = [
        point for point in frame.points
        if 0.75 < point.d_rel < 160.0 and abs(point.y_rel) < 12.0
      ]
      if not candidates:
        continue
      candidate_ids = {point.track_id for point in candidates}
      teacher_selection = teacher.select(frame, frame_index) if teacher is not None else None
      has_target, target_track_ids, label_source = _combined_training_target(
        frame_index, frame, labels, manual_only, teacher_selection
      )
      if not has_target:
        continue
      if not target_track_ids.issubset(candidate_ids):
        stats["skipped"] += 1
        continue

      stats["groups"] += 1
      stats[f"{label_source}_groups"] += 1
      stats["none_groups"] += int(not target_track_ids)
      target_text = ";".join(str(track_id) for track_id in sorted(target_track_ids))
      for point in candidates:
        is_positive = int(point.track_id in target_track_ids)
        row: dict[str, Any] = {
          "frame": frame_index,
          "time_s": f"{frame.time_s:.3f}",
          "role": "selected",
          "label_source": label_source,
          "target_track_ids": target_text,
          "track_id": point.track_id,
          "is_positive": is_positive,
        }
        row.update({name: f"{value:.5f}" for name, value in candidate_feature_values(frames, frame_index, point).items()})
        writer.writerow(row)
        stats["rows"] += 1
        stats["positives"] += is_positive
  return stats


class MLPLeadSelector:
  """Compact NumPy MLP loaded from radar_lead_train.py output."""

  def __init__(self, model_path: Path, frames: list[RadarFrame]) -> None:
    try:
      import numpy as np
    except ModuleNotFoundError as exc:
      raise RuntimeError("numpy is required for MLP radar lead inference") from exc
    self.np = np
    self.frames = frames
    self.model_path = model_path
    with np.load(model_path, allow_pickle=False) as model:
      feature_names = tuple(str(value) for value in model["feature_names"].tolist())
      if feature_names != MODEL_FEATURE_NAMES:
        raise RuntimeError("model feature schema does not match this simulator; retrain the model")
      self.mean = model["mean"].astype(np.float32)
      self.std = model["std"].astype(np.float32)
      self.w1 = model["w1"].astype(np.float32)
      self.b1 = model["b1"].astype(np.float32)
      self.w2 = model["w2"].astype(np.float32)
      self.b2 = model["b2"].astype(np.float32)
      self.w3 = model["w3"].astype(np.float32)
      self.b3 = model["b3"].astype(np.float32)
      thresholds = model["thresholds"].astype(np.float32)
      calibration = model["calibration"].astype(np.float32) if "calibration" in model.files else np.asarray([1.0, 0.0])
    self.thresholds = (
      (float(thresholds[0]), float(thresholds[1]))
      if len(thresholds) >= 2 else (float(thresholds[0]), float(thresholds[0]))
    )
    calibration = calibration.reshape(-1, 2)
    self.calibrations = (
      (float(calibration[0, 0]), float(calibration[0, 1])),
      (float(calibration[1, 0]), float(calibration[1, 1])) if len(calibration) >= 2 else
      (float(calibration[0, 0]), float(calibration[0, 1])),
    )
    self.name = f"mlp:{model_path.stem}"
    self.selections = self._precompute_selections()

  def _probabilities(self, features: Any, source_index: int) -> Any:
    np = self.np
    normalized = (features - self.mean) / self.std
    hidden1 = np.maximum(normalized @ self.w1 + self.b1, 0.0)
    hidden2 = np.maximum(hidden1 @ self.w2 + self.b2, 0.0)
    logits = (hidden2 @ self.w3 + self.b3).reshape(-1)
    calibration_scale, calibration_bias = self.calibrations[source_index]
    logits = logits * calibration_scale + calibration_bias
    return 1.0 / (1.0 + np.exp(-np.clip(logits, -30.0, 30.0)))

  def _precompute_selections(self) -> tuple[Selection, ...]:
    feature_rows: list[list[list[float]]] = [[], []]
    references: list[list[tuple[int, int]]] = [[], []]
    for frame_index, frame in enumerate(self.frames):
      candidates = [
        point for point in frame.points
        if 0.75 < point.d_rel < 160.0 and abs(point.y_rel) < 12.0
      ]
      for point in candidates:
        source_index = int(point.source.startswith("corner"))
        values = candidate_feature_values(self.frames, frame_index, point)
        feature_rows[source_index].append([values[name] for name in MODEL_FEATURE_NAMES])
        references[source_index].append((frame_index, point.track_id))

    scores: list[list[list[tuple[float, int]]]] = [
      [[] for _ in self.frames],
      [[] for _ in self.frames],
    ]
    for source_index in range(2):
      if not feature_rows[source_index]:
        continue
      matrix = self.np.asarray(feature_rows[source_index], dtype=self.np.float32)
      probabilities = self._probabilities(matrix, source_index)
      for (frame_index, track_id), probability in zip(references[source_index], probabilities, strict=True):
        scores[source_index][frame_index].append((float(probability), track_id))

    selections: list[Selection] = []
    for frame_index in range(len(self.frames)):
      source_candidates: list[tuple[Candidate, ...]] = []
      for source_index, source_name in enumerate(("front", "corner")):
        threshold = self.thresholds[source_index]
        selected = tuple(
          Candidate(track_id, probability, f"MLP {source_name} lead-th {threshold:.2f}", threshold)
          for probability, track_id in sorted(
            scores[source_index][frame_index], key=lambda item: item[0], reverse=True
          )[:2]
          if probability >= MLP_CANDIDATE_FLOOR
        )
        source_candidates.append(selected)
      ranked = sorted(
        (candidate for source in source_candidates for candidate in source if candidate.eligible),
        key=lambda candidate: candidate.score,
        reverse=True,
      )
      selections.append(Selection(
        ranked[0] if ranked else None,
        ranked[1] if len(ranked) > 1 else None,
        source_candidates[0],
        source_candidates[1],
      ))
    return tuple(selections)

  def select(self, frame: RadarFrame, frame_index: int | None = None) -> Selection:
    if frame_index is None:
      raise ValueError("MLP selector requires a frame index for track history")
    return self.selections[frame_index]


class MultitaskLeadSelector:
  """Fused object model with separate lead/cut-in/external outputs."""

  def __init__(
    self, model_path: Path, frames: list[RadarFrame], include_scc: bool = False,
    vision_match_primary: bool = False,
  ) -> None:
    self.model = RadarLeadModel(model_path)
    self.name = f"hybrid:{model_path.stem}" if vision_match_primary else f"multitask:{model_path.stem}"
    fusion = RadarObjectFusion(include_scc=include_scc)
    feature_builder = RadarLeadFeatureBuilder()
    vision_matcher = VisionRadarMatcher() if vision_match_primary else None
    decision_filter = RadarLeadDecisionFilter(
      lead_threshold=max(0.5, float(self.model.thresholds[0])),
      cutin_threshold=max(0.5, min(CUTIN_TEMPORAL_THRESHOLD_MAX, float(self.model.thresholds[1]))),
      external_threshold=max(0.5, float(self.model.thresholds[2])),
    )
    selections = []
    stealth_aliases: frozenset[str] = frozenset()
    stealth_hold_until = 0.0
    primary_hold_aliases: frozenset[str] = frozenset()
    primary_hold_until = 0.0
    displaced_primary_aliases: frozenset[str] = frozenset()
    displaced_primary_hold_until = 0.0
    for frame in frames:
      objects = fusion.update(frame.mono_time_s, frame.points)
      context = RadarLeadContext(
        time_s=frame.mono_time_s,
        v_ego=frame.v_ego,
        path=frame.path,
        lane_lines=frame.lane_lines,
        lane_probs=frame.lane_probs,
        model_leads=tuple(
          VisionLeadContext(
            lead.probability, lead.x - RADAR_TO_CAMERA, -lead.y, lead.v, lead.a,
            lead.x_std, lead.y_std, lead.v_std,
          ) for lead in frame.model_leads[:1]
        ),
      )
      features = feature_builder.update(context, objects)
      predictions = self.model.predict(features)
      decision = decision_filter.update(frame.mono_time_s, predictions)

      def track_id(prediction: Any) -> int:
        obj = prediction.features.radar_object
        return next(value for value in (obj.front_track_id, obj.corner_track_id, obj.scc_track_id) if value is not None)

      raw_leads = tuple(
        Candidate(track_id(prediction), prediction.lead_prob, "MLP fused lead", float(self.model.thresholds[0]))
        for prediction in sorted(predictions, key=lambda value: value.lead_prob, reverse=True)[:2]
        if prediction.lead_prob >= MLP_CANDIDATE_FLOOR
      )
      raw_cutins = tuple(
        Candidate(track_id(prediction), prediction.cutin_prob, "MLP fused cutin", float(self.model.thresholds[1]))
        for prediction in sorted(predictions, key=lambda value: value.cutin_prob, reverse=True)[:2]
        if prediction.cutin_prob >= MLP_CANDIDATE_FLOOR
      )
      raw_external = tuple(
        Candidate(track_id(prediction), prediction.external_prob, "MLP fused external", float(self.model.thresholds[2]))
        for prediction in sorted(predictions, key=lambda value: value.external_prob, reverse=True)[:2]
        if prediction.external_prob >= MLP_CANDIDATE_FLOOR
      )
      if vision_matcher is not None:
        vision_match = vision_matcher.match_context(
          context.model_leads[0] if context.model_leads else None,
          predictions,
          frame.v_ego,
        )
        if vision_match is None:
          lead_one = None
        else:
          obj = vision_match.prediction.features.radar_object
          lead_one = Candidate(
            track_id(vision_match.prediction),
            vision_match.probability,
            "vision-radar Laplacian match",
            d_rel=obj.front_d_rel if obj.front_d_rel is not None else obj.d_rel,
            y_rel=obj.y_rel,
          )
        primary_aliases = frozenset(vision_match.prediction.features.aliases) if vision_match is not None else frozenset()
        if primary_aliases:
          if primary_hold_aliases and not primary_aliases & primary_hold_aliases:
            displaced_primary_aliases = primary_hold_aliases
            displaced_primary_hold_until = frame.mono_time_s + PRIMARY_STEALTH_HOLD_S
          primary_hold_aliases = primary_aliases
          primary_hold_until = frame.mono_time_s + PRIMARY_STEALTH_HOLD_S
      else:
        lead_one_prediction = RadarLeadModelController._lead_one_prediction(decision.lead_candidates)
        lead_one = Candidate(
          track_id(lead_one_prediction), lead_one_prediction.lead_prob, "MLP active lead",
        ) if lead_one_prediction is not None else None
        if lead_one is None and frame.model_leads and frame.model_leads[0].probability > 0.5:
          vision = frame.model_leads[0]
          vision_distance = vision.x - RADAR_TO_CAMERA
          if vision_distance > 0.5:
            lead_one = Candidate(
              -1, vision.probability, "MLP vision fallback",
              d_rel=vision_distance, y_rel=-vision.y,
            )
        primary_aliases = frozenset()

      def matches_primary(prediction: Any, aliases: frozenset[str] = primary_aliases) -> bool:
        return bool(aliases & frozenset(prediction.features.aliases))
      primary_d_rel = lead_one.d_rel if lead_one is not None else None

      def cutin_relevant(prediction: Any, primary_distance: float | None = primary_d_rel) -> bool:
        return cutin_is_ahead_of_primary(prediction.features.radar_object.d_rel, primary_distance)

      active_cutins = [
        Candidate(track_id(value), value.cutin_prob, "MLP active cutin")
        for value in decision.cutin_candidates if not matches_primary(value) and cutin_relevant(value)
      ]
      active_external = [
        Candidate(track_id(value), value.external_prob, "MLP active external")
        for value in decision.external_candidates
      ]
      stealth_predictions = sorted((
        prediction for prediction in decision.lead_candidates
        if not matches_primary(prediction) and VisionModelRadarController._stealth_control_usable(prediction)
      ), key=lambda prediction: prediction.features.radar_object.d_rel)
      if stealth_predictions:
        stealth_aliases = frozenset(stealth_predictions[0].features.aliases)
        stealth_hold_until = frame.mono_time_s + STEALTH_LEAD_HOLD_S
      elif stealth_aliases and frame.mono_time_s <= stealth_hold_until:
        held_prediction = next((
          prediction for prediction in predictions
          if stealth_aliases & frozenset(prediction.features.aliases)
          and not matches_primary(prediction)
          and VisionModelRadarController._stealth_control_usable(prediction)
        ), None)
        if held_prediction is not None:
          stealth_predictions = [held_prediction]
      else:
        stealth_aliases = frozenset()
        stealth_hold_until = 0.0
      primary_hold_prediction = None
      hold_aliases = displaced_primary_aliases if primary_aliases else primary_hold_aliases
      hold_until = displaced_primary_hold_until if primary_aliases else primary_hold_until
      if hold_aliases and frame.mono_time_s <= hold_until:
        primary_hold_prediction = next((
          prediction for prediction in predictions
          if hold_aliases & frozenset(prediction.features.aliases)
          and not matches_primary(prediction)
          and VisionModelRadarController._primary_hold_usable(prediction)
        ), None)
      lead_two_prediction = next((
        prediction for prediction in decision.cutin_candidates
        if not matches_primary(prediction)
        and cutin_relevant(prediction)
        and VisionModelRadarController._external_control_usable(prediction)
      ), None)
      lead_two_reason = "MLP active cutin"
      lead_two_probability = lead_two_prediction.cutin_prob if lead_two_prediction is not None else 0.0
      if lead_two_prediction is None:
        lead_two_prediction = next((
          prediction for prediction in decision.external_candidates
          if not matches_primary(prediction)
          and VisionModelRadarController._external_control_usable(prediction)
        ), None)
        lead_two_reason = "MLP active external"
        lead_two_probability = lead_two_prediction.external_prob if lead_two_prediction is not None else 0.0
      if lead_two_prediction is None and primary_hold_prediction is not None:
        lead_two_prediction = primary_hold_prediction
        lead_two_reason = "MLP primary stealth"
        lead_two_probability = lead_two_prediction.lead_prob
      if lead_two_prediction is None and stealth_predictions:
        lead_two_prediction = stealth_predictions[0]
        lead_two_reason = "MLP active stealth"
        lead_two_probability = lead_two_prediction.lead_prob
      lead_two = Candidate(
        track_id(lead_two_prediction),
        lead_two_probability,
        lead_two_reason,
      ) if lead_two_prediction is not None else None
      selections.append(Selection(
        lead_one=lead_one,
        lead_two=lead_two,
        front_candidates=() if vision_matcher is not None else raw_leads,
        corner_candidates=raw_cutins,
        active_cutin_candidates=tuple(active_cutins),
        external_candidates=raw_external,
        active_external_candidates=tuple(active_external),
      ))
    self.selections = tuple(selections)

  def select(self, frame: RadarFrame, frame_index: int | None = None) -> Selection:
    if frame_index is None:
      raise ValueError("multitask selector requires a frame index")
    return self.selections[frame_index]


def _production_point(point: RadarPoint) -> SimpleNamespace:
  return SimpleNamespace(
    trackId=point.track_id,
    dRel=point.d_rel,
    yRel=point.y_rel,
    vRel=point.v_rel,
    aRel=point.a_rel,
    yvRel=point.yv_rel,
    vLead=point.v_lead,
    aLead=point.a_lead,
    jLead=point.j_lead,
    measured=point.measured,
    radarSource=point.source,
  )


def _production_xy(
  points: tuple[tuple[float, float], ...],
  y_stds: tuple[float, ...] = (),
) -> SimpleNamespace:
  return SimpleNamespace(
    x=tuple(point[0] for point in points),
    y=tuple(point[1] for point in points),
    yStd=y_stds,
  )


def _production_model(frame: RadarFrame) -> SimpleNamespace:
  leads = tuple(SimpleNamespace(
    prob=lead.probability,
    x=(lead.x,),
    y=(lead.y,),
    v=(lead.v,),
    a=(lead.a,),
    xStd=(lead.x_std,),
    yStd=(lead.y_std,),
    vStd=(lead.v_std,),
  ) for lead in frame.model_leads)
  path_std_by_x = {round(distance, 3): std for distance, std in frame.path_y_stds}
  path_y_stds = tuple(
    path_std_by_x.get(round(point[0], 3), 0.35)
    for point in frame.path
  )
  return SimpleNamespace(
    position=_production_xy(frame.path, path_y_stds),
    laneLines=tuple(_production_xy(line) for line in frame.lane_lines),
    laneLineProbs=frame.lane_probs,
    laneLineStds=frame.lane_stds,
    leadsV3=leads,
  )


def _production_car_state(frame: RadarFrame) -> SimpleNamespace:
  return SimpleNamespace(
    steeringAngleDeg=frame.steering_angle_deg,
    steeringRateDeg=frame.steering_rate_deg_s,
  )


def _production_live_pose(frame: RadarFrame) -> SimpleNamespace | None:
  if frame.yaw_rate_source != "livePose":
    return None
  return SimpleNamespace(
    angularVelocityDevice=SimpleNamespace(
      valid=True,
      z=frame.yaw_rate_rad_s,
    ),
    inputsOK=True,
    sensorsOK=True,
  )


def _output_candidate(
  lead: dict[str, Any] | None,
  reason: str,
  track_aliases: tuple[int, ...] = (),
) -> Candidate | None:
  if lead is None or not lead.get("status", False):
    return None
  return Candidate(
    track_id=int(lead["radarTrackId"]),
    score=float(lead.get("modelProb", lead.get("score", 0.0))),
    reason=reason,
    d_rel=float(lead.get("dRel", 0.0)),
    y_rel=float(lead.get("yRel", 0.0)),
    track_aliases=track_aliases,
  )


class ProductionHybridLeadSelector:
  """Replay the exact on-device model controller over normalized log inputs."""

  def __init__(
    self, model_path: Path, frames: list[RadarFrame], corner_model_path: Path | None = None,
  ) -> None:
    front_model_path = Path(model_path)
    corner_model_path = Path(corner_model_path) if corner_model_path is not None else front_model_path
    controller = VisionModelRadarController()
    has_front = any(point.source == "frontRadar" for frame in frames for point in frame.points)
    has_scc = any(point.source == "scc" for frame in frames for point in frame.points)
    has_corner = any(point.source.startswith("corner") for frame in frames for point in frame.points)
    self.name = (
      f"hybrid:{front_model_path.stem}+{corner_model_path.stem}"
      if has_corner else f"hybrid:{front_model_path.stem}:front-only"
    )
    enable_radar_tracks = 2 if has_front and has_scc else 1 if has_front else 0
    controller.runtime = RadarLeadRuntime(
      front_model_path=front_model_path,
      corner_model_path=corner_model_path,
      include_scc=True,
      enable_radar_tracks=enable_radar_tracks,
      corner_radar_enabled=has_corner,
      steer_ratio=frames[0].steer_ratio if frames else 14.0,
      wheelbase=frames[0].wheelbase if frames else 2.8,
    )
    selections: list[Selection] = []
    for frame in frames:
      output = controller.update(
        frame.mono_time_s,
        frame.v_ego,
        tuple(_production_point(point) for point in frame.points),
        _production_model(frame),
        _production_car_state(frame),
        _production_live_pose(frame),
        0.0 if frame.yaw_rate_source == "livePose" else math.inf,
      )
      result = controller.last_runtime_result
      if not output.available or result is None or not result.available:
        selections.append(Selection(None, None))
        continue

      def prediction_track_id(prediction: Any) -> int:
        obj = prediction.features.radar_object
        return next(value for value in (
          obj.front_track_id, obj.scc_track_id, obj.corner_track_id,
        ) if value is not None)

      all_predictions = tuple(getattr(result, "predictions", ()))
      front_predictions = tuple(getattr(result, "front_predictions", ())) or tuple(
        prediction for prediction in all_predictions
        if (
          prediction.features.radar_object.front_track_id is not None
          or prediction.features.radar_object.scc_track_id is not None
        )
      )
      corner_predictions = tuple(getattr(result, "corner_predictions", ())) or tuple(
        prediction for prediction in all_predictions
        if prediction.features.radar_object.corner_track_id is not None
      )
      use_corner = has_corner
      secondary_predictions = corner_predictions if use_corner else front_predictions
      secondary_model = (
        controller.runtime.corner_model if use_corner else controller.runtime.front_model
      ) or controller.runtime.model
      secondary_filter = (
        controller.runtime.corner_decisions if use_corner else controller.runtime.front_decisions
      )
      assert secondary_model is not None
      source_name = "corner" if use_corner else "front"
      trajectory_result = getattr(controller, "last_trajectory_result", None)
      trajectory_available = bool(
        trajectory_result is not None and trajectory_result.available
      )
      trajectory_predictions = (
        trajectory_result.predictions
      ) if trajectory_available else ()
      trajectory_confirmed = (
        trajectory_result.decision.confirmed
      ) if trajectory_available else ()
      trajectory_exiting = (
        trajectory_result.decision.exiting
      ) if trajectory_available else ()
      trajectory_decision = trajectory_result.decision if trajectory_available else None
      trajectory_filter = (
        controller.runtime.trajectory.corner_filter
        if use_corner else controller.runtime.trajectory.front_filter
      ) if trajectory_available else None
      cutin_threshold = (
        float(trajectory_filter.threshold)
        if trajectory_filter is not None
        else max(0.5, min(CUTIN_TEMPORAL_THRESHOLD_MAX, float(secondary_model.thresholds[1])))
      )

      def trajectory_threshold(
        prediction: Any,
        fallback_threshold: float = cutin_threshold,
      ) -> float:
        filter_value = (
          controller.runtime.trajectory.corner_filter
          if prediction.source.startswith("corner")
          else controller.runtime.trajectory.front_filter
        )
        return float(filter_value.threshold) if filter_value is not None else fallback_threshold

      def trajectory_exit_threshold(
        prediction: Any,
        fallback_threshold: float = cutin_threshold,
      ) -> float:
        filter_value = (
          controller.runtime.trajectory.corner_filter
          if prediction.source.startswith("corner")
          else controller.runtime.trajectory.front_filter
        )
        return float(filter_value.exit_threshold) if filter_value is not None else fallback_threshold
      external_threshold = (
        float(secondary_filter.external_threshold)
        if secondary_filter is not None
        else max(0.5, float(secondary_model.thresholds[2]))
      )

      alias_by_track_id: dict[int, set[int]] = {}

      def prediction_ids(prediction: Any) -> set[int]:
        obj = prediction.features.radar_object
        return {
          int(track_id)
          for track_id in (obj.front_track_id, obj.scc_track_id, obj.corner_track_id)
          if track_id is not None
        }

      def register_aliases(
        track_ids: set[int],
        aliases_by_track: dict[int, set[int]] = alias_by_track_id,
      ) -> None:
        if not track_ids:
          return
        merged = set(track_ids)
        for track_id in track_ids:
          merged.update(aliases_by_track.get(track_id, ()))
        for track_id in merged:
          aliases_by_track[track_id] = set(merged)

      for prediction in front_predictions:
        register_aliases(prediction_ids(prediction))
      for prediction in corner_predictions:
        track_ids = prediction_ids(prediction)
        matched = match_corner_to_front_identity(prediction, front_predictions)
        if matched is not None:
          track_ids.update(prediction_ids(matched))
        register_aliases(track_ids)

      def aliases_for(
        track_id: int,
        aliases_by_track: dict[int, set[int]] = alias_by_track_id,
      ) -> tuple[int, ...]:
        return tuple(sorted(set(aliases_by_track.get(track_id, ())) - {track_id}))

      raw_cutins = (
        tuple(
          Candidate(
            prediction.track_id,
            prediction.probability,
            f"trajectory {_source_mode_name(prediction.source)} path-entry",
            trajectory_threshold(prediction),
            d_rel=float(getattr(prediction.point, "d_rel", getattr(prediction.point, "dRel", 0.0))),
            y_rel=float(getattr(prediction.point, "y_rel", getattr(prediction.point, "yRel", 0.0))),
            track_aliases=aliases_for(prediction.track_id),
            horizon_scores=prediction.horizon_probabilities,
            out_horizon_scores=prediction.horizon_out_probabilities,
            path_exit_score=prediction.path_exit_probability,
            path_exit_threshold=trajectory_exit_threshold(prediction),
            current_path_occupancy=prediction.current_path_occupancy,
            source=prediction.source,
            horizon_x=tuple(getattr(prediction, "horizon_x", ())),
            horizon_y=tuple(getattr(prediction, "horizon_y", ())),
            horizon_x_stds=tuple(getattr(prediction, "horizon_x_stds", ())),
            horizon_y_stds=tuple(getattr(prediction, "horizon_y_stds", ())),
          )
          for prediction in sorted(
            trajectory_predictions, key=lambda value: value.probability, reverse=True,
          )
        )
        if trajectory_available
        else ()
      )
      raw_external = tuple(
        Candidate(
          prediction_track_id(prediction), prediction.external_prob, f"MLP {source_name} external",
          external_threshold, d_rel=prediction.features.radar_object.d_rel,
          y_rel=prediction.features.radar_object.y_rel,
          track_aliases=aliases_for(prediction_track_id(prediction)),
        )
        for prediction in sorted(secondary_predictions, key=lambda value: value.external_prob, reverse=True)[:2]
        if prediction.external_prob >= MLP_CANDIDATE_FLOOR
      )
      decision_cutins = (
        tuple(
          Candidate(
            prediction.track_id,
            prediction.probability,
            "trajectory decision cutin",
            trajectory_threshold(prediction),
            d_rel=float(getattr(prediction.point, "d_rel", getattr(prediction.point, "dRel", 0.0))),
            y_rel=float(getattr(prediction.point, "y_rel", getattr(prediction.point, "yRel", 0.0))),
            track_aliases=aliases_for(prediction.track_id),
            horizon_scores=prediction.horizon_probabilities,
            out_horizon_scores=prediction.horizon_out_probabilities,
            path_exit_score=prediction.path_exit_probability,
            path_exit_threshold=trajectory_exit_threshold(prediction),
            current_path_occupancy=prediction.current_path_occupancy,
            source=prediction.source,
            horizon_x=tuple(getattr(prediction, "horizon_x", ())),
            horizon_y=tuple(getattr(prediction, "horizon_y", ())),
            horizon_x_stds=tuple(getattr(prediction, "horizon_x_stds", ())),
            horizon_y_stds=tuple(getattr(prediction, "horizon_y_stds", ())),
          )
          for prediction in trajectory_confirmed
        )
        if trajectory_decision is not None
        else tuple(
          Candidate(
            prediction_track_id(prediction), prediction.cutin_prob, "MLP decision cutin",
            cutin_threshold, d_rel=prediction.features.radar_object.d_rel,
            y_rel=prediction.features.radar_object.y_rel,
            track_aliases=aliases_for(prediction_track_id(prediction)),
          )
          for prediction in getattr(getattr(result, "decision", None), "cutin_candidates", ())
        )
      )
      active_cutins = tuple(
        candidate for lead in output.leads_cutin
        if (candidate := _output_candidate(
          lead,
          "trajectory active cutin" if trajectory_available else "MLP active cutin",
          aliases_for(int(lead["radarTrackId"])),
        )) is not None
      )
      active_external = tuple(
        candidate for lead in (output.lead_external,)
        if lead is not None
        if (candidate := _output_candidate(
          lead, "MLP active external", aliases_for(int(lead["radarTrackId"])),
        )) is not None
      )
      cutin_ids = {candidate.track_id for candidate in active_cutins}
      external_ids = {candidate.track_id for candidate in active_external}
      lead_two_id = int(output.lead_two["radarTrackId"]) if output.lead_two is not None else None
      if lead_two_id in cutin_ids:
        model_name = "trajectory" if trajectory_available else "MLP"
        state = "tentative" if output.lead_two_tentative else "confirmed"
        lead_two_reason = f"{model_name} {state} cutin"
      elif lead_two_id in external_ids:
        lead_two_reason = "MLP active external"
      else:
        lead_two_reason = "MLP active stealth"

      decision_prediction_ids = (
        tuple(
          frozenset((prediction.track_id, *aliases_for(prediction.track_id)))
          for prediction in trajectory_confirmed
        )
        if trajectory_decision is not None
        else tuple(
          prediction_ids(prediction)
          for prediction in getattr(getattr(result, "decision", None), "cutin_candidates", ())
        )
      )
      active_candidate_ids = tuple(candidate_track_ids(candidate) for candidate in active_cutins)
      selected_cutin_ids = (
        candidate_track_ids(_output_candidate(
          output.lead_two,
          lead_two_reason,
          aliases_for(int(output.lead_two["radarTrackId"])),
        ))
        if output.lead_two is not None and "cutin" in lead_two_reason
        else frozenset()
      )
      cutin_diagnostics: list[Candidate] = []
      if trajectory_available:
        primary_d_rel = (
          float(output.lead_one["dRel"])
          if output.lead_one is not None
          else None
        )
        exiting_ids = {
          (prediction.source, prediction.track_id, prediction.trajectory.continuity_id)
          for prediction in trajectory_exiting
        }
        for prediction in trajectory_predictions:
          track_id = prediction.track_id
          ids = frozenset((track_id, *aliases_for(track_id)))
          in_decision = any(ids.intersection(candidate_ids) for candidate_ids in decision_prediction_ids)
          in_output = any(ids.intersection(candidate_ids) for candidate_ids in active_candidate_ids)
          is_selected = bool(ids.intersection(selected_cutin_ids))
          identity = (prediction.source, track_id, prediction.trajectory.continuity_id)
          point_threshold = trajectory_threshold(prediction)
          point_exit_threshold = trajectory_exit_threshold(prediction)
          raw = "/".join(f"{value:.2f}" for value in prediction.horizon_probabilities)
          raw_out = "/".join(
            f"{value:.2f}" for value in prediction.horizon_out_probabilities
          )
          prediction_x = tuple(getattr(prediction, "horizon_x", ()))
          prediction_y = tuple(getattr(prediction, "horizon_y", ()))
          prediction_x_stds = tuple(getattr(prediction, "horizon_x_stds", ()))
          prediction_y_stds = tuple(getattr(prediction, "horizon_y_stds", ()))
          raw_x = "/".join(f"{value:.1f}" for value in prediction_x)
          raw_y = "/".join(f"{value:+.2f}" for value in prediction_y)
          raw_x_std = "/".join(f"{value:.2f}" for value in prediction_x_stds)
          raw_y_std = "/".join(f"{value:.2f}" for value in prediction_y_stds)
          ahead = "/".join(
            f"{forward_probability(x_value, x_std):.2f}"
            for x_value, x_std in zip(
              prediction_x, prediction_x_stds, strict=True,
            )
          )
          if is_selected:
            stage = "SELECTED"
            final_detail = "selected as leadTwo"
          elif in_output:
            stage = "OUTPUT"
            final_detail = "controller output; another target selected"
          elif in_decision:
            stage = "DECISION"
            final_detail = "CUT-IN active; not selected as leadTwo"
          elif identity in exiting_ids:
            stage = "CUT-OUT"
            final_detail = f"CUT-OUT active {prediction.path_out_probability:.2f}"
          elif primary_d_rel is not None and float(getattr(
            prediction.point, "d_rel", getattr(prediction.point, "dRel", math.inf),
          )) > primary_d_rel:
            stage = "FARTHER-L1"
            final_detail = f"farther than leadOne {primary_d_rel:.1f}m"
          else:
            stage = "BELOW"
            final_detail = f"IN {prediction.path_in_probability:.2f} < ON {point_threshold:.2f}"
          cutin_diagnostics.append(Candidate(
            track_id=track_id,
            score=float(prediction.probability),
            reason=f"trajectory {_source_mode_name(prediction.source)} path-entry",
            decision_threshold=point_threshold,
            d_rel=float(getattr(prediction.point, "d_rel", getattr(prediction.point, "dRel", 0.0))),
            y_rel=float(getattr(prediction.point, "y_rel", getattr(prediction.point, "yRel", 0.0))),
            track_aliases=aliases_for(track_id),
            base_score=float(prediction.horizon_probabilities[0]),
            temporal_score=float(prediction.horizon_probabilities[1]),
            horizon_scores=prediction.horizon_probabilities,
            out_horizon_scores=prediction.horizon_out_probabilities,
            path_exit_score=float(prediction.path_exit_probability),
            path_exit_threshold=point_exit_threshold,
            current_path_occupancy=bool(prediction.current_path_occupancy),
            stage=stage,
            detail=(
              f"X {raw_x}; Y {raw_y}; SX {raw_x_std}; SY {raw_y_std}; "
              + f"I {raw}; O {raw_out}; A {ahead}; "
              + f"IN {prediction.path_in_probability:.2f}; "
              + f"OUT {prediction.path_out_probability:.2f}; {final_detail}"
            ),
            source=prediction.source,
            horizon_x=prediction_x,
            horizon_y=prediction_y,
            horizon_x_stds=prediction_x_stds,
            horizon_y_stds=prediction_y_stds,
          ))

      selections.append(Selection(
        lead_one=_output_candidate(
          output.lead_one,
          "vision-radar Laplacian match",
          aliases_for(int(output.lead_one["radarTrackId"])) if output.lead_one is not None else (),
        ),
        lead_two=_output_candidate(
          output.lead_two,
          lead_two_reason,
          aliases_for(int(output.lead_two["radarTrackId"])) if output.lead_two is not None else (),
        ),
        lead_two_tentative=output.lead_two_tentative if lead_two_id in cutin_ids else None,
        front_candidates=tuple(
          candidate for candidate in raw_cutins if "trajectory front" in candidate.reason
        ),
        corner_candidates=tuple(
          candidate for candidate in raw_cutins
          if "trajectory corner" in candidate.reason or not trajectory_available
        ),
        cutin_diagnostics=tuple(sorted(
          cutin_diagnostics,
          key=lambda candidate: (-candidate.score, candidate.d_rel or math.inf),
        )),
        decision_cutin_candidates=decision_cutins,
        active_cutin_candidates=active_cutins,
        external_candidates=raw_external,
        active_external_candidates=active_external,
      ))
    self.selections = tuple(selections)

  def select(self, frame: RadarFrame, frame_index: int | None = None) -> Selection:
    if frame_index is None:
      raise ValueError("production hybrid selector requires a frame index")
    return self.selections[frame_index]


class SimulatorUI:
  BG = (14, 18, 23, 255)
  PANEL = (22, 28, 35, 255)
  GRID = (49, 59, 69, 255)
  TEXT = (221, 228, 234, 255)
  MUTED = (139, 151, 162, 255)
  ORANGE = (255, 157, 49, 255)
  YELLOW = (255, 220, 74, 255)
  GREEN = (61, 214, 140, 255)
  PURPLE = (203, 111, 255, 255)
  RED = (247, 94, 94, 255)
  CYAN = (78, 201, 224, 255)

  def __init__(
    self,
    frames: list[RadarFrame],
    selector: LeadSelector,
    title: str,
    labels: ManualLabels,
    label_path: Path,
    log_path: Path,
    include_scc_fusion: bool = False,
    review: ValidationReview | None = None,
    reviews: tuple[ValidationReview, ...] = (),
    validation_cases_path: Path | None = None,
    radard_selector: LeadSelector | None = None,
    initial_probability: float | None = None,
    manual_probability_mode: bool = False,
  ) -> None:
    import pyray as rl
    self.rl = rl
    self.frames = frames
    self.times = [frame.time_s for frame in frames]
    self.selector = selector
    self.title = title
    self.index = 0
    self.paused = False
    self.speed = 1.0
    self.forward_range_m = DEFAULT_FORWARD_RANGE_M
    self.show_labels = False
    self.show_recorded_one = False
    self.show_recorded_two = False
    self.show_front_candidates = review is None
    self.show_corner_candidates = review is None
    self.show_external_candidates = review is None
    self.show_radar_points = True
    self.show_trajectories = review is not None or bool(reviews)
    # Raw sensor points are the least ambiguous default for diagnosis. FUSED
    # can still be enabled to inspect the objects actually passed to the model.
    self.show_fused_objects = False
    self.review_settings_path = default_review_settings_path()
    self.device_review_mode = initial_probability is None and not manual_probability_mode
    self.min_candidate_probability = (
      min(max(float(initial_probability), 0.0), 1.0)
      if initial_probability is not None
      else load_review_probability(self.review_settings_path)
    )
    if initial_probability is not None:
      save_review_probability(self.min_candidate_probability, self.review_settings_path)
    self.filter_checkboxes: dict[str, Any] = {}
    self.review_label_buttons: dict[str, Any] = {}
    self.probability_slider: Any | None = None
    self.trajectory_horizon_slider: Any | None = None
    self.trajectory_horizon_s = 1.0
    self.playback_time = 0.0
    self.labels = labels
    self.label_path = label_path
    self.log_path = log_path
    self.selected_role = "leadOne"
    self.range_anchor: tuple[int, str, int | None] | None = None
    self.label_status = f"labels: {labels.count()} loaded"
    self.reviews = reviews or ((review,) if review is not None else ())
    self.review = review or (self.reviews[0] if self.reviews else None)
    self.validation_cases_path = validation_cases_path
    self.trajectory_review_labels = self._load_trajectory_review_labels()
    self.radard_selector = radard_selector
    self.review_events: dict[int, tuple[str, ...]] = {}
    self.review_handled: set[int] = set()
    self.review_suppressed: set[int] = set()
    self.review_status = ""
    self.lead_expected_prefix: list[int] = [0]
    self.lead_matched_prefix: list[int] = [0]
    self.lead_drop_runs: list[int] = []
    self.lead_max_drop_s: list[float] = []
    self.lead_current_drop_s: list[float] = []
    self.lead_two_continuity: tuple[CutinContinuity | None, ...] = ()
    self.lead_comparison = (
      lead_comparison_series(self.frames, self.radard_selector, self.selector)
      if self.review is not None else ()
    )
    self.cutin_stages = (
      cutin_stage_series(self.frames, self.selector)
      if self.review is not None else ()
    )
    self.trajectories = radar_trajectory_series(self.frames)
    # Validation replays show the exact independent production inputs. Legacy
    # fusion remains available only in the unlabeled general-purpose simulator.
    if self.review is None:
      fusion = RadarObjectFusion(include_scc=include_scc_fusion)
      self.fused_frames = tuple(fusion.update(frame.mono_time_s, frame.points) for frame in frames)
    else:
      self.fused_frames = ()
    self.video_path = qcamera_path_for_log(log_path)
    self.video_reader: Any | None = None
    self.video_texture: Any | None = None
    self.video_texture_size: tuple[int, int] | None = None
    self.video_frame_id: str | None = None
    self.font: Any | None = None
    self._prepare_lead_continuity()
    self.lead_two_continuity = cutin_continuity_series(self.frames, self.selector)
    self._prepare_review_events()
    if self.video_path.is_file():
      cluster_dir = Path(__file__).resolve().parents[2] / "cluster"
      if str(cluster_dir) not in sys.path:
        sys.path.insert(0, str(cluster_dir))
      from cluster_route_replay import RouteVideoFrameReader, RouteVideoSegment
      video_end_s = max((frame.video_time_s if frame.video_time_s is not None else frame.time_s) for frame in self.frames)
      self.video_reader = RouteVideoFrameReader([
        RouteVideoSegment(None, self.video_path, 0.0, video_end_s),
      ])

  def _prepare_lead_continuity(self) -> None:
    drop_runs = 0
    drop_started_at: float | None = None
    max_drop_s = 0.0
    for index, frame in enumerate(self.frames):
      vision_expected = bool(frame.model_leads and frame.model_leads[0].probability > 0.5)
      lead_present = self.selector.select(frame, index).lead_one is not None
      dropped = vision_expected and not lead_present
      self.lead_expected_prefix.append(self.lead_expected_prefix[-1] + int(vision_expected))
      self.lead_matched_prefix.append(self.lead_matched_prefix[-1] + int(vision_expected and lead_present))

      if dropped:
        if drop_started_at is None:
          drop_started_at = frame.time_s
          drop_runs += 1
        frame_dt = self.times[index] - self.times[index - 1] if index > 0 else 0.05
        current_drop_s = max(0.0, frame.time_s - drop_started_at) + max(frame_dt, 0.0)
        max_drop_s = max(max_drop_s, current_drop_s)
      else:
        drop_started_at = None
        current_drop_s = 0.0
      self.lead_drop_runs.append(drop_runs)
      self.lead_max_drop_s.append(max_drop_s)
      self.lead_current_drop_s.append(current_drop_s)

  def _lead_continuity_text(self) -> tuple[str, bool]:
    end = self.index + 1
    expected = self.lead_expected_prefix[end]
    matched = self.lead_matched_prefix[end]
    recent_start = bisect.bisect_left(self.times, max(0.0, self.times[self.index] - 5.0))
    recent_expected = expected - self.lead_expected_prefix[recent_start]
    recent_matched = matched - self.lead_matched_prefix[recent_start]
    if expected == 0:
      return "leadOne continuity: waiting for vision lead", False
    total_rate = 100.0 * matched / expected
    recent_rate = 100.0 * recent_matched / max(recent_expected, 1)
    lost_frames = expected - matched
    current_drop_s = self.lead_current_drop_s[self.index]
    text = (
      f"leadOne continuity {total_rate:5.1f}%  recent5s {recent_rate:5.1f}%  "
      + f"loss {lost_frames}f/{self.lead_drop_runs[self.index]}x  "
      + f"max {self.lead_max_drop_s[self.index]:.2f}s"
    )
    if current_drop_s > 0.0:
      text += f"  NOW LOST {current_drop_s:.2f}s"
    return text, current_drop_s > 0.0

  def _lead_two_continuity_text(self) -> tuple[str, bool]:
    continuity = self.lead_two_continuity[self.index]
    if continuity is None:
      return "leadTwo cut-in continuity: waiting for active cut-in", False
    rate = 100.0 * continuity.matched_frames / max(continuity.episode_frames, 1)
    lost_frames = continuity.episode_frames - continuity.matched_frames
    text = (
      f"leadTwo cut-in id{continuity.track_id} {rate:5.1f}%  "
      + f"loss {lost_frames}f/{continuity.drop_runs}x  max {continuity.max_drop_s:.2f}s"
    )
    if continuity.current_drop_s > 0.0:
      text += f"  NOW LOST {continuity.current_drop_s:.2f}s"
    elif not continuity.active:
      text += "  ENDED"
    return text, continuity.current_drop_s > 0.0

  def _prepare_review_events(self) -> None:
    if self.review is None:
      return
    if len(self.reviews) > 1:
      base_events = validation_log_review_events(self.frames, self.selector, self.reviews)
    else:
      base_events = validation_review_events(self.frames, self.selector, self.review)
    self.review_events = dict(base_events)
    self.review_suppressed = set()
    self.review_handled = set()
    display_mode = (
      "model thresholds"
      if self.device_review_mode
      else f"display prob {self.min_candidate_probability:.2f}"
    )
    self.review_status = (
      f"LEAD2 REVIEW ARMED  {len(self.reviews)} case(s), "
      + f"{len(base_events)} production selection event(s); {display_mode}"
    )

  @staticmethod
  def _trajectory_event_track_ids(events: Iterable[str]) -> tuple[int, ...]:
    track_ids: list[int] = []
    for event in events:
      if not event.startswith("TRAJECTORY "):
        continue
      tokens = event.replace(":", " ").split()
      for token_index, token in enumerate(tokens[:-1]):
        if token != "id":
          continue
        try:
          track_ids.append(int(tokens[token_index + 1]))
        except ValueError:
          pass
    return tuple(track_ids)

  def _trajectory_label_key_at(self, frame_index: int, track_id: int) -> tuple[float, int]:
    time_key = round(self.frames[frame_index].time_s * 20.0) / 20.0
    return time_key, track_id

  def _trajectory_events_fully_labeled(self, frame_index: int, events: Iterable[str]) -> bool:
    track_ids = self._trajectory_event_track_ids(events)
    return bool(track_ids) and all(
      self._trajectory_label_key_at(frame_index, track_id) in self.trajectory_review_labels
      for track_id in track_ids
    )

  def _review_containing_time(self, time_s: float) -> ValidationReview | None:
    matches = [
      review for review in self.reviews
      if review.start_s <= time_s <= review.end_s
    ]
    return min(
      matches,
      key=lambda review: (review.end_s - review.start_s, abs((review.start_s + review.end_s) * 0.5 - time_s)),
      default=None,
    )

  def _review_for_time(self, time_s: float) -> ValidationReview | None:
    containing = self._review_containing_time(time_s)
    if containing is not None:
      return containing
    return min(
      self.reviews,
      key=lambda review: min(abs(time_s - review.start_s), abs(time_s - review.end_s)),
      default=None,
    )

  def _pause_for_review(self, previous_index: int, current_index: int) -> None:
    if self.review is None or current_index <= previous_index:
      return
    for index in range(previous_index + 1, current_index + 1):
      if index in self.review_events and index not in self.review_handled:
        self.review_handled.add(index)
        self.seek(self.times[index])
        self.paused = True
        self.review_status = f"PAUSED @{self.times[index]:.2f}s: " + " + ".join(self.review_events[index])
        play_review_alert(self.review_events[index])
        return

  def _rearm_review_after(self, frame_index: int) -> None:
    suppressed = getattr(self, "review_suppressed", set())
    self.review_handled.difference_update(
      index
      for index in self.review_events
      if index > frame_index and index not in suppressed
    )
    mode = (
      "AUTO MODEL THRESHOLDS"
      if getattr(self, "device_review_mode", False)
      else f"DISPLAY PROB {self.min_candidate_probability:.2f}"
    )
    self.review_status = (
      f"LEAD2 REVIEW REARMED after {self.times[frame_index]:.2f}s  {mode}"
    )

  def _user_seek(self, time_s: float) -> None:
    previous_index = self.index
    self.seek(time_s)
    if self.index < previous_index:
      self._rearm_review_after(self.index)
    elif self.review is not None:
      self.review_status = (
        f"MANUAL SEEK @{self.times[self.index]:.2f}s; "
        + "leadTwo pause events remain armed"
      )

  def _color(self, value: tuple[int, int, int, int]) -> Any:
    return self.rl.Color(*value)

  def _draw_text(self, text: str, x: float, y: float, size: float, color: Any) -> None:
    if self.font is None:
      self.rl.draw_text(text, int(x), int(y), int(size), color)
    else:
      self.rl.draw_text_ex(self.font, text, self.rl.Vector2(x, y), size, 0.0, color)

  def _measure_text(self, text: str, size: float) -> float:
    if self.font is None:
      return float(self.rl.measure_text(text, int(size)))
    return float(self.rl.measure_text_ex(self.font, text, size, 0.0).x)

  def seek(self, time_s: float) -> None:
    self.playback_time = min(max(0.0, time_s), self.times[-1])
    self.index = min(bisect.bisect_right(self.times, self.playback_time) - 1, len(self.frames) - 1)
    self.index = max(0, self.index)
    active_review = self._review_for_time(self.playback_time)
    if active_review is not None:
      self.review = active_review

  def _track_point(self, frame: RadarFrame, track_id: int) -> RadarPoint | None:
    return next((point for point in frame.points if point.track_id == track_id), None)

  def _world_to_screen(self, map_rect: Any, distance: float, lateral: float) -> Any:
    rl = self.rl
    bottom = map_rect.y + map_rect.height - 34.0
    top = map_rect.y + 34.0
    scale_y = (bottom - top) / self.forward_range_m
    lateral_zoom = 4.0 if self.review is not None else 1.0
    scale_x = min(map_rect.width / (24.0 / lateral_zoom), scale_y * 2.4 * lateral_zoom)
    return rl.Vector2(map_rect.x + map_rect.width * 0.5 - lateral * scale_x, bottom - distance * scale_y)

  def _draw_world_line(self, map_rect: Any, points: tuple[tuple[float, float], ...], color: Any, width: float) -> None:
    rl = self.rl
    previous = None
    for distance, model_lateral in points:
      if distance < 0.0 or distance > self.forward_range_m:
        continue
      current = self._world_to_screen(map_rect, distance, -model_lateral)
      if previous is not None:
        rl.draw_line_ex(previous, current, width, color)
      previous = current

  def _source_color(self, source: str) -> Any:
    if source == "scc":
      return self._color(self.YELLOW)
    if source.startswith("corner"):
      return self._color(self.PURPLE)
    return self._color(self.CYAN)

  @staticmethod
  def _stage_code(stage: str) -> str:
    return {
      "SELECTED": "S",
      "OUTPUT": "O",
      "DECISION": "D",
      "CUT-OUT": "X",
      "SOURCE-OFF": "F",
      "FARTHER-L1": "L",
      "BELOW": "-",
    }.get(stage, "-")

  def _stage_color(self, stage: str) -> tuple[int, int, int, int]:
    if stage in ("SELECTED", "OUTPUT", "DECISION"):
      return self.GREEN
    if stage == "CUT-OUT":
      return self.PURPLE
    if stage == "FARTHER-L1":
      return self.RED
    return self.MUTED

  @staticmethod
  def _diagnostic_for_point(selection: Selection, point: RadarPoint) -> Candidate | None:
    exact = tuple(
      candidate for candidate in selection.cutin_diagnostics
      if candidate.track_id == point.track_id
      and (not candidate.source or candidate.source == point.source)
    )
    if exact:
      return max(exact, key=lambda candidate: candidate.score)
    aliases = (
      candidate for candidate in selection.cutin_diagnostics
      if point.track_id in candidate_track_ids(candidate)
      and (not candidate.source or _source_mode_name(candidate.source) == _source_mode_name(point.source))
    )
    return max(aliases, key=lambda candidate: candidate.score, default=None)

  @staticmethod
  def _selected_trajectory_diagnostic(
    selection: Selection,
    candidate: Candidate | None,
  ) -> Candidate | None:
    if candidate is None or "cutin" not in candidate.reason.lower():
      return None
    selected_ids = candidate_track_ids(candidate)
    return next(
      (
        diagnostic for diagnostic in selection.cutin_diagnostics
        if diagnostic.stage == "SELECTED"
        and bool(selected_ids & candidate_track_ids(diagnostic))
      ),
      None,
    )

  @staticmethod
  def _point_identity(point: RadarPoint) -> str:
    prefix = (
      "C" if point.source.startswith("corner")
      else "F" if point.source == "frontRadar"
      else "S"
    )
    return f"{prefix}{point.track_id}"

  def _visible_radar_points(
    self,
    frame: RadarFrame,
    selection: Selection,
  ) -> tuple[RadarPoint, ...]:
    points = list(preferred_radar_points(
      frame, self.review.source if self.review is not None else None,
    ))
    keys = {(point.source, point.track_id) for point in points}
    for diagnostic in selection.cutin_diagnostics:
      if diagnostic.stage != "SELECTED":
        continue
      selected_point = next(
        (
          point for point in frame.points
          if point.track_id == diagnostic.track_id
          and point.source == diagnostic.source
        ),
        None,
      )
      if selected_point is None:
        continue
      key = (selected_point.source, selected_point.track_id)
      if key not in keys:
        points.append(selected_point)
        keys.add(key)
    return tuple(points)

  @staticmethod
  def _max_future_occupancy_probability(diagnostic: Candidate) -> float:
    return max(diagnostic.horizon_scores, default=diagnostic.score)

  def _future_occupancy_at_display_horizon(self, diagnostic: Candidate) -> float:
    if not diagnostic.horizon_scores:
      return diagnostic.score
    horizons_s = (0.5, 1.0, 1.5, 2.0)
    index = min(
      range(min(len(horizons_s), len(diagnostic.horizon_scores))),
      key=lambda value: abs(horizons_s[value] - self.trajectory_horizon_s),
    )
    return diagnostic.horizon_scores[index]

  def _future_out_at_display_horizon(self, diagnostic: Candidate) -> float:
    if not diagnostic.out_horizon_scores:
      return diagnostic.path_out_score
    horizons_s = (0.5, 1.0, 1.5, 2.0)
    index = min(
      range(min(len(horizons_s), len(diagnostic.out_horizon_scores))),
      key=lambda value: abs(horizons_s[value] - self.trajectory_horizon_s),
    )
    return diagnostic.out_horizon_scores[index]

  def _display_probability_threshold(self, diagnostic: Candidate) -> float:
    if getattr(self, "device_review_mode", False) and diagnostic.decision_threshold > 0.0:
      return diagnostic.decision_threshold
    return self.min_candidate_probability

  def _trajectory_for_point(self, point: RadarPoint) -> RadarTrajectory | None:
    return self.trajectories[self.index].get((point.source, point.track_id))

  def _trajectory_color(self, probability: float, alpha: int = 255) -> Any:
    if probability >= 0.60:
      value = self.ORANGE
    elif probability >= 0.30:
      value = self.CYAN
    else:
      value = self.MUTED
    return self._color((*value[:3], alpha))

  def _draw_trajectory(
    self,
    map_rect: Any,
    frame: RadarFrame,
    point: RadarPoint,
    trajectory: RadarTrajectory,
    diagnostic: Candidate | None,
    show_label: bool = True,
  ) -> None:
    rl = self.rl
    if point.d_rel <= 0.5 or point.d_rel > self.forward_range_m:
      return

    history = [
      sample for sample in trajectory.history
      if sample.d_rel <= self.forward_range_m and sample.age_s > 0.01
    ]
    history.sort(key=lambda sample: sample.age_s, reverse=True)
    previous = None
    for sample in history:
      position = self._world_to_screen(
        map_rect,
        sample.d_rel,
        trajectory_history_display_y(frame, sample),
      )
      if previous is not None:
        rl.draw_line_ex(previous, position, 1.5, self._color((*self.MUTED[:3], 105)))
      rl.draw_circle_v(position, 2.4, self._color((*self.MUTED[:3], 125)))
      previous = position

    current = self._world_to_screen(map_rect, point.d_rel, point.y_rel)
    if previous is not None:
      rl.draw_line_ex(previous, current, 1.8, self._color((*self.MUTED[:3], 150)))
    previous = current
    model_distribution = (
      diagnostic is not None
      and len(diagnostic.horizon_x) == len(TARGET_HORIZONS_S)
      and len(diagnostic.horizon_y) == len(TARGET_HORIZONS_S)
      and len(diagnostic.horizon_x_stds) == len(TARGET_HORIZONS_S)
      and len(diagnostic.horizon_y_stds) == len(TARGET_HORIZONS_S)
    )
    selected_index = min(
      range(len(TARGET_HORIZONS_S)),
      key=lambda value: abs(TARGET_HORIZONS_S[value] - self.trajectory_horizon_s),
    )
    if model_distribution:
      plotted = []
      for index, horizon_s in enumerate(TARGET_HORIZONS_S):
        if horizon_s > self.trajectory_horizon_s + 1e-6:
          continue
        path_sample = trajectory_sample_at(trajectory, horizon_s)
        path_center_y = path_sample.y_rel - path_sample.d_path
        future_x = diagnostic.horizon_x[index]
        future_y = path_center_y + diagnostic.horizon_y[index]
        probability = diagnostic.horizon_scores[index]
        position = self._world_to_screen(map_rect, future_x, future_y)
        color = self._trajectory_color(probability, 205)
        rl.draw_line_ex(previous, position, 2.0, color)
        rl.draw_circle_v(position, 5.0 if index == selected_index else 3.2, color)
        previous = position
        plotted.append((index, position, future_x, future_y))
      selected_item = next(
        (item for item in plotted if item[0] == selected_index),
        plotted[-1] if plotted else None,
      )
      if selected_item is None:
        return
      _, selected_position, selected_x, selected_y = selected_item
      selected_x_sigma = diagnostic.horizon_x_stds[selected_index]
      selected_y_sigma = diagnostic.horizon_y_stds[selected_index]
      selected_d_path = diagnostic.horizon_y[selected_index]
      selected_probability = diagnostic.horizon_scores[selected_index]
    else:
      selected = trajectory_sample_at(trajectory, self.trajectory_horizon_s)
      for sample in trajectory.samples:
        if sample.horizon_s <= 0.0 or sample.horizon_s > self.trajectory_horizon_s + 1e-6:
          continue
        position = self._world_to_screen(map_rect, sample.d_rel, sample.y_rel)
        color = self._trajectory_color(sample.occupancy_prob, 205)
        rl.draw_line_ex(previous, position, 2.0, color)
        rl.draw_circle_v(position, 3.2 if sample is not selected else 5.0, color)
        previous = position
      selected_position = self._world_to_screen(map_rect, selected.d_rel, selected.y_rel)
      selected_x = selected.d_rel
      selected_y = selected.y_rel
      selected_x_sigma = 0.0
      selected_y_sigma = selected.lateral_sigma
      selected_d_path = selected.d_path
      selected_probability = selected.occupancy_prob
    sigma_left = self._world_to_screen(
      map_rect, selected_x, selected_y - selected_y_sigma,
    )
    sigma_right = self._world_to_screen(
      map_rect, selected_x, selected_y + selected_y_sigma,
    )
    selected_color = self._trajectory_color(selected_probability, 220)
    rl.draw_line_ex(sigma_left, sigma_right, 2.0, selected_color)
    if selected_x_sigma > 0.0:
      sigma_near = self._world_to_screen(
        map_rect, selected_x - selected_x_sigma, selected_y,
      )
      sigma_far = self._world_to_screen(
        map_rect, selected_x + selected_x_sigma, selected_y,
      )
      rl.draw_line_ex(sigma_near, sigma_far, 2.0, selected_color)
    if show_label:
      label = (
        f"+{TARGET_HORIZONS_S[selected_index]:.2f}s "
        + f"x{selected_x:.1f}±{selected_x_sigma:.1f} "
        + f"y{selected_d_path:+.2f}±{selected_y_sigma:.2f} "
        + f"I{selected_probability:.2f}"
      )
      self._draw_text(label, selected_position.x + 7.0, selected_position.y - 18.0, 12, selected_color)

  def _draw_marker(
    self,
    map_rect: Any,
    frame: RadarFrame,
    point: RadarPoint,
    diagnostic: Candidate | None = None,
    show_score_label: bool = True,
  ) -> None:
    rl = self.rl
    if point.d_rel <= 0.5 or point.d_rel > self.forward_range_m or abs(point.y_rel) > 12.0:
      return
    position = self._world_to_screen(map_rect, point.d_rel, point.y_rel)
    confirmed_cutin = (
      diagnostic is not None
      and diagnostic.stage in ("SELECTED", "OUTPUT", "DECISION")
    )
    color = (
      self._color(self.GREEN)
      if diagnostic is not None and (diagnostic.current_path_occupancy or confirmed_cutin)
      else self._color(self.PURPLE)
      if diagnostic is not None
      else self._source_color(point.source)
    )
    radius = 8.0 if self.review is not None else 6.0 if point.measured else 4.5
    rl.draw_circle_v(position, radius, color)
    high_path_in_probability = (
      diagnostic is not None
      and not diagnostic.current_path_occupancy
      and diagnostic.path_in_score >= self._display_probability_threshold(diagnostic)
    )
    high_path_out_probability = (
      diagnostic is not None
      and diagnostic.current_path_occupancy
      and diagnostic.path_out_score >= (
        diagnostic.path_exit_threshold
        if diagnostic.path_exit_threshold > 0.0
        else diagnostic.decision_threshold
      )
    )
    if high_path_in_probability:
      rl.draw_circle_lines(
        int(position.x), int(position.y), radius + 3.0,
        self._color(self.GREEN),
      )
    if high_path_out_probability:
      rl.draw_circle_lines(
        int(position.x), int(position.y), radius + 6.0,
        self._color(self.PURPLE),
      )
    if diagnostic is not None and show_score_label:
      future_in = self._future_occupancy_at_display_horizon(diagnostic)
      future_out = self._future_out_at_display_horizon(diagnostic)
      current_probability = int(diagnostic.current_path_occupancy)
      stage_color = self._color(self._stage_color(diagnostic.stage))
      label = (
        f"{self._point_identity(point)} P0={current_probability} "
        + f"IN{diagnostic.path_in_score:.2f} OUT{diagnostic.path_out_score:.2f} "
        + f"I{self.trajectory_horizon_s:.2f}={future_in:.2f} "
        + f"O{self.trajectory_horizon_s:.2f}={future_out:.2f} "
        + f"{self._stage_code(diagnostic.stage)}"
      )
      self._draw_text(label, position.x + 11, position.y - 10, 14, stage_color)
    elif show_score_label and self.show_labels:
      label = f"{point.track_id} P-- {point.d_rel:.0f}m"
      self._draw_text(label, position.x + 11, position.y - 9, 13, self._color(self.MUTED))

  def _draw_fused_marker(self, map_rect: Any, obj: RadarObject) -> None:
    if obj.d_rel <= 0.5 or obj.d_rel > self.forward_range_m or abs(obj.y_rel) > 12.0:
      return
    matched = obj.front_track_id is not None and obj.corner_track_id is not None
    if matched:
      color = self._color(self.GREEN)
      source = f"F{obj.front_track_id}/C{obj.corner_track_id}"
    elif obj.corner_track_id is not None:
      color = self._color(self.PURPLE)
      source = f"C{obj.corner_track_id}"
    elif obj.scc_track_id is not None:
      color = self._color(self.YELLOW)
      source = "SCC"
    else:
      color = self._color(self.CYAN)
      source = f"F{obj.front_track_id}"
    position = self._world_to_screen(map_rect, obj.d_rel, obj.y_rel)
    self.rl.draw_circle_v(position, 7.0 if matched else 5.0, color)
    if self.show_labels:
      self._draw_text(source, position.x + 8.0, position.y - 8.0, 13, color)

  def _draw_recorded(
    self, map_rect: Any, frame: RadarFrame, lead: RecordedLead, color: Any, radius: float, label: str,
  ) -> None:
    if not lead.status:
      return
    point = self._track_point(frame, lead.track_id) if lead.radar else None
    distance = point.d_rel if point is not None else lead.d_rel
    lateral = point.y_rel if point is not None else lead.y_rel
    if distance <= 0.0 or distance > self.forward_range_m:
      return
    position = self._world_to_screen(map_rect, distance, lateral)
    self.rl.draw_circle_lines(int(position.x), int(position.y), radius, color)
    self.rl.draw_circle_lines(int(position.x), int(position.y), radius + 1.0, color)
    self._draw_text(label, position.x + radius + 4.0, position.y - radius, 13, color)

  def _draw_candidate(
    self, map_rect: Any, frame: RadarFrame, candidate: Candidate | None, color: Any, size: float, label: str,
  ) -> None:
    if candidate is None:
      return
    point = self._track_point(frame, candidate.track_id)
    distance = point.d_rel if point is not None else candidate.d_rel
    lateral = point.y_rel if point is not None else candidate.y_rel
    if distance is None or lateral is None or distance > self.forward_range_m:
      return
    position = self._world_to_screen(map_rect, distance, lateral)
    rect = self.rl.Rectangle(position.x - size * 0.5, position.y - size * 0.5, size, size)
    self.rl.draw_rectangle_lines_ex(rect, 3.0, color)
    if candidate.track_id == -1:
      display_label = f"{label}/VISION"
    else:
      display_label = f"{label}/SCC" if point is not None and point.source == "scc" else label
    label_x = (
      rect.x - self._measure_text(display_label, 13) - 4.0
      if label.startswith("lead") else rect.x + rect.width + 4.0
    )
    self._draw_text(display_label, label_x, rect.y - 1.0, 13, color)

  def _draw_radard_candidate(
    self, map_rect: Any, frame: RadarFrame, candidate: Candidate | None, color: Any, radius: float, label: str,
    label_above: bool,
  ) -> None:
    if candidate is None:
      return
    point = self._track_point(frame, candidate.track_id)
    distance = point.d_rel if point is not None else candidate.d_rel
    lateral = point.y_rel if point is not None else candidate.y_rel
    if distance is None or lateral is None or distance <= 0.0 or distance > self.forward_range_m:
      return
    position = self._world_to_screen(map_rect, distance, lateral)
    self.rl.draw_circle_lines(int(position.x), int(position.y), radius, color)
    self.rl.draw_circle_lines(int(position.x), int(position.y), radius + 1.0, color)
    label_y = position.y - radius - 14.0 if label_above else position.y + radius + 2.0
    self._draw_text(label, position.x + radius + 4.0, label_y, 13, color)

  def _candidate_visible(self, candidate: Candidate | None) -> bool:
    if candidate is None:
      return False
    if getattr(self, "device_review_mode", False):
      return True
    return not self.selector.name.startswith(("mlp:", "multitask:", "hybrid:")) or candidate.score >= self.min_candidate_probability

  def _draw_manual_label(self, map_rect: Any, frame: RadarFrame, role: str, color: Any, radius: float) -> None:
    is_set, track_id = self.labels.get(self.index, role)
    if not is_set or track_id is None:
      return
    point = self._track_point(frame, track_id)
    if point is None or point.d_rel > self.forward_range_m:
      return
    position = self._world_to_screen(map_rect, point.d_rel, point.y_rel)
    rl = self.rl
    rl.draw_circle_lines(int(position.x), int(position.y), radius, self._color(self.TEXT))
    rl.draw_line_ex(rl.Vector2(position.x - radius, position.y), rl.Vector2(position.x + radius, position.y), 2.0, color)
    rl.draw_line_ex(rl.Vector2(position.x, position.y - radius), rl.Vector2(position.x, position.y + radius), 2.0, color)

  def _video_texture_for_time(self, playback_time: float) -> Any | None:
    if self.video_reader is None:
      return None
    aligned_time = self.frames[self.index].video_time_s
    video_frame = self.video_reader.frame_at(aligned_time if aligned_time is not None else playback_time)
    if video_frame is None:
      return None
    size = (video_frame.width, video_frame.height)
    if self.video_texture is None or self.video_texture_size != size:
      if self.video_texture is not None:
        self.rl.unload_texture(self.video_texture)
      image = self.rl.gen_image_color(video_frame.width, video_frame.height, self._color((0, 0, 0, 255)))
      self.video_texture = self.rl.load_texture_from_image(image)
      self.rl.unload_image(image)
      self.rl.set_texture_filter(self.video_texture, self.rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
      self.video_texture_size = size
      self.video_frame_id = None
    if self.video_frame_id != video_frame.frame_id:
      expected = video_frame.width * video_frame.height * 4
      if len(video_frame.rgba) == expected:
        pixels = self.rl.ffi.new("unsigned char[]", video_frame.rgba)
        self.rl.update_texture(self.video_texture, pixels)
        self.video_frame_id = video_frame.frame_id
    return self.video_texture

  def _draw_video(self, rect: Any) -> None:
    rl = self.rl
    rl.draw_rectangle_rec(rect, self._color((8, 11, 14, 255)))
    texture = self._video_texture_for_time(self.playback_time)
    if texture is None:
      if self.video_reader is not None:
        status = self.video_reader.status_text()
      else:
        status = f"camera missing: {self.video_path.name}"
      width = self._measure_text(status, 18)
      self._draw_text(status, rect.x + (rect.width - width) * 0.5, rect.y + rect.height * 0.5 - 9, 18, self._color(self.MUTED))
      return
    source = rl.Rectangle(0.0, 0.0, float(texture.width), float(texture.height))
    scale = min(rect.width / texture.width, rect.height / texture.height)
    draw_width = texture.width * scale
    draw_height = texture.height * scale
    destination = rl.Rectangle(
      rect.x + (rect.width - draw_width) * 0.5,
      rect.y + (rect.height - draw_height) * 0.5,
      draw_width,
      draw_height,
    )
    rl.draw_texture_pro(texture, source, destination, rl.Vector2(0.0, 0.0), 0.0, self._color((255, 255, 255, 255)))
    self._draw_text("QCAMERA", rect.x + 10, rect.y + 9, 14, self._color(self.TEXT))

  def _draw_map(self, rect: Any, frame: RadarFrame, selection: Selection) -> None:
    rl = self.rl
    rl.draw_rectangle_rec(rect, self._color(self.BG))
    center_x = rect.x + rect.width * 0.5
    for distance in range(0, int(self.forward_range_m) + 1, 10):
      position = self._world_to_screen(rect, float(distance), 0.0)
      rl.draw_line_ex(rl.Vector2(rect.x + 12, position.y), rl.Vector2(rect.x + rect.width - 12, position.y), 1.0, self._color(self.GRID))
      self._draw_text(f"{distance}m", rect.x + 16, position.y - 16, 13, self._color(self.MUTED))
    rl.draw_line_ex(rl.Vector2(center_x, rect.y + 20), rl.Vector2(center_x, rect.y + rect.height - 20), 1.0, self._color(self.GRID))

    for index, lane in enumerate(frame.lane_lines):
      probability = frame.lane_probs[index] if index < len(frame.lane_probs) else 0.5
      alpha = int(45 + 155 * min(max(probability, 0.0), 1.0))
      self._draw_world_line(rect, lane, self._color((195, 203, 211, alpha)), 2.0)
    self._draw_world_line(rect, frame.path, self._color((63, 132, 255, 220)), 4.0)

    ego = self._world_to_screen(rect, 0.0, 0.0)
    rl.draw_triangle(
      rl.Vector2(ego.x, ego.y - 17),
      rl.Vector2(ego.x - 11, ego.y + 8),
      rl.Vector2(ego.x + 11, ego.y + 8),
      self._color(self.TEXT),
    )
    if self.show_radar_points:
      if self.show_fused_objects:
        for obj in self.fused_frames[self.index]:
          self._draw_fused_marker(rect, obj)
      else:
        visible_points = self._visible_radar_points(frame, selection)
        selected_ids: set[int] = set()
        for candidate in (selection.lead_one, selection.lead_two):
          if candidate is not None:
            selected_ids.update(candidate_track_ids(candidate))
        annotation_rows: list[tuple[bool, float, float, tuple[str, int]]] = []
        point_diagnostics: dict[tuple[str, int], Candidate | None] = {}
        for point in visible_points:
          key = (point.source, point.track_id)
          diagnostic = self._diagnostic_for_point(selection, point)
          point_diagnostics[key] = diagnostic
          trajectory = self._trajectory_for_point(point)
          trajectory_probability = (
            trajectory_entry_probability(trajectory, self.trajectory_horizon_s)
            if trajectory is not None else 0.0
          )
          model_probability = diagnostic.score if diagnostic is not None else 0.0
          future_probability = (
            self._max_future_occupancy_probability(diagnostic)
            if diagnostic is not None
            else 0.0
          )
          selected = point.track_id in selected_ids
          if (
            point.d_rel <= REVIEW_POINT_TEXT_MAX_DREL_M
            and (diagnostic is not None or selected)
          ):
            annotation_rows.append((
              selected,
              max(model_probability, future_probability, trajectory_probability),
              -point.d_rel,
              key,
            ))
        annotation_rows.sort(reverse=True)
        annotation_keys = {row[3] for row in annotation_rows[:3]}
        if self.show_trajectories:
          for point in visible_points:
            trajectory = self._trajectory_for_point(point)
            if trajectory is not None:
              key = (point.source, point.track_id)
              self._draw_trajectory(
                rect,
                frame,
                point,
                trajectory,
                point_diagnostics[key],
                key in annotation_keys,
              )
        for point in visible_points:
          key = (point.source, point.track_id)
          self._draw_marker(
            rect,
            frame,
            point,
            point_diagnostics[key],
            key in annotation_keys,
          )

    if self.show_recorded_one:
      self._draw_recorded(rect, frame, frame.recorded_one, self._color(self.ORANGE), 12.0, "L1")
    if self.show_recorded_two:
      self._draw_recorded(rect, frame, frame.recorded_two, self._color(self.YELLOW), 16.0, "L2")
    if self.selector.name.startswith(("mlp:", "multitask:", "hybrid:")):
      multitask = self.selector.name.startswith(("multitask:", "hybrid:"))
      source_markers = (
        (self.show_front_candidates, "L" if multitask else "F", selection.front_candidates, (self.GREEN, self.CYAN)),
        (self.show_corner_candidates, "X" if multitask else "C", selection.corner_candidates, (self.PURPLE, self.YELLOW)),
        (self.show_external_candidates and multitask, "E", selection.external_candidates, (self.ORANGE, self.RED)),
      )
      for source_index, (source_visible, source_label, candidates, colors) in enumerate(source_markers):
        if not source_visible:
          continue
        for candidate_index, candidate in enumerate(candidates):
          if not self._candidate_visible(candidate):
            continue
          radius = 23.0 + source_index * 12.0 + candidate_index * 5.0
          self._draw_candidate(
            rect, frame, candidate, self._color(colors[candidate_index]), radius, f"{source_label}{candidate_index}",
          )
      # Final temporal outputs are control-facing decisions. Always show them,
      # even when the raw probability slider hides the instantaneous candidate.
      self._draw_candidate(rect, frame, selection.lead_one, self._color(self.ORANGE), 40.0, "leadOne")
      self._draw_candidate(rect, frame, selection.lead_two, self._color(self.YELLOW), 48.0, "leadTwo")
      if self.review is not None and self.review.validation_stage == "decision":
        targets = set(self.review.target_track_ids)
        decision = next(
          (
            candidate for candidate in selection.decision_cutin_candidates
            if candidate_matches_targets(candidate, targets)
          ),
          next(iter(selection.decision_cutin_candidates), None),
        )
        self._draw_candidate(rect, frame, decision, self._color(self.PURPLE), 56.0, "cutin decision")
    else:
      self._draw_candidate(rect, frame, selection.lead_one, self._color(self.GREEN), 23.0, "1")
      self._draw_candidate(rect, frame, selection.lead_two, self._color(self.PURPLE), 29.0, "2")
    if self.radard_selector is not None:
      radard = self.radard_selector.select(frame, self.index)
      self._draw_radard_candidate(rect, frame, radard.lead_one, self._color(self.CYAN), 14.0, "R1", True)
      self._draw_radard_candidate(rect, frame, radard.lead_two, self._color(self.PURPLE), 18.0, "R2", False)
    if self.review is None:
      self._draw_manual_label(rect, frame, "leadOne", self._color(self.ORANGE), 18.0)
      self._draw_manual_label(rect, frame, "leadTwo", self._color(self.YELLOW), 22.0)

    if self.review is not None:
      legend_lines = (
        "P0: measured path state | X/Y: learned means | SX/SY: learned std | I/O: Gaussian path mass",
        "IN=max(P0,future I) | OUT=max(1-P0,future O) | threshold + small hysteresis only",
        "fill green: inside/confirmed CUT-IN | purple: outside | green/purple ring: entering/leaving",
        "F/C: front/corner | orange/yellow box: leadOne/leadTwo | gray trail: measured past dPath",
      )
      legend_rect = rl.Rectangle(rect.x + 10, rect.y + 8, 680.0, 72.0)
      rl.draw_rectangle_rec(legend_rect, self._color((14, 18, 23, 220)))
      for line_index, text in enumerate(legend_lines):
        self._draw_text(
          text,
          rect.x + 18,
          rect.y + 13 + line_index * 16,
          11,
          self._color(self.MUTED),
        )
    else:
      point_legend = "fused objects" if self.show_fused_objects else "radar points"
      extras = f"   {point_legend} enabled" if self.show_radar_points else ""
      comparison_legend = "   circles: current radard L1 cyan / L2 purple" if self.radard_selector is not None else ""
      legend = f"boxes: model L1 orange / L2 yellow{comparison_legend}{extras}"
      self._draw_text(legend, rect.x + 18, rect.y + rect.height - 24, 14, self._color(self.MUTED))

  @staticmethod
  def _lead_text(frame: RadarFrame, lead: RecordedLead) -> str:
    if not lead.status:
      return "NONE"
    if lead.radar:
      resolved_id = resolved_recorded_track_id(frame, lead)
      identity = f"id {lead.track_id}"
      if resolved_id is not None and resolved_id != lead.track_id:
        identity += f"->{resolved_id}"
    else:
      identity = "VISION"
    return f"{identity}  d {lead.d_rel:5.1f}  y {lead.y_rel:+5.1f}  vRel {lead.v_rel:+5.1f}"

  def _candidate_text(
    self,
    frame: RadarFrame,
    candidate: Candidate | None,
    selection: Selection | None = None,
  ) -> str:
    if candidate is None:
      return "NONE"
    point = self._track_point(frame, candidate.track_id)
    if candidate.track_id == -1:
      return f"VISION  prob {candidate.score:.2f}  {candidate.reason}"
    diagnostic = (
      self._selected_trajectory_diagnostic(selection, candidate)
      if selection is not None
      else None
    )
    if diagnostic is not None:
      source = _source_mode_name(diagnostic.source).upper()
      return (
        f"{source} id {diagnostic.track_id}  "
        + f"P0={int(diagnostic.current_path_occupancy)} "
        + f"IN {diagnostic.path_in_score:.2f} OUT {diagnostic.path_out_score:.2f} "
        + f"ON {diagnostic.decision_threshold:.2f}  {candidate.reason}"
      )
    source = "SCC " if point is not None and point.source == "scc" else ""
    value_name = "prob" if candidate.reason.startswith("MLP") else "score"
    return f"{source}id {candidate.track_id}  {value_name} {candidate.score:.2f}  {candidate.reason}"

  def _draw_panel(self, rect: Any, frame: RadarFrame, selection: Selection) -> None:
    rl = self.rl
    x = int(rect.x + 20)
    y = int(rect.y + 18)
    rl.draw_rectangle_rec(rect, self._color(self.PANEL))

    def line(text: str, color: tuple[int, int, int, int] = self.TEXT, size: int = 17, gap: int = 24) -> None:
      nonlocal y
      self._draw_text(text, x, y, size, self._color(color))
      y += gap

    line("RADAR LEAD WORKBENCH", self.TEXT, 22, 34)
    line(self.title[:52], self.MUTED, 15, 24)
    line(f"t {frame.time_s:6.2f}s / {self.times[-1]:.2f}s    frame {self.index + 1}/{len(self.frames)}", self.TEXT)
    line(f"ego {frame.v_ego * 3.6:5.1f} km/h    points {len(frame.points)}    zoom {self.forward_range_m:.0f}m", self.TEXT)
    line(f"input age {frame.input_age_s * 1000:4.0f}ms    model age {frame.model_age_s * 1000:4.0f}ms", self.MUTED, 15, 30)

    if self.review is not None:
      review_number = self.reviews.index(self.review) + 1 if self.review in self.reviews else 1
      line(
        f"REPLAY CASE {review_number}/{max(len(self.reviews), 1)}  "
        + f"{self.review.case_id}  {self.review.source}",
        self.PURPLE, 14, 21,
      )
      line(self.review.scene[:58], self.MUTED, 13, 20)
      line(
        (
          f"LABEL {self.review.expected.upper()} "
          + f"{'HUMAN VERIFIED' if self.review.human_verified else 'UNVERIFIED'}  "
          + f"window {self.review.start_s:.2f}-{self.review.end_s:.2f}s"
        ),
        self.YELLOW, 13, 20,
      )
      status_color = self.YELLOW if self.paused else self.GREEN
      line(self.review_status, status_color, 14, 27)

    if self.selector.name.startswith(("multitask:", "hybrid:")):
      line("MODEL RESULT", self.MUTED, 15, 22)
      line("leadOne  " + self._candidate_text(frame, selection.lead_one), self.ORANGE, 17, 24)
      line(
        "leadTwo  " + self._candidate_text(frame, selection.lead_two, selection),
        self.YELLOW, 17, 31,
      )
      if self.review is not None and self.review.validation_stage == "decision":
        targets = set(self.review.target_track_ids)
        decision = next(
          (
            candidate for candidate in selection.decision_cutin_candidates
            if candidate_matches_targets(candidate, targets)
          ),
          next(iter(selection.decision_cutin_candidates), None),
        )
        line("decision  " + self._candidate_text(frame, decision), self.PURPLE, 15, 24)

      visible_points = self._visible_radar_points(frame, selection)
      has_visible_corner = any(point.source.startswith("corner") for point in visible_points)
      has_visible_front = any(point.source == "frontRadar" for point in visible_points)
      point_source = (
        "FRONT+CORNER" if has_visible_front and has_visible_corner
        else "CORNER" if has_visible_corner
        else "FRONT" if has_visible_front
        else "SCC"
      )
      line(
        f"{point_source} FUTURE X/Y DISTRIBUTION + IN/OUT  display {self.trajectory_horizon_s:.2f}s",
        self.MUTED, 14, 21,
      )
      line(
        (
          f"ego steer {frame.steering_angle_deg:+.0f}deg "
          + f"rate {frame.steering_rate_deg_s:+.0f}deg/s "
          + f"yaw {math.degrees(frame.yaw_rate_rad_s):+.1f}deg/s"
          + f" {frame.yaw_rate_source}"
        ),
        self.MUTED, 12, 17,
      )
      point_rows = []
      selected_ids = set()
      for candidate in (selection.lead_one, selection.lead_two):
        selected_ids.update(candidate_track_ids(candidate))
      for point in visible_points:
        if (
          point.d_rel > REVIEW_POINT_TEXT_MAX_DREL_M
          and point.track_id not in selected_ids
        ):
          continue
        diagnostic = self._diagnostic_for_point(selection, point)
        trajectory = self._trajectory_for_point(point)
        stage_priority = {
          "SELECTED": 5,
          "OUTPUT": 4,
          "DECISION": 3,
          "CUT-OUT": 2,
          "FARTHER-L1": 1,
        }.get(diagnostic.stage if diagnostic is not None else "", 0)
        point_rows.append((
          stage_priority,
          diagnostic.path_in_score if diagnostic is not None else 0.0,
          point,
          diagnostic,
          trajectory,
        ))
      point_rows.sort(key=lambda item: (-item[0], item[2].d_rel, -item[1]))
      # One full position-distribution row fits above the validation controls. Remaining
      # objects keep their map color/rings without obscuring the controls.
      for _, _, point, diagnostic, trajectory in point_rows[:1]:
        stage = diagnostic.stage if diagnostic is not None else "NO-MODEL"
        current_path = (
          f" dP{trajectory.d_path:+.2f}/{trajectory.lane_half_width:.2f}"
          if trajectory is not None
          else ""
        )
        line(
          (
            (
              f"{self._point_identity(point)} "
              + f"P0={int(diagnostic.current_path_occupancy)} "
              + f"IN{diagnostic.path_in_score:.2f} "
              + f"OUT{diagnostic.path_out_score:.2f} "
              + f"{stage}{current_path}"
            )
            if diagnostic is not None
            else f"id{point.track_id} P unavailable"
          ),
          self._stage_color(diagnostic.stage) if diagnostic is not None else self.MUTED,
          13,
          18,
        )
        if trajectory is not None and diagnostic is not None:
          raw_x = "/".join(f"{value:.1f}" for value in diagnostic.horizon_x) or "--"
          raw_y = "/".join(f"{value:+.2f}" for value in diagnostic.horizon_y) or "--"
          raw_x_std = "/".join(f"{value:.1f}" for value in diagnostic.horizon_x_stds) or "--"
          raw_y_std = "/".join(f"{value:.2f}" for value in diagnostic.horizon_y_stds) or "--"
          raw_in = (
            "/".join(f"{value:.2f}" for value in diagnostic.horizon_scores)
            if diagnostic.horizon_scores
            else "--"
          )
          raw_out = (
            "/".join(f"{value:.2f}" for value in diagnostic.out_horizon_scores)
            if diagnostic.out_horizon_scores
            else "--"
          )
          line(
            f"  X .5/1/1.5/2 {raw_x}  Xstd {raw_x_std}"[:76],
            self.MUTED, 11, 15,
          )
          line(
            f"  Y .5/1/1.5/2 {raw_y}  Ystd {raw_y_std}"[:76],
            self.MUTED, 11, 15,
          )
          line(
            f"  IN {raw_in}  OUT {raw_out}"[:76],
            self.MUTED, 11, 15,
          )
        elif diagnostic is not None:
          line("  no trajectory history; " + diagnostic.detail[:48], self.MUTED, 12, 17)
      if not point_rows:
        line("no displayed sensor points", self.MUTED, 13, 20)

    if self.radard_selector is not None:
      radard = self.radard_selector.select(frame, self.index)
      line("CURRENT RADARD RESULT", self.MUTED, 15, 22)
      line("leadOne  " + self._candidate_text(frame, radard.lead_one), self.CYAN, 15, 22)
      line("leadTwo  " + self._candidate_text(frame, radard.lead_two), self.PURPLE, 15, 29)

    if self.show_recorded_one or self.show_recorded_two:
      line("RECORDED RADARSTATE", self.MUTED, 15, 22)
      if self.show_recorded_one:
        line("leadOne  " + self._lead_text(frame, frame.recorded_one), self.ORANGE, 15, 22)
      if self.show_recorded_two:
        line("leadTwo  " + self._lead_text(frame, frame.recorded_two), self.YELLOW, 15, 29)

    show_candidates = self.show_front_candidates or self.show_corner_candidates or self.show_external_candidates
    one_manual, one_target = self.labels.get(self.index, "leadOne")
    two_manual, two_target = self.labels.get(self.index, "leadTwo")
    one_expected = one_target if one_manual else resolved_recorded_track_id(frame, frame.recorded_one)
    two_expected = two_target if two_manual else resolved_recorded_track_id(frame, frame.recorded_two)
    one_match = one_expected == candidate_track_id(selection.lead_one)
    two_match = two_expected == candidate_track_id(selection.lead_two)
    if show_candidates and self.selector.name.startswith(("mlp:", "multitask:", "hybrid:")):
      line(f"CANDIDATE  {self.selector.name[:34]}", self.MUTED, 15, 22)
      multitask = self.selector.name.startswith(("multitask:", "hybrid:"))
      source_rows = (
        ("L0" if multitask else "F0", selection.front_candidates[0] if selection.front_candidates else None, self.GREEN, self.show_front_candidates),
        ("L1" if multitask else "F1", selection.front_candidates[1] if len(selection.front_candidates) > 1 else None, self.CYAN, self.show_front_candidates),
        ("X0" if multitask else "C0", selection.corner_candidates[0] if selection.corner_candidates else None, self.PURPLE, self.show_corner_candidates),
        (
          "X1" if multitask else "C1",
          selection.corner_candidates[1] if len(selection.corner_candidates) > 1 else None,
          self.YELLOW,
          self.show_corner_candidates,
        ),
        ("E0", selection.external_candidates[0] if selection.external_candidates else None, self.ORANGE, self.show_external_candidates and multitask),
        ("E1", selection.external_candidates[1] if len(selection.external_candidates) > 1 else None, self.RED, self.show_external_candidates and multitask),
      )
      for label, candidate, color, source_visible in source_rows:
        if not source_visible:
          value = "SOURCE HIDDEN"
          row_color = self.MUTED
        elif candidate is not None and not self._candidate_visible(candidate):
          value = f"BELOW {self.min_candidate_probability:.2f}"
          row_color = self.MUTED
        else:
          value = self._candidate_text(frame, candidate)
          row_color = color
        line(f"{label}  {value}", row_color, 15, 21)
      if not multitask:
        line("CUT-IN DECISION  NOT APPLIED", self.MUTED, 13, 24)
    elif self.review is None:
      line(f"CANDIDATE  {self.selector.name[:34]}", self.MUTED, 15, 22)
      line("1  " + self._candidate_text(frame, selection.lead_one), self.GREEN if one_match else self.RED, 16, 23)
      line("2  " + self._candidate_text(frame, selection.lead_two), self.PURPLE if two_match else self.RED, 16, 31)

    if self.review is None:
      line(f"MANUAL LABEL  ACTIVE {self.selected_role}", self.MUTED, 15, 22)
      for role, color in (("leadOne", self.ORANGE), ("leadTwo", self.YELLOW), ("cutin", self.PURPLE)):
        is_set, track_id = self.labels.get(self.index, role)
        value = "INHERIT RECORDED" if not is_set else ("NONE" if track_id is None else f"id {track_id}")
        line(f"{role}: {value}", color if is_set else self.MUTED, 15, 21)
      line(f"{self.label_status}  total {self.labels.count()}", self.MUTED, 13, 27)

    if self.review is None:
      line("MODEL LEADS", self.MUTED, 15, 22)
      for index, lead in enumerate(frame.model_leads[:3]):
        line(f"{index}  p {lead.probability:.2f}  x {lead.x:5.1f}  y {lead.y:+4.1f}  v {lead.v * 3.6:5.1f}km/h", self.TEXT, 15, 21)
      if not frame.model_leads:
        line("none", self.MUTED, 15, 21)
      y += 9

      if self.show_fused_objects:
        line("NEAREST FUSED OBJECTS", self.MUTED, 15, 22)
        for obj in self.fused_frames[self.index][:2]:
          ids = (
            f"F{obj.front_track_id}/C{obj.corner_track_id}"
            if obj.front_track_id is not None and obj.corner_track_id is not None else
            f"C{obj.corner_track_id}" if obj.corner_track_id is not None else
            "SCC" if obj.scc_track_id is not None else f"F{obj.front_track_id}"
          )
          line(f"{ids:11s} d {obj.d_rel:5.1f}  y {obj.y_rel:+5.1f}  v {obj.v_rel:+5.1f}", self.MUTED, 14, 19)
      else:
        line("NEAREST USABLE POINTS", self.MUTED, 15, 22)
        nearest = sorted((point for point in frame.points if point.d_rel > 0.75), key=lambda point: point.d_rel)[:2]
        for point in nearest:
          source = {"frontRadar": "F", "scc": "S", "corner235": "C235", "corner180": "C180"}.get(point.source, point.source[:5])
          line(f"{point.track_id:5d}  {source:4s}  d {point.d_rel:5.1f}  y {point.y_rel:+5.1f}  v {point.v_rel:+5.1f}", self.MUTED, 14, 19)

    self._draw_review_label_controls(rect)
    self._draw_filter_controls(rect)

  def _draw_review_label_controls(self, rect: Any) -> None:
    self.review_label_buttons = {}
    if self.review is None:
      return

    rl = self.rl
    x = rect.x + 20.0
    y = rect.y + rect.height - 205.0
    review_point = self._trajectory_event_point()
    maintained_review = (
      None if review_point is not None
      else self._review_containing_time(self.playback_time)
    )
    if maintained_review is None and review_point is None:
      review_point = self._review_event_point()
    manual_expected = (
      self.trajectory_review_labels.get(self._trajectory_review_label_key(review_point))
      if review_point is not None
      else None
    )
    if maintained_review is None:
      target = f" id{review_point.track_id}" if review_point is not None else ""
      saved = f"  SAVED {manual_expected.upper()}" if manual_expected else "  UNVERIFIED"
      heading = f"TRAJECTORY LABEL{target}{saved}  SEPARATE FILE"
    else:
      verification = "HUMAN VERIFIED" if maintained_review.human_verified else "UNVERIFIED"
      heading = f"GROUND TRUTH  {verification}"
    self._draw_text(heading, x, y, 13, self._color(self.MUTED))
    y += 21.0
    labels = (
      ("detect", "CUT-IN", self.ORANGE),
      ("clear", "CLEAR", self.GREEN),
      ("stationary", "STATIONARY", self.CYAN),
    )
    button_x = x
    for expected, label, color_value in labels:
      width = self._measure_text(label, 13) + 28.0
      button = rl.Rectangle(button_x, y, width, 25.0)
      self.review_label_buttons[expected] = button
      active = (
        maintained_review.expected == expected
        if maintained_review is not None
        else manual_expected == expected
      )
      color = self._color(color_value)
      if active:
        rl.draw_rectangle_rec(button, self._color((*color_value[:3], 42)))
      rl.draw_rectangle_lines_ex(button, 2.0 if active else 1.0, color if active else self._color(self.MUTED))
      self._draw_text(label, button.x + 14.0, button.y + 5.0, 13, color if active else self._color(self.TEXT))
      button_x += width + 8.0

  def _draw_filter_controls(self, rect: Any) -> None:
    rl = self.rl
    x = rect.x + 20.0
    y = rect.y + rect.height - (136.0 if self.review is not None else 105.0)
    self._draw_text("DISPLAY FILTERS", x, y, 13, self._color(self.MUTED))
    y += 22.0
    self.filter_checkboxes = {}
    multitask = self.selector.name.startswith(("multitask:", "hybrid:"))
    controls = (
      ("recorded_one", "LOG L1", self.show_recorded_one),
      ("recorded_two", "LOG L2", self.show_recorded_two),
      ("front", "LEAD" if multitask else "FRONT", self.show_front_candidates),
      ("corner", "CUTIN" if multitask else "CORNER", self.show_corner_candidates),
      ("external", "EXT", self.show_external_candidates),
      ("points", "POINTS", self.show_radar_points),
      ("trajectory", "PATH", self.show_trajectories),
    ) + (
      (("fused", "FUSED", self.show_fused_objects),)
      if self.review is None
      else ()
    )
    control_x = x
    for key, label, checked in controls:
      control_width = self._measure_text(label, 13) + 39.0
      if control_x > x and control_x + control_width > rect.x + rect.width - 18.0:
        control_x = x
        y += 24.0
      box = rl.Rectangle(control_x, y, 14.0, 14.0)
      self.filter_checkboxes[key] = box
      rl.draw_rectangle_lines_ex(box, 1.5, self._color(self.TEXT if checked else self.MUTED))
      if checked:
        rl.draw_line_ex(rl.Vector2(box.x + 3.0, box.y + 7.0), rl.Vector2(box.x + 6.0, box.y + 11.0), 2.0, self._color(self.GREEN))
        rl.draw_line_ex(rl.Vector2(box.x + 6.0, box.y + 11.0), rl.Vector2(box.x + 12.0, box.y + 2.0), 2.0, self._color(self.GREEN))
      self._draw_text(label, box.x + 20.0, box.y - 1.0, 13, self._color(self.TEXT if checked else self.MUTED))
      control_x += control_width

    slider_y = y + 29.0
    if getattr(self, "device_review_mode", False):
      self.probability_slider = None
      self._draw_text(
        "AUTO DISPLAY: MODEL THRESHOLDS | PAUSE: leadTwo",
        x, slider_y - 4.0, 13, self._color(self.GREEN),
      )
    else:
      label = f"DISPLAY PROB >= {self.min_candidate_probability:.2f} | PAUSE: leadTwo"
      self._draw_text(label, x, slider_y - 4.0, 13, self._color(self.TEXT))
      slider_x = x + 230.0
      slider_width = max(80.0, rect.width - 272.0)
      self.probability_slider = rl.Rectangle(slider_x, slider_y - 7.0, slider_width, 18.0)
      track_y = slider_y + 1.0
      rl.draw_line_ex(rl.Vector2(slider_x, track_y), rl.Vector2(slider_x + slider_width, track_y), 4.0, self._color(self.GRID))
      knob_x = slider_x + slider_width * self.min_candidate_probability
      rl.draw_line_ex(rl.Vector2(slider_x, track_y), rl.Vector2(knob_x, track_y), 4.0, self._color(self.GREEN))
      rl.draw_circle_v(rl.Vector2(knob_x, track_y), 7.0, self._color(self.TEXT))

    if self.review is not None:
      horizon_y = slider_y + 29.0
      self._draw_text(
        f"DISPLAY FUTURE {self.trajectory_horizon_s:.2f}s",
        x, horizon_y - 4.0, 13, self._color(self.TEXT),
      )
      horizon_x = x + 150.0
      horizon_width = max(100.0, rect.width - 192.0)
      self.trajectory_horizon_slider = rl.Rectangle(
        horizon_x, horizon_y - 7.0, horizon_width, 18.0,
      )
      horizon_track_y = horizon_y + 1.0
      horizon_ratio = (self.trajectory_horizon_s - 0.25) / 1.75
      horizon_knob_x = horizon_x + horizon_width * horizon_ratio
      rl.draw_line_ex(
        rl.Vector2(horizon_x, horizon_track_y),
        rl.Vector2(horizon_x + horizon_width, horizon_track_y),
        4.0, self._color(self.GRID),
      )
      rl.draw_line_ex(
        rl.Vector2(horizon_x, horizon_track_y),
        rl.Vector2(horizon_knob_x, horizon_track_y),
        4.0, self._color(self.CYAN),
      )
      rl.draw_circle_v(
        rl.Vector2(horizon_knob_x, horizon_track_y), 7.0, self._color(self.TEXT),
      )
    else:
      self.trajectory_horizon_slider = None

  def _draw_timeline(self, width: int, height: int) -> Any:
    rl = self.rl
    rect = rl.Rectangle(24.0, float(height - 38), float(width - 48), 13.0)
    plot_rect = self._draw_lead_comparison_plot(width, rect.y) if self.review is not None else None
    rl.draw_rectangle_rounded(rect, 1.0, 8, self._color((55, 65, 75, 255)))
    if self.review is not None and self.times[-1] > 0.0:
      for review in self.reviews:
        start_ratio = min(max(review.start_s / self.times[-1], 0.0), 1.0)
        end_ratio = min(max(review.end_s / self.times[-1], 0.0), 1.0)
        review_rect = rl.Rectangle(
          rect.x + rect.width * start_ratio,
          rect.y - 4.0,
          max(2.0, rect.width * (end_ratio - start_ratio)),
          3.0,
        )
        color = self.PURPLE if review.expected == "detect" else self.CYAN
        rl.draw_rectangle_rec(review_rect, self._color((*color[:3], 180)))
      for index in self.review_events:
        event_ratio = min(max(self.times[index] / self.times[-1], 0.0), 1.0)
        marker_x = rect.x + rect.width * event_ratio
        events = self.review_events[index]
        if any("TRAJECTORY" in event for event in events):
          marker_color = self.ORANGE
        elif any("CUT-IN" in event for event in events):
          marker_color = self.PURPLE
        else:
          marker_color = self.RED
        rl.draw_line_ex(
          rl.Vector2(marker_x, rect.y - 5.0), rl.Vector2(marker_x, rect.y + rect.height + 5.0),
          2.0, self._color(marker_color),
        )
    ratio = 0.0 if self.times[-1] <= 0.0 else self.playback_time / self.times[-1]
    fill = rl.Rectangle(rect.x, rect.y, rect.width * ratio, rect.height)
    rl.draw_rectangle_rounded(fill, 1.0, 8, self._color((72, 145, 255, 255)))
    rl.draw_circle_v(rl.Vector2(rect.x + rect.width * ratio, rect.y + rect.height * 0.5), 8.0, self._color(self.TEXT))
    state = "PAUSED" if self.paused else f"{self.speed:.2g}x"
    if self.review is not None:
      lead_one_text, lead_one_lost = self._lead_continuity_text()
      lead_two_text, lead_two_lost = self._lead_two_continuity_text()
      text_y = plot_rect.y - 40.0 if plot_rect is not None else rect.y - 40.0
      self._draw_text(lead_one_text, rect.x, text_y, 13, self._color(self.RED if lead_one_lost else self.GREEN))
      self._draw_text(lead_two_text, rect.x, text_y + 17.0, 13, self._color(self.RED if lead_two_lost else self.PURPLE))
    self._draw_text(state, rect.x + rect.width - 85, rect.y - 23, 15, self._color(self.TEXT))
    return rect

  def _draw_lead_comparison_plot(self, width: int, timeline_y: float) -> Any:
    rl = self.rl
    rect = rl.Rectangle(24.0, timeline_y - 214.0, float(width - 48), 190.0)
    rl.draw_rectangle_rec(rect, self._color((11, 15, 20, 255)))
    rl.draw_rectangle_lines_ex(rect, 1.0, self._color(self.GRID))

    legend = (
      *((
        ("CURRENT RADARD L1", "radard_one", self.CYAN),
        ("CURRENT RADARD L2", "radard_two", self.PURPLE),
      ) if self.radard_selector is not None else ()),
      ("MODEL L1", "model_one", self.ORANGE),
      ("MODEL L2", "model_two", self.YELLOW),
    )
    legend_x = rect.x + 10.0
    current = self.lead_comparison[self.index]
    for label, field, color_value in legend:
      value = getattr(current, field)
      value_text = "--" if value is None else f"{value.d_rel:.0f}m"
      item = f"{label} {value_text}"
      rl.draw_line_ex(
        rl.Vector2(legend_x, rect.y + 13.0), rl.Vector2(legend_x + 18.0, rect.y + 13.0),
        3.0, self._color(color_value),
      )
      self._draw_text(item, legend_x + 24.0, rect.y + 5.0, 13, self._color(color_value))
      legend_x += self._measure_text(item, 13) + 52.0

    graph_top = rect.y + 29.0
    graph_bottom = rect.y + 105.0
    graph_height = graph_bottom - graph_top
    max_distance = 100.0
    for distance in (0.0, 50.0, 100.0):
      y = graph_bottom - graph_height * distance / max_distance
      rl.draw_line_ex(rl.Vector2(rect.x + 38.0, y), rl.Vector2(rect.x + rect.width - 8.0, y), 1.0, self._color(self.GRID))
      self._draw_text(f"{int(distance)}m", rect.x + 5.0, y - 7.0, 11, self._color(self.MUTED))

    plot_left = rect.x + 38.0
    plot_width = rect.width - 46.0
    total_time = max(self.times[-1], 1e-3)
    max_gap_s = 0.15
    for _, field, color_value in legend:
      previous: tuple[float, float, LeadPlotValue] | None = None
      color = self._color(color_value)
      for frame, values in zip(self.frames, self.lead_comparison, strict=True):
        value = getattr(values, field)
        if value is None:
          previous = None
          continue
        x = plot_left + plot_width * min(max(frame.time_s / total_time, 0.0), 1.0)
        y = graph_bottom - graph_height * min(value.d_rel, max_distance) / max_distance
        if previous is not None:
          previous_time, previous_x, previous_value = previous
          if frame.time_s - previous_time <= max_gap_s:
            previous_y = graph_bottom - graph_height * min(previous_value.d_rel, max_distance) / max_distance
            rl.draw_line_ex(rl.Vector2(previous_x, previous_y), rl.Vector2(x, y), 2.0, color)
        previous = (frame.time_s, x, value)

      value = getattr(current, field)
      if value is not None:
        x = plot_left + plot_width * min(max(self.times[self.index] / total_time, 0.0), 1.0)
        y = graph_bottom - graph_height * min(value.d_rel, max_distance) / max_distance
        rl.draw_circle_v(rl.Vector2(x, y), 4.0, color)

    cursor_x = plot_left + plot_width * min(max(self.playback_time / total_time, 0.0), 1.0)
    rl.draw_line_ex(
      rl.Vector2(cursor_x, graph_top), rl.Vector2(cursor_x, rect.y + rect.height - 8.0),
      1.5, self._color(self.TEXT),
    )

    probability_top = rect.y + 121.0
    probability_bottom = rect.y + rect.height - 9.0
    probability_height = probability_bottom - probability_top
    self._draw_text("IN / OUT", rect.x + 5.0, probability_top - 3.0, 11, self._color(self.MUTED))
    if getattr(self, "device_review_mode", False):
      self._draw_text("AUTO TH", rect.x + 5.0, probability_top + 14.0, 10, self._color(self.GREEN))
    else:
      threshold_y = probability_bottom - probability_height * self.min_candidate_probability
      rl.draw_line_ex(
        rl.Vector2(plot_left, threshold_y), rl.Vector2(rect.x + rect.width - 8.0, threshold_y),
        1.0, self._color(self.GRID),
      )
      self._draw_text(f"{self.min_candidate_probability:.2f}", rect.x + 5.0, threshold_y - 6.0, 10, self._color(self.MUTED))
    stage_legend = (
      ("IN", "model", self.GREEN),
      ("OUT", "path_out", self.PURPLE),
      ("DECISION", "decision", self.CYAN),
      ("OUTPUT", "output", self.YELLOW),
      ("SELECTED", "selected", self.GREEN),
    )
    def stage_probability(candidate: Candidate, field: str) -> float:
      return candidate.path_out_score if field == "path_out" else candidate.path_in_score

    stage_x = rect.x + 70.0
    current_stage = self.cutin_stages[self.index]
    for label, field, color_value in stage_legend:
      candidate = getattr(current_stage, field)
      value = "--" if candidate is None else f"{stage_probability(candidate, field):.2f}"
      item = f"{label} {value}"
      self._draw_text(item, stage_x, probability_top - 2.0, 11, self._color(color_value))
      stage_x += self._measure_text(item, 11) + 20.0

    for _, field, color_value in stage_legend:
      previous: tuple[float, float, float] | None = None
      color = self._color(color_value)
      for frame, values in zip(self.frames, self.cutin_stages, strict=True):
        candidate = getattr(values, field)
        if candidate is None:
          previous = None
          continue
        x = plot_left + plot_width * min(max(frame.time_s / total_time, 0.0), 1.0)
        score = min(max(stage_probability(candidate, field), 0.0), 1.0)
        y = probability_bottom - probability_height * score
        if previous is not None:
          previous_time, previous_x, previous_y = previous
          if frame.time_s - previous_time <= max_gap_s:
            rl.draw_line_ex(rl.Vector2(previous_x, previous_y), rl.Vector2(x, y), 1.8, color)
        previous = (frame.time_s, x, y)
      candidate = getattr(current_stage, field)
      if candidate is not None:
        score = min(max(stage_probability(candidate, field), 0.0), 1.0)
        y = probability_bottom - probability_height * score
        rl.draw_circle_v(rl.Vector2(cursor_x, y), 3.5, color)
    return rect

  def _save_labels(self) -> None:
    try:
      self.labels.save(self.label_path, self.log_path, self.frames)
      self.label_status = f"saved {self.label_path.name}"
    except OSError as exc:
      self.label_status = f"SAVE FAILED: {exc}"

  def _set_label(self, track_id: int | None) -> None:
    self.labels.set(self.index, self.selected_role, track_id)
    value = "NONE" if track_id is None else f"id {track_id}"
    self.label_status = f"frame {self.index + 1} {self.selected_role} = {value}"
    self._save_labels()

  def _set_review_expected(self, expected: str) -> bool:
    if self.review is None or self.validation_cases_path is None:
      return False

    trajectory_point = self._trajectory_event_point()
    maintained_review = (
      None if trajectory_point is not None
      else self._review_containing_time(self.playback_time)
    )
    try:
      if trajectory_point is not None:
        labels_path = self.validation_cases_path.with_name("radar_trajectory_labels.json")
        saved_target = upsert_trajectory_review_label(
          labels_path,
          self.log_path,
          self.frames[self.index],
          trajectory_point,
          expected,
          self.trajectory_horizon_s,
        )
        self.trajectory_review_labels[
          self._trajectory_review_label_key(trajectory_point)
        ] = expected
      elif maintained_review is not None:
        update_validation_case_label(
          self.validation_cases_path, maintained_review.case_id, expected,
        )
        updated = replace(maintained_review, expected=expected, human_verified=True)
        self.reviews = tuple(
          updated if review.case_id == updated.case_id else review
          for review in self.reviews
        )
        self.review = updated
        saved_target = updated.case_id
      else:
        point = self._review_event_point()
        if point is None:
          self.review_status = "LABEL SAVE FAILED: no trajectory point at this frame"
          return False
        labels_path = self.validation_cases_path.with_name("radar_trajectory_labels.json")
        saved_target = upsert_trajectory_review_label(
          labels_path,
          self.log_path,
          self.frames[self.index],
          point,
          expected,
          self.trajectory_horizon_s,
        )
        self.trajectory_review_labels[self._trajectory_review_label_key(point)] = expected
    except (OSError, ValueError, json.JSONDecodeError) as exc:
      self.review_status = f"LABEL SAVE FAILED: {exc}"
      return False

    self._prepare_review_events()
    self.review_status = (
      f"LABEL SAVED: {saved_target} = {expected.upper()} (HUMAN VERIFIED)"
    )
    self.paused = False
    return True

  def _trajectory_review_label_key(self, point: RadarPoint) -> tuple[float, int]:
    return self._trajectory_label_key_at(self.index, point.track_id)

  def _load_trajectory_review_labels(self) -> dict[tuple[float, int], str]:
    if self.validation_cases_path is None:
      return {}
    labels_path = self.validation_cases_path.with_name("radar_trajectory_labels.json")
    if not labels_path.is_file():
      return {}
    try:
      payload = json.loads(labels_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
      return {}

    vehicle_folder = self.log_path.parent.parent.name
    relative_log = f"{self.log_path.parent.name}/{self.log_path.name}"
    labels: dict[tuple[float, int], str] = {}
    for item in payload.get("labels", ()):
      if (
        str(item.get("vehicle_folder", "")) != vehicle_folder
        or str(item.get("log", "")) != relative_log
      ):
        continue
      try:
        key = float(item["time_s"]), int(item["track_id"])
      except (KeyError, TypeError, ValueError):
        continue
      labels[key] = str(item.get("expected", ""))
    return labels

  def _review_event_point(self) -> RadarPoint | None:
    trajectory_point = self._trajectory_event_point()
    if trajectory_point is not None:
      return trajectory_point

    frame = self.frames[self.index]
    preferred = preferred_radar_points(frame, self.review.source if self.review else None)
    candidates = []
    for point in preferred:
      trajectory = self._trajectory_for_point(point)
      if trajectory is not None:
        candidates.append((
          trajectory_entry_probability(trajectory, self.trajectory_horizon_s),
          point,
        ))
    return max(candidates, key=lambda item: item[0], default=(0.0, None))[1]

  def _trajectory_event_point(self) -> RadarPoint | None:
    events = self.review_events.get(self.index, ())
    event_track_ids = self._trajectory_event_track_ids(events)
    if not event_track_ids:
      return None
    frame = self.frames[self.index]
    unlabeled_track_ids = tuple(
      track_id for track_id in event_track_ids
      if self._trajectory_label_key_at(self.index, track_id) not in self.trajectory_review_labels
    )
    for track_id in (*unlabeled_track_ids, *event_track_ids):
      point = next((item for item in frame.points if item.track_id == track_id), None)
      if point is not None:
        return point
    return None

  def _nearest_clicked_point(self, map_rect: Any, frame: RadarFrame, mouse: Any) -> RadarPoint | None:
    visible = [
      point for point in frame.points
      if 0.75 < point.d_rel <= self.forward_range_m and abs(point.y_rel) < 12.0
    ]
    if not visible:
      return None
    nearest = min(
      visible,
      key=lambda point: math.hypot(
        self._world_to_screen(map_rect, point.d_rel, point.y_rel).x - mouse.x,
        self._world_to_screen(map_rect, point.d_rel, point.y_rel).y - mouse.y,
      ),
    )
    position = self._world_to_screen(map_rect, nearest.d_rel, nearest.y_rel)
    return nearest if math.hypot(position.x - mouse.x, position.y - mouse.y) <= 18.0 else None

  def _handle_input(self, timeline: Any, map_rect: Any, frame: RadarFrame) -> None:
    rl = self.rl
    if rl.is_key_pressed(rl.KEY_SPACE):
      self.paused = not self.paused
      if self.review is not None:
        state = "MANUAL PAUSE" if self.paused else "PLAYING"
        self.review_status = (
          f"{state} @{self.times[self.index]:.2f}s; "
          + "leadTwo pause events remain armed"
        )
    if rl.is_key_pressed(rl.KEY_ONE):
      self.selected_role = "leadOne"
      self.label_status = "active role: leadOne"
    if rl.is_key_pressed(rl.KEY_TWO):
      self.selected_role = "leadTwo"
      self.label_status = "active role: leadTwo"
    if rl.is_key_pressed(rl.KEY_C):
      self.selected_role = "cutin"
      self.label_status = "active role: cutin"
    shift = rl.is_key_down(rl.KEY_LEFT_SHIFT) or rl.is_key_down(rl.KEY_RIGHT_SHIFT)
    if rl.is_key_pressed(rl.KEY_LEFT):
      self.paused = True
      self._user_seek(self.playback_time - (5.0 if shift else 0.05))
    if rl.is_key_pressed(rl.KEY_RIGHT):
      self.paused = True
      self._user_seek(self.playback_time + (5.0 if shift else 0.05))
    if rl.is_key_pressed(rl.KEY_HOME):
      self.paused = True
      self._user_seek(0.0)
    if rl.is_key_pressed(rl.KEY_END):
      self.paused = True
      self._user_seek(self.times[-1])
    if rl.is_key_pressed(rl.KEY_UP):
      self.speed = min(8.0, self.speed * 2.0)
    if rl.is_key_pressed(rl.KEY_DOWN):
      self.speed = max(0.125, self.speed * 0.5)
    if rl.is_key_pressed(rl.KEY_L):
      self.show_labels = not self.show_labels
    if rl.is_key_pressed(rl.KEY_R) and self.review is not None:
      self.review_handled = set(getattr(self, "review_suppressed", set()))
      self.review_status = "LEAD2 REVIEW RESTARTED"
      self.paused = False
      self.seek(0.0)
      self._pause_for_review(self.index - 1, self.index)
    if rl.is_key_pressed(rl.KEY_N):
      self.paused = True
      self._set_label(None)
    if rl.is_key_pressed(rl.KEY_DELETE):
      self.labels.clear(self.index, self.selected_role)
      self.label_status = f"frame {self.index + 1} {self.selected_role} inherits recorded"
      self._save_labels()
    if rl.is_key_pressed(rl.KEY_LEFT_BRACKET):
      is_set, track_id = self.labels.get(self.index, self.selected_role)
      if is_set:
        self.range_anchor = (self.index, self.selected_role, track_id)
        value = "NONE" if track_id is None else f"id {track_id}"
        self.label_status = f"range start {self.index + 1}: {self.selected_role} {value}"
      else:
        self.label_status = "set a manual label before starting a range"
    if rl.is_key_pressed(rl.KEY_RIGHT_BRACKET):
      if self.range_anchor is None:
        self.label_status = "no range start; press [ first"
      else:
        first, role, track_id = self.range_anchor
        self.labels.fill(first, self.index, role, track_id)
        self.label_status = f"filled {role} frames {min(first, self.index) + 1}-{max(first, self.index) + 1}"
        self.range_anchor = None
        self._save_labels()

    wheel = rl.get_mouse_wheel_move()
    if wheel:
      self.forward_range_m = min(180.0, max(30.0, self.forward_range_m - wheel * 10.0))

    mouse = rl.get_mouse_position()
    if (
      self.probability_slider is not None
      and rl.is_mouse_button_down(rl.MOUSE_BUTTON_LEFT)
      and rl.check_collision_point_rec(mouse, self.probability_slider)
    ):
      ratio = (mouse.x - self.probability_slider.x) / self.probability_slider.width
      probability = round(min(max(ratio, 0.0), 1.0) * 20.0) / 20.0
      if probability != self.min_candidate_probability:
        self.min_candidate_probability = probability
        save_review_probability(probability, self.review_settings_path)
      return
    if (
      self.trajectory_horizon_slider is not None
      and rl.is_mouse_button_down(rl.MOUSE_BUTTON_LEFT)
      and rl.check_collision_point_rec(mouse, self.trajectory_horizon_slider)
    ):
      ratio = (
        (mouse.x - self.trajectory_horizon_slider.x)
        / self.trajectory_horizon_slider.width
      )
      horizon_s = round((0.25 + min(max(ratio, 0.0), 1.0) * 1.75) * 4.0) / 4.0
      if horizon_s != self.trajectory_horizon_s:
        self.trajectory_horizon_s = horizon_s
      return
    if rl.is_mouse_button_pressed(rl.MOUSE_BUTTON_LEFT):
      for expected, button in self.review_label_buttons.items():
        if rl.check_collision_point_rec(mouse, button):
          self._set_review_expected(expected)
          return
      for key, checkbox in self.filter_checkboxes.items():
        if not rl.check_collision_point_rec(mouse, checkbox):
          continue
        if key == "recorded_one":
          self.show_recorded_one = not self.show_recorded_one
        elif key == "recorded_two":
          self.show_recorded_two = not self.show_recorded_two
        elif key == "front":
          self.show_front_candidates = not self.show_front_candidates
        elif key == "corner":
          self.show_corner_candidates = not self.show_corner_candidates
        elif key == "external":
          self.show_external_candidates = not self.show_external_candidates
        elif key == "points":
          self.show_radar_points = not self.show_radar_points
        elif key == "trajectory":
          self.show_trajectories = not self.show_trajectories
        elif key == "fused":
          self.show_fused_objects = not self.show_fused_objects
        return
      if rl.check_collision_point_rec(mouse, timeline):
        self.paused = True
        previous_index = self.index
        self._user_seek((mouse.x - timeline.x) / timeline.width * self.times[-1])
        self._pause_for_review(previous_index, self.index)
      elif rl.check_collision_point_rec(mouse, map_rect):
        clicked = self._nearest_clicked_point(map_rect, frame, mouse)
        if clicked is not None:
          self.paused = True
          self._set_label(clicked.track_id)
    if rl.is_mouse_button_pressed(rl.MOUSE_BUTTON_RIGHT) and rl.check_collision_point_rec(mouse, map_rect):
      self.paused = True
      self._set_label(None)

  def run(self, start_s: float, paused: bool, screenshot: Path | None) -> None:
    rl = self.rl
    rl.set_config_flags(rl.FLAG_WINDOW_RESIZABLE | rl.FLAG_VSYNC_HINT)
    rl.init_window(1440, 960, "Radar Lead Workbench")
    rl.set_target_fps(60)
    font_path = Path(__file__).resolve().parents[3] / "assets" / "fonts" / "Inter-Regular.ttf"
    if font_path.is_file():
      self.font = rl.load_font_ex(str(font_path), 32, None, 0)
      rl.set_texture_filter(self.font.texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
    self.paused = paused
    self.seek(start_s)
    self._pause_for_review(self.index - 1, self.index)
    captured = False
    rendered_frames = 0
    screenshot_path = screenshot.resolve() if screenshot is not None else None
    local_screenshot: Path | None = None
    if screenshot_path is not None:
      screenshot_path.parent.mkdir(parents=True, exist_ok=True)
    try:
      while not rl.window_should_close():
        if not self.paused:
          previous_index = self.index
          self.seek(self.playback_time + rl.get_frame_time() * self.speed)
          self._pause_for_review(previous_index, self.index)
          if self.playback_time >= self.times[-1]:
            self.paused = True

        width = rl.get_screen_width()
        height = rl.get_screen_height()
        panel_width = min(510.0, max(430.0, width * 0.35))
        content_width = float(width) - panel_width - 30.0
        timeline_height = 315.0 if self.review is not None else 65.0
        content_height = float(height) - timeline_height
        video_height = max(230.0, content_height * (0.49 if self.review is not None else 0.57))
        video_rect = rl.Rectangle(12.0, 12.0, content_width, video_height)
        map_rect = rl.Rectangle(12.0, video_rect.y + video_rect.height + 8.0, content_width, content_height - video_height - 8.0)
        panel_height = content_height if self.review is not None else float(height) - 65.0
        panel_rect = rl.Rectangle(float(width) - panel_width - 10.0, 12.0, panel_width, panel_height)
        frame = self.frames[self.index]
        selection = self.selector.select(frame, self.index)

        rl.begin_drawing()
        rl.clear_background(self._color(self.BG))
        self._draw_video(video_rect)
        self._draw_map(map_rect, frame, selection)
        self._draw_panel(panel_rect, frame, selection)
        timeline = self._draw_timeline(width, height)
        rl.end_drawing()
        rendered_frames += 1

        if screenshot_path is not None and not captured and rendered_frames >= 3:
          # raylib intentionally strips directories from screenshot paths.
          local_screenshot = Path.cwd() / screenshot_path.name
          rl.take_screenshot(screenshot_path.name)
          captured = True
          break
        self._handle_input(timeline, map_rect, frame)
    finally:
      if self.video_reader is not None:
        self.video_reader.close()
      if self.video_texture is not None:
        rl.unload_texture(self.video_texture)
      if self.font is not None:
        rl.unload_font(self.font)
      rl.close_window()
    if local_screenshot is not None and local_screenshot.resolve() != screenshot_path:
      shutil.move(str(local_screenshot), str(screenshot_path))


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Compare recorded and candidate radar lead selection on an rlog")
  parser.add_argument("rlog", nargs="?", type=Path, help="rlog, rlog.zst, or rlog.bz2 file")
  parser.add_argument("--start", type=float, help="initial time in seconds")
  parser.add_argument("--paused", action="store_true", help="start paused")
  parser.add_argument(
    "--prob", type=float,
    help="manual probability display threshold (0.00-1.00); pauses still follow production leadTwo",
  )
  parser.add_argument(
    "--manual-prob", action="store_true",
    help="probability display using the last saved slider value; pauses still follow production leadTwo",
  )
  parser.add_argument("--summary", action="store_true", help="print comparison summary without opening a window")
  parser.add_argument("--export-csv", type=Path, help="export one comparison row per radar frame")
  parser.add_argument("--labels", type=Path, help="manual label JSON (default: beside the rlog)")
  parser.add_argument("--export-dataset", type=Path, help="export candidate-ranking training CSV or CSV.GZ")
  parser.add_argument("--manual-only", action="store_true", help="dataset: use only manually labeled frames")
  parser.add_argument("--model", type=Path, help="trained MLP .npz to compare instead of simple-v0")
  parser.add_argument("--front-model", type=Path, help="hybrid: front/SCC source model")
  parser.add_argument("--corner-model", type=Path, help="hybrid: corner-radar source model")
  parser.add_argument(
    "--fusion-scc", action="store_true",
    help="allow SCC-only fallback objects in fused view (disabled by default)",
  )
  parser.add_argument(
    "--teacher-current-radard", action="store_true",
    help="use the pure-Python current-radard source port as selector and dataset teacher",
  )
  parser.add_argument(
    "--hybrid", action="store_true",
    help="use independent vision-radar leadOne with model cut-in/external leadTwo",
  )
  parser.add_argument(
    "--compare-radard", action="store_true",
    help="recompute and display the current radard result and distance graph (slow)",
  )
  parser.add_argument(
    "--front-only", action="store_true",
    help="remove corner-radar points and replay the production front-only path",
  )
  parser.add_argument(
    "--validation-case", action="append",
    help="auto-review a case id; repeat for cases that share one log",
  )
  parser.add_argument("--validation-root", type=Path, default=Path(r"W:\routes"), help="validation route root")
  parser.add_argument("--validation-cases", type=Path, default=DEFAULT_VALIDATION_CASES)
  parser.add_argument("--screenshot", type=Path, help="render one frame to PNG and exit")
  return parser.parse_args()


def _validation_review_from_case(case: dict[str, Any]) -> ValidationReview:
  window = case["window"]
  return ValidationReview(
    case_id=str(case["id"]), expected=str(case["expected"]), source=str(case["source"]),
    start_s=float(window[0]), end_s=float(window[1]), scene=str(case["scene"]),
    validation_stage=str(case.get("validation_stage", "output")),
    target_track_ids=tuple(int(value) for value in case.get("target_track_ids", ())),
    forbidden_lead_two_ids=tuple(int(value) for value in case.get("forbidden_lead_two_ids", ())),
    human_verified=bool(case.get("human_verified", False)),
  )


def resolve_validation_cases(
  cases_path: Path, route_root: Path, queries: Sequence[str],
) -> tuple[Path, tuple[ValidationReview, ...]]:
  payload = json.loads(cases_path.read_text(encoding="utf-8"))
  cases = list(payload.get("cases", ()))
  matched_cases: list[dict[str, Any]] = []
  for query in queries:
    exact = [case for case in cases if case["id"].lower() == query.lower()]
    matches = exact or [case for case in cases if query.lower() in case["id"].lower()]
    if len(matches) != 1:
      names = ", ".join(case["id"] for case in matches[:8]) or "none"
      raise SystemExit(f"validation case must match exactly one case; matched: {names}")
    if matches[0] not in matched_cases:
      matched_cases.append(matches[0])
  if not matched_cases:
    raise SystemExit("at least one validation case is required")

  route_keys = {
    (str(case["vehicle_folder"]), str(case["log"]))
    for case in matched_cases
  }
  if len(route_keys) != 1:
    raise SystemExit("repeated --validation-case entries must reference the same rlog")
  vehicle_folder, log = route_keys.pop()
  reviews = tuple(
    sorted(
      (_validation_review_from_case(case) for case in matched_cases),
      key=lambda value: (value.start_s, value.end_s, value.case_id),
    )
  )
  return route_root / vehicle_folder / Path(log), reviews


def resolve_validation_case(
  cases_path: Path, route_root: Path, query: str,
) -> tuple[Path, ValidationReview]:
  route, reviews = resolve_validation_cases(cases_path, route_root, (query,))
  return route, reviews[0]


def update_validation_case_label(cases_path: Path, case_id: str, expected: str) -> None:
  expected = expected.lower()
  if expected not in VALIDATION_EXPECTED_LABELS:
    raise ValueError(f"invalid validation label: {expected}")

  payload = json.loads(cases_path.read_text(encoding="utf-8"))
  matches = [case for case in payload.get("cases", ()) if str(case.get("id", "")).lower() == case_id.lower()]
  if len(matches) != 1:
    raise ValueError(f"validation case must match exactly once: {case_id}")

  text = cases_path.read_text(encoding="utf-8")
  lines = text.splitlines(keepends=True)
  id_token = f'"id": {json.dumps(str(matches[0]["id"]), ensure_ascii=False)}'
  id_lines = [index for index, line in enumerate(lines) if id_token in line]
  if len(id_lines) != 1:
    raise ValueError(f"unable to locate validation case text: {case_id}")

  start = id_lines[0]
  next_ids = [
    index for index in range(start + 1, len(lines))
    if lines[index].lstrip().startswith('"id":')
  ]
  end = next_ids[0] if next_ids else len(lines)
  expected_lines = [
    index for index in range(start, end)
    if lines[index].lstrip().startswith('"expected":')
  ]
  if len(expected_lines) != 1:
    raise ValueError(f"unable to locate expected label for: {case_id}")

  expected_index = expected_lines[0]
  old_line = lines[expected_index]
  indent = old_line[:len(old_line) - len(old_line.lstrip())]
  newline = "\r\n" if old_line.endswith("\r\n") else "\n" if old_line.endswith("\n") else ""
  lines[expected_index] = f'{indent}"expected": "{expected}",{newline}'

  verified_lines = [
    index for index in range(start, end)
    if lines[index].lstrip().startswith('"human_verified":')
  ]
  if verified_lines:
    verified_index = verified_lines[0]
    old_verified = lines[verified_index]
    verified_indent = old_verified[:len(old_verified) - len(old_verified.lstrip())]
    verified_newline = "\r\n" if old_verified.endswith("\r\n") else "\n" if old_verified.endswith("\n") else ""
    comma = "," if old_verified.rstrip("\r\n").endswith(",") else ""
    lines[verified_index] = f'{verified_indent}"human_verified": true{comma}{verified_newline}'
  else:
    lines.insert(expected_index + 1, f'{indent}"human_verified": true,{newline}')

  with cases_path.open("w", encoding="utf-8", newline="") as file:
    file.write("".join(lines))


def upsert_trajectory_review_label(
  labels_path: Path,
  log_path: Path,
  frame: RadarFrame,
  point: RadarPoint,
  expected: str,
  horizon_s: float,
) -> str:
  payload: dict[str, Any] = {"version": 1, "labels": []}
  if labels_path.is_file():
    payload = json.loads(labels_path.read_text(encoding="utf-8"))
  labels = list(payload.get("labels", ()))
  vehicle_folder = log_path.parent.parent.name
  relative_log = f"{log_path.parent.name}/{log_path.name}"
  time_key = round(frame.time_s * 20.0) / 20.0
  source = "corner" if point.source.startswith("corner") else "front"
  label_id = (
    f"manual-{log_path.parent.name.split('--')[0]}-"
    + f"{time_key:06.2f}-{source}-{point.track_id}"
  ).replace(".", "p")
  entry = {
    "id": label_id,
    "vehicle_folder": vehicle_folder,
    "log": relative_log,
    "source": source,
    "time_s": time_key,
    "window": [max(0.0, time_key - 0.75), time_key + 0.75],
    "track_id": point.track_id,
    "expected": expected,
    "prediction_horizon_s": horizon_s,
    "human_verified": True,
  }
  key = (vehicle_folder, relative_log, time_key, point.track_id)
  replaced = False
  for index, item in enumerate(labels):
    item_key = (
      str(item.get("vehicle_folder", "")),
      str(item.get("log", "")),
      float(item.get("time_s", -1.0)),
      int(item.get("track_id", -1)),
    )
    if item_key == key:
      labels[index] = entry
      replaced = True
      break
  if not replaced:
    labels.append(entry)
  labels.sort(key=lambda item: (
    str(item.get("vehicle_folder", "")),
    str(item.get("log", "")),
    float(item.get("time_s", 0.0)),
    int(item.get("track_id", -1)),
  ))
  payload["labels"] = labels
  labels_path.parent.mkdir(parents=True, exist_ok=True)
  labels_path.write_text(
    json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
    encoding="utf-8",
  )
  return label_id


def print_summary(log_path: Path, frames: list[RadarFrame], selector: LeadSelector) -> None:
  print(f"log: {log_path}")
  print(f"frames: {len(frames)}  duration: {frames[-1].time_s:.2f}s  selector: {selector.name}")
  if selector.name.startswith(("mlp:", "multitask:", "hybrid:")):
    multitask = selector.name.startswith(("multitask:", "hybrid:"))
    front_outputs = 0
    corner_outputs = 0
    external_outputs = 0
    six_output_frames = 0
    recorded_total = 0
    recorded_covered = 0
    for frame_index, frame in enumerate(frames):
      selection = selector.select(frame, frame_index)
      outputs = (*selection.front_candidates, *selection.corner_candidates, *selection.external_candidates)
      output_ids = {candidate.track_id for candidate in outputs}
      front_outputs += len(selection.front_candidates)
      corner_outputs += len(selection.corner_candidates)
      external_outputs += len(selection.external_candidates)
      six_output_frames += int(len(outputs) == 6)
      for lead in (frame.recorded_one, frame.recorded_two):
        track_id = resolved_recorded_track_id(frame, lead)
        if track_id is not None:
          recorded_total += 1
          recorded_covered += int(track_id in output_ids)
    print(
      f"source outputs/frame: front {front_outputs / len(frames):.2f}  "
      + f"cutin {corner_outputs / len(frames):.2f}  external {external_outputs / len(frames):.2f}  "
      + f"six-output frames {six_output_frames}/{len(frames)}"
    )
    print(
      f"recorded radar-id candidate coverage: {recorded_covered}/{recorded_total} "
      + f"({recorded_covered / max(recorded_total, 1) * 100.0:.1f}%)"
    )
    print(
      "temporal lead/cut-in policy: applied"
      if multitask else "final leadOne/leadTwo policy: not applied (source candidates preserved)"
    )
    return

  summary = comparison_summary(frames, selector)
  one_total = summary["recorded_one_radar"]
  two_total = summary["recorded_two_radar"]
  one_rate = 100.0 * summary["lead_one_matches"] / one_total if one_total else 0.0
  two_rate = 100.0 * summary["lead_two_matches"] / two_total if two_total else 0.0
  true_positive = summary["selected_true_positive"]
  precision = true_positive / max(true_positive + summary["selected_false_positive"], 1)
  recall = true_positive / max(true_positive + summary["selected_false_negative"], 1)
  print(
    f"combined selected-set agreement: {summary['selected_exact']}/{summary['frames']} "
    + f"({summary['selected_exact'] / max(summary['frames'], 1) * 100.0:.1f}%)  "
    + f"precision {precision * 100.0:.1f}%  recall {recall * 100.0:.1f}%"
  )
  print(f"leadOne recorded radar-id agreement: {summary['lead_one_matches']}/{one_total} ({one_rate:.1f}%)")
  print(f"leadTwo recorded radar-id agreement: {summary['lead_two_matches']}/{two_total} ({two_rate:.1f}%)")


def main() -> int:
  args = parse_args()
  if args.prob is not None and args.manual_prob:
    raise SystemExit("--prob and --manual-prob cannot be used together")
  if args.prob is not None and not 0.0 <= args.prob <= 1.0:
    raise SystemExit("--prob must be between 0.00 and 1.00")
  review: ValidationReview | None = None
  reviews: tuple[ValidationReview, ...] = ()
  if args.validation_case:
    if args.rlog is not None:
      raise SystemExit("do not provide rlog together with --validation-case")
    args.rlog, reviews = resolve_validation_cases(
      args.validation_cases, args.validation_root, args.validation_case,
    )
    review = reviews[0]
    if args.model is None and args.front_model is None:
      args.front_model = DEFAULT_FRONT_MODEL if DEFAULT_FRONT_MODEL.is_file() else DEFAULT_MULTITASK_MODEL
    if args.corner_model is None:
      args.corner_model = DEFAULT_CORNER_MODEL if DEFAULT_CORNER_MODEL.is_file() else args.front_model
    args.start = 0.0 if args.start is None else args.start
  if args.rlog is None:
    raise SystemExit("rlog or --validation-case is required")
  args.start = 0.0 if args.start is None else args.start
  if not args.rlog.is_file():
    raise SystemExit(f"log file does not exist: {args.rlog}")

  print(f"Loading {args.rlog} ...", flush=True)
  frames = load_frames(args.rlog)
  if args.front_only:
    frames, removed_corner_points = front_only_frames(frames)
    print(f"Front-only replay: removed {removed_corner_points} corner-radar points.", flush=True)
  if args.model is not None and not args.model.is_file():
    raise SystemExit(f"model file does not exist: {args.model}")
  for name in ("front_model", "corner_model"):
    path = getattr(args, name)
    if path is not None and not path.is_file():
      raise SystemExit(f"{name.replace('_', '-')} file does not exist: {path}")
  if args.model is not None and args.teacher_current_radard and not args.hybrid:
    raise SystemExit("--model and --teacher-current-radard cannot be used together")
  selector: LeadSelector
  radard_selector: LeadSelector | None = None
  if args.hybrid:
    front_model = args.front_model or args.model or (
      DEFAULT_FRONT_MODEL if DEFAULT_FRONT_MODEL.is_file() else DEFAULT_MULTITASK_MODEL
    )
    corner_model = args.corner_model or args.model or (
      DEFAULT_CORNER_MODEL if DEFAULT_CORNER_MODEL.is_file() else front_model
    )
    selector = ProductionHybridLeadSelector(front_model, frames, corner_model)
  elif args.model is not None:
    try:
      import numpy as np
      with np.load(args.model, allow_pickle=False) as model_data:
        is_multitask = "head_names" in model_data.files
    except Exception as exc:
      raise SystemExit(f"failed to inspect model: {exc}") from exc
    selector = (
      MultitaskLeadSelector(args.model, frames, include_scc=args.fusion_scc)
      if is_multitask else MLPLeadSelector(args.model, frames)
    )
  elif args.teacher_current_radard:
    print("Recomputing current radard cut-in history ...", flush=True)
    selector = CurrentRadardTeacher(frames, current_cutin_track_ids(args.rlog, frames))
  else:
    selector = SimpleLeadSelector()
  if args.compare_radard and review is not None and not isinstance(selector, CurrentRadardTeacher):
    print("Recomputing current radard comparison ...", flush=True)
    radard_selector = CurrentRadardTeacher(frames, current_cutin_track_ids(args.rlog, frames))
  label_path = args.labels or args.rlog.with_name(args.rlog.name + ".lead-labels.json")
  labels = ManualLabels.load(label_path, len(frames))
  print_summary(args.rlog, frames, selector)
  print(f"manual labels: {labels.count()}  file: {label_path}")
  manual_matches, manual_total = manual_comparison_summary(frames, selector, labels)
  if manual_total:
    print(f"manual ground-truth agreement: {manual_matches}/{manual_total} ({manual_matches / manual_total * 100.0:.1f}%)")
  if args.export_csv is not None:
    export_csv(args.export_csv, frames, selector)
    print(f"CSV written: {args.export_csv}")
  if args.export_dataset is not None:
    teacher = selector if args.teacher_current_radard else None
    stats = export_training_dataset(args.export_dataset, frames, labels, args.manual_only, teacher)
    print(
      f"dataset written: {args.export_dataset}  rows {stats['rows']}  groups {stats['groups']} "
      + f"(manual {stats['manual_groups']}, teacher {stats['teacher_groups']}, recorded {stats['recorded_groups']})  "
      + f"positive targets {stats['positives']}  none groups {stats['none_groups']}  skipped {stats['skipped']}"
    )
  if args.summary:
    return 0

  display_name = f"{args.rlog.parent.name}/{args.rlog.name}"
  SimulatorUI(
    frames, selector, display_name, labels, label_path, args.rlog, include_scc_fusion=args.fusion_scc,
    review=review, reviews=reviews,
    validation_cases_path=args.validation_cases if review is not None else None,
    radard_selector=radard_selector,
    initial_probability=args.prob,
    manual_probability_mode=args.manual_prob,
  ).run(args.start, args.paused, args.screenshot)
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
