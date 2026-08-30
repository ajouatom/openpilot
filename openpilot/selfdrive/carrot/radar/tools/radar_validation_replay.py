#!/usr/bin/env python3
"""Replay the physical dPath predictor without mixing in production radard output."""

from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import math
import os
import pickle
import shutil
import sys
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, replace
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Protocol

REPO_ROOT = Path(__file__).resolve().parents[5]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.selfdrive.carrot.radar_motion import (
  CORNER_RADAR_MEASUREMENT_DELAY_S,
  CORNER_CUT_IN_THRESHOLD,
  CUT_IN_BOUNDARY_HOLD_S,
  CUT_IN_CONFIRMATION_S,
  CornerCutInPredecelTracker,
  DPathRadarController,
  DPathLeadCandidate,
  DPathStationaryPrimaryHandoffTracker,
  DPathStationaryShadowTracker,
  DPathLeadTwoTracker,
  FrontRadarKinematicAssociator,
  FRONT_CUT_IN_THRESHOLD,
  IMMEDIATE_LANE_SCOPE_HALF_WIDTH_M,
  POSITION_ONLY_MAX_ABS_VLEAD_MPS,
  RADAR_MOTION_MAX_TIME_SKEW_S,
  STATIONARY_MAX_ABS_VLEAD_MPS,
  RadarMotionDecisionTracker,
  RadarMotionCutIn,
  RadarMotionPrediction,
  RadarMotionPredictor,
  VisionRadarMatcher,
  apply_vision_bracket_cutin_support,
  cutin_can_compete_with_primary,
  corner_cutin_predecel_score,
  front_cutin_motion_supported,
  lead_from_vision,
  lead_duplicates_primary,
  lead_from_radar_point,
  lead_from_vision_match,
  match_dpath_primary_lead,
  prefer_front_radar_kinematics,
  radar_motion_sensitivity,
  stationary_shadow_corner_supported,
  turning_corner_path_entry_allowed,
  model_path_point_at_s,
  project_to_model_path,
  visible_motion_points,
  vision_lead_from_model,
  vision_only_lead_allowed,
)
from openpilot.selfdrive.carrot.radar_motion.occupancy_v2 import (
  OccupancyEstimate,
  OccupancyEvidence,
  OccupancyStage,
  RadarOccupancyModelV2,
  early_control_eligible,
)
from openpilot.selfdrive.carrot.radar_motion.occupancy_v3 import (
  RadarOccupancyModelV3,
  V3Estimate,
  V3Evidence,
  V3Stage,
)


RADAR_TO_CAMERA = 1.52
DISPLAY_MIN_DREL_M = -30.0
DEFAULT_FORWARD_RANGE_M = 130.0
DISPLAY_TOP_PADDING_PX = 72.0
DISPLAY_BOTTOM_PADDING_PX = 18.0
CARROT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_VALIDATION_CASES = CARROT_ROOT / "cluster" / "cutin_validation_cases.json"
DEFAULT_TRAJECTORY_LABELS = CARROT_ROOT / "cluster" / "radar_trajectory_labels.json"
SHADOW_CUTIN_THRESHOLD = 0.50
VALIDATION_DEFAULT_CORNER_PROBABILITY = CORNER_CUT_IN_THRESHOLD
VALIDATION_DEFAULT_FRONT_PROBABILITY = FRONT_CUT_IN_THRESHOLD
CUTIN_CONFIRMATION_S = CUT_IN_CONFIRMATION_S
CUTIN_DECISION_HOLD_S = CUT_IN_BOUNDARY_HOLD_S
VALIDATION_EXPECTED_LABELS = ("detect", "clear", "stationary")
MAX_POINT_MODEL_TIME_SKEW_S = RADAR_MOTION_MAX_TIME_SKEW_S
VALIDATION_CORNER_MAX_MEASUREMENT_AGE_S = 0.10
VALIDATION_GROUP2_RADAR_START_ADDR = 0x3A5
VALIDATION_GROUP2_RADAR_TRACK_COUNT = 32
VALIDATION_GROUP2_QUALITY_MAX_AGE_S = 0.15
KOREAN_FONT_BASE_SIZE = 40
VALIDATION_PROBABILITY_MIN = 0.20
VALIDATION_PROBABILITY_MAX = 0.80
VALIDATION_DEFAULT_LOOKAHEAD_S = 5.00
VALIDATION_MIN_CONTINUOUS_OVERLAP_S = 0.50
STATIONARY_HANDOFF_MAX_DREL_DELTA_M = 3.5
STATIONARY_HANDOFF_MAX_YREL_DELTA_M = 1.5
VALIDATION_SETTINGS_ENV = "CARROT_RADAR_VALIDATION_SETTINGS"
VALIDATION_MOTION_MODES = ("normal", "front")
VALIDATION_DEFAULT_SENSITIVITY = 3
VISUAL_REPLAY_CACHE_VERSION = 3
LEAD_ONE_RADAR_RGB = (246, 142, 55)
LEAD_ONE_VISION_RGB = (72, 145, 255)
LEAD_ONE_VISION_WEAK_RGB = (104, 205, 255)
LEAD_ONE_VISION_INACTIVE_RGB = (112, 145, 160)
LEAD_TWO_RGB = (245, 211, 72)
LEAD_ONE_SPEED_RGB = (62, 205, 130)
SCC_DISTANCE_RGB = (247, 94, 160)
SCC_ACCEL_RGB = (247, 94, 160)
CARROT_ACCEL_RGB = (62, 205, 130)
VISION_LEAD_DISPLAY_MIN_PROBABILITY = 0.20
VISION_LEAD_STRONG_DISPLAY_MIN_PROBABILITY = 0.40
SCC_SAMPLE_MAX_AGE_S = 0.15
LONGITUDINAL_PLAN_MAX_AGE_S = 0.20
ACCEL_GRAPH_MIN_MPS2 = -4.0
ACCEL_GRAPH_MAX_MPS2 = 2.0
LEAD_SPEED_GRAPH_MAX_KPH = 140.0


def radar_replay_source_fingerprint() -> str:
  """Fingerprint every source file that changes cached lead decisions."""
  digest = hashlib.sha256()
  source_files = (
    Path(__file__),
    *sorted((CARROT_ROOT / "radar_motion").glob("*.py")),
  )
  for source_path in source_files:
    digest.update(str(source_path.relative_to(REPO_ROOT)).encode("utf-8"))
    digest.update(source_path.read_bytes())
  return digest.hexdigest()[:20]


def radar_replay_baseline_source_fingerprint() -> str:
  """Fingerprint V1/V2 replay sources without V3 shadow-only code."""
  digest = hashlib.sha256()
  replay_source = Path(__file__).read_text(encoding="utf-8")
  v3_start = replay_source.index("\ndef _v3_candidate(")
  v3_end = replay_source.index("\ndef visual_replay_cache_path(")
  ui_start = replay_source.index("\nclass SimulatorUI:")
  replay_source = replay_source[:v3_start] + replay_source[v3_end:ui_start]
  digest.update(replay_source.encode("utf-8"))
  for source_path in sorted((CARROT_ROOT / "radar_motion").glob("*.py")):
    if source_path.name == "occupancy_v3.py":
      continue
    digest.update(str(source_path.relative_to(REPO_ROOT)).encode("utf-8"))
    digest.update(source_path.read_bytes())
  return digest.hexdigest()[:20]


VALIDATION_SENSITIVITY_LABELS = (
  "사용 안 함",
  "둔감",
  "약간 둔감",
  "보통",
  "민감",
  "아주 민감",
)


def _validation_lookahead_s(value: float) -> float:
  del value
  return VALIDATION_DEFAULT_LOOKAHEAD_S


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
  radar_track_state: int = 0


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


def vision_lead_rgb(probability: float) -> tuple[int, int, int]:
  return (
    LEAD_ONE_VISION_RGB
    if float(probability) >= VISION_LEAD_STRONG_DISPLAY_MIN_PROBABILITY
    else LEAD_ONE_VISION_WEAK_RGB
  )


def vision_lead_display_value(
  vision: ModelLead | None,
) -> tuple[str, tuple[int, int, int], float | None]:
  if vision is None:
    return "--", LEAD_ONE_VISION_INACTIVE_RGB, None
  probability = float(vision.probability)
  distance = float(vision.x - RADAR_TO_CAMERA)
  if (
    probability >= VISION_LEAD_DISPLAY_MIN_PROBABILITY
    and math.isfinite(distance)
    and 0.0 <= distance <= DEFAULT_FORWARD_RANGE_M
  ):
    return (
      f"{distance:.1f}m p{probability:.2f}",
      vision_lead_rgb(probability),
      distance,
    )
  return (
    f"-- p{probability:.2f}",
    LEAD_ONE_VISION_INACTIVE_RGB,
    None,
  )


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
  radar_delay_s: float = 0.0
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
  scc_bus: int | None = None
  scc_distance_m: float | None = None
  scc_a_req_raw: float | None = None
  carrot_a_target: float | None = None


@dataclass(frozen=True)
class Candidate:
  track_id: int
  score: float
  reason: str
  decision_threshold: float = 0.0
  d_rel: float | None = None
  y_rel: float | None = None
  v_lead: float | None = None
  track_aliases: tuple[int, ...] = ()
  base_score: float | None = None
  temporal_score: float | None = None
  horizon_scores: tuple[float, ...] = ()
  out_horizon_scores: tuple[float, ...] = ()
  path_exit_score: float = 0.0
  path_exit_threshold: float = SHADOW_CUTIN_THRESHOLD
  current_path_occupancy: bool = False
  stage: str = ""
  detail: str = ""
  source: str = ""
  horizon_x: tuple[float, ...] = ()
  horizon_y: tuple[float, ...] = ()
  horizon_x_stds: tuple[float, ...] = ()
  horizon_y_stds: tuple[float, ...] = ()
  continuity_id: int | None = None

  @property
  def eligible(self) -> bool:
    return self.score >= self.decision_threshold

  @property
  def path_in_score(self) -> float:
    return self.score

  @property
  def path_out_score(self) -> float:
    return self.path_exit_score


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
  cutin_predecel_candidate: Candidate | None = None


def lead_one_rgb(track_id: int | None) -> tuple[int, int, int]:
  return (
    LEAD_ONE_VISION_RGB
    if track_id is not None and track_id < 0
    else LEAD_ONE_RADAR_RGB
  )


@dataclass(frozen=True)
class ValidationReview:
  case_id: str
  expected: str
  source: str
  start_s: float
  end_s: float
  scene: str
  target_track_ids: tuple[int, ...] = ()
  forbidden_lead_one_ids: tuple[int, ...] = ()
  forbidden_lead_two_ids: tuple[int, ...] = ()
  validation_stage: str = "output"
  human_verified: bool = False


class LeadSelector(Protocol):
  name: str

  def select(self, frame: RadarFrame, frame_index: int | None = None) -> Selection:
    ...


def _finite(value: Any, fallback: float = 0.0) -> float:
  try:
    parsed = float(value)
  except (TypeError, ValueError):
    return fallback
  return parsed if math.isfinite(parsed) else fallback


def validation_settings_path() -> Path:
  override = os.environ.get(VALIDATION_SETTINGS_ENV)
  if override:
    return Path(override)
  local_app_data = os.environ.get("LOCALAPPDATA")
  if local_app_data:
    root = Path(local_app_data)
  else:
    xdg_config = os.environ.get("XDG_CONFIG_HOME")
    root = Path(xdg_config) if xdg_config else Path.home() / ".config"
  return root / "carrotpilot" / "radar_validation.json"


def _read_validation_settings(path: Path) -> dict[str, Any]:
  try:
    payload = json.loads(path.read_text(encoding="utf-8"))
  except (OSError, TypeError, ValueError, json.JSONDecodeError):
    return {}
  return payload if isinstance(payload, dict) else {}


def _write_validation_settings(path: Path, payload: dict[str, Any]) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  temporary = path.with_suffix(path.suffix + ".tmp")
  temporary.write_text(
    json.dumps(payload, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
  )
  temporary.replace(path)


def _default_validation_probability(sensor: str) -> float:
  if sensor == "front":
    return VALIDATION_DEFAULT_FRONT_PROBABILITY
  if sensor == "corner":
    return VALIDATION_DEFAULT_CORNER_PROBABILITY
  raise ValueError(f"unsupported radar motion sensor: {sensor}")


def load_validation_probability(
  path: Path | None = None,
  default: float | None = None,
  *,
  sensor: str = "corner",
) -> float:
  settings_path = path or validation_settings_path()
  fallback = (
    _default_validation_probability(sensor)
    if default is None
    else float(default)
  )
  try:
    value = float(
      _read_validation_settings(settings_path)[f"{sensor}_probability"],
    )
  except (KeyError, TypeError, ValueError):
    return fallback
  return (
    value
    if VALIDATION_PROBABILITY_MIN <= value <= VALIDATION_PROBABILITY_MAX
    else fallback
  )


def save_validation_probability(
  probability: float,
  path: Path | None = None,
  *,
  sensor: str = "corner",
) -> None:
  _default_validation_probability(sensor)
  value = min(max(
    float(probability),
    VALIDATION_PROBABILITY_MIN,
  ), VALIDATION_PROBABILITY_MAX)
  settings_path = path or validation_settings_path()
  payload = _read_validation_settings(settings_path)
  payload.pop("probability", None)
  payload[f"{sensor}_probability"] = round(value, 2)
  _write_validation_settings(settings_path, payload)


def load_validation_lookahead(
  path: Path | None = None,
  default: float = VALIDATION_DEFAULT_LOOKAHEAD_S,
  *,
  sensor: str = "corner",
) -> float:
  del path, default
  _default_validation_probability(sensor)
  return VALIDATION_DEFAULT_LOOKAHEAD_S


def save_validation_lookahead(
  lookahead_s: float,
  path: Path | None = None,
  *,
  sensor: str = "corner",
) -> None:
  _default_validation_probability(sensor)
  _validation_lookahead_s(lookahead_s)
  settings_path = path or validation_settings_path()
  payload = _read_validation_settings(settings_path)
  payload.pop(f"{sensor}_lookahead_s", None)
  _write_validation_settings(settings_path, payload)


def load_validation_sensitivity(
  path: Path | None = None,
  default: int = VALIDATION_DEFAULT_SENSITIVITY,
) -> int:
  fallback = max(0, min(5, int(default)))
  try:
    value = int(
      _read_validation_settings(
        path or validation_settings_path(),
      )["cut_in_sensitivity"],
    )
  except (KeyError, TypeError, ValueError):
    return fallback
  return value if 0 <= value <= 5 else fallback


def save_validation_sensitivity(
  sensitivity: int,
  path: Path | None = None,
) -> None:
  value = max(0, min(5, int(sensitivity)))
  settings_path = path or validation_settings_path()
  payload = _read_validation_settings(settings_path)
  payload["cut_in_sensitivity"] = value
  payload.pop("corner_lookahead_s", None)
  payload.pop("front_lookahead_s", None)
  _write_validation_settings(settings_path, payload)


def load_validation_motion_mode(
  path: Path | None = None,
  default: str = "normal",
) -> str:
  if default not in VALIDATION_MOTION_MODES:
    raise ValueError(f"unsupported radar motion mode: {default}")
  value = str(
    _read_validation_settings(path or validation_settings_path()).get(
      "motion_mode", default,
    ),
  )
  return value if value in VALIDATION_MOTION_MODES else default


def save_validation_motion_mode(
  mode: str,
  path: Path | None = None,
) -> None:
  if mode not in VALIDATION_MOTION_MODES:
    raise ValueError(f"unsupported radar motion mode: {mode}")
  settings_path = path or validation_settings_path()
  payload = _read_validation_settings(settings_path)
  payload["motion_mode"] = mode
  _write_validation_settings(settings_path, payload)


def candidate_track_ids(candidate: Candidate | None) -> frozenset[int]:
  if candidate is None:
    return frozenset()
  return frozenset((candidate.track_id, *candidate.track_aliases))


def candidate_track_id(candidate: Candidate | None) -> int | None:
  return candidate.track_id if candidate is not None else None


def candidate_matches_targets(candidate: Candidate | None, targets: set[int]) -> bool:
  return candidate is not None and (not targets or bool(candidate_track_ids(candidate) & targets))


def lead_continuity_segments(
  frames: Sequence[RadarFrame],
  selections: Sequence[Selection],
  role: str,
) -> tuple[tuple[tuple[float, float, int], ...], ...]:
  if role not in ("lead_one", "lead_two"):
    raise ValueError(f"unsupported lead role: {role}")
  if len(frames) != len(selections):
    raise ValueError("lead selections must align with radar frames")
  segments: list[tuple[tuple[float, float, int], ...]] = []
  current: list[tuple[float, float, int]] = []
  previous_candidate: Candidate | None = None
  for frame, selection in zip(frames, selections, strict=True):
    candidate = getattr(selection, role)
    if (
      candidate is None
      or candidate.d_rel is None
      or not math.isfinite(candidate.d_rel)
      or not 0.0 <= candidate.d_rel <= DEFAULT_FORWARD_RANGE_M
    ):
      if current:
        segments.append(tuple(current))
        current = []
      previous_candidate = None
      continue
    point = (frame.time_s, float(candidate.d_rel), candidate.track_id)
    physical_motion_handoff = (
      previous_candidate is not None
      and previous_candidate.source == candidate.source
      and previous_candidate.continuity_id is not None
      and previous_candidate.continuity_id == candidate.continuity_id
    )
    stationary_handoff = (
      previous_candidate is not None
      and previous_candidate.v_lead is not None
      and candidate.v_lead is not None
      and abs(previous_candidate.v_lead)
      <= STATIONARY_MAX_ABS_VLEAD_MPS
      and abs(candidate.v_lead) <= STATIONARY_MAX_ABS_VLEAD_MPS
      and previous_candidate.d_rel is not None
      and abs(candidate.d_rel - previous_candidate.d_rel)
      <= STATIONARY_HANDOFF_MAX_DREL_DELTA_M
      and previous_candidate.y_rel is not None
      and candidate.y_rel is not None
      and abs(candidate.y_rel - previous_candidate.y_rel)
      <= STATIONARY_HANDOFF_MAX_YREL_DELTA_M
    )
    vision_radar_display_changed = (
      previous_candidate is not None
      and (previous_candidate.track_id < 0) != (candidate.track_id < 0)
    )
    if current and (
      current[-1][2] != point[2]
      and (
        vision_radar_display_changed
        or not (physical_motion_handoff or stationary_handoff)
      )
      or point[0] - current[-1][0] > 0.15
    ):
      segments.append(tuple(current))
      current = []
    current.append(point)
    previous_candidate = candidate
  if current:
    segments.append(tuple(current))
  return tuple(segments)


def lead_speed_continuity_segments(
  frames: Sequence[RadarFrame],
  selections: Sequence[Selection],
) -> tuple[tuple[tuple[float, float, int], ...], ...]:
  """Return leadOne absolute-speed runs in km/h for the replay graph."""
  if len(frames) != len(selections):
    raise ValueError("lead selections must align with radar frames")
  segments: list[tuple[tuple[float, float, int], ...]] = []
  current: list[tuple[float, float, int]] = []
  for frame, selection in zip(frames, selections, strict=True):
    candidate = selection.lead_one
    if (
      candidate is None
      or candidate.v_lead is None
      or not math.isfinite(candidate.v_lead)
    ):
      if current:
        segments.append(tuple(current))
        current = []
      continue
    point = (
      frame.time_s,
      max(0.0, float(candidate.v_lead) * 3.6),
      candidate.track_id,
    )
    if current and (
      current[-1][2] != point[2]
      or point[0] - current[-1][0] > 0.15
    ):
      segments.append(tuple(current))
      current = []
    current.append(point)
  if current:
    segments.append(tuple(current))
  return tuple(segments)


def frame_value_continuity_segments(
  frames: Sequence[RadarFrame],
  attribute: str,
) -> tuple[tuple[tuple[float, float], ...], ...]:
  """Return finite scalar frame values split across missing/stale samples."""
  segments: list[tuple[tuple[float, float], ...]] = []
  current: list[tuple[float, float]] = []
  for frame in frames:
    value = getattr(frame, attribute)
    if value is None or not math.isfinite(value):
      if current:
        segments.append(tuple(current))
        current = []
      continue
    point = (frame.time_s, float(value))
    if current and point[0] - current[-1][0] > 0.15:
      segments.append(tuple(current))
      current = []
    current.append(point)
  if current:
    segments.append(tuple(current))
  return tuple(segments)


def vision_lead_continuity_segments(
  frames: Sequence[RadarFrame],
) -> tuple[tuple[tuple[float, float, float], ...], ...]:
  """Return visible model leadV3[0] distance runs for the replay graph."""
  segments: list[tuple[tuple[float, float, float], ...]] = []
  current: list[tuple[float, float, float]] = []
  for frame in frames:
    lead = frame.model_leads[0] if frame.model_leads else None
    distance = lead.x - RADAR_TO_CAMERA if lead is not None else math.nan
    if (
      lead is None
      or lead.probability < VISION_LEAD_DISPLAY_MIN_PROBABILITY
      or not math.isfinite(distance)
      or not 0.0 <= distance <= DEFAULT_FORWARD_RANGE_M
    ):
      if current:
        segments.append(tuple(current))
        current = []
      continue
    point = (frame.time_s, float(distance), float(lead.probability))
    if current and (
      point[0] - current[-1][0] > 0.15
      or vision_lead_rgb(point[2]) != vision_lead_rgb(current[-1][2])
    ):
      segments.append(tuple(current))
      current = []
    current.append(point)
  if current:
    segments.append(tuple(current))
  return tuple(segments)


def model_line_y(points: Sequence[tuple[float, float]], distance: float) -> float:
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
  ratio = 0.0 if abs(x1 - x0) < 1e-6 else (distance - x0) / (x1 - x0)
  return -(y0 + (y1 - y0) * ratio)


def trajectory_history_display_position(
  frame: RadarFrame,
  sample: Any,
) -> tuple[float, float]:
  return model_path_point_at_s(
    frame.path,
    float(sample.path_x),
    float(sample.d_path),
  )


def trajectory_history_display_y(frame: RadarFrame, sample: Any) -> float:
  return trajectory_history_display_position(frame, sample)[1]


def confirmed_cutin_overlap_at(
  prediction: RadarMotionPrediction,
  horizon_s: float,
) -> bool:
  """Return whether a confirmed CUT-IN is inside its predicted overlap run."""
  if prediction.current_path_occupancy:
    return True
  start_s = prediction.predicted_path_overlap_start_s
  if start_s is None:
    return False
  return (
    start_s - 1e-6
    <= float(horizon_s)
    <= start_s + prediction.predicted_path_overlap_s + 1e-6
  )


def motion_points_at_model_time(
  frame: RadarFrame,
  motion_sensor: str,
) -> tuple[RadarPoint, ...]:
  """Select one sensor from points aligned to the model-path timestamp."""
  return tuple(
    point for point in radar_points_at_model_time(frame)
    if (
      point.source.startswith("corner")
      if motion_sensor == "corner"
      else point.source == "frontRadar"
    )
  )


def radar_points_at_model_time(
  frame: RadarFrame,
) -> tuple[RadarPoint, ...]:
  """Mirror device alignment with each radar source's own delay."""
  if (
    not math.isfinite(frame.input_age_s)
    or not math.isfinite(frame.model_age_s)
  ):
    return ()
  aligned = []
  for point in frame.points:
    if not point.measured:
      continue
    measurement_delay_s = (
      CORNER_RADAR_MEASUREMENT_DELAY_S
      if point.source.startswith("corner")
      else frame.radar_delay_s
    )
    time_delta_s = (
      frame.input_age_s
      - frame.model_age_s
      + measurement_delay_s
    )
    if abs(time_delta_s) > MAX_POINT_MODEL_TIME_SKEW_S:
      continue
    aligned.append(replace(
      point,
      d_rel=point.d_rel + point.v_rel * time_delta_s,
      y_rel=point.y_rel + point.yv_rel * time_delta_s,
    ))
  return tuple(aligned)


def radar_trajectory_series(
  frames: Iterable[RadarFrame],
  motion_sensor: str | None = None,
  lead_one_outputs: Sequence[dict[str, Any] | None] | None = None,
  *,
  directional_min_consistency: float | None = None,
) -> tuple[dict[tuple[str, int], RadarMotionPrediction], ...]:
  frame_values = tuple(frames)
  selected_sensor = motion_sensor or preferred_radar_motion_sensor(frame_values)
  lead_values = (
    tuple(lead_one_outputs)
    if lead_one_outputs is not None
    else (None,) * len(frame_values)
  )
  if len(lead_values) != len(frame_values):
    raise ValueError("leadOne outputs must align with radar frames")
  predictor = (
    RadarMotionPredictor()
    if directional_min_consistency is None
    else RadarMotionPredictor(
      directional_min_consistency=directional_min_consistency,
    )
  )
  return tuple(
    predictor.update(
      frame.time_s,
      motion_points_at_model_time(frame, selected_sensor),
      frame.path,
      frame.v_ego,
      frame.yaw_rate_rad_s,
      lead_one_d_rel=(
        _finite(lead["dRel"])
        if lead is not None and bool(lead.get("status", True))
        else None
      ),
    )
    for frame, lead in zip(frame_values, lead_values, strict=True)
  )


def preferred_radar_motion_sensor(frames: Iterable[RadarFrame]) -> str:
  return (
    "corner"
    if any(
      point.measured and point.source.startswith("corner")
      for frame in frames
      for point in frame.points
    )
    else "front"
  )


def is_position_only_reference(
  frame: RadarFrame,
  point: RadarPoint,
  motion_sensor: str,
) -> bool:
  source_matches = (
    point.source.startswith("corner")
    if motion_sensor == "corner"
    else point.source == "frontRadar"
  )
  return (
    source_matches
    and point.measured
    and abs(point.v_lead) <= POSITION_ONLY_MAX_ABS_VLEAD_MPS
    and abs(
      project_to_model_path(frame.path, point.d_rel, point.y_rel).d_path
    ) <= IMMEDIATE_LANE_SCOPE_HALF_WIDTH_M
  )


def _shadow_candidate(prediction: RadarMotionPrediction) -> Candidate:
  samples = prediction.samples
  recent_entry = (
    prediction.path_entry_age_s is not None
    and prediction.path_entry_age_s
    <= CUTIN_CONFIRMATION_S + CUTIN_DECISION_HOLD_S
  )
  if (
    prediction.current_path_occupancy
    and prediction.front_tracked_close_entry
  ):
    stage = "TRACKED-IN"
  elif (
    prediction.current_path_occupancy
    and prediction.cut_in_detection_allowed
  ):
    stage = "IN"
  elif prediction.current_path_occupancy:
    stage = "NEAR-IN"
  elif prediction.reason == "position/velocity motion mismatch":
    stage = "MISMATCH"
  elif prediction.cut_in_probability >= SHADOW_CUTIN_THRESHOLD:
    stage = "PENDING"
  elif prediction.cut_out_probability >= SHADOW_CUTIN_THRESHOLD:
    stage = "SHADOW-CUTOUT"
  else:
    stage = "BELOW"
  return Candidate(
    track_id=prediction.track_id,
    score=max(
      prediction.cut_in_probability,
      prediction.path_entry_probability if recent_entry else 0.0,
    ),
    reason=f"physical {prediction.sensor} dPath shadow",
    decision_threshold=SHADOW_CUTIN_THRESHOLD,
    d_rel=samples[0].d_rel if samples else None,
    y_rel=samples[0].y_rel if samples else None,
    horizon_scores=tuple(sample.occupancy_prob for sample in samples),
    out_horizon_scores=tuple(1.0 - sample.occupancy_prob for sample in samples),
    path_exit_score=prediction.cut_out_probability,
    current_path_occupancy=prediction.current_path_occupancy,
    stage=stage,
    detail=(
      f"dP={prediction.d_path:+.2f} "
      + f"dP속={prediction.d_path_rate_short:+.2f}/{prediction.d_path_rate_long:+.2f} "
      + f"방향={prediction.directional_inward_displacement_m:.2f}m/"
      + f"{prediction.directional_consistency:.2f} "
      + f"각(이력/레이더)={prediction.vector_heading_deg:+.1f}/"
      + f"{prediction.reported_heading_deg:+.1f} "
      + f"일치={prediction.motion_consistency:.2f} "
      + f"최근={prediction.recent_motion_support:.2f} "
      + f"침범={prediction.predicted_path_overlap_s:.1f}s"
      + (
        f"@+{prediction.predicted_path_overlap_start_s:.1f}s"
        if prediction.predicted_path_overlap_start_s is not None
        else ""
      )
    ),
    source=prediction.source,
    horizon_x=tuple(sample.path_x for sample in samples),
    horizon_y=tuple(sample.d_path for sample in samples),
    horizon_x_stds=tuple(sample.longitudinal_sigma for sample in samples),
    horizon_y_stds=tuple(sample.lateral_sigma for sample in samples),
    continuity_id=prediction.continuity_id,
  )


def _controller_model(frame: RadarFrame) -> Any:
  return SimpleNamespace(
    position=SimpleNamespace(
      x=tuple(point[0] for point in frame.path),
      y=tuple(point[1] for point in frame.path),
    ),
    leadsV3=tuple(
      SimpleNamespace(
        prob=lead.probability,
        x=(lead.x,),
        y=(lead.y,),
        v=(lead.v,),
        a=(lead.a,),
        xStd=(lead.x_std,),
        yStd=(lead.y_std,),
        vStd=(lead.v_std,),
      )
      for lead in frame.model_leads
    ),
    velocity=SimpleNamespace(x=(frame.v_ego,)),
  )


def _controller_candidate(
  frame: RadarFrame,
  lead: dict[str, Any] | None,
  reason: str,
  continuity_by_identity: Mapping[tuple[str, int], int] | None = None,
  track_aliases: Iterable[int] = (),
) -> Candidate | None:
  if lead is None or not lead.get("status"):
    return None
  track_id = int(lead.get("radarTrackId", -1))
  d_rel = float(lead.get("dRel", 0.0))
  y_rel = float(lead.get("yRel", 0.0))
  matched_point = min(
    (
      point for point in frame.points
      if point.measured and point.track_id == track_id
    ),
    key=lambda point: (
      abs(point.d_rel - d_rel),
      abs(point.y_rel - y_rel),
    ),
    default=None,
  )
  source = matched_point.source if matched_point is not None else ""
  aliases: tuple[int, ...] = tuple(
    sorted({int(value) for value in track_aliases if int(value) != track_id})
  )
  if matched_point is not None:
    counterpart_is_front = matched_point.source.startswith("corner")
    counterparts = tuple(
      (
        abs(point.d_rel - matched_point.d_rel) / 5.0
        + abs(point.y_rel - matched_point.y_rel) / 2.0
        + abs(point.v_lead - matched_point.v_lead) / 2.5,
        point,
      )
      for point in frame.points
      if (
        point.measured
        and (
          point.source == "frontRadar"
          if counterpart_is_front
          else point.source.startswith("corner")
        )
        and abs(point.d_rel - matched_point.d_rel) <= 5.0
        and abs(point.y_rel - matched_point.y_rel) <= 2.0
        and abs(point.v_lead - matched_point.v_lead) <= 2.5
      )
    )
    if counterparts:
      aliases = tuple(sorted({
        *aliases,
        min(counterparts, key=lambda value: value[0])[1].track_id,
      }))
  return Candidate(
    track_id=track_id,
    score=float(lead.get("score", lead.get("modelProb", 0.0))),
    reason=reason,
    d_rel=d_rel,
    y_rel=y_rel,
    v_lead=float(lead.get("vLead", 0.0)),
    source=source,
    track_aliases=aliases,
    continuity_id=(
      continuity_by_identity.get((source, track_id))
      if continuity_by_identity is not None
      else None
    ),
  )


def front_radar_display_points(frame: RadarFrame) -> tuple[RadarPoint, ...]:
  return tuple(
    point for point in motion_points_at_model_time(frame, "front")
    if (
      point.measured
      and point.source == "frontRadar"
      and DISPLAY_MIN_DREL_M <= point.d_rel <= DEFAULT_FORWARD_RANGE_M
    )
  )


def corner_radar_display_points(frame: RadarFrame) -> tuple[RadarPoint, ...]:
  return tuple(
    point for point in radar_points_at_model_time(frame)
    if (
      point.measured
      and point.source.startswith("corner")
      and DISPLAY_MIN_DREL_M
      <= point.d_rel
      <= DEFAULT_FORWARD_RANGE_M
    )
  )


def _recorded_candidate(lead: RecordedLead, reason: str) -> Candidate | None:
  if not lead.status:
    return None
  return Candidate(
    track_id=lead.track_id if lead.radar else -1,
    score=lead.model_prob if lead.model_prob > 0.0 else 1.0,
    reason=reason,
    d_rel=lead.d_rel,
    y_rel=lead.y_rel,
    v_lead=lead.v_lead,
  )


class CurrentRadardSelector:
  """Existing radard replay: recorded lead roles plus current cut-in recomputation."""

  name = "existing-radard"

  def __init__(self, frames: Sequence[RadarFrame], cutin_ids: Sequence[set[int]]) -> None:
    if len(frames) != len(cutin_ids):
      raise ValueError("cut-in ID frames must align with radar frames")
    selections: list[Selection] = []
    for frame, active_ids in zip(frames, cutin_ids, strict=True):
      active = tuple(
        Candidate(
          point.track_id,
          1.0,
          "current radard active cutin",
          d_rel=point.d_rel,
          y_rel=point.y_rel,
          source=point.source,
        )
        for point in sorted(frame.points, key=lambda value: value.d_rel)
        if point.track_id in active_ids
      )
      lead_one = _recorded_candidate(frame.recorded_one, "recorded radard leadOne")
      lead_two = active[0] if active else _recorded_candidate(
        frame.recorded_two, "recorded radard leadTwo",
      )
      selections.append(Selection(
        lead_one=lead_one,
        lead_two=lead_two,
        active_cutin_candidates=active,
      ))
    self.selections = tuple(selections)

  def select(self, frame: RadarFrame, frame_index: int | None = None) -> Selection:
    if frame_index is None:
      raise ValueError("existing radard selector requires a frame index")
    return self.selections[frame_index]


# Compatibility name retained for validation scripts that used the old teacher.
CurrentRadardTeacher = CurrentRadardSelector


def prediction_with_validation_lookahead(
  prediction: RadarMotionPrediction,
  maximum_lookahead_s: float,
) -> RadarMotionPrediction:
  """Limit a cached 5-second prediction to the selected review horizon."""
  lookahead_s = _validation_lookahead_s(maximum_lookahead_s)
  overlap_start_s = prediction.predicted_path_overlap_start_s
  overlap_end_s = (
    overlap_start_s + prediction.predicted_path_overlap_s
    if overlap_start_s is not None
    else None
  )
  overlap_within_horizon_s = (
    max(0.0, min(overlap_end_s, lookahead_s) - overlap_start_s)
    if overlap_start_s is not None and overlap_end_s is not None
    else 0.0
  )
  entry_within_horizon_s = (
    prediction.time_to_entry_s
    if (
      prediction.time_to_entry_s is not None
      and prediction.time_to_entry_s <= lookahead_s
    )
    else None
  )
  has_sustained_future_overlap = (
    overlap_within_horizon_s
    >= VALIDATION_MIN_CONTINUOUS_OVERLAP_S - 1e-6
  )
  if (
    prediction.current_path_occupancy
    or prediction.near_side_directional_entry
    or prediction.lane_boundary_directional_entry
    or has_sustained_future_overlap
  ):
    return replace(
      prediction,
      predicted_path_overlap_s=overlap_within_horizon_s,
      time_to_entry_s=entry_within_horizon_s,
    )
  return replace(
    prediction,
    cut_in_probability=0.0,
    path_entry_probability=0.0,
    predicted_path_overlap_s=overlap_within_horizon_s,
    time_to_entry_s=None,
    reason="no sustained overlap inside validation lookahead",
  )


class RadarMotionShadowSelector:
  """Expose physical predictions without importing existing radard lead roles."""

  name = "physical-dpath-predictor"

  def __init__(
    self,
    frames: Sequence[RadarFrame],
    decision_threshold: float | None = None,
    *,
    cut_in_sensitivity: int = VALIDATION_DEFAULT_SENSITIVITY,
    motion_sensor: str | None = None,
    motion_points: Sequence[tuple[RadarPoint, ...]] | None = None,
    trajectories: Sequence[
      dict[tuple[str, int], RadarMotionPrediction]
    ] | None = None,
    lead_one_outputs: Sequence[dict[str, Any] | None] | None = None,
    maximum_lookahead_s: float = VALIDATION_DEFAULT_LOOKAHEAD_S,
    enable_radar_tracks: int = 1,
  ) -> None:
    if motion_sensor not in (None, "corner", "front"):
      raise ValueError(f"unsupported radar motion sensor: {motion_sensor}")
    self.motion_sensor = motion_sensor or preferred_radar_motion_sensor(frames)
    self.cut_in_sensitivity = max(0, min(5, int(cut_in_sensitivity)))
    self.motion_sensitivity = radar_motion_sensitivity(
      self.cut_in_sensitivity,
      self.motion_sensor,
    )
    self.decision_threshold = (
      self.motion_sensitivity.cut_in_threshold
      if decision_threshold is None
      else float(decision_threshold)
    )
    self.maximum_lookahead_s = _validation_lookahead_s(
      maximum_lookahead_s,
    )
    self.enable_radar_tracks = int(enable_radar_tracks)
    production_outputs = (
      _production_controller_outputs(
        frames,
        motion_sensor=self.motion_sensor,
        enable_radar_tracks=self.enable_radar_tracks,
        cut_in_sensitivity=self.cut_in_sensitivity,
      )
      if self.enable_radar_tracks != 1
      else (None,) * len(frames)
    )
    if self.enable_radar_tracks != 1:
      lead_one_values = tuple(
        output.lead_one for output in production_outputs
      )
    elif lead_one_outputs is None:
      matcher = VisionRadarMatcher()
      lead_one_results = []
      for frame in frames:
        aligned_points = radar_points_at_model_time(frame)
        stationary_points = (
          aligned_points
          if self.motion_sensor == "corner"
          else tuple(
            point for point in aligned_points
            if not point.source.startswith("corner")
          )
        )
        match = match_dpath_primary_lead(
          matcher,
          _controller_model(frame),
          aligned_points,
          frame.path,
          time_s=frame.time_s,
          stationary_points=stationary_points,
          enable_radar_tracks=self.enable_radar_tracks,
          yaw_rate_rad_s=frame.yaw_rate_rad_s,
        )
        if match is not None:
          lead_one_results.append(lead_from_vision_match(match))
        else:
          vision = matcher.vision_fallback
          lead_one_results.append(
            lead_from_vision(
              vision,
              frame.path,
              frame.v_ego,
              model_v_ego=frame.v_ego,
            )
            if (
              vision is not None
              and vision_only_lead_allowed(
                self.enable_radar_tracks,
              )
            )
            else None
          )
      lead_one_values = tuple(lead_one_results)
    else:
      lead_one_values = tuple(lead_one_outputs)
    trajectory_values = (
      tuple(trajectories)
      if trajectories is not None
      else radar_trajectory_series(
        frames,
        self.motion_sensor,
        lead_one_values,
        directional_min_consistency=(
          self.motion_sensitivity.directional_min_consistency
        ),
      )
    )
    self.motion_points = (
      tuple(motion_points)
      if motion_points is not None
      else tuple(
        tuple(
          point
          for point in selected_points
          if (
            (point.source, point.track_id) in visible_identities
            or (point.source, point.track_id) in predictions
          )
        )
        for frame, lead, predictions in zip(
          frames,
          lead_one_values,
          trajectory_values,
          strict=True,
        )
        for selected_points in (
          motion_points_at_model_time(frame, self.motion_sensor),
        )
        for visible_identities in ({
          (point.source, point.track_id)
          for point in visible_motion_points(
            selected_points,
            frame.path,
            (
              _finite(lead["dRel"])
              if lead is not None and bool(lead.get("status", True))
              else None
            ),
          )
        },)
      )
    )
    if (
      len(self.motion_points) != len(frames)
      or len(trajectory_values) != len(frames)
      or len(lead_one_values) != len(frames)
    ):
      raise ValueError("cached predictor inputs must align with radar frames")
    selections: list[Selection] = []
    front_kinematic_associator = FrontRadarKinematicAssociator()
    lead_two_tracker = DPathLeadTwoTracker()
    stationary_shadow_tracker = DPathStationaryShadowTracker()
    stationary_primary_handoff_tracker = (
      DPathStationaryPrimaryHandoffTracker()
    )
    primary_cut_out_predictor = RadarMotionPredictor()
    decision_tracker = RadarMotionDecisionTracker(
      threshold=self.decision_threshold,
      confirmation_s=self.motion_sensitivity.confirmation_s,
    )
    predecel_tracker = CornerCutInPredecelTracker()
    for frame, predictions, lead_one, production_output in zip(
      frames,
      trajectory_values,
      lead_one_values,
      production_outputs,
      strict=True,
    ):
      selected_points = motion_points_at_model_time(
        frame, self.motion_sensor,
      )
      all_aligned_points = radar_points_at_model_time(frame)
      front_motion_points = tuple(
        point for point in all_aligned_points
        if point.source == "frontRadar"
      )
      primary_cut_out_predictions = primary_cut_out_predictor.update(
        frame.time_s,
        front_motion_points,
        frame.path,
        frame.v_ego,
      )
      primary_track_id = (
        int(lead_one.get("radarTrackId", -1))
        if lead_one is not None and lead_one.get("radar")
        else -1
      )
      primary_cut_out_probability = max((
        float(prediction.cut_out_probability)
        for prediction in primary_cut_out_predictions.values()
        if prediction.track_id == primary_track_id
      ), default=0.0)
      front_kinematic_matches = front_kinematic_associator.update(
        all_aligned_points,
      )
      active_identity = lead_two_tracker.active_identity
      protected_identities = (
        ()
        if active_identity is None
        else ((active_identity[0], active_identity[1]),)
      )
      visible_points = visible_motion_points(
        selected_points,
        frame.path,
        (
          _finite(lead_one["dRel"])
          if lead_one is not None and bool(lead_one.get("status", True))
          else None
        ),
        protected_identities,
      )
      point_by_identity = {
        (point.source, point.track_id): point
        for point in visible_points
      }
      point_by_identity.update({
        (point.source, point.track_id): point
        for point in selected_points
        if (point.source, point.track_id) in predictions
      })
      vision = vision_lead_from_model(_controller_model(frame))
      predictions = {
        identity: prediction_with_validation_lookahead(
          (
          apply_vision_bracket_cutin_support(
            prediction,
            point,
            all_aligned_points,
            vision,
            lead_one,
          )
          if (
            point := point_by_identity.get(
              (prediction.source, prediction.track_id),
            )
          ) is not None
          else prediction
          ),
          self.maximum_lookahead_s,
        )
        for identity, prediction in predictions.items()
      }
      allowed_predictions = {
        identity: prediction
        for identity, prediction in predictions.items()
        if (
          (point := point_by_identity.get(identity)) is not None
          and turning_corner_path_entry_allowed(
            prediction.source,
            point.y_rel,
            prediction.d_path,
            frame.yaw_rate_rad_s,
            cross_sensor_confirmed=(
              identity in front_kinematic_matches
            ),
          )
        )
      }
      predecel = predecel_tracker.update(
        frame.time_s,
        (
          RadarMotionCutIn(
            prediction,
            corner_cutin_predecel_score(
              prediction,
              point.d_rel,
              point.v_rel,
              v_ego=frame.v_ego,
              cross_sensor_confirmed=(
                (prediction.source, prediction.track_id)
                in front_kinematic_matches
              ),
            ),
          )
          for prediction in allowed_predictions.values()
          if (
            self.motion_sensitivity.cut_in_enabled
            and (
              point := point_by_identity.get(
                (prediction.source, prediction.track_id),
              )
            ) is not None
          )
        ),
      )
      predecel_candidate = None
      if predecel is not None:
        point = point_by_identity.get((
          predecel.prediction.source,
          predecel.prediction.track_id,
        ))
        if point is not None:
          predecel_candidate = replace(
            _shadow_candidate(predecel.prediction),
            score=predecel.score,
            reason="corner CUT-IN pre-deceleration risk",
            decision_threshold=0.0,
            d_rel=point.d_rel,
            y_rel=point.y_rel,
            v_lead=point.v_lead,
            stage="PRE-DECEL",
            source=predecel.prediction.source,
          )
      raw_diagnostics = tuple(sorted(
        (_shadow_candidate(prediction) for prediction in predictions.values()),
        key=lambda candidate: (
          -candidate.current_path_occupancy,
          -candidate.score,
          candidate.d_rel if candidate.d_rel is not None else math.inf,
        ),
      ))
      decision = decision_tracker.update(
        frame.time_s,
        (
          allowed_predictions.values()
          if self.motion_sensitivity.cut_in_enabled
          else ()
        ),
      )
      confirmed_by_identity = {
        (
          cutin.prediction.source,
          cutin.prediction.track_id,
          cutin.prediction.continuity_id,
        ): cutin
        for cutin in decision.confirmed
      }
      lead_candidates = []
      primary_row_waiting_keys: set[tuple[str, int]] = set()
      primary_future_blocked_keys: set[tuple[str, int]] = set()
      for prediction in predictions.values():
        point = point_by_identity.get((prediction.source, prediction.track_id))
        if point is None:
          continue
        identity = (
          prediction.source,
          prediction.track_id,
          prediction.continuity_id,
        )
        cutin = confirmed_by_identity.get(identity)
        front_motion_supported = front_cutin_motion_supported(
          prediction.source,
          prediction.d_path_rate_long,
          d_rel=point.d_rel,
          v_rel=point.v_rel,
          d_path=prediction.d_path,
          d_path_rate_short=getattr(
            prediction, "d_path_rate_short", prediction.d_path_rate_long,
          ),
          reported_normal_speed=getattr(
            prediction, "reported_normal_speed", 0.0,
          ),
          current_path_occupancy=prediction.current_path_occupancy,
          predicted_path_overlap_s=getattr(
            prediction, "predicted_path_overlap_s", 0.0,
          ),
          directional_inward_displacement_m=getattr(
            prediction, "directional_inward_displacement_m", 0.0,
          ),
          directional_consistency=getattr(
            prediction, "directional_consistency", 0.0,
          ),
          directional_inward_sample_ratio=getattr(
            prediction, "directional_inward_sample_ratio", 0.0,
          ),
          corner_directional_entry=(
            getattr(prediction, "near_side_directional_entry", False)
            or getattr(prediction, "lane_boundary_directional_entry", False)
          ),
          tracked_close_entry=getattr(
            prediction, "front_tracked_close_entry", False,
          ),
          minimum_directional_consistency=(
            self.motion_sensitivity.directional_min_consistency
          ),
        )
        lead_point = prefer_front_radar_kinematics(
          point, all_aligned_points, front_kinematic_matches,
        )
        lead_d_path = (
          project_to_model_path(
            frame.path, lead_point.d_rel, lead_point.y_rel,
          ).d_path
          if lead_point is not point
          else prediction.d_path
        )
        lead = lead_from_radar_point(
          lead_point,
          lead_d_path,
          0.03,
          (
            cutin.score
            if cutin is not None
            else prediction.path_entry_probability
          ),
        )
        if lead_duplicates_primary(lead, lead_one):
          if lead_two_tracker.active_identity == identity:
            lead_two_tracker.reset()
          continue
        primary_competition_allowed = cutin_can_compete_with_primary(
          lead,
          lead_one,
          projected_path_entry=prediction.time_to_entry_s is not None,
          entry_horizon_s=getattr(
            prediction,
            "predicted_path_overlap_start_s",
            prediction.time_to_entry_s,
          ),
        )
        confirmed_cutin = (
          self.motion_sensitivity.cut_in_enabled
          and cutin is not None
          and front_motion_supported
          and primary_competition_allowed
        )
        if cutin is not None and not confirmed_cutin:
          waiting_key = (prediction.source, prediction.track_id)
          primary_row_waiting_keys.add(waiting_key)
          if not primary_competition_allowed:
            primary_future_blocked_keys.add(waiting_key)
        lead_candidates.append(DPathLeadCandidate(
          lead=lead,
          source=prediction.source,
          track_id=prediction.track_id,
          continuity_id=prediction.continuity_id,
          retainable=(
            prediction.current_path_occupancy
            or prediction.d_path * prediction.d_path_rate_long <= 0.0
          ),
          confirmed_cutin=confirmed_cutin,
        ))
      stationary_primary_candidates = []
      for point in selected_points:
        if not point.source.startswith("corner"):
          continue
        d_path = project_to_model_path(
          frame.path, point.d_rel, point.y_rel,
        ).d_path
        stationary_primary_candidates.append(DPathLeadCandidate(
          lead=lead_from_radar_point(point, d_path, 0.03, 0.0),
          source=point.source,
          track_id=point.track_id,
          continuity_id=0,
          retainable=True,
          confirmed_cutin=False,
        ))
      stationary_primary_handoff = (
        stationary_primary_handoff_tracker.update(
          frame.time_s,
          lead_one,
          stationary_primary_candidates,
          active_identity,
        )
      )
      if (
        stationary_primary_handoff is not None
        and not any(
          candidate.identity == stationary_primary_handoff.identity
          for candidate in lead_candidates
        )
      ):
        lead_candidates.append(stationary_primary_handoff)
      stationary_shadow_inputs = []
      for point in visible_motion_points(
        front_motion_points, frame.path, None,
      ):
        identity = (point.source, point.track_id, 0)
        retained_stationary_shadow = active_identity == identity
        corner_supported = stationary_shadow_corner_supported(
          point, all_aligned_points, frame.path,
        )
        if (
          point.radar_track_state < 2
          or not (corner_supported or retained_stationary_shadow)
        ):
          continue
        d_path = project_to_model_path(
          frame.path, point.d_rel, point.y_rel,
        ).d_path
        lead = lead_from_radar_point(
          point, d_path, 0.03, primary_cut_out_probability,
        )
        candidate = DPathLeadCandidate(
          lead=lead,
          source=point.source,
          track_id=point.track_id,
          continuity_id=0,
          retainable=True,
          confirmed_cutin=False,
        )
        if corner_supported:
          stationary_shadow_inputs.append(candidate)
        if (
          retained_stationary_shadow
          and point.track_id != primary_track_id
        ):
          lead_candidates.append(candidate)
      stationary_shadow = stationary_shadow_tracker.update(
        frame.time_s,
        lead_one,
        primary_cut_out_probability,
        stationary_shadow_inputs,
      )
      if (
        stationary_shadow is not None
        and stationary_shadow.confirmed_stationary_shadow
        and stationary_shadow.track_id != primary_track_id
        and not any(
          candidate.identity == stationary_shadow.identity
          for candidate in lead_candidates
        )
      ):
        lead_candidates.append(stationary_shadow)
      if (
        active_identity is not None
        and not any(
          candidate.identity == active_identity
          for candidate in lead_candidates
        )
      ):
        source, track_id, continuity_id = active_identity
        point = point_by_identity.get((source, track_id))
        if point is not None:
          lead_point = prefer_front_radar_kinematics(
            point, all_aligned_points, front_kinematic_matches,
          )
          d_path = project_to_model_path(
            frame.path,
            lead_point.d_rel,
            lead_point.y_rel,
          ).d_path
          lead = lead_from_radar_point(
            lead_point, d_path, 0.03, 0.0,
          )
          if lead_duplicates_primary(lead, lead_one):
            lead_two_tracker.reset()
          else:
            lead_candidates.append(DPathLeadCandidate(
              lead=lead,
              source=source,
              track_id=track_id,
              continuity_id=continuity_id,
              retainable=True,
              confirmed_cutin=False,
            ))
      lead_selection = lead_two_tracker.update(
        frame.time_s,
        lead_one,
        lead_candidates,
        frame.v_ego,
      )
      confirmed_keys = {
        (cutin.prediction.source, cutin.prediction.track_id)
        for cutin in decision.confirmed
      }
      confirmed_scores = {
        (cutin.prediction.source, cutin.prediction.track_id): cutin.score
        for cutin in decision.confirmed
      }
      selected_cutin_ids = {
        int(lead["radarTrackId"]) for lead in lead_selection.cutins
      }
      diagnostics = tuple(
        replace(
          candidate,
          stage=(
            "CUT-IN"
            if candidate.track_id in selected_cutin_ids
            else (
              (
                "L1-FUTURE"
                if (candidate.source, candidate.track_id)
                in primary_future_blocked_keys
                else "ROW-WAIT"
              )
              if (candidate.source, candidate.track_id)
              in primary_row_waiting_keys else "FILTERED"
            )
          ),
          score=max(
            candidate.score,
            confirmed_scores[(candidate.source, candidate.track_id)],
          ),
        )
        if (candidate.source, candidate.track_id) in confirmed_keys
        else candidate
        for candidate in raw_diagnostics
      )
      front = tuple(candidate for candidate in diagnostics if not candidate.source.startswith("corner"))
      corner = tuple(candidate for candidate in diagnostics if candidate.source.startswith("corner"))
      decisions = tuple(
        candidate for candidate in diagnostics
        if (
          (candidate.source, candidate.track_id) in confirmed_keys
          and candidate.track_id in selected_cutin_ids
        )
      )
      selections.append(Selection(
        lead_one=_controller_candidate(
          frame,
          lead_one,
          "RadarMotion leadOne",
          {
            (prediction.source, prediction.track_id): prediction.continuity_id
            for prediction in predictions.values()
          },
        ),
        lead_two=_controller_candidate(
          frame,
          (
            production_output.lead_two
            if production_output is not None
            else lead_selection.lead_two
          ),
          "RadarMotion leadTwo",
          {
            (prediction.source, prediction.track_id): prediction.continuity_id
            for prediction in predictions.values()
          },
        ),
        front_candidates=front,
        corner_candidates=corner,
        cutin_diagnostics=diagnostics,
        decision_cutin_candidates=decisions,
        cutin_predecel_candidate=predecel_candidate,
      ))
    self.trajectories = trajectory_values
    self.lead_one_outputs = lead_one_values
    self.selections = tuple(selections)

  def select(self, frame: RadarFrame, frame_index: int | None = None) -> Selection:
    if frame_index is None:
      raise ValueError("shadow selector requires a frame index")
    return self.selections[frame_index]


def _occupancy_v2_candidate(
  estimate: OccupancyEstimate,
  point: RadarPoint,
) -> Candidate:
  prediction = estimate.evidence
  overlap_eta = (
    "--"
    if estimate.time_to_overlap_s is None
    else f"{estimate.time_to_overlap_s:.2f}s"
  )
  time_gap = "--" if estimate.time_gap_s is None else f"{estimate.time_gap_s:.2f}s"
  collision_eta = (
    "--"
    if estimate.time_to_collision_s is None
    else f"{estimate.time_to_collision_s:.2f}s"
  )
  return Candidate(
    track_id=prediction.track_id,
    score=estimate.lead_score,
    reason="probabilistic lane occupancy V2",
    decision_threshold=0.0,
    d_rel=point.d_rel,
    y_rel=point.y_rel,
    v_lead=point.v_lead,
    base_score=estimate.risk_score,
    temporal_score=estimate.confidence,
    current_path_occupancy=prediction.current_path_occupancy,
    stage=estimate.stage.name,
    detail=(
      f"occ={estimate.occupancy_score:.2f} "
      + f"risk={estimate.risk_score:.2f} lead={estimate.lead_score:.2f} "
      + f"intent={estimate.intent_score:.2f} conf={estimate.confidence:.2f} "
      + f"clear={estimate.path_clearance_m:.2f}m "
      + f"inward={estimate.inward_rate_mps:.2f}m/s eta={overlap_eta} "
      + f"gap={time_gap} ttc={collision_eta} "
      + f"urgency={estimate.control_urgency:.2f}"
    ),
    source=prediction.source,
    continuity_id=prediction.continuity_id,
  )


class RadarOccupancyV2Selector:
  """Replay V1 with the bounded early-control additions from occupancy V2."""

  name = "probabilistic-occupancy-v2"

  def __init__(
    self,
    frames: Sequence[RadarFrame],
    *,
    baseline: RadarMotionShadowSelector | None = None,
    enable_radar_tracks: int = 2,
  ) -> None:
    baseline = baseline or RadarMotionShadowSelector(
      frames,
      enable_radar_tracks=enable_radar_tracks,
    )
    occupancy_model = RadarOccupancyModelV2()
    front_associator = FrontRadarKinematicAssociator()
    lead_two_tracker = DPathLeadTwoTracker()
    selections: list[Selection] = []
    estimate_series: list[tuple[OccupancyEstimate, ...]] = []

    for index, (frame, predictions, lead_one) in enumerate(zip(
      frames,
      baseline.trajectories,
      baseline.lead_one_outputs,
      strict=True,
    )):
      selected_points = baseline.motion_points[index]
      all_points = radar_points_at_model_time(frame)
      front_matches = front_associator.update(all_points)
      cross_sensor_identities = set(front_matches)
      cross_sensor_identities.update(
        (point.source, point.track_id)
        for point in front_matches.values()
      )
      point_by_identity = {
        (point.source, point.track_id): point
        for point in selected_points
      }
      point_by_identity.update({
        (point.source, point.track_id): point
        for point in all_points
        if (point.source, point.track_id) in predictions
      })
      vision = vision_lead_from_model(_controller_model(frame))
      evidence_values: list[OccupancyEvidence] = []
      point_by_v2_identity: dict[tuple[str, int], RadarPoint] = {}
      for prediction in predictions.values():
        identity = prediction.source, prediction.track_id
        point = point_by_identity.get(identity)
        if point is None:
          continue
        vision_supported = (
          apply_vision_bracket_cutin_support(
            prediction,
            point,
            all_points,
            vision,
            lead_one,
          ).reason
          == "vision-bracketed physical CUT-IN"
        )
        evidence = OccupancyEvidence(
          source=prediction.source,
          track_id=prediction.track_id,
          continuity_id=prediction.continuity_id,
          d_rel=point.d_rel,
          v_rel=point.v_rel,
          v_lead=point.v_lead,
          v_ego=frame.v_ego,
          d_path=prediction.d_path,
          d_path_rate_short=prediction.d_path_rate_short,
          d_path_rate_long=prediction.d_path_rate_long,
          reported_normal_speed=prediction.reported_normal_speed,
          normal_speed_disagreement=prediction.normal_speed_disagreement,
          directional_inward_displacement_m=(
            prediction.directional_inward_displacement_m
          ),
          directional_consistency=prediction.directional_consistency,
          directional_inward_sample_ratio=(
            prediction.directional_inward_sample_ratio
          ),
          motion_consistency=prediction.motion_consistency,
          recent_motion_support=prediction.recent_motion_support,
          history_count=prediction.history_count,
          uncertainty=prediction.uncertainty,
          current_path_occupancy=prediction.current_path_occupancy,
          cross_sensor_confirmed=identity in cross_sensor_identities,
          vision_supported=vision_supported,
        )
        evidence_values.append(evidence)
        point_by_v2_identity[evidence.identity] = point

      estimates = occupancy_model.update(frame.time_s, evidence_values)
      estimate_series.append(estimates)
      estimate_by_identity = {
        estimate.evidence.identity: estimate
        for estimate in estimates
      }
      diagnostics = tuple(sorted(
        (
          _occupancy_v2_candidate(
            estimate,
            point_by_v2_identity[estimate.evidence.identity],
          )
          for estimate in estimates
        ),
        key=lambda candidate: (
          -OccupancyStage[candidate.stage],
          -candidate.score,
          candidate.d_rel if candidate.d_rel is not None else math.inf,
        ),
      ))
      baseline_selection = baseline.selections[index]
      control_eligible_identities = {
        estimate.evidence.identity
        for estimate in estimates
        if early_control_eligible(estimate)
      }
      lead_candidates = []
      continuity_by_identity = {}
      for identity, point in point_by_identity.items():
        prediction = predictions.get(identity)
        if prediction is None:
          continue
        estimate = estimate_by_identity.get((
          prediction.source,
          prediction.continuity_id,
        ))
        if estimate is None:
          continue
        continuity_by_identity[identity] = prediction.continuity_id
        lead_point = prefer_front_radar_kinematics(
          point, all_points, front_matches,
        )
        lead_d_path = (
          project_to_model_path(
            frame.path, lead_point.d_rel, lead_point.y_rel,
          ).d_path
          if lead_point is not point
          else prediction.d_path
        )
        lead = lead_from_radar_point(
          lead_point,
          lead_d_path,
          0.03,
          estimate.lead_score,
        )
        if lead_duplicates_primary(lead, lead_one):
          control_eligible_identities.discard(estimate.evidence.identity)
          if lead_two_tracker.active_identity == (
            prediction.source,
            prediction.track_id,
            prediction.continuity_id,
          ):
            lead_two_tracker.reset()
          continue
        if estimate.evidence.identity not in control_eligible_identities:
          continue
        lead_candidates.append(DPathLeadCandidate(
          lead=lead,
          source=prediction.source,
          track_id=prediction.track_id,
          continuity_id=prediction.continuity_id,
          retainable=estimate.stage >= OccupancyStage.LIMIT,
          confirmed_cutin=estimate.stage >= OccupancyStage.LEAD,
        ))

      v2_limit_candidates = tuple(
        candidate for candidate in diagnostics
        if (
          OccupancyStage[candidate.stage] >= OccupancyStage.LIMIT
          and (candidate.source, candidate.continuity_id)
          in control_eligible_identities
        )
      )
      v2_decision_candidates = tuple(
        candidate for candidate in diagnostics
        if (
          OccupancyStage[candidate.stage] >= OccupancyStage.LEAD
          and (candidate.source, candidate.continuity_id)
          in control_eligible_identities
        )
      )
      baseline_decision_keys = {
        (candidate.source, candidate.track_id)
        for candidate in baseline_selection.decision_cutin_candidates
      }
      decision_candidates = (
        baseline_selection.decision_cutin_candidates
        + tuple(
          candidate for candidate in v2_decision_candidates
          if (candidate.source, candidate.track_id)
          not in baseline_decision_keys
        )
      )
      predecel_candidate = min(
        (
          *v2_limit_candidates,
          *(
            ()
            if baseline_selection.cutin_predecel_candidate is None
            else (baseline_selection.cutin_predecel_candidate,)
          ),
        ),
        key=lambda candidate: (
          candidate.d_rel if candidate.d_rel is not None else math.inf,
          -float(candidate.base_score or 0.0),
        ),
        default=None,
      )

      lead_selection = lead_two_tracker.update(
        frame.time_s,
        lead_one,
        lead_candidates,
        frame.v_ego,
      )
      selected_ids = {
        int(lead["radarTrackId"])
        for lead in lead_selection.cutins
      }
      active_candidates = tuple(
        candidate for candidate in v2_decision_candidates
        if candidate.track_id in selected_ids
      )
      active_candidates = (
        baseline_selection.active_cutin_candidates
        + active_candidates
      )
      v2_lead_two = _controller_candidate(
        frame,
        lead_selection.lead_two,
        "RadarOccupancy V2 leadTwo",
        continuity_by_identity,
      )
      if (
        v2_lead_two is None
        and baseline_selection.lead_two is not None
      ):
        # V2 is an early-control augmentation. V1 retains ownership whenever
        # no independently eligible V2 lead is ready.
        v2_lead_two = baseline_selection.lead_two
      selections.append(Selection(
        lead_one=baseline_selection.lead_one,
        lead_two=v2_lead_two,
        front_candidates=tuple(
          candidate for candidate in diagnostics
          if not candidate.source.startswith("corner")
        ),
        corner_candidates=tuple(
          candidate for candidate in diagnostics
          if candidate.source.startswith("corner")
        ),
        cutin_diagnostics=diagnostics,
        decision_cutin_candidates=decision_candidates,
        active_cutin_candidates=active_candidates,
        cutin_predecel_candidate=predecel_candidate,
      ))

    self.motion_sensor = baseline.motion_sensor
    self.enable_radar_tracks = baseline.enable_radar_tracks
    self.cut_in_sensitivity = baseline.cut_in_sensitivity
    self.motion_sensitivity = baseline.motion_sensitivity
    self.decision_threshold = baseline.decision_threshold
    self.maximum_lookahead_s = baseline.maximum_lookahead_s
    self.motion_points = baseline.motion_points
    self.trajectories = baseline.trajectories
    self.lead_one_outputs = baseline.lead_one_outputs
    self.baseline = baseline
    self.estimates = tuple(estimate_series)
    self.selections = tuple(selections)

  def select(
    self,
    frame: RadarFrame,
    frame_index: int | None = None,
  ) -> Selection:
    del frame
    if frame_index is None:
      raise ValueError("occupancy V2 selector requires a frame index")
    return self.selections[frame_index]


def _v3_candidate(
  estimate: V3Estimate,
  candidate: Candidate,
) -> Candidate:
  evidence: V3Evidence = estimate.evidence
  occupancy = evidence.occupancy
  overlap = (
    "--" if occupancy.time_to_overlap_s is None
    else f"{occupancy.time_to_overlap_s:.2f}s"
  )
  return replace(
    candidate,
    score=estimate.score,
    reason=f"V3 {estimate.reason}",
    stage=estimate.stage.name,
    detail=" ".join((
      f"{estimate.reason} risk={occupancy.risk_score:.2f}",
      f"lead={occupancy.lead_score:.2f}",
      f"intent={occupancy.intent_score:.2f}",
      f"conf={occupancy.confidence:.2f}",
      f"urgency={occupancy.control_urgency:.2f}",
      f"inward={occupancy.inward_rate_mps:.2f} eta={overlap}",
    )),
  )


class RadarOccupancyV3Selector:
  """V2 safety floor plus independent staged V3 control extensions."""

  name = "radar-occupancy-v3"

  def __init__(
    self,
    frames: Sequence[RadarFrame],
    *,
    baseline: RadarMotionShadowSelector | None = None,
    v2_selector: RadarOccupancyV2Selector | None = None,
    enable_radar_tracks: int = 2,
    cut_in_sensitivity: int = VALIDATION_DEFAULT_SENSITIVITY,
  ) -> None:
    baseline = baseline or RadarMotionShadowSelector(
      frames,
      cut_in_sensitivity=cut_in_sensitivity,
      enable_radar_tracks=enable_radar_tracks,
    )
    v2_selector = v2_selector or RadarOccupancyV2Selector(
      frames,
      baseline=baseline,
      enable_radar_tracks=enable_radar_tracks,
    )
    model = RadarOccupancyModelV3()
    associator = FrontRadarKinematicAssociator()
    lead_two_tracker = DPathLeadTwoTracker()
    selections: list[Selection] = []
    estimate_series: list[tuple[V3Estimate, ...]] = []

    for index, (frame, predictions, primary, base_selection, v2_estimates) in enumerate(zip(
      frames,
      baseline.trajectories,
      baseline.lead_one_outputs,
      v2_selector.selections,
      v2_selector.estimates,
      strict=True,
    )):
      estimates = model.update(frame.time_s, v2_estimates)
      estimate_series.append(estimates)
      all_points = radar_points_at_model_time(frame)
      front_matches = associator.update(all_points)
      selected_points = baseline.motion_points[index]
      point_by_identity = {
        (point.source, point.track_id): point for point in selected_points
      }
      point_by_identity.update({
        (point.source, point.track_id): point
        for point in all_points
        if (point.source, point.track_id) in predictions
      })
      base_candidates = {
        (candidate.source, candidate.continuity_id): candidate
        for candidate in base_selection.cutin_diagnostics
      }
      v3_by_identity = {
        estimate.evidence.identity: estimate for estimate in estimates
      }
      diagnostics = tuple(
        _v3_candidate(v3_by_identity[identity], candidate)
        if identity in v3_by_identity else candidate
        for identity, candidate in base_candidates.items()
      )

      extra_limits = []
      extra_decisions = []
      lead_candidates = []
      continuity_by_identity = {}
      for estimate in estimates:
        if estimate.reason == "baseline" or estimate.stage < V3Stage.PREDECEL:
          continue
        occupancy = estimate.evidence.occupancy
        evidence = occupancy.evidence
        identity = evidence.source, evidence.track_id
        prediction = predictions.get(identity)
        point = point_by_identity.get(identity)
        candidate = base_candidates.get(evidence.identity)
        if prediction is None or point is None or candidate is None:
          continue
        lead_point = prefer_front_radar_kinematics(
          point, all_points, front_matches,
        )
        lead_d_path = (
          project_to_model_path(
            frame.path, lead_point.d_rel, lead_point.y_rel,
          ).d_path
          if lead_point is not point else prediction.d_path
        )
        lead = lead_from_radar_point(
          lead_point, lead_d_path, 0.03, estimate.score,
        )
        if lead_duplicates_primary(lead, primary) or not cutin_can_compete_with_primary(
          lead,
          primary,
          projected_path_entry=True,
          entry_horizon_s=occupancy.time_to_overlap_s,
        ):
          continue
        v3_candidate = _v3_candidate(estimate, candidate)
        extra_limits.append(v3_candidate)
        continuity_by_identity[identity] = prediction.continuity_id
        if estimate.stage >= V3Stage.LEAD:
          extra_decisions.append(v3_candidate)
          lead_candidates.append(DPathLeadCandidate(
            lead=lead,
            source=prediction.source,
            track_id=prediction.track_id,
            continuity_id=prediction.continuity_id,
            retainable=True,
            confirmed_cutin=True,
          ))

      lead_selection = lead_two_tracker.update(
        frame.time_s, primary, lead_candidates, frame.v_ego,
      )
      v3_lead_two = _controller_candidate(
        frame,
        lead_selection.lead_two,
        "RadarOccupancy V3 leadTwo",
        continuity_by_identity,
      )
      lead_two = v3_lead_two or base_selection.lead_two
      base_decision_keys = {
        (candidate.source, candidate.track_id)
        for candidate in base_selection.decision_cutin_candidates
      }
      decision = base_selection.decision_cutin_candidates + tuple(
        candidate for candidate in extra_decisions
        if (candidate.source, candidate.track_id) not in base_decision_keys
      )
      selected_ids = {
        int(lead["radarTrackId"]) for lead in lead_selection.cutins
      }
      active = base_selection.active_cutin_candidates + tuple(
        candidate for candidate in extra_decisions
        if candidate.track_id in selected_ids
      )
      predecel = min(
        (
          *extra_limits,
          *(
            () if base_selection.cutin_predecel_candidate is None
            else (base_selection.cutin_predecel_candidate,)
          ),
        ),
        key=lambda candidate: (
          candidate.d_rel if candidate.d_rel is not None else math.inf
        ),
        default=None,
      )
      selections.append(Selection(
        lead_one=base_selection.lead_one,
        lead_two=lead_two,
        front_candidates=tuple(
          candidate for candidate in diagnostics
          if not candidate.source.startswith("corner")
        ),
        corner_candidates=tuple(
          candidate for candidate in diagnostics
          if candidate.source.startswith("corner")
        ),
        cutin_diagnostics=diagnostics,
        decision_cutin_candidates=decision,
        active_cutin_candidates=active,
        external_candidates=decision,
        active_external_candidates=active,
        cutin_predecel_candidate=predecel,
      ))

    self.motion_sensor = baseline.motion_sensor
    self.enable_radar_tracks = baseline.enable_radar_tracks
    self.cut_in_sensitivity = baseline.cut_in_sensitivity
    self.motion_sensitivity = baseline.motion_sensitivity
    self.decision_threshold = baseline.decision_threshold
    self.maximum_lookahead_s = baseline.maximum_lookahead_s
    self.motion_points = baseline.motion_points
    self.trajectories = baseline.trajectories
    self.lead_one_outputs = baseline.lead_one_outputs
    self.baseline = baseline
    self.v2_selector = v2_selector
    self.estimates = tuple(estimate_series)
    self.selections = tuple(selections)

  def select(
    self, frame: RadarFrame, frame_index: int | None = None,
  ) -> Selection:
    del frame
    if frame_index is None:
      raise ValueError("occupancy V3 selector requires a frame index")
    return self.selections[frame_index]


def visual_replay_cache_path(
  cache_dir: Path,
  log_path: Path,
  *,
  motion_mode: str,
  cut_in_sensitivity: int,
  probability_override: float | None,
  enable_radar_tracks: int,
) -> Path:
  """Return the private cache path for one exact visual replay setup."""
  log_stat = log_path.stat()
  identity = json.dumps({
    "version": VISUAL_REPLAY_CACHE_VERSION,
    "source_fingerprint": radar_replay_source_fingerprint(),
    "log": str(log_path.resolve()),
    "log_size": log_stat.st_size,
    "log_mtime_ns": log_stat.st_mtime_ns,
    "motion_mode": motion_mode,
    "cut_in_sensitivity": int(cut_in_sensitivity),
    "probability_override": probability_override,
    "enable_radar_tracks": int(enable_radar_tracks),
  }, sort_keys=True, separators=(",", ":"))
  digest = hashlib.sha256(identity.encode("utf-8")).hexdigest()
  return cache_dir / f"visual-replay-{digest}.pickle"


def save_visual_replay_cache(
  cache_path: Path,
  frames: list[RadarFrame],
  selector: RadarOccupancyV2Selector,
  v3_selector: RadarOccupancyV3Selector,
  production_selector: ProductionDPathSelector,
) -> None:
  """Atomically save a fully built replay for the foreground reviewer."""
  cache_path.parent.mkdir(parents=True, exist_ok=True)
  temporary_path = cache_path.with_name(
    f"{cache_path.name}.{os.getpid()}.tmp",
  )
  try:
    with temporary_path.open("wb") as cache_file:
      pickle.dump(
        {
          "version": VISUAL_REPLAY_CACHE_VERSION,
          "source_fingerprint": radar_replay_source_fingerprint(),
          "frames": frames,
          "selector": selector,
          "v3_selector": v3_selector,
          "production_selector": production_selector,
        },
        cache_file,
        protocol=pickle.HIGHEST_PROTOCOL,
      )
    temporary_path.replace(cache_path)
  finally:
    temporary_path.unlink(missing_ok=True)


def load_visual_replay_cache(
  cache_path: Path,
) -> tuple[
  list[RadarFrame],
  RadarOccupancyV2Selector,
  RadarOccupancyV3Selector,
  ProductionDPathSelector,
] | None:
  """Load a prepared replay, returning None for stale or damaged data."""
  if not cache_path.is_file():
    return None
  try:
    with cache_path.open("rb") as cache_file:
      payload = pickle.load(cache_file)
    if (
      not isinstance(payload, dict)
      or payload.get("version") != VISUAL_REPLAY_CACHE_VERSION
      or payload.get("source_fingerprint")
      != radar_replay_source_fingerprint()
      or not isinstance(payload.get("frames"), list)
      or not isinstance(payload.get("selector"), RadarOccupancyV2Selector)
      or not isinstance(payload.get("v3_selector"), RadarOccupancyV3Selector)
      or not isinstance(
        payload.get("production_selector"), ProductionDPathSelector,
      )
    ):
      return None
    return (
      payload["frames"],
      payload["selector"],
      payload["v3_selector"],
      payload["production_selector"],
    )
  except (EOFError, OSError, pickle.PickleError, AttributeError, TypeError):
    return None


def _route_replay_module() -> Any:
  cluster_dir = Path(__file__).resolve().parents[2] / "cluster"
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
      elif 300 <= track_id < 412:
        source = "corner430"
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
      radar_track_state=int(_finite(getattr(point, "trackState", 0))),
    ))
  return tuple(copied)


def _copy_points(message: Any) -> tuple[RadarPoint, ...]:
  return _copy_track_points(message.points)


def _with_group2_front_track_state(
  point: RadarPoint,
  event_ns: int,
  quality: dict[int, tuple[int, int]],
) -> RadarPoint:
  if point.source != "frontRadar" or not 32 <= point.track_id < 64:
    return point
  state = quality.get(point.track_id)
  if state is None:
    return point
  quality_ns, track_state = state
  age_s = max(0.0, (event_ns - quality_ns) / 1e9)
  if age_s > VALIDATION_GROUP2_QUALITY_MAX_AGE_S:
    return point
  return replace(point, radar_track_state=track_state)


def _xy_points(data: Any) -> tuple[tuple[float, float], ...]:
  return tuple(
    (_finite(data.x[index]), _finite(data.y[index]))
    for index in range(min(len(data.x), len(data.y)))
  )


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


def _production_controller_replay(
  frames: Sequence[RadarFrame],
  *,
  motion_sensor: str,
  enable_radar_tracks: int,
  cut_in_sensitivity: int,
) -> tuple[tuple[Any, ...], tuple[tuple[Any, ...], ...]]:
  """Replay the real dPath controller for option-dependent lead roles."""
  controller = DPathRadarController(
    prefer_corner_radar=motion_sensor == "corner",
    enable_radar_tracks=enable_radar_tracks,
    cut_in_sensitivity=cut_in_sensitivity,
  )
  outputs = []
  estimates = []
  for frame in frames:
    controller.front_radar_measurement_delay_s = max(
      0.0, float(frame.radar_delay_s),
    )
    radar_points = (
      tuple(
        point for point in frame.points
        if not point.source.startswith("corner")
      )
      if motion_sensor == "front"
      else frame.points
    )
    outputs.append(controller.update(
      frame.time_s,
      frame.v_ego,
      radar_points,
      _controller_model(frame),
      frame.yaw_rate_rad_s,
      frame.input_age_s - frame.model_age_s,
    ))
    estimates.append(controller.trajectory_cutin.last_estimates)
  return tuple(outputs), tuple(estimates)


def _production_controller_outputs(
  frames: Sequence[RadarFrame],
  *,
  motion_sensor: str,
  enable_radar_tracks: int,
  cut_in_sensitivity: int,
) -> tuple[Any, ...]:
  return _production_controller_replay(
    frames,
    motion_sensor=motion_sensor,
    enable_radar_tracks=enable_radar_tracks,
    cut_in_sensitivity=cut_in_sensitivity,
  )[0]


class ProductionDPathSelector:
  """Replay only the production controller without rebuilding shadow models."""

  name = "production-dpath-controller"

  def __init__(
    self,
    frames: Sequence[RadarFrame],
    *,
    motion_sensor: str | None = None,
    enable_radar_tracks: int = 2,
    cut_in_sensitivity: int = VALIDATION_DEFAULT_SENSITIVITY,
  ) -> None:
    self.motion_sensor = motion_sensor or preferred_radar_motion_sensor(frames)
    self.enable_radar_tracks = int(enable_radar_tracks)
    self.cut_in_sensitivity = max(0, min(5, int(cut_in_sensitivity)))
    self.motion_sensitivity = radar_motion_sensitivity(
      self.cut_in_sensitivity,
      self.motion_sensor,
    )
    self.decision_threshold = self.motion_sensitivity.cut_in_threshold
    outputs, estimate_series = _production_controller_replay(
      frames,
      motion_sensor=self.motion_sensor,
      enable_radar_tracks=self.enable_radar_tracks,
      cut_in_sensitivity=self.cut_in_sensitivity,
    )
    selections = []
    for frame, output, estimates in zip(
      frames, outputs, estimate_series, strict=True,
    ):
      estimate_aliases: dict[int, tuple[int, ...]] = {}
      for estimate in estimates:
        if estimate.cross_sensor_track_id is None:
          continue
        point_id = estimate.point.track_id
        cross_id = estimate.cross_sensor_track_id
        estimate_aliases[point_id] = tuple({
          *estimate_aliases.get(point_id, ()), cross_id,
        })
        estimate_aliases[cross_id] = tuple({
          *estimate_aliases.get(cross_id, ()), point_id,
        })
      lead_one = _controller_candidate(
        frame, output.lead_one, "production dPath leadOne",
      )
      lead_two = _controller_candidate(
        frame,
        output.lead_two,
        "production dPath leadTwo",
        track_aliases=estimate_aliases.get(
          int((output.lead_two or {}).get("radarTrackId", -1)), (),
        ),
      )
      decisions = tuple(
        candidate
        for lead in output.leads_cutin
        if (
          candidate := _controller_candidate(
            frame,
            lead,
            "trajectory CUT-IN",
            track_aliases=estimate_aliases.get(
              int(lead.get("radarTrackId", -1)), (),
            ),
          )
        ) is not None
      )
      risk = _controller_candidate(
        frame,
        output.lead_cutin_risk,
        "trajectory CUT-IN pre-deceleration",
        track_aliases=estimate_aliases.get(
          int((output.lead_cutin_risk or {}).get("radarTrackId", -1)), (),
        ),
      )
      diagnostics = tuple(
        Candidate(
          track_id=estimate.point.track_id,
          score=estimate.confidence,
          reason=estimate.reason,
          d_rel=estimate.point.d_rel,
          y_rel=estimate.point.y_rel,
          v_lead=estimate.point.v_lead,
          current_path_occupancy=estimate.current_path,
          stage=(
            "CUT-IN" if estimate.confirmed_cutin
            else "RAW-CUTIN" if estimate.raw_cutin
            else "PREDECEL" if estimate.predecel_risk
            else "FILTERED" if not estimate.close_front_supported
            else "JITTER" if estimate.jittering
            else "TRACK"
          ),
          detail=(
            f"dRel={estimate.point.d_rel:.2f} "
            + f"vRel={estimate.point.v_rel:.2f} "
            + f"yRel={estimate.point.y_rel:.2f} "
            + f"dPath={estimate.d_path:.2f}->{estimate.future_d_path:.2f} "
            + "| "
            + f"h={estimate.history_s:.2f} "
            + f"p={estimate.inward_progress:.2f} "
            + f"dir={estimate.direction_consistency:.2f} "
            + f"yaw={estimate.recent_abs_yaw_max:.3f} "
            + f"vSp={estimate.recent_v_rel_spread:.2f} "
            + f"rp={estimate.recent_inward_progress:.2f} "
            + f"rd={estimate.recent_direction_consistency:.2f} "
            + "| "
            + f"J={int(estimate.jittering)} "
            + f"FJ={int(estimate.unstable_fast_motion)} "
            + f"close={int(estimate.close_front_supported)} "
            + f"hist={int(estimate.front_history_supported)} "
            + f"vision={int(estimate.vision_supported)} "
            + f"cross={int(estimate.cross_sensor_supported)} "
            + f"ctrl={int(estimate.control_eligible)} "
            + f"H={estimate.horizon_s:.2f} "
            + f"rate={estimate.inward_rate:.2f} "
            + f"radar={estimate.reported_inward_rate:.2f} "
            + f"move={estimate.lateral_travel:.2f} "
            + f"net={estimate.lateral_net_fraction:.2f} "
            + f"vMin={estimate.recent_v_rel_min:.2f} "
            + "crossId="
            + (
              "--" if estimate.cross_sensor_track_id is None
              else str(estimate.cross_sensor_track_id)
            )
            + " "
            + f"curve={int(estimate.curve_alias)} "
            + "eta="
            + (
              "--" if estimate.time_to_overlap_s is None
              else f"{estimate.time_to_overlap_s:.2f}s"
            )
          ),
          source=estimate.point.source,
          continuity_id=estimate.continuity_id,
        )
        for estimate in estimates
      )
      selections.append(Selection(
        lead_one=lead_one,
        lead_two=lead_two,
        front_candidates=tuple(
          candidate for candidate in diagnostics
          if not candidate.source.startswith("corner")
        ),
        corner_candidates=tuple(
          candidate for candidate in diagnostics
          if candidate.source.startswith("corner")
        ),
        cutin_diagnostics=diagnostics,
        decision_cutin_candidates=decisions,
        active_cutin_candidates=decisions,
        external_candidates=decisions,
        active_external_candidates=decisions,
        cutin_predecel_candidate=risk,
      ))
    self.motion_points = tuple(
      motion_points_at_model_time(frame, self.motion_sensor)
      for frame in frames
    )
    # Legacy trajectories use a different predictor and must not be drawn over
    # production decisions. Production state remains in the track diagnostics.
    self.trajectories: tuple[dict[Any, Any], ...] = tuple(
      {} for _ in frames
    )
    self.lead_one_outputs = tuple(output.lead_one for output in outputs)
    self.outputs = outputs
    self.selections = tuple(selections)

  def select(
    self,
    frame: RadarFrame,
    frame_index: int | None = None,
  ) -> Selection:
    del frame
    if frame_index is None:
      raise ValueError("production dPath selector requires a frame index")
    return self.selections[frame_index]


def _empty_recorded_lead() -> RecordedLead:
  return RecordedLead(False, False, -1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def aligned_video_time_s(qcamera_start_eof_ns: int, model_eof_ns: int) -> float | None:
  if qcamera_start_eof_ns <= 0 or model_eof_ns < qcamera_start_eof_ns:
    return None
  return (model_eof_ns - qcamera_start_eof_ns) / 1e9


def monotonic_log_events(events: Iterable[Any]) -> list[Any]:
  """Return rlog events in the timestamp order seen by live subscribers."""
  return sorted(events, key=lambda event: int(event.logMonoTime))


def predictor_reference_time_ns(event_ns: int, model_reference_ns: int) -> int:
  """Use the model exposure timestamp, matching production RadarD."""
  return model_reference_ns if model_reference_ns > 0 else event_ns


def _hyundai_scc_dbc_messages(
  route_replay: Any,
  car_fingerprint: str,
) -> dict[int, tuple[str, dict[str, Any]]]:
  """Load the route vehicle's SCC signal layout from its powertrain DBC."""
  # Standalone validation runs only put the openpilot root on sys.path. The
  # vendored opendbc package lives beside it, so make that import independent
  # of the caller's PYTHONPATH (the normal review launcher does not set one).
  opendbc_repo = REPO_ROOT / "opendbc_repo"
  if opendbc_repo.is_dir() and str(opendbc_repo) not in sys.path:
    sys.path.insert(0, str(opendbc_repo))
  try:
    from opendbc.car.hyundai.values import DBC
  except ImportError:
    return {}
  dbc_by_bus = DBC.get(car_fingerprint)
  if dbc_by_bus is None:
    dbc_by_bus = next(
      (
        value for fingerprint, value in DBC.items()
        if str(fingerprint) == car_fingerprint
      ),
      None,
    )
  if dbc_by_bus is None:
    return {}
  dbc_name = next(
    (
      value for bus, value in dbc_by_bus.items()
      if str(getattr(bus, "value", bus)) == "pt"
    ),
    None,
  )
  if not dbc_name:
    return {}
  dbc_path = route_replay.find_opendbc_file(
    REPO_ROOT,
    "opendbc",
    "dbc",
    f"{dbc_name}.dbc",
  )
  if dbc_path is None:
    return {}

  message_names = {"SCC_CONTROL", "SCC11", "SCC12"}
  signal_names = {
    "ACC_ObjDist", "ACC_ObjStatus", "ObjValid", "aReqRaw",
  }
  messages: dict[int, tuple[str, dict[str, Any]]] = {}
  current_address: int | None = None
  current_name = ""
  with open(dbc_path, encoding="utf-8") as dbc_file:
    for line in dbc_file:
      if line.startswith("BO_ "):
        parts = line.split()
        current_address = int(parts[1], 0) if len(parts) > 2 else None
        current_name = parts[2].rstrip(":") if len(parts) > 2 else ""
        if current_address is not None and current_name in message_names:
          messages[current_address] = (current_name, {})
        continue
      if current_address not in messages:
        continue
      match = route_replay.DBC_SIGNAL_RE.match(line)
      if match is None:
        continue
      name, start, length, endian, sign, factor, offset = match.groups()
      if name not in signal_names:
        continue
      messages[current_address][1][name] = route_replay.DbcSignalSpec(
        start=int(start),
        length=int(length),
        byte_order="le" if endian == "1" else "be",
        signed=sign == "-",
        factor=float(factor),
        offset=float(offset),
      )
  return messages


def _decode_scc_can_message(
  route_replay: Any,
  address: int,
  data: bytes,
  messages: dict[int, tuple[str, dict[str, Any]]],
) -> tuple[float | None, float | None, bool | None]:
  message = messages.get(address)
  if message is None:
    return None, None, None
  _, signals = message
  values: dict[str, float] = {}
  for name, signal in signals.items():
    raw_value = route_replay.dbc_unsigned(
      data, signal.start, signal.length, signal.byte_order,
    )
    if signal.signed:
      sign_bit = 1 << (signal.length - 1)
      raw_value = raw_value - (1 << signal.length) if raw_value & sign_bit else raw_value
    values[name] = raw_value * signal.factor + signal.offset
  object_valid = (
    values["ACC_ObjStatus"] > 0.0
    if "ACC_ObjStatus" in values
    else values["ObjValid"] > 0.0
    if "ObjValid" in values
    else None
  )
  return values.get("ACC_ObjDist"), values.get("aReqRaw"), object_valid


def _yaw_metadata(
  v_ego: float,
  steering_angle_deg: float,
  live_pose: Any | None,
  live_pose_age_s: float,
  steer_ratio: float,
  wheelbase: float,
) -> tuple[float, bool, str]:
  angular_velocity = getattr(live_pose, "angularVelocityDevice", None)
  if (
    angular_velocity is not None
    and bool(getattr(angular_velocity, "valid", False))
    and bool(getattr(live_pose, "inputsOK", False))
    and bool(getattr(live_pose, "sensorsOK", False))
    and live_pose_age_s <= 0.25
  ):
    value = _finite(getattr(angular_velocity, "z", math.nan), math.nan)
    if math.isfinite(value):
      return value, False, "livePose"
  ratio = max(abs(_finite(steer_ratio, 14.0)), 1.0)
  base = max(abs(_finite(wheelbase, 2.8)), 1.5)
  road_wheel_angle = math.radians(_finite(steering_angle_deg) / ratio)
  return -_finite(v_ego) * math.tan(road_wheel_angle) / base, True, "steering"


def load_frames(log_path: Path) -> list[RadarFrame]:
  route_replay = _route_replay_module()
  schema = route_replay.load_openpilot_log_schema()
  data = route_replay.read_log_bytes(log_path)
  events = monotonic_log_events(schema.Event.read_multiple_bytes(data))
  latest_points: tuple[RadarPoint, ...] | None = None
  latest_points_ns = 0
  latest_v_ego = 0.0
  latest_steering_angle_deg = 0.0
  latest_steering_rate_deg_s = 0.0
  latest_live_pose: Any | None = None
  latest_live_pose_ns = 0
  latest_steer_ratio = 14.0
  latest_wheelbase = 2.8
  latest_scc_distance_m: float | None = None
  latest_scc_distance_ns = 0
  latest_scc_a_req_raw: float | None = None
  latest_scc_a_req_raw_ns = 0
  latest_scc_object_valid: bool | None = None
  latest_scc_bus: int | None = None
  latest_scc_bus_ns = 0
  latest_carrot_a_target: float | None = None
  latest_carrot_a_target_ns = 0
  scc_dbc_messages: dict[int, tuple[str, dict[str, Any]]] = {}
  # carParams can be emitted well into a segment. Resolve the static vehicle
  # DBC before consuming CAN so SCC_CONTROL is decoded from the first frame.
  for event in events:
    try:
      if event.which() == "carParams":
        scc_dbc_messages = _hyundai_scc_dbc_messages(
          route_replay,
          str(event.carParams.carFingerprint),
        )
        break
    except Exception:
      continue
  latest_radar_delay_s = 0.0
  latest_path: tuple[tuple[float, float], ...] = ()
  latest_lanes: tuple[tuple[tuple[float, float], ...], ...] = ()
  latest_lane_probs: tuple[float, ...] = ()
  latest_model_leads: tuple[ModelLead, ...] = ()
  latest_path_y_stds: tuple[tuple[float, float], ...] = ()
  latest_lane_stds: tuple[float, ...] = ()
  latest_model_ns = 0
  latest_model_eof_ns = 0
  qcamera_start_eof_ns = 0
  absolute_frames: list[tuple[int, int, int, RadarFrame]] = []
  recorded_leads_by_model_ns: dict[
    int, tuple[RecordedLead, RecordedLead]
  ] = {}
  corner_tracker = route_replay.StableCornerObjectTracker()
  group2_front_quality: dict[int, tuple[int, int]] = {}

  for event in events:
    try:
      which = event.which()
    except Exception:
      continue
    event_ns = int(event.logMonoTime)
    if which == "qRoadEncodeIdx" and qcamera_start_eof_ns == 0:
      qcamera_start_eof_ns = int(event.qRoadEncodeIdx.timestampEof)
    elif which == "liveTracks":
      event_t = event_ns / 1e9
      reconstructed = corner_tracker.live_tracks_at(
        event_t,
        latest_v_ego,
        max_measurement_age_s=VALIDATION_CORNER_MAX_MEASUREMENT_AGE_S,
      )
      merged = route_replay.merge_recorded_and_reconstructed_tracks(
        tuple(event.liveTracks.points), reconstructed,
      )
      copied_points = _copy_track_points(merged)
      latest_points = tuple(
        _with_group2_front_track_state(
          point, event_ns, group2_front_quality,
        ) for point in copied_points
      )
      latest_points_ns = event_ns
    elif which == "can":
      event_t = event_ns / 1e9
      for can_message in event.can:
        address = int(can_message.address)
        raw = bytes(can_message.dat)
        scc_distance, scc_a_req_raw, scc_object_valid = (
          _decode_scc_can_message(
            route_replay,
            address,
            raw,
            scc_dbc_messages,
          )
          if int(can_message.src) < 0x80
          else (None, None, None)
        )
        if any(value is not None for value in (
          scc_distance, scc_a_req_raw, scc_object_valid,
        )):
          latest_scc_bus = int(can_message.src)
          latest_scc_bus_ns = event_ns
        if scc_object_valid is not None:
          latest_scc_object_valid = scc_object_valid
          if not scc_object_valid:
            latest_scc_distance_m = None
            latest_scc_distance_ns = event_ns
        if scc_distance is not None:
          latest_scc_distance_m = (
            float(scc_distance)
            if (
              0.0 < scc_distance < 150.0
              and latest_scc_object_valid is not False
            )
            else None
          )
          latest_scc_distance_ns = event_ns
        if scc_a_req_raw is not None:
          latest_scc_a_req_raw = float(scc_a_req_raw)
          latest_scc_a_req_raw_ns = event_ns
        if (
          VALIDATION_GROUP2_RADAR_START_ADDR
          <= address
          < VALIDATION_GROUP2_RADAR_START_ADDR
          + VALIDATION_GROUP2_RADAR_TRACK_COUNT
          and len(raw) == 24
        ):
          track_id = 32 + address - VALIDATION_GROUP2_RADAR_START_ADDR
          valid_state = route_replay.dbc_unsigned(raw, 25, 2, "be")
          group2_front_quality[track_id] = (
            event_ns,
            valid_state,
          )
        if int(can_message.src) != route_replay.RAW_CORNER_RADAR_BUS or int(can_message.src) >= 0x80:
          continue
        for obj in route_replay.decode_raw_corner_objects(
          event_t, address, raw,
        ):
          if route_replay.raw_corner_object_is_valid(obj):
            corner_tracker.update(obj)
    elif which == "carParams":
      car_fingerprint = str(event.carParams.carFingerprint)
      scc_dbc_messages = _hyundai_scc_dbc_messages(
        route_replay,
        car_fingerprint,
      )
      steer_ratio = _finite(event.carParams.steerRatio)
      wheelbase = _finite(event.carParams.wheelbase)
      latest_steer_ratio = steer_ratio if 5.0 <= steer_ratio <= 30.0 else 14.0
      latest_wheelbase = wheelbase if 1.8 <= wheelbase <= 4.5 else 2.8
      latest_radar_delay_s = max(
        0.0, _finite(event.carParams.radarDelay),
      )
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
    elif which == "longitudinalPlan":
      latest_carrot_a_target = _finite(
        event.longitudinalPlan.aTarget,
        math.nan,
      )
      if not math.isfinite(latest_carrot_a_target):
        latest_carrot_a_target = None
      latest_carrot_a_target_ns = event_ns
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
      if latest_points is None:
        continue
      live_pose_age_s = (
        max(0.0, (event_ns - latest_live_pose_ns) / 1e9)
        if latest_live_pose_ns else math.inf
      )
      yaw_rate, yaw_estimated, yaw_source = _yaw_metadata(
        latest_v_ego,
        latest_steering_angle_deg,
        latest_live_pose,
        live_pose_age_s,
        latest_steer_ratio,
        latest_wheelbase,
      )
      model_reference_ns = (
        latest_model_eof_ns
        if latest_model_eof_ns > 0
        else latest_model_ns
      )
      predictor_time_ns = predictor_reference_time_ns(
        event_ns, model_reference_ns,
      )
      absolute_frames.append((
        event_ns, predictor_time_ns, latest_model_ns, RadarFrame(
        mono_time_s=event_ns / 1e9,
        time_s=predictor_time_ns / 1e9,
        input_age_s=max(0.0, (event_ns - latest_points_ns) / 1e9),
        # modelV2 coordinates belong to the camera exposure time. Its event
        # logMonoTime is publication time after inference and is too recent.
        model_age_s=max(0.0, (event_ns - model_reference_ns) / 1e9) if model_reference_ns else math.inf,
        v_ego=latest_v_ego,
        points=latest_points,
        path=latest_path,
        lane_lines=latest_lanes,
        lane_probs=latest_lane_probs,
        model_leads=latest_model_leads,
        recorded_one=_empty_recorded_lead(),
        recorded_two=_empty_recorded_lead(),
        radar_delay_s=latest_radar_delay_s,
        video_time_s=aligned_video_time_s(qcamera_start_eof_ns, latest_model_eof_ns),
        path_y_stds=latest_path_y_stds,
        lane_stds=latest_lane_stds,
        steering_angle_deg=latest_steering_angle_deg,
        steering_rate_deg_s=latest_steering_rate_deg_s,
        yaw_rate_rad_s=yaw_rate,
        yaw_rate_estimated=yaw_estimated,
        yaw_rate_source=yaw_source,
        steer_ratio=latest_steer_ratio,
        wheelbase=latest_wheelbase,
        scc_bus=(
          latest_scc_bus
          if (
            latest_scc_bus_ns > 0
            and (event_ns - latest_scc_bus_ns) / 1e9
            <= SCC_SAMPLE_MAX_AGE_S
          )
          else None
        ),
        scc_distance_m=(
          latest_scc_distance_m
          if (
            latest_scc_distance_ns > 0
            and (event_ns - latest_scc_distance_ns) / 1e9
            <= SCC_SAMPLE_MAX_AGE_S
          )
          else None
        ),
        scc_a_req_raw=(
          latest_scc_a_req_raw
          if (
            latest_scc_a_req_raw_ns > 0
            and (event_ns - latest_scc_a_req_raw_ns) / 1e9
            <= SCC_SAMPLE_MAX_AGE_S
          )
          else None
        ),
        carrot_a_target=(
          latest_carrot_a_target
          if (
            latest_carrot_a_target_ns > 0
            and (event_ns - latest_carrot_a_target_ns) / 1e9
            <= LONGITUDINAL_PLAN_MAX_AGE_S
          )
          else None
        ),
      )))
    elif which == "radarState":
      radar_state = event.radarState
      model_ns = int(radar_state.mdMonoTime)
      if model_ns > 0:
        recorded_leads_by_model_ns[model_ns] = (
          _copy_recorded_lead(radar_state.leadOne),
          _copy_recorded_lead(radar_state.leadTwo),
        )

  if not absolute_frames:
    raise RuntimeError("no aligned liveTracks/modelV2 frames were found in this log")
  origin_ns = absolute_frames[0][1]
  frames = []
  for _, predictor_time_ns, model_ns, frame in absolute_frames:
    recorded = recorded_leads_by_model_ns.get(model_ns)
    if recorded is not None:
      frame = replace(
        frame,
        recorded_one=recorded[0],
        recorded_two=recorded[1],
      )
    frames.append(replace(
      frame, time_s=(predictor_time_ns - origin_ns) / 1e9,
    ))
  return frames


def front_only_frames(frames: list[RadarFrame]) -> tuple[list[RadarFrame], int]:
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
  """Align the current radard cut-in replay to normalized radar frames."""
  route_replay = _route_replay_module()
  aligned: list[set[int]] = [set() for _ in frames]
  for radar_source in radar_sources:
    route_frames = route_replay.RouteLogParser(
      reconstruct_corner_live_tracks=True,
      cutin_radar_source=radar_source,
    ).parse_file(log_path, route_replay.load_openpilot_log_schema())
    if not route_frames:
      continue
    route_times = [frame.t for frame in route_frames]
    for frame_index, radar_frame in enumerate(frames):
      index = bisect.bisect_left(route_times, radar_frame.mono_time_s)
      nearest_indices = [
        candidate for candidate in (index - 1, index)
        if 0 <= candidate < len(route_frames)
      ]
      nearest = min(
        nearest_indices,
        key=lambda candidate: abs(route_times[candidate] - radar_frame.mono_time_s),
      )
      available_ids = {point.track_id for point in radar_frame.points}
      aligned[frame_index].update(
        int(vehicle.radar_track_id)
        for vehicle in route_frames[nearest].detected_vehicles
        if vehicle.cut_in
        and vehicle.source.startswith("cutinReplay")
        and vehicle.radar_track_id is not None
        and int(vehicle.radar_track_id) in available_ids
      )
  return aligned


def recorded_track_id(lead: RecordedLead) -> int | None:
  return lead.track_id if lead.status and lead.radar and lead.track_id >= 0 else None


def resolved_recorded_track_id(frame: RadarFrame, lead: RecordedLead) -> int | None:
  track_id = recorded_track_id(lead)
  if track_id is None or any(point.track_id == track_id for point in frame.points):
    return track_id
  nearby = [
    point for point in frame.points
    if abs(point.d_rel - lead.d_rel) <= 2.0
    and abs(point.y_rel - lead.y_rel) <= 1.0
    and abs(point.v_rel - lead.v_rel) <= 3.0
  ]
  if not nearby:
    return track_id
  return min(
    nearby,
    key=lambda point: (
      abs(point.d_rel - lead.d_rel),
      abs(point.y_rel - lead.y_rel),
      abs(point.v_rel - lead.v_rel),
    ),
  ).track_id


def qcamera_path_for_log(log_path: Path) -> Path:
  return log_path.parent / "qcamera.ts"


def preferred_radar_points(
  frame: RadarFrame,
  review_source: str | None,
) -> tuple[RadarPoint, ...]:
  if review_source == "corner":
    return tuple(point for point in frame.points if point.source.startswith("corner"))
  if review_source == "front":
    return tuple(point for point in frame.points if point.source == "frontRadar")
  if review_source == "front+corner":
    return tuple(
      point for point in frame.points
      if point.source == "frontRadar" or point.source.startswith("corner")
    )
  return frame.points


def _validation_review_from_case(case: dict[str, Any]) -> ValidationReview:
  window = case["window"]
  return ValidationReview(
    case_id=str(case["id"]),
    expected=str(case["expected"]),
    source=str(case["source"]),
    start_s=float(window[0]),
    end_s=float(window[1]),
    scene=str(case["scene"]),
    validation_stage=str(case.get("validation_stage", "output")),
    target_track_ids=tuple(int(value) for value in case.get("target_track_ids", ())),
    forbidden_lead_one_ids=tuple(int(value) for value in case.get("forbidden_lead_one_ids", ())),
    forbidden_lead_two_ids=tuple(int(value) for value in case.get("forbidden_lead_two_ids", ())),
    human_verified=bool(case.get("human_verified", False)),
  )


def resolve_validation_cases(
  cases_path: Path,
  route_root: Path,
  queries: Sequence[str],
) -> tuple[Path, tuple[ValidationReview, ...]]:
  payload = json.loads(cases_path.read_text(encoding="utf-8"))
  cases = list(payload.get("cases", ()))
  matched: list[dict[str, Any]] = []
  for query in queries:
    exact = [case for case in cases if str(case["id"]).lower() == query.lower()]
    matches = exact or [
      case for case in cases if query.lower() in str(case["id"]).lower()
    ]
    if len(matches) != 1:
      names = ", ".join(str(case["id"]) for case in matches[:8]) or "none"
      raise SystemExit(f"validation case must match exactly one case; matched: {names}")
    if matches[0] not in matched:
      matched.append(matches[0])
  if not matched:
    raise SystemExit("at least one validation case is required")
  route_keys = {
    (str(case["vehicle_folder"]), str(case["log"]))
    for case in matched
  }
  if len(route_keys) != 1:
    raise SystemExit("repeated --validation-case entries must reference the same rlog")
  vehicle_folder, log = route_keys.pop()
  reviews = tuple(sorted(
    (_validation_review_from_case(case) for case in matched),
    key=lambda value: (value.start_s, value.end_s, value.case_id),
  ))
  return route_root / vehicle_folder / Path(log), reviews


def resolve_validation_case(
  cases_path: Path,
  route_root: Path,
  query: str,
) -> tuple[Path, ValidationReview]:
  route, reviews = resolve_validation_cases(cases_path, route_root, (query,))
  return route, reviews[0]


def update_validation_case_label(cases_path: Path, case_id: str, expected: str) -> None:
  expected = expected.lower()
  if expected not in VALIDATION_EXPECTED_LABELS:
    raise ValueError(f"invalid validation label: {expected}")
  payload = json.loads(cases_path.read_text(encoding="utf-8"))
  matches = [
    case for case in payload.get("cases", ())
    if str(case.get("id", "")).lower() == case_id.lower()
  ]
  if len(matches) != 1:
    raise ValueError(f"validation case must match exactly once: {case_id}")
  matches[0]["expected"] = expected
  matches[0]["human_verified"] = True
  cases_path.write_text(
    json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
    encoding="utf-8",
  )


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
  for index, item in enumerate(labels):
    item_key = (
      str(item.get("vehicle_folder", "")),
      str(item.get("log", "")),
      float(item.get("time_s", -1.0)),
      int(item.get("track_id", -1)),
    )
    if item_key == key:
      labels[index] = entry
      break
  else:
    labels.append(entry)
  labels.sort(key=lambda item: (
    str(item.get("vehicle_folder", "")),
    str(item.get("log", "")),
    float(item.get("time_s", 0.0)),
    int(item.get("track_id", -1)),
  ))
  payload["labels"] = labels
  labels_path.write_text(
    json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
    encoding="utf-8",
  )
  return label_id


def validation_review_events(
  frames: Sequence[RadarFrame],
  selector: LeadSelector,
  review: ValidationReview | None,
) -> dict[int, tuple[str, ...]]:
  events: dict[int, tuple[str, ...]] = {}
  previous: set[int] = set()
  for index, frame in enumerate(frames):
    selection = selector.select(frame, index)
    current = {
      candidate.track_id for candidate in selection.active_cutin_candidates
      if candidate_matches_targets(
        candidate,
        set(review.target_track_ids) if review is not None else set(),
      )
    }
    added = current - previous
    values = [f"CUT-IN id {track_id}" for track_id in sorted(added)]
    if review is not None and selection.lead_one is not None:
      if selection.lead_one.track_id in set(review.forbidden_lead_one_ids):
        values.append(f"FALSE leadOne id {selection.lead_one.track_id}")
    if review is not None and selection.lead_two is not None:
      if selection.lead_two.track_id in set(review.forbidden_lead_two_ids):
        values.append(f"FALSE leadTwo id {selection.lead_two.track_id}")
    if values:
      events[index] = tuple(values)
    previous = current
  return events


def trajectory_review_events(
  frames: Sequence[RadarFrame],
  trajectories: Sequence[dict[tuple[str, int], RadarMotionPrediction]],
  sources: Iterable[str],
  horizon_s: float = 1.0,
  probability_threshold: float = 0.60,
) -> dict[int, tuple[str, ...]]:
  requested = {value.lower() for value in sources}
  previous: set[tuple[str, int, int]] = set()
  events: dict[int, tuple[str, ...]] = {}
  for index, predictions in enumerate(trajectories):
    current: set[tuple[str, int, int]] = set()
    labels: list[str] = []
    for prediction in predictions.values():
      source_match = (
        ("corner" in requested and prediction.sensor == "corner")
        or ("front" in requested and prediction.sensor == "front")
        or ("front+corner" in requested)
      )
      sample = min(
        prediction.samples,
        key=lambda value: abs(value.horizon_s - horizon_s),
      )
      if (
        source_match
        and not prediction.current_path_occupancy
        and abs(sample.d_path) < abs(prediction.d_path)
        and sample.occupancy_prob >= probability_threshold
      ):
        key = (prediction.source, prediction.track_id, prediction.continuity_id)
        current.add(key)
        if key not in previous:
          labels.append(
            f"TRAJECTORY {prediction.sensor.upper()} id {prediction.track_id} "
            + f"p{sample.occupancy_prob:.2f} t{sample.horizon_s:.2f}s"
          )
    if labels:
      events[index] = tuple(labels)
    previous = current
  return events


def trajectory_model_review_events(
  frames: Sequence[RadarFrame],
  selector: LeadSelector,
  sources: Iterable[str],
  probability_threshold: float,
) -> dict[int, tuple[str, ...]]:
  requested = {value.lower() for value in sources}
  events: dict[int, tuple[str, ...]] = {}
  emitted: set[tuple[str, str, int, int]] = set()
  trajectories = getattr(selector, "trajectories", ())
  for index, frame in enumerate(frames):
    selection = selector.select(frame, index)
    candidates = selection.decision_cutin_candidates
    labels: list[str] = []
    predecel = selection.cutin_predecel_candidate
    if predecel is not None:
      sensor = "corner" if predecel.source.startswith("corner") else "front"
      prediction = (
        trajectories[index].get((predecel.source, predecel.track_id))
        if index < len(trajectories)
        else None
      )
      key = (
        "predecel",
        predecel.source,
        predecel.track_id,
        prediction.continuity_id if prediction is not None else -1,
      )
      if (
        (sensor in requested or "front+corner" in requested)
        and key not in emitted
      ):
        labels.append(
          f"예비감속 위험 {sensor} id {predecel.track_id} "
          + f"위험도 {predecel.score:.2f}"
        )
        emitted.add(key)
    for candidate in candidates:
      sensor = "corner" if candidate.source.startswith("corner") else "front"
      if (
        sensor not in requested
        and "front+corner" not in requested
      ):
        continue
      if candidate.score < probability_threshold:
        continue
      prediction = (
        trajectories[index].get((candidate.source, candidate.track_id))
        if index < len(trajectories)
        else None
      )
      key = (
        "cutin",
        candidate.source,
        candidate.track_id,
        prediction.continuity_id if prediction is not None else -1,
      )
      if key not in emitted:
        labels.append(
          f"물리 예측 CUT-IN {sensor} id {candidate.track_id} "
          + f"진입 {candidate.score:.2f} 이탈 {candidate.path_exit_score:.2f}"
        )
        emitted.add(key)
    if labels:
      events[index] = tuple(labels)
  return events


def comparison_summary(frames: Sequence[RadarFrame], selector: LeadSelector) -> dict[str, int]:
  summary = {
    "frames": len(frames),
    "lead_one_matches": 0,
    "lead_two_matches": 0,
    "recorded_one_radar": 0,
    "recorded_two_radar": 0,
    "shadow_cutins": 0,
    "radard_cutins": 0,
  }
  for index, frame in enumerate(frames):
    selection = selector.select(frame, index)
    recorded_one = resolved_recorded_track_id(frame, frame.recorded_one)
    recorded_two = resolved_recorded_track_id(frame, frame.recorded_two)
    summary["recorded_one_radar"] += int(recorded_one is not None)
    summary["recorded_two_radar"] += int(recorded_two is not None)
    summary["lead_one_matches"] += int(
      recorded_one is not None and candidate_track_id(selection.lead_one) == recorded_one
    )
    summary["lead_two_matches"] += int(
      recorded_two is not None and candidate_track_id(selection.lead_two) == recorded_two
    )
    summary["shadow_cutins"] += len(selection.decision_cutin_candidates)
    summary["radard_cutins"] += len(selection.active_cutin_candidates)
  return summary


def print_summary(
  log_path: Path,
  frames: Sequence[RadarFrame],
  selector: LeadSelector,
) -> None:
  candidate_frames = sum(
    bool(selector.select(frame, index).decision_cutin_candidates)
    for index, frame in enumerate(frames)
  )
  print(f"log: {log_path}")
  print(
    f"frames: {len(frames)} duration: {frames[-1].time_s:.2f}s "
    + f"selector: {selector.name}"
  )
  print(
    f"motion sensor: {getattr(selector, 'motion_sensor', 'unknown')} "
    + f"physical predictor CUT-IN frames: {candidate_frames}"
  )


class SimulatorUI:
  """Replay production dPath decisions with cached legacy comparisons."""

  def __init__(
    self,
    frames: list[RadarFrame],
    selector: RadarMotionShadowSelector | RadarOccupancyV2Selector,
    title: str,
    log_path: Path,
    reviews: tuple[ValidationReview, ...] = (),
    validation_cases_path: Path | None = None,
    display_threshold: float | None = None,
    display_lookahead_s: float | None = None,
    settings_path: Path | None = None,
    motion_mode: str = "normal",
    cut_in_sensitivity: int | None = None,
    sensor_probabilities: dict[str, float] | None = None,
    sensor_lookaheads: dict[str, float] | None = None,
    v3_selector: RadarOccupancyV3Selector | None = None,
    production_selector: ProductionDPathSelector | None = None,
  ) -> None:
    import pyray as rl
    self.rl = rl
    self.frames = frames
    self.production_selector = production_selector
    self.selector = production_selector or selector
    self.occupancy_version = (
      4
      if production_selector is not None
      else 2 if isinstance(selector, RadarOccupancyV2Selector) else 1
    )
    self.use_occupancy_v2 = self.occupancy_version == 2
    self.v1_selector = (
      selector.baseline
      if isinstance(selector, RadarOccupancyV2Selector)
      else selector
    )
    self.v2_selector = (
      selector if isinstance(selector, RadarOccupancyV2Selector) else None
    )
    self.v3_selector = v3_selector
    self.enable_radar_tracks = self.selector.enable_radar_tracks
    self.title = title
    self.log_path = log_path
    self.reviews = reviews
    self.validation_cases_path = validation_cases_path
    self.settings_path = settings_path or validation_settings_path()
    if motion_mode not in VALIDATION_MOTION_MODES:
      raise ValueError(f"unsupported radar motion mode: {motion_mode}")
    self.motion_mode = motion_mode
    self.cut_in_sensitivity = (
      self.selector.cut_in_sensitivity
      if cut_in_sensitivity is None
      else max(0, min(5, int(cut_in_sensitivity)))
    )
    self.pending_sensitivity = self.cut_in_sensitivity
    self.sensitivity_dragging = False
    self.sensor_probabilities = (
      {
        "corner": load_validation_probability(
          self.settings_path, sensor="corner",
        ),
        "front": load_validation_probability(
          self.settings_path, sensor="front",
        ),
      }
      if sensor_probabilities is None
      else {
        "corner": float(sensor_probabilities["corner"]),
        "front": float(sensor_probabilities["front"]),
      }
    )
    applied_threshold = (
      self.selector.decision_threshold
      if display_threshold is None
      else float(display_threshold)
    )
    self.sensor_probabilities[self.selector.motion_sensor] = applied_threshold
    self.display_threshold = applied_threshold
    self.pending_probability = applied_threshold
    self.probability_dragging = False
    del display_lookahead_s, sensor_lookaheads
    self.probability_cache: dict[
      tuple[str, int, float], RadarMotionShadowSelector
    ] = {
      (
        self.v1_selector.motion_sensor,
        self.v1_selector.cut_in_sensitivity,
        round(self.v1_selector.decision_threshold, 2),
      ): self.v1_selector,
    }
    self.sensor_history_cache = {
      self.v1_selector.motion_sensor: (
        self.v1_selector.motion_points,
        self.v1_selector.trajectories,
        self.v1_selector.lead_one_outputs,
      ),
    }
    self.index = 0
    self.paused = False
    self.speed = 1.0
    self.times = tuple(frame.time_s for frame in frames)
    self.playback_time = 0.0
    self.events = trajectory_model_review_events(
      frames,
      self.selector,
      ("front+corner",),
      applied_threshold,
    )
    self._refresh_lead_continuity()
    self.handled_events: set[int] = set()
    self.status = (
      f"dPath predictor 적용: {self.selector.motion_sensor} 레이더 · "
      + f"EnableRadarTracks {self.enable_radar_tracks}"
    )
    self.font: Any | None = None
    self._owns_font = False
    self.video_path = qcamera_path_for_log(log_path)
    self.video_reader: Any | None = None
    self.video_texture: Any | None = None
    self.video_texture_size: tuple[int, int] | None = None
    self.video_frame_id: str | None = None
    self.video_error = ""
    self.show_shadow_markers = True
    self.show_trajectory_history = True
    self.show_radar_observed_history = False
    self.show_front_radar = False
    if self.video_path.is_file():
      try:
        route_replay = _route_replay_module()
        video_end_s = max(
          frame.video_time_s if frame.video_time_s is not None else frame.time_s
          for frame in frames
        )
        self.video_reader = route_replay.RouteVideoFrameReader([
          route_replay.RouteVideoSegment(
            None,
            self.video_path,
            0.0,
            video_end_s,
          ),
        ])
      except Exception as exc:
        self.video_error = f"qcamera unavailable: {exc}"
    else:
      self.video_error = f"camera missing: {self.video_path.name}"

  def _activate_selector_pair(
    self,
    value: int,
    baseline: RadarMotionShadowSelector,
  ) -> None:
    self.v1_selector = baseline
    self.v2_selector = RadarOccupancyV2Selector(
      self.frames,
      baseline=baseline,
      enable_radar_tracks=self.enable_radar_tracks,
    )
    if self.v3_selector is not None or self.occupancy_version == 3:
      self.v3_selector = RadarOccupancyV3Selector(
        self.frames,
        baseline=self.v1_selector,
        v2_selector=self.v2_selector,
        enable_radar_tracks=self.enable_radar_tracks,
        cut_in_sensitivity=value,
      )
    if self.occupancy_version != 4:
      targets = {
        1: self.v1_selector,
        2: self.v2_selector,
        3: self.v3_selector,
      }
      target = targets[self.occupancy_version]
      assert target is not None
      self._activate_sensitivity(value, target)

  def _toggle_occupancy_version(self) -> None:
    version_cycle = (
      {4: 1, 1: 2, 2: 3, 3: 4}
      if self.production_selector is not None
      else {1: 2, 2: 3, 3: 1}
    )
    self.occupancy_version = version_cycle[self.occupancy_version]
    self.use_occupancy_v2 = self.occupancy_version == 2
    if self.occupancy_version == 2 and self.v2_selector is None:
      self.v2_selector = RadarOccupancyV2Selector(
        self.frames,
        baseline=self.v1_selector,
        enable_radar_tracks=self.enable_radar_tracks,
      )
    if self.occupancy_version == 3 and self.v3_selector is None:
      self.v3_selector = RadarOccupancyV3Selector(
        self.frames,
        baseline=self.v1_selector,
        v2_selector=self.v2_selector,
        enable_radar_tracks=self.enable_radar_tracks,
        cut_in_sensitivity=self.cut_in_sensitivity,
      )
    target = {
      1: self.v1_selector,
      2: self.v2_selector,
      3: self.v3_selector,
      4: self.production_selector,
    }[self.occupancy_version]
    assert target is not None
    self._activate_sensitivity(self.cut_in_sensitivity, target)
    version = {
      1: "이전 V1",
      2: "이전 V2 확률 점유",
      3: "이전 V3 단계 융합",
      4: "현재 Trajectory (production)",
    }[self.occupancy_version]
    self.status = f"검증 모델 {version}로 전환"

  def _refresh_lead_continuity(self) -> None:
    self.lead_one_segments = lead_continuity_segments(
      self.frames,
      self.selector.selections,
      "lead_one",
    )
    self.lead_two_segments = lead_continuity_segments(
      self.frames,
      self.selector.selections,
      "lead_two",
    )
    self.vision_lead_segments = vision_lead_continuity_segments(self.frames)
    self.lead_one_speed_segments = lead_speed_continuity_segments(
      self.frames,
      self.selector.selections,
    )
    self.scc_distance_segments = frame_value_continuity_segments(
      self.frames,
      "scc_distance_m",
    )
    self.scc_accel_segments = frame_value_continuity_segments(
      self.frames,
      "scc_a_req_raw",
    )
    self.carrot_accel_segments = frame_value_continuity_segments(
      self.frames,
      "carrot_a_target",
    )

  @staticmethod
  def _clamp_probability(probability: float) -> float:
    return round(min(max(
      float(probability),
      VALIDATION_PROBABILITY_MIN,
    ), VALIDATION_PROBABILITY_MAX), 2)

  def _activate_sensitivity(
    self,
    value: int,
    selector: (
      RadarMotionShadowSelector
      | RadarOccupancyV2Selector
      | RadarOccupancyV3Selector
      | ProductionDPathSelector
    ),
  ) -> None:
    self.selector = selector
    self.cut_in_sensitivity = value
    self.pending_sensitivity = value
    self.display_threshold = selector.decision_threshold
    self.sensor_probabilities[selector.motion_sensor] = (
      selector.decision_threshold
    )
    self.events = trajectory_model_review_events(
      self.frames,
      selector,
      ("front+corner",),
      selector.decision_threshold,
    )
    self.handled_events.clear()
    self._refresh_lead_continuity()
    lead_two_frames = sum(
      selection.lead_two is not None
      for selection in selector.selections
    )
    label = VALIDATION_SENSITIVITY_LABELS[value]
    confirmation_s = selector.motion_sensitivity.confirmation_s
    policy_text = (
      "CUT-IN 사용 안 함"
      if value == 0
      else f"확인 {confirmation_s:.2f}초"
    )
    self.status = (
      f"CUT-IN 감도 {value} {label} · {policy_text} "
      + f"적용 완료: 진입 {len(self.events)}회, "
      + f"L2 {lead_two_frames}프레임"
    )

  def _request_sensitivity(self, sensitivity: int) -> None:
    value = max(0, min(5, int(sensitivity)))
    sensor = self.selector.motion_sensor
    self.pending_sensitivity = value
    save_error: OSError | None = None
    try:
      save_validation_sensitivity(value, self.settings_path)
    except OSError as exc:
      save_error = exc

    policy = radar_motion_sensitivity(value, sensor)
    cache_key = (sensor, value, round(policy.cut_in_threshold, 2))
    cached = self.probability_cache.get(cache_key)
    if cached is None:
      cached_history = self.sensor_history_cache.get(sensor)
      cached = RadarMotionShadowSelector(
        self.frames,
        cut_in_sensitivity=value,
        motion_sensor=sensor,
        enable_radar_tracks=self.enable_radar_tracks,
        motion_points=(
          cached_history[0]
          if cached_history is not None
          else None
        ),
        lead_one_outputs=(
          cached_history[2]
          if cached_history is not None
          else None
        ),
        maximum_lookahead_s=VALIDATION_DEFAULT_LOOKAHEAD_S,
      )
      self.probability_cache[cache_key] = cached
      self.sensor_history_cache[sensor] = (
        cached.motion_points,
        cached.trajectories,
        cached.lead_one_outputs,
      )
      while len(self.probability_cache) > 12:
        self.probability_cache.pop(next(iter(self.probability_cache)))

    if (
      self.selector.cut_in_sensitivity != value
      or abs(
        self.selector.decision_threshold
        - policy.cut_in_threshold
      ) >= 0.005
    ):
      self._activate_selector_pair(value, cached)
      if self.production_selector is not None:
        self.production_selector = ProductionDPathSelector(
          self.frames,
          motion_sensor=sensor,
          enable_radar_tracks=self.enable_radar_tracks,
          cut_in_sensitivity=value,
        )
        if self.occupancy_version == 4:
          self._activate_sensitivity(value, self.production_selector)
    else:
      label = VALIDATION_SENSITIVITY_LABELS[value]
      self.status = f"CUT-IN 감도 {value} {label} 이미 적용됨"
    if save_error is not None:
      self.status += f" · 저장 실패: {save_error}"

  def _request_motion_mode(self, mode: str) -> None:
    if mode not in VALIDATION_MOTION_MODES:
      raise ValueError(f"unsupported radar motion mode: {mode}")
    target_sensor = (
      "front"
      if mode == "front"
      else preferred_radar_motion_sensor(self.frames)
    )
    policy = radar_motion_sensitivity(
      self.cut_in_sensitivity,
      target_sensor,
    )
    cache_key = (
      target_sensor,
      self.cut_in_sensitivity,
      round(policy.cut_in_threshold, 2),
    )
    cached = self.probability_cache.get(cache_key)
    if cached is None:
      cached = RadarMotionShadowSelector(
        self.frames,
        cut_in_sensitivity=self.cut_in_sensitivity,
        motion_sensor=target_sensor,
        enable_radar_tracks=self.enable_radar_tracks,
        maximum_lookahead_s=VALIDATION_DEFAULT_LOOKAHEAD_S,
      )
      self.probability_cache[cache_key] = cached
      self.sensor_history_cache[target_sensor] = (
        cached.motion_points,
        cached.trajectories,
        cached.lead_one_outputs,
      )
    self.motion_mode = mode
    save_error: OSError | None = None
    try:
      save_validation_motion_mode(mode, self.settings_path)
    except OSError as exc:
      save_error = exc
    self._activate_selector_pair(self.cut_in_sensitivity, cached)
    if self.production_selector is not None:
      self.production_selector = ProductionDPathSelector(
        self.frames,
        motion_sensor=target_sensor,
        enable_radar_tracks=self.enable_radar_tracks,
        cut_in_sensitivity=self.cut_in_sensitivity,
      )
      if self.occupancy_version == 4:
        self._activate_sensitivity(
          self.cut_in_sensitivity,
          self.production_selector,
        )
    mode_text = "일반(코너 우선)" if mode == "normal" else "프런트 전용"
    self.status = (
      f"{mode_text} 모드 적용 · {target_sensor} · production 포함 재계산"
    )
    if save_error is not None:
      self.status += f" · 저장 실패: {save_error}"

  @staticmethod
  def _color(
    values: tuple[int, int, int] | tuple[int, int, int, int],
  ) -> Any:
    import pyray as rl
    alpha = values[3] if len(values) == 4 else 255
    return rl.Color(values[0], values[1], values[2], alpha)

  @staticmethod
  def _font_candidates() -> tuple[Path, ...]:
    return (
      REPO_ROOT / "openpilot/selfdrive/assets/fonts/KaiGenGothicKR-Bold.ttf",
      REPO_ROOT / "openpilot/selfdrive/assets/fonts/Pretendard-Medium.ttf",
      Path(os.environ.get("WINDIR", "C:/Windows")) / "Fonts" / "malgun.ttf",
      REPO_ROOT / "openpilot/selfdrive/assets/fonts/JetBrainsMono-Medium.ttf",
    )

  def _load_font(self) -> None:
    rl = self.rl
    codepoints = (
      *range(0x20, 0x0250),
      *range(0x3131, 0x3190),
      *range(0xAC00, 0xD7A4),
    )
    glyph_buffer = rl.ffi.new("int[]", codepoints)
    glyphs = rl.ffi.cast("int *", glyph_buffer)
    for candidate in self._font_candidates():
      if not candidate.is_file():
        continue
      try:
        rl.set_trace_log_level(rl.TraceLogLevel.LOG_ERROR)
        font = rl.load_font_ex(
          str(candidate),
          KOREAN_FONT_BASE_SIZE,
          glyphs,
          len(codepoints),
        )
        rl.set_trace_log_level(rl.TraceLogLevel.LOG_WARNING)
        if font.texture.id > 0:
          rl.set_texture_filter(
            font.texture,
            rl.TextureFilter.TEXTURE_FILTER_BILINEAR,
          )
          self.font = font
          self._owns_font = True
          return
      except Exception:
        rl.set_trace_log_level(rl.TraceLogLevel.LOG_WARNING)
    self.font = rl.get_font_default()

  def _draw_text(
    self,
    text: str,
    x: int,
    y: int,
    size: int,
    color: Any,
  ) -> None:
    if self.font is None:
      self.rl.draw_text(text, x, y, size, color)
      return
    self.rl.draw_text_ex(
      self.font,
      text,
      self.rl.Vector2(float(x), float(y)),
      float(size),
      0.0,
      color,
    )

  def _screen(self, rect: Any, d_rel: float, y_rel: float) -> tuple[float, float]:
    lateral_scale = rect.width / 20.0
    plot_top = rect.y + DISPLAY_TOP_PADDING_PX
    plot_bottom = rect.y + rect.height - DISPLAY_BOTTOM_PADDING_PX
    forward_scale = (
      (plot_bottom - plot_top)
      / (DEFAULT_FORWARD_RANGE_M - DISPLAY_MIN_DREL_M)
    )
    return (
      rect.x + rect.width * 0.5 - y_rel * lateral_scale,
      plot_bottom - (d_rel - DISPLAY_MIN_DREL_M) * forward_scale,
    )

  def _draw_path(self, rect: Any, frame: RadarFrame) -> None:
    rl = self.rl
    distance_ticks = (
      int(DISPLAY_MIN_DREL_M),
      *range(0, int(DEFAULT_FORWARD_RANGE_M), 20),
      int(DEFAULT_FORWARD_RANGE_M),
    )
    for distance in distance_ticks:
      _, y = self._screen(rect, float(distance), 0.0)
      rl.draw_line(
        int(rect.x + 8.0),
        int(y),
        int(rect.x + rect.width - 8.0),
        int(y),
        self._color((45, 56, 67)),
      )
      self._draw_text(
        f"{distance}m",
        int(rect.x + 12.0),
        int(y - 14.0),
        11,
        self._color((120, 135, 148)),
      )
    for lane_index, lane in enumerate(frame.lane_lines):
      probability = frame.lane_probs[lane_index] if lane_index < len(frame.lane_probs) else 0.5
      lane_color = self._color((105, 115, 125)) if probability >= 0.3 else self._color((65, 72, 80))
      previous_lane = None
      for d_rel, y_rel in lane:
        if not 0.0 <= d_rel <= DEFAULT_FORWARD_RANGE_M:
          continue
        vector = rl.Vector2(*self._screen(rect, d_rel, -y_rel))
        if previous_lane is not None:
          rl.draw_line_ex(previous_lane, vector, 1.5, lane_color)
        previous_lane = vector
    if (
      len(frame.lane_lines) >= 4
      and len(frame.lane_probs) >= 4
      and frame.lane_probs[1] >= 0.3
      and frame.lane_probs[2] >= 0.3
    ):
      previous_center = None
      segment_index = 0
      for d_rel, _ in frame.path:
        if not 0.0 <= d_rel <= DEFAULT_FORWARD_RANGE_M:
          continue
        center_y = 0.5 * (
          model_line_y(frame.lane_lines[1], d_rel)
          + model_line_y(frame.lane_lines[2], d_rel)
        )
        vector = rl.Vector2(*self._screen(rect, d_rel, center_y))
        if previous_center is not None and segment_index % 2 == 0:
          rl.draw_line_ex(
            previous_center,
            vector,
            2.0,
            self._color((225, 231, 237, 180)),
          )
        previous_center = vector
        segment_index += 1
    previous = None
    for d_rel, _ in frame.path:
      if not 0.0 <= d_rel <= DEFAULT_FORWARD_RANGE_M:
        continue
      position = self._screen(rect, d_rel, model_line_y(frame.path, d_rel))
      vector = rl.Vector2(*position)
      if previous is not None:
        rl.draw_line_ex(previous, vector, 3.0, self._color((65, 110, 145)))
      previous = vector

  def _video_texture_for_time(self) -> Any | None:
    if self.video_reader is None:
      return None
    aligned_time = self.frames[self.index].video_time_s
    video_frame = self.video_reader.frame_at(
      aligned_time if aligned_time is not None else self.playback_time,
    )
    if video_frame is None:
      return None
    size = (video_frame.width, video_frame.height)
    if self.video_texture is None or self.video_texture_size != size:
      if self.video_texture is not None:
        self.rl.unload_texture(self.video_texture)
      image = self.rl.gen_image_color(
        video_frame.width,
        video_frame.height,
        self._color((0, 0, 0)),
      )
      self.video_texture = self.rl.load_texture_from_image(image)
      self.rl.unload_image(image)
      self.rl.set_texture_filter(
        self.video_texture,
        self.rl.TextureFilter.TEXTURE_FILTER_BILINEAR,
      )
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
    rl.draw_rectangle_rec(rect, self._color((7, 10, 14)))
    texture = self._video_texture_for_time()
    if texture is None:
      status = (
        self.video_reader.status_text()
        if self.video_reader is not None
        else self.video_error
      )
      self._draw_text(
        status[:80],
        int(rect.x + 18.0),
        int(rect.y + rect.height * 0.5),
        18,
        self._color((145, 158, 170)),
      )
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
    rl.draw_texture_pro(
      texture,
      source,
      destination,
      rl.Vector2(0.0, 0.0),
      0.0,
      self._color((255, 255, 255)),
    )
    self._draw_text("전방 카메라", int(rect.x + 10), int(rect.y + 9), 15, self._color((225, 231, 237)))

  def _draw_prediction_trajectory(
    self,
    rect: Any,
    frame: RadarFrame,
    point: RadarPoint,
    prediction: RadarMotionPrediction,
    highlighted: bool,
    cutin_confirmed: bool,
    control_eligible_current_path: bool,
  ) -> None:
    if not self.show_trajectory_history:
      return
    rl = self.rl
    history = prediction.history
    previous_observed = None
    previous_path_relative = None
    oldest_visible = None
    max_age_s = max((sample.age_s for sample in history), default=0.0)
    history_rgb = (
      (194, 112, 240)
      if point.source.startswith("corner")
      else (70, 190, 220)
    )
    for sample in history:
      age_ratio = 0.0 if max_age_s <= 1e-6 else sample.age_s / max_age_s
      alpha = int(55 + 150 * (1.0 - age_ratio))
      history_color = self._color((*history_rgb, alpha))
      if (
        self.show_radar_observed_history
        and DISPLAY_MIN_DREL_M
        <= sample.actual_x
        <= DEFAULT_FORWARD_RANGE_M
      ):
        observed_position = rl.Vector2(*self._screen(
          rect, sample.actual_x, sample.actual_y,
        ))
        if previous_observed is not None:
          rl.draw_line_ex(
            previous_observed,
            observed_position,
            1.0,
            self._color((120, 130, 140, min(alpha, 105))),
          )
        rl.draw_circle_lines(
          int(observed_position.x),
          int(observed_position.y),
          2.0,
          self._color((120, 130, 140, min(alpha, 120))),
        )
        previous_observed = observed_position
      if DISPLAY_MIN_DREL_M <= sample.path_x <= DEFAULT_FORWARD_RANGE_M:
        history_x, history_y = trajectory_history_display_position(
          frame,
          sample,
        )
        path_position = rl.Vector2(*self._screen(
          rect,
          history_x,
          history_y,
        ))
        if previous_path_relative is not None:
          rl.draw_line_ex(
            previous_path_relative,
            path_position,
            2.2 if highlighted else 1.3,
            history_color,
          )
        rl.draw_circle_v(
          path_position,
          2.8 if highlighted else 1.8,
          history_color,
        )
        if highlighted:
          rl.draw_circle_lines(
            int(path_position.x),
            int(path_position.y),
            3.5,
            self._color((225, 231, 237, min(alpha, 180))),
          )
        if oldest_visible is None:
          oldest_visible = (path_position, sample.age_s)
        previous_path_relative = path_position
    if highlighted and oldest_visible is not None and oldest_visible[1] >= 0.15:
      position, age_s = oldest_visible
      self._draw_text(
        f"-{age_s:.1f}s",
        int(position.x + 4.0),
        int(position.y - 12.0),
        10,
        self._color((*history_rgb, 180)),
      )

    previous = rl.Vector2(*self._screen(rect, point.d_rel, point.y_rel))
    for sample in prediction.samples:
      future_x, future_y = model_path_point_at_s(
        frame.path,
        sample.path_x,
        sample.d_path,
      )
      position = rl.Vector2(*self._screen(rect, future_x, future_y))
      predicted_cutin = (
        cutin_confirmed
        and confirmed_cutin_overlap_at(prediction, sample.horizon_s)
      )
      if predicted_cutin:
        future_color = (246, 142, 55)
      elif control_eligible_current_path:
        future_color = (62, 205, 130)
      else:
        future_color = (175, 188, 200)
      rl.draw_line_ex(
        previous,
        position,
        2.5 if highlighted else 1.5,
        self._color((*future_color, 230 if highlighted else 150)),
      )
      rl.draw_circle_lines(
        int(position.x),
        int(position.y),
        4.0 if highlighted else 2.5,
        self._color((*future_color, 230 if highlighted else 150)),
      )
      if highlighted:
        self._draw_text(
          f"+{sample.horizon_s:g}",
          int(position.x + 5.0),
          int(position.y - 11.0),
          10,
          self._color((*future_color, 220)),
        )
      previous = position

  def _draw_front_radar_overlay(self, rect: Any, frame: RadarFrame) -> None:
    if not self.show_front_radar:
      return
    rl = self.rl
    color = self._color((70, 190, 220, 190))
    for point in front_radar_display_points(frame):
      x, y = self._screen(rect, point.d_rel, point.y_rel)
      rl.draw_circle_lines(int(x), int(y), 5.0, color)
      self._draw_text(
        f"F{point.track_id}",
        int(x + 6.0),
        int(y + 3.0),
        10,
        color,
      )

  def _draw_lead_roles(self, rect: Any, selection: Selection) -> None:
    rl = self.rl
    roles = (
      (
        "L1",
        selection.lead_one,
        lead_one_rgb(
          selection.lead_one.track_id
          if selection.lead_one is not None
          else None
        ),
      ),
      ("L2", selection.lead_two, LEAD_TWO_RGB),
    )
    for label, candidate, rgb in roles:
      if (
        candidate is None
        or candidate.d_rel is None
        or candidate.y_rel is None
        or not DISPLAY_MIN_DREL_M
        <= candidate.d_rel
        <= DEFAULT_FORWARD_RANGE_M
      ):
        continue
      x, y = self._screen(rect, candidate.d_rel, candidate.y_rel)
      color = self._color(rgb)
      rl.draw_rectangle_lines_ex(
        rl.Rectangle(x - 11.0, y - 11.0, 22.0, 22.0),
        3.0,
        color,
      )
      self._draw_text(
        f"{label} {candidate.track_id} {candidate.d_rel:.1f}m",
        int(x + 14.0),
        int(y - 11.0),
        13,
        color,
      )

  def _draw_map(self, rect: Any, frame: RadarFrame, selection: Selection) -> None:
    rl = self.rl
    rl.draw_rectangle_rec(rect, self._color((13, 18, 24)))
    self._draw_path(rect, frame)
    ego_x, ego_y = self._screen(rect, 0.0, 0.0)
    rl.draw_circle(
      int(ego_x),
      int(ego_y),
      6.0,
      self._color((245, 247, 250)),
    )
    self._draw_text(
      "내 차",
      int(ego_x + 9.0),
      int(ego_y - 7.0),
      12,
      self._color((245, 247, 250)),
    )
    self._draw_front_radar_overlay(rect, frame)
    selected_keys = {
      (candidate.source, candidate.track_id)
      for candidate in (selection.lead_one, selection.lead_two)
      if candidate is not None
    }
    diagnostics = {
      (candidate.source, candidate.track_id): candidate
      for candidate in selection.cutin_diagnostics
    }
    confirmed_cutin_keys = {
      (candidate.source, candidate.track_id)
      for candidate in selection.decision_cutin_candidates
    }
    predecel_key = (
      (selection.cutin_predecel_candidate.source,
       selection.cutin_predecel_candidate.track_id)
      if selection.cutin_predecel_candidate is not None
      else None
    )
    display_points = {
      (point.source, point.track_id): point
      for point in self.selector.motion_points[self.index]
    }
    for point in corner_radar_display_points(frame):
      display_points.setdefault((point.source, point.track_id), point)
    for point in display_points.values():
      if not DISPLAY_MIN_DREL_M <= point.d_rel <= DEFAULT_FORWARD_RANGE_M:
        continue
      prediction = self.selector.trajectories[self.index].get(
        (point.source, point.track_id),
      )
      position_only = is_position_only_reference(
        frame, point, self.selector.motion_sensor,
      )
      if (
        prediction is None
        and (point.source, point.track_id) not in selected_keys
        and not position_only
        and not point.source.startswith("corner")
      ):
        continue
      x, y = self._screen(rect, point.d_rel, point.y_rel)
      diagnostic = diagnostics.get((point.source, point.track_id))
      control_eligible_current_path = (
        diagnostic is not None
        and diagnostic.current_path_occupancy
        and (point.source, point.track_id) in selected_keys
      )
      if (point.source, point.track_id) in confirmed_cutin_keys:
        color = (246, 142, 55)
        radius = 7.0
      elif (point.source, point.track_id) == predecel_key:
        color = (70, 190, 220)
        radius = 8.0
      elif control_eligible_current_path:
        color = (62, 205, 130)
        radius = 7.0
      elif diagnostic is not None and diagnostic.current_path_occupancy:
        color = (115, 125, 135)
        radius = 3.0
      elif point.source.startswith("corner"):
        color = (194, 112, 240)
        radius = 5.0
      elif position_only:
        color = (115, 125, 135)
        radius = 3.0
      else:
        color = (70, 190, 220)
        radius = 5.0
      highlighted = (
        (point.source, point.track_id) in selected_keys
        or control_eligible_current_path
        or (point.source, point.track_id) in confirmed_cutin_keys
      )
      if prediction is not None:
        self._draw_prediction_trajectory(
          rect,
          frame,
          point,
          prediction,
          highlighted,
          (point.source, point.track_id) in confirmed_cutin_keys,
          control_eligible_current_path,
        )
      rl.draw_circle(int(x), int(y), radius, self._color(color))
      suffix = " P" if position_only and prediction is None else ""
      self._draw_text(
        f"{point.track_id}{suffix}",
        int(x + 8),
        int(y - 8),
        13,
        self._color(color),
      )
    self._draw_lead_roles(rect, selection)
    self._draw_text(
      "-30~130m | 흰 점: 내 차 | 주황 □: radar L1 | 파랑 □: vision L1 | 노랑 □: L2",
      int(rect.x + 12.0),
      int(rect.y + 8.0),
      14,
      self._color((145, 158, 170)),
    )
    front_text = (
      "front radar 표시 중(F: 숨김)"
      if self.show_front_radar
      else "F: front radar 표시"
    )
    self._draw_text(
      f"회색: 차선 | 흰 점선: 차선 중심 | 파랑: model path | 보라: corner radar | {front_text}",
      int(rect.x + 12.0),
      int(rect.y + 27.0),
      14,
      self._color((145, 158, 170)),
    )
    trajectory_text = (
      "현재 production: 우측 진단의 dPath 현재→미래 / rate / direction 확인"
      if self.occupancy_version == 4
      else "이전 모델 과거: S,dPath 실선 | 미래: 회색 / 녹색 IN / 주황 CUT-IN"
    )
    self._draw_text(
      trajectory_text,
      int(rect.x + 12.0),
      int(rect.y + 46.0),
      14,
      self._color((145, 158, 170)),
    )

  def _probability_slider_rect(self, panel_rect: Any) -> Any:
    return self.rl.Rectangle(
      panel_rect.x + 18.0,
      panel_rect.y + 160.0,
      panel_rect.width - 36.0,
      8.0,
    )

  def _draw_probability_slider(self, panel_rect: Any) -> None:
    rl = self.rl
    slider = self._probability_slider_rect(panel_rect)
    ratio = self.pending_sensitivity / 5.0
    knob_x = slider.x + slider.width * ratio
    applying = self.selector.cut_in_sensitivity != self.pending_sensitivity
    applied_text = (
      f"현재 {self.selector.cut_in_sensitivity} · 놓으면 적용"
      if applying
      else "적용 완료"
    )
    sensor_text = (
      "코너"
      if self.selector.motion_sensor == "corner"
      else "프런트"
    )
    pending_policy = radar_motion_sensitivity(
      self.pending_sensitivity,
      self.selector.motion_sensor,
    )
    sensitivity_text = VALIDATION_SENSITIVITY_LABELS[
      self.pending_sensitivity
    ]
    confirmation_text = (
      "CUT-IN 사용 안 함"
      if self.pending_sensitivity == 0
      else f"확인 {pending_policy.confirmation_s:.2f}초"
    )
    self._draw_text(
      f"{sensor_text} CUT-IN 감도 {self.pending_sensitivity} "
      + f"{sensitivity_text} · {confirmation_text}"
      + " · production 재계산"
      + f"  {applied_text}",
      int(slider.x),
      int(slider.y - 27.0),
      14,
      self._color((225, 231, 237)),
    )
    rl.draw_rectangle_rec(slider, self._color((55, 65, 75)))
    rl.draw_rectangle(
      int(slider.x),
      int(slider.y),
      max(1, int(slider.width * ratio)),
      int(slider.height),
      self._color((72, 145, 255)),
    )
    rl.draw_circle_v(
      rl.Vector2(knob_x, slider.y + slider.height * 0.5),
      8.0,
      self._color((245, 247, 250)),
    )
    self._draw_text(
      "0 사용 안 함",
      int(slider.x),
      int(slider.y + 12.0),
      11,
      self._color((145, 158, 170)),
    )
    self._draw_text(
      "3 보통",
      int(slider.x + slider.width * 0.5 - 20.0),
      int(slider.y + 12.0),
      11,
      self._color((145, 158, 170)),
    )
    self._draw_text(
      "5 아주 민감",
      int(slider.x + slider.width - 72.0),
      int(slider.y + 12.0),
      11,
      self._color((145, 158, 170)),
    )

  def _draw_lead_continuity(self, rect: Any) -> None:
    rl = self.rl
    rl.draw_rectangle_rec(rect, self._color((13, 18, 24)))
    time_axis = self._continuity_time_axis_rect(rect)
    plot_left = time_axis.x
    plot_right = time_axis.x + time_axis.width
    plot_top = rect.y + 28.0
    plot_bottom = rect.y + rect.height * 0.64
    accel_top = plot_bottom + 30.0
    accel_bottom = rect.y + rect.height - 18.0
    plot_width = max(1.0, plot_right - plot_left)
    plot_height = max(1.0, plot_bottom - plot_top)
    for distance in (0.0, 50.0, 100.0, DEFAULT_FORWARD_RANGE_M):
      y = plot_bottom - distance / DEFAULT_FORWARD_RANGE_M * plot_height
      rl.draw_line(
        int(plot_left),
        int(y),
        int(plot_right),
        int(y),
        self._color((45, 56, 67)),
      )
      self._draw_text(
        f"{distance:.0f}m",
        int(rect.x + 4.0),
        int(y - 6.0),
        10,
        self._color((120, 135, 148)),
      )
    for speed_kph in (0.0, 70.0, LEAD_SPEED_GRAPH_MAX_KPH):
      y = plot_bottom - speed_kph / LEAD_SPEED_GRAPH_MAX_KPH * plot_height
      self._draw_text(
        f"{speed_kph:.0f}",
        int(plot_right - 25.0),
        int(y - 6.0),
        10,
        self._color(LEAD_ONE_SPEED_RGB),
      )

    total = max(self.times[-1], 1e-6)

    def time_x(time_s: float) -> float:
      return plot_left + min(max(time_s / total, 0.0), 1.0) * plot_width

    def distance_position(time_s: float, distance: float) -> Any:
      return rl.Vector2(
        time_x(time_s),
        plot_bottom
        - min(max(distance / DEFAULT_FORWARD_RANGE_M, 0.0), 1.0)
        * plot_height,
      )

    def speed_position(time_s: float, speed_kph: float) -> Any:
      return rl.Vector2(
        time_x(time_s),
        plot_bottom
        - min(max(speed_kph / LEAD_SPEED_GRAPH_MAX_KPH, 0.0), 1.0)
        * plot_height,
      )

    def accel_position(time_s: float, accel: float) -> Any:
      fraction = (
        (min(max(accel, ACCEL_GRAPH_MIN_MPS2), ACCEL_GRAPH_MAX_MPS2)
         - ACCEL_GRAPH_MIN_MPS2)
        / (ACCEL_GRAPH_MAX_MPS2 - ACCEL_GRAPH_MIN_MPS2)
      )
      return rl.Vector2(
        time_x(time_s),
        accel_bottom - fraction * max(1.0, accel_bottom - accel_top),
      )

    def draw_segments(
      segments: Sequence[Sequence[Sequence[float]]],
      position: Any,
      rgb: tuple[int, int, int],
      width: float = 1.7,
    ) -> None:
      color = self._color(rgb)
      for segment in segments:
        previous = None
        for sample in segment:
          current = position(float(sample[0]), float(sample[1]))
          if previous is not None:
            rl.draw_line_ex(previous, current, width, color)
          previous = current
        if len(segment) == 1 and previous is not None:
          rl.draw_circle_v(previous, 2.0, color)

    draw_segments(
      self.scc_distance_segments,
      distance_position,
      SCC_DISTANCE_RGB,
      1.8,
    )
    # Keep Carrot/model results above the stock SCC reference when lines
    # overlap, since those are the primary series being validated.
    series = (
      (self.lead_one_segments, True),
      (self.lead_two_segments, False),
      (self.vision_lead_segments, None),
    )
    for segments, is_lead_one in series:
      for segment in segments:
        rgb = (
          lead_one_rgb(segment[0][2])
          if is_lead_one
          else (
            LEAD_TWO_RGB
            if is_lead_one is False
            else vision_lead_rgb(segment[0][2])
          )
        )
        color = self._color(rgb)
        previous = None
        for time_s, distance, _ in segment:
          current = distance_position(time_s, distance)
          if previous is not None:
            rl.draw_line_ex(previous, current, 1.7, color)
          previous = current
        if len(segment) == 1:
          rl.draw_circle_v(previous, 2.0, color)
    draw_segments(
      self.lead_one_speed_segments,
      speed_position,
      LEAD_ONE_SPEED_RGB,
      1.4,
    )

    for accel in (-4.0, -2.0, 0.0, 2.0):
      y = accel_position(0.0, accel).y
      rl.draw_line(
        int(plot_left),
        int(y),
        int(plot_right),
        int(y),
        self._color((55, 66, 77) if accel != 0.0 else (95, 108, 120)),
      )
      self._draw_text(
        f"{accel:+.0f}",
        int(rect.x + 8.0),
        int(y - 6.0),
        10,
        self._color((120, 135, 148)),
      )
    draw_segments(
      self.scc_accel_segments,
      accel_position,
      SCC_ACCEL_RGB,
      1.8,
    )
    # Carrot is the primary series under review, so keep it visible when both
    # acceleration targets overlap.
    draw_segments(
      self.carrot_accel_segments,
      accel_position,
      CARROT_ACCEL_RGB,
      2.2,
    )

    current_selection = self.selector.select(
      self.frames[self.index],
      self.index,
    )
    frame = self.frames[self.index]
    vision = frame.model_leads[0] if frame.model_leads else None
    vision_value, vision_rgb, vision_distance = vision_lead_display_value(
      vision,
    )
    lead_values = (
      (
        "L1",
        current_selection.lead_one,
        lead_one_rgb(
          current_selection.lead_one.track_id
          if current_selection.lead_one is not None
          else None
        ),
      ),
      ("L2", current_selection.lead_two, LEAD_TWO_RGB),
    )
    legend_x = int(plot_left)
    for label, candidate, rgb in lead_values:
      value = (
        f"id{candidate.track_id} {candidate.d_rel:.1f}m"
        if candidate is not None and candidate.d_rel is not None
        else "--"
      )
      self._draw_text(
        f"{label} {value}",
        legend_x,
        int(rect.y + 7.0),
        12,
        self._color(rgb),
      )
      legend_x += 145
      if (
        candidate is not None
        and candidate.d_rel is not None
        and 0.0 <= candidate.d_rel <= DEFAULT_FORWARD_RANGE_M
      ):
        rl.draw_circle_v(
          distance_position(self.playback_time, candidate.d_rel),
          3.5,
          self._color(rgb),
        )
      if (
        label == "L1"
        and candidate is not None
        and candidate.v_lead is not None
      ):
        rl.draw_circle_v(
          speed_position(self.playback_time, candidate.v_lead * 3.6),
          3.0,
          self._color(LEAD_ONE_SPEED_RGB),
        )
      if label == "L1":
        speed_value = (
          f"{candidate.v_lead * 3.6:.1f}km/h"
          if candidate is not None and candidate.v_lead is not None
          else "--"
        )
        self._draw_text(
          f"L1 vLead {speed_value}",
          legend_x,
          int(rect.y + 7.0),
          12,
          self._color(LEAD_ONE_SPEED_RGB),
        )
        legend_x += 165
    scc_bus_text = (
      f"[B{frame.scc_bus}]" if frame.scc_bus is not None else ""
    )
    scc_value = (
      f"{frame.scc_distance_m:.1f}m"
      if frame.scc_distance_m is not None
      else "--"
    )
    self._draw_text(
      f"SCC{scc_bus_text} ObjDist {scc_value}",
      legend_x,
      int(rect.y + 7.0),
      12,
      self._color(SCC_DISTANCE_RGB),
    )
    if frame.scc_distance_m is not None:
      rl.draw_circle_v(
        distance_position(self.playback_time, frame.scc_distance_m),
        3.5,
        self._color(SCC_DISTANCE_RGB),
      )
    legend_x += 200
    self._draw_text(
      f"V {vision_value}",
      legend_x,
      int(rect.y + 7.0),
      12,
      self._color(vision_rgb),
    )
    if vision_distance is not None:
      rl.draw_circle_v(
        distance_position(self.playback_time, vision_distance),
        3.5,
        self._color(vision_rgb),
      )

    scc_accel_text = (
      f"{frame.scc_a_req_raw:+.2f}"
      if frame.scc_a_req_raw is not None
      else "--"
    )
    carrot_accel_text = (
      f"{frame.carrot_a_target:+.2f}"
      if frame.carrot_a_target is not None
      else "--"
    )
    self._draw_text(
      f"SCC{scc_bus_text} aReqRaw {scc_accel_text}",
      int(plot_left),
      int(accel_top - 21.0),
      12,
      self._color(SCC_ACCEL_RGB),
    )
    self._draw_text(
      f"Carrot aTarget {carrot_accel_text} m/s²",
      int(plot_left + 190.0),
      int(accel_top - 21.0),
      12,
      self._color(CARROT_ACCEL_RGB),
    )
    if frame.scc_a_req_raw is not None:
      rl.draw_circle_v(
        accel_position(self.playback_time, frame.scc_a_req_raw),
        3.0,
        self._color(SCC_ACCEL_RGB),
      )
    if frame.carrot_a_target is not None:
      rl.draw_circle_v(
        accel_position(self.playback_time, frame.carrot_a_target),
        3.0,
        self._color(CARROT_ACCEL_RGB),
      )

    cursor_x = time_x(self.playback_time)
    rl.draw_line(
      int(cursor_x),
      int(plot_top),
      int(cursor_x),
      int(accel_bottom),
      self._color((225, 231, 237, 150)),
    )

  def _draw_panel(self, rect: Any, frame: RadarFrame, selection: Selection) -> None:
    rl = self.rl
    white = self._color((225, 231, 237))
    muted = self._color((145, 158, 170))
    rl.draw_rectangle_rec(rect, self._color((22, 29, 37)))
    x = int(rect.x + 18.0)
    self._draw_text(self.title[:62], x, int(rect.y + 16.0), 19, white)
    self._draw_text(
      f"{frame.time_s:7.2f}초  프레임 {self.index + 1}/{len(self.frames)} "
      + f"{'일시정지' if self.paused else f'x{self.speed:g}'}",
      x,
      int(rect.y + 46.0),
      18,
      white,
    )
    sensor_text = "코너" if self.selector.motion_sensor == "corner" else "프런트"
    mode_text = (
      "일반(코너 우선)"
      if self.motion_mode == "normal"
      else "프런트 전용"
    )
    version_text = {
      1: "이전 V1",
      2: "이전 V2 확률 점유",
      3: "이전 V3 단계 융합",
      4: "현재 Trajectory (production)",
    }[self.occupancy_version]
    self._draw_text(
      f"dPath predictor: {version_text} · {mode_text} · "
      + f"{sensor_text} 레이더 사용 · "
      + f"EnableRadarTracks {self.enable_radar_tracks}",
      x,
      int(rect.y + 82.0),
      18,
      self._color((246, 142, 55)),
    )
    lead_one_id = (
      str(selection.lead_one.track_id)
      if selection.lead_one is not None
      else "--"
    )
    lead_two_id = (
      str(selection.lead_two.track_id)
      if selection.lead_two is not None
      else "--"
    )
    predecel_id = (
      str(selection.cutin_predecel_candidate.track_id)
      if selection.cutin_predecel_candidate is not None
      else "--"
    )
    self._draw_text(
      f"L1 {lead_one_id}  L2 {lead_two_id}  | 예비감속 {predecel_id}  | CUT-IN "
      + str([value.track_id for value in selection.decision_cutin_candidates]),
      x,
      int(rect.y + 110.0),
      16,
      white,
    )
    self._draw_probability_slider(rect)
    y = rect.y + 194.0
    max_rows = max(3, int((rect.height - 348.0) // 79.0))
    predecel = selection.cutin_predecel_candidate
    if predecel is not None:
      self._draw_text(
        f"{predecel.source[:9]:9} id{predecel.track_id:4d} "
        + f"위험도{predecel.score:.2f} 예비감속 활성 (CUT-IN 미확정)",
        x,
        int(y),
        15,
        self._color((70, 190, 220)),
      )
      detail_lines = predecel.detail.split(" | ", 2)
      self._draw_text(detail_lines[0][:59], x + 18, int(y + 20.0), 14, muted)
      if len(detail_lines) > 1:
        self._draw_text(detail_lines[1][:59], x + 18, int(y + 38.0), 14, muted)
      if len(detail_lines) > 2:
        self._draw_text(detail_lines[2][:59], x + 18, int(y + 56.0), 14, muted)
      y += 79.0
      max_rows -= 1
    for candidate in selection.cutin_diagnostics[:max_rows]:
      stage_text = {
        "CLEAR": "관찰 해제",
        "WATCH": "진입 관찰",
        "CAUTION": "진입 주의",
        "PREDECEL": "가속 금지/예비감속",
        "LIMIT": "가속 제한/예비감속",
        "LEAD": "L2 승격",
        "OCCUPIED": "현재 경로 점유",
        "IN": "현재 경로",
        "PENDING": "CUT-IN 확인 중",
        "CUT-IN": "CUT-IN 확정",
        "FILTERED": "제어 후보 제외",
        "SHADOW-CUTOUT": "CUT-OUT 예측",
        "MISMATCH": "위치/속도 불일치",
        "ROW-WAIT": "L1 동일 거리대·실제 진입 없음",
        "L1-FUTURE": "진입 예상 시점에 L1보다 앞이 아님",
        "BELOW": "기준 미달",
      }.get(candidate.stage, candidate.stage)
      self._draw_text(
        f"{candidate.source[:9]:9} id{candidate.track_id:4d} "
        + f"진입{candidate.score:.2f} 이탈{candidate.path_exit_score:.2f} {stage_text}",
        x,
        int(y),
        15,
        white,
      )
      detail_lines = candidate.detail.split(" | ", 2)
      self._draw_text(detail_lines[0][:59], x + 18, int(y + 20.0), 14, muted)
      if len(detail_lines) > 1:
        self._draw_text(detail_lines[1][:59], x + 18, int(y + 38.0), 14, muted)
      if len(detail_lines) > 2:
        self._draw_text(detail_lines[2][:59], x + 18, int(y + 56.0), 14, muted)
      y += 79.0
    if self.reviews:
      active_reviews = [
        review for review in self.reviews
        if review.start_s <= frame.time_s <= review.end_s
      ]
      review_text = " | ".join(
        f"{review.case_id}:{review.expected}" for review in active_reviews
      ) or "검증 지정 구간 밖"
      self._draw_text(
        review_text[:62],
        x,
        int(rect.y + rect.height - 115.0),
        14,
        self._color((245, 211, 72)),
      )
    self._draw_text(self.status[:65], x, int(rect.y + rect.height - 83.0), 15, white)
    self._draw_text(
      "Space 재생  클릭 탐색  R 처음  V 현재/이전 비교  T 처리모드  F 표시  H 궤적  A raw  M 마커",
      x,
      int(rect.y + rect.height - 51.0),
      13,
      muted,
    )
    self._draw_text(
      "I/C/S: 검증 라벨 변경",
      x,
      int(rect.y + rect.height - 29.0),
      13,
      self._color((247, 94, 94)),
    )

  @staticmethod
  def _panel_width(width: int) -> float:
    return min(500.0, max(410.0, width * 0.35))

  def _timeline_rect(self, width: int, height: int) -> Any:
    return self.rl.Rectangle(
      56.0,
      float(height - 42),
      max(100.0, float(width) - 80.0),
      13.0,
    )

  def _panel_rect(self, width: int, bottom: float) -> Any:
    panel_width = self._panel_width(width)
    return self.rl.Rectangle(
      float(width) - panel_width - 10.0,
      12.0,
      panel_width,
      max(300.0, bottom - 12.0),
    )

  def _continuity_time_axis_rect(self, rect: Any) -> Any:
    return self.rl.Rectangle(
      rect.x + 44.0,
      rect.y,
      max(1.0, rect.width - 56.0),
      rect.height,
    )

  def _layout_rects(
    self,
    width: int,
    height: int,
  ) -> tuple[Any, Any, Any, Any, Any]:
    timeline = self._timeline_rect(width, height)
    continuity_bottom = timeline.y - 47.0
    continuity_height = min(
      360.0,
      max(300.0, float(height) * 0.33),
    )
    continuity_rect = self.rl.Rectangle(
      12.0,
      continuity_bottom - continuity_height,
      max(120.0, float(width) - 24.0),
      continuity_height,
    )
    upper_bottom = continuity_rect.y - 8.0
    panel_rect = self._panel_rect(width, upper_bottom)
    content_width = float(width) - panel_rect.width - 34.0
    upper_height = max(400.0, upper_bottom - 12.0)
    video_height = max(
      250.0,
      min(upper_height - 210.0, upper_height * 0.50),
    )
    video_rect = self.rl.Rectangle(
      12.0,
      12.0,
      content_width,
      video_height,
    )
    map_y = video_rect.y + video_rect.height + 8.0
    map_rect = self.rl.Rectangle(
      12.0,
      map_y,
      content_width,
      max(150.0, upper_bottom - map_y),
    )
    return timeline, panel_rect, video_rect, map_rect, continuity_rect

  def _draw_timeline(self, rect: Any) -> None:
    rl = self.rl
    total = max(self.times[-1], 1e-6)
    rl.draw_rectangle_rec(rect, self._color((55, 65, 75)))
    for review in self.reviews:
      start_ratio = min(max(review.start_s / total, 0.0), 1.0)
      end_ratio = min(max(review.end_s / total, 0.0), 1.0)
      color = (
        (194, 112, 240)
        if review.expected == "detect"
        else (70, 190, 220)
        if review.expected == "clear"
        else (245, 211, 72)
      )
      rl.draw_rectangle(
        int(rect.x + rect.width * start_ratio),
        int(rect.y - 8.0),
        max(2, int(rect.width * (end_ratio - start_ratio))),
        4,
        self._color(color),
      )
    if self.show_shadow_markers:
      for frame_index in self.events:
        marker_x = rect.x + rect.width * self.times[frame_index] / total
        predecel_event = any(
          label.startswith("예비감속 위험")
          for label in self.events[frame_index]
        )
        rl.draw_line_ex(
          rl.Vector2(marker_x, rect.y - 15.0),
          rl.Vector2(marker_x, rect.y + rect.height + 7.0),
          3.0,
          self._color((70, 190, 220) if predecel_event else (246, 142, 55)),
        )
    progress = min(max(self.playback_time / total, 0.0), 1.0)
    rl.draw_rectangle(
      int(rect.x),
      int(rect.y),
      int(rect.width * progress),
      int(rect.height),
      self._color((72, 145, 255)),
    )
    knob_x = rect.x + rect.width * progress
    rl.draw_circle_v(
      rl.Vector2(knob_x, rect.y + rect.height * 0.5),
      8.0,
      self._color((225, 231, 237)),
    )
    self._draw_text(
      f"{self.playback_time:6.2f}s / {self.times[-1]:.2f}s",
      int(rect.x),
      int(rect.y - 33.0),
      15,
      self._color((225, 231, 237)),
    )
    shadow_text = (
      "하늘: 예비감속 위험 · 주황: CUT-IN 확정(M: 숨김)"
      if self.show_shadow_markers
      else "예비감속/CUT-IN 마커 숨김(M: 표시)"
    )
    self._draw_text(
      f"{shadow_text}   위쪽 막대: 검증 구간",
      int(rect.x + 190.0),
      int(rect.y - 33.0),
      14,
      self._color((145, 158, 170)),
    )

  def _draw_frame(self) -> tuple[Any, Any]:
    rl = self.rl
    width = rl.get_screen_width()
    height = rl.get_screen_height()
    (
      timeline,
      panel_rect,
      video_rect,
      map_rect,
      continuity_rect,
    ) = self._layout_rects(
      width,
      height,
    )
    frame = self.frames[self.index]
    selection = self.selector.select(frame, self.index)
    rl.clear_background(self._color((13, 18, 24)))
    self._draw_video(video_rect)
    self._draw_map(map_rect, frame, selection)
    self._draw_lead_continuity(continuity_rect)
    self._draw_panel(panel_rect, frame, selection)
    self._draw_timeline(timeline)
    return timeline, map_rect

  def seek(self, time_s: float) -> None:
    self.playback_time = min(max(float(time_s), 0.0), self.times[-1])
    index = bisect.bisect_left(self.times, self.playback_time)
    choices = tuple(
      candidate
      for candidate in (index - 1, index)
      if 0 <= candidate < len(self.frames)
    )
    self.index = min(
      choices,
      key=lambda candidate: abs(self.times[candidate] - self.playback_time),
    )

  def _seek_from_time_axis(
    self,
    axis: Any,
    mouse_x: float,
    source: str,
  ) -> None:
    ratio = min(max((mouse_x - axis.x) / axis.width, 0.0), 1.0)
    self.seek(ratio * self.times[-1])
    self._rearm_events_from_current()
    self.paused = True
    self.status = (
      f"{source} 탐색 @{self.playback_time:.2f}초; "
      + "이후 CUT-IN 자동정지 재설정됨"
    )

  def _rearm_events_from_current(self) -> None:
    self.handled_events = {
      frame_index
      for frame_index in getattr(self, "handled_events", ())
      if frame_index < self.index
    }

  def _pause_for_event(self, previous_index: int, current_index: int) -> bool:
    if current_index < previous_index:
      return False
    first = max(0, previous_index + 1)
    for frame_index in range(first, current_index + 1):
      if frame_index not in self.events or frame_index in self.handled_events:
        continue
      self.index = frame_index
      self.playback_time = self.times[frame_index]
      self.paused = True
      self.handled_events.add(frame_index)
      self.status = (
        f"자동 일시정지 @{self.playback_time:.2f}초: "
        + " | ".join(self.events[frame_index])
      )
      return True
    return False

  def _handle_input(
    self,
    timeline: Any,
    probability_slider: Any,
    continuity_axis: Any,
  ) -> None:
    rl = self.rl
    if rl.is_key_pressed(rl.KEY_SPACE):
      self.paused = not self.paused
      self.status = (
        "수동 일시정지"
        if self.paused
        else "재생 중: predictor CUT-IN 자동정지 대기"
      )
    shift = rl.is_key_down(rl.KEY_LEFT_SHIFT) or rl.is_key_down(rl.KEY_RIGHT_SHIFT)
    if rl.is_key_pressed(rl.KEY_RIGHT):
      self.seek(self.playback_time + (5.0 if shift else 0.5))
      self._rearm_events_from_current()
      self.paused = True
      self.status = f"수동 탐색 @{self.playback_time:.2f}초"
    if rl.is_key_pressed(rl.KEY_LEFT):
      self.seek(self.playback_time - (5.0 if shift else 0.5))
      self._rearm_events_from_current()
      self.paused = True
      self.status = f"수동 탐색 @{self.playback_time:.2f}초"
    if rl.is_key_pressed(rl.KEY_HOME):
      self.seek(0.0)
      self._rearm_events_from_current()
      self.paused = True
    if rl.is_key_pressed(rl.KEY_END):
      self.seek(self.times[-1])
      self._rearm_events_from_current()
      self.paused = True
    if rl.is_key_pressed(rl.KEY_UP):
      self.speed = min(8.0, self.speed * 2.0)
    if rl.is_key_pressed(rl.KEY_DOWN):
      self.speed = max(0.25, self.speed * 0.5)
    if rl.is_key_pressed(rl.KEY_R):
      self.handled_events.clear()
      self.seek(0.0)
      self.paused = False
      self.status = "처음부터 재생: 모든 predictor CUT-IN 자동정지 대기"
      self._pause_for_event(-1, self.index)
    if rl.is_key_pressed(rl.KEY_M):
      self.show_shadow_markers = not self.show_shadow_markers
      state = "표시" if self.show_shadow_markers else "숨김"
      self.status = f"dPath predictor 타임라인 마커 {state}"
    if rl.is_key_pressed(rl.KEY_H):
      self.show_trajectory_history = not self.show_trajectory_history
      state = "표시" if self.show_trajectory_history else "숨김"
      self.status = f"predictor S,dPath 과거·미래 궤적 {state}"
    if rl.is_key_pressed(rl.KEY_A):
      self.show_radar_observed_history = not self.show_radar_observed_history
      state = "표시" if self.show_radar_observed_history else "숨김"
      self.status = f"별도 raw radar 관측 궤적 {state}"
    if rl.is_key_pressed(rl.KEY_F):
      self.show_front_radar = not self.show_front_radar
      state = "표시" if self.show_front_radar else "숨김"
      self.status = f"front radar 현재 포인트 {state}"
    if rl.is_key_pressed(rl.KEY_T):
      mode = "front" if self.motion_mode == "normal" else "normal"
      self.status = "처리 센서 모드 변경 계산 중..."
      self._request_motion_mode(mode)
    if rl.is_key_pressed(rl.KEY_V):
      self._toggle_occupancy_version()
    if rl.is_key_pressed(rl.KEY_I):
      self._label("detect")
    if rl.is_key_pressed(rl.KEY_C):
      self._label("clear")
    if rl.is_key_pressed(rl.KEY_S):
      self._label("stationary")
    mouse = rl.get_mouse_position()
    slider_hit = rl.Rectangle(
      probability_slider.x,
      probability_slider.y - 10.0,
      probability_slider.width,
      probability_slider.height + 24.0,
    )
    slider_interaction = self.sensitivity_dragging
    if (
      rl.is_mouse_button_pressed(rl.MOUSE_BUTTON_LEFT)
      and rl.check_collision_point_rec(mouse, slider_hit)
    ):
      self.sensitivity_dragging = True
      slider_interaction = True
      self.paused = True
    if (
      self.sensitivity_dragging
      and rl.is_mouse_button_down(rl.MOUSE_BUTTON_LEFT)
    ):
      ratio = min(max(
        (mouse.x - probability_slider.x) / probability_slider.width,
        0.0,
      ), 1.0)
      self.pending_sensitivity = max(0, min(5, round(ratio * 5.0)))
    if (
      self.sensitivity_dragging
      and rl.is_mouse_button_released(rl.MOUSE_BUTTON_LEFT)
    ):
      self.sensitivity_dragging = False
      slider_interaction = True
      self._request_sensitivity(self.pending_sensitivity)
    timeline_clicked = (
      not slider_interaction
      and rl.is_mouse_button_pressed(rl.MOUSE_BUTTON_LEFT)
      and rl.check_collision_point_rec(mouse, timeline)
    )
    continuity_clicked = (
      not slider_interaction
      and rl.is_mouse_button_pressed(rl.MOUSE_BUTTON_LEFT)
      and rl.check_collision_point_rec(mouse, continuity_axis)
    )
    if timeline_clicked or continuity_clicked:
      axis = continuity_axis if continuity_clicked else timeline
      source = "L1/L2 그래프" if continuity_clicked else "seek bar"
      self._seek_from_time_axis(axis, mouse.x, source)

  def _label(self, expected: str) -> None:
    frame = self.frames[self.index]
    active_reviews = [
      review for review in self.reviews
      if review.start_s <= frame.time_s <= review.end_s
    ]
    if active_reviews and self.validation_cases_path is not None:
      for review in active_reviews:
        update_validation_case_label(
          self.validation_cases_path, review.case_id, expected,
        )
      self.status = f"검증 사례 {len(active_reviews)}개를 {expected}(으)로 변경"
      return
    measured = [point for point in frame.points if point.measured]
    if not measured:
      self.status = "라벨을 지정할 measured 포인트 없음"
      return
    point = min(measured, key=lambda value: (abs(value.y_rel), value.d_rel))
    label = "detect" if expected == "detect" else "clear"
    label_id = upsert_trajectory_review_label(
      DEFAULT_TRAJECTORY_LABELS,
      self.log_path,
      frame,
      point,
      label,
      1.0,
    )
    self.status = f"검증 전용 라벨 저장: {label_id}"

  def run(
    self,
    start_s: float = 0.0,
    paused: bool = False,
    screenshot: Path | None = None,
    exit_at_end: bool = False,
  ) -> None:
    rl = self.rl
    self.seek(start_s)
    self.paused = paused or screenshot is not None
    self._pause_for_event(self.index - 1, self.index)
    rl.set_config_flags(rl.FLAG_WINDOW_RESIZABLE | rl.FLAG_VSYNC_HINT)
    rl.init_window(1440, 1080, "carrotpilot radar video validation")
    self._load_font()
    rl.set_target_fps(60)
    local_screenshot: Path | None = None
    rendered_frames = 0
    try:
      while not rl.window_should_close():
        if not self.paused:
          previous_index = self.index
          next_time = min(
            self.times[-1],
            self.playback_time + rl.get_frame_time() * self.speed,
          )
          self.seek(next_time)
          paused_for_event = self._pause_for_event(previous_index, self.index)
          if (
            not paused_for_event
            and self.playback_time >= self.times[-1]
          ):
            if exit_at_end:
              break
            self.paused = True
            self.status = "로그 끝: Esc로 닫기 또는 R로 다시 재생"
        (
          timeline,
          panel,
          _,
          _,
          continuity,
        ) = self._layout_rects(
          rl.get_screen_width(),
          rl.get_screen_height(),
        )
        probability_slider = self._probability_slider_rect(panel)
        continuity_axis = self._continuity_time_axis_rect(continuity)
        if screenshot is None:
          self._handle_input(
            timeline,
            probability_slider,
            continuity_axis,
          )
        rl.begin_drawing()
        self._draw_frame()
        rl.end_drawing()
        rendered_frames += 1
        if screenshot is not None and rendered_frames >= 3:
          local_screenshot = Path.cwd() / screenshot.name
          rl.take_screenshot(screenshot.name)
          break
    finally:
      if self.video_reader is not None:
        self.video_reader.close()
      if self.video_texture is not None:
        rl.unload_texture(self.video_texture)
      if self._owns_font and self.font is not None:
        rl.unload_font(self.font)
      rl.close_window()
    if (
      screenshot is not None
      and local_screenshot is not None
      and local_screenshot.is_file()
      and local_screenshot.resolve() != screenshot.resolve()
    ):
      screenshot.parent.mkdir(parents=True, exist_ok=True)
      shutil.move(str(local_screenshot), str(screenshot))


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Replay only the physical dPath predictor",
  )
  parser.add_argument("rlog", nargs="?", type=Path)
  parser.add_argument("--start", type=float, default=0.0)
  parser.add_argument("--paused", action="store_true")
  parser.add_argument("--summary", action="store_true")
  parser.add_argument(
    "--motion-mode",
    choices=VALIDATION_MOTION_MODES,
    default=None,
    help=(
      "normal prefers corner radar when present; front ignores all corner "
      + "points (the saved UI mode is used when omitted)"
    ),
  )
  parser.add_argument(
    "--front-only",
    action="store_true",
    help="compatibility alias for --motion-mode front",
  )
  parser.add_argument(
    "--prob",
    type=float,
    default=None,
    help=(
      "legacy V1/V2/V3 comparison threshold override; production behavior "
      + "is controlled only by CUT-IN sensitivity"
    ),
  )
  parser.add_argument(
    "--sensitivity",
    type=int,
    default=None,
    help=(
      "Carrot Radar CUT-IN sensitivity 0..5; otherwise use the validation "
      + "UI's saved level (default 3)"
    ),
  )
  parser.add_argument(
    "--enable-radar-tracks",
    type=int,
    choices=range(-2, 4),
    default=2,
    help=(
      "EnableRadarTracks value used by the real dPath controller replay "
      + "(default: 2)"
    ),
  )
  parser.add_argument(
    "--legacy-v1",
    action="store_true",
    help="start legacy-only visual review with V1 (V cycles old V1/V2/V3)",
  )
  parser.add_argument("--validation-case", action="append", default=[])
  parser.add_argument(
    "--validation-root", type=Path,
    default=Path(r"\\DS1821P\openpilot\routes"),
  )
  parser.add_argument("--validation-cases", type=Path, default=DEFAULT_VALIDATION_CASES)
  parser.add_argument("--screenshot", type=Path)
  parser.add_argument(
    "--exit-at-end",
    action="store_true",
    help="close the current log at its end so a review queue can open the next log",
  )
  parser.add_argument(
    "--review-position",
    default="",
    help="queue position shown in the window title, for example 1/40",
  )
  parser.add_argument(
    "--cache-dir",
    type=Path,
    help="private replay cache directory used by the validation queue",
  )
  parser.add_argument(
    "--preload-only",
    action="store_true",
    help="build the replay cache and exit without opening a window",
  )
  parser.add_argument(
    "--consume-cache",
    action="store_true",
    help="remove a prepared replay cache after loading it",
  )
  return parser.parse_args()


def main() -> int:
  args = parse_args()
  reviews: tuple[ValidationReview, ...] = ()
  if args.validation_case:
    if args.rlog is not None:
      raise SystemExit("do not provide rlog together with --validation-case")
    args.rlog, reviews = resolve_validation_cases(
      args.validation_cases,
      args.validation_root,
      args.validation_case,
    )
  if args.rlog is None:
    raise SystemExit("rlog or --validation-case is required")
  if not args.rlog.is_file():
    raise SystemExit(f"log file does not exist: {args.rlog}")
  if args.prob is not None and not 0.0 <= args.prob <= 1.0:
    raise SystemExit("--prob must be between 0.00 and 1.00")
  if args.sensitivity is not None and not 0 <= args.sensitivity <= 5:
    raise SystemExit("--sensitivity must be between 0 and 5")
  if args.front_only and args.motion_mode not in (None, "front"):
    raise SystemExit("--front-only conflicts with --motion-mode normal")
  if args.preload_only and args.cache_dir is None:
    raise SystemExit("--preload-only requires --cache-dir")
  motion_mode = (
    "front"
    if args.front_only
    else args.motion_mode or load_validation_motion_mode()
  )
  cut_in_sensitivity = (
    load_validation_sensitivity()
    if args.sensitivity is None
    else int(args.sensitivity)
  )
  cache_path = (
    visual_replay_cache_path(
      args.cache_dir,
      args.rlog,
      motion_mode=motion_mode,
      cut_in_sensitivity=cut_in_sensitivity,
      probability_override=args.prob,
      enable_radar_tracks=args.enable_radar_tracks,
    )
    if args.cache_dir is not None
    else None
  )
  cached = (
    load_visual_replay_cache(cache_path)
    if cache_path is not None
    else None
  )
  if cached is not None:
    frames, v2_selector, v3_selector, production_selector = cached
    v1_selector = v2_selector.baseline
    motion_sensor = v1_selector.motion_sensor
    probability = v1_selector.decision_threshold
    print(f"Using prepared replay cache for {args.rlog}", flush=True)
    if args.consume_cache and cache_path is not None:
      cache_path.unlink(missing_ok=True)
  else:
    print(f"Loading {args.rlog} ...", flush=True)
    frames = load_frames(args.rlog)
    motion_sensor = (
      "front"
      if motion_mode == "front"
      else preferred_radar_motion_sensor(frames)
    )
    motion_policy = radar_motion_sensitivity(
      cut_in_sensitivity,
      motion_sensor,
    )
    probability = (
      motion_policy.cut_in_threshold
      if args.prob is None
      else float(args.prob)
    )
    print("Building physical dPath predictor history ...", flush=True)
    v1_selector = RadarMotionShadowSelector(
      frames,
      probability if args.prob is not None else None,
      cut_in_sensitivity=cut_in_sensitivity,
      motion_sensor=motion_sensor,
      enable_radar_tracks=args.enable_radar_tracks,
    )
    v2_selector = RadarOccupancyV2Selector(
      frames,
      baseline=v1_selector,
      enable_radar_tracks=args.enable_radar_tracks,
    )
    v3_selector = RadarOccupancyV3Selector(
      frames,
      baseline=v1_selector,
      v2_selector=v2_selector,
      enable_radar_tracks=args.enable_radar_tracks,
      cut_in_sensitivity=cut_in_sensitivity,
    )
    production_selector = ProductionDPathSelector(
      frames,
      motion_sensor=motion_sensor,
      enable_radar_tracks=args.enable_radar_tracks,
      cut_in_sensitivity=cut_in_sensitivity,
    )
    if args.preload_only and cache_path is not None:
      save_visual_replay_cache(
        cache_path,
        frames,
        v2_selector,
        v3_selector,
        production_selector,
      )
  motion_policy = radar_motion_sensitivity(
    cut_in_sensitivity,
    motion_sensor,
  )
  if motion_mode == "front":
    ignored = sum(
      point.measured and point.source.startswith("corner")
      for frame in frames
      for point in frame.points
    )
    print(
      f"Front-only replay: ignoring {ignored} measured corner-radar points.",
      flush=True,
    )
  if args.legacy_v1:
    print(
      f"Legacy validation {motion_sensor} CUT-IN: sensitivity "
      + f"{cut_in_sensitivity} "
      + f"({VALIDATION_SENSITIVITY_LABELS[cut_in_sensitivity]}), "
      + f"probability {probability:.2f}, confirmation "
      + f"{motion_policy.confirmation_s:.2f}s, future lookahead 5.0s, "
      + "continuous overlap "
      + f">={VALIDATION_MIN_CONTINUOUS_OVERLAP_S:.1f}s",
      flush=True,
    )
  else:
    print(
      f"Production Trajectory CUT-IN: sensitivity {cut_in_sensitivity} "
      + f"({VALIDATION_SENSITIVITY_LABELS[cut_in_sensitivity]}), "
      + f"{motion_sensor} preferred, full dPath controller replay",
      flush=True,
    )
  if args.preload_only:
    print(f"Prepared replay cache for {args.rlog}", flush=True)
    return 0
  selector = v1_selector if args.legacy_v1 else production_selector
  print_summary(args.rlog, frames, selector)
  if args.summary:
    return 0
  position = f"[{args.review_position}] " if args.review_position else ""
  display_name = f"{position}{args.rlog.parent.name}/{args.rlog.name}"
  ui = SimulatorUI(
    frames,
    v1_selector if args.legacy_v1 else v2_selector,
    display_name,
    args.rlog,
    reviews=reviews,
    validation_cases_path=args.validation_cases if reviews else None,
    display_threshold=selector.decision_threshold,
    motion_mode=motion_mode,
    cut_in_sensitivity=cut_in_sensitivity,
    v3_selector=v3_selector,
    production_selector=None if args.legacy_v1 else production_selector,
  )
  if args.legacy_v1:
    ui.v2_selector = v2_selector
  ui.run(args.start, args.paused, args.screenshot, args.exit_at_end)
  return 0


__all__ = (
  "Candidate",
  "CurrentRadardSelector",
  "CurrentRadardTeacher",
  "LeadSelector",
  "ModelLead",
  "ProductionDPathSelector",
  "RadarFrame",
  "RadarMotionShadowSelector",
  "RadarOccupancyV2Selector",
  "RadarOccupancyV3Selector",
  "RadarPoint",
  "RecordedLead",
  "Selection",
  "SHADOW_CUTIN_THRESHOLD",
  "VALIDATION_DEFAULT_CORNER_PROBABILITY",
  "VALIDATION_DEFAULT_FRONT_PROBABILITY",
  "VALIDATION_DEFAULT_SENSITIVITY",
  "VALIDATION_SENSITIVITY_LABELS",
  "SimulatorUI",
  "ValidationReview",
  "_copy_points",
  "_copy_track_points",
  "_route_replay_module",
  "aligned_video_time_s",
  "candidate_matches_targets",
  "candidate_track_id",
  "candidate_track_ids",
  "comparison_summary",
  "confirmed_cutin_overlap_at",
  "corner_radar_display_points",
  "current_cutin_track_ids",
  "front_only_frames",
  "front_radar_display_points",
  "frame_value_continuity_segments",
  "is_position_only_reference",
  "lead_continuity_segments",
  "lead_speed_continuity_segments",
  "lead_one_rgb",
  "load_validation_motion_mode",
  "load_validation_lookahead",
  "load_validation_probability",
  "load_validation_sensitivity",
  "load_frames",
  "load_visual_replay_cache",
  "main",
  "model_line_y",
  "motion_points_at_model_time",
  "monotonic_log_events",
  "preferred_radar_points",
  "radar_replay_source_fingerprint",
  "preferred_radar_motion_sensor",
  "predictor_reference_time_ns",
  "qcamera_path_for_log",
  "radar_points_at_model_time",
  "radar_trajectory_series",
  "resolve_validation_case",
  "resolve_validation_cases",
  "resolved_recorded_track_id",
  "save_validation_motion_mode",
  "save_validation_lookahead",
  "save_validation_probability",
  "save_validation_sensitivity",
  "save_visual_replay_cache",
  "prediction_with_validation_lookahead",
  "trajectory_history_display_y",
  "trajectory_history_display_position",
  "trajectory_model_review_events",
  "trajectory_review_events",
  "upsert_trajectory_review_label",
  "update_validation_case_label",
  "validation_review_events",
  "validation_settings_path",
  "visual_replay_cache_path",
  "vision_lead_continuity_segments",
  "vision_lead_display_value",
  "vision_lead_rgb",
)


if __name__ == "__main__":
  raise SystemExit(main())
