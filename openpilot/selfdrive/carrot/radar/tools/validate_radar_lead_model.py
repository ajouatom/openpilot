#!/usr/bin/env python3
"""Validate existing radard and report the physical dPath predictor in shadow."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import pickle
from pathlib import Path
import sys
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[5]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.selfdrive.carrot.radar.tools.radar_validation_replay import (
  CurrentRadardSelector,
  ProductionDPathSelector,
  candidate_matches_targets,
  current_cutin_track_ids,
  front_only_frames,
  load_frames,
)
from openpilot.selfdrive.carrot.radar_motion import (
  STATIONARY_MAX_ABS_VLEAD_MPS,
)


CARROT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CASES = CARROT_ROOT / "cluster" / "cutin_validation_cases.json"
DEFAULT_LABELS = CARROT_ROOT / "cluster" / "radar_trajectory_labels.json"
DEFAULT_CACHE_DIR = REPO_ROOT / ".tmp_radar_validation_cache"
FRAME_CACHE_VERSION = 1
DEADLINE_SAMPLE_TOLERANCE_S = 0.02


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Replay maintained validation logs through radard and physical dPath shadow",
  )
  parser.add_argument(
    "--root", type=Path,
    default=Path(r"\\DS1821P\openpilot\routes"),
  )
  parser.add_argument("--cases", type=Path, default=DEFAULT_CASES)
  parser.add_argument("--labels", type=Path, default=DEFAULT_LABELS)
  parser.add_argument("--case", action="append", default=[])
  parser.add_argument(
    "--trace-track",
    type=int,
    action="append",
    default=[],
    help="print compact trajectory diagnostics for a radar track ID",
  )
  parser.add_argument("--expected", choices=("detect", "clear", "stationary"))
  parser.add_argument("--front-only", action="store_true")
  parser.add_argument(
    "--enable-radar-tracks",
    type=int,
    choices=range(-2, 4),
    default=2,
    help="EnableRadarTracks value used by the dPath shadow (default: 2)",
  )
  parser.add_argument(
    "--shadow-only",
    action="store_true",
    help=(
      "skip the expensive existing-radard CUT-IN replay and validate only "
      + "the physical dPath implementation"
    ),
  )
  parser.add_argument(
    "--cases-only",
    action="store_true",
    help="skip radar_trajectory_labels.json; full validation uses both sources",
  )
  parser.add_argument("--report", type=Path)
  parser.add_argument(
    "--cache-dir",
    type=Path,
    default=DEFAULT_CACHE_DIR,
    help="parsed-frame cache directory (default: .tmp_radar_validation_cache)",
  )
  parser.add_argument(
    "--no-cache",
    action="store_true",
    help="read and decode every rlog instead of using the parsed-frame cache",
  )
  parser.add_argument(
    "--strict-radard",
    action="store_true",
    help="return nonzero when an existing-radard expectation fails",
  )
  parser.add_argument(
    "--strict-shadow",
    action="store_true",
    help="return nonzero when a physical dPath shadow expectation fails",
  )
  parser.add_argument(
    "--strict-predecel",
    action="store_true",
    help=(
      "return nonzero when a maintained clear window emits corner "
      + "pre-deceleration risk or a required pre-deceleration case is missed"
    ),
  )
  return parser.parse_args()


def _frame_cache_path(cache_dir: Path, log_path: Path) -> Path:
  stat = log_path.stat()
  identity = json.dumps({
    "version": FRAME_CACHE_VERSION,
    "path": str(log_path.resolve()),
    "size": stat.st_size,
    "mtime_ns": stat.st_mtime_ns,
  }, sort_keys=True, separators=(",", ":"))
  digest = hashlib.sha256(identity.encode("utf-8")).hexdigest()
  return cache_dir / f"frames-{digest}.pickle"


def _load_frames(
  log_path: Path,
  cache_dir: Path | None,
) -> tuple[list[Any], bool]:
  cache_path = (
    _frame_cache_path(cache_dir, log_path)
    if cache_dir is not None else None
  )
  if cache_path is not None and cache_path.is_file():
    try:
      with cache_path.open("rb") as cache_file:
        payload = pickle.load(cache_file)
      if (
        isinstance(payload, dict)
        and payload.get("version") == FRAME_CACHE_VERSION
        and isinstance(payload.get("frames"), list)
      ):
        return payload["frames"], True
    except (EOFError, OSError, pickle.PickleError):
      pass

  frames = load_frames(log_path)
  if cache_path is not None:
    cache_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = cache_path.with_name(
      f"{cache_path.name}.{os.getpid()}.tmp",
    )
    try:
      with temporary_path.open("wb") as cache_file:
        pickle.dump(
          {"version": FRAME_CACHE_VERSION, "frames": frames},
          cache_file,
          protocol=pickle.HIGHEST_PROTOCOL,
        )
      temporary_path.replace(cache_path)
    finally:
      temporary_path.unlink(missing_ok=True)
  return frames, False


def _entries(cases_path: Path, labels_path: Path, cases_only: bool) -> list[dict[str, Any]]:
  cases_payload = json.loads(cases_path.read_text(encoding="utf-8"))
  values = [
    {**case, "validation_set": "cases"}
    for case in cases_payload.get("cases", ())
  ]
  if not cases_only:
    labels_payload = json.loads(labels_path.read_text(encoding="utf-8"))
    values.extend({
      **label,
      "target_track_ids": [int(label["track_id"])],
      "scene": f"validation-only manual {label['expected']} label",
      "validation_set": "trajectory_labels",
    } for label in labels_payload.get("labels", ()))
  return values


def _source_matches(source: str, candidate_source: str) -> bool:
  if source == "corner":
    return candidate_source.startswith("corner")
  if source == "front":
    return not candidate_source.startswith("corner")
  return True


def _candidate_matches_entry(candidate: Any, entry: dict[str, Any]) -> bool:
  targets = {int(value) for value in entry.get("target_track_ids", ())}
  if candidate_matches_targets(candidate, targets):
    return True
  spatial = entry.get("target_spatial_match")
  if candidate is None or not isinstance(spatial, dict):
    return False
  d_rel_range = spatial.get("d_rel")
  y_rel_range = spatial.get("y_rel")
  if not (
    isinstance(d_rel_range, list | tuple)
    and len(d_rel_range) == 2
    and isinstance(y_rel_range, list | tuple)
    and len(y_rel_range) == 2
    and candidate.d_rel is not None
    and candidate.y_rel is not None
  ):
    return False
  return (
    float(d_rel_range[0]) <= float(candidate.d_rel) <= float(d_rel_range[1])
    and float(y_rel_range[0]) <= float(candidate.y_rel) <= float(y_rel_range[1])
  )


def _first_event(
  selector: Any,
  frames: list[Any],
  entry: dict[str, Any],
  attribute: str,
) -> tuple[float, int] | None:
  start_s, end_s = (float(value) for value in entry["window"])
  source = str(entry.get("source", "front+corner"))
  for index, frame in enumerate(frames):
    if not start_s <= frame.time_s <= end_s:
      continue
    value = getattr(selector.select(frame, index), attribute)
    candidates = (
      () if value is None
      else (value,) if attribute in (
        "lead_one", "lead_two", "cutin_predecel_candidate",
      )
      else tuple(value)
    )
    matches = [
      candidate for candidate in candidates
      if _candidate_matches_entry(candidate, entry)
      and _source_matches(source, candidate.source)
    ]
    if matches:
      best = min(
        matches,
        key=lambda candidate: candidate.d_rel if candidate.d_rel is not None else float("inf"),
      )
      return frame.time_s, best.track_id
  return None


def _lead_one_continuous(
  selector: Any,
  frames: list[Any],
  entry: dict[str, Any],
) -> bool | None:
  window = entry.get("lead_one_continuous_window")
  if window is None:
    return None
  start_s, end_s = (float(value) for value in window)
  samples = [
    selector.select(frame, index).lead_one is not None
    for index, frame in enumerate(frames)
    if start_s <= frame.time_s <= end_s
  ]
  return bool(samples) and all(samples)


def _lead_two_continuous(
  selector: Any,
  frames: list[Any],
  entry: dict[str, Any],
) -> bool | None:
  window = entry.get("lead_two_continuous_window")
  if window is None:
    return None
  start_s, end_s = (float(value) for value in window)
  samples = [
    _candidate_matches_entry(
      selector.select(frame, index).lead_two,
      entry,
    )
    for index, frame in enumerate(frames)
    if start_s <= frame.time_s <= end_s
  ]
  return bool(samples) and all(samples)


def _first_role_constraint_event(
  selector: Any,
  frames: list[Any],
  entry: dict[str, Any],
  role: str,
  track_ids: list[int] | tuple[int, ...],
) -> tuple[float, int] | None:
  if selector is None or not track_ids:
    return None
  start_s, end_s = (float(value) for value in entry["window"])
  targets = {int(value) for value in track_ids}
  for index, frame in enumerate(frames):
    if not start_s <= frame.time_s <= end_s:
      continue
    candidate = getattr(selector.select(frame, index), role)
    if candidate_matches_targets(candidate, targets):
      assert candidate is not None
      return frame.time_s, candidate.track_id
  return None


def _stationary_event(
  selector: Any,
  frames: list[Any],
  entry: dict[str, Any],
  *,
  require_target_ids: bool = True,
  maximum_abs_v_lead: float = 3.0 / 3.6,
) -> tuple[float, int] | None:
  start_s, end_s = (float(value) for value in entry["window"])
  targets = {int(value) for value in entry.get("target_track_ids", ())}
  for index, frame in enumerate(frames):
    if not start_s <= frame.time_s <= end_s:
      continue
    selection = selector.select(frame, index)
    for candidate in (selection.lead_one, selection.lead_two):
      if candidate is None:
        continue
      if (
        require_target_ids
        and not candidate_matches_targets(candidate, targets)
      ):
        continue
      point = next((
        value for value in frame.points
        if value.track_id in {candidate.track_id, *candidate.track_aliases}
        and abs(value.v_lead) <= maximum_abs_v_lead
        and 0.8 < value.d_rel < 130.0
      ), None)
      if point is not None:
        return frame.time_s, point.track_id
  return None


def _event_text(value: tuple[float, int] | None) -> str:
  return "--" if value is None else f"{value[0]:.2f}s/id{value[1]}"


def _print_track_trace(
  selector: Any,
  frames: list[Any],
  entries: list[dict[str, Any]],
  track_ids: set[int],
) -> None:
  if not track_ids:
    return
  trace_all = -1 in track_ids
  windows = tuple(
    (float(entry["window"][0]), float(entry["window"][1]))
    for entry in entries
  )
  last_print_s: dict[int, float] = {}
  last_stage: dict[int, str] = {}
  for index, frame in enumerate(frames):
    if not any(start <= frame.time_s <= end for start, end in windows):
      continue
    selection = selector.select(frame, index)
    diagnostic_ids = {
      candidate.track_id for candidate in selection.cutin_diagnostics
    }
    for candidate in selection.cutin_diagnostics:
      if not trace_all and candidate.track_id not in track_ids:
        continue
      stage_changed = last_stage.get(candidate.track_id) != candidate.stage
      periodic = frame.time_s - last_print_s.get(candidate.track_id, -math.inf) >= 0.25
      if not stage_changed and not periodic:
        continue
      print(
        f"    trace {frame.time_s:.2f}s id{candidate.track_id} "
        + f"{candidate.stage} yaw={frame.yaw_rate_rad_s:.3f} "
        + candidate.detail
        + " l1="
        + (
          "--" if selection.lead_one is None
          else f"{selection.lead_one.track_id}@{selection.lead_one.d_rel:.1f}"
        )
        + " l2="
        + (
          "--" if selection.lead_two is None
          else f"{selection.lead_two.track_id}@{selection.lead_two.d_rel:.1f}"
        ),
        flush=True,
      )
      last_stage[candidate.track_id] = candidate.stage
      last_print_s[candidate.track_id] = frame.time_s
    for point in frame.points:
      if (
        (not trace_all and point.track_id not in track_ids)
        or point.track_id in diagnostic_ids
      ):
        continue
      if frame.time_s - last_print_s.get(point.track_id, -math.inf) < 0.25:
        continue
      print(
        f"    trace {frame.time_s:.2f}s id{point.track_id} RAW "
        + f"source={point.source} dRel={point.d_rel:.2f} "
        + f"yRel={point.y_rel:.2f} vLead={point.v_lead:.2f}",
        flush=True,
      )
      last_stage[point.track_id] = "RAW"
      last_print_s[point.track_id] = frame.time_s


def _metrics(rows: list[dict[str, Any]], field: str) -> dict[str, float | int]:
  scorable = [
    row for row in rows
    if row["expected"] in ("detect", "clear")
    and (field != "shadow_event" or row.get("shadow_applicable", True))
  ]
  tp = sum(row["expected"] == "detect" and row[field] is not None for row in scorable)
  fp = sum(row["expected"] == "clear" and row[field] is not None for row in scorable)
  fn = sum(row["expected"] == "detect" and row[field] is None for row in scorable)
  tn = sum(row["expected"] == "clear" and row[field] is None for row in scorable)
  precision = tp / max(tp + fp, 1)
  recall = tp / max(tp + fn, 1)
  return {
    "labels": len(scorable),
    "tp": tp,
    "fp": fp,
    "fn": fn,
    "tn": tn,
    "precision": precision,
    "recall": recall,
    "f1": 2.0 * precision * recall / max(precision + recall, 1e-9),
    "accuracy": (tp + tn) / max(len(scorable), 1),
  }


def _print_metrics(title: str, values: dict[str, float | int]) -> None:
  print(
    f"{title}: {values['labels']} labels "
    + f"P={float(values['precision']):.3f} "
    + f"R={float(values['recall']):.3f} "
    + f"F1={float(values['f1']):.3f} "
    + f"Acc={float(values['accuracy']):.3f} "
    + f"TP/FP/FN/TN={values['tp']}/{values['fp']}/{values['fn']}/{values['tn']}"
  )


def main() -> int:
  args = parse_args()
  if args.shadow_only and args.strict_radard:
    raise SystemExit("--shadow-only conflicts with --strict-radard")
  filters = tuple(value.lower() for value in args.case)
  entries = [
    entry for entry in _entries(args.cases, args.labels, args.cases_only)
    if (args.expected is None or entry["expected"] == args.expected)
    and (not filters or any(value in str(entry["id"]).lower() for value in filters))
    and (
      not args.front_only
      or str(entry.get("source", "front+corner")) in ("front", "front+corner")
    )
  ]
  if not entries:
    raise SystemExit("no matching validation entries")

  grouped: dict[Path, list[dict[str, Any]]] = {}
  for entry in entries:
    path = args.root / str(entry["vehicle_folder"]) / Path(str(entry["log"]))
    grouped.setdefault(path, []).append(entry)

  rows: list[dict[str, Any]] = []
  missing = 0
  for log_index, (path, log_entries) in enumerate(grouped.items(), 1):
    if not path.is_file():
      missing += len(log_entries)
      for entry in log_entries:
        print(f"MISSING {entry['id']}: {path}", flush=True)
      continue
    cache_dir = None if args.no_cache else args.cache_dir
    frames, cache_hit = _load_frames(path, cache_dir)
    print(
      f"[{log_index:02d}/{len(grouped):02d}] "
      + f"{'cached' if cache_hit else 'loaded'} "
      + f"{path.parent.name}/{path.name} ({len(log_entries)} labels)",
      flush=True,
    )
    if args.front_only:
      frames, _ = front_only_frames(frames)
    sources = (
      ("corner", "front")
      if any(str(entry.get("source", "")) == "front+corner" for entry in log_entries)
      else tuple(sorted({
        "corner" if str(entry.get("source", "")).startswith("corner") else "front"
        for entry in log_entries
      }))
    )
    radard = None
    if not args.shadow_only:
      cutin_ids = current_cutin_track_ids(path, frames, sources)
      radard = CurrentRadardSelector(frames, cutin_ids)
    shadow = ProductionDPathSelector(
      frames,
      enable_radar_tracks=args.enable_radar_tracks,
    )
    _print_track_trace(
      shadow, frames, log_entries, set(args.trace_track),
    )
    for entry in log_entries:
      expected = str(entry["expected"])
      validation_stage = str(entry.get("validation_stage", "output"))
      required_lead_one_ids = entry.get("required_lead_one_ids", ())
      forbidden_lead_one_ids = entry.get("forbidden_lead_one_ids", ())
      forbidden_lead_two_ids = entry.get("forbidden_lead_two_ids", ())
      role_constraints_present = bool(
        required_lead_one_ids
        or forbidden_lead_one_ids
        or forbidden_lead_two_ids
      )
      if expected == "stationary":
        radard_event = (
          _stationary_event(radard, frames, entry)
          if radard is not None else None
        )
        shadow_event = _stationary_event(
          shadow,
          frames,
          entry,
          require_target_ids=False,
          maximum_abs_v_lead=STATIONARY_MAX_ABS_VLEAD_MPS,
        )
        radard_pass = radard is None or radard_event is not None
      else:
        validation_attribute = {
          "lead_one": "lead_one",
          "lead_two": "lead_two",
          "predecel": "cutin_predecel_candidate",
        }.get(validation_stage, "active_cutin_candidates")
        radard_event = (
          _first_event(radard, frames, entry, validation_attribute)
          if radard is not None else None
        )
        entry_source = str(entry.get("source", "front+corner"))
        shadow_applicable = (
          bool(entry.get("shadow_applicable", True))
          and (
            entry_source == "front+corner"
            or entry_source == shadow.motion_sensor
            or role_constraints_present
          )
        )
        shadow_event = (
          _first_event(
            shadow,
            frames,
            entry,
            (
              validation_attribute
              if validation_attribute in (
                "lead_one", "lead_two", "cutin_predecel_candidate",
              )
              else "decision_cutin_candidates"
            ),
          )
          if shadow_applicable
          else None
        )
        radard_pass = (
          radard is None
          or (radard_event is not None) == (expected == "detect")
        )
      if expected == "stationary":
        shadow_applicable = True
      shadow_pass = (
        not shadow_applicable
        or (shadow_event is not None) == (expected in ("detect", "stationary"))
      )
      radard_required_l1 = _first_role_constraint_event(
        radard, frames, entry, "lead_one", required_lead_one_ids,
      )
      shadow_required_l1 = _first_role_constraint_event(
        shadow, frames, entry, "lead_one", required_lead_one_ids,
      )
      radard_forbidden_l1 = _first_role_constraint_event(
        radard, frames, entry, "lead_one", forbidden_lead_one_ids,
      )
      shadow_forbidden_l1 = _first_role_constraint_event(
        shadow, frames, entry, "lead_one", forbidden_lead_one_ids,
      )
      radard_forbidden_l2 = _first_role_constraint_event(
        radard, frames, entry, "lead_two", forbidden_lead_two_ids,
      )
      shadow_forbidden_l2 = _first_role_constraint_event(
        shadow, frames, entry, "lead_two", forbidden_lead_two_ids,
      )
      if radard is not None:
        radard_pass = (
          radard_pass
          and (not required_lead_one_ids or radard_required_l1 is not None)
          and radard_forbidden_l1 is None
          and radard_forbidden_l2 is None
        )
      if shadow_applicable:
        shadow_pass = (
          shadow_pass
          and (not required_lead_one_ids or shadow_required_l1 is not None)
          and shadow_forbidden_l1 is None
          and shadow_forbidden_l2 is None
        )
      entry_source = str(entry.get("source", "front+corner"))
      predecel_applicable = (
        shadow_applicable
        and shadow.motion_sensor == "corner"
        and entry_source in ("corner", "front+corner")
        and expected in ("detect", "clear")
      )
      predecel_event = (
        _first_event(
          shadow,
          frames,
          entry,
          "cutin_predecel_candidate",
        )
        if predecel_applicable
        else None
      )
      predecel_required = validation_stage == "predecel"
      predecel_pass = (
        not predecel_applicable
        or (
          predecel_event is None
          if expected == "clear"
          else predecel_event is not None or not predecel_required
        )
      )
      if expected == "clear" or predecel_required:
        shadow_pass = shadow_pass and predecel_pass
      deadline = entry.get("latest_detection_s")
      if (
        radard_pass
        and deadline is not None
        and radard_event is not None
        and expected == "detect"
      ):
        radard_pass = (
          radard_event[0]
          <= float(deadline) + DEADLINE_SAMPLE_TOLERANCE_S
        )
      if (
        shadow_pass
        and deadline is not None
        and shadow_event is not None
        and expected == "detect"
      ):
        shadow_pass = (
          shadow_event[0]
          <= float(deadline) + DEADLINE_SAMPLE_TOLERANCE_S
        )
      if (
        predecel_pass
        and deadline is not None
        and predecel_event is not None
        and predecel_required
      ):
        predecel_pass = (
          predecel_event[0]
          <= float(deadline) + DEADLINE_SAMPLE_TOLERANCE_S
        )
        shadow_pass = shadow_pass and predecel_pass
      radard_continuous = (
        _lead_one_continuous(radard, frames, entry)
        if radard is not None else None
      )
      shadow_continuous = _lead_one_continuous(
        shadow, frames, entry,
      )
      radard_lead_two_continuous = (
        _lead_two_continuous(radard, frames, entry)
        if radard is not None else None
      )
      shadow_lead_two_continuous = _lead_two_continuous(
        shadow, frames, entry,
      )
      if radard_continuous is not None:
        radard_pass = radard_pass and radard_continuous
      if shadow_continuous is not None and shadow_applicable:
        shadow_pass = shadow_pass and shadow_continuous
      if radard_lead_two_continuous is not None:
        radard_pass = radard_pass and radard_lead_two_continuous
      if shadow_lead_two_continuous is not None and shadow_applicable:
        shadow_pass = shadow_pass and shadow_lead_two_continuous
      row = {
        "id": str(entry["id"]),
        "validation_set": str(entry["validation_set"]),
        "source": str(entry.get("source", "")),
        "expected": expected,
        "radard_event": radard_event,
        "shadow_event": shadow_event,
        "predecel_event": predecel_event,
        "shadow_applicable": shadow_applicable,
        "shadow_sensor": shadow.motion_sensor,
        "radard_pass": radard_pass,
        "shadow_pass": shadow_pass,
        "predecel_pass": predecel_pass,
        "radard_continuous": radard_continuous,
        "shadow_continuous": shadow_continuous,
        "radard_lead_two_continuous": radard_lead_two_continuous,
        "shadow_lead_two_continuous": shadow_lead_two_continuous,
        "radard_required_lead_one": radard_required_l1,
        "shadow_required_lead_one": shadow_required_l1,
        "radard_forbidden_lead_one": radard_forbidden_l1,
        "shadow_forbidden_lead_one": shadow_forbidden_l1,
        "radard_forbidden_lead_two": radard_forbidden_l2,
        "shadow_forbidden_lead_two": shadow_forbidden_l2,
      }
      rows.append(row)
      print(
        f"  {entry['id']} expected={expected} "
        + (
          f"radard={'PASS' if radard_pass else 'FAIL'}:{_event_text(radard_event)} "
          if radard is not None else "radard=SKIP "
        )
        + (
          f"shadow={'PASS' if shadow_pass else 'FAIL'}:{_event_text(shadow_event)}"
          if shadow_applicable
          else f"shadow=N/A({shadow.motion_sensor})"
        )
        + (
          " continuity="
          + f"radard:{'PASS' if radard_continuous else 'FAIL'}"
          + f"/shadow:{'PASS' if shadow_continuous else 'FAIL'}"
          if radard_continuous is not None
          and shadow_continuous is not None
          else ""
        )
        + (
          " lead2-continuity="
          + f"radard:{'PASS' if radard_lead_two_continuous else 'FAIL'}"
          + f"/shadow:{'PASS' if shadow_lead_two_continuous else 'FAIL'}"
          if radard_lead_two_continuous is not None
          and shadow_lead_two_continuous is not None
          else (
            " lead2-continuity="
            + f"shadow:{'PASS' if shadow_lead_two_continuous else 'FAIL'}"
            if shadow_lead_two_continuous is not None else ""
          )
        )
        + (
          f" predecel={'PASS' if predecel_pass else 'FAIL'}:"
          + _event_text(predecel_event)
          if predecel_applicable
          and (expected == "clear" or predecel_required)
          else ""
        )
        + (
          " role-check="
          + f"requiredL1:{_event_text(shadow_required_l1)} "
          + f"forbiddenL1:{_event_text(shadow_forbidden_l1)} "
          + f"forbiddenL2:{_event_text(shadow_forbidden_l2)}"
          if role_constraints_present
          else ""
        ),
        flush=True,
      )

  print()
  for validation_set in ("cases", "trajectory_labels", "all"):
    subset = rows if validation_set == "all" else [
      row for row in rows if row["validation_set"] == validation_set
    ]
    if not subset:
      continue
    print(f"[{validation_set}]")
    if not args.shadow_only:
      _print_metrics("existing radard", _metrics(subset, "radard_event"))
    _print_metrics("physical dPath shadow", _metrics(subset, "shadow_event"))
    predecel_subset = [
      row for row in subset
      if row["expected"] == "clear" or row["predecel_event"] is not None
    ]
    if predecel_subset:
      _print_metrics(
        "corner pre-deceleration safeguards",
        _metrics(predecel_subset, "predecel_event"),
      )
  radard_failures = (
    0
    if args.shadow_only
    else sum(not row["radard_pass"] for row in rows)
  )
  shadow_failures = sum(not row["shadow_pass"] for row in rows)
  predecel_failures = sum(not row["predecel_pass"] for row in rows)
  print(
    f"\nprocessed={len(rows)} missing={missing} "
    + (
      "existing-radard=SKIP "
      if args.shadow_only
      else f"existing-radard expectation failures={radard_failures} "
    )
    + f"physical-dpath expectation failures={shadow_failures}"
    + f" pre-deceleration expectation failures={predecel_failures}"
  )

  if args.report is not None:
    payload = {
      "version": 1,
      "description": "existing radard control versus physical dPath shadow; manual labels validation-only",
      "rows": rows,
      "summary": {
        "existing_radard": (
          None
          if args.shadow_only
          else _metrics(rows, "radard_event")
        ),
        "physical_dpath_shadow": _metrics(rows, "shadow_event"),
        "missing": missing,
        "radard_expectation_failures": radard_failures,
        "physical_dpath_expectation_failures": shadow_failures,
        "predeceleration_expectation_failures": predecel_failures,
      },
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(
      json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
      encoding="utf-8",
    )
    print(f"report written: {args.report}")
  return int(
    missing > 0
    or (args.strict_radard and radard_failures > 0)
    or (args.strict_shadow and shadow_failures > 0)
    or (args.strict_predecel and predecel_failures > 0)
  )


if __name__ == "__main__":
  raise SystemExit(main())
