#!/usr/bin/env python3
"""Export and compare full-log cut-in events across radar model revisions."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import importlib
import json
import math
from pathlib import Path
import subprocess
import sys
from typing import Any


SCRIPT_REPO_ROOT = Path(__file__).resolve().parents[5]
DEFAULT_CASES = SCRIPT_REPO_ROOT / "openpilot/selfdrive/carrot/cluster/cutin_validation_cases.json"
DEFAULT_ROUTE_ROOT = Path(r"W:\routes")
EPISODE_GAP_S = 0.25


@dataclass
class CutinEpisode:
  log: str
  start_s: float
  end_s: float
  track_ids: list[int]
  first_d_rel: float | None
  min_d_rel: float | None
  max_score: float
  reasons: list[str]
  review: str = "unreviewed"
  review_cases: list[str] | None = None


def _load_simulator(checkout_root: Path) -> Any:
  checkout = str(checkout_root.resolve())
  sys.path.insert(0, checkout)
  module_name = "openpilot.selfdrive.carrot.radar.tools.radar_lead_simulator"
  return importlib.import_module(module_name)


def _candidate_ids(candidate: Any) -> set[int]:
  if candidate is None:
    return set()
  return {
    int(value)
    for value in (candidate.track_id, *getattr(candidate, "track_aliases", ()))
  }


def _active_cutins(selection: Any) -> list[Any]:
  active = list(getattr(selection, "active_cutin_candidates", ()))
  lead_two = getattr(selection, "lead_two", None)
  if lead_two is not None and str(getattr(lead_two, "reason", "")).endswith("active cutin"):
    active.append(lead_two)

  deduplicated: list[Any] = []
  seen: list[set[int]] = []
  for candidate in active:
    ids = _candidate_ids(candidate)
    if not ids or any(ids & existing for existing in seen):
      continue
    seen.append(ids)
    deduplicated.append(candidate)
  return deduplicated


def _decision_cutins(selection: Any) -> list[Any]:
  return list(getattr(selection, "decision_cutin_candidates", ()))


def _selector(simulator: Any, frames: list[Any]) -> tuple[Any, dict[str, str]]:
  multitask_model = Path(simulator.DEFAULT_MULTITASK_MODEL)
  front_model = Path(getattr(simulator, "DEFAULT_FRONT_MODEL", multitask_model))
  corner_model = Path(getattr(simulator, "DEFAULT_CORNER_MODEL", front_model))
  if not front_model.is_file():
    front_model = multitask_model
  if not corner_model.is_file():
    corner_model = front_model

  selector_type = simulator.ProductionHybridLeadSelector
  try:
    selector = selector_type(front_model, frames, corner_model_path=corner_model)
  except TypeError:
    selector = selector_type(front_model, frames)
  return selector, {
    "front": str(front_model),
    "corner": str(corner_model),
    "selector": str(selector.name),
  }


def _case_log(case: dict[str, Any]) -> str:
  return str(Path(str(case["vehicle_folder"])) / Path(str(case["log"]))).replace("\\", "/")


def _event_review(
  episode: CutinEpisode, cases: list[dict[str, Any]],
) -> tuple[str, list[str]]:
  matched_detect: list[str] = []
  matched_clear: list[str] = []
  episode_ids = set(episode.track_ids)
  for case in cases:
    expected = str(case.get("expected", ""))
    if expected not in ("detect", "clear"):
      continue
    start_s, end_s = (float(value) for value in case["window"])
    if episode.end_s < start_s or episode.start_s > end_s:
      continue
    target_ids = {
      int(value)
      for value in (
        case.get("target_track_ids", ())
        if expected == "detect"
        else case.get("forbidden_lead_two_ids", ())
      )
    }
    if target_ids and not episode_ids & target_ids:
      continue
    (matched_detect if expected == "detect" else matched_clear).append(str(case["id"]))
  if matched_clear:
    return "known_clear", matched_clear
  if matched_detect:
    return "known_detect", matched_detect
  return "unreviewed", []


def _export_log(
  simulator: Any,
  route_root: Path,
  relative_log: str,
  cases: list[dict[str, Any]],
) -> tuple[list[CutinEpisode], list[CutinEpisode], dict[str, str]]:
  log_path = route_root / Path(relative_log)
  if not log_path.is_file():
    raise FileNotFoundError(log_path)
  frames = simulator.load_frames(log_path)
  selector, model_info = _selector(simulator, frames)

  open_by_stage: dict[str, list[dict[str, Any]]] = {
    "output": [],
    "decision": [],
  }
  completed_by_stage: dict[str, list[CutinEpisode]] = {
    "output": [],
    "decision": [],
  }

  def finish(stage: str, state: dict[str, Any]) -> None:
    episode = CutinEpisode(
      log=relative_log,
      start_s=round(float(state["start_s"]), 3),
      end_s=round(float(state["last_s"]), 3),
      track_ids=sorted(int(value) for value in state["track_ids"]),
      first_d_rel=state["first_d_rel"],
      min_d_rel=state["min_d_rel"],
      max_score=round(float(state["max_score"]), 4),
      reasons=sorted(str(value) for value in state["reasons"]),
    )
    episode.review, episode.review_cases = _event_review(episode, cases)
    completed_by_stage[stage].append(episode)

  def record(stage: str, frame_time: float, candidates: list[Any]) -> None:
    open_episodes = open_by_stage[stage]
    matched_states: set[int] = set()
    for candidate in candidates:
      ids = _candidate_ids(candidate)
      state_index = next(
        (
          state_index
          for state_index, state in enumerate(open_episodes)
          if frame_time - float(state["last_s"]) <= EPISODE_GAP_S
          and bool(ids & state["track_ids"])
        ),
        None,
      )
      distance = getattr(candidate, "d_rel", None)
      distance = (
        float(distance)
        if distance is not None and math.isfinite(float(distance)) and float(distance) > 0.0
        else None
      )
      if state_index is None:
        state = {
          "start_s": frame_time,
          "last_s": frame_time,
          "track_ids": set(ids),
          "first_d_rel": distance,
          "min_d_rel": distance,
          "max_score": float(getattr(candidate, "score", 0.0)),
          "reasons": {str(getattr(candidate, "reason", ""))},
        }
        open_episodes.append(state)
        state_index = len(open_episodes) - 1
      else:
        state = open_episodes[state_index]
        state["last_s"] = frame_time
        state["track_ids"].update(ids)
        state["max_score"] = max(state["max_score"], float(getattr(candidate, "score", 0.0)))
        state["reasons"].add(str(getattr(candidate, "reason", "")))
        if distance is not None:
          state["min_d_rel"] = (
            distance if state["min_d_rel"] is None else min(state["min_d_rel"], distance)
          )
      matched_states.add(state_index)

    expired = [
      state
      for state_index, state in enumerate(open_episodes)
      if state_index not in matched_states and frame_time - float(state["last_s"]) > EPISODE_GAP_S
    ]
    for state in expired:
      finish(stage, state)
      open_episodes.remove(state)

  for index, frame in enumerate(frames):
    frame_time = float(frame.time_s)
    selection = selector.select(frame, index)
    record("output", frame_time, _active_cutins(selection))
    record("decision", frame_time, _decision_cutins(selection))

  for stage, open_episodes in open_by_stage.items():
    for state in open_episodes:
      finish(stage, state)
  return completed_by_stage["output"], completed_by_stage["decision"], model_info


def _registered_logs(cases: list[dict[str, Any]]) -> list[str]:
  return sorted({_case_log(case) for case in cases})


def export_events(args: argparse.Namespace) -> int:
  cases_payload = json.loads(args.cases.read_text(encoding="utf-8"))
  cases = list(cases_payload.get("cases", ()))
  logs = _registered_logs(cases)
  simulator = _load_simulator(args.checkout_root)
  events: list[CutinEpisode] = []
  decision_events: list[CutinEpisode] = []
  models: dict[str, str] = {}
  failures: list[dict[str, str]] = []
  for index, relative_log in enumerate(logs, 1):
    print(f"[{index}/{len(logs)}] {relative_log}", flush=True)
    log_cases = [case for case in cases if _case_log(case) == relative_log]
    try:
      log_events, log_decisions, model_info = _export_log(
        simulator, args.route_root, relative_log, log_cases,
      )
      events.extend(log_events)
      decision_events.extend(log_decisions)
      models.update(model_info)
    except Exception as exc:  # Keep the full audit moving and report every unreadable log.
      failures.append({"log": relative_log, "error": f"{type(exc).__name__}: {exc}"})
      print(f"  ERROR {failures[-1]['error']}", flush=True)

  try:
    checkout_commit = subprocess.check_output(
      ["git", "rev-parse", "HEAD"],
      cwd=args.checkout_root,
      text=True,
      stderr=subprocess.DEVNULL,
    ).strip()
  except (OSError, subprocess.CalledProcessError):
    checkout_commit = "unknown"
  normalized_models = {
    key: Path(value).name if key in ("front", "corner") else value
    for key, value in models.items()
  }
  payload = {
    "version": 2,
    "checkout_commit": checkout_commit,
    "models": normalized_models,
    "registered_logs": len(logs),
    "events": [asdict(event) for event in events],
    "decision_events": [asdict(event) for event in decision_events],
    "failures": failures,
  }
  args.output.parent.mkdir(parents=True, exist_ok=True)
  args.output.write_text(json.dumps(payload, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")
  counts = {
    review: sum(event.review == review for event in events)
    for review in ("known_detect", "known_clear", "unreviewed")
  }
  print(
    f"wrote {args.output}: {len(events)} events, "
    f"{len(decision_events)} decisions, "
    f"detect {counts['known_detect']}, clear {counts['known_clear']}, "
    f"unreviewed {counts['unreviewed']}, failures {len(failures)}",
  )
  return int(bool(failures))


def _events_by_log(
  payload: dict[str, Any], key: str = "events",
) -> dict[str, list[dict[str, Any]]]:
  result: dict[str, list[dict[str, Any]]] = {}
  for event in payload.get(key, ()):
    result.setdefault(str(event["log"]), []).append(event)
  return result


def _event_distance(left: dict[str, Any], right: dict[str, Any]) -> float:
  if float(left["end_s"]) < float(right["start_s"]):
    return float(right["start_s"]) - float(left["end_s"])
  if float(right["end_s"]) < float(left["start_s"]):
    return float(left["start_s"]) - float(right["end_s"])
  return 0.0


def compare_payloads(
  baseline: dict[str, Any],
  current: dict[str, Any],
  match_tolerance: float = 1.0,
  late_tolerance: float = 0.35,
) -> dict[str, Any]:
  baseline_by_log = _events_by_log(baseline)
  current_by_log = _events_by_log(current)
  current_decisions_by_log = _events_by_log(current, "decision_events")
  comparisons: list[dict[str, Any]] = []
  summary = {
    "matched": 0,
    "removed_known_clear": 0,
    "retained_known_clear": 0,
    "suppressed_current": 0,
    "decision_only_current": 0,
    "missing_current": 0,
    "new_current": 0,
    "later_current": 0,
  }

  for log_name in sorted(set(baseline_by_log) | set(current_by_log)):
    remaining = list(enumerate(current_by_log.get(log_name, ())))
    matched_current: set[int] = set()
    for old_event in baseline_by_log.get(log_name, ()):
      candidates = [
        (index, event, _event_distance(old_event, event))
        for index, event in remaining
        if _event_distance(old_event, event) <= match_tolerance
      ]
      if not candidates:
        decisions = [
          (event, _event_distance(old_event, event))
          for event in current_decisions_by_log.get(log_name, ())
          if _event_distance(old_event, event) <= match_tolerance
        ]
        if decisions:
          decision, _ = min(
            decisions,
            key=lambda item: (
              abs(float(item[0]["start_s"]) - float(old_event["start_s"])),
              item[1],
            ),
          )
        else:
          decision = None
        if str(old_event.get("review", "")) == "known_clear":
          summary["removed_known_clear"] += 1
          comparisons.append({
            "status": "removed_known_clear",
            "log": log_name,
            "baseline": old_event,
            "current": None,
            "current_decision": decision,
          })
          continue
        if decision is not None:
          status = (
            "suppressed_current"
            if (
              str(old_event.get("review", "")) == "known_detect"
              or str(decision.get("review", "")) == "known_detect"
            )
            else "decision_only_current"
          )
          summary[status] += 1
          comparisons.append({
            "status": status,
            "log": log_name,
            "delay_s": round(float(decision["start_s"]) - float(old_event["start_s"]), 3),
            "baseline": old_event,
            "current": None,
            "current_decision": decision,
          })
          continue
        summary["missing_current"] += 1
        comparisons.append({
          "status": "missing_current",
          "log": log_name,
          "baseline": old_event,
          "current": None,
          "current_decision": None,
        })
        continue
      index, new_event, _ = min(
        candidates,
        key=lambda item: (
          abs(float(item[1]["start_s"]) - float(old_event["start_s"])),
          item[2],
        ),
      )
      matched_current.add(index)
      delay_s = round(float(new_event["start_s"]) - float(old_event["start_s"]), 3)
      status = (
        "retained_known_clear"
        if str(old_event.get("review", "")) == "known_clear"
        else ("later_current" if delay_s > late_tolerance else "matched")
      )
      summary[status] += 1
      comparisons.append({
        "status": status,
        "log": log_name,
        "delay_s": delay_s,
        "baseline": old_event,
        "current": new_event,
        "current_decision": None,
      })

    for index, new_event in remaining:
      if index in matched_current:
        continue
      summary["new_current"] += 1
      comparisons.append({
        "status": "new_current",
        "log": log_name,
        "baseline": None,
        "current": new_event,
        "current_decision": None,
      })

  return {
    "version": 2,
    "summary": summary,
    "comparisons": comparisons,
  }


def compare_events(args: argparse.Namespace) -> int:
  baseline = json.loads(args.baseline.read_text(encoding="utf-8"))
  current = json.loads(args.current.read_text(encoding="utf-8"))
  result = compare_payloads(
    baseline,
    current,
    match_tolerance=args.match_tolerance,
    late_tolerance=args.late_tolerance,
  )
  result.update({
    "baseline": str(args.baseline),
    "current": str(args.current),
  })
  args.output.parent.mkdir(parents=True, exist_ok=True)
  args.output.write_text(json.dumps(result, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")
  print(json.dumps(result["summary"], indent=2))
  print(f"wrote {args.output}")
  return 0


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description=__doc__)
  subparsers = parser.add_subparsers(dest="command", required=True)

  export_parser = subparsers.add_parser("export", help="run one checkout over every registered log")
  export_parser.add_argument("--checkout-root", type=Path, default=SCRIPT_REPO_ROOT)
  export_parser.add_argument("--cases", type=Path, default=DEFAULT_CASES)
  export_parser.add_argument("--route-root", type=Path, default=DEFAULT_ROUTE_ROOT)
  export_parser.add_argument("--output", type=Path, required=True)
  export_parser.set_defaults(handler=export_events)

  compare_parser = subparsers.add_parser("compare", help="compare two exported event manifests")
  compare_parser.add_argument("--baseline", type=Path, required=True)
  compare_parser.add_argument("--current", type=Path, required=True)
  compare_parser.add_argument("--output", type=Path, required=True)
  compare_parser.add_argument("--match-tolerance", type=float, default=1.0)
  compare_parser.add_argument("--late-tolerance", type=float, default=0.35)
  compare_parser.set_defaults(handler=compare_events)
  return parser.parse_args()


def main() -> int:
  args = parse_args()
  return int(args.handler(args))


if __name__ == "__main__":
  raise SystemExit(main())
