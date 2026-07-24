#!/usr/bin/env python3
"""Validate production model cut-in decisions against maintained route scenes."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[5]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.selfdrive.carrot.radar.tools.radar_lead_simulator import (
  DEFAULT_CORNER_MODEL,
  DEFAULT_FRONT_MODEL,
  DEFAULT_MULTITASK_MODEL,
  ProductionHybridLeadSelector,
  candidate_matches_targets,
  current_cutin_track_ids,
  load_frames,
)


DEFAULT_CASES = Path(__file__).resolve().parents[2] / "cluster" / "cutin_validation_cases.json"


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Run model cut-in regression routes without opening the UI")
  parser.add_argument("--model", type=Path, help="legacy single model for both radar sources")
  parser.add_argument("--front-model", type=Path, default=DEFAULT_FRONT_MODEL)
  parser.add_argument("--corner-model", type=Path, default=DEFAULT_CORNER_MODEL)
  parser.add_argument("--baseline-model", type=Path, default=DEFAULT_MULTITASK_MODEL)
  parser.add_argument("--baseline-front-model", type=Path)
  parser.add_argument("--baseline-corner-model", type=Path)
  parser.add_argument("--no-baseline", action="store_true", help="skip the comparison model for faster regression")
  parser.add_argument("--root", type=Path, default=Path(r"W:\routes"))
  parser.add_argument("--cases", type=Path, default=DEFAULT_CASES)
  parser.add_argument("--case", action="append", default=[])
  parser.add_argument("--expected", choices=("detect", "clear"))
  parser.add_argument(
    "--compare-radard", action="store_true",
    help="also recompute current radard cut-ins (slow)",
  )
  return parser.parse_args()


def first_model_cutin(selector, frames, case) -> tuple[float, int] | None:
  start, end = (float(value) for value in case["window"])
  targets = {int(value) for value in case.get("target_track_ids", ())}
  for index, frame in enumerate(frames):
    if not start <= frame.time_s <= end:
      continue
    selection = selector.select(frame, index)
    lead_one_id = selection.lead_one.track_id if selection.lead_one is not None else None
    candidates = [
      candidate for candidate in selection.active_cutin_candidates
      if candidate.track_id != lead_one_id and candidate_matches_targets(candidate, targets)
    ]
    if candidates:
      return frame.time_s, candidates[0].track_id
  return None


def first_internal_model_cutin(selector, frames, case) -> tuple[float, int] | None:
  start, end = (float(value) for value in case["window"])
  targets = {int(value) for value in case.get("target_track_ids", ())}
  for index, frame in enumerate(frames):
    if not start <= frame.time_s <= end:
      continue
    selection = selector.select(frame, index)
    candidates = [
      candidate for candidate in selection.decision_cutin_candidates
      if candidate_matches_targets(candidate, targets)
    ]
    if candidates:
      return frame.time_s, candidates[0].track_id
  return None


def control_suppression_status(selector, frames, case) -> tuple[bool, str]:
  if not case.get("require_control_suppressed", False):
    return True, ""
  start, end = (float(value) for value in case["window"])
  targets = {int(value) for value in case.get("target_track_ids", ())}
  event = next((
    (frame.time_s, candidate.track_id)
    for index, frame in enumerate(frames)
    if start <= frame.time_s <= end
    for candidate in selector.select(frame, index).active_cutin_candidates
    if candidate_matches_targets(candidate, targets)
  ), None)
  return event is None, f"control={'suppressed' if event is None else event_text(event)}"


def first_raw_model_cutin(selector, frames, case) -> tuple[float, int] | None:
  """Return the first base/auxiliary cut-in score above the artifact threshold."""
  start, end = (float(value) for value in case["window"])
  targets = {int(value) for value in case.get("target_track_ids", ())}
  for index, frame in enumerate(frames):
    if not start <= frame.time_s <= end:
      continue
    selection = selector.select(frame, index)
    candidates = [
      candidate for candidate in selection.corner_candidates
      if candidate.eligible and candidate_matches_targets(candidate, targets)
    ]
    if candidates:
      return frame.time_s, candidates[0].track_id
  return None


def first_radard_cutin(frames, ids_by_frame, case) -> tuple[float, int] | None:
  start, end = (float(value) for value in case["window"])
  targets = {int(value) for value in case.get("target_track_ids", ())}
  for frame, track_ids in zip(frames, ids_by_frame, strict=True):
    matches = sorted(track_id for track_id in track_ids if not targets or track_id in targets)
    if start <= frame.time_s <= end and matches:
      return frame.time_s, matches[0]
  return None


def event_text(event: tuple[float, int] | None) -> str:
  return "--" if event is None else f"{event[0]:.2f}s/id{event[1]}"


def detection_deadline_status(event: tuple[float, int] | None, case: dict) -> tuple[bool, str]:
  deadline = case.get("latest_detection_s")
  if deadline is None or event is None:
    return True, ""
  passed = event[0] <= float(deadline)
  return passed, f"deadline={'ok' if passed else 'late'}@{float(deadline):.2f}s"


def lead_constraint_status(selector, frames, case) -> tuple[bool, str]:
  start, end = (float(value) for value in case["window"])
  indexed = [
    (frame, selector.select(frame, index))
    for index, frame in enumerate(frames)
    if start <= frame.time_s <= end
  ]
  details: list[str] = []
  passed = True
  required_one = {int(value) for value in case.get("required_lead_one_ids", ())}
  if required_one:
    matches = [
      (frame.time_s, selection.lead_one.track_id)
      for frame, selection in indexed
      if candidate_matches_targets(selection.lead_one, required_one)
    ]
    required_pass = bool(matches)
    if case.get("require_lead_one_continuous", False):
      required_pass = bool(indexed) and all(
        candidate_matches_targets(selection.lead_one, required_one)
        for _, selection in indexed
      )
    passed &= required_pass
    details.append(f"L1={'ok' if required_pass else 'missing'}")

  for field, attribute, label in (
    ("forbidden_lead_one_ids", "lead_one", "badL1"),
    ("forbidden_lead_two_ids", "lead_two", "badL2"),
  ):
    forbidden = {int(value) for value in case.get(field, ())}
    if not forbidden:
      continue
    event = next((
      (frame.time_s, lead.track_id)
      for frame, selection in indexed
      if candidate_matches_targets((lead := getattr(selection, attribute)), forbidden)
    ), None)
    passed &= event is None
    details.append(f"{label}={event_text(event)}")
  return passed, " ".join(details)


def cutin_continuity_status(selector, frames, case) -> tuple[bool, str]:
  if not case.get("require_cutin_continuity", False):
    return True, ""

  start, end = (float(value) for value in case["window"])
  targets = {int(value) for value in case.get("target_track_ids", ())}
  indexed = [
    (frame, selector.select(frame, index))
    for index, frame in enumerate(frames)
    if start <= frame.time_s <= end
  ]

  first_active = next((offset for offset, (_, selection) in enumerate(indexed) if any(
    candidate_matches_targets(candidate, targets)
    for candidate in selection.active_cutin_candidates
  )), None)
  if first_active is None:
    return False, "continuity=no-start"

  takeover = next((offset for offset in range(first_active, len(indexed)) if (
    indexed[offset][1].lead_one is not None
    and candidate_matches_targets(indexed[offset][1].lead_one, targets)
  )), None)
  if case.get("require_lead_one_takeover", False) and takeover is None:
    return False, "continuity=no-L1-takeover"

  stop = takeover if takeover is not None else len(indexed) - 1
  last_present_s = indexed[first_active][0].time_s
  max_gap_s = 0.0
  for frame, selection in indexed[first_active:stop + 1]:
    lead_one_present = candidate_matches_targets(selection.lead_one, targets)
    cutin_present = any(
      candidate_matches_targets(candidate, targets)
      for candidate in selection.active_cutin_candidates
    )
    if lead_one_present or cutin_present:
      last_present_s = frame.time_s
    else:
      max_gap_s = max(max_gap_s, frame.time_s - last_present_s)

  allowed_gap_s = float(case.get("max_cutin_gap_s", 0.15))
  passed = max_gap_s <= allowed_gap_s
  takeover_text = "--" if takeover is None else f"{indexed[takeover][0].time_s:.2f}s"
  return passed, f"continuity=gap{max_gap_s:.2f}s/L1@{takeover_text}"


def main() -> int:
  args = parse_args()
  if args.model is not None:
    args.front_model = args.corner_model = args.model
  if not args.front_model.is_file() or not args.corner_model.is_file():
    raise SystemExit(
      f"source models do not exist: front={args.front_model} corner={args.corner_model}"
    )
  baseline_front = args.baseline_front_model or args.baseline_model
  baseline_corner = args.baseline_corner_model or args.baseline_model
  payload = json.loads(args.cases.read_text(encoding="utf-8"))
  filters = tuple(value.lower() for value in args.case)
  cases = [
    case for case in payload["cases"]
    if case["expected"] in ("detect", "clear")
    and (args.expected is None or case["expected"] == args.expected)
    and (not filters or any(value in case["id"].lower() for value in filters))
  ]
  if not cases:
    raise SystemExit("no matching detect/clear cases")

  cache: dict[Path, tuple] = {}
  failures = 0
  source_totals: dict[str, int] = {}
  source_passes: dict[str, int] = {}
  for index, case in enumerate(cases, 1):
    source = str(case["source"])
    source_totals[source] = source_totals.get(source, 0) + 1
    path = args.root / case["vehicle_folder"] / Path(case["log"])
    if not path.is_file():
      failures += 1
      print(f"[{index:02d}/{len(cases):02d}] MISSING {case['id']}: {path}", flush=True)
      continue
    if path not in cache:
      print(f"loading {path.name} ...", flush=True)
      frames = load_frames(path)
      baseline = None if args.no_baseline else ProductionHybridLeadSelector(baseline_front, frames, baseline_corner)
      candidate = ProductionHybridLeadSelector(args.front_model, frames, args.corner_model)
      radard_ids = (
        current_cutin_track_ids(path, frames, (str(case["source"]),))
        if args.compare_radard else tuple(frozenset() for _ in frames)
      )
      cache[path] = frames, baseline, candidate, {str(case["source"]): radard_ids}
    frames, baseline, candidate, source_ids = cache[path]
    if args.compare_radard and source not in source_ids:
      source_ids[source] = current_cutin_track_ids(path, frames, (source,))
    radard_event = first_radard_cutin(frames, source_ids[source], case) if args.compare_radard else None
    decision_stage = case.get("validation_stage") == "decision"
    event_fn = first_internal_model_cutin if decision_stage else first_model_cutin
    baseline_event = event_fn(baseline, frames, case) if baseline is not None else None
    candidate_event = event_fn(candidate, frames, case)
    baseline_raw_event = first_raw_model_cutin(baseline, frames, case) if baseline is not None else None
    candidate_raw_event = first_raw_model_cutin(candidate, frames, case)
    deadline_passed, deadline_status = detection_deadline_status(candidate_event, case)
    lead_passed, lead_status = lead_constraint_status(candidate, frames, case)
    continuity_passed, continuity_status = cutin_continuity_status(candidate, frames, case)
    suppression_passed, suppression_status = control_suppression_status(candidate, frames, case)
    passed = (
      (candidate_event is not None) == (case["expected"] == "detect")
      and deadline_passed and lead_passed and continuity_passed and suppression_passed
    )
    source_passes[source] = source_passes.get(source, 0) + int(passed)
    failures += not passed
    delta = ""
    if args.compare_radard and candidate_event is not None and radard_event is not None:
      delta = f" delta={candidate_event[0] - radard_event[0]:+.2f}s"
    radard_text = f"radard={event_text(radard_event)} " if args.compare_radard else ""
    result_text = f"[{index:02d}/{len(cases):02d}] {'PASS' if passed else 'FAIL'} {case['id']} "
    result_text += f"{radard_text}raw={event_text(baseline_raw_event)}->{event_text(candidate_raw_event)} "
    result_text += f"filtered={event_text(baseline_event)}->{event_text(candidate_event)}{delta} "
    result_text += f"{deadline_status} {lead_status} {continuity_status} {suppression_status}"
    print(result_text, flush=True)
  print(f"\n{len(cases) - failures}/{len(cases)} passed")
  for source in sorted(source_totals):
    print(f"  {source}: {source_passes.get(source, 0)}/{source_totals[source]} passed")
  return int(bool(failures))


if __name__ == "__main__":
  raise SystemExit(main())
