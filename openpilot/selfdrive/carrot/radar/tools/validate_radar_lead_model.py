#!/usr/bin/env python3
"""Validate existing radard and report the physical dPath predictor in shadow."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[5]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.selfdrive.carrot.radar.tools.radar_validation_replay import (
  CurrentRadardSelector,
  RadarMotionShadowSelector,
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


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Replay maintained validation logs through radard and physical dPath shadow",
  )
  parser.add_argument("--root", type=Path, default=Path(r"W:\routes"))
  parser.add_argument("--cases", type=Path, default=DEFAULT_CASES)
  parser.add_argument("--labels", type=Path, default=DEFAULT_LABELS)
  parser.add_argument("--case", action="append", default=[])
  parser.add_argument("--expected", choices=("detect", "clear", "stationary"))
  parser.add_argument("--front-only", action="store_true")
  parser.add_argument(
    "--cases-only",
    action="store_true",
    help="skip radar_trajectory_labels.json; full validation uses both sources",
  )
  parser.add_argument("--report", type=Path)
  parser.add_argument(
    "--strict-radard",
    action="store_true",
    help="return nonzero when an existing-radard expectation fails",
  )
  return parser.parse_args()


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


def _first_event(
  selector: Any,
  frames: list[Any],
  entry: dict[str, Any],
  attribute: str,
) -> tuple[float, int] | None:
  start_s, end_s = (float(value) for value in entry["window"])
  targets = {int(value) for value in entry.get("target_track_ids", ())}
  source = str(entry.get("source", "front+corner"))
  for index, frame in enumerate(frames):
    if not start_s <= frame.time_s <= end_s:
      continue
    candidates = tuple(getattr(selector.select(frame, index), attribute))
    matches = [
      candidate for candidate in candidates
      if candidate_matches_targets(candidate, targets)
      and _source_matches(source, candidate.source)
    ]
    if matches:
      best = min(
        matches,
        key=lambda candidate: candidate.d_rel if candidate.d_rel is not None else float("inf"),
      )
      return frame.time_s, best.track_id
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
    print(
      f"[{log_index:02d}/{len(grouped):02d}] loading {path.parent.name}/{path.name} "
      + f"({len(log_entries)} labels) ...",
      flush=True,
    )
    frames = load_frames(path)
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
    cutin_ids = current_cutin_track_ids(path, frames, sources)
    radard = CurrentRadardSelector(frames, cutin_ids)
    shadow = RadarMotionShadowSelector(frames)
    for entry in log_entries:
      expected = str(entry["expected"])
      if expected == "stationary":
        radard_event = _stationary_event(radard, frames, entry)
        shadow_event = _stationary_event(
          shadow,
          frames,
          entry,
          require_target_ids=False,
          maximum_abs_v_lead=STATIONARY_MAX_ABS_VLEAD_MPS,
        )
        radard_pass = radard_event is not None
      else:
        radard_event = _first_event(
          radard, frames, entry, "active_cutin_candidates",
        )
        entry_source = str(entry.get("source", "front+corner"))
        shadow_applicable = (
          entry_source == "front+corner"
          or entry_source == shadow.motion_sensor
        )
        shadow_event = (
          _first_event(
            shadow, frames, entry, "decision_cutin_candidates",
          )
          if shadow_applicable
          else None
        )
        radard_pass = (radard_event is not None) == (expected == "detect")
      if expected == "stationary":
        shadow_applicable = True
      deadline = entry.get("latest_detection_s")
      if (
        radard_pass
        and deadline is not None
        and radard_event is not None
        and expected == "detect"
      ):
        radard_pass = radard_event[0] <= float(deadline)
      row = {
        "id": str(entry["id"]),
        "validation_set": str(entry["validation_set"]),
        "source": str(entry.get("source", "")),
        "expected": expected,
        "radard_event": radard_event,
        "shadow_event": shadow_event,
        "shadow_applicable": shadow_applicable,
        "shadow_sensor": shadow.motion_sensor,
        "radard_pass": radard_pass,
      }
      rows.append(row)
      print(
        f"  {'PASS' if radard_pass else 'FAIL'} {entry['id']} "
        + f"expected={expected} radard={_event_text(radard_event)} "
        + (
          f"shadow={_event_text(shadow_event)}"
          if shadow_applicable
          else f"shadow=N/A({shadow.motion_sensor})"
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
    _print_metrics("existing radard", _metrics(subset, "radard_event"))
    _print_metrics("physical dPath shadow", _metrics(subset, "shadow_event"))
  radard_failures = sum(not row["radard_pass"] for row in rows)
  print(
    f"\nprocessed={len(rows)} missing={missing} "
    + f"existing-radard expectation failures={radard_failures}"
  )

  if args.report is not None:
    payload = {
      "version": 1,
      "description": "existing radard control versus physical dPath shadow; manual labels validation-only",
      "rows": rows,
      "summary": {
        "existing_radard": _metrics(rows, "radard_event"),
        "physical_dpath_shadow": _metrics(rows, "shadow_event"),
        "missing": missing,
        "radard_expectation_failures": radard_failures,
      },
    }
    args.report.write_text(
      json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
      encoding="utf-8",
    )
    print(f"report written: {args.report}")
  return int(missing > 0 or (args.strict_radard and radard_failures > 0))


if __name__ == "__main__":
  raise SystemExit(main())
