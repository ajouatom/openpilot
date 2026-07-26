#!/usr/bin/env python3
"""Compare carrot-wip and occupancy cut-in decisions on identical held-out logs."""

from __future__ import annotations

import argparse
from collections import defaultdict
from collections.abc import Sequence
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import sys
from typing import Any


CURRENT_REPO_ROOT = Path(__file__).resolve().parents[5]


@dataclass(frozen=True)
class ComparisonLabel:
  label_id: str
  vehicle_folder: str
  log: str
  source: str
  start_s: float
  end_s: float
  track_ids: tuple[int, ...]
  expected: str

  @property
  def log_group(self) -> str:
    return f"{self.vehicle_folder}/{self.log}"


@dataclass(frozen=True)
class ComparisonRow:
  label_id: str
  source: str
  expected: str
  predicted: str
  selected: bool
  max_probability: float
  scored_frames: int


def _path_occupancy_selected(candidate: Any) -> bool:
  """Count only the production candidate that became final leadTwo."""
  return str(getattr(candidate, "stage", "")) == "SELECTED"


def _load_labels(path: Path) -> list[ComparisonLabel]:
  payload = json.loads(path.read_text(encoding="utf-8"))
  labels = []
  for item in payload.get("labels", ()):
    if not item.get("human_verified", False) or item.get("expected") not in ("detect", "clear"):
      continue
    window = item.get("window", (item["time_s"], item["time_s"]))
    labels.append(ComparisonLabel(
      label_id=str(item["id"]),
      vehicle_folder=str(item["vehicle_folder"]),
      log=str(item["log"]),
      source=str(item["source"]),
      start_s=float(window[0]),
      end_s=float(window[1]),
      track_ids=tuple(int(value) for value in item.get("track_ids", (item["track_id"],))),
      expected=str(item["expected"]),
    ))
  return labels


def _summary(rows: Sequence[ComparisonRow]) -> dict[str, dict[str, float | int]]:
  tables = {}
  for source in ("front", "corner"):
    source_rows = [row for row in rows if row.source == source]
    tp = sum(row.expected == "detect" and row.selected for row in source_rows)
    fp = sum(row.expected == "clear" and row.selected for row in source_rows)
    fn = sum(row.expected == "detect" and not row.selected for row in source_rows)
    tn = sum(row.expected == "clear" and not row.selected for row in source_rows)
    precision = tp / max(tp + fp, 1)
    recall = tp / max(tp + fn, 1)
    tables[source] = {
      "labels": len(source_rows),
      "precision": precision,
      "recall": recall,
      "f1": 2.0 * precision * recall / max(precision + recall, 1e-9),
      "accuracy": (tp + tn) / max(tp + fp + fn + tn, 1),
      "tp": tp,
      "fp": fp,
      "fn": fn,
      "tn": tn,
    }
  return tables


def _configure_imports(repo_root: Path) -> None:
  roots = {str(CURRENT_REPO_ROOT.resolve()).lower(), str(repo_root.resolve()).lower()}
  sys.path[:] = [
    value for value in sys.path
    if not value or str(Path(value).resolve()).lower() not in roots
  ]
  sys.path.insert(0, str(repo_root.resolve()))
  if repo_root.resolve() != CURRENT_REPO_ROOT.resolve():
    # Unchanged openpilot dependencies are supplied by the working repository.
    sys.path.insert(1, str(CURRENT_REPO_ROOT.resolve()))


def evaluate(
  implementation: str,
  repo_root: Path,
  routes_root: Path,
  labels_path: Path,
) -> dict[str, Any]:
  _configure_imports(repo_root)
  from openpilot.selfdrive.carrot.radar.tools.radar_lead_simulator import (
    ProductionHybridLeadSelector,
    candidate_matches_targets,
    candidate_track_ids,
    load_frames,
  )

  labels = _load_labels(labels_path)
  by_log: dict[str, list[ComparisonLabel]] = defaultdict(list)
  for label in labels:
    by_log[label.log_group].append(label)

  front_model = repo_root / "openpilot/selfdrive/carrot/radar/models/radar_lead_front.npz"
  corner_model = repo_root / "openpilot/selfdrive/carrot/radar/models/radar_lead_corner.npz"
  selected_frames: dict[str, list[tuple[float, bool]]] = defaultdict(list)
  scores: dict[str, list[float]] = defaultdict(list)
  for log_index, log_labels in enumerate(by_log.values(), 1):
    log_path = routes_root / log_labels[0].vehicle_folder / Path(
      *log_labels[0].log.replace("\\", "/").split("/"),
    )
    print(
      f"[{implementation} {log_index:02d}/{len(by_log):02d}] {log_path}",
      flush=True,
    )
    frames = load_frames(log_path)
    selector = ProductionHybridLeadSelector(front_model, frames, corner_model)
    for frame_index, frame in enumerate(frames):
      selection = selector.select(frame, frame_index)
      candidates = (
        selection.decision_cutin_candidates
        if implementation == "carrot-wip"
        else selection.cutin_diagnostics
      )
      for label in log_labels:
        if not label.start_s <= frame.time_s <= label.end_s:
          continue
        matching = []
        for candidate in candidates:
          if not candidate_matches_targets(candidate, set(label.track_ids)):
            continue
          ids = candidate_track_ids(candidate)
          source_matches = any(
            point.measured
            and point.track_id in ids
            and ((label.source == "corner") == point.source.startswith("corner"))
            for point in frame.points
          )
          if source_matches:
            matching.append(candidate)
        if not matching:
          continue
        best = max(matching, key=lambda value: value.score)
        selected = (
          True
          if implementation == "carrot-wip"
          else _path_occupancy_selected(best)
        )
        selected_frames[label.label_id].append((frame.time_s, selected))
        scores[label.label_id].append(float(best.score))

  rows = []
  for label in labels:
    values = selected_frames[label.label_id]
    selected = any(value[1] for value in values)
    rows.append(ComparisonRow(
      label_id=label.label_id,
      source=label.source,
      expected=label.expected,
      predicted="detect" if selected else "clear",
      selected=selected,
      max_probability=max(scores[label.label_id], default=0.0),
      scored_frames=len(values),
    ))
  return {
    "implementation": implementation,
    "repo_root": str(repo_root),
    "labels_path": str(labels_path),
    "logs": len(by_log),
    "summary": _summary(rows),
    "rows": [asdict(row) for row in rows],
  }


def merge_results(
  baseline_path: Path,
  occupancy_path: Path,
  report_path: Path,
) -> None:
  baseline = json.loads(baseline_path.read_text(encoding="utf-8"))
  occupancy = json.loads(occupancy_path.read_text(encoding="utf-8"))
  for result in (baseline, occupancy):
    result.pop("repo_root", None)
    result.pop("labels_path", None)
  report = json.loads(report_path.read_text(encoding="utf-8"))
  audit_rows = {
    str(row["label_id"]): row
    for row in report.get("manual_evaluation", {}).get("rows", ())
  }

  def actual_future_summary(result: dict[str, Any]) -> dict[str, dict[str, float | int]]:
    rows = []
    for row in result.get("rows", ()):
      audit = audit_rows.get(str(row["label_id"]))
      if audit is None or int(audit.get("actual_valid_frames", 0)) <= 0:
        continue
      rows.append(ComparisonRow(
        label_id=str(row["label_id"]),
        source=str(row["source"]),
        expected="detect" if int(audit.get("actual_entry_frames", 0)) > 0 else "clear",
        predicted=str(row["predicted"]),
        selected=bool(row["selected"]),
        max_probability=float(row["max_probability"]),
        scored_frames=int(row["scored_frames"]),
      ))
    return _summary(rows)

  report["carrot_wip_comparison"] = {
    "basis": " ".join((
      "identical held-out manual windows; carrot-wip production decision",
      "versus production path-occupancy decision after final lead selection",
    )),
    "baseline_revision": "9088829005",
    "carrot_wip": baseline,
    "path_occupancy": occupancy,
    "actual_future_summary": {
      "basis": " ".join((
        "same scorable manual windows, but expected detect/clear is replaced",
        "by measured same-vehicle future path entry",
      )),
      "carrot_wip": actual_future_summary(baseline),
      "path_occupancy": actual_future_summary(occupancy),
    },
  }
  report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--implementation", choices=("carrot-wip", "path-occupancy"))
  parser.add_argument("--repo-root", type=Path)
  parser.add_argument("--routes-root", type=Path, default=Path("W:/routes"))
  parser.add_argument(
    "--labels",
    type=Path,
    default=CURRENT_REPO_ROOT / "openpilot/selfdrive/carrot/cluster/radar_trajectory_labels.json",
  )
  parser.add_argument("--output", type=Path)
  parser.add_argument("--baseline-results", type=Path)
  parser.add_argument("--occupancy-results", type=Path)
  parser.add_argument("--merge-report", type=Path)
  return parser.parse_args()


def main() -> int:
  args = parse_args()
  merge_mode = all((
    args.baseline_results,
    args.occupancy_results,
    args.merge_report,
  ))
  if merge_mode:
    merge_results(args.baseline_results, args.occupancy_results, args.merge_report)
    print(f"comparison merged: {args.merge_report}")
    return 0
  if args.implementation is None or args.repo_root is None or args.output is None:
    raise SystemExit("--implementation, --repo-root, and --output are required")
  result = evaluate(args.implementation, args.repo_root, args.routes_root, args.labels)
  args.output.parent.mkdir(parents=True, exist_ok=True)
  args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
  print(json.dumps(result["summary"], indent=2))
  print(f"results: {args.output}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
