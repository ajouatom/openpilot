#!/usr/bin/env python3
"""Replay routes and pause on cut-in, vision-only, or unmatched vision events."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys


SCRIPT_DIR = Path(__file__).resolve().parent
RADAR_ROOT = SCRIPT_DIR.parent
CARROT_ROOT = RADAR_ROOT.parent
DEFAULT_CASES = CARROT_ROOT / "cluster" / "cutin_validation_cases.json"
DEFAULT_MODEL = RADAR_ROOT / "models" / "radar_lead_multitask.npz"
DEFAULT_FRONT_MODEL = RADAR_ROOT / "models" / "radar_lead_front.npz"
DEFAULT_CORNER_MODEL = RADAR_ROOT / "models" / "radar_lead_corner.npz"
DEFAULT_TRAJECTORY_REPORT = RADAR_ROOT / "models" / "radar_path_occupancy_report.json"
SIMULATOR = SCRIPT_DIR / "radar_lead_simulator.py"


def group_cases_by_log(cases: list[dict]) -> list[list[dict]]:
  groups: dict[tuple[str, str], list[dict]] = {}
  for case in cases:
    key = (str(case["vehicle_folder"]), str(case["log"]))
    groups.setdefault(key, []).append(case)
  return list(groups.values())


def print_trajectory_evaluation_table(report_path: Path) -> None:
  report = json.loads(report_path.read_text(encoding="utf-8"))
  summary = report["manual_evaluation"]["summary"]
  print("Manual CUT-IN/CLEAR vs path-occupancy model (labels are validation-only)")
  print("| model | labels | precision | recall | F1 | accuracy | TP | FP | FN | TN | manual/future |")
  print("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
  for source in ("front", "corner"):
    values = summary[source]
    print(
      f"| {source} | {values['labels']} | {float(values['precision']):.3f} | "
      + f"{float(values['recall']):.3f} | {float(values['f1']):.3f} | "
      + f"{float(values['accuracy']):.3f} | {values['tp']} | {values['fp']} | "
      + f"{values['fn']} | {values['tn']} | "
      + f"{values['manual_actual_agree']}/{values['manual_actual_scorable']} |"
    )


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Replay routes and pause on cut-in, vision-only, or unmatched vision events")
  parser.add_argument("--root", type=Path, default=Path(r"W:\routes"), help="route log root")
  parser.add_argument("--model", type=Path, help="legacy single model for both radar sources")
  parser.add_argument(
    "--front-model", type=Path,
    default=DEFAULT_FRONT_MODEL if DEFAULT_FRONT_MODEL.is_file() else DEFAULT_MODEL,
  )
  parser.add_argument(
    "--corner-model", type=Path,
    default=DEFAULT_CORNER_MODEL if DEFAULT_CORNER_MODEL.is_file() else DEFAULT_MODEL,
  )
  parser.add_argument("--cases", type=Path, default=DEFAULT_CASES)
  parser.add_argument("--case", action="append", default=[], help="case-id substring; repeat to select more")
  parser.add_argument("--expected", choices=("all", "detect", "clear", "stationary"), default="all")
  parser.add_argument(
    "--prob", type=float,
    help="initial review probability (0.00-1.00); omitted means reuse the last slider value",
  )
  parser.add_argument(
    "--compare-radard", action="store_true",
    help="also recompute and display the current radard result and graph (slow)",
  )
  parser.add_argument(
    "--front-only", action="store_true",
    help="remove corner-radar points and validate the production front-only path",
  )
  parser.add_argument("--list", action="store_true")
  parser.add_argument(
    "--trajectory-table",
    action="store_true",
    help="print the saved front/corner manual-label comparison before replay",
  )
  parser.add_argument("--trajectory-report", type=Path, default=DEFAULT_TRAJECTORY_REPORT)
  parser.add_argument(
    "--trajectory-table-only",
    action="store_true",
    help="print the saved trajectory comparison and exit",
  )
  return parser.parse_args()


def main() -> int:
  args = parse_args()
  if args.prob is not None and not 0.0 <= args.prob <= 1.0:
    raise SystemExit("--prob must be between 0.00 and 1.00")
  if args.trajectory_table or args.trajectory_table_only:
    print_trajectory_evaluation_table(args.trajectory_report)
    if args.trajectory_table_only:
      return 0
  payload = json.loads(args.cases.read_text(encoding="utf-8"))
  filters = tuple(value.lower() for value in args.case)
  cases = [
    case for case in payload.get("cases", ())
    if (args.expected == "all" or case["expected"] == args.expected)
    and (not filters or any(value in case["id"].lower() for value in filters))
  ]
  if not cases:
    print("No validation cases matched.")
    return 2
  if args.list:
    for index, case in enumerate(cases, 1):
      window = case.get("window", ("?", "?"))
      verification = "H" if case.get("human_verified", False) else "-"
      print(
        f"[{index:02d}/{len(cases):02d}] [{verification}] {case['id']}: "
        + f"{case['expected']} {case['source']} {window[0]}-{window[1]}s - {case['scene']}"
      )
    return 0

  missing = 0
  groups = group_cases_by_log(cases)
  opened_cases = 0
  opened_logs = 0
  for index, group in enumerate(groups, 1):
    case = group[0]
    route = args.root / case["vehicle_folder"] / Path(case["log"])
    if not route.is_file():
      missing += len(group)
      print(
        f"[{index:02d}/{len(groups):02d}] MISSING {len(group)} cases: {route}",
        flush=True,
      )
      continue
    ids = ", ".join(str(item["id"]) for item in group)
    heading = f"\n[{index:02d}/{len(groups):02d}] {len(group)} cases in one log: {ids}"
    print(heading, flush=True)
    command = [
      sys.executable, str(SIMULATOR),
      "--validation-root", str(args.root),
      "--validation-cases", str(args.cases),
      "--front-model", str(args.model or args.front_model),
      "--corner-model", str(args.model or args.corner_model),
      "--hybrid",
    ]
    for item in group:
      command.extend(("--validation-case", str(item["id"])))
    if args.compare_radard:
      command.append("--compare-radard")
    if args.front_only:
      command.append("--front-only")
    if args.prob is not None:
      command.extend(("--prob", str(args.prob)))
    result = subprocess.run(command, check=False)
    if result.returncode != 0:
      return result.returncode
    opened_logs += 1
    opened_cases += len(group)
  summary = (
    f"\nVisual review complete: {opened_cases}/{len(cases)} labeled windows "
    + f"in {opened_logs}/{len(groups)} unique logs"
  )
  print(summary)
  return int(missing > 0)


if __name__ == "__main__":
  raise SystemExit(main())
