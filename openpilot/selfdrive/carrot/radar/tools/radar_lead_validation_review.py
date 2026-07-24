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
SIMULATOR = SCRIPT_DIR / "radar_lead_simulator.py"


def group_cases_by_log(cases: list[dict]) -> list[list[dict]]:
  """Keep validation windows intact while opening each physical log once."""
  groups: dict[tuple[str, str], list[dict]] = {}
  for case in cases:
    key = (
      str(case["vehicle_folder"]).replace("\\", "/").casefold(),
      str(case["log"]).replace("\\", "/").casefold(),
    )
    groups.setdefault(key, []).append(case)
  return list(groups.values())


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
    "--compare-radard", action="store_true",
    help="also recompute and display the current radard result and graph (slow)",
  )
  parser.add_argument(
    "--front-only", action="store_true",
    help="remove corner-radar points and validate the production front-only path",
  )
  parser.add_argument("--list", action="store_true")
  return parser.parse_args()


def main() -> int:
  args = parse_args()
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
  case_groups = group_cases_by_log(cases)
  if args.list:
    for group in case_groups:
      case = group[0]
      extra = f" (+{len(group) - 1} validation windows)" if len(group) > 1 else ""
      print(f"{case['id']}{extra}: {case['source']} - {case['scene']}")
    return 0

  missing = 0
  for index, group in enumerate(case_groups, 1):
    case = group[0]
    route = args.root / case["vehicle_folder"] / Path(case["log"])
    if not route.is_file():
      missing += 1
      print(f"[{index:02d}/{len(case_groups):02d}] MISSING {case['id']}: {route}", flush=True)
      continue
    grouped_ids = ", ".join(item["id"] for item in group[1:])
    grouped_note = f"  grouped={grouped_ids}" if grouped_ids else ""
    heading = f"\n[{index:02d}/{len(case_groups):02d}] {case['id']}  "
    heading += f"source={case['source']}  {case['scene']}{grouped_note}"
    print(heading, flush=True)
    command = [
      sys.executable, str(SIMULATOR),
      "--validation-case", str(case["id"]),
      "--validation-root", str(args.root),
      "--validation-cases", str(args.cases),
      "--front-model", str(args.model or args.front_model),
      "--corner-model", str(args.model or args.corner_model),
      "--hybrid",
    ]
    if args.compare_radard:
      command.append("--compare-radard")
    if args.front_only:
      command.append("--front-only")
    result = subprocess.run(command, check=False)
    if result.returncode != 0:
      return result.returncode
  duplicate_count = len(cases) - len(case_groups)
  summary = f"\nVisual review complete: {len(case_groups) - missing}/{len(case_groups)} routes opened"
  summary += f" ({duplicate_count} duplicate validation entries skipped)"
  print(summary)
  return int(missing > 0)


if __name__ == "__main__":
  raise SystemExit(main())
