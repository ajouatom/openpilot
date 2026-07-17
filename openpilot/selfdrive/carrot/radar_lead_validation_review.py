#!/usr/bin/env python3
"""Replay routes and pause on cut-in, vision-only, or unmatched vision events."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CASES = SCRIPT_DIR / "cluster" / "cutin_validation_cases.json"
DEFAULT_MODEL = SCRIPT_DIR / "models" / "radar_lead_multitask.npz"
SIMULATOR = SCRIPT_DIR / "radar_lead_simulator.py"


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Replay routes and pause on cut-in, vision-only, or unmatched vision events")
  parser.add_argument("--root", type=Path, default=Path(r"W:\routes"), help="route log root")
  parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
  parser.add_argument("--cases", type=Path, default=DEFAULT_CASES)
  parser.add_argument("--case", action="append", default=[], help="case-id substring; repeat to select more")
  parser.add_argument("--expected", choices=("all", "detect", "clear", "stationary"), default="all")
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
  if args.list:
    for case in cases:
      print(f"{case['id']}: {case['source']} - {case['scene']}")
    return 0

  missing = 0
  for index, case in enumerate(cases, 1):
    route = args.root / case["vehicle_folder"] / Path(case["log"])
    if not route.is_file():
      missing += 1
      print(f"[{index:02d}/{len(cases):02d}] MISSING {case['id']}: {route}", flush=True)
      continue
    print(
      f"\n[{index:02d}/{len(cases):02d}] {case['id']}  "
      f"source={case['source']}  {case['scene']}",
      flush=True,
    )
    result = subprocess.run([
      sys.executable, str(SIMULATOR),
      "--validation-case", str(case["id"]),
      "--validation-root", str(args.root),
      "--validation-cases", str(args.cases),
      "--model", str(args.model),
      "--hybrid",
    ], check=False)
    if result.returncode != 0:
      return result.returncode
  print(f"\nVisual review complete: {len(cases) - missing}/{len(cases)} routes opened")
  return int(missing > 0)


if __name__ == "__main__":
  raise SystemExit(main())
