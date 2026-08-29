#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[4]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from cluster_route_replay import (
  ROUTE_CUTIN_SENSITIVITY_ENV,
  RouteLogParser,
  load_openpilot_log_schema,
  normalize_route_frames,
)


CASES_PATH = Path(__file__).with_name("cutin_validation_cases.json")


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Run the maintained route cut-in regression set.")
  parser.add_argument(
    "--root", type=Path,
    default=Path(r"\\DS1821P\openpilot\routes"),
    help="Route log root directory",
  )
  parser.add_argument("--case", action="append", default=[], help="Run cases whose id contains this text")
  parser.add_argument("--list", action="store_true", help="List selected cases without parsing logs")
  return parser.parse_args()


def load_cases() -> tuple[float, list[dict[str, Any]]]:
  manifest = json.loads(CASES_PATH.read_text(encoding="utf-8"))
  return float(manifest["sensitivity"]), list(manifest["cases"])


def selected_cases(cases: list[dict[str, Any]], filters: list[str]) -> list[dict[str, Any]]:
  cases = [case for case in cases if case["expected"] in ("detect", "clear")]
  if not filters:
    return cases
  lowered = [value.lower() for value in filters]
  return [case for case in cases if any(value in case["id"].lower() for value in lowered)]


def current_cutins(frames: list[Any], start_s: float, end_s: float) -> list[tuple[float, Any]]:
  results: list[tuple[float, Any]] = []
  seen: set[tuple[int | None, int]] = set()
  for frame in frames:
    if not start_s <= frame.t <= end_s:
      continue
    for vehicle in frame.detected_vehicles:
      if not vehicle.cut_in or vehicle.label != "NEW CUT-IN":
        continue
      key = (vehicle.radar_track_id, round(frame.t * 10))
      if key not in seen:
        seen.add(key)
        results.append((frame.t, vehicle))
  return results


def main() -> int:
  args = parse_args()
  sensitivity, cases = load_cases()
  cases = selected_cases(cases, args.case)
  if not cases:
    print("No validation cases matched.")
    return 2

  if args.list:
    for case in cases:
      print(f"{case['id']}: {case['expected']} {case['window']} {case['source']} - {case['scene']}")
    return 0

  os.environ[ROUTE_CUTIN_SENSITIVITY_ENV] = str(sensitivity)
  schema = load_openpilot_log_schema()
  failures = 0
  for index, case in enumerate(cases, 1):
    route_path = args.root / case["vehicle_folder"] / Path(case["log"])
    if not route_path.is_file():
      failures += 1
      print(f"[{index:02d}/{len(cases):02d}] MISSING {case['id']}: {route_path}", flush=True)
      continue

    parser = RouteLogParser(cutin_radar_source=case["source"])
    parser.front_radar_only = case["source"] == "front"
    parser.show_recorded_cutins = False
    frames = parser.parse_file(route_path, schema)
    if frames:
      frames = normalize_route_frames(frames, min(frame.t for frame in frames))
    detections = current_cutins(frames, *case["window"])
    passed = bool(detections) == (case["expected"] == "detect")
    failures += not passed

    detail = "no detection"
    if detections:
      first_t, vehicle = detections[0]
      detail = (
        f"first={first_t:.2f}s id={vehicle.radar_track_id} "
        + f"d={vehicle.longitudinal_m:.1f}m"
      )
    print(
      f"[{index:02d}/{len(cases):02d}] {'PASS' if passed else 'FAIL'} "
      + f"{case['id']}: {detail}",
      flush=True,
    )

  print(f"\n{len(cases) - failures}/{len(cases)} passed")
  return 1 if failures else 0


if __name__ == "__main__":
  raise SystemExit(main())
