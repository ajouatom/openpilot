#!/usr/bin/env python3
"""Open maintained radar validation logs with the physical dPath predictor."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile


SCRIPT_DIR = Path(__file__).resolve().parent
CARROT_ROOT = SCRIPT_DIR.parents[1]
DEFAULT_CASES = CARROT_ROOT / "cluster" / "cutin_validation_cases.json"
SIMULATOR_MODULE = "openpilot.selfdrive.carrot.radar.tools.radar_lead_simulator"
VALIDATION_PRELOAD_AHEAD = 5
ROUTE_SCHEMA_CACHE_ENV = "CARROT_ROUTE_SCHEMA_CACHE_DIR"


def group_cases_by_log(cases: list[dict]) -> list[list[dict]]:
  groups: dict[tuple[str, str], list[dict]] = {}
  for case in cases:
    key = (str(case["vehicle_folder"]), str(case["log"]))
    groups.setdefault(key, []).append(case)
  return list(groups.values())


def rolling_preload_indexes(
  current_index: int,
  route_available: list[bool],
  scheduled_indexes: set[int],
  preload_ahead: int = VALIDATION_PRELOAD_AHEAD,
) -> list[int]:
  """Return unscheduled indexes needed to keep a rolling look-ahead full."""
  window = [
    candidate_index
    for candidate_index in range(current_index + 1, len(route_available))
    if route_available[candidate_index]
  ][:preload_ahead]
  return [
    candidate_index
    for candidate_index in window
    if candidate_index not in scheduled_indexes
  ]


def simulator_command(
  group: list[dict],
  root: Path,
  cases: Path,
  probability: float | None,
  position: str,
  front_only: bool,
  motion_mode: str | None = None,
  sensitivity: int | None = None,
  enable_radar_tracks: int = 2,
  cache_dir: Path | None = None,
  preload_only: bool = False,
  consume_cache: bool = False,
) -> list[str]:
  command = [
    sys.executable,
    "-m",
    SIMULATOR_MODULE,
    "--validation-root",
    str(root),
    "--validation-cases",
    str(cases),
    "--review-position",
    position,
    "--exit-at-end",
    "--enable-radar-tracks",
    str(enable_radar_tracks),
  ]
  if probability is not None:
    command.extend(("--prob", str(probability)))
  if sensitivity is not None:
    command.extend(("--sensitivity", str(sensitivity)))
  for item in group:
    command.extend(("--validation-case", str(item["id"])))
  if front_only:
    command.append("--front-only")
  elif motion_mode is not None:
    command.extend(("--motion-mode", motion_mode))
  if cache_dir is not None:
    command.extend(("--cache-dir", str(cache_dir)))
  if preload_only:
    command.append("--preload-only")
  if consume_cache:
    command.append("--consume-cache")
  return command


def simulator_environment(cache_dir: Path, group_index: int) -> dict[str, str]:
  environment = os.environ.copy()
  environment[ROUTE_SCHEMA_CACHE_ENV] = str(
    cache_dir / "schema" / f"log-{group_index:03d}",
  )
  return environment


def _stop_preloads(preloads: dict[int, subprocess.Popen]) -> None:
  for process in preloads.values():
    if process.poll() is None:
      process.terminate()
  for process in preloads.values():
    try:
      process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
      process.kill()
      process.wait()


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Replay validation logs showing only the physical dPath predictor",
  )
  parser.add_argument(
    "--root", type=Path,
    default=Path(r"\\DS1821P\openpilot\routes"),
  )
  parser.add_argument("--cases", type=Path, default=DEFAULT_CASES)
  parser.add_argument("--case", action="append", default=[])
  parser.add_argument(
    "--expected",
    choices=("all", "detect", "clear", "stationary"),
    default="all",
  )
  parser.add_argument(
    "--prob",
    type=float,
    default=None,
    help=(
      "advanced one-run probability threshold override"
    ),
  )
  parser.add_argument(
    "--sensitivity",
    type=int,
    default=None,
    help=(
      "Carrot Radar CUT-IN sensitivity 0..5; otherwise use the validation "
      + "UI's saved level"
    ),
  )
  parser.add_argument(
    "--enable-radar-tracks",
    type=int,
    choices=range(-2, 4),
    default=2,
    help="EnableRadarTracks value used by the dPath replay (default: 2)",
  )
  parser.add_argument("--front-only", action="store_true")
  parser.add_argument(
    "--motion-mode",
    choices=("normal", "front"),
    default=None,
    help="initial processing mode; otherwise use the mode saved by the replay UI",
  )
  parser.add_argument("--list", action="store_true")
  return parser.parse_args()


def main() -> int:
  args = parse_args()
  if args.prob is not None and not 0.0 <= args.prob <= 1.0:
    raise SystemExit("--prob must be between 0.00 and 1.00")
  if args.sensitivity is not None and not 0 <= args.sensitivity <= 5:
    raise SystemExit("--sensitivity must be between 0 and 5")
  if args.front_only and args.motion_mode not in (None, "front"):
    raise SystemExit("--front-only conflicts with --motion-mode normal")
  payload = json.loads(args.cases.read_text(encoding="utf-8"))
  filters = tuple(value.lower() for value in args.case)
  cases = [
    case for case in payload.get("cases", ())
    if (args.expected == "all" or case["expected"] == args.expected)
    and (not filters or any(value in str(case["id"]).lower() for value in filters))
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
        + f"{case['expected']} {case['source']} "
        + f"{window[0]}-{window[1]}s - {case['scene']}"
      )
    return 0

  groups = group_cases_by_log(cases)
  routes = [
    args.root / group[0]["vehicle_folder"] / Path(group[0]["log"])
    for group in groups
  ]
  route_available = [route.is_file() for route in routes]
  missing = 0
  failed_logs = 0
  opened_cases = 0
  opened_logs = 0
  with tempfile.TemporaryDirectory(prefix="carrot-radar-review-") as directory:
    cache_dir = Path(directory)
    preloads: dict[int, subprocess.Popen] = {}

    def start_next_preloads(current_index: int) -> None:
      next_indexes = rolling_preload_indexes(
        current_index,
        route_available,
        set(preloads),
      )
      for candidate_index in next_indexes:
        command = simulator_command(
          groups[candidate_index],
          args.root,
          args.cases,
          args.prob,
          f"{candidate_index + 1}/{len(groups)}",
          args.front_only,
          args.motion_mode,
          args.sensitivity,
          args.enable_radar_tracks,
          cache_dir=cache_dir,
          preload_only=True,
        )
        preloads[candidate_index] = subprocess.Popen(
          command,
          env=simulator_environment(cache_dir, candidate_index),
          stdout=subprocess.DEVNULL,
          stderr=subprocess.DEVNULL,
        )

    try:
      print(
        f"Rolling preload: keeping the next {VALIDATION_PRELOAD_AHEAD} logs ready.",
        flush=True,
      )
      for group_index, (group, route) in enumerate(zip(groups, routes, strict=True)):
        index = group_index + 1
        if not route_available[group_index]:
          missing += len(group)
          print(
            f"[{index:02d}/{len(groups):02d}] MISSING {len(group)} cases: {route}",
            flush=True,
          )
          continue
        preload = preloads.pop(group_index, None)
        if preload is not None:
          returncode = preload.wait()
          if returncode != 0:
            print(
              f"[{index:02d}/{len(groups):02d}] preload failed; loading normally",
              flush=True,
            )
        start_next_preloads(group_index)
        ids = ", ".join(str(item["id"]) for item in group)
        print(
          f"\n[{index:02d}/{len(groups):02d}] {len(group)} cases in one log: {ids}",
          flush=True,
        )
        command = simulator_command(
          group,
          args.root,
          args.cases,
          args.prob,
          f"{index}/{len(groups)}",
          args.front_only,
          args.motion_mode,
          args.sensitivity,
          args.enable_radar_tracks,
          cache_dir=cache_dir,
          consume_cache=True,
        )
        result = subprocess.run(
          command,
          check=False,
          env=simulator_environment(cache_dir, group_index),
        )
        if result.returncode != 0:
          failed_logs += 1
          print(
            f"[{index:02d}/{len(groups):02d}] viewer exited with "
            + f"code {result.returncode}; continuing with the next log",
            flush=True,
          )
          continue
        opened_logs += 1
        opened_cases += len(group)
    finally:
      _stop_preloads(preloads)
  print(
    f"\nVisual review complete: {opened_cases}/{len(cases)} labeled windows "
    + f"in {opened_logs}/{len(groups)} unique logs"
  )
  return int(missing > 0 or failed_logs > 0)


if __name__ == "__main__":
  raise SystemExit(main())
