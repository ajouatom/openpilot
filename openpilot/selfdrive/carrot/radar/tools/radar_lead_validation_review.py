#!/usr/bin/env python3
"""Open maintained radar validation logs with the physical dPath predictor."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys


SCRIPT_DIR = Path(__file__).resolve().parent
CARROT_ROOT = SCRIPT_DIR.parents[1]
DEFAULT_CASES = CARROT_ROOT / "cluster" / "cutin_validation_cases.json"
SIMULATOR_MODULE = "openpilot.selfdrive.carrot.radar.tools.radar_lead_simulator"


def group_cases_by_log(cases: list[dict]) -> list[list[dict]]:
  groups: dict[tuple[str, str], list[dict]] = {}
  for case in cases:
    key = (str(case["vehicle_folder"]), str(case["log"]))
    groups.setdefault(key, []).append(case)
  return list(groups.values())


def simulator_command(
  group: list[dict],
  root: Path,
  cases: Path,
  probability: float | None,
  position: str,
  front_only: bool,
  motion_mode: str | None = None,
  sensitivity: int | None = None,
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
  return command


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Replay validation logs showing only the physical dPath predictor",
  )
  parser.add_argument("--root", type=Path, default=Path(r"W:\routes"))
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
  missing = 0
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
    )
    result = subprocess.run(command, check=False)
    if result.returncode != 0:
      return result.returncode
    opened_logs += 1
    opened_cases += len(group)
  print(
    f"\nVisual review complete: {opened_cases}/{len(cases)} labeled windows "
    + f"in {opened_logs}/{len(groups)} unique logs"
  )
  return int(missing > 0)


if __name__ == "__main__":
  raise SystemExit(main())
