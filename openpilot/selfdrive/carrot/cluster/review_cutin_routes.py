#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys
from typing import Any

from validate_cutin_routes import load_cases, selected_cases


REPO_ROOT = Path(__file__).resolve().parents[4]
REPLAY_WRAPPER = REPO_ROOT / "openpilot/selfdrive/carrot/cluster_replay_usb.py"


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Review maintained cut-in route windows in sequence.")
  parser.add_argument(
    "--root", type=Path,
    default=Path(r"\\DS1821P\openpilot\routes"),
    help="Route log root directory",
  )
  parser.add_argument("--case", action="append", default=[], help="Review cases whose id contains this text")
  parser.add_argument("--start-at", help="Start at the first case whose id contains this text")
  parser.add_argument("--output", choices=("window", "usb", "both"), default="window")
  parser.add_argument("--speed", type=float, default=1.0, help="Replay speed multiplier")
  parser.add_argument("--before", type=float, default=3.0, help="Seconds to include before each validation window")
  parser.add_argument("--after", type=float, default=3.0, help="Seconds to include after each validation window")
  parser.add_argument("--manual", action="store_true", help="Keep each replay open until its window is closed")
  parser.add_argument("--pause-on-cutin", action="store_true", help="Pause when a cut-in becomes active")
  parser.add_argument("--show-recorded-cutins", action="store_true", help="Overlay decisions stored in radarState")
  parser.add_argument("--dry-run", action="store_true", help="Print the sequence without opening replay windows")
  return parser.parse_args()


def cases_from_start(cases: list[dict[str, Any]], start_at: str | None) -> list[dict[str, Any]]:
  if start_at is None:
    return cases
  needle = start_at.lower()
  for index, case in enumerate(cases):
    if needle in case["id"].lower():
      return cases[index:]
  return []


def replay_command(
  case: dict[str, Any],
  root: Path,
  sensitivity: float,
  args: argparse.Namespace,
) -> tuple[list[str], float, float | None]:
  window_start, window_end = case["window"]
  start_s = max(0.0, float(window_start) - max(0.0, args.before))
  end_s = float(window_end) + max(0.0, args.after)
  duration_s = None if args.manual else max(1.0, (end_s - start_s) / args.speed)
  route_path = root / case["vehicle_folder"] / Path(case["log"])

  command = [
    sys.executable,
    str(REPLAY_WRAPPER),
    str(route_path),
    "--output", args.output,
    "--route-overlay", "full",
    "--route-tools", "separate",
    "--start-time", f"{start_s:g}",
    "--speed", f"{args.speed:g}",
    "--cutin-radar-source", case["source"],
    "--cutin-sensitivity", f"{sensitivity:g}",
    "--pause-on-cutin" if args.pause_on_cutin else "--no-pause-on-cutin",
  ]
  if duration_s is not None:
    command.extend(("--duration", f"{duration_s:g}"))
  if case["source"] == "front":
    command.append("--front-radar-only")
  if args.show_recorded_cutins:
    command.append("--show-recorded-cutins")
  return command, start_s, duration_s


def main() -> int:
  args = parse_args()
  if args.speed <= 0.0:
    raise SystemExit("--speed must be greater than zero")

  sensitivity, cases = load_cases()
  cases = cases_from_start(selected_cases(cases, args.case), args.start_at)
  if not cases:
    print("No review cases matched.")
    return 2

  for index, case in enumerate(cases, 1):
    command, start_s, duration_s = replay_command(case, args.root, sensitivity, args)
    duration_text = "manual close" if duration_s is None else f"{duration_s:.1f}s wall time"
    print(
      f"\n[{index:02d}/{len(cases):02d}] {case['id']} "
      + f"expected={case['expected']} window={case['window'][0]:g}-{case['window'][1]:g}s",
      flush=True,
    )
    print(f"  {case['scene']}", flush=True)
    print(f"  starts at {start_s:.1f}s, {duration_text}", flush=True)
    if args.dry_run:
      continue

    route_path = Path(command[2])
    if not route_path.is_file():
      print(f"  MISSING: {route_path}", flush=True)
      continue
    try:
      result = subprocess.run(command, cwd=REPO_ROOT, check=False)
    except KeyboardInterrupt:
      print("\nReview stopped.")
      return 130
    if result.returncode not in (0, 130):
      print(f"  Replay exited with code {result.returncode}", flush=True)

  return 0


if __name__ == "__main__":
  raise SystemExit(main())
