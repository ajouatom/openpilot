#!/usr/bin/env python3
"""Build current-radard teacher datasets from folders of local route logs."""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.selfdrive.carrot.radar_lead_simulator import (
  CurrentRadardTeacher,
  ManualLabels,
  current_cutin_track_ids,
  export_training_dataset,
  load_frames,
)


RLOG_PATTERN = re.compile(r"^rlog(?:\.\d+)?\.(?:zst|bz2)$", re.IGNORECASE)


def discover_logs(roots: list[Path]) -> list[Path]:
  return sorted({
    path.resolve()
    for root in roots
    for path in root.rglob("*")
    if path.is_file() and RLOG_PATTERN.match(path.name)
  })


def route_key(path: Path) -> str:
  return path.parent.name.rsplit("--", 1)[0]


def evenly_spaced(items: list[Path], count: int) -> list[Path]:
  if count <= 0 or len(items) <= count:
    return items
  indices = {round(index * (len(items) - 1) / (count - 1)) for index in range(count)} if count > 1 else {len(items) // 2}
  return [items[index] for index in sorted(indices)]


def select_logs(logs: list[Path], per_route: int) -> list[Path]:
  by_route: dict[str, list[Path]] = {}
  for path in logs:
    by_route.setdefault(route_key(path), []).append(path)
  return [
    path
    for key in sorted(by_route)
    for path in evenly_spaced(sorted(by_route[key]), per_route)
  ]


def dataset_name(path: Path) -> str:
  stem = re.sub(r"[^A-Za-z0-9_.-]+", "_", path.parent.name)
  return f"{stem}_{path.name}.teacher.csv.gz"


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Create current-radard P0/P1 teacher datasets from route folders")
  parser.add_argument("roots", nargs="+", type=Path, help="route root folders")
  parser.add_argument("--output-dir", required=True, type=Path)
  parser.add_argument("--per-route", type=int, default=2, help="evenly sampled segments per route; 0 means all")
  parser.add_argument("--max-logs", type=int, default=0, help="evenly cap selected logs; 0 means no cap")
  parser.add_argument("--force", action="store_true", help="rebuild existing datasets")
  return parser.parse_args()


def main() -> int:
  args = parse_args()
  missing = [root for root in args.roots if not root.is_dir()]
  if missing:
    raise SystemExit(f"route folder does not exist: {missing[0]}")
  args.output_dir.mkdir(parents=True, exist_ok=True)

  discovered = discover_logs(args.roots)
  selected = select_logs(discovered, args.per_route)
  selected = evenly_spaced(selected, args.max_logs)
  print(f"discovered {len(discovered)} logs; selected {len(selected)} across {len({route_key(p) for p in selected})} routes")
  manifest_path = args.output_dir / "manifest.json"
  manifest: dict[str, Any] = {
    "version": 1,
    "teacher": CurrentRadardTeacher.name,
    "roots": [str(root.resolve()) for root in args.roots],
    "per_route": args.per_route,
    "max_logs": args.max_logs,
    "datasets": [],
    "errors": [],
  }

  for index, log_path in enumerate(selected, 1):
    output_path = args.output_dir / dataset_name(log_path)
    print(f"[{index}/{len(selected)}] {log_path.parent.name}/{log_path.name}", flush=True)
    try:
      if output_path.is_file() and not args.force:
        stats: dict[str, Any] = {"cached": True}
      else:
        frames = load_frames(log_path)
        cutin_ids = current_cutin_track_ids(log_path, frames)
        teacher = CurrentRadardTeacher(frames, cutin_ids)
        stats = export_training_dataset(output_path, frames, ManualLabels(), teacher=teacher)
        stats["frames"] = len(frames)
        stats["cutin_frames"] = sum(bool(ids) for ids in cutin_ids)
      manifest["datasets"].append({
        "route": route_key(log_path),
        "log": str(log_path),
        "dataset": str(output_path),
        "stats": stats,
      })
    except Exception as exc:
      print(f"  ERROR: {exc}", flush=True)
      manifest["errors"].append({"log": str(log_path), "error": repr(exc)})
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")

  print(f"manifest written: {manifest_path}")
  print(f"completed {len(manifest['datasets'])}; errors {len(manifest['errors'])}")
  return 0 if not manifest["errors"] else 1


if __name__ == "__main__":
  raise SystemExit(main())
