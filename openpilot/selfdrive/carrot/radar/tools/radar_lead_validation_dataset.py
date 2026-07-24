#!/usr/bin/env python3
"""Build strong cut-in labels directly from the maintained route validation cases."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[5]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.selfdrive.carrot.radar.tools.radar_lead_fused_dataset import export_fused_dataset


DEFAULT_CASES = Path(__file__).resolve().parents[2] / "cluster" / "cutin_validation_cases.json"
RLOG_PATTERN = re.compile(r"[^A-Za-z0-9_.-]+")


def validation_annotations(cases: list[dict[str, Any]]) -> dict[str, Any]:
  logs: dict[str, list[dict[str, Any]]] = {}
  for case in cases:
    if case.get("expected") not in ("detect", "clear"):
      continue
    target_track_ids = [int(value) for value in case.get("target_track_ids", ())]
    # Broad detect windows without a confirmed radar ID remain regression cases,
    # but are not precise enough to become object-level training labels.
    if case["expected"] == "detect" and not target_track_ids:
      continue
    start_s, end_s = (float(value) for value in case.get("label_window", case["window"]))
    entry = {
      "start_s": start_s,
      "end_s": end_s,
      "cutin_targets": target_track_ids if case["expected"] == "detect" else [],
      "validation_case": str(case["id"]),
    }
    logs.setdefault(str(case["log"]).replace("\\", "/"), []).append(entry)
  return {"version": 2, "source": "cutin_validation_cases", "logs": logs}


def _dataset_name(case: dict[str, Any]) -> str:
  vehicle = RLOG_PATTERN.sub("_", str(case["vehicle_folder"])).strip("_")
  segment = RLOG_PATTERN.sub("_", str(Path(case["log"]).parent)).strip("_")
  return f"{vehicle}_{segment}.validation.fused.csv.gz"


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Export video-reviewed cut-in validation windows as strong labels")
  parser.add_argument("--root", type=Path, default=Path(r"W:\routes"))
  parser.add_argument("--cases", type=Path, default=DEFAULT_CASES)
  parser.add_argument("--output-dir", required=True, type=Path)
  parser.add_argument("--sensor-mode", choices=("fused", "front", "corner"), default="fused")
  parser.add_argument("--enable-radar-tracks", type=int, default=None)
  parser.add_argument("--include-scc", action="store_true")
  parser.add_argument("--force", action="store_true")
  return parser.parse_args()


def main() -> int:
  args = parse_args()
  payload = json.loads(args.cases.read_text(encoding="utf-8"))
  cases = [case for case in payload["cases"] if case.get("expected") in ("detect", "clear")]
  annotations = validation_annotations(cases)
  args.output_dir.mkdir(parents=True, exist_ok=True)
  annotations_path = args.output_dir / "validation_annotations.json"
  annotations_path.write_text(json.dumps(annotations, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")

  reviewed_logs = set(annotations["logs"])
  unique_logs: dict[str, dict[str, Any]] = {}
  for case in cases:
    normalized_log = str(case["log"]).replace("\\", "/")
    if normalized_log in reviewed_logs:
      unique_logs.setdefault(normalized_log, case)

  manifest: dict[str, Any] = {
    "version": 2,
    "sensor_mode": args.sensor_mode,
    "enable_radar_tracks": args.enable_radar_tracks,
    "include_scc": args.include_scc,
    "datasets": [],
    "errors": [],
  }
  for index, case in enumerate(unique_logs.values(), 1):
    log_path = args.root / str(case["vehicle_folder"]) / Path(case["log"])
    output_path = args.output_dir / _dataset_name(case)
    print(f"[{index}/{len(unique_logs)}] {case['id']} {log_path.parent.name}", flush=True)
    try:
      if not log_path.is_file():
        raise FileNotFoundError(log_path)
      stats = (
        export_fused_dataset(
          log_path, output_path, include_scc=args.include_scc, annotations_path=annotations_path,
          reviewed_cutin_only=True, sensor_mode=args.sensor_mode,
          enable_radar_tracks=args.enable_radar_tracks,
        )
        if args.force or not output_path.is_file()
        else {"cached": True}
      )
      manifest["datasets"].append({"log": str(log_path), "dataset": str(output_path), "stats": stats})
    except Exception as exc:
      print(f"  ERROR: {exc}", flush=True)
      manifest["errors"].append({"log": str(log_path), "error": repr(exc)})
    (args.output_dir / "manifest.json").write_text(
      json.dumps(manifest, indent=2, ensure_ascii=True) + "\n", encoding="utf-8",
    )
  print(f"completed {len(manifest['datasets'])}; errors {len(manifest['errors'])}")
  return int(bool(manifest["errors"]))


if __name__ == "__main__":
  raise SystemExit(main())
