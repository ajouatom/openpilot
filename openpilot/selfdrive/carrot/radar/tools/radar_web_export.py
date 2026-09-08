"""JSON adapter for the same production replay used by the desktop reviewer."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path

from openpilot.selfdrive.carrot.radar.tools import radar_validation_replay as replay


SCHEMA_VERSION = 1


def source_version() -> str:
  digest = hashlib.sha256()
  roots = (replay.CARROT_ROOT / "radar_motion", replay.CARROT_ROOT / "cluster",
           replay.REPO_ROOT / "openpilot/selfdrive/controls/lib")
  files = {Path(__file__), Path(replay.__file__)}
  for root in roots:
    files.update(root.glob("*.py"))
  files.update((replay.REPO_ROOT / "openpilot/cereal").glob("*.capnp"))
  for path in sorted(files):
    digest.update(str(path.relative_to(replay.REPO_ROOT)).encode())
    digest.update(path.read_bytes())
  return digest.hexdigest()[:20]


def finite_json(value):
  if isinstance(value, float):
    return round(value, 5) if math.isfinite(value) else None
  if isinstance(value, dict):
    return {key: finite_json(item) for key, item in value.items()}
  if isinstance(value, (tuple, list)):
    return [finite_json(item) for item in value]
  return value


def export_frames(frames, *, sensor="auto", sensitivity=replay.VALIDATION_DEFAULT_SENSITIVITY):
  if not frames:
    raise ValueError("No radar replay frames were found in this log")
  selected_sensor = replay.preferred_radar_motion_sensor(frames) if sensor == "auto" else sensor
  selector = replay.ProductionDPathSelector(frames, motion_sensor=selected_sensor,
                                          cut_in_sensitivity=sensitivity)
  output = []
  for index, frame in enumerate(frames):
    item = asdict(frame)
    item.pop("mono_time_s", None)
    selection = asdict(selector.select(frame, index))
    item["selection"] = {key: selection[key] for key in ("lead_one", "lead_two", "cutin_diagnostics", "cutin_predecel_candidate")}
    output.append(item)
  return finite_json({
    "schemaVersion": SCHEMA_VERSION,
    "sourceVersion": source_version(),
    "engine": selector.name,
    "sensor": selected_sensor,
    "sensitivity": sensitivity,
    "enableRadarTracks": 2,
    "radarToCamera": replay.RADAR_TO_CAMERA,
    "videoAligned": all(frame.video_time_s is not None for frame in frames),
    "frames": output,
  })


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("log", type=Path)
  parser.add_argument("output", type=Path)
  parser.add_argument("--sensor", choices=("auto", "front", "corner"), default="auto")
  parser.add_argument("--sensitivity", type=int, choices=range(6), default=replay.VALIDATION_DEFAULT_SENSITIVITY)
  args = parser.parse_args()
  payload = export_frames(replay.load_frames(args.log), sensor=args.sensor, sensitivity=args.sensitivity)
  payload["sourceLog"] = args.log.name
  args.output.write_text(json.dumps(payload, separators=(",", ":"), allow_nan=False), encoding="utf-8")


if __name__ == "__main__":
  main()
