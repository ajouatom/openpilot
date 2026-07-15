#!/usr/bin/env python3
"""Export fused radar objects with separate lead and cut-in labels."""

from __future__ import annotations

import argparse
import csv
import gzip
import json
import re
import sys
from dataclasses import replace
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.selfdrive.carrot.radar_lead_model import (
  MODEL_FEATURE_NAMES,
  RadarLeadContext,
  RadarLeadFeatureBuilder,
  VisionLeadContext,
  object_contains_track,
)
from openpilot.selfdrive.carrot.radar_lead_simulator import (
  CurrentRadardTeacher,
  ManualLabels,
  RadarFrame,
  candidate_track_id,
  current_cutin_track_ids,
  load_frames,
)
from openpilot.selfdrive.carrot.radar_object_fusion import RadarObjectFusion


RLOG_PATTERN = re.compile(r"^rlog(?:\.\d+)?\.(?:zst|bz2)$", re.IGNORECASE)
DEFAULT_ANNOTATIONS = Path(__file__).with_name("radar_lead_annotations.json")
METADATA_COLUMNS = (
  "frame", "time_s", "object_id", "aliases", "front_track_id", "corner_track_id", "scc_track_id",
  "lead_label", "cutin_label", "external_label",
  "lead_weight", "cutin_weight", "external_weight",
  "lead_source", "cutin_source", "external_source",
)


def _context(frame: RadarFrame) -> RadarLeadContext:
  model_leads = tuple(
    VisionLeadContext(
      lead.probability, lead.x - 1.52, -lead.y, lead.v, lead.a,
      lead.x_std, lead.y_std, lead.v_std,
    )
    for lead in frame.model_leads[:1]
  )
  return RadarLeadContext(
    frame.mono_time_s, frame.v_ego, frame.path, frame.lane_lines, frame.lane_probs, model_leads,
  )


def _role_targets(
  frame_index: int, labels: ManualLabels, teacher: CurrentRadardTeacher, role: str,
) -> tuple[set[int], str]:
  selection = teacher.select(teacher.frames[frame_index], frame_index)
  candidates = {"leadOne": selection.lead_one, "leadTwo": selection.lead_two}
  is_manual, track_id = labels.get(frame_index, role)
  if not is_manual:
    track_id = candidate_track_id(candidates[role])
  return ({track_id} if track_id is not None else set()), "manual" if is_manual else "teacher"


def _cutin_targets(
  frame_index: int, labels: ManualLabels, weak_cutin_ids: list[set[int]],
) -> tuple[set[int], str]:
  is_manual, track_id = labels.get(frame_index, "cutin")
  if is_manual:
    return ({track_id} if track_id is not None else set()), "manual"
  return set(weak_cutin_ids[frame_index]), "teacher"


def apply_annotations(
  labels: ManualLabels, frames: list[RadarFrame], log_path: Path, annotations_path: Path,
) -> int:
  if not annotations_path.is_file():
    return 0
  payload = json.loads(annotations_path.read_text(encoding="utf-8"))
  normalized_log = f"{log_path.parent.name}/{log_path.name}".replace("\\", "/")
  entries = next((value for key, value in payload.get("logs", {}).items() if normalized_log.endswith(key)), [])
  applied = 0
  for entry in entries:
    start_s = float(entry.get("start_s", 0.0))
    end_s = float(entry.get("end_s", start_s))
    for frame_index, frame in enumerate(frames):
      if not start_s <= frame.time_s <= end_s:
        continue
      for role in ("leadOne", "leadTwo", "cutin"):
        if role not in entry or labels.get(frame_index, role)[0]:
          continue
        value = entry[role]
        labels.set(frame_index, role, None if value is None else int(value))
        applied += 1
  return applied


def _targets_available(targets: set[int], samples: tuple[Any, ...]) -> bool:
  return not targets or all(any(object_contains_track(sample.radar_object, track_id) for sample in samples) for track_id in targets)


def export_fused_dataset(
  log_path: Path,
  output_path: Path,
  labels: ManualLabels | None = None,
  include_scc: bool = False,
  front_only: bool = False,
  annotations_path: Path = DEFAULT_ANNOTATIONS,
) -> dict[str, int]:
  frames = load_frames(log_path)
  if front_only:
    frames = [replace(frame, points=tuple(point for point in frame.points if point.source == "frontRadar")) for frame in frames]
  labels = labels or ManualLabels.load(log_path.with_name(log_path.name + ".lead-labels.json"), len(frames))
  annotation_count = apply_annotations(labels, frames, log_path, annotations_path)
  weak_cutin_ids = current_cutin_track_ids(log_path, frames, ("front",) if front_only else ("corner", "front"))
  teacher = CurrentRadardTeacher(frames, weak_cutin_ids)
  fusion = RadarObjectFusion(include_scc=include_scc)
  feature_builder = RadarLeadFeatureBuilder()
  output_path.parent.mkdir(parents=True, exist_ok=True)
  opener = gzip.open if output_path.suffix.lower() == ".gz" else open
  stats = {
    "frames": len(frames), "groups": 0, "rows": 0,
    "lead_positives": 0, "cutin_positives": 0, "external_positives": 0,
    "manual_lead_groups": 0, "manual_cutin_groups": 0, "manual_external_groups": 0,
    "invalid_lead_groups": 0, "invalid_cutin_groups": 0, "invalid_external_groups": 0,
    "annotations": annotation_count,
  }
  with opener(output_path, "wt", newline="", encoding="utf-8") as output:
    writer = csv.DictWriter(output, fieldnames=[*METADATA_COLUMNS, *MODEL_FEATURE_NAMES])
    writer.writeheader()
    for frame_index, frame in enumerate(frames):
      objects = tuple(
        obj for obj in fusion.update(frame.mono_time_s, frame.points)
        if 0.75 < obj.d_rel < 160.0 and abs(obj.y_rel) < 12.0
      )
      samples = feature_builder.update(_context(frame), objects)
      if not samples:
        continue
      lead_targets, lead_source = _role_targets(frame_index, labels, teacher, "leadOne")
      external_targets, external_source = _role_targets(frame_index, labels, teacher, "leadTwo")
      cutin_targets, cutin_source = _cutin_targets(frame_index, labels, weak_cutin_ids)
      # External is the radar-only leadTwo role. Keep it distinct from both the
      # vision-matched leadOne role and the dedicated cut-in role.
      external_targets -= lead_targets | cutin_targets
      lead_valid = _targets_available(lead_targets, samples)
      cutin_valid = _targets_available(cutin_targets, samples)
      external_valid = _targets_available(external_targets, samples)
      stats["invalid_lead_groups"] += int(not lead_valid)
      stats["invalid_cutin_groups"] += int(not cutin_valid)
      stats["invalid_external_groups"] += int(not external_valid)
      stats["manual_lead_groups"] += int(lead_source == "manual")
      stats["manual_cutin_groups"] += int(cutin_source == "manual")
      stats["manual_external_groups"] += int(external_source == "manual")
      if not lead_valid and not cutin_valid and not external_valid:
        continue
      stats["groups"] += 1
      for sample in samples:
        obj = sample.radar_object
        lead_label = int(lead_valid and any(object_contains_track(obj, target) for target in lead_targets))
        cutin_label = int(cutin_valid and any(object_contains_track(obj, target) for target in cutin_targets))
        external_label = int(external_valid and any(object_contains_track(obj, target) for target in external_targets))
        # Missing cut-ins in the heuristic teacher are weak negatives. Video-confirmed manual
        # labels carry enough weight to correct both teacher false positives and false negatives.
        lead_weight = 6.0 if lead_source == "manual" else (1.0 if lead_valid else 0.0)
        cutin_weight = (
          10.0 if cutin_source == "manual" else
          (1.0 if cutin_label else 0.08) if cutin_valid else 0.0
        )
        external_weight = (
          8.0 if external_source == "manual" else
          (1.0 if external_label else 0.25) if external_valid else 0.0
        )
        row: dict[str, Any] = {
          "frame": frame_index,
          "time_s": f"{frame.time_s:.3f}",
          "object_id": sample.object_id,
          "aliases": ";".join(sample.aliases),
          "front_track_id": "" if obj.front_track_id is None else obj.front_track_id,
          "corner_track_id": "" if obj.corner_track_id is None else obj.corner_track_id,
          "scc_track_id": "" if obj.scc_track_id is None else obj.scc_track_id,
          "lead_label": lead_label,
          "cutin_label": cutin_label,
          "external_label": external_label,
          "lead_weight": f"{lead_weight:.3f}",
          "cutin_weight": f"{cutin_weight:.3f}",
          "external_weight": f"{external_weight:.3f}",
          "lead_source": lead_source,
          "cutin_source": cutin_source,
          "external_source": external_source,
        }
        row.update({name: f"{value:.6g}" for name, value in zip(MODEL_FEATURE_NAMES, sample.values, strict=True)})
        writer.writerow(row)
        stats["rows"] += 1
        stats["lead_positives"] += lead_label
        stats["cutin_positives"] += cutin_label
        stats["external_positives"] += external_label
  return stats


def _discover(roots: list[Path]) -> list[Path]:
  return sorted({
    path.resolve() for root in roots for path in ([root] if root.is_file() else root.rglob("*"))
    if path.is_file() and RLOG_PATTERN.match(path.name)
  })


def _evenly_spaced(items: list[Path], count: int) -> list[Path]:
  if count <= 0 or len(items) <= count:
    return items
  indices = {round(index * (len(items) - 1) / (count - 1)) for index in range(count)} if count > 1 else {len(items) // 2}
  return [items[index] for index in sorted(indices)]


def _dataset_name(path: Path) -> str:
  parent = re.sub(r"[^A-Za-z0-9_.-]+", "_", path.parent.name)
  return f"{parent}_{path.name}.fused.csv.gz"


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Build fused lead/cut-in datasets from rlogs")
  parser.add_argument("roots", nargs="+", type=Path)
  parser.add_argument("--output-dir", required=True, type=Path)
  parser.add_argument("--max-logs", type=int, default=0)
  parser.add_argument("--force", action="store_true")
  parser.add_argument("--fusion-scc", action="store_true")
  parser.add_argument("--front-only", action="store_true", help="augment/train without corner or SCC points")
  parser.add_argument("--annotations", type=Path, default=DEFAULT_ANNOTATIONS)
  return parser.parse_args()


def main() -> int:
  args = parse_args()
  logs = _evenly_spaced(_discover(args.roots), args.max_logs)
  args.output_dir.mkdir(parents=True, exist_ok=True)
  manifest: dict[str, Any] = {"version": 2, "datasets": [], "errors": []}
  print(f"selected {len(logs)} logs", flush=True)
  for index, log_path in enumerate(logs, 1):
    output = args.output_dir / _dataset_name(log_path)
    print(f"[{index}/{len(logs)}] {log_path.parent.name}/{log_path.name}", flush=True)
    try:
      if output.is_file() and not args.force:
        stats: dict[str, Any] = {"cached": True}
      else:
        stats = export_fused_dataset(
          log_path, output, include_scc=args.fusion_scc, front_only=args.front_only,
          annotations_path=args.annotations,
        )
      manifest["datasets"].append({"log": str(log_path), "dataset": str(output), "stats": stats})
    except Exception as exc:
      print(f"  ERROR: {exc}", flush=True)
      manifest["errors"].append({"log": str(log_path), "error": repr(exc)})
    (args.output_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")
  print(f"completed {len(manifest['datasets'])}; errors {len(manifest['errors'])}")
  return 0 if not manifest["errors"] else 1


if __name__ == "__main__":
  raise SystemExit(main())
