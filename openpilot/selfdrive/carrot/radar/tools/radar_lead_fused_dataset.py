#!/usr/bin/env python3
"""Export source-specific radar objects with separate lead and cut-in labels."""

from __future__ import annotations

import argparse
import csv
import gzip
import json
import re
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[5]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.selfdrive.carrot.radar.radar_lead_model import (
  MODEL_FEATURE_NAMES,
  RadarLeadContext,
  RadarLeadFeatureBuilder,
  VisionLeadContext,
  object_contains_track,
)
from openpilot.selfdrive.carrot.radar.radar_sensor_objects import independent_radar_objects
from openpilot.selfdrive.carrot.radar.tools.radar_lead_simulator import (
  CurrentRadardTeacher,
  ManualLabels,
  RadarFrame,
  candidate_track_id,
  current_cutin_track_ids,
  load_frames,
)
from openpilot.selfdrive.carrot.radar.radar_object_fusion import RadarObjectFusion


RLOG_PATTERN = re.compile(r"^rlog(?:\.\d+)?\.(?:zst|bz2)$", re.IGNORECASE)
DEFAULT_ANNOTATIONS = Path(__file__).resolve().parents[1] / "data" / "radar_lead_annotations.json"
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


def _annotation_entries(log_path: Path, annotations_path: Path) -> list[dict[str, Any]]:
  if not annotations_path.is_file():
    return []
  payload = json.loads(annotations_path.read_text(encoding="utf-8"))
  normalized_log = f"{log_path.parent.name}/{log_path.name}".replace("\\", "/")
  return next((value for key, value in payload.get("logs", {}).items() if normalized_log.endswith(key)), [])


def candidate_cutin_overrides(
  frames: list[RadarFrame], entries: list[dict[str, Any]],
) -> tuple[list[dict[int, bool]], int]:
  overrides: list[dict[int, bool]] = [{} for _ in frames]
  applied = 0
  for entry in entries:
    if "cutin_candidate" not in entry or "cutin_label" not in entry:
      continue
    start_s = float(entry.get("start_s", 0.0))
    end_s = float(entry.get("end_s", start_s))
    track_id = int(entry["cutin_candidate"])
    label = bool(entry["cutin_label"])
    for frame_index, frame in enumerate(frames):
      if start_s <= frame.time_s <= end_s:
        overrides[frame_index][track_id] = label
        applied += 1
  return overrides, applied


def frame_cutin_targets(
  frames: list[RadarFrame], entries: list[dict[str, Any]],
) -> tuple[list[set[int] | None], int]:
  """Return exact manual cut-in targets for fully reviewed frame ranges."""
  targets: list[set[int] | None] = [None for _ in frames]
  applied = 0
  for entry in entries:
    if "cutin_targets" not in entry:
      continue
    start_s = float(entry.get("start_s", 0.0))
    end_s = float(entry.get("end_s", start_s))
    entry_targets = {int(value) for value in entry["cutin_targets"]}
    for frame_index, frame in enumerate(frames):
      if start_s <= frame.time_s <= end_s:
        current_targets = targets[frame_index]
        if not entry_targets or current_targets == set():
          # An explicit reviewed clear window overrides an overlapping broad
          # detection window regardless of annotation order.
          targets[frame_index] = set()
        else:
          targets[frame_index] = entry_targets if current_targets is None else current_targets | entry_targets
        applied += 1
  return targets, applied


def apply_annotations(
  labels: ManualLabels, frames: list[RadarFrame], log_path: Path, annotations_path: Path,
  entries: list[dict[str, Any]] | None = None,
) -> int:
  entries = _annotation_entries(log_path, annotations_path) if entries is None else entries
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


def _object_track_ids(obj: Any) -> set[int]:
  return {
    int(track_id) for track_id in (obj.front_track_id, obj.corner_track_id, obj.scc_track_id)
    if track_id is not None
  }


def _label_aliases(objects: tuple[Any, ...]) -> tuple[frozenset[int], ...]:
  """Build same-vehicle ID groups used only to transfer offline labels."""
  return tuple(
    frozenset(track_ids)
    for obj in objects
    if (track_ids := _object_track_ids(obj))
  )


def _expand_targets(targets: set[int], aliases: tuple[frozenset[int], ...]) -> set[int]:
  expanded = set(targets)
  for target in targets:
    expanded.update(next((group for group in aliases if target in group), ()))
  return expanded


def _translate_targets(
  targets: set[int], aliases: tuple[frozenset[int], ...], samples: tuple[Any, ...],
) -> set[int]:
  available = {
    track_id
    for sample in samples
    for track_id in _object_track_ids(sample.radar_object)
  }
  translated: set[int] = set()
  for target in targets:
    alternatives = next((group for group in aliases if target in group), frozenset((target,)))
    translated.add(
      target if target in available
      else next((track_id for track_id in alternatives if track_id in available), target)
    )
  return translated


def export_fused_dataset(
  log_path: Path,
  output_path: Path,
  labels: ManualLabels | None = None,
  include_scc: bool = False,
  front_only: bool = False,
  annotations_path: Path = DEFAULT_ANNOTATIONS,
  reviewed_cutin_only: bool = False,
  sensor_mode: str = "fused",
  enable_radar_tracks: int | None = None,
) -> dict[str, Any]:
  if front_only:
    sensor_mode = "front"
    include_scc = False
    enable_radar_tracks = 1
  if sensor_mode not in ("fused", "front", "corner"):
    raise ValueError(f"unknown radar sensor mode: {sensor_mode}")
  frames = load_frames(log_path)
  labels = labels or ManualLabels.load(log_path.with_name(log_path.name + ".lead-labels.json"), len(frames))
  annotation_entries = _annotation_entries(log_path, annotations_path)
  annotation_count = apply_annotations(labels, frames, log_path, annotations_path, annotation_entries)
  cutin_overrides, candidate_annotation_count = candidate_cutin_overrides(frames, annotation_entries)
  exact_cutin_targets, target_annotation_count = frame_cutin_targets(frames, annotation_entries)
  annotation_count += candidate_annotation_count + target_annotation_count
  cutin_sources = {
    "front": ("front",),
    "corner": ("corner",),
    "fused": ("corner", "front"),
  }[sensor_mode]
  weak_cutin_ids = current_cutin_track_ids(log_path, frames, cutin_sources)
  teacher = CurrentRadardTeacher(frames, weak_cutin_ids)
  feature_fusion = RadarObjectFusion(include_scc=include_scc) if sensor_mode == "fused" else None
  # This association object never supplies model features. It only translates
  # a reviewed front ID to the corresponding corner ID, or vice versa.
  label_association = RadarObjectFusion(include_scc=True)
  feature_builder = RadarLeadFeatureBuilder()
  output_path.parent.mkdir(parents=True, exist_ok=True)
  opener = gzip.open if output_path.suffix.lower() == ".gz" else open
  stats = {
    "frames": len(frames), "groups": 0, "rows": 0,
    "lead_positives": 0, "cutin_positives": 0, "external_positives": 0,
    "manual_lead_groups": 0, "manual_cutin_groups": 0, "manual_external_groups": 0,
    "invalid_lead_groups": 0, "invalid_cutin_groups": 0, "invalid_external_groups": 0,
    "annotations": annotation_count,
    "sensor_mode": sensor_mode,
  }
  with opener(output_path, "wt", newline="", encoding="utf-8") as output:
    writer = csv.DictWriter(output, fieldnames=[*METADATA_COLUMNS, *MODEL_FEATURE_NAMES])
    writer.writeheader()
    for frame_index, frame in enumerate(frames):
      associated_objects = tuple(label_association.update(frame.mono_time_s, frame.points))
      aliases = _label_aliases(associated_objects)
      if sensor_mode == "fused":
        assert feature_fusion is not None
        raw_objects = tuple(feature_fusion.update(frame.mono_time_s, frame.points))
      else:
        independent = independent_radar_objects(
          frame.points,
          include_scc=include_scc,
          enable_radar_tracks=enable_radar_tracks,
        )
        raw_objects = independent.front if sensor_mode == "front" else independent.corner
      objects = tuple(
        obj for obj in raw_objects
        if 0.75 < obj.d_rel < 160.0 and abs(obj.y_rel) < 12.0
      )
      samples = feature_builder.update(_context(frame), objects)
      # Keep sensor-specific temporal feature history warm before a reviewed range.
      if reviewed_cutin_only and exact_cutin_targets[frame_index] is None:
        continue
      if not samples:
        continue
      lead_targets, lead_source = _role_targets(frame_index, labels, teacher, "leadOne")
      external_targets, external_source = _role_targets(frame_index, labels, teacher, "leadTwo")
      cutin_targets, cutin_source = _cutin_targets(frame_index, labels, weak_cutin_ids)
      frame_cutin_overrides = cutin_overrides[frame_index]
      reviewed_targets = exact_cutin_targets[frame_index]
      if reviewed_targets is not None:
        cutin_targets = set(reviewed_targets)
        cutin_source = "manual"
      lead_targets = _translate_targets(lead_targets, aliases, samples)
      external_targets = _translate_targets(external_targets, aliases, samples)
      cutin_targets = _translate_targets(cutin_targets, aliases, samples)
      frame_cutin_overrides = {
        alias: label
        for track_id, label in frame_cutin_overrides.items()
        for alias in _expand_targets({track_id}, aliases)
      }
      if reviewed_targets is not None:
        reviewed_targets = _translate_targets(set(reviewed_targets), aliases, samples)
      # External is the radar-only leadTwo role. Keep it distinct from both the
      # vision-matched leadOne role and the dedicated cut-in role.
      external_targets -= lead_targets | cutin_targets
      lead_valid = _targets_available(lead_targets, samples)
      override_valid = any(
        object_contains_track(sample.radar_object, track_id)
        for track_id in frame_cutin_overrides for sample in samples
      )
      cutin_valid = (
        _targets_available(reviewed_targets, samples)
        if reviewed_targets is not None
        else _targets_available(cutin_targets, samples) or override_valid
      )
      external_valid = _targets_available(external_targets, samples)
      stats["invalid_lead_groups"] += int(not lead_valid)
      stats["invalid_cutin_groups"] += int(not cutin_valid)
      stats["invalid_external_groups"] += int(not external_valid)
      stats["manual_lead_groups"] += int(lead_source == "manual")
      stats["manual_cutin_groups"] += int(cutin_source == "manual" or bool(frame_cutin_overrides))
      stats["manual_external_groups"] += int(external_source == "manual")
      if not lead_valid and not cutin_valid and not external_valid:
        continue
      stats["groups"] += 1
      for sample in samples:
        obj = sample.radar_object
        lead_label = int(lead_valid and any(object_contains_track(obj, target) for target in lead_targets))
        object_overrides = [
          label for track_id, label in frame_cutin_overrides.items()
          if object_contains_track(obj, track_id)
        ]
        # Video-reviewed candidates are manual ground truth for the existing
        # trainer, but remain candidate-specific instead of replacing the frame.
        row_cutin_source = "manual" if reviewed_targets is not None or object_overrides else cutin_source
        cutin_label = (
          int(any(object_contains_track(obj, target) for target in reviewed_targets))
          if reviewed_targets is not None else
          int(any(object_overrides)) if object_overrides else
          int(cutin_valid and any(object_contains_track(obj, target) for target in cutin_targets))
        )
        external_label = int(external_valid and any(object_contains_track(obj, target) for target in external_targets))
        # Missing cut-ins in the heuristic teacher are weak negatives. Video-confirmed manual
        # labels carry enough weight to correct both teacher false positives and false negatives.
        lead_weight = 6.0 if lead_source == "manual" else (1.0 if lead_valid else 0.0)
        cutin_weight = (
          10.0 if row_cutin_source in ("manual", "video") else
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
          "cutin_source": row_cutin_source,
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
  return f"{parent}_{path.name}.csv.gz"


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Build source-specific lead/cut-in datasets from rlogs")
  parser.add_argument("roots", nargs="+", type=Path)
  parser.add_argument("--output-dir", required=True, type=Path)
  parser.add_argument("--max-logs", type=int, default=0)
  parser.add_argument("--force", action="store_true")
  parser.add_argument("--fusion-scc", action="store_true")
  parser.add_argument("--front-only", action="store_true", help="augment/train without corner or SCC points")
  parser.add_argument("--sensor-mode", choices=("fused", "front", "corner"), default="fused")
  parser.add_argument(
    "--enable-radar-tracks", type=int, default=None,
    help="front mode SCC policy matching the device EnableRadarTracks value",
  )
  parser.add_argument("--reviewed-cutin-only", action="store_true", help="write only fully reviewed cut-in frames")
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
          annotations_path=args.annotations, reviewed_cutin_only=args.reviewed_cutin_only,
          sensor_mode=args.sensor_mode, enable_radar_tracks=args.enable_radar_tracks,
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
