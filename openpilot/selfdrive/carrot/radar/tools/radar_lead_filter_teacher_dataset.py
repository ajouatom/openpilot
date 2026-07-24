#!/usr/bin/env python3
"""Relabel fused radar rows from the production decision filter with same-object lookback."""

from __future__ import annotations

import argparse
import csv
import gzip
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[5]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.selfdrive.carrot.radar.radar_lead_model import (
  CUTIN_TEMPORAL_THRESHOLD_MAX,
  MODEL_FEATURE_NAMES,
  RadarLeadDecisionFilter,
  RadarLeadFeatures,
  RadarLeadModel,
)
from openpilot.selfdrive.carrot.radar.radar_object_fusion import FusedRadarObject


def _number(row: dict[str, str], name: str, default: float = 0.0) -> float:
  try:
    return float(row.get(name, default) or default)
  except (TypeError, ValueError):
    return default


def _optional_int(row: dict[str, str], name: str) -> int | None:
  value = row.get(name, "")
  return int(value) if value not in (None, "") else None


def _aliases(row: dict[str, str]) -> tuple[str, ...]:
  aliases = tuple(value for value in row.get("aliases", "").split(";") if value)
  return aliases or (row["object_id"],)


def _sample(row: dict[str, str]) -> RadarLeadFeatures:
  front_track_id = _optional_int(row, "front_track_id")
  corner_track_id = _optional_int(row, "corner_track_id")
  scc_track_id = _optional_int(row, "scc_track_id")
  front_present = _number(row, "front_d_present") > 0.5
  corner_present = _number(row, "corner_d_present") > 0.5
  if _number(row, "corner180") > 0.5:
    lateral_source = "corner180"
  elif _number(row, "corner235") > 0.5:
    lateral_source = "corner235"
  elif front_track_id is not None:
    lateral_source = "front"
  else:
    lateral_source = "scc"
  distance_source = "front" if front_present else "corner" if corner_present else "scc"
  obj = FusedRadarObject(
    object_id=row["object_id"],
    d_rel=_number(row, "d_rel"),
    y_rel=_number(row, "y_rel"),
    v_rel=_number(row, "v_rel"),
    a_rel=_number(row, "a_rel"),
    yv_rel=_number(row, "yv_rel"),
    v_lead=_number(row, "v_lead"),
    front_track_id=front_track_id,
    corner_track_id=corner_track_id,
    scc_track_id=scc_track_id,
    front_d_rel=_number(row, "front_d_rel") if front_present else None,
    corner_d_rel=_number(row, "corner_d_rel") if corner_present else None,
    front_y_rel=_number(row, "front_y_rel") if _number(row, "front_y_present") > 0.5 else None,
    corner_y_rel=_number(row, "corner_y_rel") if _number(row, "corner_y_present") > 0.5 else None,
    front_v_rel=_number(row, "front_v_rel") if _number(row, "front_v_present") > 0.5 else None,
    corner_v_rel=_number(row, "corner_v_rel") if _number(row, "corner_v_present") > 0.5 else None,
    distance_source=distance_source,
    lateral_source=lateral_source,
    match_confidence=_number(row, "match_confidence"),
    pair_age=int(_number(row, "pair_age")),
  )
  return RadarLeadFeatures(
    object_id=row["object_id"],
    aliases=_aliases(row),
    radar_object=obj,
    values=tuple(_number(row, name) for name in MODEL_FEATURE_NAMES),
    track_age=int(_number(row, "track_age")),
    d_path=_number(row, "d_path"),
    d_path_future=_number(row, "future_d_path"),
    in_lane_prob=_number(row, "in_lane_prob"),
  )


def _hard_negative(row: dict[str, str]) -> bool:
  return (
    _number(row, "track_age") >= 5.0
    and 2.0 < _number(row, "d_rel") < 65.0
    and 0.6 < abs(_number(row, "d_path")) < 5.0
    and (_number(row, "h4_present") > 0.5 or _number(row, "h8_present") > 0.5)
  )


def _early_geometry(row: dict[str, str]) -> bool:
  current = abs(_number(row, "d_path"))
  if not (
    _number(row, "track_age") >= 8.0
    and 3.0 < _number(row, "d_rel") < 60.0
    and _number(row, "v_lead") > 1.0
    and 1.15 < current < 4.8
    and _number(row, "h4_present") > 0.5
    and _number(row, "h8_present") > 0.5
    and _number(row, "lane1_prob") > 0.40
    and _number(row, "lane2_prob") > 0.40
  ):
    return False
  h4_dt = max(_number(row, "h4_dt"), 1e-3)
  h8_dt = max(_number(row, "h8_dt"), 1e-3)
  h4_inward = (abs(_number(row, "h4_d_path")) - current) / h4_dt
  h8_inward = (abs(_number(row, "h8_d_path")) - current) / h8_dt
  return (
    0.10 < h4_inward < 3.2
    and 0.10 < h8_inward < 3.2
    and abs(h4_inward - h8_inward) < 1.0
    and abs(_number(row, "future_d_path")) < 3.2
  )


def _manual(row: dict[str, str]) -> bool:
  return row.get("cutin_source") in ("manual", "video")


def _relabel(rows: list[dict[str, str]], model: RadarLeadModel, lookback_s: float) -> dict[str, int]:
  decision_filter = RadarLeadDecisionFilter(
    lead_threshold=max(0.5, float(model.thresholds[0])),
    cutin_threshold=max(0.5, min(CUTIN_TEMPORAL_THRESHOLD_MAX, float(model.thresholds[1]))),
    external_threshold=max(0.5, float(model.thresholds[2])),
  )
  frames: dict[int, list[int]] = {}
  for index, row in enumerate(rows):
    frames.setdefault(int(row["frame"]), []).append(index)

  active_events: list[tuple[float, frozenset[str]]] = []
  stats = {"teacher_positive": 0, "early_positive": 0, "manual": 0, "negative": 0}
  for indexes in frames.values():
    samples = tuple(_sample(rows[index]) for index in indexes)
    predictions = model.predict(samples)
    time_s = _number(rows[indexes[0]], "time_s")
    decision = decision_filter.update(time_s, predictions)
    active_aliases = frozenset(
      alias for prediction in decision.cutin_candidates for alias in prediction.features.aliases
    )
    if active_aliases:
      active_events.append((time_s, active_aliases))
    for index, sample in zip(indexes, samples, strict=True):
      row = rows[index]
      if _manual(row):
        stats["manual"] += 1
        continue
      active = bool(active_aliases.intersection(sample.aliases))
      row["cutin_label"] = "1" if active else "0"
      row["cutin_weight"] = "4.000" if active else "0.500" if _hard_negative(row) else "0.080"
      row["cutin_source"] = "filter-teacher" if active else "filter-negative"
      stats["teacher_positive" if active else "negative"] += 1

  if lookback_s <= 0.0:
    return stats
  for event_time, aliases in active_events:
    for row in reversed(rows):
      age = event_time - _number(row, "time_s")
      if age < 0.0:
        continue
      if age > lookback_s:
        break
      if _manual(row) or _number(row, "cutin_label") > 0.5:
        continue
      if not aliases.intersection(_aliases(row)) or not _early_geometry(row):
        continue
      row["cutin_label"] = "1"
      row["cutin_weight"] = f"{1.5 + 1.5 * (1.0 - age / lookback_s):.3f}"
      row["cutin_source"] = "filter-early"
      stats["early_positive"] += 1
      stats["negative"] -= 1
  return stats


def _process(source: Path, destination: Path, model_path: Path, lookback_s: float) -> dict[str, int]:
  with gzip.open(source, "rt", newline="", encoding="utf-8") as input_file:
    reader = csv.DictReader(input_file)
    fieldnames = reader.fieldnames
    if not fieldnames or not set(MODEL_FEATURE_NAMES) <= set(fieldnames):
      raise RuntimeError(f"unsupported fused dataset: {source}")
    rows = list(reader)
  stats = _relabel(rows, RadarLeadModel(model_path), lookback_s)
  destination.parent.mkdir(parents=True, exist_ok=True)
  with gzip.open(destination, "wt", newline="", encoding="utf-8", compresslevel=4) as output_file:
    writer = csv.DictWriter(output_file, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(rows)
  return stats


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Create production-filter teacher labels for temporal cut-in training")
  parser.add_argument("source", type=Path)
  parser.add_argument("output", type=Path)
  parser.add_argument("--model", required=True, type=Path)
  parser.add_argument("--lookback", type=float, default=0.75)
  parser.add_argument("--skip-existing", action="store_true")
  return parser.parse_args()


def main() -> int:
  args = parse_args()
  sources = sorted(args.source.glob("*.fused.csv.gz")) if args.source.is_dir() else [args.source]
  if not sources:
    raise SystemExit("no fused datasets found")
  totals = {"teacher_positive": 0, "early_positive": 0, "manual": 0, "negative": 0}
  for position, source in enumerate(sources, 1):
    destination = args.output / source.name if len(sources) > 1 or args.output.is_dir() else args.output
    if args.skip_existing and destination.is_file():
      print(f"[{position}/{len(sources)}] skip {source.name}", flush=True)
      continue
    stats = _process(source, destination, args.model, max(0.0, args.lookback))
    for name, value in stats.items():
      totals[name] += value
    print(f"[{position}/{len(sources)}] {source.name} {stats}", flush=True)
  print(f"total {totals}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
