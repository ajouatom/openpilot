#!/usr/bin/env python3
"""Create anticipatory cut-in labels from later current-radard confirmations."""

from __future__ import annotations

import argparse
import bisect
import csv
import gzip
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable


DEFAULT_LOOKBACK_S = 0.8
ANTICIPATORY_WEIGHT = 0.8


@dataclass
class AugmentStats:
  rows: int = 0
  original_positives: int = 0
  anticipatory_positives: int = 0
  manual_negatives_preserved: int = 0
  earliest_lead_s: float = 0.0
  output_rows: int = 0


def _number(row: dict[str, str], name: str, default: float = 0.0) -> float:
  try:
    value = float(row.get(name, default) or default)
  except (TypeError, ValueError):
    return default
  return value if math.isfinite(value) else default


def _aliases(row: dict[str, str]) -> tuple[str, ...]:
  aliases = tuple(value for value in row.get("aliases", "").split(";") if value)
  return aliases or (row.get("object_id", ""),)


def _geometry_allows_anticipation(row: dict[str, str]) -> bool:
  distance = _number(row, "d_rel")
  d_path = abs(_number(row, "d_path"))
  future_d_path = abs(_number(row, "future_d_path"))
  h8_dt = max(_number(row, "h8_dt"), 1e-3)
  h12_dt = max(_number(row, "h12_dt"), 1e-3)
  h8_lane_inward = (abs(_number(row, "h8_d_path")) - d_path) / h8_dt
  h12_lane_inward = (abs(_number(row, "h12_d_path")) - d_path) / h12_dt
  lanes_reliable = _number(row, "lane1_prob") > 0.5 and _number(row, "lane2_prob") > 0.5
  return (
    2.5 < distance < 60.0
    and _number(row, "v_lead") > 2.0
    and _number(row, "track_age") >= 7.0
    and 1.15 < d_path < 4.2
    and future_d_path < 2.5
    and future_d_path + 0.10 < d_path
    and _number(row, "h8_present") > 0.5
    and _number(row, "h12_present") > 0.5
    and (distance < 12.0 or lanes_reliable)
    and 0.12 < h8_lane_inward < 3.2
    and 0.12 < h12_lane_inward < 3.2
  )


def augment_rows(
  rows: list[dict[str, str]], lookback_s: float = DEFAULT_LOOKBACK_S,
) -> AugmentStats:
  """Mutate rows with soft early labels confirmed by the same future radar identity."""
  stats = AugmentStats(rows=len(rows))
  positive_times_by_alias: dict[str, list[float]] = {}
  for row in rows:
    # Do not let our soft labels walk the lookback window farther back when a
    # generated dataset is processed again.
    if _number(row, "cutin_label") <= 0.5 or row.get("cutin_source") == "anticipatory":
      continue
    stats.original_positives += 1
    time_s = _number(row, "time_s")
    for alias in _aliases(row):
      positive_times_by_alias.setdefault(alias, []).append(time_s)
  for times in positive_times_by_alias.values():
    times.sort()

  for row in rows:
    if _number(row, "cutin_label") > 0.5:
      continue
    if row.get("cutin_source") == "manual":
      stats.manual_negatives_preserved += 1
      continue
    time_s = _number(row, "time_s")
    confirmation_time: float | None = None
    for alias in _aliases(row):
      times = positive_times_by_alias.get(alias)
      if not times:
        continue
      index = bisect.bisect_right(times, time_s + 1e-4)
      if index < len(times) and times[index] - time_s <= lookback_s:
        confirmation_time = times[index] if confirmation_time is None else min(confirmation_time, times[index])
    if confirmation_time is None or not _geometry_allows_anticipation(row):
      continue

    lead_s = confirmation_time - time_s
    strength = 0.55 + 0.40 * (1.0 - min(max(lead_s / lookback_s, 0.0), 1.0))
    row["cutin_label"] = f"{strength:.3f}"
    row["cutin_weight"] = f"{ANTICIPATORY_WEIGHT:.3f}"
    row["cutin_source"] = "anticipatory"
    stats.anticipatory_positives += 1
    stats.earliest_lead_s = max(stats.earliest_lead_s, lead_s)
  return stats


def _hard_negative(row: dict[str, str]) -> bool:
  distance = _number(row, "d_rel")
  return (
    2.5 < distance < 70.0
    and _number(row, "v_lead") > 1.5
    and _number(row, "track_age") >= 5.0
    and abs(_number(row, "d_path")) < 4.5
    and abs(_number(row, "future_d_path")) < 4.5
    and _number(row, "h8_present") > 0.5
  )


def compact_rows(rows: list[dict[str, str]], easy_negative_limit: int = 3000) -> list[dict[str, str]]:
  """Keep every useful cut-in row while thinning repetitive easy negatives."""
  required: set[int] = set()
  easy: list[int] = []
  for index, row in enumerate(rows):
    if _number(row, "cutin_label") > 0.5 or row.get("cutin_source") == "manual" or _hard_negative(row):
      required.add(index)
    else:
      easy.append(index)
  if easy and easy_negative_limit > 0:
    count = min(len(easy), easy_negative_limit)
    required.update(easy[min(len(easy) - 1, (slot * len(easy)) // count)] for slot in range(count))
  return [row for index, row in enumerate(rows) if index in required]


def _open_read(path: Path):
  return gzip.open(path, "rt", newline="", encoding="utf-8") if path.suffix.lower() == ".gz" else path.open(
    "r", newline="", encoding="utf-8",
  )


def _open_write(path: Path):
  return gzip.open(path, "wt", newline="", encoding="utf-8", compresslevel=6) if path.suffix.lower() == ".gz" else path.open(
    "w", newline="", encoding="utf-8",
  )


def augment_file(
  source: Path, destination: Path, lookback_s: float = DEFAULT_LOOKBACK_S,
  compact: bool = False, easy_negative_limit: int = 3000,
) -> AugmentStats:
  with _open_read(source) as input_file:
    reader = csv.DictReader(input_file)
    fieldnames = reader.fieldnames
    if not fieldnames or not {"object_id", "aliases", "cutin_label", "cutin_weight", "cutin_source"} <= set(fieldnames):
      raise RuntimeError(f"not a fused radar dataset: {source}")
    rows = list(reader)
  stats = augment_rows(rows, lookback_s)
  if compact:
    rows = compact_rows(rows, easy_negative_limit)
  stats.output_rows = len(rows)
  destination.parent.mkdir(parents=True, exist_ok=True)
  # Keep the final suffix so gzip output is still compressed while it is
  # written atomically.
  temporary = destination.with_name(f"{destination.stem}.tmp{destination.suffix}")
  with _open_write(temporary) as output_file:
    writer = csv.DictWriter(output_file, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(rows)
  temporary.replace(destination)
  return stats


def _datasets(roots: Iterable[Path]) -> list[Path]:
  return sorted({
    path.resolve() for root in roots
    for path in ([root] if root.is_file() else root.glob("*.fused.csv.gz"))
    if path.is_file()
  })


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Back-label stable inward motion before current-radard cut-in confirmation")
  parser.add_argument("roots", nargs="+", type=Path)
  parser.add_argument("--output-dir", required=True, type=Path)
  parser.add_argument("--lookback", type=float, default=DEFAULT_LOOKBACK_S)
  parser.add_argument("--force", action="store_true")
  parser.add_argument("--compact", action="store_true", help="keep positives, hard negatives, and sampled easy negatives")
  parser.add_argument("--easy-negative-limit", type=int, default=3000, help="maximum sampled easy negatives per file")
  return parser.parse_args()


def main() -> int:
  args = parse_args()
  if not 0.1 <= args.lookback <= 1.5:
    raise SystemExit("--lookback must be between 0.1 and 1.5 seconds")
  files = _datasets(args.roots)
  manifest: dict[str, Any] = {"version": 1, "lookback_s": args.lookback, "datasets": [], "errors": []}
  args.output_dir.mkdir(parents=True, exist_ok=True)
  for index, source in enumerate(files, 1):
    destination = args.output_dir / source.name
    print(f"[{index}/{len(files)}] {source.name}", flush=True)
    try:
      if destination.is_file() and not args.force:
        stats: dict[str, Any] = {"cached": True}
      else:
        stats = asdict(augment_file(
          source, destination, args.lookback, args.compact, max(0, args.easy_negative_limit),
        ))
      manifest["datasets"].append({"source": str(source), "dataset": str(destination), "stats": stats})
    except Exception as exc:
      print(f"  ERROR: {exc}", flush=True)
      manifest["errors"].append({"source": str(source), "error": repr(exc)})
    (args.output_dir / "manifest.json").write_text(
      json.dumps(manifest, indent=2, ensure_ascii=True) + "\n", encoding="utf-8",
    )
  print(f"completed {len(manifest['datasets'])}; errors {len(manifest['errors'])}")
  return int(bool(manifest["errors"]))


if __name__ == "__main__":
  raise SystemExit(main())
