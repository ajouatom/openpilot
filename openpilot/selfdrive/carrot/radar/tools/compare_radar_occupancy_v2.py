#!/usr/bin/env python3
"""Compare staged occupancy V2 with the maintained physical dPath V1 replay."""

from __future__ import annotations

import argparse
import hashlib
import json
import pickle
import sys
import time
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[5]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.selfdrive.carrot.radar.tools.radar_validation_replay import (
  RadarMotionShadowSelector,
  RadarOccupancyV2Selector,
  RadarOccupancyV3Selector,
  load_frames,
  radar_replay_baseline_source_fingerprint,
)
from openpilot.selfdrive.carrot.radar.tools.validate_radar_lead_model import (
  DEFAULT_CASES,
  DEFAULT_LABELS,
  _entries,
  _event_text,
  _first_event,
  _first_role_constraint_event,
  _lead_one_continuous,
  _lead_two_continuous,
  _metrics,
  _print_metrics,
  _stationary_event,
)
from openpilot.selfdrive.carrot.radar_motion import (
  STATIONARY_MAX_ABS_VLEAD_MPS,
)


DEFAULT_ROOT = Path(r"\\DS1821P\openpilot\routes")
DEFAULT_CACHE_DIR = REPO_ROOT / ".tmp_radar_occupancy_cache"
BASELINE_CACHE_VERSION = 1


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Compare physical dPath V1 and probabilistic occupancy V2",
  )
  parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
  parser.add_argument("--cases", type=Path, default=DEFAULT_CASES)
  parser.add_argument("--labels", type=Path, default=DEFAULT_LABELS)
  parser.add_argument("--case", action="append", default=[])
  parser.add_argument("--expected", choices=("detect", "clear", "stationary"))
  parser.add_argument("--cases-only", action="store_true")
  parser.add_argument("--report", type=Path)
  parser.add_argument("--cache-dir", type=Path, default=DEFAULT_CACHE_DIR)
  parser.add_argument("--no-cache", action="store_true")
  parser.add_argument("--strict-v2", action="store_true")
  parser.add_argument("--strict-v3", action="store_true")
  return parser.parse_args()


def _cache_path(cache_dir: Path, log_path: Path) -> Path:
  stat = log_path.stat()
  identity = (
    f"{BASELINE_CACHE_VERSION}|{log_path}|{stat.st_size}|"
    + f"{stat.st_mtime_ns}|{radar_replay_baseline_source_fingerprint()}"
  )
  digest = hashlib.sha256(identity.encode("utf-8")).hexdigest()[:20]
  return cache_dir / f"{digest}.pickle"


def _load_or_build_baseline(
  log_path: Path,
  cache_dir: Path,
  use_cache: bool,
) -> tuple[list[Any], RadarMotionShadowSelector, bool]:
  cache_path = _cache_path(cache_dir, log_path)
  if use_cache and cache_path.is_file():
    try:
      with cache_path.open("rb") as stream:
        frames, selector = pickle.load(stream)
      return frames, selector, True
    except (EOFError, OSError, pickle.PickleError):
      pass

  frames = load_frames(log_path)
  selector = RadarMotionShadowSelector(frames, enable_radar_tracks=2)
  if use_cache:
    cache_dir.mkdir(parents=True, exist_ok=True)
    temporary = cache_path.with_suffix(".tmp")
    with temporary.open("wb") as stream:
      pickle.dump((frames, selector), stream, protocol=pickle.HIGHEST_PROTOCOL)
    temporary.replace(cache_path)
  return frames, selector, False


def _applicable_metrics(
  rows: list[dict[str, Any]],
  version: str,
) -> dict[str, float | int]:
  event_field = f"{version}_event"
  scorable = [
    row for row in rows
    if row["expected"] in ("detect", "clear")
    and bool(row[version]["applicable"])
  ]
  return _metrics(scorable, event_field)


def _validation_attribute(entry: dict[str, Any]) -> str:
  return {
    "lead_one": "lead_one",
    "lead_two": "lead_two",
    "predecel": "cutin_predecel_candidate",
  }.get(str(entry.get("validation_stage", "output")), "decision_cutin_candidates")


def _evaluate(
  selector: Any,
  frames: list[Any],
  entry: dict[str, Any],
) -> dict[str, Any]:
  expected = str(entry["expected"])
  required_l1_ids = entry.get("required_lead_one_ids", ())
  forbidden_l1_ids = entry.get("forbidden_lead_one_ids", ())
  forbidden_l2_ids = entry.get("forbidden_lead_two_ids", ())
  applicable = (
    expected == "stationary"
    or str(entry.get("source", "front+corner")) == "front+corner"
    or str(entry.get("source", "")) == selector.motion_sensor
    or bool(required_l1_ids or forbidden_l1_ids or forbidden_l2_ids)
  )

  if expected == "stationary":
    event = _stationary_event(
      selector,
      frames,
      entry,
      require_target_ids=False,
      maximum_abs_v_lead=STATIONARY_MAX_ABS_VLEAD_MPS,
    )
  elif applicable:
    event = _first_event(
      selector,
      frames,
      entry,
      _validation_attribute(entry),
    )
  else:
    event = None

  required_l1 = _first_role_constraint_event(
    selector, frames, entry, "lead_one", required_l1_ids,
  )
  forbidden_l1 = _first_role_constraint_event(
    selector, frames, entry, "lead_one", forbidden_l1_ids,
  )
  forbidden_l2 = _first_role_constraint_event(
    selector, frames, entry, "lead_two", forbidden_l2_ids,
  )
  passed = (
    not applicable
    or (event is not None) == (expected in ("detect", "stationary"))
  )
  if applicable:
    passed = (
      passed
      and (not required_l1_ids or required_l1 is not None)
      and forbidden_l1 is None
      and forbidden_l2 is None
    )

  predecel_applicable = (
    selector.motion_sensor == "corner"
    and str(entry.get("source", "front+corner")) in ("corner", "front+corner")
    and expected in ("detect", "clear")
  )
  predecel_event = (
    _first_event(selector, frames, entry, "cutin_predecel_candidate")
    if predecel_applicable else None
  )
  predecel_required = str(entry.get("validation_stage", "output")) == "predecel"
  predecel_pass = (
    not predecel_applicable
    or (
      predecel_event is None
      if expected == "clear"
      else predecel_event is not None or not predecel_required
    )
  )
  if expected == "clear" or predecel_required:
    passed = passed and predecel_pass

  deadline = entry.get("latest_detection_s")
  if (
    passed
    and deadline is not None
    and event is not None
    and expected == "detect"
  ):
    passed = event[0] <= float(deadline)
  if (
    predecel_pass
    and deadline is not None
    and predecel_event is not None
    and predecel_required
  ):
    predecel_pass = predecel_event[0] <= float(deadline)
    passed = passed and predecel_pass

  lead_one_continuous = _lead_one_continuous(selector, frames, entry)
  lead_two_continuous = _lead_two_continuous(selector, frames, entry)
  if lead_one_continuous is not None and applicable:
    passed = passed and lead_one_continuous
  if lead_two_continuous is not None and applicable:
    passed = passed and lead_two_continuous
  return {
    "event": event,
    "predecel_event": predecel_event,
    "applicable": applicable,
    "pass": passed,
    "predecel_pass": predecel_pass,
    "lead_one_continuous": lead_one_continuous,
    "lead_two_continuous": lead_two_continuous,
    "required_lead_one": required_l1,
    "forbidden_lead_one": forbidden_l1,
    "forbidden_lead_two": forbidden_l2,
  }


def main() -> int:
  args = parse_args()
  filters = tuple(value.lower() for value in args.case)
  entries = [
    entry for entry in _entries(args.cases, args.labels, args.cases_only)
    if (args.expected is None or entry["expected"] == args.expected)
    and (
      not filters
      or any(value in str(entry["id"]).lower() for value in filters)
    )
  ]
  if not entries:
    raise SystemExit("no matching validation entries")

  grouped: dict[Path, list[dict[str, Any]]] = {}
  for entry in entries:
    path = args.root / str(entry["vehicle_folder"]) / Path(str(entry["log"]))
    grouped.setdefault(path, []).append(entry)

  rows: list[dict[str, Any]] = []
  missing = 0
  load_time_s = 0.0
  v1_time_s = 0.0
  v2_time_s = 0.0
  v3_time_s = 0.0
  cache_hits = 0
  for log_index, (path, log_entries) in enumerate(grouped.items(), 1):
    if not path.is_file():
      missing += len(log_entries)
      print(f"MISSING {path}", flush=True)
      continue
    print(
      f"[{log_index:02d}/{len(grouped):02d}] "
      + f"{path.parent.name}/{path.name} ({len(log_entries)} labels)",
      flush=True,
    )
    started = time.perf_counter()
    frames, v1, cache_hit = _load_or_build_baseline(
      path, args.cache_dir, not args.no_cache,
    )
    baseline_elapsed = time.perf_counter() - started
    if cache_hit:
      cache_hits += 1
      load_time_s += baseline_elapsed
    else:
      v1_time_s += baseline_elapsed
    started = time.perf_counter()
    v2 = RadarOccupancyV2Selector(frames, baseline=v1)
    v2_time_s += time.perf_counter() - started
    started = time.perf_counter()
    v3 = RadarOccupancyV3Selector(
      frames, baseline=v1, v2_selector=v2,
    )
    v3_time_s += time.perf_counter() - started

    for entry in log_entries:
      v1_result = _evaluate(v1, frames, entry)
      v2_result = _evaluate(v2, frames, entry)
      v3_result = _evaluate(v3, frames, entry)
      latency_delta_s = None
      if v1_result["event"] is not None and v2_result["event"] is not None:
        latency_delta_s = (
          float(v2_result["event"][0]) - float(v1_result["event"][0])
        )
      v3_latency_delta_s = None
      if v2_result["event"] is not None and v3_result["event"] is not None:
        v3_latency_delta_s = (
          float(v3_result["event"][0]) - float(v2_result["event"][0])
        )
      row = {
        "id": str(entry["id"]),
        "validation_set": str(entry["validation_set"]),
        "source": str(entry.get("source", "")),
        "expected": str(entry["expected"]),
        "validation_stage": str(entry.get("validation_stage", "output")),
        "v1_event": v1_result["event"],
        "v2_event": v2_result["event"],
        "v3_event": v3_result["event"],
        "v1_predecel_event": v1_result["predecel_event"],
        "v2_predecel_event": v2_result["predecel_event"],
        "v3_predecel_event": v3_result["predecel_event"],
        "v1_pass": v1_result["pass"],
        "v2_pass": v2_result["pass"],
        "v3_pass": v3_result["pass"],
        "latency_delta_s": latency_delta_s,
        "v3_latency_delta_s": v3_latency_delta_s,
        "v1": v1_result,
        "v2": v2_result,
        "v3": v3_result,
      }
      rows.append(row)
      delta_text = (
        "--" if v3_latency_delta_s is None
        else f"{v3_latency_delta_s:+.2f}s"
      )
      print(
        f"  {entry['id']} expected={entry['expected']} "
        + f"V1={'PASS' if v1_result['pass'] else 'FAIL'}:"
        + f"{_event_text(v1_result['event'])} "
        + f"V2={'PASS' if v2_result['pass'] else 'FAIL'}:"
        + f"{_event_text(v2_result['event'])} "
        + f"V3={'PASS' if v3_result['pass'] else 'FAIL'}:"
        + f"{_event_text(v3_result['event'])} V3-V2={delta_text}",
        flush=True,
      )

  print()
  v1_metrics = _applicable_metrics(rows, "v1")
  v2_metrics = _applicable_metrics(rows, "v2")
  v3_metrics = _applicable_metrics(rows, "v3")
  _print_metrics("physical dPath V1", v1_metrics)
  _print_metrics("probabilistic occupancy V2", v2_metrics)
  _print_metrics("staged occupancy V3", v3_metrics)
  v1_failures = sum(not row["v1_pass"] for row in rows)
  v2_failures = sum(not row["v2_pass"] for row in rows)
  v3_failures = sum(not row["v3_pass"] for row in rows)
  latency_deltas = [
    float(row["latency_delta_s"])
    for row in rows
    if row["expected"] == "detect" and row["latency_delta_s"] is not None
  ]
  mean_delta = (
    sum(latency_deltas) / len(latency_deltas)
    if latency_deltas else None
  )
  v3_latency_deltas = [
    float(row["v3_latency_delta_s"])
    for row in rows
    if row["expected"] == "detect"
    and row["v3_latency_delta_s"] is not None
  ]
  mean_v3_delta = (
    sum(v3_latency_deltas) / len(v3_latency_deltas)
    if v3_latency_deltas else None
  )
  print(
    f"processed={len(rows)} missing={missing} "
    + f"V1 failures={v1_failures} V2 failures={v2_failures} "
    + f"V3 failures={v3_failures} "
    + "mean detection delta="
    + ("--" if mean_delta is None else f"{mean_delta:+.3f}s")
    + " V3-V2="
    + ("--" if mean_v3_delta is None else f"{mean_v3_delta:+.3f}s"),
  )
  print(
    f"timing cached-load={load_time_s:.2f}s "
    + f"baseline-build={v1_time_s:.2f}s "
    + f"V2 incremental={v2_time_s:.2f}s "
    + f"V3 incremental={v3_time_s:.2f}s "
    + f"cache={cache_hits}/{len(grouped)}",
  )

  if args.report is not None:
    payload = {
      "version": 1,
      "description": "physical dPath V1 versus occupancy V2 and staged V3",
      "rows": rows,
      "summary": {
        "v1": v1_metrics,
        "v2": v2_metrics,
        "v3": v3_metrics,
        "missing": missing,
        "v1_expectation_failures": v1_failures,
        "v2_expectation_failures": v2_failures,
        "v3_expectation_failures": v3_failures,
        "mean_detection_delta_s": mean_delta,
        "mean_v3_latency_delta_s": mean_v3_delta,
        "timing_s": {
          "load": load_time_s,
          "baseline_build": v1_time_s,
          "v2_incremental": v2_time_s,
          "v3_incremental": v3_time_s,
          "cache_hits": cache_hits,
        },
      },
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(
      json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
      encoding="utf-8",
    )
    print(f"report written: {args.report}")
  return int(
    missing > 0
    or (args.strict_v2 and v2_failures > 0)
    or (args.strict_v3 and v3_failures > 0)
  )


if __name__ == "__main__":
  raise SystemExit(main())
