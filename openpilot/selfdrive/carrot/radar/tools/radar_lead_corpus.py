#!/usr/bin/env python3
"""Inventory and select a diverse radar-lead training corpus from rlogs."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable


REPO_ROOT = Path(__file__).resolve().parents[5]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

RLOG_NAMES = {"rlog.zst", "rlog.1.zst"}
DEFAULT_GOLD_CASES = Path(__file__).resolve().parents[2] / "cluster" / "cutin_validation_cases.json"
SCENARIO_TAGS = (
  "stationary-selected", "stationary-center", "stationary-side", "recorded-cutin",
  "side-motion", "night", "congestion", "curve", "highway", "front-only", "corner",
)


def _finite(value: Any, default: float = 0.0) -> float:
  try:
    result = float(value)
  except (TypeError, ValueError):
    return default
  return result if math.isfinite(result) else default


def _fraction(count: int, total: int) -> float:
  return count / max(total, 1)


def _percentile(values: list[float], fraction: float) -> float:
  if not values:
    return 0.0
  ordered = sorted(values)
  return ordered[round((len(ordered) - 1) * fraction)]


def _stable_number(value: str, seed: int = 0) -> int:
  digest = hashlib.sha256(f"{seed}:{value}".encode()).digest()
  return int.from_bytes(digest[:8], "big")


def route_group(path: Path) -> str:
  segment = path.parent.name
  parts = segment.rsplit("--", 1)
  route = parts[0] if len(parts) == 2 and parts[1].isdigit() else segment
  return f"{path.parent.parent.name}/{route}"


def normalized_log_suffix(path: Path) -> str:
  return f"{path.parent.parent.name}/{path.parent.name}/{path.name}".replace("\\", "/")


@dataclass(frozen=True)
class LogProfile:
  log: str
  vehicle: str
  route_group: str
  size_bytes: int
  duration_s: float
  car_state_frames: int
  radar_state_frames: int
  live_tracks_frames: int
  model_frames: int
  camera_frames: int
  mean_speed_kph: float
  p90_speed_kph: float
  stopped_fraction: float
  congestion_fraction: float
  highway_fraction: float
  curve_fraction: float
  mean_exposure: float
  p90_exposure: float
  dark_fraction: float
  mean_radar_points: float
  front_points: int
  corner_points: int
  scc_points: int
  raw_corner_messages: int
  side_frames: int
  side_motion_frames: int
  stationary_center_frames: int
  stationary_side_frames: int
  stationary_selected_frames: int
  recorded_lead_one_frames: int
  recorded_lead_two_frames: int
  recorded_cutin_frames: int
  model_lead_frames: int
  has_qcamera: bool

  @property
  def usable(self) -> bool:
    return (
      self.duration_s >= 20.0
      and self.car_state_frames >= 100
      and self.radar_state_frames >= 100
      and self.live_tracks_frames >= 20
      and self.model_frames >= 20
      and self.mean_radar_points > 0.0
    )


def scenario_tags(profile: LogProfile) -> tuple[str, ...]:
  tags: list[str] = []
  radar_frames = max(profile.live_tracks_frames, 1)
  if profile.stationary_selected_frames >= 3:
    tags.append("stationary-selected")
  if _fraction(profile.stationary_center_frames, radar_frames) >= 0.03:
    tags.append("stationary-center")
  if _fraction(profile.stationary_side_frames, radar_frames) >= 0.05:
    tags.append("stationary-side")
  if profile.recorded_cutin_frames >= 2:
    tags.append("recorded-cutin")
  if _fraction(profile.side_motion_frames, radar_frames) >= 0.03:
    tags.append("side-motion")
  if profile.dark_fraction >= 0.20 or profile.p90_exposure >= 60.0:
    tags.append("night")
  if profile.congestion_fraction >= 0.35 and profile.p90_speed_kph >= 8.0:
    tags.append("congestion")
  if profile.curve_fraction >= 0.10:
    tags.append("curve")
  if profile.highway_fraction >= 0.30 or profile.p90_speed_kph >= 90.0:
    tags.append("highway")
  if profile.front_points > 0 and profile.raw_corner_messages == 0:
    tags.append("front-only")
  if profile.raw_corner_messages > 0:
    tags.append("corner")
  return tuple(tags)


def profile_score(profile: LogProfile) -> float:
  frames = max(profile.live_tracks_frames, 1)
  return (
    2.0 * len(scenario_tags(profile))
    + 8.0 * min(_fraction(profile.stationary_selected_frames, profile.radar_state_frames), 0.25)
    + 5.0 * min(_fraction(profile.stationary_center_frames, frames), 0.40)
    + 3.0 * min(_fraction(profile.stationary_side_frames, frames), 0.40)
    + 6.0 * min(_fraction(profile.side_motion_frames, frames), 0.30)
    + 4.0 * min(_fraction(profile.recorded_cutin_frames, profile.radar_state_frames), 0.20)
    + min(profile.mean_radar_points / 10.0, 2.0)
  )


def _radar_source(point: Any) -> str:
  source = str(getattr(point, "radarSource", "frontRadar"))
  track_id = int(getattr(point, "trackId", -1))
  if source == "frontRadar" and (200 <= track_id < 220 or 240 <= track_id < 250):
    return "corner"
  lowered = source.lower()
  if "corner" in lowered:
    return "corner"
  if "scc" in lowered:
    return "scc"
  return "front"


def profile_log(log_path: Path) -> LogProfile:
  # Import through the simulator helper so cluster-local imports work on Windows too.
  from openpilot.selfdrive.carrot.radar.tools.radar_lead_simulator import _route_replay_module

  replay = _route_replay_module()
  schema = replay.load_openpilot_log_schema()
  data = replay.read_log_bytes(log_path)

  first_ns: int | None = None
  last_ns = 0
  speeds: list[float] = []
  exposures: list[float] = []
  car_state_frames = radar_state_frames = live_tracks_frames = model_frames = camera_frames = 0
  stopped = congested = highway = curves = 0
  radar_points = front_points = corner_points = scc_points = raw_corner_messages = 0
  side_frames = side_motion_frames = stationary_center_frames = stationary_side_frames = 0
  stationary_selected_frames = recorded_lead_one_frames = recorded_lead_two_frames = 0
  recorded_cutin_frames = model_lead_frames = 0

  for event in schema.Event.read_multiple_bytes(data):
    try:
      which = event.which()
    except Exception:
      continue
    event_ns = int(event.logMonoTime)
    first_ns = event_ns if first_ns is None else min(first_ns, event_ns)
    last_ns = max(last_ns, event_ns)
    if which == "carState":
      car_state_frames += 1
      speed = max(0.0, _finite(event.carState.vEgo) * 3.6)
      speeds.append(speed)
      stopped += int(speed < 3.0)
      congested += int(speed < 30.0)
      highway += int(speed >= 80.0)
      curves += int(speed >= 15.0 and abs(_finite(event.carState.steeringAngleDeg)) >= 7.0)
    elif which == "roadCameraState":
      camera_frames += 1
      exposures.append(max(0.0, _finite(event.roadCameraState.exposureValPercent)))
    elif which == "modelV2":
      model_frames += 1
      leads = event.modelV2.leadsV3
      model_lead_frames += int(bool(leads) and _finite(leads[0].prob) >= 0.5)
    elif which == "liveTracks":
      live_tracks_frames += 1
      has_side = has_side_motion = has_stationary_center = has_stationary_side = False
      for point in event.liveTracks.points:
        if not bool(getattr(point, "measured", True)):
          continue
        d_rel = _finite(point.dRel)
        y_rel = _finite(point.yRel)
        v_lead = _finite(point.vLead)
        yv_rel = _finite(point.yvRel)
        if d_rel <= 0.5 or abs(y_rel) >= 15.0:
          continue
        radar_points += 1
        source = _radar_source(point)
        front_points += int(source == "front")
        corner_points += int(source == "corner")
        scc_points += int(source == "scc")
        if d_rel <= 60.0 and 1.5 <= abs(y_rel) <= 8.0:
          has_side = True
          has_side_motion |= abs(yv_rel) >= 0.20
        stationary = abs(v_lead) * 3.6 < 3.0
        has_stationary_center |= stationary and 3.0 <= d_rel <= 100.0 and abs(y_rel) < 2.0
        has_stationary_side |= stationary and 3.0 <= d_rel <= 80.0 and 2.0 <= abs(y_rel) <= 8.0
      side_frames += int(has_side)
      side_motion_frames += int(has_side_motion)
      stationary_center_frames += int(has_stationary_center)
      stationary_side_frames += int(has_stationary_side)
    elif which == "radarState":
      radar_state_frames += 1
      state = event.radarState
      leads = (state.leadOne, state.leadTwo)
      recorded_lead_one_frames += int(bool(leads[0].status))
      recorded_lead_two_frames += int(bool(leads[1].status))
      stationary_selected_frames += int(any(
        bool(lead.status) and bool(lead.radar) and 3.0 <= _finite(lead.dRel) <= 100.0
        and abs(_finite(lead.vLead)) * 3.6 < 3.0
        for lead in leads
      ))
      recorded_cutin_frames += int(bool(state.leadsCutIn))
    elif which == "can":
      for message in event.can:
        if int(message.src) != replay.RAW_CORNER_RADAR_BUS or int(message.src) >= 0x80:
          continue
        decoded = replay.decode_raw_corner_objects(
          event_ns / 1e9, int(message.address), bytes(message.dat),
        )
        raw_corner_messages += int(any(replay.raw_corner_object_is_valid(obj) for obj in decoded))

  duration_s = max(0.0, (last_ns - first_ns) / 1e9) if first_ns is not None else 0.0
  mean_speed = sum(speeds) / max(len(speeds), 1)
  mean_exposure = sum(exposures) / max(len(exposures), 1)
  dark = sum(value >= 60.0 for value in exposures)
  return LogProfile(
    log=str(log_path.resolve()),
    vehicle=log_path.parent.parent.name,
    route_group=route_group(log_path),
    size_bytes=log_path.stat().st_size,
    duration_s=duration_s,
    car_state_frames=car_state_frames,
    radar_state_frames=radar_state_frames,
    live_tracks_frames=live_tracks_frames,
    model_frames=model_frames,
    camera_frames=camera_frames,
    mean_speed_kph=mean_speed,
    p90_speed_kph=_percentile(speeds, 0.90),
    stopped_fraction=_fraction(stopped, car_state_frames),
    congestion_fraction=_fraction(congested, car_state_frames),
    highway_fraction=_fraction(highway, car_state_frames),
    curve_fraction=_fraction(curves, car_state_frames),
    mean_exposure=mean_exposure,
    p90_exposure=_percentile(exposures, 0.90),
    dark_fraction=_fraction(dark, camera_frames),
    mean_radar_points=_fraction(radar_points, live_tracks_frames),
    front_points=front_points,
    corner_points=corner_points,
    scc_points=scc_points,
    raw_corner_messages=raw_corner_messages,
    side_frames=side_frames,
    side_motion_frames=side_motion_frames,
    stationary_center_frames=stationary_center_frames,
    stationary_side_frames=stationary_side_frames,
    stationary_selected_frames=stationary_selected_frames,
    recorded_lead_one_frames=recorded_lead_one_frames,
    recorded_lead_two_frames=recorded_lead_two_frames,
    recorded_cutin_frames=recorded_cutin_frames,
    model_lead_frames=model_lead_frames,
    has_qcamera=(log_path.parent / "qcamera.ts").is_file(),
  )


def discover_logs(roots: Iterable[Path]) -> list[Path]:
  return sorted({
    path.resolve()
    for root in roots
    for path in ([root] if root.is_file() else root.rglob("*"))
    if path.is_file() and (path.name.lower() in RLOG_NAMES or path.name.lower().startswith("rlog."))
    and path.suffix.lower() in (".zst", ".bz2")
  })


def _scan_one(path_text: str) -> dict[str, Any]:
  try:
    return {"profile": asdict(profile_log(Path(path_text)))}
  except Exception as exc:
    return {"log": path_text, "error": repr(exc)}


def _scan_subprocess(path: Path, timeout_s: float) -> dict[str, Any]:
  try:
    process = subprocess.run(
      [sys.executable, str(Path(__file__).resolve()), "_profile-one", str(path)],
      cwd=REPO_ROOT,
      capture_output=True,
      text=True,
      timeout=timeout_s,
      check=False,
    )
  except subprocess.TimeoutExpired:
    return {"log": str(path), "error": f"profile timeout after {timeout_s:.0f}s"}
  lines = process.stdout.splitlines()
  if process.returncode == 0 and lines:
    try:
      return json.loads(lines[-1])
    except json.JSONDecodeError:
      pass
  detail = process.stderr.strip().splitlines()
  error = detail[-1] if detail else f"profile process exited {process.returncode}"
  return {"log": str(path), "error": error}


def _dataset_filename(log_path: Path) -> str:
  identity = normalized_log_suffix(log_path)
  digest = hashlib.sha256(identity.lower().encode()).hexdigest()[:10]
  vehicle = re.sub(r"[^A-Za-z0-9_.-]+", "_", log_path.parent.parent.name)[:48]
  segment = re.sub(r"[^A-Za-z0-9_.-]+", "_", log_path.parent.name)
  return f"{vehicle}_{segment}_{log_path.name}_{digest}.fused.csv.gz"


def _export_subprocess(
  log_path: Path, output_path: Path, annotations: Path, include_scc: bool,
  front_only: bool, sensor_mode: str, enable_radar_tracks: int | None, timeout_s: float,
) -> dict[str, Any]:
  command = [
    sys.executable, str(Path(__file__).resolve()), "_export-one", str(log_path),
    "--output", str(output_path), "--annotations", str(annotations),
  ]
  if include_scc:
    command.append("--fusion-scc")
  if front_only:
    command.append("--front-only")
  command.extend(("--sensor-mode", sensor_mode))
  if enable_radar_tracks is not None:
    command.extend(("--enable-radar-tracks", str(enable_radar_tracks)))
  try:
    with tempfile.TemporaryDirectory(prefix="radar-dataset-") as temporary_dir:
      environment = os.environ.copy()
      environment["TMP"] = temporary_dir
      environment["TEMP"] = temporary_dir
      process = subprocess.run(
        command, cwd=REPO_ROOT, capture_output=True, text=True, env=environment,
        timeout=timeout_s, check=False,
      )
  except subprocess.TimeoutExpired:
    output_path.unlink(missing_ok=True)
    return {"log": str(log_path), "error": f"dataset timeout after {timeout_s:.0f}s"}
  lines = process.stdout.splitlines()
  if process.returncode == 0 and lines:
    try:
      return json.loads(lines[-1])
    except json.JSONDecodeError:
      pass
  output_path.unlink(missing_ok=True)
  detail = process.stderr.strip().splitlines()
  error = detail[-1] if detail else f"dataset process exited {process.returncode}"
  return {"log": str(log_path), "error": error}


def read_inventory(path: Path) -> tuple[list[LogProfile], list[dict[str, str]]]:
  profiles: dict[str, LogProfile] = {}
  errors: dict[str, dict[str, str]] = {}
  if not path.is_file():
    return [], []
  for line in path.read_text(encoding="utf-8").splitlines():
    if not line.strip():
      continue
    item = json.loads(line)
    if "profile" in item:
      profile = LogProfile(**item["profile"])
      key = profile.log.lower()
      profiles[key] = profile
      errors.pop(key, None)
    elif "error" in item:
      key = str(item["log"]).lower()
      if key not in profiles:
        errors[key] = item
  return list(profiles.values()), list(errors.values())


def scan_logs(
  roots: list[Path], output: Path, workers: int, resume: bool, max_logs: int, timeout_s: float,
) -> int:
  logs = discover_logs(roots)
  if max_logs > 0:
    logs = logs[:max_logs]
  existing, existing_errors = read_inventory(output) if resume else ([], [])
  completed = {profile.log.lower() for profile in existing} | {item["log"].lower() for item in existing_errors}
  pending = [path for path in logs if str(path).lower() not in completed]
  output.parent.mkdir(parents=True, exist_ok=True)
  mode = "a" if resume and output.is_file() else "w"
  print(f"discovered {len(logs)} logs; pending {len(pending)}; workers {workers}", flush=True)
  errors = 0
  completed_now = 0
  with output.open(mode, encoding="utf-8") as stream:
    # Each log gets a fresh Python process. A malformed Cap'n Proto stream or a
    # large arena can then fail without taking down the full NAS inventory job.
    batch_size = max(1, workers) * 2
    for batch_start in range(0, len(pending), batch_size):
      batch = pending[batch_start:batch_start + batch_size]
      with ThreadPoolExecutor(max_workers=max(1, workers)) as executor:
        futures = {executor.submit(_scan_subprocess, path, timeout_s): path for path in batch}
        results = {str(futures[future]).lower(): future.result() for future in as_completed(futures)}
      for path in batch:
        result = results[str(path).lower()]
        errors += int("error" in result)
        completed_now += 1
        stream.write(json.dumps(result, ensure_ascii=True) + "\n")
        stream.flush()
      print(f"scanned {completed_now}/{len(pending)} errors {errors}", flush=True)
  profiles, all_errors = read_inventory(output)
  usable = sum(profile.usable for profile in profiles)
  print(f"inventory {len(profiles)} profiles, {usable} usable, {len(all_errors)} errors")
  return 0 if not errors else 1


def load_gold_suffixes(path: Path) -> set[str]:
  if not path.is_file():
    return set()
  payload = json.loads(path.read_text(encoding="utf-8"))
  return {
    f"{case['vehicle_folder']}/{case['log']}".replace("\\", "/").lower()
    for case in payload.get("cases", [])
  }


def _is_gold(profile: LogProfile, suffixes: set[str]) -> bool:
  normalized = normalized_log_suffix(Path(profile.log)).lower()
  return any(normalized.endswith(suffix) for suffix in suffixes)


def select_profiles(
  profiles: list[LogProfile], count: int, max_per_vehicle: int, gold_suffixes: set[str], seed: int,
) -> tuple[list[LogProfile], list[LogProfile]]:
  gold = [profile for profile in profiles if _is_gold(profile, gold_suffixes)]
  candidates = [profile for profile in profiles if profile.usable and not _is_gold(profile, gold_suffixes)]
  by_bucket: dict[tuple[str, str], list[LogProfile]] = {}
  for profile in candidates:
    for tag in scenario_tags(profile) or ("general",):
      by_bucket.setdefault((tag, profile.vehicle), []).append(profile)
  for bucket in by_bucket.values():
    bucket.sort(key=lambda item: (-profile_score(item), _stable_number(item.log, seed)))

  vehicles = sorted({profile.vehicle for profile in candidates}, key=lambda item: _stable_number(item, seed))
  selected: list[LogProfile] = []
  selected_logs: set[str] = set()
  vehicle_counts: dict[str, int] = {}
  tags = (*SCENARIO_TAGS, "general")
  made_progress = True
  while len(selected) < count and made_progress:
    made_progress = False
    for tag in tags:
      for vehicle in vehicles:
        bucket = by_bucket.get((tag, vehicle), [])
        while bucket and bucket[0].log in selected_logs:
          bucket.pop(0)
        if not bucket or vehicle_counts.get(vehicle, 0) >= max_per_vehicle:
          continue
        profile = bucket.pop(0)
        selected.append(profile)
        selected_logs.add(profile.log)
        vehicle_counts[vehicle] = vehicle_counts.get(vehicle, 0) + 1
        made_progress = True
        if len(selected) >= count:
          break
      if len(selected) >= count:
        break

  if len(selected) < count:
    remaining = sorted(
      (profile for profile in candidates if profile.log not in selected_logs),
      key=lambda item: (-profile_score(item), _stable_number(item.log, seed)),
    )
    for profile in remaining:
      if vehicle_counts.get(profile.vehicle, 0) >= max_per_vehicle:
        continue
      selected.append(profile)
      vehicle_counts[profile.vehicle] = vehicle_counts.get(profile.vehicle, 0) + 1
      if len(selected) >= count:
        break
  return selected, gold


def split_profiles(
  profiles: list[LogProfile], validation_fraction: float, test_fraction: float, seed: int,
) -> dict[str, list[LogProfile]]:
  groups: dict[str, list[LogProfile]] = {}
  for profile in profiles:
    groups.setdefault(profile.route_group, []).append(profile)
  result: dict[str, list[LogProfile]] = {"train": [], "validation": [], "test": []}
  for key in sorted(groups, key=lambda item: _stable_number(item, seed)):
    unit = _stable_number(key, seed) / float(2**64)
    split = "test" if unit < test_fraction else "validation" if unit < test_fraction + validation_fraction else "train"
    result[split].extend(groups[key])
  return result


def write_selection(
  inventory: Path, output: Path, count: int, max_per_vehicle: int, validation_fraction: float,
  test_fraction: float, gold_cases: Path, seed: int,
) -> int:
  profiles, errors = read_inventory(inventory)
  selected, gold = select_profiles(profiles, count, max_per_vehicle, load_gold_suffixes(gold_cases), seed)
  splits = split_profiles(selected, validation_fraction, test_fraction, seed)
  payload = {
    "version": 1,
    "inventory": str(inventory.resolve()),
    "config": {
      "requested_logs": count,
      "max_per_vehicle": max_per_vehicle,
      "validation_fraction": validation_fraction,
      "test_fraction": test_fraction,
      "seed": seed,
    },
    "inventory_stats": {"profiles": len(profiles), "usable": sum(item.usable for item in profiles), "errors": len(errors)},
    "splits": {
      name: [{**asdict(profile), "tags": scenario_tags(profile), "score": profile_score(profile)} for profile in items]
      for name, items in splits.items()
    },
    "gold_validation": [{**asdict(profile), "tags": scenario_tags(profile)} for profile in gold],
  }
  output.parent.mkdir(parents=True, exist_ok=True)
  output.write_text(json.dumps(payload, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")
  print("selected " + " ".join(f"{name}={len(items)}" for name, items in splits.items()))
  print(f"gold validation logs present={len(gold)}; output={output}")
  return 0


def build_datasets(
  selection_path: Path, output_root: Path, splits: list[str], workers: int,
  timeout_s: float, annotations: Path, include_scc: bool, front_only: bool,
  sensor_mode: str, enable_radar_tracks: int | None, force: bool,
) -> int:
  selection = json.loads(selection_path.read_text(encoding="utf-8"))
  total_errors = 0
  for split in splits:
    if split not in selection.get("splits", {}):
      raise SystemExit(f"unknown selection split: {split}")
    output_dir = output_root / split
    output_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = output_dir / "manifest.json"
    if manifest_path.is_file() and not force:
      manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    else:
      manifest = {
        "version": 4,
        "selection": str(selection_path.resolve()),
        "split": split,
        "sensor_mode": sensor_mode,
        "enable_radar_tracks": enable_radar_tracks,
        "include_scc": include_scc,
        "datasets": [],
        "errors": [],
      }
    if (
      manifest.get("sensor_mode", "fused") != sensor_mode
      or manifest.get("enable_radar_tracks") != enable_radar_tracks
      or bool(manifest.get("include_scc", False)) != include_scc
    ):
      raise SystemExit(f"{manifest_path} was built with different radar source settings; use --force or another output directory")
    completed = {
      str(item["log"]).lower(): item for item in manifest.get("datasets", [])
      if Path(item["dataset"]).is_file()
    }
    logs = [Path(item["log"]) for item in selection["splits"][split]]
    pending = [log for log in logs if force or str(log).lower() not in completed]
    if force:
      manifest["datasets"] = []
      manifest["errors"] = []
    print(f"{split}: selected {len(logs)} pending {len(pending)} workers {workers}", flush=True)
    for batch_start in range(0, len(pending), max(1, workers)):
      batch = pending[batch_start:batch_start + max(1, workers)]
      with ThreadPoolExecutor(max_workers=max(1, workers)) as executor:
        futures = {}
        for log_path in batch:
          output_path = output_dir / _dataset_filename(log_path)
          futures[executor.submit(
            _export_subprocess, log_path, output_path, annotations,
            include_scc, front_only, sensor_mode, enable_radar_tracks, timeout_s,
          )] = log_path
        results = [future.result() for future in as_completed(futures)]
      for result in results:
        log_key = str(result["log"]).lower()
        manifest["datasets"] = [item for item in manifest["datasets"] if str(item["log"]).lower() != log_key]
        manifest["errors"] = [item for item in manifest["errors"] if str(item["log"]).lower() != log_key]
        if "error" in result:
          manifest["errors"].append(result)
          total_errors += 1
        else:
          manifest["datasets"].append(result)
      manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")
      done = min(batch_start + len(batch), len(pending))
      print(f"{split}: built {done}/{len(pending)} errors {len(manifest['errors'])}", flush=True)
  return 0 if total_errors == 0 else 1


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description=__doc__)
  subparsers = parser.add_subparsers(dest="command", required=True)
  scan = subparsers.add_parser("scan", help="scan rlogs into a resumable JSONL inventory")
  scan.add_argument("roots", nargs="+", type=Path)
  scan.add_argument("--output", required=True, type=Path)
  scan.add_argument("--workers", type=int, default=max(1, min(2, (os.cpu_count() or 2) // 2)))
  scan.add_argument("--resume", action="store_true")
  scan.add_argument("--max-logs", type=int, default=0)
  scan.add_argument("--timeout", type=float, default=180.0, help="maximum seconds allowed for one log")
  one = subparsers.add_parser("_profile-one", help=argparse.SUPPRESS)
  one.add_argument("rlog", type=Path)
  export_one = subparsers.add_parser("_export-one", help=argparse.SUPPRESS)
  export_one.add_argument("rlog", type=Path)
  export_one.add_argument("--output", required=True, type=Path)
  export_one.add_argument("--annotations", required=True, type=Path)
  export_one.add_argument("--fusion-scc", action="store_true")
  export_one.add_argument("--front-only", action="store_true")
  export_one.add_argument("--sensor-mode", choices=("fused", "front", "corner"), default="fused")
  export_one.add_argument("--enable-radar-tracks", type=int, default=None)
  select = subparsers.add_parser("select", help="select balanced train/validation/test logs")
  select.add_argument("--inventory", required=True, type=Path)
  select.add_argument("--output", required=True, type=Path)
  select.add_argument("--logs", type=int, default=180)
  select.add_argument("--max-per-vehicle", type=int, default=18)
  select.add_argument("--validation-fraction", type=float, default=0.15)
  select.add_argument("--test-fraction", type=float, default=0.15)
  select.add_argument("--gold-cases", type=Path, default=DEFAULT_GOLD_CASES)
  select.add_argument("--seed", type=int, default=42)
  build = subparsers.add_parser("build", help="materialize teacher datasets from a selection")
  build.add_argument("--selection", required=True, type=Path)
  build.add_argument("--output-dir", required=True, type=Path)
  build.add_argument("--splits", nargs="+", choices=("train", "validation", "test"), default=("train", "validation", "test"))
  build.add_argument("--workers", type=int, default=1)
  build.add_argument("--timeout", type=float, default=600.0)
  build.add_argument(
    "--annotations", type=Path,
    default=Path(__file__).resolve().parents[1] / "data" / "radar_lead_annotations.json",
  )
  build.add_argument("--fusion-scc", action="store_true")
  build.add_argument("--front-only", action="store_true")
  build.add_argument("--sensor-mode", choices=("fused", "front", "corner"), default="fused")
  build.add_argument("--enable-radar-tracks", type=int, default=None)
  build.add_argument("--force", action="store_true")
  return parser.parse_args()


def main() -> int:
  args = parse_args()
  if args.command == "_profile-one":
    print(json.dumps(_scan_one(str(args.rlog)), ensure_ascii=True))
    return 0
  if args.command == "_export-one":
    from openpilot.selfdrive.carrot.radar.tools.radar_lead_fused_dataset import export_fused_dataset
    result = {
      "log": str(args.rlog.resolve()),
      "dataset": str(args.output.resolve()),
      "stats": export_fused_dataset(
        args.rlog, args.output, include_scc=args.fusion_scc,
        front_only=args.front_only, annotations_path=args.annotations,
        sensor_mode=args.sensor_mode, enable_radar_tracks=args.enable_radar_tracks,
      ),
    }
    print(json.dumps(result, ensure_ascii=True))
    return 0
  if args.command == "scan":
    return scan_logs(args.roots, args.output, args.workers, args.resume, args.max_logs, args.timeout)
  if args.command == "build":
    return build_datasets(
      args.selection, args.output_dir, args.splits, args.workers, args.timeout,
      args.annotations, args.fusion_scc, args.front_only,
      args.sensor_mode, args.enable_radar_tracks, args.force,
    )
  if args.validation_fraction < 0.0 or args.test_fraction < 0.0 or args.validation_fraction + args.test_fraction >= 1.0:
    raise SystemExit("validation/test fractions must be non-negative and sum to less than 1")
  return write_selection(
    args.inventory, args.output, args.logs, args.max_per_vehicle,
    args.validation_fraction, args.test_fraction, args.gold_cases, args.seed,
  )


if __name__ == "__main__":
  raise SystemExit(main())
