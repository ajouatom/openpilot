"""JSON adapter for the same production replay used by the desktop reviewer."""
from __future__ import annotations

import argparse
import bisect
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys

from openpilot.selfdrive.carrot.radar.tools import radar_validation_replay as replay
from openpilot.selfdrive.carrot.radar_motion.native import BACKEND as MOTION_BACKEND


SCHEMA_VERSION = 4
CLASSIC_238_MAX_AGE_S = 0.15


def _classic_238_decoder():
  opendbc_repo = replay.REPO_ROOT / "opendbc_repo"
  if str(opendbc_repo) not in sys.path:
    sys.path.insert(0, str(opendbc_repo))
  from opendbc.car.hyundai.radar_classic_238 import (
    CLASSIC_238_END_ADDR,
    CLASSIC_238_START_ADDR,
    Classic238Triplet,
    classic_238_address_role,
  )
  return CLASSIC_238_START_ADDR, CLASSIC_238_END_ADDR, Classic238Triplet, classic_238_address_role


def _classic_238_object_dict(slot, triplet):
  obj = triplet.obj
  return {
    "slot": slot,
    "status": obj.status,
    "d_rel": obj.d_rel,
    "y_rel": obj.y_rel,
    "v_lead": obj.v_lead,
    "object_sequence": triplet.object_sequence,
    "first_raw_hex": f"{obj.raw:016x}",
    "second_raw_hex": triplet.second_frame.hex(),
    "third_raw_hex": triplet.third_frame.hex(),
    "rolling_counter": triplet.rolling_counters[0],
    "counter_consistent": triplet.counter_consistent,
    "confirmed_candidate": triplet.counter_consistent and obj.status == 2,
  }


def _track_classic_238_snapshots(snapshots):
  opendbc_repo = replay.REPO_ROOT / "opendbc_repo"
  if str(opendbc_repo) not in sys.path:
    sys.path.insert(0, str(opendbc_repo))
  from opendbc.car.hyundai.radar_classic_238 import (
    Classic238AssociationTracker,
    Classic238RadarKinematics,
    Classic238TrackObservation,
  )
  tracker = Classic238AssociationTracker()
  previous_unknown = {}
  tracked_snapshots = []
  for time_s, objects in snapshots:
    mono_time_ns = round(time_s * 1e9)
    observations = [Classic238TrackObservation(
      slot=item["slot"], status=item["status"], object_sequence=item["object_sequence"],
      kinematics=Classic238RadarKinematics(
        d_rel=item["d_rel"], y_rel=item["y_rel"], v_rel=0.0, v_lead=item["v_lead"],
        a_rel=float("nan"), yv_rel=0.0,
      ),
    ) for item in objects]
    tracks = tracker.update_scan(mono_time_ns, observations)
    ids_by_slot = {track.slot: track.track_id for track in tracks if track.last_seen_ns == mono_time_ns}
    tracked_objects = []
    for item in objects:
      track_id = ids_by_slot[item["slot"]]
      tracked = dict(item, track_id=track_id)
      second = int(str(item.get("second_raw_hex") or "0"), 16) & 0x3fffffffffffffff
      third = int(str(item.get("third_raw_hex") or "0"), 16) & 0x3fffff00ffffffff
      previous = previous_unknown.get(track_id)
      tracked["second_unknown_change_mask"] = None if previous is None else f"{second ^ previous[0]:016x}"
      tracked["third_unknown_change_mask"] = None if previous is None else f"{third ^ previous[1]:016x}"
      tracked["second_unknown_changed_bits"] = None if previous is None else (second ^ previous[0]).bit_count()
      tracked["third_unknown_changed_bits"] = None if previous is None else (third ^ previous[1]).bit_count()
      tracked["object_sequence_delta"] = None if previous is None else (item["object_sequence"] - previous[2]) & 0xff
      previous_unknown[track_id] = (second, third, item["object_sequence"])
      tracked_objects.append(tracked)
    tracked_snapshots.append((time_s, tracked_objects))
  return tracked_snapshots


def load_classic_238_snapshots(log_path):
  """Decode passive 0x238 triplets for the NAS viewer without feeding control."""
  start_addr, end_addr, triplet_type, address_role = _classic_238_decoder()
  route_replay = replay._route_replay_module()
  schema = route_replay.load_openpilot_log_schema()
  events = replay.monotonic_log_events(
    schema.Event.read_multiple_bytes(route_replay.read_log_bytes(log_path)),
  )
  states = {}
  snapshots_by_bus = {}
  complete_by_bus = {}
  for event in events:
    try:
      if event.which() != "can":
        continue
    except Exception:
      continue
    event_time_s = int(event.logMonoTime) / 1e9
    touched_buses = set()
    for message in event.can:
      address = int(message.address)
      bus = int(message.src)
      raw = bytes(message.dat)
      if bus >= 0x80 or not start_addr <= address <= end_addr or len(raw) != 8:
        continue
      slot, role = address_role(address)
      states.setdefault(bus, {}).setdefault(slot, {})[role] = (event_time_s, raw)
      touched_buses.add(bus)
    for bus in touched_buses:
      objects = []
      for slot, parts in states[bus].items():
        if not all(role in parts for role in range(3)):
          continue
        times = [parts[role][0] for role in range(3)]
        if max(times) - min(times) > CLASSIC_238_MAX_AGE_S or event_time_s - max(times) > CLASSIC_238_MAX_AGE_S:
          continue
        triplet = triplet_type.from_frames(*(parts[role][1] for role in range(3)))
        if not triplet.counter_consistent:
          continue
        complete_by_bus[bus] = complete_by_bus.get(bus, 0) + 1
        obj = triplet.obj
        if obj.status == 0 or not (0.2 <= obj.d_rel <= 250.0 and abs(obj.y_rel) <= 40.0):
          continue
        objects.append(_classic_238_object_dict(slot, triplet))
      snapshots_by_bus.setdefault(bus, []).append((event_time_s, objects))
  if not snapshots_by_bus:
    return None, []
  selected_bus = max(snapshots_by_bus, key=lambda bus: (complete_by_bus.get(bus, 0), len(snapshots_by_bus[bus])))
  return selected_bus, snapshots_by_bus[selected_bus]


def _classic_238_at(frame_time_s, snapshots, snapshot_times):
  if not snapshots:
    return []
  index = bisect.bisect_right(snapshot_times, frame_time_s) - 1
  if index < 0 or frame_time_s - snapshot_times[index] > CLASSIC_238_MAX_AGE_S:
    return []
  return snapshots[index][1]


def source_version() -> str:
  digest = hashlib.sha256()
  digest.update(f"motion_backend={MOTION_BACKEND}".encode())
  roots = (replay.CARROT_ROOT / "radar_motion", replay.CARROT_ROOT / "cluster",
           replay.REPO_ROOT / "openpilot/selfdrive/controls/lib")
  files = {Path(__file__), Path(replay.__file__), *replay._radar_input_sources()}
  for root in roots:
    files.update(root.glob("*.py"))
    files.update(root.glob("*.pyx"))
  files.update((replay.REPO_ROOT / "openpilot/cereal").glob("*.capnp"))
  for path in (replay.REPO_ROOT / "opendbc_repo/opendbc").rglob("*"):
    if path.suffix in {".py", ".pyx", ".capnp", ".dbc"}:
      files.add(path)
  for path in sorted(files):
    digest.update(path.relative_to(replay.REPO_ROOT).as_posix().encode())
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


def export_frames(frames, *, sensor="auto", sensitivity=replay.VALIDATION_DEFAULT_SENSITIVITY,
                  classic_238_bus=None, classic_238_snapshots=()):
  if not frames:
    raise ValueError("No radar replay frames were found in this log")
  selected_sensor = replay.preferred_radar_motion_sensor(frames) if sensor == "auto" else sensor
  # Source selection controls the motion sensor, not the vehicle's SCC/front
  # policy. Historical logs without settings retain the analysis default.
  recorded_mode = frames[0].recorded_radar_track_mode
  enable_radar_tracks = recorded_mode if recorded_mode is not None else 2
  selector = replay.ProductionDPathSelector(frames, motion_sensor=selected_sensor,
                                          enable_radar_tracks=enable_radar_tracks,
                                          cut_in_sensitivity=sensitivity)
  output = []
  selections = []
  classic_238_snapshots = _track_classic_238_snapshots(classic_238_snapshots)
  classic_238_snapshot_times = [snapshot[0] for snapshot in classic_238_snapshots]
  for index, frame in enumerate(frames):
    item = asdict(frame)
    item["classic_238_objects"] = _classic_238_at(
      frame.mono_time_s, classic_238_snapshots, classic_238_snapshot_times,
    )
    item.pop("mono_time_s", None)
    selected = selector.select(frame, index)
    selections.append(selected)
    selection = asdict(selected)
    item["selection"] = {key: selection[key] for key in ("lead_one", "lead_two", "cutin_diagnostics", "cutin_predecel_candidate", "lane_change_gap")}
    output.append(item)
  def runs(segments, color):
    return [{"color": "#%02x%02x%02x" % (color(segment[0][2]) if callable(color) else color),
             "samples": [point[:2] for point in segment]} for segment in segments]

  graphs = {
    "leadOne": runs(replay.lead_continuity_segments(frames, selections, "lead_one"), replay.lead_one_rgb),
    "leadTwo": runs(replay.lead_continuity_segments(frames, selections, "lead_two"), replay.LEAD_TWO_RGB),
    "vision": runs(replay.vision_lead_continuity_segments(frames), replay.vision_lead_rgb),
    "leadSpeed": runs(replay.lead_speed_continuity_segments(frames, selections), replay.LEAD_ONE_SPEED_RGB),
    "sccDistance": runs(replay.frame_value_continuity_segments(frames, "scc_distance_m"), replay.SCC_DISTANCE_RGB),
    "sccAccel": runs(replay.frame_value_continuity_segments(frames, "scc_a_req_raw"), replay.SCC_ACCEL_RGB),
    "carrotAccel": runs(replay.frame_value_continuity_segments(frames, "carrot_a_target"), replay.CARROT_ACCEL_RGB),
  }
  return finite_json({
    "schemaVersion": SCHEMA_VERSION,
    "sourceVersion": source_version(),
    "engine": selector.name,
    "sensor": selected_sensor,
    "sensitivity": sensitivity,
    "enableRadarTracks": enable_radar_tracks,
    "classic238": {
      "available": classic_238_bus is not None,
      "bus": classic_238_bus,
      "controlConnected": False,
      "confirmedStatus": 2,
    },
    "radarToCamera": replay.RADAR_TO_CAMERA,
    "videoAligned": all(frame.video_time_s is not None for frame in frames),
    "frames": output,
    "graphs": graphs,
  })


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("log", type=Path)
  parser.add_argument("output", type=Path)
  parser.add_argument("--sensor", choices=("auto", "front", "corner"), default="auto")
  parser.add_argument("--sensitivity", type=int, choices=range(6), default=replay.VALIDATION_DEFAULT_SENSITIVITY)
  parser.add_argument("--radar-track-flip", choices=("recorded", "normal", "flipped"), default="recorded")
  args = parser.parse_args()
  flip = {"recorded": None, "normal": False, "flipped": True}[args.radar_track_flip]
  classic_bus, classic_snapshots = load_classic_238_snapshots(args.log)
  payload = export_frames(replay.load_frames(args.log, radar_track_flip=flip), sensor=args.sensor,
                          sensitivity=args.sensitivity, classic_238_bus=classic_bus,
                          classic_238_snapshots=classic_snapshots)
  payload["radarTrackFlip"] = args.radar_track_flip
  payload["sourceLog"] = args.log.name
  args.output.write_text(json.dumps(payload, separators=(",", ":"), allow_nan=False), encoding="utf-8")


if __name__ == "__main__":
  main()
