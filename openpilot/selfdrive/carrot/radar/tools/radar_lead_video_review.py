#!/usr/bin/env python3
"""Mine physical dPath predictor events and build video-review batches."""

from __future__ import annotations

import argparse
from collections.abc import Iterable
from dataclasses import asdict, dataclass, field
from datetime import UTC, datetime
import hashlib
import json
from pathlib import Path
import re
import sys
import textwrap
from typing import Any

try:
  from PIL import Image, ImageDraw, ImageFont
except ModuleNotFoundError:
  Image = ImageDraw = ImageFont = None

REPO_ROOT = Path(__file__).resolve().parents[5]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.selfdrive.carrot.radar.tools.radar_lead_simulator import (
  Candidate,
  RadarFrame,
  RadarMotionShadowSelector,
  RadarPoint,
  load_frames,
  qcamera_path_for_log,
  _route_replay_module,
)


VERDICTS = ("positive", "negative", "ambiguous")
CONFIDENCES = ("high", "medium", "low")


@dataclass
class ReviewEvent:
  id: str
  log: str
  kind: str
  track_id: int
  start_s: float
  end_s: float
  focus_s: float
  shadow_peak: float
  d_rel: float
  y_rel: float
  source: str
  priority: float
  signal_kinds: list[str] = field(default_factory=list)
  status: str = "pending"
  verdict: str | None = None
  confidence: str | None = None
  note: str = ""
  contact_sheet: str | None = None


def _point(frame: RadarFrame, track_id: int) -> RadarPoint | None:
  return next((point for point in frame.points if point.track_id == track_id), None)


def _candidate_point(frame: RadarFrame, candidate: Candidate) -> RadarPoint | None:
  return _point(frame, candidate.track_id)


def _same_object(frame: RadarFrame, left_id: int, right_id: int) -> bool:
  if left_id == right_id:
    return True
  left = _point(frame, left_id)
  right = _point(frame, right_id)
  if left is None or right is None:
    return False
  return (
    abs(left.d_rel - right.d_rel) <= 3.0
    and abs(left.y_rel - right.y_rel) <= 1.5
    and abs(left.v_rel - right.v_rel) <= 3.0
  )


def _event_id(log_path: Path, kind: str, track_id: int, start_s: float) -> str:
  key = f"{log_path.as_posix().lower()}|{kind}|{track_id}|{start_s:.2f}"
  return hashlib.sha1(key.encode("utf-8")).hexdigest()[:12]


def _related_events(left: ReviewEvent, right: ReviewEvent, frames: list[RadarFrame], merge_gap_s: float) -> bool:
  if right.start_s - left.end_s > merge_gap_s or left.start_s - right.end_s > merge_gap_s:
    return False
  if left.track_id == right.track_id:
    return True
  overlap_start = max(left.start_s, right.start_s)
  overlap_end = min(left.end_s, right.end_s)
  sample_s = (overlap_start + overlap_end) * 0.5 if overlap_start <= overlap_end else (left.end_s + right.start_s) * 0.5
  return _same_object(_nearest_frame(frames, sample_s), left.track_id, right.track_id)


def _merge_related_events(events: list[ReviewEvent], frames: list[RadarFrame], merge_gap_s: float) -> list[ReviewEvent]:
  merged: list[ReviewEvent] = []
  for candidate in sorted(events, key=lambda event: (event.start_s, event.end_s)):
    match = next((event for event in reversed(merged) if _related_events(event, candidate, frames, merge_gap_s)), None)
    if match is None:
      candidate.signal_kinds = list(dict.fromkeys(candidate.signal_kinds or [candidate.kind]))
      merged.append(candidate)
      continue

    match.start_s = min(match.start_s, candidate.start_s)
    match.end_s = max(match.end_s, candidate.end_s)
    match.signal_kinds = list(dict.fromkeys([
      *(match.signal_kinds or [match.kind]),
      *(candidate.signal_kinds or [candidate.kind]),
    ]))
    if candidate.priority > match.priority:
      match.kind = candidate.kind
      match.track_id = candidate.track_id
      match.focus_s = candidate.focus_s
      match.d_rel = candidate.d_rel
      match.y_rel = candidate.y_rel
      match.source = candidate.source
      match.priority = candidate.priority
    match.shadow_peak = max(match.shadow_peak, candidate.shadow_peak)
  return merged


def _priority(kind: str, score: float, d_rel: float) -> float:
  base = {"predictor-cutin": 100.0, "uncertain": 35.0}[kind]
  return base + 10.0 * score + max(0.0, 50.0 - d_rel) / 10.0


def _signals_for_frame(
  selection: Any,
) -> list[tuple[str, Candidate]]:
  active = tuple(selection.decision_cutin_candidates)
  signals = [("predictor-cutin", candidate) for candidate in active]

  active_ids = {candidate.track_id for candidate in active}
  raw = next((
    candidate for candidate in selection.cutin_diagnostics
    if candidate.track_id not in active_ids and 0.65 <= candidate.score < candidate.decision_threshold
  ), None)
  if raw is not None:
    signals.append(("uncertain", raw))
  return signals


def mine_events(
  log_path: Path,
  frames: list[RadarFrame],
  merge_gap_s: float = 1.5,
) -> list[ReviewEvent]:
  selector = RadarMotionShadowSelector(frames)
  open_events: dict[tuple[str, int], ReviewEvent] = {}
  completed: list[ReviewEvent] = []

  for index, frame in enumerate(frames):
    seen: set[tuple[str, int]] = set()
    for kind, candidate in _signals_for_frame(selector.select(frame, index)):
      point = _candidate_point(frame, candidate)
      d_rel = point.d_rel if point is not None else float(candidate.d_rel or 0.0)
      y_rel = point.y_rel if point is not None else float(candidate.y_rel or 0.0)
      source = point.source if point is not None else "unknown"
      key = (kind, candidate.track_id)
      seen.add(key)
      event = open_events.get(key)
      if event is None or frame.time_s - event.end_s > merge_gap_s:
        if event is not None:
          completed.append(event)
        event = ReviewEvent(
          id=_event_id(log_path, kind, candidate.track_id, frame.time_s),
          log=str(log_path), kind=kind, track_id=candidate.track_id,
          start_s=frame.time_s, end_s=frame.time_s, focus_s=frame.time_s,
          shadow_peak=candidate.score, d_rel=d_rel, y_rel=y_rel, source=source,
          priority=_priority(kind, candidate.score, d_rel), signal_kinds=[kind],
        )
        open_events[key] = event
      else:
        event.end_s = frame.time_s
        priority = _priority(kind, candidate.score, d_rel)
        if priority >= event.priority:
          event.focus_s = frame.time_s
          event.shadow_peak = max(event.shadow_peak, candidate.score)
          event.d_rel = d_rel
          event.y_rel = y_rel
          event.source = source
          event.priority = priority

    for key, event in tuple(open_events.items()):
      if key not in seen and frame.time_s - event.end_s > merge_gap_s:
        completed.append(event)
        open_events.pop(key)

  completed.extend(open_events.values())
  completed = _merge_related_events(completed, frames, merge_gap_s)
  for event in completed:
    event.start_s = round(event.start_s, 3)
    event.end_s = round(event.end_s, 3)
    event.focus_s = round(event.focus_s, 3)
    event.shadow_peak = round(event.shadow_peak, 4)
    event.d_rel = round(event.d_rel, 3)
    event.y_rel = round(event.y_rel, 3)
    event.priority = round(event.priority, 3)
  return sorted(completed, key=lambda event: (-event.priority, event.log, event.start_s))


def _selection_logs(path: Path, splits: Iterable[str]) -> list[Path]:
  payload = json.loads(path.read_text(encoding="utf-8"))
  logs: list[Path] = []
  for split in splits:
    logs.extend(Path(entry["log"]) for entry in payload.get("splits", {}).get(split, ()))
  return logs


def _segment_key(path: Path) -> str:
  return str(path.parent).lower()


def _inventory_logs(path: Path, vehicle_pattern: str | None = None, max_logs_per_vehicle: int = 0) -> list[Path]:
  pattern = re.compile(vehicle_pattern, re.IGNORECASE) if vehicle_pattern else None
  profiles: list[dict[str, Any]] = []
  with path.open(encoding="utf-8") as inventory:
    for line in inventory:
      if not line.strip():
        continue
      payload = json.loads(line)
      profile = payload.get("profile")
      if not isinstance(profile, dict):
        continue
      if pattern is not None and pattern.search(str(profile.get("vehicle", ""))) is None:
        continue
      if not profile.get("has_qcamera") or profile.get("live_tracks_frames", 0) <= 0 or profile.get("model_frames", 0) <= 0:
        continue
      profiles.append(profile)

  unique_segments: dict[str, dict[str, Any]] = {}
  for profile in profiles:
    log_path = Path(profile["log"])
    segment_key = _segment_key(log_path)
    previous = unique_segments.get(segment_key)
    if previous is None or (log_path.name == "rlog.zst" and Path(previous["log"]).name != "rlog.zst"):
      unique_segments[segment_key] = profile
  profiles = list(unique_segments.values())

  # Review likely lateral-motion scenes first, with deterministic ordering.
  profiles.sort(key=lambda profile: (
    int(profile.get("recorded_cutin_frames", 0) > 0),
    int(profile.get("raw_corner_messages", 0) > 0),
    int(profile.get("side_motion_frames", 0)),
    int(profile.get("recorded_cutin_frames", 0)),
    int(profile.get("corner_points", 0)),
    int(profile.get("front_points", 0)),
    str(profile.get("log", "")),
  ), reverse=True)
  if max_logs_per_vehicle:
    counts: dict[str, int] = {}
    diverse_profiles: list[dict[str, Any]] = []
    for profile in profiles:
      vehicle = str(profile.get("vehicle", ""))
      if counts.get(vehicle, 0) >= max_logs_per_vehicle:
        continue
      counts[vehicle] = counts.get(vehicle, 0) + 1
      diverse_profiles.append(profile)
    profiles = diverse_profiles
  return [Path(profile["log"]) for profile in profiles]


def _reviewed_event(event: ReviewEvent, annotations: dict[str, Any]) -> bool:
  log_path = Path(event.log)
  key = f"{log_path.parent.name}/{log_path.name}"
  for entry in annotations.get("logs", {}).get(key, ()):
    candidate = entry.get("cutin_candidate")
    if candidate is None or int(candidate) != event.track_id:
      continue
    if float(entry.get("end_s", -1.0)) >= event.start_s and float(entry.get("start_s", float("inf"))) <= event.end_s:
      return True
  return False


def _load_queue(path: Path) -> dict[str, Any]:
  payload = json.loads(path.read_text(encoding="utf-8"))
  for event in payload.get("events", ()):
    if "shadow_peak" not in event and "model_peak" in event:
      event["shadow_peak"] = event.pop("model_peak")
  return payload


def _queue_log_paths(path: Path) -> set[Path]:
  payload = _load_queue(path)
  logs = {Path(log) for log in payload.get("scanned_logs", ())}
  logs.update(Path(event["log"]) for event in payload.get("events", ()) if event.get("log"))
  return logs


def _save_queue(path: Path, payload: dict[str, Any]) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  temporary = path.with_name(path.name + ".tmp")
  temporary.write_text(json.dumps(payload, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")
  temporary.replace(path)


def build_queue(args: argparse.Namespace) -> int:
  logs = list(args.log)
  if args.selection is not None:
    logs.extend(_selection_logs(args.selection, args.split or ["test"]))
  if args.inventory is not None:
    logs.extend(_inventory_logs(args.inventory, args.vehicle_pattern, args.max_logs_per_vehicle))
  unique_logs = list(dict.fromkeys(Path(log) for log in logs))
  excluded_segments: set[str] = set()
  for queue_path in args.exclude_queue:
    if queue_path.is_file():
      excluded_segments.update(_segment_key(log) for log in _queue_log_paths(queue_path))
  unique_logs = [log for log in unique_logs if _segment_key(log) not in excluded_segments]
  if args.max_logs:
    unique_logs = unique_logs[:args.max_logs]
  if not unique_logs:
    raise SystemExit("provide --log, --selection, or --inventory")

  events: list[ReviewEvent] = []
  errors: list[dict[str, str]] = []
  for index, log_path in enumerate(unique_logs, 1):
    print(f"[{index}/{len(unique_logs)}] {log_path}", flush=True)
    try:
      if not log_path.is_file():
        raise FileNotFoundError(log_path)
      if not qcamera_path_for_log(log_path).is_file():
        raise FileNotFoundError(f"qcamera missing beside {log_path}")
      frames = load_frames(log_path)
      log_events = mine_events(log_path, frames, args.merge_gap)
      if args.max_events_per_log:
        log_events = log_events[:args.max_events_per_log]
      events.extend(log_events)
    except Exception as exc:
      errors.append({"log": str(log_path), "error": repr(exc)})
      print(f"  ERROR {exc}", flush=True)

  events.sort(key=lambda event: (-event.priority, event.log, event.start_s))
  if args.exclude_annotations is not None and args.exclude_annotations.is_file():
    annotations = _load_queue(args.exclude_annotations)
    events = [event for event in events if not _reviewed_event(event, annotations)]
  if args.max_events:
    events = events[:args.max_events]
  payload = {
    "version": 1,
    "created_at": datetime.now(UTC).isoformat(),
    "predictor": "measured dPath physical predictor",
    "merge_gap_s": args.merge_gap,
    "scanned_logs": [str(log) for log in unique_logs],
    "events": [asdict(event) for event in events],
    "errors": errors,
  }
  _save_queue(args.output, payload)
  counts = {kind: sum(event.kind == kind for event in events) for kind in ("predictor-cutin", "uncertain")}
  print(f"wrote {len(events)} events to {args.output}: {counts}; errors {len(errors)}")
  return int(bool(errors) and not events)


def _nearest_frame(frames: list[RadarFrame], time_s: float) -> RadarFrame:
  return min(frames, key=lambda frame: abs(frame.time_s - time_s))


def _font(size: int) -> ImageFont.ImageFont:
  if ImageFont is None:
    raise RuntimeError("Pillow is required to render radar review contact sheets")
  path = Path(__file__).resolve().parents[3] / "assets" / "fonts" / "Inter-Regular.ttf"
  return ImageFont.truetype(str(path), size) if path.is_file() else ImageFont.load_default()


def _draw_map(frame: RadarFrame, track_id: int, width: int, height: int) -> Image.Image:
  image = Image.new("RGB", (width, height), (14, 18, 23))
  draw = ImageDraw.Draw(image)
  center = width // 2
  lateral_scale = 16.0

  def xy(d_rel: float, y_rel: float) -> tuple[float, float]:
    return center - y_rel * lateral_scale, height - 10.0 - min(max(d_rel, 0.0), 100.0) / 100.0 * (height - 20.0)

  for line, color in ((frame.path, (70, 145, 255)), *[(lane, (125, 135, 145)) for lane in frame.lane_lines]):
    if len(line) > 1:
      draw.line([xy(x, -y) for x, y in line if 0.0 <= x <= 100.0], fill=color, width=2)
  for point in frame.points:
    x, y = xy(point.d_rel, point.y_rel)
    selected = point.track_id == track_id
    color = (235, 75, 225) if selected else ((70, 205, 225) if point.source == "frontRadar" else (165, 100, 235))
    radius = 6 if selected else 3
    draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=color)
  return image


def render_contact_sheet(event: dict[str, Any], frames: list[RadarFrame], output: Path) -> None:
  route_replay = _route_replay_module()
  log_path = Path(event["log"])
  video_path = qcamera_path_for_log(log_path)
  video_end_s = max((frame.video_time_s if frame.video_time_s is not None else frame.time_s) for frame in frames)
  reader = route_replay.RouteVideoFrameReader([
    route_replay.RouteVideoSegment(None, video_path, 0.0, video_end_s),
  ])
  start_s = float(event["start_s"])
  end_s = float(event["end_s"])
  focus_s = float(event["focus_s"])
  samples = (
    ("PRE", max(0.0, start_s - 1.0)),
    ("START", start_s),
    ("FOCUS", focus_s),
    ("END", end_s),
    ("POST", end_s + 1.0),
  )
  card_width, camera_height, map_height = 388, 244, 116
  title_height = 52
  card_height = title_height + camera_height + map_height
  columns, rows = 3, 2
  sheet = Image.new("RGB", (card_width * columns, card_height * rows), (14, 18, 23))
  draw = ImageDraw.Draw(sheet)
  title_font = _font(16)
  small_font = _font(13)
  try:
    for index, (phase, sample_s) in enumerate(samples):
      frame = _nearest_frame(frames, sample_s)
      video_time = frame.video_time_s if frame.video_time_s is not None else frame.time_s
      video = reader.frame_at(video_time)
      x = (index % columns) * card_width
      y = (index // columns) * card_height
      if video is not None:
        camera = Image.frombytes("RGBA", (video.width, video.height), video.rgba).convert("RGB")
        sheet.paste(camera, (x, y + title_height))
      sheet.paste(_draw_map(frame, int(event["track_id"]), card_width, map_height), (x, y + title_height + camera_height))
      draw.text((x + 8, y + 5), f"{phase:<5} t={frame.time_s:.2f}", fill=(235, 238, 241), font=title_font)
      point = _point(frame, int(event["track_id"]))
      detail = "track absent" if point is None else f"id {point.track_id} {point.source}  d {point.d_rel:.1f} y {point.y_rel:+.1f} v {point.v_lead*3.6:.0f}km/h"
      draw.text((x + 8, y + 29), detail, fill=(190, 198, 207), font=small_font)

    summary_x = 2 * card_width + 18
    summary_y = card_height + 24
    kinds = " -> ".join(event.get("signal_kinds") or [event["kind"]])
    lines = [
      "REVIEW EVENT",
      f"{event['id']}  id {event['track_id']}  {event['source']}",
      f"interval: {event['start_s']:.2f} - {event['end_s']:.2f}s",
      f"peak: {event['shadow_peak']:.2f}  d {event['d_rel']:.1f}m  y {event['y_rel']:+.1f}m",
      "magenta: selected track",
      "cyan: front radar   purple: corner radar",
    ]
    lines[2:2] = ["signals:", *textwrap.wrap(kinds, width=42)]
    for line_index, line in enumerate(lines):
      draw.text((summary_x, summary_y + line_index * 30), line, fill=(235, 238, 241), font=title_font if line_index == 0 else small_font)
  finally:
    reader.close()
  output.parent.mkdir(parents=True, exist_ok=True)
  sheet.save(output)


def render_queue(args: argparse.Namespace) -> int:
  payload = _load_queue(args.queue)
  events = [
    event for event in payload.get("events", ())
    if event.get("status", "pending") == args.status
  ][:args.limit]
  grouped: dict[str, list[dict[str, Any]]] = {}
  for event in events:
    grouped.setdefault(event["log"], []).append(event)
  rendered = 0
  for log, log_events in grouped.items():
    frames = load_frames(Path(log))
    for event in log_events:
      output = args.output_dir / f"{event['id']}_{event['kind']}_id{event['track_id']}.jpg"
      print(f"render {output.name}", flush=True)
      render_contact_sheet(event, frames, output)
      event["contact_sheet"] = str(output)
      rendered += 1
  _save_queue(args.queue, payload)
  print(f"rendered {rendered} contact sheets in {args.output_dir}")
  return 0


def mark_event(args: argparse.Namespace) -> int:
  payload = _load_queue(args.queue)
  matches = [event for event in payload.get("events", ()) if event["id"] == args.id]
  if len(matches) != 1:
    raise SystemExit(f"event id matched {len(matches)} entries")
  event = matches[0]
  event["status"] = "reviewed"
  event["verdict"] = args.verdict
  event["confidence"] = args.confidence
  event["note"] = args.note
  if args.track_id is not None:
    event["track_id"] = args.track_id
  if args.start_s is not None:
    event["start_s"] = args.start_s
  if args.end_s is not None:
    event["end_s"] = args.end_s
  if float(event["end_s"]) < float(event["start_s"]):
    raise SystemExit("end time must not be before start time")
  _save_queue(args.queue, payload)
  print(f"{event['id']} -> {event['verdict']} ({event['confidence']})")
  return 0


def list_queue(args: argparse.Namespace) -> int:
  payload = _load_queue(args.queue)
  events = [event for event in payload.get("events", ()) if args.status == "all" or event.get("status", "pending") == args.status]
  for event in events[:args.limit]:
    print(
      f"{event['id']} {event['status']:<8} "
      + f"{'+'.join(event.get('signal_kinds') or [event['kind']]):<28} "
      + f"t={event['focus_s']:6.2f} id={event['track_id']:4d} "
      + f"d={event['d_rel']:5.1f} p={event['shadow_peak']:.2f} {event['log']}"
    )
  print(f"shown {min(len(events), args.limit)}/{len(events)}")
  return 0


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Build and review video-grounded radar cut-in labels")
  subparsers = parser.add_subparsers(dest="command", required=True)

  build = subparsers.add_parser("build", help="mine physical dPath predictor CUT-IN events")
  build.add_argument("--log", action="append", type=Path, default=[])
  build.add_argument("--selection", type=Path)
  build.add_argument("--inventory", type=Path, help="scan usable logs from a corpus JSONL inventory")
  build.add_argument("--vehicle-pattern", help="case-insensitive vehicle-name regex for --inventory")
  build.add_argument("--split", action="append", choices=("train", "validation", "test"), default=[])
  build.add_argument("--exclude-annotations", type=Path, help="skip candidate/time ranges already reviewed")
  build.add_argument("--exclude-queue", action="append", type=Path, default=[], help="skip logs scanned by an earlier queue")
  build.add_argument("--merge-gap", type=float, default=1.5, help="seconds to merge flickering signals for one object")
  build.add_argument("--max-logs", type=int, default=0)
  build.add_argument("--max-logs-per-vehicle", type=int, default=0, help="diversify inventory scans across vehicle folders")
  build.add_argument("--max-events-per-log", type=int, default=0, help="retain at most this many top candidates from each log")
  build.add_argument("--max-events", type=int, default=100)
  build.add_argument("--output", type=Path, required=True)

  render = subparsers.add_parser("render", help="render compact five-frame contact sheets")
  render.add_argument("--queue", type=Path, required=True)
  render.add_argument("--output-dir", type=Path, required=True)
  render.add_argument("--status", choices=("pending", "reviewed"), default="pending")
  render.add_argument("--limit", type=int, default=10)

  mark = subparsers.add_parser("mark", help="store a human/video verdict")
  mark.add_argument("--queue", type=Path, required=True)
  mark.add_argument("--id", required=True)
  mark.add_argument("--verdict", choices=VERDICTS, required=True)
  mark.add_argument("--confidence", choices=CONFIDENCES, required=True)
  mark.add_argument("--track-id", type=int)
  mark.add_argument("--start-s", type=float, help="correct the visually observed event start")
  mark.add_argument("--end-s", type=float, help="correct the visually observed event end")
  mark.add_argument("--note", default="")

  listing = subparsers.add_parser("list", help="list review events")
  listing.add_argument("--queue", type=Path, required=True)
  listing.add_argument("--status", choices=("all", "pending", "reviewed"), default="pending")
  listing.add_argument("--limit", type=int, default=10)

  return parser.parse_args()


def main() -> int:
  args = parse_args()
  return {
    "build": build_queue,
    "render": render_queue,
    "mark": mark_event,
    "list": list_queue,
  }[args.command](args)


if __name__ == "__main__":
  raise SystemExit(main())
