#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path


def add_repo_paths() -> None:
  cur = Path(__file__).resolve()
  for parent in cur.parents:
    opendbc_repo = parent / "opendbc_repo"
    if opendbc_repo.is_dir():
      sys.path.insert(0, str(opendbc_repo))
      sys.path.insert(0, str(parent))
      return


add_repo_paths()

from opendbc.car.logreader import LogReader  # noqa: E402


START_ADDR = 0x235
END_ADDR = 0x248
GROUP_180_START_ADDR = 0x180
GROUP_180_END_ADDR = 0x184
DEFAULT_STALE_S = 0.25
WHEEL_SPEED_ADDR = 0xA0
KPH_TO_MS = 1000.0 / 3600.0
RAW_CAN_BUS = 1
VEHICLE_WIDTH_M = 1.82
VEHICLE_LENGTH_M = 4.35
RADAR_MOVING_VEHICLE_MIN_SPEED_KPH = 8.0
PLOT_HEADING_MIN_SPEED_KPH = 1.0
PLOT_HEADING_COMPONENT_MIN_MPS = 0.5
CORNER_RADAR_ENDPOINT_SPEED_MIN_MPS = 0.5
CLUSTER_DEFAULT_VEHICLE = (70, 78, 88)
CLUSTER_PRIMARY_VEHICLE = (50, 66, 82)
CLUSTER_AMBER = (244, 172, 54)
CLUSTER_RED = (222, 72, 64)
CLUSTER_GREEN = (20, 188, 104)
CLUSTER_ORANGE = (230, 132, 42)
LABEL_FIELD_CHOICES = (
  "slot",
  "id",
  "speed",
  "x",
  "y",
  "obj_width",
  "obj_class",
  "quality",
  "vx",
  "vy",
  "age",
  "addr",
)


@dataclass(frozen=True)
class CornerObject:
  t: float
  group: str
  address: int
  slot: int
  quality: int
  age: int
  object_id: int
  object_class: int
  width: float
  x: float
  y: float
  vx: float
  vy: float
  ax: float

  @property
  def speed(self) -> float:
    return math.hypot(self.vx, self.vy)


@dataclass(frozen=True)
class SummaryCornerObject:
  t: float
  source: str
  label: str
  detect: float
  x: float
  y: float


@dataclass(frozen=True)
class Snapshot:
  t: float
  video_t: float
  rlog_path: Path
  log_index: int
  objects: list[CornerObject]
  summary_objects: list[SummaryCornerObject]
  ego_speed: float


class VideoSampler:
  def __init__(self, path: Path, fallback_fps: float, offset: float):
    import cv2

    self.cv2 = cv2
    self.path = path
    self.offset = offset
    self.cap = cv2.VideoCapture(str(path))
    if not self.cap.isOpened():
      raise RuntimeError(f"Could not open video: {path}")
    native_fps = self.cap.get(cv2.CAP_PROP_FPS)
    self.fps = native_fps if native_fps and native_fps > 1e-3 else fallback_fps

  def frame_at(self, t: float):
    target_t = max(0.0, t + self.offset)
    self.cap.set(self.cv2.CAP_PROP_POS_FRAMES, int(round(target_t * self.fps)))
    ok, frame = self.cap.read()
    if not ok:
      return None
    return self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2RGB)


def dbc_unsigned(data: bytes, start: int, length: int) -> int:
  return (int.from_bytes(data, "little") >> start) & ((1 << length) - 1)


def dbc_signed(data: bytes, start: int, length: int) -> int:
  raw = dbc_unsigned(data, start, length)
  if raw & (1 << (length - 1)):
    raw -= 1 << length
  return raw


def decode_object_at(t: float, group: str, address: int, slot: int, data: bytes, base: int) -> CornerObject:
  return CornerObject(
    t=t,
    group=group,
    address=address,
    slot=slot,
    quality=dbc_unsigned(data, base + 0, 7),
    age=dbc_unsigned(data, base + 8, 8),
    object_id=dbc_unsigned(data, base + 20, 7),
    object_class=dbc_unsigned(data, base + 36, 3),
    width=dbc_unsigned(data, base + 28, 7) * 0.05,
    x=dbc_unsigned(data, base + 40, 13) * 0.05,
    y=dbc_unsigned(data, base + 54, 12) * 0.05 - 102.4,
    vx=dbc_unsigned(data, base + 67, 12) * 0.05 - 100.0,
    vy=dbc_unsigned(data, base + 80, 10) * 0.05 - 25.0,
    ax=dbc_signed(data, base + 91, 9) * 0.05,
  )


def decode_corner_objects(t: float, address: int, data: bytes, args: argparse.Namespace) -> list[CornerObject]:
  if args.profile in ("auto", "180"):
    if not GROUP_180_START_ADDR <= address <= GROUP_180_END_ADDR or len(data) != 32:
      if args.profile == "180":
        return []
    else:
      base_slot = (address - GROUP_180_START_ADDR) * 2
      return [
        decode_object_at(t, "180", address, base_slot, data, 24),
        decode_object_at(t, "180", address, base_slot + 1, data, 152),
      ]

  if args.profile in ("auto", "235") and START_ADDR <= address <= END_ADDR and len(data) == 32:
    return [decode_object_at(t, "235", address, address - START_ADDR, data, 24)]
  return []


def decode_wheel_speed_mps(data: bytes) -> float:
  speeds_kph = [dbc_unsigned(data, start, 16) * 0.03125 for start in (64, 80, 96, 112)]
  return sum(speeds_kph) / len(speeds_kph) * KPH_TO_MS


def is_valid_object(obj: CornerObject, min_quality: int, max_x: float, max_abs_y: float) -> bool:
  if obj.quality < min_quality:
    return False
  if not 0.2 <= obj.x <= max_x:
    return False
  if abs(obj.y) > max_abs_y:
    return False
  # Empty slots decode to these offset defaults.
  if obj.vx <= -99.0 and obj.x < 0.5:
    return False
  return True


def find_repo_root() -> Path:
  cur = Path(__file__).resolve()
  for parent in cur.parents:
    if (parent / "opendbc_repo").is_dir():
      return parent
  raise RuntimeError("Could not find repo root containing opendbc_repo")


def load_summary_dbc():
  import cantools

  dbc_path = find_repo_root() / "opendbc_repo" / "opendbc" / "dbc" / "generator" / "hyundai" / "hyundai_canfd.dbc"
  return cantools.database.load_file(str(dbc_path), strict=False)


def decode_summary_corners(t: float, source: str, values: dict[str, float], args: argparse.Namespace) -> list[SummaryCornerObject]:
  objects: list[SummaryCornerObject] = []
  for label, forward_sign, lateral_sign in (
    ("LF", 1.0, 1.0),
    ("RF", 1.0, -1.0),
    ("LR", -1.0, 1.0),
    ("RR", -1.0, -1.0),
  ):
    detect = float(values.get(f"{label}_DETECT", 0.0))
    distance = float(values.get(f"{label}_DETECT_DISTANCE", 0.0))
    lateral = float(values.get(f"{label}_DETECT_LATERAL", 0.0))
    if detect <= 0.0 or not 0.2 <= distance <= args.summary_max_distance:
      continue

    objects.append(SummaryCornerObject(
      t=t,
      source=source,
      label=label,
      detect=detect,
      x=forward_sign * distance,
      y=lateral_sign * lateral,
    ))
  return objects


def resolve_existing_rlog(path: Path) -> Path | None:
  if path.exists():
    return path
  if path.name == "rlog":
    zst = path.with_name("rlog.zst")
    return zst if zst.exists() else None
  if path.name == "rlog.zst":
    plain = path.with_name("rlog")
    return plain if plain.exists() else None
  return None


def discover_rlog_playlist(initial_rlog: Path, max_logs: int | None) -> list[Path]:
  first = resolve_existing_rlog(initial_rlog)
  if first is None:
    raise FileNotFoundError(initial_rlog)

  paths = [first]
  match = re.match(r"^(.*?)(\d+)$", first.parent.name)
  if match is None:
    return paths

  prefix, digits = match.groups()
  width = len(digits)
  number = int(digits)
  limit = max_logs if max_logs is not None and max_logs > 0 else None

  while limit is None or len(paths) < limit:
    number += 1
    next_dir = first.parent.with_name(f"{prefix}{number:0{width}d}")
    next_path = resolve_existing_rlog(next_dir / first.name)
    if next_path is None:
      break
    paths.append(next_path)

  return paths


def get_video_duration(args: argparse.Namespace, rlog_path: Path) -> float | None:
  video_path = resolve_video_path(args, rlog_path)
  if video_path is None:
    return None

  try:
    import cv2
  except ImportError:
    return None

  cap = cv2.VideoCapture(str(video_path))
  try:
    if not cap.isOpened():
      return None
    fps = cap.get(cv2.CAP_PROP_FPS)
    frame_count = cap.get(cv2.CAP_PROP_FRAME_COUNT)
    if not fps or fps <= 1e-3 or not frame_count or frame_count <= 0:
      return None
    return float(frame_count) / float(fps)
  finally:
    cap.release()


def normalize_video_duration(duration: float, plot_fps: float) -> float:
  nearest_second = round(duration)
  tolerance = max(0.1, 1.0 / max(plot_fps, 1e-3))
  if abs(duration - nearest_second) <= tolerance:
    return float(nearest_second)
  return duration


def get_log_end_t(args: argparse.Namespace, rlog_path: Path, start_t: float) -> float | None:
  end_t = None if args.duration is None else start_t + args.duration
  if args.no_video_duration_limit:
    return end_t

  video_duration = get_video_duration(args, rlog_path)
  if video_duration is None:
    return end_t

  video_end_t = max(0.0, normalize_video_duration(video_duration, args.fps) - args.video_offset)
  return video_end_t if end_t is None else min(end_t, video_end_t)


def read_snapshots_from_log(args: argparse.Namespace, rlog_path: Path, log_index: int) -> list[Snapshot]:
  latest: dict[tuple[str, int], CornerObject] = {}
  latest_summary: dict[str, SummaryCornerObject] = {}
  snapshots: list[Snapshot] = []
  first_data_t: int | None = None
  start_t = args.start if log_index == 0 else 0.0
  end_t = get_log_end_t(args, rlog_path, start_t)
  next_frame_t = start_t
  frame_dt = 1.0 / args.fps
  latest_ego_speed = 0.0
  summary_dbc = None if args.no_summary_corners else load_summary_dbc()
  summary_messages = {} if summary_dbc is None else {
    0x162: summary_dbc.get_message_by_frame_id(0x162),
    0x1ea: summary_dbc.get_message_by_frame_id(0x1ea),
  }

  for msg in LogReader(str(rlog_path), only_union_types=True):
    msg_type = msg.which()
    if msg_type not in ("can", "carState"):
      continue

    if first_data_t is None:
      first_data_t = msg.logMonoTime

    t = (msg.logMonoTime - first_data_t) / 1e9
    if t < start_t:
      continue
    if end_t is not None and t >= end_t:
      break

    if msg_type == "carState":
      latest_ego_speed = float(msg.carState.vEgo)
      continue

    for can in msg.can:
      if can.src == args.speed_bus and can.address == WHEEL_SPEED_ADDR and len(can.dat) == 24:
        latest_ego_speed = decode_wheel_speed_mps(bytes(can.dat))

      if can.src == RAW_CAN_BUS:
        for obj in decode_corner_objects(t, can.address, bytes(can.dat), args):
          key = (obj.group, obj.slot)
          if is_valid_object(obj, args.min_quality, args.max_x, args.max_abs_y):
            latest[key] = obj
          else:
            latest.pop(key, None)

      if summary_dbc is not None and can.src == args.summary_bus and can.address in summary_messages and len(can.dat) == 32:
        values = summary_messages[can.address].decode(bytes(can.dat), decode_choices=False)
        source = f"0x{can.address:x}"
        for obj in decode_summary_corners(t, source, values, args):
          latest_summary[obj.label] = obj

    while t >= next_frame_t and (end_t is None or next_frame_t < end_t):
      active = [obj for obj in latest.values() if t - obj.t <= args.stale]
      active.sort(key=lambda o: (o.group, o.slot))
      summary_active = [obj for obj in latest_summary.values() if t - obj.t <= args.summary_stale]
      summary_active.sort(key=lambda o: o.label)
      snapshots.append(Snapshot(next_frame_t, next_frame_t, rlog_path, log_index, active, summary_active, latest_ego_speed))
      next_frame_t += frame_dt

  return snapshots


def read_snapshots(args: argparse.Namespace) -> list[Snapshot]:
  playlist = discover_rlog_playlist(Path(args.rlog), args.max_logs)
  if len(playlist) > 1:
    print("playlist:")
    for path in playlist:
      print(f"  {path}")

  snapshots: list[Snapshot] = []
  for log_index, rlog_path in enumerate(playlist):
    log_snapshots = read_snapshots_from_log(args, rlog_path, log_index)
    print(f"log {log_index + 1}/{len(playlist)}: {rlog_path} snapshots={len(log_snapshots)}")
    snapshots.extend(log_snapshots)
  return snapshots


def setup_axes(ax, args: argparse.Namespace) -> None:
  ax.set_title(f"Hyundai {args.profile.upper()} corner radar candidates")
  ax.set_xlabel("lateral y, vehicle left + [m]")
  ax.set_ylabel("longitudinal x, forward + [m]")
  # Top view: vehicle left should appear on the left side of the screen.
  ax.set_xlim(args.max_abs_y, -args.max_abs_y)
  ax.set_ylim(args.min_plot_x, args.max_x)
  vertical_range = args.max_x - args.min_plot_x
  horizontal_range = 2.0 * args.max_abs_y
  ax.set_aspect("auto")
  ax.set_box_aspect(vertical_range / (horizontal_range * args.plot_width_scale))
  ax.grid(True, alpha=0.25)
  ax.axhline(0.0, color="0.65", linewidth=0.8)
  ax.axvline(0.0, color="0.65", linewidth=0.8)

  draw_vehicle_marker(ax, 0.0, 0.0, 0.0, 1.0, CLUSTER_PRIMARY_VEHICLE, 1.0, label="ego")


def plot_y(obj: CornerObject, args: argparse.Namespace) -> float:
  return -obj.y if args.flip_raw_y else obj.y


def plot_vy(obj: CornerObject, args: argparse.Namespace) -> float:
  return -obj.vy if args.flip_raw_y else obj.vy


def clamp(value: float, low: float, high: float) -> float:
  return max(low, min(high, value))


def blend_color(color: tuple[int, int, int], target: tuple[int, int, int], amount: float) -> tuple[int, int, int]:
  amount = clamp(amount, 0.0, 1.0)
  return tuple(int(round(channel + (target_channel - channel) * amount)) for channel, target_channel in zip(color, target))


def lighten(color: tuple[int, int, int], amount: float) -> tuple[int, int, int]:
  return blend_color(color, (255, 255, 255), amount)


def darken(color: tuple[int, int, int], amount: float) -> tuple[int, int, int]:
  return blend_color(color, (0, 0, 0), amount)


def mpl_rgba(color: tuple[int, int, int], alpha: float = 1.0) -> tuple[float, float, float, float]:
  return color[0] / 255.0, color[1] / 255.0, color[2] / 255.0, clamp(alpha, 0.0, 1.0)


def normalize2(x: float, y: float) -> tuple[float, float]:
  length = math.hypot(x, y)
  if length <= 0.0001:
    return 0.0, 1.0
  return x / length, y / length


def vehicle_heading_from_velocity(forward_speed_mps: float | None, lateral_speed_mps: float | None, default_heading: tuple[float, float]) -> tuple[float, float]:
  if forward_speed_mps is None and lateral_speed_mps is None:
    return default_heading
  forward_speed = forward_speed_mps or 0.0
  lateral_speed = lateral_speed_mps or 0.0
  if abs(forward_speed) < PLOT_HEADING_COMPONENT_MIN_MPS or abs(lateral_speed) < PLOT_HEADING_COMPONENT_MIN_MPS:
    return default_heading
  if math.hypot(forward_speed, lateral_speed) * 3.6 < PLOT_HEADING_MIN_SPEED_KPH:
    return default_heading
  return normalize2(lateral_speed, forward_speed)


def adjusted_vehicle_center(ref_y: float, ref_x: float, forward_y: float, forward_x: float, relative_forward_speed_mps: float | None, anchor: str) -> tuple[float, float]:
  if anchor == "center":
    return ref_y, ref_x
  if anchor in ("rear-center", "rear"):
    return ref_y + forward_y * VEHICLE_LENGTH_M * 0.5, ref_x + forward_x * VEHICLE_LENGTH_M * 0.5
  if anchor in ("front-center", "front"):
    return ref_y - forward_y * VEHICLE_LENGTH_M * 0.5, ref_x - forward_x * VEHICLE_LENGTH_M * 0.5
  rel_speed = relative_forward_speed_mps or 0.0
  if abs(rel_speed) < CORNER_RADAR_ENDPOINT_SPEED_MIN_MPS:
    return ref_y, ref_x
  endpoint_sign = 1.0 if anchor == "endpoint" else -1.0
  shift_m = endpoint_sign * (VEHICLE_LENGTH_M * 0.5 if rel_speed > 0.0 else -VEHICLE_LENGTH_M * 0.5)
  return ref_y + forward_y * shift_m, ref_x + forward_x * shift_m


def pixels_per_meter(ax, center_y: float, center_x: float) -> tuple[float, float]:
  center_px = ax.transData.transform((center_y, center_x))
  lateral_px = ax.transData.transform((center_y + 1.0, center_x))
  longitudinal_px = ax.transData.transform((center_y, center_x + 1.0))
  return (
    max(1.0, math.hypot(lateral_px[0] - center_px[0], lateral_px[1] - center_px[1])),
    max(1.0, math.hypot(longitudinal_px[0] - center_px[0], longitudinal_px[1] - center_px[1])),
  )


def vehicle_screen_corners(ax, center_y: float, center_x: float, forward_y: float, forward_x: float, width_m: float = VEHICLE_WIDTH_M, length_m: float = VEHICLE_LENGTH_M) -> list[tuple[float, float]]:
  center_px = ax.transData.transform((center_y, center_x))
  forward_px = ax.transData.transform((center_y + forward_y, center_x + forward_x))
  forward_dx = forward_px[0] - center_px[0]
  forward_dy = forward_px[1] - center_px[1]
  forward_len = math.hypot(forward_dx, forward_dy)
  if forward_len <= 0.0001:
    forward_dx, forward_dy = 0.0, 1.0
  else:
    forward_dx, forward_dy = forward_dx / forward_len, forward_dy / forward_len

  right_dx, right_dy = forward_dy, -forward_dx
  lateral_px_per_m, longitudinal_px_per_m = pixels_per_meter(ax, center_y, center_x)
  vehicle_px_per_m = math.sqrt(lateral_px_per_m * longitudinal_px_per_m)
  half_width_px = width_m * vehicle_px_per_m * 0.5
  half_length_px = length_m * vehicle_px_per_m * 0.5
  display_points = []
  for local_width_px, local_length_px in (
    (-half_width_px, -half_length_px),
    (half_width_px, -half_length_px),
    (half_width_px, half_length_px),
    (-half_width_px, half_length_px),
  ):
    display_points.append((
      center_px[0] + right_dx * local_width_px + forward_dx * local_length_px,
      center_px[1] + right_dy * local_width_px + forward_dy * local_length_px,
    ))
  return [tuple(ax.transData.inverted().transform(point)) for point in display_points]


def inset_polygon(points: list[tuple[float, float]], amount: float) -> list[tuple[float, float]]:
  center_y = sum(point[0] for point in points) / len(points)
  center_x = sum(point[1] for point in points) / len(points)
  return [
    (point_y + (center_y - point_y) * amount, point_x + (center_x - point_x) * amount)
    for point_y, point_x in points
  ]


def draw_vehicle_marker(ax, center_y: float, center_x: float, forward_y: float, forward_x: float, color: tuple[int, int, int], confidence: float, label: str | None = None) -> None:
  alpha = (92 + 163 * clamp(confidence, 0.0, 1.0)) / 255.0
  body = vehicle_screen_corners(ax, center_y, center_x, forward_y, forward_x)
  shadow = vehicle_screen_corners(ax, center_y, center_x, forward_y, forward_x, VEHICLE_WIDTH_M * 1.12, VEHICLE_LENGTH_M * 1.08)
  ax.add_patch(plt.Polygon(shadow, closed=True, facecolor=mpl_rgba((0, 0, 0), 0.12 + 0.20 * confidence), edgecolor="none", zorder=2))
  ax.add_patch(plt.Polygon(body, closed=True, facecolor=mpl_rgba(color, alpha), edgecolor=mpl_rgba(darken(color, 0.42), alpha), linewidth=1.1, zorder=4))
  ax.add_patch(plt.Polygon(inset_polygon(body, 0.24), closed=True, facecolor=mpl_rgba(lighten(color, 0.16), min(0.92, alpha)), edgecolor="none", zorder=5))

  nose = vehicle_screen_corners(ax, center_y, center_x, forward_y, forward_x, VEHICLE_WIDTH_M * 0.52, VEHICLE_LENGTH_M * 0.18)
  center_px = ax.transData.transform((center_y, center_x))
  forward_px = ax.transData.transform((center_y + forward_y, center_x + forward_x))
  forward_dx = forward_px[0] - center_px[0]
  forward_dy = forward_px[1] - center_px[1]
  forward_len = math.hypot(forward_dx, forward_dy)
  if forward_len > 0.0001:
    forward_dx, forward_dy = forward_dx / forward_len, forward_dy / forward_len
  lateral_px_per_m, longitudinal_px_per_m = pixels_per_meter(ax, center_y, center_x)
  nose_shift_px = VEHICLE_LENGTH_M * math.sqrt(lateral_px_per_m * longitudinal_px_per_m) * 0.38
  nose = [
    tuple(ax.transData.inverted().transform((
      ax.transData.transform((point_y, point_x))[0] + forward_dx * nose_shift_px,
      ax.transData.transform((point_y, point_x))[1] + forward_dy * nose_shift_px,
    )))
    for point_y, point_x in nose
  ]
  ax.add_patch(plt.Polygon(nose, closed=True, facecolor=mpl_rgba(lighten(color, 0.34), min(0.95, alpha)), edgecolor="none", zorder=6))

  if label:
    ax.text(center_y, center_x, label, ha="center", va="center", fontsize=8, color="white", zorder=7)


def object_vehicle_color(obj: CornerObject, ego_speed: float) -> tuple[int, int, int]:
  absolute_speed_kph = (ego_speed + obj.vx) * 3.6
  if absolute_speed_kph <= -RADAR_MOVING_VEHICLE_MIN_SPEED_KPH:
    return CLUSTER_RED
  if abs(obj.vy) * 3.6 >= RADAR_MOVING_VEHICLE_MIN_SPEED_KPH:
    return CLUSTER_AMBER
  return CLUSTER_DEFAULT_VEHICLE


def color_slot(obj: CornerObject) -> int:
  return obj.slot + (20 if obj.group == "180" else 0)


def snapshot_profile_label(snapshot: Snapshot, args: argparse.Namespace) -> str:
  if args.profile != "auto":
    return f"0x{args.profile}"
  groups = sorted({obj.group for obj in snapshot.objects})
  return "auto" if not groups else "auto " + "/".join(f"0x{group}" for group in groups)


def selected_label_fields(args: argparse.Namespace) -> tuple[str, ...]:
  if hasattr(args, "active_label_fields"):
    return tuple(args.active_label_fields)
  fields = []
  for field in args.label_fields.split(","):
    normalized = field.strip().lower()
    if normalized:
      if normalized == "width":
        normalized = "obj_width"
      elif normalized == "class":
        normalized = "obj_class"
      fields.append(normalized)
  return tuple(fields)


def format_object_label(obj: CornerObject, ego_speed: float, obj_y: float, fields: tuple[str, ...]) -> str:
  values = {
    "slot": f"{obj.group}:{obj.slot:02d}",
    "addr": f"0x{obj.address:x}",
    "group": obj.group,
    "id": f"id={obj.object_id}",
    "speed": f"v={(ego_speed + obj.vx) * 3.6:.1f}kph",
    "vx": f"vx={obj.vx:+.1f}",
    "vy": f"vy={obj.vy:+.1f}",
    "x": f"x={obj.x:.1f}m",
    "y": f"y={obj_y:+.1f}m",
    "width": f"w={obj.width:.2f}m",
    "obj_width": f"w={obj.width:.2f}m",
    "class": f"c={obj.object_class}",
    "obj_class": f"c={obj.object_class}",
    "quality": f"q={obj.quality}",
    "age": f"age={obj.age}",
  }
  return " ".join(values[field] for field in fields if field in values)


def draw_snapshot(ax, snapshot: Snapshot, args: argparse.Namespace, paused: bool = False) -> None:
  ax.clear()
  setup_axes(ax, args)
  t = snapshot.t
  objects = snapshot.objects
  summary_objects = snapshot.summary_objects
  ego_speed = snapshot.ego_speed
  pause_label = "  PAUSED" if paused else ""
  ax.set_title(
    f"Hyundai {snapshot_profile_label(snapshot, args)} corner radar candidates  "
    f"log={snapshot.log_index} t={t:.2f}s video={snapshot.video_t:.2f}s  vEgo={ego_speed * 3.6:.1f}kph  "
    f"raw={len(objects)} summary={len(summary_objects)}{pause_label}"
  )

  if objects:
    label_fields = selected_label_fields(args)
    for obj in objects:
      obj_y = plot_y(obj, args)
      absolute_forward_speed_mps = ego_speed + obj.vx
      lateral_speed_mps = plot_vy(obj, args)
      forward_y, forward_x = vehicle_heading_from_velocity(absolute_forward_speed_mps, lateral_speed_mps, (0.0, 1.0))
      center_y, center_x = adjusted_vehicle_center(obj_y, obj.x, forward_y, forward_x, obj.vx, args.point_anchor)
      confidence = clamp(0.56 + min(100, max(0, obj.quality)) / 100.0 * 0.36, 0.56, 0.92)
      draw_vehicle_marker(ax, center_y, center_x, forward_y, forward_x, object_vehicle_color(obj, ego_speed), confidence)
      ax.arrow(
        center_y,
        center_x,
        lateral_speed_mps * args.velocity_scale,
        absolute_forward_speed_mps * args.velocity_scale,
        head_width=0.35,
        head_length=0.7,
        length_includes_head=True,
        color="tab:red",
        alpha=0.75,
        zorder=8,
      )
      label = format_object_label(obj, ego_speed, obj_y, label_fields)
      if label:
        ax.text(
          center_y,
          center_x + 2.6,
          label,
          ha="center",
          va="bottom",
          fontsize=8,
          zorder=9,
        )
    ax.plot([], [], color=mpl_rgba(CLUSTER_DEFAULT_VEHICLE, 0.9), linewidth=6, label="raw corner vehicle")

  if summary_objects:
    front = [obj for obj in summary_objects if obj.x >= 0.0]
    rear = [obj for obj in summary_objects if obj.x < 0.0]
    for obj in front:
      draw_vehicle_marker(ax, obj.y, obj.x, 0.0, 1.0, CLUSTER_GREEN, 0.78)
    for obj in rear:
      draw_vehicle_marker(ax, obj.y, obj.x, 0.0, -1.0, CLUSTER_ORANGE, 0.78)
    if front:
      ax.plot([], [], color=mpl_rgba(CLUSTER_GREEN, 0.9), linewidth=6, label="0x162/0x1ea front")
    if rear:
      ax.plot([], [], color=mpl_rgba(CLUSTER_ORANGE, 0.9), linewidth=6, label="0x162/0x1ea rear")
    for obj in summary_objects:
      ax.text(obj.y, obj.x - 2.7 if obj.x < 0 else obj.x + 2.7, f"{obj.label} {obj.source}", ha="center", va="top" if obj.x < 0 else "bottom", fontsize=8, zorder=9)

  if objects or summary_objects:
    ax.legend(loc="lower right")


def resolve_video_path(args: argparse.Namespace, rlog_path: Path) -> Path | None:
  if args.video.lower() in ("none", "off", "false", "0"):
    return None
  if args.video.lower() != "auto":
    return Path(args.video)

  for name in ("qcamera.ts", "fcamera.ts", "ecamera.ts", "dcamera.ts"):
    candidate = rlog_path.with_name(name)
    if candidate.exists():
      return candidate
  return None


def setup_video_axis(ax, t: float, video: VideoSampler | None) -> None:
  ax.clear()
  ax.axis("off")
  if video is None:
    ax.set_title("video: none")
    return

  frame = video.frame_at(t)
  ax.set_title(f"{video.path.name}  t={t + video.offset:.2f}s")
  if frame is None:
    ax.text(0.5, 0.5, "no video frame", ha="center", va="center", transform=ax.transAxes)
    return
  ax.imshow(frame)


def plot_snapshots(snapshots: list[Snapshot], args: argparse.Namespace) -> None:
  global plt
  import matplotlib.pyplot as plt
  from matplotlib.animation import FuncAnimation
  from matplotlib.widgets import CheckButtons

  if not snapshots:
    raise RuntimeError("No corner radar objects decoded. Check --start, --duration, or filter thresholds.")

  video_cache: dict[Path, VideoSampler | None] = {}

  def get_video(rlog_path: Path) -> VideoSampler | None:
    if rlog_path not in video_cache:
      video_path = resolve_video_path(args, rlog_path)
      video_cache[rlog_path] = VideoSampler(video_path, args.video_fps or args.fps, args.video_offset) if video_path is not None else None
      if video_cache[rlog_path] is not None:
        print(f"video: {video_cache[rlog_path].path} ({video_cache[rlog_path].fps:.2f} fps)")
    return video_cache[rlog_path]

  if args.save_png:
    first_video = get_video(snapshots[0].rlog_path)
    if first_video is not None:
      fig, (ax, video_ax) = plt.subplots(1, 2, figsize=(16, 8), gridspec_kw={"width_ratios": [1.0, 1.25]})
    else:
      fig, ax = plt.subplots(figsize=(9, 9))
      video_ax = None
    idx = min(range(len(snapshots)), key=lambda i: abs(snapshots[i].t - args.snapshot_time))
    snapshot = snapshots[idx]
    draw_snapshot(ax, snapshot, args)
    if video_ax is not None:
      setup_video_axis(video_ax, snapshot.video_t, get_video(snapshot.rlog_path))
    fig.tight_layout()
    fig.savefig(args.save_png, dpi=160)
    print(f"saved {args.save_png}")
    return

  first_video = get_video(snapshots[0].rlog_path)
  if first_video is not None:
    fig, (ax, video_ax, controls_ax) = plt.subplots(1, 3, figsize=(18, 8), gridspec_kw={"width_ratios": [1.0, 1.25, 0.34]})
  else:
    fig, (ax, controls_ax) = plt.subplots(1, 2, figsize=(11, 9), gridspec_kw={"width_ratios": [1.0, 0.25]})
    video_ax = None
  controls_ax.set_title("labels")
  controls_ax.set_xticks([])
  controls_ax.set_yticks([])

  initial_fields = set(selected_label_fields(args))
  args.active_label_fields = [field for field in LABEL_FIELD_CHOICES if field in initial_fields]
  label_checks = CheckButtons(
    controls_ax,
    LABEL_FIELD_CHOICES,
    [field in initial_fields for field in LABEL_FIELD_CHOICES],
  )

  paused = {"value": False}
  current_frame = {"idx": 0}
  ani_holder: dict[str, FuncAnimation] = {}

  def update(frame_idx: int):
    current_frame["idx"] = frame_idx
    snapshot = snapshots[frame_idx]
    draw_snapshot(ax, snapshot, args, paused["value"])
    if video_ax is not None:
      setup_video_axis(video_ax, snapshot.video_t, get_video(snapshot.rlog_path))
    return []

  def on_click(event):
    if event.canvas != fig.canvas:
      return
    if event.inaxes == controls_ax:
      return
    paused["value"] = not paused["value"]
    ani = ani_holder.get("ani")
    if ani is None:
      return
    if paused["value"]:
      ani.event_source.stop()
    else:
      ani.event_source.start()
    update(current_frame["idx"])
    fig.canvas.draw_idle()

  def on_label_toggle(label: str):
    active = [
      field
      for field, status in zip(LABEL_FIELD_CHOICES, label_checks.get_status())
      if status
    ]
    args.active_label_fields = active
    update(current_frame["idx"])
    fig.canvas.draw_idle()

  ani = FuncAnimation(fig, update, frames=len(snapshots), interval=1000 / args.fps, blit=False, repeat=True)
  ani_holder["ani"] = ani
  fig.canvas.mpl_connect("button_press_event", on_click)
  label_checks.on_clicked(on_label_toggle)
  if args.save_gif:
    ani.save(args.save_gif, writer="pillow", fps=args.fps)
    print(f"saved {args.save_gif}")
    return

  plt.show()


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Plot candidate Hyundai CAN-FD corner radar objects from bus 1 rlog data. Click the plot to pause/resume playback.",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  )
  parser.add_argument("rlog", help="Path to rlog.zst")
  parser.add_argument("--profile", choices=("auto", "235", "180"), default="auto", help="Raw corner radar decode profile")
  parser.add_argument("--speed-bus", type=int, default=0, help="Bus containing WHEEL_SPEEDS 0xA0 for vEgo fallback")
  parser.add_argument("--start", type=float, default=0.0, help="Start time in seconds from the beginning of the rlog")
  parser.add_argument("--duration", type=float, default=0.0, help="Seconds to read; 0 reads the whole rlog")
  parser.add_argument("--max-logs", type=int, default=0, help="Maximum number of incrementing segment logs to read; 0 reads until the next segment is missing")
  parser.add_argument("--fps", type=float, default=15.0)
  parser.add_argument("--stale", type=float, default=DEFAULT_STALE_S)
  parser.add_argument("--min-quality", type=int, default=1)
  parser.add_argument("--max-x", type=float, default=120.0)
  parser.add_argument("--min-plot-x", type=float, default=-30.0)
  parser.add_argument("--max-abs-y", type=float, default=20.0)
  parser.add_argument("--plot-width-scale", type=float, default=2.0, help="Visual width multiplier for the radar plot while keeping the same meter range")
  parser.add_argument("--velocity-scale", type=float, default=0.35, help="Arrow length multiplier")
  parser.add_argument("--point-anchor", choices=("center", "rear-center", "front-center", "endpoint", "endpoint-reversed"), default="rear-center", help="How to interpret raw corner radar object position before drawing the vehicle")
  parser.add_argument("--label-fields", default="slot,id,speed,y", help="Comma-separated raw object label fields: slot,addr,group,id,speed,vx,vy,x,y,width,obj_width,class,obj_class,quality,age")
  parser.add_argument("--snapshot-time", type=float, default=5.0)
  parser.add_argument("--flip-raw-y", action="store_true", help="Flip raw 0x235-0x248 RelPosY sign for comparison")
  parser.add_argument("--raw-y", action="store_true", help=argparse.SUPPRESS)
  parser.add_argument("--summary-bus", type=int, default=2, help="Bus for existing 0x162/0x1ea corner summary messages")
  parser.add_argument("--summary-stale", type=float, default=0.8)
  parser.add_argument("--summary-max-distance", type=float, default=80.0)
  parser.add_argument("--no-summary-corners", action="store_true", help="Hide existing 0x162/0x1ea LF/RF/LR/RR summary overlay")
  parser.add_argument("--video", default="auto", help="TS video path, auto for qcamera.ts next to rlog, or none to disable")
  parser.add_argument("--video-fps", type=float, default=None, help="Override video FPS when OpenCV cannot detect it")
  parser.add_argument("--video-offset", type=float, default=0.0, help="Seconds added to radar time when selecting the video frame")
  parser.add_argument("--no-video-duration-limit", action="store_true", help="Do not cap each rlog segment to the matching TS video duration")
  parser.add_argument("--save-png", default=None)
  parser.add_argument("--save-gif", default=None)
  args = parser.parse_args()
  if args.duration == 0:
    args.duration = None
  return args


if __name__ == "__main__":
  parsed_args = parse_args()
  plot_snapshots(read_snapshots(parsed_args), parsed_args)

