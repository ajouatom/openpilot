#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from statistics import median


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
EGO_LATERAL_COMP_MAX_MPS = 3.5
EGO_LATERAL_COMP_STATIC_SPEED_KPH = 8.0
CUTIN_DEFAULT_SENSITIVITY = 50.0
CUTIN_DEFAULT_HORIZON_S = 1.5
CUTIN_DEFAULT_CONFIRM_S = 0.20
CUTIN_DEFAULT_STICKY_S = 0.7
CUTIN_DEFAULT_MIN_AGE_S = 0.25
CUTIN_DEFAULT_ENTER_MIN_X_M = 1.0
CUTIN_DEFAULT_ENTER_MAX_X_M = 55.0
CUTIN_DEFAULT_ENTER_MIN_ABS_Y_M = 1.5
CUTIN_DEFAULT_KEEP_MIN_X_M = 0.5
CUTIN_DEFAULT_KEEP_MAX_X_M = 60.0
CUTIN_DEFAULT_ENTER_FUTURE_IN_LANE_PROB = 0.20
CUTIN_DEFAULT_ENTER_PROB_GAIN = 0.12
CUTIN_DEFAULT_ENTER_CENTERING_GAIN = 0.20
CUTIN_DEFAULT_KEEP_FUTURE_IN_LANE_PROB = 0.12
CUTIN_DEFAULT_KEEP_MAX_DPATH_FUTURE = 1.6
CUTIN_DEFAULT_KEEP_MAX_MOVING_AWAY = 0.3
CUTIN_DEFAULT_LANE_HALF_WIDTH_M = 1.8
CORNER_RADAR_OBJECT_AGE_HZ = 33.0
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


@dataclass(frozen=True)
class LogTiming:
  start_t: float
  end_t: float | None
  video_base_t: float


@dataclass(frozen=True)
class CutInInfo:
  count: int
  entering: bool
  keep: bool
  y_future: float
  x_future: float
  in_lane_prob: float
  in_lane_prob_future: float


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
  width_bits = 8 if group == "235" else 7
  width_factor = 0.01 if group == "235" else 0.05
  class_bits = 4 if group == "235" else 3
  return CornerObject(
    t=t,
    group=group,
    address=address,
    slot=slot,
    quality=dbc_unsigned(data, base + 0, 7),
    age=dbc_unsigned(data, base + 8, 8),
    object_id=dbc_unsigned(data, base + 20, 7),
    object_class=dbc_unsigned(data, base + 36, class_bits),
    width=dbc_unsigned(data, base + 28, width_bits) * width_factor,
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


def get_rlog_data_duration(rlog_path: Path) -> float | None:
  first_data_t: int | None = None
  last_data_t: int | None = None
  for msg in LogReader(str(rlog_path), only_union_types=True):
    if msg.which() not in ("can", "carState"):
      continue
    if first_data_t is None:
      first_data_t = msg.logMonoTime
    last_data_t = msg.logMonoTime
  if first_data_t is None or last_data_t is None:
    return None
  return max(0.0, (last_data_t - first_data_t) / 1e9)


def get_log_timing(args: argparse.Namespace, rlog_path: Path, log_index: int) -> LogTiming:
  video_base_t = 0.0
  video_duration = None
  if args.no_video_duration_limit:
    video_duration = None
  else:
    raw_video_duration = get_video_duration(args, rlog_path)
    video_duration = None if raw_video_duration is None else normalize_video_duration(raw_video_duration, args.fps)
    if video_duration is not None and args.video_trim == "front":
      rlog_duration = get_rlog_data_duration(rlog_path)
      if rlog_duration is not None:
        trim_tolerance = max(0.1, 1.0 / max(args.fps, 1e-3))
        excess_duration = rlog_duration - video_duration
        if excess_duration > trim_tolerance:
          video_base_t = excess_duration

  start_t = video_base_t + (args.start if log_index == 0 else 0.0)
  end_t = None if args.duration is None else start_t + args.duration
  if video_duration is not None:
    video_end_t = video_base_t + max(0.0, video_duration - args.video_offset)
    end_t = video_end_t if end_t is None else min(end_t, video_end_t)
  return LogTiming(start_t=start_t, end_t=end_t, video_base_t=video_base_t)


def read_snapshots_from_log(args: argparse.Namespace, rlog_path: Path, log_index: int) -> list[Snapshot]:
  latest: dict[tuple[str, int], CornerObject] = {}
  latest_summary: dict[str, SummaryCornerObject] = {}
  snapshots: list[Snapshot] = []
  first_data_t: int | None = None
  timing = get_log_timing(args, rlog_path, log_index)
  start_t = timing.start_t
  end_t = timing.end_t
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
      display_t = max(0.0, next_frame_t - timing.video_base_t)
      snapshots.append(Snapshot(display_t, display_t, rlog_path, log_index, active, summary_active, latest_ego_speed))
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


def object_key(obj: CornerObject) -> tuple[str, int]:
  return obj.group, obj.slot


def estimate_common_lateral_speed_mps(objects: list[CornerObject], ego_speed: float, args: argparse.Namespace) -> float:
  if args.no_ego_lateral_compensation:
    return 0.0
  candidates = [
    plot_vy(obj, args)
    for obj in objects
    if obj.quality >= args.min_quality
    and 2.5 <= obj.x <= args.max_x
    and abs((ego_speed + obj.vx) * 3.6) <= EGO_LATERAL_COMP_STATIC_SPEED_KPH
  ]
  if len(candidates) < 2:
    return 0.0
  return clamp(float(median(candidates)), -EGO_LATERAL_COMP_MAX_MPS, EGO_LATERAL_COMP_MAX_MPS)


def display_vy(obj: CornerObject, args: argparse.Namespace, lateral_speed_offset_mps: float) -> float:
  return plot_vy(obj, args) - lateral_speed_offset_mps


def clamp(value: float, low: float, high: float) -> float:
  return max(low, min(high, value))


def cutin_tuning_from_sensitivity(sensitivity: float) -> dict[str, float]:
  s = clamp(sensitivity, 0.0, 100.0)
  xp = [0.0, 50.0, 100.0]
  return {
    "cutin_horizon": float(np_interp(s, xp, [0.5, 1.5, 2.5])),
    "cutin_confirm": float(np_interp(s, xp, [0.25, 0.10, 0.06])),
    "cutin_min_age": float(np_interp(s, xp, [0.50, 0.25, 0.10])),
    "cutin_enter_min_x": float(np_interp(s, xp, [3.0, 1.0, 0.5])),
    "cutin_enter_max_x": float(np_interp(s, xp, [50.0, 55.0, 65.0])),
    "cutin_enter_min_abs_y": float(np_interp(s, xp, [1.9, 1.5, 1.2])),
    "cutin_enter_future_prob": float(np_interp(s, xp, [0.30, 0.15, 0.08])),
    "cutin_enter_centering_gain": float(np_interp(s, xp, [0.30, 0.18, 0.10])),
  }


def np_interp(x: float, xp: list[float], fp: list[float]) -> float:
  if x <= xp[0]:
    return fp[0]
  for idx in range(1, len(xp)):
    if x <= xp[idx]:
      scale = (x - xp[idx - 1]) / max(1e-6, xp[idx] - xp[idx - 1])
      return fp[idx - 1] + (fp[idx] - fp[idx - 1]) * scale
  return fp[-1]


def apply_cutin_sensitivity(args: argparse.Namespace, provided_flags: set[str] | None = None) -> None:
  if args.cutin_sensitivity <= 0.0:
    return
  provided_flags = provided_flags or set()
  for attr, value in cutin_tuning_from_sensitivity(args.cutin_sensitivity).items():
    flag = "--" + attr.replace("_", "-")
    if flag not in provided_flags:
      setattr(args, attr, value)


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


def draw_vehicle_label(ax, center_y: float, center_x: float, label: str, cutin: bool = False) -> None:
  if not label:
    return

  ax.annotate(
    label,
    xy=(center_y, center_x),
    xytext=(0, 12),
    textcoords="offset points",
    ha="center",
    va="bottom",
    fontsize=7.5,
    color="white" if cutin else "0.08",
    bbox={
      "boxstyle": "round,pad=0.18",
      "facecolor": mpl_rgba(CLUSTER_RED if cutin else (248, 250, 252), 0.88),
      "edgecolor": mpl_rgba((90, 96, 104), 0.45),
      "linewidth": 0.7,
    },
    zorder=10,
  )


def object_vehicle_color(obj: CornerObject, ego_speed: float, lateral_speed_mps: float | None = None) -> tuple[int, int, int]:
  absolute_speed_kph = (ego_speed + obj.vx) * 3.6
  if absolute_speed_kph <= -RADAR_MOVING_VEHICLE_MIN_SPEED_KPH:
    return CLUSTER_RED
  if abs(lateral_speed_mps if lateral_speed_mps is not None else obj.vy) * 3.6 >= RADAR_MOVING_VEHICLE_MIN_SPEED_KPH:
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


def lane_probability_from_y(y: float, lane_half_width_m: float) -> float:
  return clamp(1.0 - abs(y) / max(0.1, lane_half_width_m), 0.0, 1.0)


def cutin_object_info(
  obj: CornerObject,
  snapshot: Snapshot,
  obj_y: float,
  obj_vy: float,
  track_age_s: float,
  previous_count: int,
  args: argparse.Namespace,
) -> CutInInfo | None:
  if args.no_cutin:
    return None
  v_lead = snapshot.ego_speed + obj.vx
  horizon_s = max(0.0, args.cutin_horizon)
  y_future = obj_y + obj_vy * horizon_s
  x_future = obj.x + obj.vx * horizon_s
  in_lane_prob = lane_probability_from_y(obj_y, args.cutin_lane_half_width)
  in_lane_prob_future = lane_probability_from_y(y_future, args.cutin_lane_half_width)

  entering = (
    track_age_s >= args.cutin_min_age
    and args.cutin_enter_min_x < obj.x < args.cutin_enter_max_x
    and abs(obj_y) >= args.cutin_enter_min_abs_y
    and v_lead > 4.0
    and in_lane_prob_future >= args.cutin_enter_future_prob
    and (in_lane_prob_future - in_lane_prob) >= args.cutin_enter_prob_gain
    and (abs(obj_y) - abs(y_future)) >= args.cutin_enter_centering_gain
  )
  moving_away = abs(y_future) - abs(obj_y)
  keep = (
    previous_count > 0
    and args.cutin_keep_min_x < obj.x < args.cutin_keep_max_x
    and v_lead > 2.0
    and moving_away <= args.cutin_keep_max_moving_away
    and (
      in_lane_prob_future > args.cutin_keep_future_prob
      or abs(y_future) < args.cutin_keep_max_dpath_future
    )
  )
  return CutInInfo(
    count=previous_count,
    entering=entering,
    keep=keep,
    y_future=y_future,
    x_future=x_future,
    in_lane_prob=in_lane_prob,
    in_lane_prob_future=in_lane_prob_future,
  )


def sensor_track_age_s(obj: CornerObject) -> float:
  # The corner radar's ALIVE_AGE lets the plot recognize a target that was
  # already tracked before it entered the visible/interesting plot region.
  return max(0.0, obj.age / CORNER_RADAR_OBJECT_AGE_HZ)


def cutin_infos_for_snapshot(snapshots: list[Snapshot], frame_idx: int, args: argparse.Namespace) -> dict[tuple[str, int], CutInInfo]:
  if args.no_cutin:
    return {}
  counts: dict[tuple[str, int], int] = {}
  first_seen: dict[tuple[str, int], float] = {}
  last_seen: dict[tuple[str, int], float] = {}
  current_infos: dict[tuple[str, int], CutInInfo] = {}
  sticky_frames = max(1, int(round(args.cutin_sticky * args.fps)))
  confirm_frames = max(1, int(round(args.cutin_confirm * args.fps)))
  for idx, snapshot in enumerate(snapshots[:frame_idx + 1]):
    lateral_offset = estimate_common_lateral_speed_mps(snapshot.objects, snapshot.ego_speed, args)
    frame_infos: dict[tuple[str, int], CutInInfo] = {}
    for obj in snapshot.objects:
      key = object_key(obj)
      if key not in last_seen or snapshot.t - last_seen[key] > max(args.stale * 2.0, 0.25):
        first_seen[key] = snapshot.t
        counts[key] = 0
      last_seen[key] = snapshot.t
      obj_y = plot_y(obj, args)
      obj_vy = display_vy(obj, args, lateral_offset)
      previous_count = counts.get(key, 0)
      track_age_s = max(snapshot.t - first_seen[key], sensor_track_age_s(obj))
      info = cutin_object_info(obj, snapshot, obj_y, obj_vy, track_age_s, previous_count, args)
      if info is None:
        counts[key] = 0
        continue
      if info.entering:
        count = min(previous_count + 1, sticky_frames)
      elif info.keep:
        count = max(previous_count - 1, 0)
      else:
        count = 0
      counts[key] = count
      frame_infos[key] = CutInInfo(
        count=count,
        entering=info.entering,
        keep=info.keep,
        y_future=info.y_future,
        x_future=info.x_future,
        in_lane_prob=info.in_lane_prob,
        in_lane_prob_future=info.in_lane_prob_future,
      )
    if idx == frame_idx:
      current_infos = {
        key: info
        for key, info in frame_infos.items()
        if info.count >= confirm_frames
      }
  return current_infos


def draw_snapshot(ax, snapshot: Snapshot, args: argparse.Namespace, paused: bool = False, cutin_infos: dict[tuple[str, int], CutInInfo] | None = None) -> None:
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

  cutin_infos = cutin_infos or {}
  if objects:
    label_fields = selected_label_fields(args)
    lateral_speed_offset_mps = estimate_common_lateral_speed_mps(objects, ego_speed, args)
    for obj in objects:
      key = object_key(obj)
      cutin_info = cutin_infos.get(key)
      obj_y = plot_y(obj, args)
      absolute_forward_speed_mps = ego_speed + obj.vx
      lateral_speed_mps = display_vy(obj, args, lateral_speed_offset_mps)
      forward_y, forward_x = vehicle_heading_from_velocity(absolute_forward_speed_mps, lateral_speed_mps, (0.0, 1.0))
      center_y, center_x = adjusted_vehicle_center(obj_y, obj.x, forward_y, forward_x, obj.vx, args.point_anchor)
      confidence = clamp(0.56 + min(100, max(0, obj.quality)) / 100.0 * 0.36, 0.56, 0.92)
      color = CLUSTER_RED if cutin_info is not None else object_vehicle_color(obj, ego_speed, lateral_speed_mps)
      draw_vehicle_marker(ax, center_y, center_x, forward_y, forward_x, color, confidence, label="CUT" if cutin_info is not None else None)
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
      if cutin_info is not None:
        ax.plot(
          [obj_y, cutin_info.y_future],
          [obj.x, cutin_info.x_future],
          color=mpl_rgba(CLUSTER_RED, 0.86),
          linewidth=2.0,
          zorder=8,
        )
        ax.scatter([cutin_info.y_future], [cutin_info.x_future], s=48, color=mpl_rgba(CLUSTER_RED, 0.86), zorder=9)
      label = format_object_label(obj, ego_speed, obj_y, label_fields)
      if label:
        draw_vehicle_label(ax, center_y, center_x, label)
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
  from matplotlib.widgets import CheckButtons, Slider

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
    draw_snapshot(ax, snapshot, args, cutin_infos=cutin_infos_for_snapshot(snapshots, idx, args))
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
  fig.subplots_adjust(bottom=0.38)
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
  updating_seek = {"value": False}
  ani_holder: dict[str, FuncAnimation] = {}
  slider_specs = (
    ("cutin_sensitivity", "sensitivity", 0.0, 100.0, "%.0f"),
    ("cutin_horizon", "horizon", 0.0, 2.5, "%.2fs"),
    ("cutin_min_age", "min age", 0.0, 1.5, "%.2fs"),
    ("cutin_enter_min_x", "x min", 0.0, 5.0, "%.1fm"),
    ("cutin_enter_min_abs_y", "y min", 0.0, 3.5, "%.1fm"),
    ("cutin_lane_half_width", "lane half", 1.2, 2.4, "%.2fm"),
    ("cutin_enter_future_prob", "future prob", 0.0, 0.8, "%.2f"),
    ("cutin_enter_prob_gain", "prob gain", 0.0, 0.5, "%.2f"),
    ("cutin_enter_centering_gain", "center gain", 0.0, 1.2, "%.2fm"),
    ("cutin_confirm", "confirm", 0.05, 1.0, "%.2fs"),
  )
  slider_axes = []
  sliders = []
  slider_by_attr = {}
  left = 0.10
  bottom = 0.33
  width = 0.78
  height = 0.018
  for index, (attr, label, valmin, valmax, valfmt) in enumerate(slider_specs):
    slider_ax = fig.add_axes([left, bottom - index * 0.028, width, height])
    slider = Slider(slider_ax, label, valmin, valmax, valinit=getattr(args, attr), valfmt=valfmt)
    slider_axes.append(slider_ax)
    sliders.append((attr, slider))
    slider_by_attr[attr] = slider
  seek_ax = fig.add_axes([left, 0.015, width, height])
  seek_slider = Slider(
    seek_ax,
    "seek",
    0,
    max(1, len(snapshots) - 1),
    valinit=0,
    valfmt="%0.0f",
    valstep=1,
  )
  slider_axes.append(seek_ax)

  def draw_frame(frame_idx: int):
    frame_idx = int(clamp(frame_idx, 0, len(snapshots) - 1))
    current_frame["idx"] = frame_idx
    snapshot = snapshots[frame_idx]
    draw_snapshot(ax, snapshot, args, paused["value"], cutin_infos_for_snapshot(snapshots, frame_idx, args))
    if video_ax is not None:
      setup_video_axis(video_ax, snapshot.video_t, get_video(snapshot.rlog_path))
    if not updating_seek["value"] and int(round(seek_slider.val)) != frame_idx:
      updating_seek["value"] = True
      seek_slider.set_val(frame_idx)
      updating_seek["value"] = False
    return []

  def animation_tick(_frame_idx: int):
    frame_idx = current_frame["idx"]
    artists = draw_frame(frame_idx)
    current_frame["idx"] = (frame_idx + 1) % len(snapshots)
    return artists

  def set_paused(value: bool):
    paused["value"] = value
    ani = ani_holder.get("ani")
    if ani is None:
      return
    if paused["value"]:
      ani.event_source.stop()
    else:
      ani.event_source.start()

  def on_click(event):
    if event.canvas != fig.canvas:
      return
    if event.inaxes == controls_ax or event.inaxes in slider_axes:
      return
    set_paused(not paused["value"])
    draw_frame(current_frame["idx"])
    fig.canvas.draw_idle()

  def on_label_toggle(label: str):
    active = [
      field
      for field, status in zip(LABEL_FIELD_CHOICES, label_checks.get_status())
      if status
    ]
    args.active_label_fields = active
    draw_frame(current_frame["idx"])
    fig.canvas.draw_idle()

  def on_slider_change(_value: float):
    changed_attr = None
    for attr, slider in sliders:
      if abs(float(getattr(args, attr)) - float(slider.val)) > 1e-9:
        changed_attr = attr
      setattr(args, attr, slider.val)
    if changed_attr == "cutin_sensitivity":
      for attr, value in cutin_tuning_from_sensitivity(args.cutin_sensitivity).items():
        setattr(args, attr, value)
        slider = slider_by_attr.get(attr)
        if slider is not None and abs(float(slider.val) - float(value)) > 1e-9:
          slider.set_val(value)
    draw_frame(current_frame["idx"])
    fig.canvas.draw_idle()

  def seek_to_frame(frame_idx: int, pause: bool = True):
    if pause:
      set_paused(True)
    draw_frame(int(clamp(frame_idx, 0, len(snapshots) - 1)))
    fig.canvas.draw_idle()

  def on_seek_change(value: float):
    if updating_seek["value"]:
      return
    seek_to_frame(int(round(value)), pause=True)

  def on_key_press(event):
    key = (event.key or "").lower()
    step = max(1, int(round(args.fps)))
    if key == " ":
      set_paused(not paused["value"])
      draw_frame(current_frame["idx"])
      fig.canvas.draw_idle()
    elif key in ("left", "a"):
      seek_to_frame(current_frame["idx"] - step)
    elif key in ("right", "d"):
      seek_to_frame(current_frame["idx"] + step)
    elif key in ("down", "pagedown"):
      seek_to_frame(current_frame["idx"] - step * 10)
    elif key in ("up", "pageup"):
      seek_to_frame(current_frame["idx"] + step * 10)
    elif key == "home":
      seek_to_frame(0)
    elif key == "end":
      seek_to_frame(len(snapshots) - 1)

  animation_frames = range(len(snapshots)) if args.save_gif else None
  ani = FuncAnimation(fig, animation_tick, frames=animation_frames, interval=1000 / args.fps, blit=False, repeat=True)
  ani_holder["ani"] = ani
  fig.canvas.mpl_connect("button_press_event", on_click)
  fig.canvas.mpl_connect("key_press_event", on_key_press)
  label_checks.on_clicked(on_label_toggle)
  for _, slider in sliders:
    slider.on_changed(on_slider_change)
  seek_slider.on_changed(on_seek_change)
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
  parser.add_argument("--no-ego-lateral-compensation", action="store_true", help="Do not subtract common lateral velocity from displayed corner radar object motion")
  parser.add_argument("--no-cutin", action="store_true", help="Disable cut-in candidate highlighting")
  parser.add_argument("--cutin-horizon", type=float, default=CUTIN_DEFAULT_HORIZON_S, help="Seconds used for cut-in lateral projection")
  parser.add_argument("--cutin-lane-half-width", type=float, default=CUTIN_DEFAULT_LANE_HALF_WIDTH_M, help="Half lane width used by plot-only cut-in scoring")
  parser.add_argument("--cutin-confirm", type=float, default=CUTIN_DEFAULT_CONFIRM_S, help="Seconds a cut-in candidate must persist before highlighting")
  parser.add_argument("--cutin-sticky", type=float, default=CUTIN_DEFAULT_STICKY_S, help="Seconds to cap the plot-only cut-in sticky counter")
  parser.add_argument("--cutin-min-age", type=float, default=CUTIN_DEFAULT_MIN_AGE_S, help="Seconds a track must exist before entering cut-in state")
  parser.add_argument("--cutin-enter-min-x", type=float, default=CUTIN_DEFAULT_ENTER_MIN_X_M, help="Minimum forward distance for cut-in entry")
  parser.add_argument("--cutin-enter-max-x", type=float, default=CUTIN_DEFAULT_ENTER_MAX_X_M, help="Maximum forward distance for cut-in entry")
  parser.add_argument("--cutin-enter-min-abs-y", type=float, default=CUTIN_DEFAULT_ENTER_MIN_ABS_Y_M, help="Minimum absolute lateral distance for cut-in entry")
  parser.add_argument("--cutin-keep-min-x", type=float, default=CUTIN_DEFAULT_KEEP_MIN_X_M, help="Minimum forward distance for keeping a cut-in candidate sticky")
  parser.add_argument("--cutin-keep-max-x", type=float, default=CUTIN_DEFAULT_KEEP_MAX_X_M, help="Maximum forward distance for keeping a cut-in candidate sticky")
  parser.add_argument("--cutin-enter-future-prob", type=float, default=CUTIN_DEFAULT_ENTER_FUTURE_IN_LANE_PROB, help="Minimum future in-lane score for cut-in entry")
  parser.add_argument("--cutin-enter-prob-gain", type=float, default=CUTIN_DEFAULT_ENTER_PROB_GAIN, help="Minimum in-lane score improvement for cut-in entry")
  parser.add_argument("--cutin-enter-centering-gain", type=float, default=CUTIN_DEFAULT_ENTER_CENTERING_GAIN, help="Minimum lateral centering improvement in meters for cut-in entry")
  parser.add_argument("--cutin-keep-future-prob", type=float, default=CUTIN_DEFAULT_KEEP_FUTURE_IN_LANE_PROB, help="Future in-lane score for keeping a cut-in candidate sticky")
  parser.add_argument("--cutin-keep-max-dpath-future", type=float, default=CUTIN_DEFAULT_KEEP_MAX_DPATH_FUTURE, help="Future lateral distance threshold for keeping a cut-in candidate sticky")
  parser.add_argument("--cutin-keep-max-moving-away", type=float, default=CUTIN_DEFAULT_KEEP_MAX_MOVING_AWAY, help="Maximum lateral movement away from lane center while keeping cut-in sticky")
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
  parser.add_argument("--video-trim", choices=("front", "back"), default="front", help="When an rlog segment is longer than video, trim the extra time from the rlog front or back")
  parser.add_argument("--no-video-duration-limit", action="store_true", help="Do not cap each rlog segment to the matching TS video duration")
  parser.add_argument("--save-png", default=None)
  parser.add_argument("--save-gif", default=None)
  parser.add_argument("--cutin-sensitivity", type=float, default=CUTIN_DEFAULT_SENSITIVITY, help="Cut-in sensitivity using the same scale as RadarLatFactor; 50 maps to horizon=1.5, confirm=0.10, future prob=0.15, center gain=0.18")
  provided_flags = {arg.split("=", 1)[0] for arg in sys.argv[1:] if arg.startswith("--")}
  args = parser.parse_args()
  apply_cutin_sensitivity(args, provided_flags)
  if args.duration == 0:
    args.duration = None
  return args


if __name__ == "__main__":
  parsed_args = parse_args()
  plot_snapshots(read_snapshots(parsed_args), parsed_args)

