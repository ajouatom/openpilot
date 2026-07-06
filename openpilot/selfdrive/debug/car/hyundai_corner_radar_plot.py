#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
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
DEFAULT_STALE_S = 0.25
WHEEL_SPEED_ADDR = 0xA0
KPH_TO_MS = 1000.0 / 3600.0


@dataclass(frozen=True)
class CornerObject:
  t: float
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


def decode_corner_object(t: float, address: int, data: bytes) -> CornerObject:
  return CornerObject(
    t=t,
    address=address,
    slot=address - START_ADDR,
    quality=dbc_unsigned(data, 24, 7),
    age=dbc_unsigned(data, 32, 8),
    object_id=dbc_unsigned(data, 44, 7),
    object_class=dbc_unsigned(data, 60, 3),
    width=dbc_unsigned(data, 52, 7) * 0.05,
    x=dbc_unsigned(data, 64, 13) * 0.05,
    y=dbc_unsigned(data, 78, 12) * 0.05 - 102.4,
    vx=dbc_unsigned(data, 91, 12) * 0.05 - 100.0,
    vy=dbc_unsigned(data, 104, 10) * 0.05 - 25.0,
    ax=dbc_signed(data, 115, 9) * 0.05,
  )


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


def read_snapshots(args: argparse.Namespace) -> list[tuple[float, list[CornerObject], list[SummaryCornerObject], float]]:
  latest: dict[int, CornerObject] = {}
  latest_summary: dict[str, SummaryCornerObject] = {}
  snapshots: list[tuple[float, list[CornerObject], list[SummaryCornerObject], float]] = []
  first_log_t: int | None = None
  next_frame_t = args.start
  frame_dt = 1.0 / args.fps
  latest_ego_speed = 0.0
  summary_dbc = None if args.no_summary_corners else load_summary_dbc()
  summary_messages = {} if summary_dbc is None else {
    0x162: summary_dbc.get_message_by_frame_id(0x162),
    0x1ea: summary_dbc.get_message_by_frame_id(0x1ea),
  }

  for msg in LogReader(args.rlog, only_union_types=True):
    msg_type = msg.which()
    if msg_type not in ("can", "carState"):
      continue

    if first_log_t is None:
      first_log_t = msg.logMonoTime

    t = (msg.logMonoTime - first_log_t) / 1e9
    if t < args.start:
      continue
    if args.duration is not None and t > args.start + args.duration:
      break

    if msg_type == "carState":
      latest_ego_speed = float(msg.carState.vEgo)
      continue

    for can in msg.can:
      if can.src == args.speed_bus and can.address == WHEEL_SPEED_ADDR and len(can.dat) == 24:
        latest_ego_speed = decode_wheel_speed_mps(bytes(can.dat))

      if can.src == args.bus and START_ADDR <= can.address <= END_ADDR and len(can.dat) == 32:
        obj = decode_corner_object(t, can.address, bytes(can.dat))
        if is_valid_object(obj, args.min_quality, args.max_x, args.max_abs_y):
          latest[can.address] = obj
        else:
          latest.pop(can.address, None)

      if summary_dbc is not None and can.src == args.summary_bus and can.address in summary_messages and len(can.dat) == 32:
        values = summary_messages[can.address].decode(bytes(can.dat), decode_choices=False)
        source = f"0x{can.address:x}"
        for obj in decode_summary_corners(t, source, values, args):
          latest_summary[obj.label] = obj

    while t >= next_frame_t:
      active = [obj for obj in latest.values() if t - obj.t <= args.stale]
      active.sort(key=lambda o: o.slot)
      summary_active = [obj for obj in latest_summary.values() if t - obj.t <= args.summary_stale]
      summary_active.sort(key=lambda o: o.label)
      snapshots.append((next_frame_t, active, summary_active, latest_ego_speed))
      next_frame_t += frame_dt

  return snapshots


def setup_axes(ax, args: argparse.Namespace) -> None:
  ax.set_title("Hyundai IONIQ 5 PE corner radar candidates")
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

  car_w = 1.9
  car_l = 4.7
  car = ax.add_patch(plt.Rectangle((-car_w / 2, -car_l / 2), car_w, car_l, fill=False, linewidth=2.0, color="black"))
  car.set_label("ego")
  ax.text(0.0, 0.2, "ego", ha="center", va="bottom", fontsize=9)


def plot_y(obj: CornerObject, args: argparse.Namespace) -> float:
  return -obj.y if args.flip_raw_y else obj.y


def plot_vy(obj: CornerObject, args: argparse.Namespace) -> float:
  return -obj.vy if args.flip_raw_y else obj.vy


def draw_snapshot(ax, t: float, objects: list[CornerObject], summary_objects: list[SummaryCornerObject], ego_speed: float, args: argparse.Namespace) -> None:
  ax.clear()
  setup_axes(ax, args)
  ax.set_title(f"Hyundai corner radar candidates  t={t:.2f}s  vEgo={ego_speed * 3.6:.1f}kph  raw={len(objects)} summary={len(summary_objects)}")

  if objects:
    y = [plot_y(obj, args) for obj in objects]
    x = [obj.x for obj in objects]
    colors = [obj.slot for obj in objects]

    sc = ax.scatter(y, x, c=colors, s=[40 + max(0, obj.quality) * 2 for obj in objects], cmap="tab20", vmin=0, vmax=19, edgecolors="black", linewidths=0.6)
    sc.set_label("raw object slot")

    for obj in objects:
      obj_y = plot_y(obj, args)
      ax.arrow(
        obj_y,
        obj.x,
        plot_vy(obj, args) * args.velocity_scale,
        obj.vx * args.velocity_scale,
        head_width=0.35,
        head_length=0.7,
        length_includes_head=True,
        color="tab:red",
        alpha=0.75,
      )
      ax.text(
        obj_y,
        obj.x + 0.8,
        f"{obj.slot:02d} id={obj.object_id} v={(ego_speed + obj.vx) * 3.6:.1f}kph y={obj_y:+.1f}m",
        ha="center",
        va="bottom",
        fontsize=8,
      )

    cbar = getattr(ax.figure, "_corner_radar_cbar", None)
    if cbar is None:
      ax.figure._corner_radar_cbar = ax.figure.colorbar(sc, ax=ax, label="slot 0x235 + n")

  if summary_objects:
    front = [obj for obj in summary_objects if obj.x >= 0.0]
    rear = [obj for obj in summary_objects if obj.x < 0.0]
    if front:
      ax.scatter([obj.y for obj in front], [obj.x for obj in front], marker="s", s=90, color="tab:green", edgecolors="black", linewidths=0.7, label="0x162/0x1ea front")
    if rear:
      ax.scatter([obj.y for obj in rear], [obj.x for obj in rear], marker="s", s=90, color="tab:orange", edgecolors="black", linewidths=0.7, label="0x162/0x1ea rear")
    for obj in summary_objects:
      ax.text(obj.y, obj.x - 1.2 if obj.x < 0 else obj.x + 1.2, f"{obj.label} {obj.source}", ha="center", va="top" if obj.x < 0 else "bottom", fontsize=8)

  if objects or summary_objects:
    ax.legend(loc="lower right")


def resolve_video_path(args: argparse.Namespace) -> Path | None:
  if args.video.lower() in ("none", "off", "false", "0"):
    return None
  if args.video.lower() != "auto":
    return Path(args.video)

  rlog_path = Path(args.rlog)
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


def plot_snapshots(snapshots: list[tuple[float, list[CornerObject], list[SummaryCornerObject], float]], args: argparse.Namespace) -> None:
  global plt
  import matplotlib.pyplot as plt
  from matplotlib.animation import FuncAnimation

  if not snapshots:
    raise RuntimeError("No corner radar objects decoded. Check --bus, --start, --duration, or filter thresholds.")

  video_path = resolve_video_path(args)
  video = VideoSampler(video_path, args.video_fps or args.fps, args.video_offset) if video_path is not None else None
  if video is not None:
    print(f"video: {video.path} ({video.fps:.2f} fps)")
    fig, (ax, video_ax) = plt.subplots(1, 2, figsize=(16, 8), gridspec_kw={"width_ratios": [1.0, 1.25]})
  else:
    fig, ax = plt.subplots(figsize=(9, 9))
    video_ax = None

  if args.save_png:
    idx = min(range(len(snapshots)), key=lambda i: abs(snapshots[i][0] - args.snapshot_time))
    draw_snapshot(ax, snapshots[idx][0], snapshots[idx][1], snapshots[idx][2], snapshots[idx][3], args)
    if video_ax is not None:
      setup_video_axis(video_ax, snapshots[idx][0], video)
    fig.tight_layout()
    fig.savefig(args.save_png, dpi=160)
    print(f"saved {args.save_png}")
    return

  def update(frame_idx: int):
    t, objects, summary_objects, ego_speed = snapshots[frame_idx]
    draw_snapshot(ax, t, objects, summary_objects, ego_speed, args)
    if video_ax is not None:
      setup_video_axis(video_ax, t, video)
    return []

  ani = FuncAnimation(fig, update, frames=len(snapshots), interval=1000 / args.fps, blit=False, repeat=True)
  if args.save_gif:
    ani.save(args.save_gif, writer="pillow", fps=args.fps)
    print(f"saved {args.save_gif}")
    return

  plt.show()


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Plot candidate Hyundai CAN-FD corner radar objects from bus 1 rlog data.",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  )
  parser.add_argument("rlog", help="Path to rlog.zst")
  parser.add_argument("--bus", type=int, default=1)
  parser.add_argument("--speed-bus", type=int, default=0, help="Bus containing WHEEL_SPEEDS 0xA0 for vEgo fallback")
  parser.add_argument("--start", type=float, default=0.0, help="Start time in seconds from the beginning of the rlog")
  parser.add_argument("--duration", type=float, default=0.0, help="Seconds to read; 0 reads the whole rlog")
  parser.add_argument("--fps", type=float, default=15.0)
  parser.add_argument("--stale", type=float, default=DEFAULT_STALE_S)
  parser.add_argument("--min-quality", type=int, default=1)
  parser.add_argument("--max-x", type=float, default=120.0)
  parser.add_argument("--min-plot-x", type=float, default=-30.0)
  parser.add_argument("--max-abs-y", type=float, default=20.0)
  parser.add_argument("--plot-width-scale", type=float, default=2.0, help="Visual width multiplier for the radar plot while keeping the same meter range")
  parser.add_argument("--velocity-scale", type=float, default=0.35, help="Arrow length multiplier")
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
  parser.add_argument("--save-png", default=None)
  parser.add_argument("--save-gif", default=None)
  args = parser.parse_args()
  if args.duration == 0:
    args.duration = None
  return args


if __name__ == "__main__":
  parsed_args = parse_args()
  plot_snapshots(read_snapshots(parsed_args), parsed_args)
