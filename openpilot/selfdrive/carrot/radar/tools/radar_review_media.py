"""Export review clips by recorded camera frame index, independent of guessed FPS."""

from __future__ import annotations

import bisect
from collections.abc import Sequence
from dataclasses import dataclass
import math
from pathlib import Path
import statistics
import subprocess
import tempfile


@dataclass(frozen=True)
class ReviewClipPlan:
  first_frame: int
  end_frame: int  # Exclusive decoder frame index.
  fps: float
  source_times_s: tuple[float, ...]

  @property
  def duration_s(self) -> float:
    return len(self.source_times_s) / self.fps


def camera_frame_times(encode_indexes: Sequence[dict]) -> tuple[float, ...]:
  """Map sequentially decoded frames to exposure time within one camera file."""
  if len(encode_indexes) < 2:
    raise ValueError("camera index needs at least two frames")
  ordered = sorted(encode_indexes, key=lambda value: int(value["segmentId"]))
  if len({int(value["segmentNum"]) for value in ordered}) != 1:
    raise ValueError("camera index spans multiple segments")
  if [int(value["segmentId"]) for value in ordered] != list(range(len(ordered))):
    raise ValueError("camera index is incomplete or duplicated")
  stamps = tuple(int(value["timestampEof"]) for value in ordered)
  if stamps[0] <= 0 or any(right <= left for left, right in zip(stamps, stamps[1:], strict=False)):
    raise ValueError("camera exposure timestamps must increase")
  return tuple((stamp - stamps[0]) / 1e9 for stamp in stamps)


def plan_review_clip(times_s: Sequence[float], start_s: float, end_s: float) -> ReviewClipPlan:
  if len(times_s) < 2 or not all(math.isfinite(value) for value in (*times_s, start_s, end_s)):
    raise ValueError("finite camera timestamps and clip bounds are required")
  if any(right <= left for left, right in zip(times_s, times_s[1:], strict=False)):
    raise ValueError("camera timestamps must increase")
  if end_s <= start_s or end_s < times_s[0] or start_s > times_s[-1]:
    raise ValueError("clip does not overlap camera timestamps")
  # Derive the acquisition cadence from exposure timestamps. MPEG-TS/OpenCV can
  # report 120 FPS for a real 20 FPS stream; neither that estimate nor its derived
  # frame count is a usable frame/time mapping.
  fps = float(round(1.0 / statistics.median(b - a for a, b in zip(times_s, times_s[1:], strict=False))))
  if not 1.0 <= fps <= 120.0:
    raise ValueError("unsupported camera acquisition cadence")

  def nearest(time_s: float) -> int:
    index = min(bisect.bisect_left(times_s, time_s), len(times_s) - 1)
    if index and abs(times_s[index - 1] - time_s) <= abs(times_s[index] - time_s):
      index -= 1
    return index

  first, last = nearest(start_s), nearest(end_s)
  return ReviewClipPlan(first, last + 1, fps, tuple(times_s[first:last + 1]))


def export_review_clip(ffmpeg: str, camera: Path, output: Path, plan: ReviewClipPlan) -> None:
  """Decode in order, trim by frame index, and check the emitted frame count."""
  output.parent.mkdir(parents=True, exist_ok=True)
  with tempfile.NamedTemporaryFile(suffix=".mp4", dir=output.parent, delete=False) as temp:
    temporary = Path(temp.name)
  try:
    result = subprocess.run([
      ffmpeg, "-hide_banner", "-loglevel", "error", "-nostdin", "-nostats",
      "-i", str(camera), "-map", "0:v:0", "-an",
      "-vf", (
        f"trim=start_frame={plan.first_frame}:end_frame={plan.end_frame},"
        + f"setpts=N/({plan.fps:g}*TB)"
      ),
      "-r", f"{plan.fps:g}", "-fps_mode", "cfr",
      "-c:v", "libx264", "-preset", "veryfast", "-crf", "18",
      "-pix_fmt", "yuv420p", "-movflags", "+faststart",
      "-progress", "pipe:1", "-y", str(temporary),
    ], capture_output=True, text=True, check=True)
    counts = [int(line.partition("=")[2]) for line in result.stdout.splitlines() if line.startswith("frame=")]
    if not counts or counts[-1] != len(plan.source_times_s):
      raise ValueError(f"camera/index mismatch: expected {len(plan.source_times_s)} frames, emitted {counts[-1:]}")
    temporary.replace(output)
  finally:
    temporary.unlink(missing_ok=True)


def main() -> None:
  import argparse
  from dataclasses import asdict
  import json

  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--camera", type=Path, required=True)
  parser.add_argument("--index", type=Path, required=True, help="JSON array of full-schema qRoadEncodeIdx messages")
  parser.add_argument("--start", type=float, required=True, help="Seconds since the first camera exposure EOF")
  parser.add_argument("--end", type=float, required=True, help="Seconds since the first camera exposure EOF")
  parser.add_argument("--output", type=Path, required=True)
  parser.add_argument("--ffmpeg", default="ffmpeg")
  args = parser.parse_args()
  times = camera_frame_times(json.loads(args.index.read_text(encoding="utf-8")))
  plan = plan_review_clip(times, args.start, args.end)
  export_review_clip(args.ffmpeg, args.camera, args.output, plan)
  args.output.with_suffix(".frames.json").write_text(json.dumps(asdict(plan), indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
  main()
