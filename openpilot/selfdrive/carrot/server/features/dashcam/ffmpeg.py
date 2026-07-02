import os
import shutil
import subprocess

from aiohttp import web

from .catalog import source_video
from .paths import cache_path, segment_dir


def run_ffmpeg(args: list[str], timeout: float = 90.0) -> subprocess.CompletedProcess:
  if not shutil.which("ffmpeg"):
    raise web.HTTPServiceUnavailable(text="ffmpeg not available")
  return subprocess.run(
    ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", *args],
    capture_output=True,
    text=True,
    timeout=timeout,
  )


def placeholder_svg(token: str = "dashcam") -> str:
  out = cache_path("placeholder", token, ".svg")
  if os.path.isfile(out) and os.path.getsize(out) > 0:
    return out
  svg = """<svg xmlns="http://www.w3.org/2000/svg" width="640" height="360" viewBox="0 0 640 360">
<rect width="640" height="360" fill="#10161d"/>
<rect x="1" y="1" width="638" height="358" fill="none" stroke="#354252" stroke-width="2"/>
<path d="M296 126h48l20 24h44a24 24 0 0 1 24 24v72a24 24 0 0 1-24 24H232a24 24 0 0 1-24-24v-72a24 24 0 0 1 24-24h44z" fill="#253241"/>
<circle cx="320" cy="210" r="42" fill="#111820" stroke="#5d6c7d" stroke-width="8"/>
<path d="M306 188v44l38-22z" fill="#ffb268"/>
<text x="320" y="306" text-anchor="middle" fill="#9aa6b2" font-family="Arial, sans-serif" font-size="24" font-weight="700">NO THUMBNAIL</text>
</svg>"""
  with open(out, "w", encoding="utf-8") as f:
    f.write(svg)
  return out


def ensure_thumbnail(segment: str) -> str:
  segment_path = segment_dir(segment)
  source, _ = source_video(segment_path)
  out = cache_path("thumb", segment, ".jpg")
  if os.path.isfile(out) and os.path.getsize(out) > 0:
    return out
  attempts = (
    ["-ss", "2", "-i", source, "-vframes", "1", "-vf", "scale=640:-1", out],
    ["-ss", "0.2", "-i", source, "-vframes", "1", "-vf", "scale=640:-1", out],
  )
  for args in attempts:
    result = run_ffmpeg(args)
    if result.returncode == 0 and os.path.isfile(out) and os.path.getsize(out) > 0:
      return out
    try:
      if os.path.exists(out):
        os.remove(out)
    except OSError:
      pass
  return placeholder_svg(segment)


def ensure_preview(segment: str) -> str:
  segment_path = segment_dir(segment)
  source, _ = source_video(segment_path)
  out = cache_path("preview", segment, ".gif")
  if os.path.isfile(out) and os.path.getsize(out) > 0:
    return out
  result = run_ffmpeg([
    "-ss", "1",
    "-t", "2.4",
    "-i", source,
    "-vf", "fps=4,scale=360:-1:flags=lanczos",
    "-loop", "0",
    out,
  ], timeout=120.0)
  if result.returncode != 0 or not os.path.isfile(out) or os.path.getsize(out) <= 0:
    try:
      if os.path.exists(out):
        os.remove(out)
    except OSError:
      pass
    return ensure_thumbnail(segment)
  return out


def browser_video(segment: str) -> tuple[str, str]:
  segment_path = segment_dir(segment)
  source, source_name = source_video(segment_path)
  if source_name.endswith(".mp4"):
    return source, "video/mp4"

  out = cache_path("video", segment, ".mp4")
  if os.path.isfile(out) and os.path.getsize(out) > 0:
    return out, "video/mp4"

  result = run_ffmpeg(["-i", source, "-c", "copy", "-an", "-movflags", "+faststart", out], timeout=180.0)
  if result.returncode == 0 and os.path.isfile(out) and os.path.getsize(out) > 0:
    return out, "video/mp4"
  try:
    if os.path.exists(out):
      os.remove(out)
  except OSError:
    pass

  # Last-resort fallback: some browsers can still handle TS, and this preserves access.
  return source, "video/mp2t"
