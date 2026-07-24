from __future__ import annotations

from dataclasses import dataclass
from typing import Any


DEFAULT_YOUTUBE_QUALITY = 0


@dataclass(frozen=True, slots=True)
class YouTubeProfile:
  quality: int
  label: str
  source: str
  process_name: str
  encoder_flag: str
  width: int
  height: int
  fps: int
  video_kbps: int
  gop_seconds: int

  @property
  def target(self) -> dict[str, int]:
    return {
      "width": self.width,
      "height": self.height,
      "fps": self.fps,
      "video_kbps": self.video_kbps,
      "gop_seconds": self.gop_seconds,
    }

  def encoder_spec(self, executable: str, *, cwd: str) -> dict[str, Any]:
    return {
      "name": self.process_name,
      "label": self.label,
      "cmd": [executable, self.encoder_flag],
      "cwd": cwd,
      "match": f"{executable}\x00{self.encoder_flag}\x00",
    }


YOUTUBE_PROFILES: dict[int, YouTubeProfile] = {
  0: YouTubeProfile(
    quality=0,
    label="low",
    source="youtubeRoadEncodeData",
    process_name="youtube_low_encoderd",
    encoder_flag="--youtube-low",
    width=854,
    height=480,
    fps=20,
    video_kbps=750,
    gop_seconds=2,
  ),
  1: YouTubeProfile(
    quality=1,
    label="medium",
    source="youtubeRoadEncodeData",
    process_name="youtube_medium_encoderd",
    encoder_flag="--youtube-medium",
    width=1280,
    height=720,
    fps=20,
    video_kbps=2_000,
    gop_seconds=2,
  ),
  2: YouTubeProfile(
    quality=2,
    label="high",
    source="youtubeRoadEncodeData",
    process_name="youtube_encoderd",
    encoder_flag="--youtube",
    width=1920,
    height=1080,
    fps=20,
    video_kbps=4_200,
    gop_seconds=2,
  ),
  3: YouTubeProfile(
    quality=3,
    label="wide",
    source="youtubeRoadEncodeData",
    process_name="youtube_wide_encoderd",
    encoder_flag="--youtube-wide",
    width=1280,
    height=720,
    fps=20,
    video_kbps=2_700,
    gop_seconds=2,
  ),
}


def youtube_profile(
  quality: int | str | None,
  *,
  fallback: int = DEFAULT_YOUTUBE_QUALITY,
) -> YouTubeProfile:
  fallback_profile = YOUTUBE_PROFILES.get(fallback, YOUTUBE_PROFILES[DEFAULT_YOUTUBE_QUALITY])
  try:
    value = int(quality)
  except (TypeError, ValueError):
    return fallback_profile
  return YOUTUBE_PROFILES.get(value, fallback_profile)
