"""Radar lead-selection runtime."""


def effective_radar_track_mode(
  brand: str,
  radar_unavailable: bool,
  configured_mode: int,
) -> int:
  """Keep radar-source options Hyundai-only and auto-select other cars."""
  if brand == "hyundai":
    return int(configured_mode)
  return -2 if radar_unavailable else 1
