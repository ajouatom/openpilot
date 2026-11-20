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


def radar_lead_comfort_extras_enabled(brand: str, classic_238: bool = False) -> bool:
  """Whether radar leads get the radar-only following extras.

  These are the opening-gap headroom hold (extra TF up to 2.5 s), the
  LeadAccelResponse catch-up boost and the lead-deceleration preview. Vision
  leads never get them. The K7 0x238 radar lead is followed like a vision lead:
  radar distance/speed, but base TF and normal MPC costs. On the K7 the hold
  kept gaps long, then released into catch-up acceleration and braking.
  """
  return not (brand == "hyundai" and classic_238)


def classic_238_track_mode(configured_mode: int, points) -> int:
  """The K7 follows the radar's own SCC11 lead (like mode 0) while it is published.

  RadarInterface publishes the SCC point only after SCC11 has been live for 1 s and then
  withholds the 0x238 objects, which stay reference-only. Otherwise the 0x238 objects
  are matched as front radar (configured mode).
  """
  if any(str(point.radarSource) == "scc" and point.measured for point in points):
    return 0
  return configured_mode
