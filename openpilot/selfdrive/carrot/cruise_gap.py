"""Limits for the driver-selected gap-button cycle."""


def supported_gap_levels(value: int) -> int:
  return value if value in (3, 4) else 3


def cruise_gap_levels(requested: int, vehicle_max: int) -> int:
  maximum = supported_gap_levels(vehicle_max)
  return min(maximum, max(2, requested)) if requested > 0 else maximum


def next_gap_personality(current: int, levels: int) -> int:
  # A reduced cycle takes effect on the next press, without skipping its top gap.
  return current - 1 if 0 < current < levels else levels - 1
