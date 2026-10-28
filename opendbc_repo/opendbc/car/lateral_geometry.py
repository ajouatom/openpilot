def apply_lateral_geometry(CP, params) -> None:
  """Override geometry once, after brand setup and before axle stiffness scaling."""
  if not params.get_bool("CenterToFrontRatioEnabled"):
    return
  try:
    percent = params.get_int("CenterToFrontRatio")
  except (TypeError, ValueError, OverflowError):
    return
  # Preserve the car-interface invariant, not a road-validated tuning range.
  # Missing/invalid overrides leave the brand-specific geometry untouched.
  if 31 <= percent <= 69:
    CP.centerToFront = CP.wheelbase * (percent * 0.01)
