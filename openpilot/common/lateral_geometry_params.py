def migrate_lateral_geometry_params(params):
  """Retire the enable switch without activating a previously disabled value."""
  if params.get("CenterToFrontRatioEnabled") is not None:
    if not params.get_bool("CenterToFrontRatioEnabled"):
      params.put_int("CenterToFrontRatio", 0)
    params.remove("CenterToFrontRatioEnabled")
