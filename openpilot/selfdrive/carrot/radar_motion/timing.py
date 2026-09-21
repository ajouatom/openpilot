"""Distance alignment policy, separate from radar ego-history compensation."""

# Persisted CarParams.flags bit (VolkswagenFlags.MEB). Avoid importing vehicle
# CAN/runtime dependencies into the standalone NAS replay bundle.
VOLKSWAGEN_MEB_FLAG = 16


def front_radar_distance_delay_s(car_params) -> float:
  """Return extra distance projection; publication-to-camera skew is separate.

  MEB's filtered CAN objects agree with vision without the legacy 0.8 s
  projection. Keep CP.radarDelay unchanged for RadarInterface's ego history:
  this distance-only calibration does not validate lead velocity filtering.
  """
  if car_params.brand == "volkswagen" and int(car_params.flags) & VOLKSWAGEN_MEB_FLAG:
    return 0.0
  return max(0.0, float(car_params.radarDelay))
