import math


DEFAULT_STEER_RATIO_RATE = 100.0
MIN_STEER_RATIO_RATE = 30.0
MAX_STEER_RATIO_RATE = 200.0
MIN_STEER_RATIO = 0.1


def resolve_vehicle_model_steer_ratio(live_steer_ratio: float, steer_ratio_rate: float,
                                      custom_steer_ratio: float, is_vw_meb: bool) -> float:
  live_steer_ratio = max(live_steer_ratio, MIN_STEER_RATIO)
  if is_vw_meb:
    return live_steer_ratio

  custom_steer_ratio *= 0.1
  if math.isfinite(custom_steer_ratio) and custom_steer_ratio > 1.0:
    return custom_steer_ratio

  if not math.isfinite(steer_ratio_rate) or not MIN_STEER_RATIO_RATE <= steer_ratio_rate <= MAX_STEER_RATIO_RATE:
    steer_ratio_rate = DEFAULT_STEER_RATIO_RATE

  return max(live_steer_ratio * steer_ratio_rate * 0.01, MIN_STEER_RATIO)
