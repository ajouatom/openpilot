import math


MODEL_SPEED_RATIO_MIN = 0.7


class LaneModelSpeedGuard:
  """Use lane MPC only after a continuous run of usable model speed trajectories."""

  def __init__(self, recovery_frames: int):
    self.recovery_frames = recovery_frames
    self.valid_frames = 0

  def update(self, v_ego: float, model_start: float, model_end: float) -> bool:
    valid = all(math.isfinite(v) and v >= 0.0 for v in (v_ego, model_start, model_end))
    # A low trajectory can have an increasing profile and pass the original
    # end/start deceleration check. Do not run lane MPC on that shortened time
    # horizon while the vehicle is still travelling substantially faster.
    valid = valid and model_start >= v_ego * MODEL_SPEED_RATIO_MIN
    valid = valid and model_end >= model_start * MODEL_SPEED_RATIO_MIN
    self.valid_frames = min(self.valid_frames + 1, self.recovery_frames + 1) if valid else 0
    return self.valid_frames > self.recovery_frames
