import math


GEOMETRY_RATE = 0.1  # Ratio per second; a one-percent adjustment takes 0.1 s.


def requested_center_to_front_ratio(params, CP) -> float:
  try:
    percent = params.get_int("CenterToFrontRatio")
  except (TypeError, ValueError, OverflowError):
    percent = 0
  return percent * 0.01 if 31 <= percent <= 69 else CP.centerToFront / CP.wheelbase


class LateralGeometry:
  """One immutable vehicle baseline, with coherent geometry and axle stiffness."""
  def __init__(self, CP):
    self.wheelbase = CP.wheelbase
    self.default_ratio = CP.centerToFront / CP.wheelbase
    self.default_front = CP.tireStiffnessFront
    self.default_rear = CP.tireStiffnessRear
    self.ratio = self.default_ratio
    self.center_to_front = CP.centerToFront
    self.stiffness_front = self.default_front
    self.stiffness_rear = self.default_rear

  def update(self, ratio: float) -> bool:
    # Zero is an absent/legacy message, never a CG at the front axle.
    if not math.isfinite(ratio) or not (0.3 < ratio < 0.7 or math.isclose(ratio, self.default_ratio, abs_tol=1e-7)):
      return False
    if math.isclose(ratio, self.ratio, rel_tol=0, abs_tol=1e-7):
      return False
    self.ratio = ratio
    self.center_to_front = self.wheelbase * ratio
    self.stiffness_front = self.default_front * (1 - ratio) / (1 - self.default_ratio)
    self.stiffness_rear = self.default_rear * ratio / self.default_ratio
    return True

  def step(self, target: float, dt: float) -> bool:
    if not math.isfinite(target) or not math.isfinite(dt) or dt <= 0:
      return False
    if not (0.3 < target < 0.7 or math.isclose(target, self.default_ratio, abs_tol=1e-7)):
      return False
    limit = GEOMETRY_RATE * min(dt, 0.05)
    return self.update(self.ratio + max(-limit, min(target - self.ratio, limit)))
