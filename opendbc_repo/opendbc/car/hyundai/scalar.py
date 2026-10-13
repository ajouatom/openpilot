"""Scalar-only operations for Hyundai float-valued control/HUD fields."""


def clip_float(value: float, lower: float, upper: float) -> float:
  # Match float(np.clip(...)) for float inputs, including NaN, reversed bounds
  # and signed zero. Array and integer clip operations keep their NumPy path.
  if value != value:
    return float(value)
  if lower != lower:
    return float(lower)
  if upper != upper or lower > upper:
    return float(upper)
  return float(lower if value < lower else upper if value > upper else value)
