"""Convert device yaw to the forward/left frame used by radar velocities.

Model paths are converted separately inside predictor._path_geometry; do
not negate those inputs again when converting angular velocity.
"""
import math


def device_yaw_to_radar(yaw_rate: float) -> float:
  return -yaw_rate if math.isfinite(yaw_rate) else 0.0
