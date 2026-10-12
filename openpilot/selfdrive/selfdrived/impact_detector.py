"""Short horizontal IMU impulses, independent of the wheel-speed filter."""
import math

import numpy as np

from openpilot.common.transformations.orientation import rot_from_euler


IMPACT_ACCEL = 1.5 * 9.81  # m/s^2, user-selected experimental threshold
REARM_ACCEL = 10.0
MAX_SAMPLE_AGE = 0.1
MAX_SAMPLE_GAP = 0.03


class ImpactDetector:
  def __init__(self):
    self.last_timestamp = 0.0
    self.above = False
    self.orientation = self.calibration = None
    self.gravity_device = self.device_from_calib = None

  def update(self, *, now, timestamp, sensor, orientation, calibration, valid):
    """Return (trigger, quiet, calibrated acceleration); duplicates never count twice."""
    if not valid or not math.isfinite(timestamp) or not 0 <= now - timestamp <= MAX_SAMPLE_AGE:
      self.above = False
      return False, False, None
    if timestamp <= self.last_timestamp:
      return False, None, None
    try:
      values = np.asarray([sensor, orientation, calibration], dtype=float)
    except (TypeError, ValueError):
      self.above = False
      return False, False, None
    if values.shape != (3, 3) or not np.isfinite(values).all():
      self.above = False
      return False, False, None

    # Same sensor->device mapping and gravity sign as locationd/PoseKalman.
    device_accel = -values[0, ::-1]
    # IMU updates faster than pose/calibration. Keep only the transforms of the
    # last validated values, not an old sample or a cached validity decision.
    orientation_key, calibration_key = tuple(values[1]), tuple(values[2])
    if orientation_key != self.orientation:
      self.gravity_device = rot_from_euler(values[1]).T @ np.array([0.0, 0.0, -9.81])
      self.orientation = orientation_key
    if calibration_key != self.calibration:
      self.device_from_calib = rot_from_euler(values[2]).T
      self.calibration = calibration_key
    accel = self.device_from_calib @ (device_accel - self.gravity_device)
    horizontal = math.hypot(accel[0], accel[1])
    above = horizontal >= IMPACT_ACCEL
    trigger = above and self.above and timestamp - self.last_timestamp <= MAX_SAMPLE_GAP
    self.above = above
    self.last_timestamp = timestamp
    return trigger, horizontal < REARM_ACCEL, accel.tolist()
