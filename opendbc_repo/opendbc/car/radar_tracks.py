"""Shared vehicle/replay radar motion filters; no vehicle platform imports."""

from collections import deque
from functools import cache
import math

import numpy as np

from openpilot.common.filter_simple import FirstOrderFilter
from opendbc.car.radar_lead_filter import RadarLeadFilter

CORNER_RADAR_SLOT_TRACK_RANGES = ((200, 220), (240, 250))
CORNER_RADAR_SLOT_DISCONTINUITY_D_REL_M = 8.0
CORNER_RADAR_SLOT_DISCONTINUITY_Y_REL_M = 1.5
CORNER_RADAR_SLOT_DISCONTINUITY_V_REL_MPS = 4.0
RADAR_ACCEL_INNOVATION_LIMIT = 3.0
# Filtered speed is retained for the existing near-standstill detector.
RADAR_SPEED_FILTER_RC = 0.10
RADAR_ACCEL_FILTER_RC = 0.05  # SCC reusable-slot path only
RADAR_JERK_HISTORY_SECONDS = 0.50
RADAR_JERK_HISTORY_MIN_SAMPLES = 7
RADAR_JERK_FILTER_RC = 0.25
RADAR_JERK_ZERO_FILTER_RC = 0.10
RADAR_JERK_DEADBAND = 0.20

def radar_track_id_is_reused_corner_slot(track_id: int) -> bool:
  return any(start <= track_id < end for start, end in CORNER_RADAR_SLOT_TRACK_RANGES)


@cache
def radar_quadratic_jerk_weights(sample_count: int) -> tuple[float, ...]:
  # Normalize sample times to integer radar frames so the weights are shared by
  # every track. The caller applies dt^-2 to convert the result to m/s^3.
  sample_times = np.arange(sample_count, dtype=float) - (sample_count - 1)
  design = np.column_stack((np.ones(sample_count), sample_times, sample_times ** 2))
  return tuple((2.0 * np.linalg.pinv(design)[2]).tolist())


def estimate_radar_jerk(velocity_history: deque[float], dt: float) -> float:
  if len(velocity_history) < RADAR_JERK_HISTORY_MIN_SAMPLES:
    return 0.0
  weights = radar_quadratic_jerk_weights(len(velocity_history))
  normalized_jerk = sum(
    weight * velocity
    for weight, velocity in zip(weights, velocity_history, strict=True)
  )
  return normalized_jerk / (dt ** 2)


def _clip_scalar(value: float, lower: float, upper: float) -> float:
  """Clamp one scalar without constructing a NumPy scalar/array."""
  return lower if value < lower else upper if value > upper else value


class MyTrack:
  def __init__(self, track_id: int, radar_point, dt: float):
    self.track_id = track_id
    self.reused_corner_slot = radar_track_id_is_reused_corner_slot(track_id)
    self.radar_source = str(radar_point.radarSource)
    self.cnt = 0
    self.dRel = radar_point.dRel
    self.vRel = radar_point.vRel
    self.yRel = radar_point.yRel
    self.yvRel = radar_point.yvRel
    self.vLead = radar_point.vLead
    self.aLead = 0.0
    self.jLead = 0.0
    self.noisy = False
    self.dt = dt
    self.vLead_avg = FirstOrderFilter(self.vLead, RADAR_SPEED_FILTER_RC, self.dt)
    self.aLead_avg = FirstOrderFilter(self.aLead, RADAR_ACCEL_FILTER_RC, self.dt)
    self.lead_filter = RadarLeadFilter(self.vLead, self.dt)
    self.jLead_avg = FirstOrderFilter(self.jLead, RADAR_JERK_FILTER_RC, self.dt)
    self.aLead_v_history: deque[float] = deque()
    jerk_history_samples = max(
      RADAR_JERK_HISTORY_MIN_SAMPLES,
      int(round(RADAR_JERK_HISTORY_SECONDS / self.dt)) + 1,
    )
    self.jLead_v_history: deque[float] = deque(maxlen=jerk_history_samples)
    self.yRel_avg = FirstOrderFilter(self.yRel, 0.1, self.dt)
    self.yvRel_avg = FirstOrderFilter(self.yvRel, 0.1, self.dt)
    self._reset_kinematics()
    self.cnt = 0

  def _reset_kinematics(self):
    self.aLead = 0.0
    self.jLead = 0.0
    self.noisy = False
    self.vLead_avg.x = self.vLead
    self.lead_filter.reset(self.vLead)
    self.aLead_avg.x = self.aLead
    self.jLead_avg.x = self.jLead
    # SCC's reusable object slot also uses acceleration innovation to detect
    # target replacement. Preserve its original three-sample discriminator.
    self.aLead_v_history = deque(maxlen=3)
    self.aLead_v_history.append(self.vLead)
    self.jLead_v_history.clear()
    self.jLead_v_history.append(self.vLead)

  def init_point(self, radar_point):
    self.radar_source = str(radar_point.radarSource)
    self.dRel = radar_point.dRel
    self.vRel = radar_point.vRel
    self.yRel = radar_point.yRel
    self.yvRel = radar_point.yvRel
    self.vLead = radar_point.vLead
    self._reset_kinematics()
    self.yRel_avg.x = self.yRel
    self.yvRel_avg.x = self.yvRel

  def is_discontinuous_corner_slot(self, radar_point) -> bool:
    if not self.reused_corner_slot:
      return False
    return (
      abs(radar_point.dRel - self.dRel) > CORNER_RADAR_SLOT_DISCONTINUITY_D_REL_M
      or abs(radar_point.yRel - self.yRel) > CORNER_RADAR_SLOT_DISCONTINUITY_Y_REL_M
      or abs(radar_point.vRel - self.vRel) > CORNER_RADAR_SLOT_DISCONTINUITY_V_REL_MPS
    )

  def write_acceleration(self, radar_point):
    radar_point.aLead = float(self.aLead) if self.cnt >= 6 else 0.0
    radar_point.jLead = float(self.jLead) if self.cnt >= 6 else 0.0

  def update(self, radar_point, a_ego):
    if not radar_point.measured:
      if self.cnt > 0:
        self.init_point(radar_point)
      self.cnt = 0
    elif self.cnt < 1 or self.is_discontinuous_corner_slot(radar_point):
      self.init_point(radar_point)
      self.cnt += 1
    else:
      self.vLead = radar_point.vLead
      if self.reused_corner_slot:
        self.yRel = radar_point.yRel
        self.yvRel = radar_point.yvRel
        self.yRel_avg.x = self.yRel
        self.yvRel_avg.x = self.yvRel
      else:
        self.yRel = self.yRel_avg.update(radar_point.yRel)
        self.yvRel = self.yvRel_avg.update(radar_point.yvRel)

      v_lead_filtered = self.vLead_avg.update(self.vLead)
      pseudo_stop = abs(v_lead_filtered) < 0.3 and abs(self.vLead - v_lead_filtered) < 0.05

      if self.radar_source == "scc":
        self.aLead_v_history.append(self.vLead)
        a_raw = ((self.aLead_v_history[-1] - self.aLead_v_history[0]) / (2.0 * self.dt)
                 if len(self.aLead_v_history) == 3 else 0.0)
        self.noisy = abs(a_raw - self.aLead) > RADAR_ACCEL_INNOVATION_LIMIT
        # SCC exposes one reusable object slot, so a large kinematic jump can mean the
        # source switched to a different lead without changing the track ID.
        if self.noisy:
          self.cnt = 0
        accel_sample = _clip_scalar(a_raw, -10.0, 5.0) if not pseudo_stop else 0.0
        self.aLead = float(self.aLead_avg.update(accel_sample))
      else:
        self.aLead = self.lead_filter.update(self.vLead, stationary=pseudo_stop)
        self.noisy = self.lead_filter.limited
        # Keep identified tracks alive through real braking. The observer bounds
        # the correction instead of resetting their age and publishing zero.

      # Estimate jerk independently from a causal quadratic velocity trend. Limit the
      # per-frame velocity step before adding it to the trend history so a single radar
      # quantization jump cannot dominate the 500 ms fit.
      trend_velocity = self.vLead
      if self.jLead_v_history:
        previous_velocity = self.jLead_v_history[-1]
        frame_acceleration = (trend_velocity - previous_velocity) / self.dt
        trend_velocity = previous_velocity + _clip_scalar(frame_acceleration, -10.0, 5.0) * self.dt
      self.jLead_v_history.append(trend_velocity)

      j_measurement = _clip_scalar(
        estimate_radar_jerk(self.jLead_v_history, self.dt), -6.0, 6.0,
      )
      if pseudo_stop or (self.noisy and self.radar_source == "scc"):
        j_target = 0.0
      else:
        j_target = math.copysign(
          max(0.0, abs(j_measurement) - RADAR_JERK_DEADBAND), j_measurement,
        )
      if self.cnt <= 2:
        j_target = 0.0

      self.jLead_avg.update_alpha(
        RADAR_JERK_ZERO_FILTER_RC if j_target == 0.0 else RADAR_JERK_FILTER_RC,
      )
      self.jLead = _clip_scalar(self.jLead_avg.update(j_target), -5.0, 5.0)
      self.jLead_avg.x = self.jLead

      # Store latest values
      self.dRel = radar_point.dRel
      self.vRel = radar_point.vRel

      self.cnt += 1
