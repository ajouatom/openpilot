"""Lead-motion and approaching-distance inputs for longitudinal MPC."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


LEAD_ACCEL_GAIN = 1.5
LEAD_ACCEL_TAU_SCALE = 0.5
LEAD_JERK_GAIN = 0.25

APPROACH_CLOSING_ENTER = 0.20
APPROACH_CLOSING_EXIT = 0.05
APPROACH_DISTANCE_RELEASE_RATE = 2.0
APPROACH_T_FOLLOW_MAX = 2.0
APPROACH_MIN_EGO_SPEED = 0.5


@dataclass
class _LeadDistanceState:
  distance_floor: float
  closing: bool = False
  anchored: bool = False


@dataclass(frozen=True)
class ApproachDistanceResult:
  t_follow: float
  distance_floor: float
  lead_active: tuple[bool, ...]


def _finite_value(value: Any, name: str, default: float = 0.0) -> float:
  try:
    result = float(getattr(value, name))
  except (AttributeError, TypeError, ValueError):
    return float(default)
  return result if np.isfinite(result) else float(default)


def _lead_identity(lead: Any, slot: int) -> tuple[str, int]:
  track_id = int(_finite_value(lead, "radarTrackId", -1.0))
  if track_id >= 0:
    return "track", track_id
  return "slot", slot


class ApproachDistanceController:
  """Preserve the physical gap present when a lead approach begins.

  The configured time gap remains the steady-state target. While relative
  speed is negative, the pre-deceleration distance becomes a floor and is
  converted back into the scalar tFollow input already understood by MPC.
  Once relative speed settles, that floor releases toward the configured gap.
  """

  def __init__(
    self,
    dt: float,
    release_rate: float = APPROACH_DISTANCE_RELEASE_RATE,
    maximum_t_follow: float = APPROACH_T_FOLLOW_MAX,
  ) -> None:
    self.dt = float(dt)
    self.release_rate = float(release_rate)
    self.maximum_t_follow = float(maximum_t_follow)
    self._states: dict[tuple[str, int], _LeadDistanceState] = {}

  def reset(self) -> None:
    self._states.clear()

  def update(
    self,
    *,
    base_t_follow: float,
    v_ego: float,
    leads: tuple[Any, ...],
    desired_distances: tuple[float, ...],
  ) -> ApproachDistanceResult:
    base_t_follow = float(base_t_follow)
    v_ego = float(v_ego)
    live_identities: set[tuple[str, int]] = set()
    distance_floors: list[float] = []
    adjustments: list[float] = []
    active = [False] * len(leads)
    observations: dict[tuple[str, int], list[tuple[int, Any, float]]] = {}

    for slot, (lead, desired_distance) in enumerate(zip(leads, desired_distances, strict=True)):
      if lead is None or not bool(getattr(lead, "status", False)):
        continue

      identity = _lead_identity(lead, slot)
      live_identities.add(identity)
      desired_distance = max(float(desired_distance), 0.0)
      observations.setdefault(identity, []).append((slot, lead, desired_distance))

    for identity, identity_observations in observations.items():
      state_desired_distance = max(item[2] for item in identity_observations)
      state = self._states.setdefault(identity, _LeadDistanceState(state_desired_distance))
      closing_speed = max(
        max(-_finite_value(item[1], "vRel"), 0.0)
        for item in identity_observations
      )

      if closing_speed >= APPROACH_CLOSING_ENTER:
        if not state.closing:
          state.distance_floor = max(
            state_desired_distance,
            max(_finite_value(item[1], "dRel") for item in identity_observations),
          )
        state.closing = True
        state.anchored = True
        state.distance_floor = max(state.distance_floor, state_desired_distance)
      elif closing_speed <= APPROACH_CLOSING_EXIT:
        state.closing = False

      if not state.closing:
        if state.anchored:
          state.distance_floor = max(
            state_desired_distance,
            state.distance_floor - self.release_rate * self.dt,
          )
          if state.distance_floor <= state_desired_distance:
            state.anchored = False
        else:
          state.distance_floor = state_desired_distance

      for slot, _, desired_distance in identity_observations:
        distance_floor = max(desired_distance, state.distance_floor)
        distance_floors.append(distance_floor)
        if v_ego > APPROACH_MIN_EGO_SPEED:
          adjustments.append(max(distance_floor - desired_distance, 0.0) / v_ego)
        active[slot] = state.closing

    self._states = {
      identity: state for identity, state in self._states.items()
      if identity in live_identities
    }
    t_follow = float(np.clip(
      base_t_follow + max(adjustments, default=0.0),
      base_t_follow,
      self.maximum_t_follow,
    ))
    return ApproachDistanceResult(
      t_follow=t_follow,
      distance_floor=max(distance_floors, default=0.0),
      lead_active=tuple(active),
    )


def extrapolate_lead_motion(
  *,
  x_lead: float,
  v_lead: float,
  a_lead: float,
  a_lead_tau: float,
  j_lead: float,
  time_indices: np.ndarray,
  time_differences: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
  """Build one physically integrated lead trajectory for the MPC obstacle."""
  time_indices = np.asarray(time_indices, dtype=float)
  time_differences = np.asarray(time_differences, dtype=float)
  acceleration = float(np.clip(a_lead * LEAD_ACCEL_GAIN, -10.0, 5.0))
  acceleration_tau = float(np.clip(a_lead_tau * LEAD_ACCEL_TAU_SCALE, 0.25, 3.0))
  jerk = float(np.clip(j_lead * LEAD_JERK_GAIN, -2.0, 2.0))
  jerk_tau = float(np.interp(jerk, (-2.0, 0.0, 2.0), (0.2, 2.0, 0.1)))
  jerk_trajectory = jerk * np.exp(-jerk_tau * time_indices ** 2 / 2.0)
  acceleration_trajectory = np.clip(
    acceleration * np.exp(-acceleration_tau * time_indices ** 2 / 2.0)
    + np.cumsum(time_differences * jerk_trajectory),
    -10.0,
    5.0,
  )
  velocity_trajectory = np.clip(
    v_lead + np.cumsum(time_differences * acceleration_trajectory),
    0.0,
    1e8,
  )
  position_trajectory = x_lead + np.cumsum(time_differences * velocity_trajectory)
  return position_trajectory, velocity_trajectory, acceleration_trajectory


__all__ = (
  "APPROACH_CLOSING_ENTER",
  "APPROACH_CLOSING_EXIT",
  "APPROACH_DISTANCE_RELEASE_RATE",
  "APPROACH_T_FOLLOW_MAX",
  "ApproachDistanceController",
  "ApproachDistanceResult",
  "LEAD_ACCEL_GAIN",
  "LEAD_ACCEL_TAU_SCALE",
  "LEAD_JERK_GAIN",
  "extrapolate_lead_motion",
)
