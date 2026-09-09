"""Bounded ACC departure credit; never reduce the common following time.

Coordinates: model/pose y is right-positive, radar yRel is left-positive.
The session frame is fixed at lane-change entry. Measured yaw and speed, not
the moving model path, establish that ego has actually started moving aside.
Confidence below is an evidence ramp, NOT a calibrated collision probability.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Any

import numpy as np


BODY_CLEARANCE_M = 2.5  # combined half widths plus uncertainty; not lane centres
MAX_CREDIT_S = 0.25
MAX_CREDIT_M = 4.0
MIN_RETAINED_RATIO = 0.80
CHECK_TIMES = np.linspace(0.0, 3.0, 61)


@dataclass(frozen=True)
class GapLead:
  radarTrackId: int
  dRel: float
  yRel: float
  vRel: float
  vLead: float
  aLeadK: float
  aLeadTau: float = 1.5
  status: bool = True
  radar: bool = True
  modelProb: float = 0.0

  @classmethod
  def read(cls, lead: Any) -> GapLead | None:
    if lead is None or not lead.status or not lead.radar or lead.radarTrackId < 0:
      return None
    values = [float(getattr(lead, key, math.nan)) for key in ('dRel', 'yRel', 'vRel', 'vLead', 'aLeadK')]
    if not all(math.isfinite(v) for v in values) or not -30.0 < values[0] < 160.0 or abs(values[1]) > 12.0:
      return None
    if not 0.0 <= values[3] <= 70.0 or abs(values[2]) > 70.0 or not -10.0 <= values[4] <= 5.0:
      return None
    return cls(int(lead.radarTrackId), *values)


@dataclass(frozen=True)
class LaneChangeGapPlan:
  active: bool = False
  primary_id: int = -1
  targets: tuple[GapLead, ...] = ()
  clearance_s: float = 0.0
  confidence: float = 0.0
  reason: str = 'inactive'

  def credit(self, primary: Any, horizons: np.ndarray, v_ego: float, max_accel: float,
             t_follow: float, stop_distance: float, ratio: float) -> np.ndarray:
    empty = np.zeros_like(horizons)
    lead = GapLead.read(primary)
    if (not self.active or not 0.0 < self.confidence <= 1.0 or not 0.0 < self.clearance_s <= 2.5 or not self.targets or lead is None
        or lead.radarTrackId != self.primary_id or lead.vLead < 4.0 or lead.aLeadK < -1.0
        or not all(math.isfinite(v) for v in (v_ego, max_accel, t_follow, stop_distance, ratio))
        or not 5.0 <= v_ego <= 35.0 or t_follow < 0.8 or not 0.0 < ratio < 1.0):
      return empty
    # Check the whole transition under the allowed ego acceleration, not only
    # the arrival point or the previous MPC trajectory. Targets retain full TF.
    accel = max(0.0, max_accel)
    ego_x = v_ego * CHECK_TIMES + 0.5 * accel * CHECK_TIMES**2
    ego_v = v_ego + accel * CHECK_TIMES
    for obstacle in (lead, *self.targets):
      brake = min(-1.0, obstacle.aLeadK)
      moving_t = np.minimum(CHECK_TIMES, max(0.0, obstacle.vLead) / -brake)
      obstacle_x = obstacle.dRel + obstacle.vLead * moving_t + 0.5 * brake * moving_t**2
      obstacle_v = np.maximum(0.0, obstacle.vLead + brake * CHECK_TIMES)
      gap = obstacle_x - ego_x
      if obstacle is lead:
        before_clearance = CHECK_TIMES <= self.clearance_s + 0.35
        minimum = max(8.0, stop_distance + 2.0) + np.maximum(0.0, ego_v - obstacle_v)**2 / 5.0
        if np.any(gap[before_clearance] <= minimum[before_clearance]):
          return empty
      else:
        minimum = stop_distance + t_follow * ego_v + np.maximum(0.0, ego_v**2 - obstacle_v**2) / 5.0
        if np.any(gap <= minimum):
          return empty
    amount = min(MAX_CREDIT_M, MAX_CREDIT_S * v_ego,
                 (1.0 - max(MIN_RETAINED_RATIO, ratio)) * t_follow * v_ego) * self.confidence
    return amount * np.clip((horizons - self.clearance_s - 0.35) / 0.5, 0.0, 1.0)


class LaneChangeGapTracker:
  def __init__(self):
    self.reset()

  def reset(self):
    self.direction = 0
    self.started = self.last_time = 0.0
    self.heading = self.ego_y = 0.0
    self.primary_id = -1
    self.history = deque()
    self.target_ids = ()
    self.targets_since = 0.0
    self.previous_leads = {}

  def update(self, *, now: float, direction: int, v_ego: float, yaw_rate: float,
             path_t: tuple, path_x: tuple, path_y: tuple, primary: Any,
             side_leads: tuple, blindspot: bool = False, valid: bool = True) -> LaneChangeGapPlan:
    active = direction in (-1, 1)
    if not active or not valid or not math.isfinite(now):
      self.reset()
      return LaneChangeGapPlan(active=active, reason='invalid-input' if active else 'inactive')
    lead = GapLead.read(primary)
    candidates = tuple(l for item in side_leads if (l := GapLead.read(item)) is not None)
    invalid_target = any(item is not None and item.status and GapLead.read(item) is None for item in side_leads)
    # These obstacles are useful even without reliable pose or a departure
    # prediction. The selected side list also catches cars alongside ego.
    targets = tuple(sorted({l.radarTrackId: l for l in candidates if l.dRel > 0.0}.values(), key=lambda l: l.dRel))[:4]
    base = {'active': True, 'primary_id': lead.radarTrackId if lead else -1, 'targets': targets}
    def inactive(reason):
      return LaneChangeGapPlan(**base, reason=reason)
    if not all(math.isfinite(v) for v in (v_ego, yaw_rate)) or not 5.0 <= v_ego <= 35.0 or abs(yaw_rate) > 0.08:
      self.reset()
      return inactive('pose-or-speed')
    if self.direction != direction or not 0.0 < now - self.last_time <= 0.15:
      self.reset()
      self.direction = direction
      self.started = self.last_time = now
      self.primary_id = lead.radarTrackId if lead else -1
    dt = now - self.last_time
    self.ego_y += v_ego * math.sin(self.heading + yaw_rate * dt * 0.5) * dt
    self.heading += yaw_rate * dt
    self.last_time = now
    current = {l.radarTrackId: l for l in (lead, *targets) if l is not None}
    discontinuous = any(
      abs(l.dRel - old.dRel - old.vRel * dt) > 2.0 or abs(l.vLead - old.vLead) > 3.0
      or abs(l.yRel - old.yRel) > 0.8
      for key, l in current.items() if (old := self.previous_leads.get(key)) is not None
    )
    self.previous_leads = current
    if discontinuous:
      self.history.clear()
      self.targets_since = now
      return inactive('track-discontinuity')
    if now - self.started > 6.0 or abs(self.heading) > 0.15:
      return inactive('session-limit')
    if lead is None or lead.radarTrackId != self.primary_id or not 8.0 < lead.dRel < 60.0:
      self.history.clear()
      return inactive('primary-changed')
    if len(targets) != len({l.radarTrackId for l in candidates if l.dRel > 0.0}):
      return inactive('too-many-targets')
    if blindspot or invalid_target or any(l.dRel <= 3.0 for l in candidates) or not targets or any(l.radarTrackId == lead.radarTrackId for l in targets):
      self.targets_since = now
      return inactive('destination-unconfirmed')
    ids = tuple(sorted(l.radarTrackId for l in targets))
    if ids != self.target_ids:
      self.targets_since = now
      self.target_ids = ids
    # Reconstruct target lateral position in a fixed frame. Raw yRel movement
    # caused only by ego yaw cannot masquerade as body clearance.
    front_y = self.ego_y + lead.dRel * math.sin(self.heading) - lead.yRel * math.cos(self.heading)
    self.history.append((now, self.ego_y, front_y))
    while self.history and now - self.history[0][0] > 0.5:
      self.history.popleft()
    if len(self.history) < 6 or now - self.history[0][0] < 0.25:
      return inactive('confirm-motion')
    old_t, old_y, old_front_y = self.history[0]
    actual_progress = direction * (self.ego_y - old_y)
    if (direction * self.ego_y < 0.25 or actual_progress < 0.10
        or actual_progress / (now - old_t) < 0.25
        or abs(front_y - old_front_y) > 0.10 or now - self.targets_since < 0.30):
      return inactive('unconfirmed-motion')
    if (len(path_t) < 6 or len(path_t) != len(path_x) or len(path_t) != len(path_y)
        or not all(math.isfinite(v) for v in (*path_t, *path_x, *path_y))
        or any(b <= a for a, b in zip(path_t, path_t[1:], strict=False)) or path_t[0] > 0.1 or path_t[-1] < 3.0):
      return inactive('invalid-path')
    times = CHECK_TIMES
    xs, ys = np.interp(times, path_t, path_x), np.interp(times, path_t, path_y)
    predicted_y = self.ego_y + xs * math.sin(self.heading) + ys * math.cos(self.heading)
    separation = direction * (predicted_y - front_y)
    # Do not assume the requested path crosses faster than the lateral motion
    # already measured. A path-only jump cannot advance the clearance time.
    observed_rate = actual_progress / (now - old_t)
    separation = np.minimum(separation, direction * (self.ego_y - front_y) + observed_rate * times)
    # Require sustained clearance, including an expanding lateral uncertainty
    # envelope. Never clear solely because a remote path point changed lanes.
    clear = separation > BODY_CLEARANCE_M + 0.2 * times
    indices = np.flatnonzero(clear & (times >= 0.35) & (times <= 2.5))
    if not len(indices):
      return inactive('no-clearance')
    first = int(indices[0])
    if not np.all(clear[first:]) or np.any(direction * np.diff(predicted_y) < -0.02):
      return inactive('path-reentry')
    confidence = min(1.0, max(0.0, (now - self.targets_since - 0.30) / 0.30))
    return LaneChangeGapPlan(**base, clearance_s=float(times[first]), confidence=confidence, reason='confirmed-departure')
