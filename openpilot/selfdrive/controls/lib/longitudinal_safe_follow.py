"""Keep prompt Safe-mode launches, then taper surplus positive acceleration."""

import math

import numpy as np

from openpilot.selfdrive.controls.lib.longitudinal_preview import DRIVING_MODE_SAFE


SETTLE_HORIZON = 2.0
ACCEL_EXCESS_DEADBAND = 0.1
TAPER_RATE = 0.8
MODE_BLEND_TIME = 0.8


class SafeFollowState:
  def __init__(self):
    self.key = None
    self.blend = 0.0

  def acceleration_limits(self, limits, times, *, level, driving_mode, enabled, track_id,
                          gap_margin, v_rel, a_lead, a_ego, dt):
    """Return an upper envelope, not a brake command or a hard jerk limit.

    For constant accelerations, gap(H) = gap + v_rel*H +
    (a_lead-a_ego)*H^2/2. Solve for the ego acceleration that uses the
    remaining gap by H, and taper toward it only when ego is out-accelerating
    the lead. Negative acceleration remains unrestricted by this envelope.
    """
    values = (gap_margin, v_rel, a_lead, a_ego, dt)
    if not enabled or level not in (4, 5) or track_id < 0 or not all(map(math.isfinite, values)) or dt <= 0:
      self.key, self.blend = None, 0.0
      return limits.copy()

    key = (level, track_id)
    if key != self.key:
      self.blend = 0.0
    self.key = key
    safe = int(getattr(driving_mode, 'value', driving_mode)) == DRIVING_MODE_SAFE
    step = dt / MODE_BLEND_TIME
    self.blend = min(1.0, self.blend + step) if safe else max(0.0, self.blend - step)
    # No launch timer and no restriction while the lead is accelerating at least
    # as strongly as ego. Preserve the existing level 4/5 boost entry and limits.
    if self.blend == 0.0 or a_ego <= max(0.0, a_lead) + ACCEL_EXCESS_DEADBAND:
      return limits.copy()

    allowed = max(0.0, a_lead + 2.0 * (gap_margin + v_rel * SETTLE_HORIZON) / SETTLE_HORIZON**2)
    # Node zero must still admit the current acceleration. Later nodes lower the
    # positive envelope; original, stronger curve/braking limits always win.
    envelope = np.maximum(allowed, max(0.0, a_ego) - TAPER_RATE * times)
    target = np.minimum(limits, envelope)
    return limits + self.blend * (target - limits)
