"""Lead acceleration persistence shared by radard, fast radar and NAS replay.

This changes the predicted obstacle, never ego comfort costs or actuator limits.
Despite its historical name, aLeadTau is the decay coefficient in
  a(t) = aLead * exp(-aLeadTau * t**2 / 2).
A smaller coefficient retains more of the measured braking in the prediction.
"""

import math


LEAD_ACCEL_TAU_S = 1.5
LEAD_ACCEL_FILTER_TAU_S = 0.45
LEAD_ACCEL_DT_S = 0.05
LEAD_ACCEL_FILTER_ALPHA = LEAD_ACCEL_DT_S / (LEAD_ACCEL_FILTER_TAU_S + LEAD_ACCEL_DT_S)

# Only corroborated braking onset gets a faster attack. Both inputs are already
# filtered by MyTrack. Smooth ramps avoid a mode jump at either threshold.
BRAKING_ACCEL_START = 0.5
BRAKING_ACCEL_FULL = 1.5
BRAKING_JERK_START = 1.5
BRAKING_JERK_FULL = 3.0
BRAKING_TAU_ATTACK_RC = 0.05
BRAKING_CONFIRM_MAX_GAP_S = 0.15


def _ramp(value: float, start: float, full: float) -> float:
  return max(0.0, min(1.0, (value - start) / (full - start)))


class LeadAccelTau:
  def __init__(self, initial_tau: float = LEAD_ACCEL_TAU_S):
    self.tau = initial_tau if math.isfinite(initial_tau) and 0.0 <= initial_tau <= LEAD_ACCEL_TAU_S else LEAD_ACCEL_TAU_S
    self._ordinary_tau = self.tau
    self._previous_strength = 0.0
    self._sample_time_s: float | None = None

  def clear_evidence(self) -> None:
    self._previous_strength = 0.0
    self._sample_time_s = None

  def update(self, a_lead: float, j_lead: float, sample_time_s: float, *, measured: bool = True) -> float:
    if not all(math.isfinite(value) for value in (a_lead, j_lead, sample_time_s)):
      self.clear_evidence()
      self.tau = LEAD_ACCEL_TAU_S
      self._ordinary_tau = self.tau
      return self.tau

    # Preserve the ordinary per-update policy, including its quiet reset. The
    # extra attack requires two distinct consecutive measurements, not repeated
    # model publications of the same radar sample or evidence across a dropout.
    strength = (_ramp(-a_lead, BRAKING_ACCEL_START, BRAKING_ACCEL_FULL)
                * _ramp(-j_lead, BRAKING_JERK_START, BRAKING_JERK_FULL)) if measured else 0.0
    confirmed = 0.0
    if self._sample_time_s is not None and sample_time_s < self._sample_time_s:
      strength = 0.0  # An out-of-order sample cannot seed the next confirmation.
    if self._sample_time_s is not None and 0.0 < sample_time_s - self._sample_time_s <= BRAKING_CONFIRM_MAX_GAP_S:
      confirmed = min(strength, self._previous_strength)
    if self._sample_time_s is None or sample_time_s != self._sample_time_s:
      self._previous_strength = strength
      self._sample_time_s = sample_time_s
    if not measured:
      self._previous_strength = 0.0

    if abs(a_lead) < 0.5 and abs(j_lead) < 0.5:
      self.tau = LEAD_ACCEL_TAU_S
      self._ordinary_tau = self.tau
    else:
      self._ordinary_tau *= 1.0 - LEAD_ACCEL_FILTER_ALPHA
      fast_alpha = LEAD_ACCEL_DT_S / (BRAKING_TAU_ATTACK_RC + LEAD_ACCEL_DT_S)
      alpha = LEAD_ACCEL_FILTER_ALPHA + confirmed * (fast_alpha - LEAD_ACCEL_FILTER_ALPHA)
      self.tau *= 1.0 - alpha
      # A braking-only correction must not prolong positive acceleration when
      # the lead pulls away before the ordinary quiet reset can occur.
      if a_lead >= 0.0:
        self.tau = self._ordinary_tau
    return self.tau
