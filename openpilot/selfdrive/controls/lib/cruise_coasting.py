"""Optional cruise overspeed brake relief; never changes the speed target."""
import math


MAX_COASTING_PERCENT = 10
TARGET_STABLE_TIME = 1.0
MIN_COASTING_SPEED = 10.0 / 3.6
MAX_PLAN_AGE = 0.2
BRAKE_RELEASE_JERK = 0.5  # maximum added brake relief per second, m/s^3


def coasting_percent(value):
  try:
    value = float(value)
  except (TypeError, ValueError):
    return 0
  return int(min(MAX_COASTING_PERCENT, max(0, value))) if math.isfinite(value) else 0


def no_coasting_lead(radar_state):
  # A cruise source labels only the first MPC node, not every future obstacle.
  # Keep both leads and cut-in protection entirely outside brake relief.
  return all(not getattr(getattr(radar_state, name, None), 'status', True)
             for name in ('leadOne', 'leadTwo', 'leadCutInRisk'))


class CruiseCoastingPlan:
  def __init__(self):
    self.reset()

  def reset(self):
    self.target = 0.0
    self.set_speed = 0.0
    self.percent = 0
    self.stable_time = 0.0

  def update(self, *, enabled, percent, set_speed, target, external_limit, dt):
    """Return a fixed physical-speed reference, or zero when ineligible.

    An external cap anywhere inside the overspeed band blocks relief, even if
    it has not yet reduced the ordinary cruise target. Targets are never raised.
    """
    percent = coasting_percent(percent)
    if (not enabled or percent == 0 or
        not all(math.isfinite(v) for v in (set_speed, target, external_limit, dt)) or
        target <= MIN_COASTING_SPEED or dt <= 0 or
        external_limit <= target * (1.0 + percent / 100.0)):
      self.reset()
      return 0.0

    if (percent != self.percent or abs(set_speed - self.set_speed) > 0.001 or
        abs(target - self.target) > 0.02):
      self.target, self.set_speed, self.percent = target, set_speed, percent
      self.stable_time = 0.0
    else:
      self.stable_time = min(TARGET_STABLE_TIME, self.stable_time + dt)
    return self.target if self.stable_time >= TARGET_STABLE_TIME else 0.0


def _smoothstep(x):
  x = min(1.0, max(0.0, x))
  return x * x * (3.0 - 2.0 * x)


def coasting_relief(v_ego, target, percent):
  """Zero outside the band; ease in over 10%, restore braking over its last 40%."""
  if (not all(math.isfinite(v) for v in (v_ego, target, percent)) or
      target <= MIN_COASTING_SPEED or not 0 < percent <= MAX_COASTING_PERCENT):
    return 0.0
  progress = (v_ego - target) / (target * percent / 100.0)
  return _smoothstep(progress / 0.1) * (1.0 - _smoothstep((progress - 0.6) / 0.4))


class CruiseCoastingControl:
  def __init__(self):
    self.reset()

  def reset(self):
    self.correction = 0.0

  def apply(self, accel, relief, dt):
    if not math.isfinite(accel) or accel >= 0.0 or relief <= 0.0:
      self.reset()
      return accel
    # Ease brake release; the speed envelope may always restore braking sooner.
    # Never retain compensation across a safety veto or create positive thrust.
    desired = -accel * min(1.0, relief)
    self.correction = min(desired, self.correction + BRAKE_RELEASE_JERK * dt)
    return min(0.0, accel + self.correction)
