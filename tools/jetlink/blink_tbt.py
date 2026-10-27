"""TBT-only physical right indicator request; never emits a steering/lane command."""
import math
from openpilot.selfdrive.controls.lib.desire_lib.constants import LANE_CHANGE_SPEED_MIN

CONFIRM_SEC = 1.0
REPEAT_DELAY_SEC = 2.0
START_TIMEOUT_SEC = 4.0
MANEUVER_TIMEOUT_SEC = 15.0
TBT_DISTANCE_MAX_M = 250.0


def candidate(cs, model, nav):
  """The ordinary maneuver controller remains responsible after lamp feedback."""
  meta = model.meta
  try:
    return (nav.xTurnInfo == 2 and math.isfinite(nav.xDistToTurn) and
            0 < nav.xDistToTurn <= TBT_DISTANCE_MAX_M and
            math.isfinite(cs.vEgo) and cs.vEgo >= LANE_CHANGE_SPEED_MIN and
            str(cs.gearShifter) == 'drive' and cs.canValid and not cs.trailerConnected and
            not cs.steeringPressed and not cs.steerFaultTemporary and not cs.steerFaultPermanent and
            cs.tbtRearInputsAvailable and not cs.tbtRightRearBlocked and not cs.rightBlindspot and
            meta.laneChangeAvailableRight and 2.5 < meta.laneWidthRight < 5.0 and
            model.laneLineProbs[2] > .5 and model.laneLineProbs[3] > .5 and
            cs.rightLaneLine >= 0 and cs.rightLaneLine % 10 in (0, 5) and cs.rightLaneLine // 10 != 2 and
            len(meta.desireState) >= 3 and meta.desireState[1] + meta.desireState[2] <= .1)
  except (AttributeError, IndexError, TypeError, ValueError):
    return False


class TbtBlinker:
  def __init__(self):
    self.phase = 'idle'
    self.ready_since = None
    self.started = 0.0
    self.completed = 0.0
    self.last_distance = None
    self.last_time = None
    self.reason = 'idle'

  def cancel(self, reason):
    self.phase, self.ready_since, self.reason = 'cancelled', None, reason
    return None

  def update(self, now, *, enabled, valid, cs, model, nav, manual=False, busy=False):
    # Gaps reset confirmation and cancel an owned indication, never replay it.
    gap = self.last_time is not None and not 0 <= now-self.last_time <= .35
    self.last_time = now
    if not enabled:
      self.phase, self.ready_since, self.last_distance = 'idle', None, None
      self.reason = 'disabled'
      return None
    if not valid or gap:
      self.ready_since = None
      return self.cancel('inputs') if self.phase in ('waiting', 'changing', 'finishing') else None
    distance = nav.xDistToTurn
    if nav.xTurnInfo != 2 or not math.isfinite(distance) or distance <= 0 or distance > TBT_DISTANCE_MAX_M:
      self.phase, self.ready_since, self.last_distance = 'idle', None, None
      self.reason = 'route'
      return None
    if str(getattr(model.meta, 'desire', 'none')) in ('turnLeft', 'turnRight'):
      return self.cancel('turn_started')
    previous, self.last_distance = self.last_distance, distance
    if previous is not None and distance > previous + 30:
      return self.cancel('route_jump')
    if manual or cs.steeringPressed or cs.leftBlinker:
      return self.cancel('driver')
    if self.phase == 'cancelled':
      return None  # Same TBT must not restart after driver cancellation/timeout.
    state, direction = str(model.meta.laneChangeState), str(model.meta.laneChangeDirection)
    active = state in ('laneChangeStarting', 'laneChangeFinishing')
    if self.phase in ('waiting', 'changing', 'finishing'):
      if now-self.started > MANEUVER_TIMEOUT_SEC or (active and direction != 'right'):
        return self.cancel('maneuver')
      if active:
        self.phase = 'finishing' if state == 'laneChangeFinishing' else 'changing'
      elif self.phase == 'finishing':
        self.phase, self.completed, self.ready_since = 'cooldown', now, None
        self.reason = 'completed'
        return None
      elif self.phase == 'changing':
        return self.cancel('interrupted')
      elif now-self.started >= START_TIMEOUT_SEC or not candidate(cs, model, nav):
        return self.cancel('start')
      self.reason = self.phase
      return ('lane', 'right')  # Existing physical-indicator policy/transport only.
    if self.phase == 'cooldown' and now-self.completed < REPEAT_DELAY_SEC:
      return None
    if state != 'off' or cs.leftBlinker or cs.rightBlinker or busy or not candidate(cs, model, nav):
      self.ready_since = None
      self.reason = 'waiting_conditions'
      return None
    self.ready_since = now if self.ready_since is None else self.ready_since
    if now-self.ready_since < CONFIRM_SEC:
      self.reason = 'confirming'
      return None
    self.phase, self.started, self.reason = 'waiting', now, 'trigger'
    return ('lane', 'right')
