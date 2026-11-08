"""Consume fresh lamp-only edges once, journal ownership before any CAN request."""
import math

from blink_boundary import BoundaryEvents
from openpilot.selfdrive.controls.lib.desire_lib.boundary_lane import MANUAL_INDICATOR_MIN_SPEED, adjacent_lane


def allowed(cs, cc, md, source, now, holding=False, tap_echo=False):
  try:
    if (not cs.canValid or str(cs.gearShifter) != 'drive' or cs.trailerConnected or
        cs.steerFaultTemporary or cs.steerFaultPermanent or (cs.blinkerLever and not (holding and tap_echo)) or
        not math.isfinite(cs.vEgo) or cs.vEgo < 0):
      return False
    if source not in ('manual', 'manual_turn') and not cc.latActive:
      return False
    if holding:
      return True
    direction = {'left':1, 'right':2}.get(str(md.meta.observedIndicator), 0)
    minimum_speed = 10/3.6 if source == 'model_turn' else 30/3.6
    if source in ('manual', 'manual_turn'):
      minimum_speed = MANUAL_INDICATOR_MIN_SPEED
    return (direction != 0 and not cs.leftBlinker and not cs.rightBlinker and cs.vEgo >= minimum_speed and
            str(md.meta.laneChangeState) in ('off', 'preLaneChange') and
            (source in ('model_turn', 'manual_turn') or adjacent_lane(cs, md, direction, now)))
  except (AttributeError, TypeError, ValueError):
    return False


class ObservationSignals:
  def __init__(self, journal=None):
    self.events = BoundaryEvents(journal)
    self.owned = None
    self.reason = 'idle'
    self.saved_at = 0.

  def update(self, now, meta, *, allowed, busy, record):
    direction = {'left':1, 'right':2}.get(str(getattr(meta, 'observedIndicator', 'none')), 0)
    stamp = getattr(meta, 'observedIndicatorMonoTime', 0)/1e9
    source = str(getattr(meta, 'observedIndicatorSource', ''))
    isolated = getattr(meta, 'indicatorIsolationMask', 0) & direction
    edge = self.events.read(now, direction, stamp, allowed=allowed and not busy and bool(isolated), record=record)
    if self.owned and (not allowed or busy or not direction or not isolated or
                       (stamp, direction) != self.owned or not 0 <= now-stamp < 30):
      self.owned = None
    if edge:
      self.owned = (stamp, direction)
      self.saved_at = now
    if self.owned and now-self.saved_at >= 1.:
      value = {'time':now, 'direction':direction, 'mask':self.events.journal.get('mask', direction) | direction}
      try:
        record(value)
      except OSError:
        self.owned = None
      else:
        self.events.journal, self.saved_at = value, now
    self.reason = source if self.owned else 'idle'
    return ('observation', 'left' if direction == 1 else 'right') if self.owned else None


class LowSpeedOwnership:
  def __init__(self):
    self.stamp = None
    self.journal = BoundaryEvents()

  def ready(self, now, started, meta, record):
    if self.stamp != started:
      # Source-created low-speed TBT has no detector edge. Persist once, then
      # wait for modeld to acknowledge lamp-feedback isolation before sending.
      previous = self.journal.journal
      mask = 2
      if isinstance(previous, dict) and isinstance(previous.get('time'), (int, float)) and 0 <= now-previous['time'] < 20:
        mask |= previous.get('mask', previous.get('direction', 0)) & 3
      value = {'time':started, 'direction':2, 'mask':mask}
      try:
        record(value)
      except OSError:
        return False
      self.journal.journal, self.stamp = value, started
    ack = getattr(meta, 'indicatorJournalMonoTime', 0)/1e9
    return math.isfinite(ack) and ack >= started-1e-8 and bool(getattr(meta, 'indicatorIsolationMask', 0) & 2)


def wire_hold(hold):
  # Receiver packet format and CAN encoding are unchanged. Local policy keeps
  # the observation origin; the existing receiver only needs direction/id/lease.
  return {**hold, 'source':'lane'} if hold and hold['source'] in ('observation', 'tbt_display') else hold
