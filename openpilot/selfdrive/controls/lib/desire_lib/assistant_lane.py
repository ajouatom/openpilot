"""One-shot assistant intent; the existing lane-change FSM still owns motion.

This channel is separate from physical remote input. A web authorization cannot
impersonate a Bluetooth button, steering torque, or a turn-signal acknowledgement.
"""
from collections import deque
from pathlib import Path
import math
import re
import time

from openpilot.selfdrive.carrot.bluetooth.model import atomic_json, read_json

ROOT = Path('/dev/shm/carrot-assistant-lane')
IDENTITY = re.compile(r'^[A-Za-z0-9_-]{43}$')
TERMINAL = {'completed', 'rejected', 'expired', 'interrupted', 'unverified', 'cancelled'}


class AssistantLane:
  def __init__(self, root=ROOT, clock=time.monotonic):
    self.root, self.clock = Path(root), clock
    self.started = clock()
    self.seen = deque(maxlen=128)
    self.active = None
    self.last_publish = 0.
    self.result = None
    self.last_tick = None

  def publish(self, status, reason=''):
    if self.active is None:
      return
    value = {'id': self.active['id'], 'status': status, 'reason': reason}
    if self.active.get('direction') in ('left', 'right'):
      value['direction'] = self.active['direction']
    if status in TERMINAL:
      self.active = None
    now = self.clock()
    if value != self.result or now - self.last_publish >= .25:
      try:
        atomic_json(self.root / 'result.json', {**value, 'time': now})
        self.result, self.last_publish = value, now
      except OSError:
        pass  # Missing feedback cannot undo a rejection/cancellation.

  @staticmethod
  def blocked(state, direction):
    if not state['valid']:
      return 'vehicle_state_unavailable'
    if not state['lateral_active']:
      return 'lateral_control_inactive'
    if not state['drive'] or state['trailer']:
      return 'vehicle_condition'
    if not math.isfinite(state['speed']) or state['speed'] < 30 / 3.6:
      return 'below_lane_change_speed'
    if state['driver_busy'] or state['other_request']:
      return 'driver_or_other_maneuver'
    side = state[direction]
    if side['blindspot']:
      return 'blindspot'
    if not side['available']:
      return 'lane_unavailable'
    return ''

  def before(self, state):
    """Return (direction to request, cancel our not-yet-started preparation)."""
    now = self.clock()
    gap = self.last_tick is not None and now - self.last_tick > .35
    self.last_tick = now
    command = read_json(self.root / 'request.json', {})
    if isinstance(command, dict) and IDENTITY.fullmatch(str(command.get('id', ''))) and command['id'] not in self.seen:
      self.seen.append(command['id'])
      if self.active is None:
        self.active = {**command, 'began': False, 'finishing': False, 'crossed': False, 'accepted': now}
        stamp = command.get('time')
        if command.get('direction') not in ('left', 'right') or type(stamp) not in (float, int) or not self.started <= stamp <= now or now - stamp > 1:
          self.publish('expired', 'request_expired')
        elif state['phase'] != 'off':
          self.publish('rejected', 'maneuver_already_active')
        else:
          reason = self.blocked(state, command['direction'])
          if reason:
            self.publish('rejected', reason)
    if self.active is None:
      return None, False
    active = self.active
    cancelled = read_json(self.root / 'cancel.json', {})
    if isinstance(cancelled, dict) and cancelled.get('id') == active['id']:
      self.publish('interrupted' if active['began'] else 'cancelled', 'permission_revoked')
      return None, not active['began']
    if active['began']:
      # Never force a steering/path transition once a maneuver is underway.
      if not state['valid'] or not state['lateral_active'] or state['driver_override']:
        self.publish('interrupted', 'driver_override_or_state_lost')
      elif now - active['accepted'] > 15:
        self.publish('unverified', 'completion_not_observed')
      return None, False
    reason = self.blocked(state, active['direction'])
    if reason or gap or now - active['accepted'] > 3:
      self.publish('rejected' if reason else 'expired', reason or 'start_not_observed')
      return None, True
    self.publish('waiting', 'existing_lane_change_conditions')
    return active['direction'], False

  def after(self, phase, direction, *, crossed=False):
    if self.active is None:
      return
    active = self.active
    if phase in ('laneChangeStarting', 'laneChangeFinishing'):
      if direction != active['direction']:
        self.publish('interrupted', 'different_maneuver')
        return
      active['began'] = True
      active['crossed'] |= crossed
      active['finishing'] |= phase == 'laneChangeFinishing'
      self.publish('running', 'lane_change_in_progress')
    elif active['began']:
      # An FSM reset/timeout alone is never evidence of crossing a lane.
      self.publish('completed' if active['finishing'] and active['crossed'] else 'unverified',
                   '' if active['finishing'] and active['crossed'] else 'completion_not_observed')
