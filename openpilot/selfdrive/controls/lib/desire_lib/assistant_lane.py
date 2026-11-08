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
from openpilot.selfdrive.controls.lib.desire_lib.indicator_feedback import TapEcho

ROOT = Path('/dev/shm/carrot-assistant-lane')
IDENTITY = re.compile(r'^[A-Za-z0-9_-]{43}$')
TERMINAL = {'completed', 'rejected', 'expired', 'interrupted', 'unverified', 'cancelled'}


class AssistantLane:
  def __init__(self, root=ROOT, clock=time.monotonic, *, fingerprint='', native_signal=False):
    self.root, self.clock = Path(root), clock
    self.fingerprint = fingerprint
    self.native_signal = native_signal
    self.started = clock()
    self.seen = deque(maxlen=128)
    self.active = None
    self.last_publish = 0.
    self.result = None
    self.last_tick = None
    self.tap_echo = TapEcho()

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

  def blocked(self, state, direction):
    if not state['valid']:
      return 'vehicle_state_unavailable'
    if not state['lateral_active']:
      return 'lateral_control_inactive'
    if not state['drive'] or state['trailer']:
      return 'vehicle_condition'
    if not math.isfinite(state['speed']) or state['speed'] < 30 / 3.6:
      return 'below_lane_change_speed'
    if state['driver_busy'] or state['other_request'] or (self.signal_route != 'ordinary' and state.get('indicator_driver_busy')):
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
          self.active['signal_route'] = self.indicator_route(now)
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
      timeout = 'indicator_not_confirmed' if self.signal_id else 'start_not_observed'
      self.publish('rejected' if reason else 'expired', reason or timeout)
      return None, True
    self.publish('waiting', 'existing_lane_change_conditions')
    return active['direction'], False

  def indicator_route(self, now):
    if self.native_signal:
      return 'native'
    available = read_json(self.root / 'indicator-device.json', {})
    if isinstance(available, dict) and self.fingerprint and available.get('fingerprint') == self.fingerprint:
      stamp, route = available.get('time'), available.get('route')
      if type(stamp) in (int, float) and 0 <= now - stamp < .6 and isinstance(route, str) and IDENTITY.fullmatch(route):
        return route
    return 'ordinary'

  @property
  def signal_route(self):
    return self.active.get('signal_route', '') if self.active else ''

  @property
  def signal_id(self):
    return self.active['id'] if self.active and 'signal_since' in self.active else ''

  def own_tap(self, cs):
    if not self.signal_id or not cs.blinkerLever:
      return False
    value = read_json(self.root / 'signal.json', {})
    if (not isinstance(value, dict) or value.get('id') != self.signal_id or value.get('route') != self.signal_route or
        value.get('direction') != self.active['direction']):
      return False
    direction = 1 if self.active['direction'] == 'left' else 2
    return self.tap_echo.matches(self.clock(), cs, direction, 0., {'stamp':value.get('time'), 'tap':value.get('tap')})

  def request_signal(self, direction, cs):
    active = self.active
    now = self.clock()
    if not active or active['direction'] != direction:
      return False
    active.setdefault('signal_since', now)
    # An external adapter follows the admitted maneuver. Only native output
    # requires vehicle feedback before the transition to laneChangeStarting.
    if self.signal_route != 'native':
      return True
    feedback = read_json(self.root / 'signal.json', {})
    if not isinstance(feedback, dict):
      return False
    stamp = feedback.get('time')
    return (feedback.get('confirmed') is True and feedback.get('id') == active['id'] and feedback.get('direction') == direction and
            type(stamp) in (int, float) and active['signal_since'] <= stamp <= now and now - stamp < .35 and
            feedback.get('route') == self.signal_route and
            not (cs.rightBlinker if direction == 'left' else cs.leftBlinker))

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
    elif self.signal_id and not active['began'] and phase != 'preLaneChange':
      self.publish('rejected', 'lane_conditions_changed')
    elif active['began']:
      # An FSM reset/timeout alone is never evidence of crossing a lane.
      self.publish('completed' if active['finishing'] and active['crossed'] else 'unverified',
                   '' if active['finishing'] and active['crossed'] else 'completion_not_observed')
