"""Consume a fresh, single speed intent; no engagement or pedal commands."""
from collections import deque
from pathlib import Path
import time

from openpilot.selfdrive.carrot.bluetooth.model import atomic_json, read_json
from openpilot.selfdrive.controls.lib.desire_lib.assistant_lane import ROOT, IDENTITY


class AssistantSpeed:
  def __init__(self, root=ROOT / 'speed', clock=time.monotonic):
    self.root, self.clock = Path(root), clock
    self.started = clock()
    self.seen = deque(maxlen=128)
    self.active = None

  def publish(self, status, reason='', current=None):
    if self.active is None:
      return
    result = {'id': self.active['id'], 'status': status, 'reason': reason, 'time': self.clock()}
    if current is not None:
      result.update(previous=self.active['previous'], current=float(current))
    self.active = None
    try:
      atomic_json(self.root / 'result.json', result)
    except OSError:
      pass  # The consumed command must remain consumed without a result file.

  def read(self, *, allowed, current, minimum, maximum):
    command = read_json(self.root / 'request.json', {})
    if not isinstance(command, dict) or not IDENTITY.fullmatch(str(command.get('id', ''))) or command['id'] in self.seen:
      return None
    self.seen.append(command['id'])
    self.active = {**command, 'previous': float(current)}
    now, stamp = self.clock(), command.get('time')
    cancelled = read_json(self.root / 'cancel.json', {})
    if isinstance(cancelled, dict) and cancelled.get('id') == command['id']:
      self.publish('cancelled', 'permission_revoked')
    elif type(stamp) not in (float, int) or not self.started <= stamp <= now or now - stamp > 1:
      self.publish('expired', 'request_expired')
    elif not allowed:
      self.publish('rejected', 'cruise_inactive_or_driver_override')
    elif command.get('action') == 'speed_up':
      return command
    elif command.get('action') in ('speed_down', 'speed_set'):
      value = command.get('value')
      if type(value) is not int or not 1 <= value <= 199:
        self.publish('rejected', 'invalid_speed')
      else:
        target = current - value if command['action'] == 'speed_down' else value
        if minimum <= target <= maximum:
          self.active['target'] = float(target)
          return {**command, 'target': target}
        self.publish('rejected', 'speed_out_of_range')
    else:
      self.publish('rejected', 'unsupported_command')
    return None

  def finish(self, current):
    if self.active is None:
      return
    target = self.active.get('target')
    if target is not None and abs(current - target) > .01:
      self.publish('unverified', 'target_overridden', current)
    elif self.active['action'] == 'speed_up' and current < self.active['previous']:
      self.publish('unverified', 'target_overridden', current)
    else:
      self.publish('unchanged' if abs(current - self.active['previous']) < .01 else 'applied', current=current)
