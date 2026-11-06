"""Separately granted, one-shot driving requests. No arbitrary commands/Params."""
import time
from pathlib import Path

from openpilot.selfdrive.carrot.bluetooth.model import atomic_json, read_json
from openpilot.selfdrive.controls.lib.desire_lib.assistant_lane import ROOT, IDENTITY, TERMINAL
from .assistant_settings import AssistantSettings

ACTIONS = {'lane_left', 'lane_right', 'speed_up', 'speed_down', 'speed_set'}
FINISHED = TERMINAL | {'applied', 'unchanged'}


class AssistantDrive:
  def __init__(self, path, stationary, root=ROOT, clock=time.monotonic):
    self.permissions = AssistantSettings(path, dict, lambda _: None, lambda *_: None, stationary, lambda *_: None, clock)
    self.root, self.clock = Path(root), clock
    self.requests = {}

  def enable(self, origin):
    return self.permissions.enable(origin)

  def revoke(self, token, origin):
    owner = self.permissions.authorize(token, origin)
    self.permissions.revoke(token, origin)
    # Runtime ownership also survives a web-server restart during preparation.
    for root in (self.root, self.root / 'speed'):
      request = read_json(root / 'request.json', {})
      if isinstance(request, dict) and request.get('owner') == owner and IDENTITY.fullmatch(str(request.get('id', ''))):
        atomic_json(root / 'cancel.json', {'id': request['id']})

  def execute(self, token, origin, command):
    owner = self.permissions.authorize(token, origin)
    now = self.clock()
    self.requests = {key: value for key, value in self.requests.items() if now - value['time'] < 120}
    identity = command.get('id')
    if not isinstance(identity, str) or not IDENTITY.fullmatch(identity):
      raise ValueError('Invalid request identity')
    existing = self.requests.get(identity)
    if existing:
      if existing['owner'] != owner:
        raise PermissionError('Request belongs to another connection')
      return self.status(existing)
    if command.get('operation') != 'execute':
      raise ValueError('Request is no longer available; do not repeat it automatically')
    action = command.get('action')
    if action not in ACTIONS:
      raise ValueError('Unsupported driving command')
    value = command.get('value')
    if action in ('speed_down', 'speed_set'):
      if type(value) is not int or not 1 <= value <= 199:
        raise ValueError('An explicit integer speed or decrease from 1 to 199 km/h is required')
    elif value is not None:
      raise ValueError('This action does not accept a speed')
    if len(self.requests) >= 1000:
      raise ValueError('Too many requests')
    # One maneuver across all approved web origins. Never queue the next one.
    for path in (self.root, self.root / 'speed'):
      pending = read_json(path / 'request.json', {})
      result = read_json(path / 'result.json', {})
      if isinstance(pending, dict) and isinstance(pending.get('time'), (int, float)) and 0 <= now - pending['time'] < 20:
        if not isinstance(result, dict) or result.get('id') != pending.get('id') or result.get('status') not in FINISHED:
          raise ValueError('A driving request is already pending; check its result')
    root = self.root if action.startswith('lane_') else self.root / 'speed'
    request = {'id': identity, 'owner': owner, 'action': action, 'value': value, 'time': now, 'root': root}
    self.requests[identity] = request  # Consume before publishing; uncertain writes are never retried.
    atomic_json(root / 'request.json', {'id': identity, 'owner': owner, 'time': now, 'action': action, 'value': value,
                                      'direction': action.removeprefix('lane_') if action.startswith('lane_') else None})
    return self.status(request)

  def status(self, request):
    now = self.clock()
    result = read_json(request['root'] / 'result.json', {})
    if isinstance(result, dict) and result.get('id') == request['id'] and result.get('status') in FINISHED | {'waiting', 'running'}:
      stamp = result.get('time')
      if isinstance(stamp, (int, float)) and request['time'] <= stamp <= now:
        if result['status'] in FINISHED or now - stamp < 1:
          return {key: result[key] for key in ('id', 'status', 'direction', 'reason', 'previous', 'current') if key in result}
    return {'id': request['id'], 'status': 'waiting' if now - request['time'] <= 1 else 'unverified',
            'reason': 'awaiting_vehicle' if now - request['time'] <= 1 else 'vehicle_feedback_unavailable'}
