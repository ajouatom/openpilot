"""Bounded, explicitly granted voice setting edits; never vehicle commands."""
from __future__ import annotations

import hashlib
import json
import math
import os
import secrets
import time
from decimal import Decimal, ROUND_HALF_UP
from pathlib import Path

# These presentation settings can change while moving. All other catalog edits
# require fresh stationary/disengaged state, including at final application.
ONROAD_SETTINGS = frozenset({'ShowCustomBrightness', 'ClusterHudBrightness', 'SoundVolumeAdjust'})
DENIED_SETTINGS = frozenset({'DriverMonitoringEnabled', 'DriverMonitoringSessionDisabled', 'JetsonLegacyUpdatePending'})


def number(value):
  if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
    raise ValueError('A finite numeric value is required')
  return value


class AssistantSettings:
  def __init__(self, path, catalog, read, write, stationary, record, clock=time.monotonic):
    self.path = Path(path)
    self.catalog, self.read, self.write = catalog, read, write
    self.stationary, self.record, self.clock = stationary, record, clock
    self.grants, self.proposals, self.results = {}, {}, {}
    try:
      saved = json.loads(self.path.read_text())
    except FileNotFoundError:
      saved = {'version': 1, 'grants': {}}
    if saved.get('version') != 1 or not isinstance(saved.get('grants'), dict) or len(saved['grants']) > 100:
      raise ValueError('Invalid voice permission store')
    for digest, origin in saved['grants'].items():
      if len(digest) != 64 or any(c not in '0123456789abcdef' for c in digest) or not isinstance(origin, str):
        raise ValueError('Invalid voice permission store')
    self.grants = saved['grants']

  def _save(self, grants):
    self.path.parent.mkdir(parents=True, exist_ok=True)
    temporary = self.path.with_suffix('.tmp')
    fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, 'w') as out:
      json.dump({'version': 1, 'grants': grants}, out)
      out.flush()
      os.fsync(out.fileno())
    os.replace(temporary, self.path)
    self.grants = grants

  @staticmethod
  def _digest(token):
    return hashlib.sha256(str(token).encode()).hexdigest()

  def enable(self, origin):
    if not self.stationary():
      raise ValueError('Stop and disengage before enabling voice setting access')
    if len(self.grants) >= 100:
      raise ValueError('Voice permission capacity reached')
    token = secrets.token_urlsafe(32)
    self._save({**self.grants, self._digest(token): origin})
    return token

  def authorize(self, token, origin):
    digest = self._digest(token)
    if self.grants.get(digest) != origin:
      raise PermissionError('Enable setting access in this Carrot Web connection first')
    return digest

  def revoke(self, token, origin):
    digest = self.authorize(token, origin)
    self._save({k: v for k, v in self.grants.items() if k != digest})
    self.proposals = {k: v for k, v in self.proposals.items() if v['owner'] != digest}
    self.results = {k: v for k, v in self.results.items() if k[0] != digest}

  def snapshot(self, name):
    meta = self.catalog().get(name)
    if not meta or name in DENIED_SETTINGS:
      raise ValueError('This setting is not available for voice access')
    low, high, step = number(meta.get('min')), number(meta.get('max')), number(meta.get('unit', 1))
    if high < low or step <= 0 or any(int(v) != v for v in (low, high, step)):
      raise ValueError('Only bounded integer catalog settings are supported')
    current = self.read(name)
    if current is None:
      raise ValueError('No current device value; a catalog default is not a live value')
    current = number(current)
    if int(current) != current:
      raise ValueError('Invalid current device value')
    return {'name': name, 'title': str(meta.get('title', name))[:160], 'current': int(current),
            'min': int(low), 'max': int(high), 'step': int(step),
            'unit_note': 'Values are the integers displayed/stored by Carrot Web; honor any scale in the setting title.',
            'stationary_required': name not in ONROAD_SETTINGS}

  def _guard(self, snapshot):
    if snapshot['stationary_required'] and not self.stationary():
      raise ValueError('This setting can be changed only with fresh stationary/disengaged state')

  def execute(self, token, origin, command):
    owner = self.authorize(token, origin)
    now = self.clock()
    self.proposals = {k: v for k, v in self.proposals.items() if v['until'] > now}
    self.results = {k: v for k, v in self.results.items() if v[0] > now}
    request_id = command.get('id')
    if not isinstance(request_id, str) or len(request_id) != 43:
      raise ValueError('Invalid request identity')
    key = (owner, request_id)
    if key in self.results:
      return self.results[key][1]
    if len(self.results) >= 1000:
      raise ValueError('Too many requests; try later')
    operation = command.get('operation')
    if operation == 'commit':
      proposal = self.proposals.get(command.get('proposal_id'))
      if not proposal or proposal['owner'] != owner:
        raise ValueError('Proposal expired or belongs to another connection; prepare it again')
      # Consume before writing. Retrying an uncertain result cannot apply twice.
      del self.proposals[command['proposal_id']]
      snapshot = self.snapshot(proposal['name'])
      self._guard(snapshot)
      if any(snapshot[k] != proposal[k] for k in ('current', 'min', 'max', 'step')):
        raise ValueError('Setting value or definition changed; prepare and confirm again')
      result = {'status': 'failed', 'name': proposal['name'], 'error': 'Application not verified; read current value before retrying'}
      self.results[key] = (now + 120, result)
      self.write(proposal['name'], proposal['proposed'])
      actual = self.read(proposal['name'])
      if actual != proposal['proposed']:
        raise ValueError('Device readback differs; application was not confirmed')
      self.record(proposal['name'], proposal['current'], actual)
      result = {'status': 'applied', 'name': proposal['name'], 'previous': proposal['current'], 'current': actual}
    elif operation in ('read', 'prepare'):
      snapshot = self.snapshot(command.get('name'))
      result = {'status': 'read', **snapshot}
      if operation == 'prepare':
        self._guard(snapshot)
        amount = number(command.get('value'))
        mode = command.get('mode')
        current = Decimal(str(snapshot['current']))
        delta = Decimal(str(amount))
        if mode == 'absolute':
          target = delta
        elif mode in ('delta', 'percent'):
          if not snapshot['min'] <= current <= snapshot['max']:
            raise ValueError('Current value has a special meaning; choose an explicit absolute value')
          target = current + (delta if mode == 'delta' else abs(current) * delta / 100)
        else:
          raise ValueError('Choose absolute, delta or percent')
        low, high, step = (Decimal(str(snapshot[k])) for k in ('min', 'max', 'step'))
        if not low <= target <= high:
          raise ValueError('Requested value is outside the allowed range; no value was changed')
        proposed = low + ((target - low) / step).quantize(Decimal('1'), rounding=ROUND_HALF_UP) * step
        if not low <= proposed <= high or proposed == current:
          raise ValueError('No valid change at this setting step; choose another value')
        if len(self.proposals) >= 100:
          raise ValueError('Too many pending proposals')
        # A correction replaces the previous proposal for this owner.
        self.proposals = {k: v for k, v in self.proposals.items() if v['owner'] != owner}
        proposal_id = secrets.token_urlsafe(32)
        self.proposals[proposal_id] = {**snapshot, 'proposed': int(proposed), 'owner': owner, 'until': now + 60}
        result = {'status': 'prepared', **snapshot, 'proposed': int(proposed), 'proposal_id': proposal_id,
                  'expires_in_seconds': 60, 'confirmation_required': True}
    elif operation == 'cancel':
      self.proposals = {k: v for k, v in self.proposals.items() if v['owner'] != owner}
      result = {'status': 'cancelled'}
    else:
      raise ValueError('Unknown setting operation')
    self.results[key] = (now + 120, result)
    return result
