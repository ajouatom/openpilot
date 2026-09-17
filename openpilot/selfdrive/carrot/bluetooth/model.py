"""Validated mapping storage, HID decoding and short-lived local commands."""
import json
import os
from pathlib import Path
import re
import time
import uuid

CONFIG_PATH = Path('/data/carrot/bluetooth.json')
RUNTIME = Path('/dev/shm/carrot-bluetooth')
ACTIONS = ('none', 'accelCruise', 'decelCruise', 'laneLeft', 'laneRight', 'paddleDecel', 'gapAdjustCruise')
DEFAULT_MAPPING = {'up': 'accelCruise', 'down': 'decelCruise', 'left': 'laneLeft', 'right': 'laneRight',
                   'center': 'paddleDecel', '1': 'gapAdjustCruise', '2': 'none'}
MAC = re.compile(r'^(?:[0-9A-F]{2}:){5}[0-9A-F]{2}$')
TOKEN = re.compile(r'^(?:up|down|left|right|center|1|2|key:[0-9]{1,4}|swipe:[xy][+-]|tap:[0-9]{1,5}:[0-9]{1,5})$')


def address(value):
  value = str(value).upper()
  if not MAC.fullmatch(value):
    raise ValueError('invalid Bluetooth address')
  return value


def read_json(path, default=None):
  try:
    return json.loads(Path(path).read_text(encoding='utf-8'))
  except (OSError, ValueError):
    return default


def atomic_json(path, value, mode=0o600):
  path = Path(path)
  path.parent.mkdir(parents=True, exist_ok=True)
  temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
  try:
    with temporary.open('x', encoding='utf-8') as f:
      os.chmod(temporary, mode)
      json.dump(value, f, ensure_ascii=False)
      f.flush()
    os.replace(temporary, path)
  finally:
    temporary.unlink(missing_ok=True)


def validate_config(data):
  if not isinstance(data, dict) or not isinstance(data.get('devices', {}), dict):
    raise ValueError('devices must be an object')
  if len(data.get('devices', {})) > 16:
    raise ValueError('at most 16 remotes')
  result = {'version': 1, 'devices': {}}
  for mac, device in data.get('devices', {}).items():
    mac = address(mac)
    if not isinstance(device, dict) or device.get('profile') not in ('yiser-j6', 'generic'):
      raise ValueError('unknown input profile')
    mapping = device.get('mapping', {})
    if not isinstance(mapping, dict) or len(mapping) > 64:
      raise ValueError('invalid mapping')
    for key, action in mapping.items():
      if not TOKEN.fullmatch(key) or action not in ACTIONS:
        raise ValueError('invalid button or action')
      if key.startswith('key:') and not 0 <= int(key[4:]) <= 767:
        raise ValueError('invalid Linux key code')
    if type(device.get('enabled', False)) is not bool:
      raise ValueError('enabled must be boolean')
    result['devices'][mac] = {'name': str(device.get('name', mac))[:80], 'profile': device['profile'],
                              'enabled': device.get('enabled', False), 'mapping': dict(mapping)}
  return result


def config(path=CONFIG_PATH):
  try:
    return validate_config(read_json(path, {}))
  except (ValueError, TypeError):
    return {'version': 1, 'devices': {}}


class Decoder:
  """Decode complete SYN_REPORT frames; dropped/incomplete gestures never fire."""
  def __init__(self, profile='generic'):
    self.profile = profile
    self.x = self.y = 0
    self.touch = False
    self.start = None
    self.last = None
    self.started = 0.0
    self.down = set()
    self.pending_keys = []
    self.dropped = False

  def feed(self, kind, code, value, now):
    if kind == 0 and code == 3:  # SYN_DROPPED: ignore until all buttons have been released
      self.start = self.last = None
      self.down.clear()
      self.pending_keys.clear()
      self.dropped = True
      return []
    if kind == 3 and code in (0, 1):
      if code == 0:
        self.x = value
      else:
        self.y = value
    elif kind == 1:
      if code == 330:
        self.touch = value != 0
      elif code < 256 or code >= 352:
        if value == 1:
          self.down.add(code)
        elif value == 0 and code in self.down:
          self.down.remove(code)
          self.pending_keys.append(code)
    if kind != 0 or code != 0:
      return []
    if self.dropped:
      self.pending_keys.clear()
      if not self.touch and not self.down:
        self.dropped = False
      return []
    tokens = [('1' if self.profile == 'yiser-j6' and key == 115 else f'key:{key}') for key in self.pending_keys]
    self.pending_keys.clear()
    if self.touch:
      if self.start is None:
        self.start = (self.x, self.y)
        self.started = now
      self.last = (self.x, self.y)
    elif self.start is not None:
      start, end = self.start, self.last
      self.start = self.last = None
      if 0 <= now - self.started <= 1.5:
        dx, dy = end[0] - start[0], end[1] - start[1]
        if max(abs(dx), abs(dy)) >= 120:
          axis, delta = ('x', dx) if abs(dx) > abs(dy) else ('y', dy)
          token = f'swipe:{axis}{"+" if delta > 0 else "-"}'
          if self.profile == 'yiser-j6':
            token = {'swipe:y+': 'up', 'swipe:y-': 'down', 'swipe:x+': 'left', 'swipe:x-': 'right'}[token]
          tokens.append(token)
        elif self.profile == 'yiser-j6':
          if abs(end[0] - 300) <= 65 and abs(end[1] - 500) <= 65:
            tokens.append('center')
          elif abs(end[0] - 420) <= 65 and abs(end[1] - 850) <= 65:
            tokens.append('2')
        else:
          tokens.append(f'tap:{round(end[0] / 25) * 25}:{round(end[1] / 25) * 25}')
    return tokens


class CommandReader:
  """Read at most once, reject startup leftovers and expired commands."""
  def __init__(self, channel, root=RUNTIME):
    self.path = Path(root) / f'{channel}.json'
    self.started = time.monotonic()
    self.last_id = None
    self.last_check = 0.0

  def read(self, allowed=True, now=None):
    now = time.monotonic() if now is None else now
    if now - self.last_check < 0.02:
      return None
    self.last_check = now
    message = read_json(self.path, {})
    if not isinstance(message, dict) or not message.get('id') or message['id'] == self.last_id:
      return None
    self.last_id = message['id']
    created = message.get('time', 0)
    if not isinstance(created, (int, float)) or not self.started <= created <= now or now - created > 0.4:
      return None
    learning = read_json(self.path.parent / 'learn.json', {})
    if isinstance(learning, dict) and learning.get('address') == message.get('address') and learning.get('until', 0) > now:
      return None
    return message.get('action') if allowed and message.get('action') in ACTIONS else None
