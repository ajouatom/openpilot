"""Validated mapping storage, HID decoding and short-lived local commands."""
from collections import deque
import json
import os
from pathlib import Path
import re
import time
import uuid

CONFIG_PATH = Path('/data/carrot/bluetooth.json')
RUNTIME = Path('/dev/shm/carrot-bluetooth')
REMOTE_BUTTONS = ('accelCruise', 'decelCruise', 'gapAdjustCruise', 'lfaButton', 'cancel')
ACTIONS = ('none', *REMOTE_BUTTONS, *(button + 'Long' for button in REMOTE_BUTTONS),
           'laneLeft', 'laneRight', 'paddleDecel', 'carrotCruise')
DOUBLE_SECONDS = 0.35
LONG_SECONDS = 0.7
REPEAT_SECONDS = 0.5
MAX_HOLD_SECONDS = 10
REPEAT_ACTIONS = ('accelCruise', 'decelCruise', 'accelCruiseLong', 'decelCruiseLong')
COMMAND_TTL = 0.4
BLUETOOTH_CANCEL = -3  # Explicit driver cancel, including PCM cruise; -1/-2 retain their existing meanings.
DEFAULT_MAPPING = {'up': 'accelCruise', 'down': 'decelCruise', 'left': 'laneLeft', 'right': 'laneRight',
                   'center': 'paddleDecel', '1': 'gapAdjustCruise', '2': 'none'}
MAC = re.compile(r'^(?:[0-9A-F]{2}:){5}[0-9A-F]{2}$')
TOKEN = re.compile(r'^(?:up|down|left|right|center|1|2|key:[0-9]{1,4}|swipe:[xy][+-]|tap:[0-9]{1,5}:[0-9]{1,5})(?:@double|@long)?$')


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
    if not isinstance(mapping, dict) or len(mapping) > 192 or len({k.split('@')[0] for k in mapping}) > 64:
      raise ValueError('invalid mapping')
    for key, action in mapping.items():
      if not TOKEN.fullmatch(key) or action not in ACTIONS:
        raise ValueError('invalid button or action')
      if key.startswith('key:') and not 0 <= int(key.split('@')[0][4:]) <= 767:
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


class Clicks:
  """Release-triggered gestures. Single clicks wait only when double is assigned."""
  def __init__(self, mapping=None, learning=False):
    self.mapping = mapping or {}
    self.learning = learning
    self.pending = {}

  def assigned(self, token, gesture):
    return self.learning or self.mapping.get(token + '@' + gesture, 'none') != 'none'

  def flush(self, now):
    result = []
    for token, released in list(self.pending.items()):
      if now - released >= DOUBLE_SECONDS:
        del self.pending[token]
        if now - released <= COMMAND_TTL:
          result.append(token)
    return result

  def release(self, token, duration, now):
    result = self.flush(now)
    if not 0 <= duration <= 10:
      return result
    if duration >= LONG_SECONDS and self.assigned(token, 'long'):
      # A preceding short click is separate from this long press.
      if token in self.pending:
        del self.pending[token]
        result.append(token)
      return result + [token + '@long']
    if self.assigned(token, 'double'):
      if token in self.pending:
        del self.pending[token]
        return result + [token + '@double']
      self.pending[token] = now
      return result
    return result + [token]


class Decoder:
  """Decode complete SYN_REPORT frames; dropped/incomplete gestures never fire."""
  def __init__(self, profile='generic', mapping=None, learning=False):
    self.profile = profile
    self.x = self.y = 0
    self.touch = False
    self.start = None
    self.last = None
    self.started = 0.0
    self.down = {}
    self.pending_keys = []
    self.dropped = False
    self.clicks = Clicks(mapping, learning)
    self.holds = {}
    self.frame_open = False
    self.repeated = set()

  @property
  def active_longs(self):
    return {hold['token'] + '@long' for hold in self.holds.values() if hold['fired'] and not hold['expired']}

  def cancel_holds(self):
    for hold in self.holds.values():
      hold['expired'] = True

  def hold(self, source, token, started):
    previous = self.holds.get(source)
    if previous and previous['token'] != token:
      if previous['fired'] or previous['expired']:
        previous['expired'] = True  # Changing direction cannot resume a cancelled or already fired hold.
        return
      self.holds.pop(source, None)
      previous = None
    if not previous and token:
      self.holds[source] = {'token': token, 'started': started, 'last': None, 'fired': False, 'expired': False}

  def key_token(self, key):
    return '1' if self.profile == 'yiser-j6' and key == 115 else f'key:{key}'

  def touch_token(self):
    dx, dy = self.last[0] - self.start[0], self.last[1] - self.start[1]
    if max(abs(dx), abs(dy)) >= 120:
      axis, delta = ('x', dx) if abs(dx) > abs(dy) else ('y', dy)
      token = f'swipe:{axis}{"+" if delta > 0 else "-"}'
      return {'swipe:y+': 'up', 'swipe:y-': 'down', 'swipe:x+': 'left', 'swipe:x-': 'right'}[token] if self.profile == 'yiser-j6' else token
    if self.profile == 'yiser-j6':
      if abs(self.last[0] - 300) <= 65 and abs(self.last[1] - 500) <= 65:
        return 'center'
      if abs(self.last[0] - 420) <= 65 and abs(self.last[1] - 850) <= 65:
        return '2'
      return None
    return f'tap:{round(self.last[0] / 25) * 25}:{round(self.last[1] / 25) * 25}'

  def flush(self, now):
    self.repeated.clear()
    if self.dropped or self.frame_open:
      return []
    result = self.clicks.flush(now)
    for source, hold in self.holds.items():
      duration = now - hold['started']
      age = now - self.started if source == 'touch' else duration
      if not 0 <= age <= MAX_HOLD_SECONDS:
        hold['expired'] = True
      token = hold['token']
      if hold['expired'] or duration < LONG_SECONDS or not self.clicks.assigned(token, 'long'):
        continue
      long_token = token + '@long'
      repeat = self.clicks.mapping.get(long_token) in REPEAT_ACTIONS
      if hold['fired'] and (not repeat or now - hold['last'] < REPEAT_SECONDS):
        continue
      if hold['fired']:
        self.repeated.add(long_token)
      hold['fired'] = True
      hold['last'] = now  # Never catch up missed ticks after a scheduler stall.
      result.append(long_token)
    return result

  def feed(self, kind, code, value, now):
    self.repeated.clear()
    self.frame_open = not (kind == 0 and code == 0)
    if kind == 0 and code == 3:  # SYN_DROPPED: ignore until all buttons have been released
      self.start = self.last = None
      self.down.clear()
      self.pending_keys.clear()
      self.clicks.pending.clear()
      self.holds.clear()
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
          self.down.setdefault(code, now)
        elif value == 0 and code in self.down:
          self.pending_keys.append((code, now - self.down.pop(code)))
    if kind != 0 or code != 0:
      return []
    if self.dropped:
      self.pending_keys.clear()
      if not self.touch and not self.down:
        self.dropped = False
      return []
    releases = []
    for key, duration in self.pending_keys:
      hold = self.holds.pop(key, {})
      if not hold.get('fired') and not hold.get('expired'):
        releases.append((self.key_token(key), duration))
    self.pending_keys.clear()
    for key, started in self.down.items():
      self.hold(key, self.key_token(key), started)
    if self.touch:
      if self.start is None:
        self.start = (self.x, self.y)
        self.started = now
      self.last = (self.x, self.y)
      self.hold('touch', self.touch_token(), now)
    elif self.start is not None:
      token = self.touch_token()
      hold = self.holds.pop('touch', {})
      self.start = self.last = None
      duration = now - self.started
      if token and not hold.get('fired') and not hold.get('expired') and 0 <= duration <= MAX_HOLD_SECONDS:
        if duration <= 1.5 or self.clicks.assigned(token, 'long'):
          releases.append((token, duration))
    tokens = []
    for token, duration in releases:
      tokens.extend(self.clicks.release(token, duration, now))
    return tokens


class CommandWriter:
  """One writer, bounded per-channel event journals; devices never overwrite peers."""
  def __init__(self, root=RUNTIME):
    self.root = root
    self.events = {'cruise': [], 'lane': []}
    self.session = uuid.uuid4().hex
    self.sequence = 0

  def publish(self, channel):
    atomic_json(self.root / f'{channel}.json', {'events': self.events[channel]})

  def prune(self, addresses, now, active_holds=None):
    for channel, events in self.events.items():
      kept = [e for e in events if e['address'] in addresses and 0 <= now - e['time'] <= COMMAND_TTL and
              (not e.get('hold') or active_holds is None or e['hold'] in active_holds)]
      if kept != events:
        self.events[channel] = kept
        self.publish(channel)

  def send(self, mac, action, now, hold=None, repeat=False):
    channel = 'lane' if action in ('laneLeft', 'laneRight') else 'cruise'
    self.sequence += 1
    events = [e for e in self.events[channel] if 0 <= now - e['time'] <= COMMAND_TTL and (not hold or e.get('hold') != hold)]
    events.append({'id': f'{self.session}:{self.sequence}', 'time': now, 'action': action, 'address': mac, 'hold': hold, 'repeat': repeat})
    self.events[channel] = events[-64:]
    self.publish(channel)


class CommandReader:
  """Read at most once, reject startup leftovers and expired commands."""
  def __init__(self, channel, root=RUNTIME):
    self.path = Path(root) / f'{channel}.json'
    self.started = time.monotonic()
    self.last_id = None
    self.seen = deque(maxlen=128)
    self.last_check = 0.0
    self.is_repeat = False

  def read(self, allowed=True, now=None):
    self.is_repeat = False
    now = time.monotonic() if now is None else now
    if now - self.last_check < 0.02:
      return None
    self.last_check = now
    payload = read_json(self.path, {})
    if not isinstance(payload, dict):
      return None
    messages = payload.get('events', [payload])
    if not isinstance(messages, list):
      return None
    learning = read_json(self.path.parent / 'learn.json', {})
    cancelled = read_json(self.path.parent / 'cancelled.json', {})
    for message in messages[-64:]:
      if not isinstance(message, dict) or not isinstance(message.get('id'), str) or message['id'] in self.seen:
        continue
      self.seen.append(message['id'])
      self.last_id = message['id']
      created = message.get('time', 0)
      if not isinstance(created, (int, float)) or not self.started <= created <= now or now - created > COMMAND_TTL:
        continue
      cutoff = cancelled.get(message.get('address'), 0) if isinstance(cancelled, dict) else 0
      if isinstance(cutoff, (int, float)) and created <= cutoff:
        continue
      if isinstance(learning, dict) and learning.get('address') == message.get('address'):
        until = learning.get('until', 0)
        if isinstance(until, (int, float)) and until > now:
          continue
      if allowed and message.get('action') in ACTIONS:
        self.is_repeat = bool(message.get('repeat', False))
        return message['action']
    return None
