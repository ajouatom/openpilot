"""Opt-in physical indicator channel, separate from HUD and lane-change requests."""
import json
import math
from pathlib import Path
import secrets
import socket
import subprocess
import sys
import time

CAPABILITY = 'carrot_blinker_control_v2'
BEGIN, ACK, DATA = 0x4030, 0x4031, 0x4032
LIMIT = 2048
CONFIG = Path.home() / '.config/carrot-jetlink/blinker.json'
PROFILE = 'hyundai-b1-3c1'
PACKAGE_CONFIG = Path(__file__).with_name('blinker_profile.json')


def configuration(path=CONFIG):
  try:
    selected = PACKAGE_CONFIG if path == CONFIG and not path.exists() else path
    value = json.loads(selected.read_text())
    if value.get('interface') == 'auto':
      candidates = []
      for entry in Path('/sys/class/net').glob('can*'):
        usb = (entry/'device').resolve().parent
        try:
          if ((entry/'device/driver').resolve().name == 'peak_usb' and
              (usb/'idVendor').read_text().strip() == '0c72' and
              (usb/'idProduct').read_text().strip() == '000c'):
            candidates.append(entry.name)
        except OSError:
          continue
      if len(candidates) != 1:
        return None
      value['interface'] = candidates[0]
    if (value.get('enabled') is True and value.get('profile') == PROFILE and
        isinstance(value.get('interface'), str) and value['interface'].startswith('can') and
        value['interface'][3:].isdigit()):
      return value
  except (OSError, ValueError, AttributeError):
    pass
  return None


def finite(value):
  return type(value) in (int, float) and math.isfinite(value)


def fresh(stamp, now, ttl=.4):
  return finite(stamp) and 0 <= now-stamp < ttl


def encode(value):
  data = json.dumps(value, separators=(',', ':'), allow_nan=False).encode()
  if len(data) > LIMIT:
    raise ValueError('blinker packet exceeds limit')
  return data


class Process:
  def __init__(self, script, *args):
    self.socket, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    self.socket.setblocking(False)
    try:
      self.process = subprocess.Popen(['chrt', '--other', '0', sys.executable, str(Path(__file__).with_name(script)), str(child.fileno()), *args],
                                      pass_fds=(child.fileno(),), stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    except BaseException:
      self.socket.close()
      raise
    finally:
      child.close()

  def send(self, value):
    try:
      return self.socket.send(encode(value)) > 0
    except (OSError, ValueError):
      return False

  def latest(self):
    value = None
    for _ in range(64):
      try:
        raw = self.socket.recv(LIMIT+1)
        if not raw:
          break
        if len(raw) <= LIMIT:
          value = json.loads(raw)
      except (OSError, ValueError):
        break
    return value

  def close(self):
    self.socket.close()  # EOF revokes the worker's guard and CAN lease.
    try:
      self.process.wait(timeout=2)
    except subprocess.TimeoutExpired:
      self.process.terminate()
      try:
        self.process.wait(timeout=2)
      except subprocess.TimeoutExpired:
        self.process.kill()
        self.process.wait(timeout=2)


class Host:
  def __init__(self):
    self.worker = None
    self.token = secrets.token_hex(16)
    self.status = {'ready': False}

  @staticmethod
  def supported():
    return sys.platform == 'linux' and Path('/etc/nv_tegra_release').is_file() and configuration() is not None

  def begin(self):
    if not self.supported():
      return {'ready': False}
    if self.worker is None:
      self.worker = Process('blink_can.py', self.token)
    return {'ready': True, 'token': self.token, 'clock': time.monotonic()}

  def send(self, payload):
    if self.worker is None or len(payload) > LIMIT:
      return
    try:
      value = json.loads(bytes(payload))
      if isinstance(value, dict) and value.get('token') == self.token:
        self.worker.send(value)
    except (ValueError, TypeError):
      pass

  def telemetry(self):
    if self.worker is not None:
      status = self.worker.latest()
      if isinstance(status, dict):
        self.status = status
      if self.worker.process.poll() is not None or not fresh(self.status.get('updated'), time.monotonic(), .5):
        self.status = {**self.status, 'ready': False, 'state': 'error' if self.status.get('error') else 'stopped'}
    return {**self.status, 'token': self.token, 'clock': time.monotonic()}

  def close(self):
    if self.worker is not None:
      self.worker.close()
      self.worker = None


def wire_record(value, token, offset, now):
  """Receiver-clock deadlines use a conservative HELLO/telemetry clock offset."""
  if not isinstance(value, dict) or not fresh(value.get('stamp'), now, .2):
    return None
  events = []
  for event in value.get('events', [])[:4]:
    if (isinstance(event, dict) and isinstance(event.get('id'), str) and len(event['id']) <= 100 and
        event.get('direction') in ('left', 'right') and fresh(event.get('time'), now)):
      events.append({'id': event['id'], 'direction': event['direction'], 'deadline': event['time']+offset+.4,
                     'device':event.get('device')})
  hold = value.get('hold')
  if (isinstance(hold, dict) and isinstance(hold.get('id'), str) and len(hold['id']) <= 100 and
      hold.get('direction') in ('left', 'right') and hold.get('source') in ('bluetooth', 'turn', 'lane') and
      finite(hold.get('time')) and 0 <= now-hold['time'] < 300):
    hold = {**hold, 'deadline':hold['time']+offset+.4}
    hold.pop('time')

  else:
    hold = None
  return {'token': token, 'ok': value.get('ok') is True,
          'guard_deadline': value['stamp']+offset+.35, 'events': events, 'hold':hold,
          'devices':[d for d in value.get('devices', [])[:16] if isinstance(d,str) and len(d)<=32]}


class Sender:
  def __init__(self, client, peer):
    self.worker = None
    self.next_send = 0.
    self.anchor = None
    self.report = None
    self.last_report = None
    self.report_after = 0.
    if peer.get('carrot_host') != 'jetson' or peer.get(CAPABILITY) is not True:
      return
    if Path('/TICI').is_file():
      from openpilot.common.params import Params
      if Params().get_bool('CanfdBlinkerControl'):
        return  # Reboot selection excludes USB2CAN before host BEGIN.
    seq = client._next_seq()
    client.t.send_json(BEGIN, seq, {'version': 2})
    response = json.loads(bytes(client._expect(ACK, seq, 1.).payload))
    if response.get('ready') is not True or not finite(response.get('clock')):
      return
    self.token = response['token']
    self.clock_received = time.monotonic()
    self.offset = response['clock'] - self.clock_received
    self.worker = Process('blink_source.py')
    if Path('/TICI').is_file():
      from openpilot.common.swaglog import cloudlog
      self.report = cloudlog.event

  def send(self, client):
    now = time.monotonic()
    if self.worker is None or now < self.next_send:
      return
    self.next_send = now + .05
    # The host stamp is taken before we receive telemetry: this estimate can
    # shorten a deadline, never extend it by transport or inference latency.
    state = (client.last_state or {}).get('carrot_blinker', {})
    if state.get('token') == self.token and finite(state.get('clock')) and state['clock'] != self.anchor:
      self.anchor = state['clock']
      self.offset = self.anchor - now
      self.clock_received = now
    if now-self.clock_received > 3:
      return  # Do not extrapolate a clock anchor through a stalled session.
    self.worker.send({'stamp':now, 'accepted_id':state.get('accepted_id'),
                      'state':state.get('state'), 'token':self.token})
    source = self.worker.latest()
    if self.report is not None and isinstance(source, dict) and now >= self.report_after:
      hold = source.get('hold') or {}
      detail = {'source':hold.get('source'), 'direction':hold.get('direction'),
                'activated':source.get('activated') is True,
                'peer_state':str(state.get('state', 'missing'))[:32],
                'accepted':bool(hold.get('id') and state.get('accepted_id') == hold['id']),
                'peer_owned':state.get('token') == self.token,
                'relights':source.get('relights', 0)}
      if detail != self.last_report:
        self.report('physicalBlinkerState', **detail)
        self.last_report, self.report_after = detail, now+.2
    value = wire_record(source, self.token, self.offset, now)
    if value is not None:
      client.t.send(DATA, client._next_seq(), [encode(value)])

  def close(self):
    if self.worker is not None:
      self.worker.close()
