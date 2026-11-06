"""Opt-in physical indicator channel, separate from HUD and lane-change requests."""
import json
import math
from pathlib import Path
import secrets
import socket
import subprocess
import sys
import threading
import time
try:
  from .blink_owner import Owner
except ImportError:
  from blink_owner import Owner

CAPABILITY = 'carrot_blinker_control_v2'
BEGIN, ACK, DATA = 0x4030, 0x4031, 0x4032
LIMIT = 2048
CONFIG = Path.home() / '.config/carrot-jetlink/blinker.json'
PROFILE = 'hyundai-b1-3c1'
PACKAGE_CONFIG = Path(__file__).with_name('blinker_profile.json')


def configuration(path=CONFIG, preferred=None, exclude=()):
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
      from blink_slcan import candidates as serial_candidates
      candidates.extend(serial_candidates())
      candidates = [c for c in candidates if c not in exclude]
      if not candidates:
        return None
      # Keep the discovered adapter until it disappears. Simultaneous startup
      # ties use discovery order; plugging in another adapter never preempts.
      value['interface'] = preferred if preferred in candidates else candidates[0]
    interface = value.get('interface')
    valid = isinstance(interface, str) and interface.startswith('can') and interface[3:].isdigit()
    if isinstance(interface, str) and interface.startswith('/dev/'):
      from blink_slcan import identity
      identity(interface)
      valid = True
    if value.get('enabled') is True and value.get('profile') == PROFILE and valid:
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

  def close(self, *, wait=True):
    self.socket.close()  # EOF revokes the worker's guard and CAN lease.
    if not wait:
      reaper = threading.Thread(target=self._wait, name='blinker-source-reaper', daemon=True)
      reaper.start()
      return reaper
    self._wait()

  def _wait(self):
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
    self.next_restart = 0.

  @staticmethod
  def supported():
    if sys.platform != 'linux' or not Path('/etc/nv_tegra_release').is_file():
      return False
    try:
      profile = json.loads((CONFIG if CONFIG.exists() else PACKAGE_CONFIG).read_text())
      return profile.get('enabled') is True and profile.get('profile') == PROFILE
    except (OSError, ValueError, AttributeError):
      return False

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
        self.status = {**self.status, 'ready': False, 'available':False, 'idle':False,
                       'state': 'error' if self.status.get('error') else 'stopped'}
      if self.worker.process.poll() is not None and time.monotonic() >= self.next_restart and self.supported():
        self.worker.close()
        self.worker = Process('blink_can.py', self.token)
        self.next_restart = time.monotonic()+1
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
    self.reaper = None
    self.next_send = 0.
    self.anchor = None
    self.report = None
    self.last_report = None
    self.report_after = 0.
    self.enabled = False
    self.owner = Owner('USB CAN')
    self.generation = None
    self.params = None
    self.last_drop = 0.
    if peer.get('carrot_host') != 'jetson' or peer.get(CAPABILITY) is not True:
      return
    if Path('/TICI').is_file():
      from openpilot.common.params import Params
      self.params = Params()
    seq = client._next_seq()
    client.t.send_json(BEGIN, seq, {'version': 2})
    response = json.loads(bytes(client._expect(ACK, seq, 1.).payload))
    if response.get('ready') is not True or not finite(response.get('clock')):
      return
    self.token = response['token']
    self.clock_received = time.monotonic()
    self.offset = response['clock'] - self.clock_received
    self.enabled = True  # Discovery stays active even while S3 owns output.
    if Path('/TICI').is_file():
      from openpilot.common.swaglog import cloudlog
      self.report = cloudlog.event

  def send(self, client):
    now = time.monotonic()
    if not self.enabled or now < self.next_send:
      return
    self.next_send = now + .05
    # Only the policy source is reaped asynchronously, never the CAN worker.
    # Do not create overlapping sources or unbounded cleanup threads on flaps.
    if self.reaper is not None:
      if self.reaper.is_alive():
        return
      self.reaper = None
    if self.worker is not None and self.worker.process.poll() is not None:
      self._report_drop('source_exit', now, exitcode=self.worker.process.returncode)
      self._stop_source()
      client.t.send(DATA, client._next_seq(), [encode({'token':self.token, 'ok':False, 'events':[]})])
      return
    # The host stamp is taken before we receive telemetry: this estimate can
    # shorten a deadline, never extend it by transport or inference latency.
    state = (client.last_state or {}).get('carrot_blinker', {})
    if state.get('token') == self.token and finite(state.get('clock')) and state['clock'] != self.anchor:
      self.anchor = state['clock']
      self.offset = self.anchor - now
      self.clock_received = now
    available = (state.get('token') == self.token and state.get('available') is True and
                 now-self.clock_received < .5)
    if self.params is not None:
      available = available and self.params.get_bool('IsOnroad') and not self.params.get_bool('CanfdBlinkerControl')
    generation = state.get('generation')
    if generation != self.generation:
      self.generation = generation
      if self.worker is not None:
        self._report_drop('adapter_changed', now)
        self._stop_source()
        client.t.send(DATA, client._next_seq(), [encode({'token':self.token, 'ok':False, 'events':[]})])
        return
      self._stop_source()
    self.owner.name = str(state.get('adapter', 'USB CAN'))[:64]
    owns = self.owner.acquire(available and (self.owner.held or state.get('idle') is True))
    if not owns:
      if self.worker is not None:
        self._report_drop('lease_lost', now, available=available, clock_age=now-self.clock_received,
                          receiver_available=state.get('available') is True,
                          receiver_state=str(state.get('state', 'missing'))[:32], receiver_error=str(state.get('error') or '')[:160])
        self._stop_source()
        client.t.send(DATA, client._next_seq(), [encode({'token':self.token, 'ok':False, 'events':[]})])
      return
    if self.worker is None:
      self.worker = Process('blink_source.py')
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
                'relights':source.get('relights', 0),
                'turn_phase':str(source.get('turn_phase', 'unknown'))[:32],
                'turn_position':source.get('turn_position') is True,
                'tbt_phase':str(source.get('tbt_phase', 'unknown'))[:32],
                'observation_phase':str(source.get('observation_phase', 'unknown'))[:32],
                'indicator_origin':str(source.get('indicator_origin', 'unknown'))[:32]}
      if detail != self.last_report:
        self.report('physicalBlinkerState', **detail)
        self.last_report, self.report_after = detail, now+.2
    value = wire_record(source, self.token, self.offset, now)
    if value is not None:
      client.t.send(DATA, client._next_seq(), [encode(value)])

  def _stop_source(self):
    if self.worker is not None:
      # This runs on the model bridge's inference thread. Close the command
      # channel immediately, but never wait here for Python worker shutdown.
      self.reaper = self.worker.close(wait=False)
      self.worker = None
    self.owner.release()

  def _report_drop(self, reason, now, **detail):
    if self.report is not None and now-self.last_drop >= 5:
      self.report('physicalBlinkerOwnershipLost', reason=reason, **detail)
      self.last_drop = now

  def close(self):
    self._stop_source()
    self.enabled = False
