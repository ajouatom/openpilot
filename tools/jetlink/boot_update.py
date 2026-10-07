"""USB-only boot gate: verify the comma's release before starting model or HUD.

Runs as the existing root update service. No model/backend/renderer imports.
Network provisioning remains in the independent Wi-Fi service.
"""
import json
from pathlib import Path
import sys
import subprocess
import threading
import time
import urllib.error

import update_host as update

CAPABILITY = 'carrot_boot_update_v1'
MESSAGE = 0x4003
READY = Path('/run/carrot-jetlink-boot-ready.json')
STATUS = Path('/run/carrot-jetlink-boot-update.json')
BOOT_ID = Path('/proc/sys/kernel/random/boot_id')


def enabled():
  marker = update.ROOT / 'boot-update-required'
  if not marker.is_file():
    return False
  # Installing the new policy on a live device must not interrupt this boot.
  return marker.read_text().strip() != BOOT_ID.read_text().strip()


def runtime_ready():
  try:
    value = json.loads(READY.read_text())
    return (value['boot_id'] == BOOT_ID.read_text().strip() and
            value['source_commit'] == (update.ROOT / 'current/SOURCE_COMMIT').read_text().strip())
  except (OSError, ValueError, KeyError, TypeError):
    return False


def wait_for_runtime():
  while enabled() and not runtime_ready():
    time.sleep(.5)


def receive_wifi(payload):
  import pwd
  from wifi_protocol import receive
  owner = pwd.getpwnam('jetlink')
  # Retain the normal server's ability to replace these files after boot.
  # Root-owned files in sticky /dev/shm would block its later provisioning.
  receive(payload, owner=(owner.pw_uid, owner.pw_gid))


class Gate:
  def __init__(self):
    self.state = 'checking'
    self.fraction = None
    self.selected = None
    self.received = 0.
    self.next_try = 0.
    self.worker = None
    self.done = False
    self.error = ''

  def report(self, state, fraction=None):
    self.state, self.fraction = state, fraction

  def telemetry(self):
    return {'state': self.state, 'fraction': self.fraction, 'error': self.error}

  def select(self, manifest):
    # Validate before trusting equality, downloading or granting permission.
    update.validate_manifest(manifest)
    update.verify_signature(manifest)
    update.validate_storage_compatibility(manifest)
    if update.PROTECTED_MARKER.exists():
      if json.loads(Path('/run/carrot-storage.json').read_text()).get('state') != 'protected':
        raise ValueError('Protected DATA storage unavailable')
    if manifest != self.selected:
      self.next_try = 0.
    self.selected = manifest
    self.received = time.monotonic()

  def _install(self, manifest):
    try:
      self.report('downloading', 0.)
      update.stage_manifest(manifest, progress=lambda fraction: self.report('downloading', fraction))
      self.report('verifying')
      # These services are still in wait_for_runtime(), but stop them before
      # swapping files so their next process imports the complete new release.
      subprocess.run(['systemctl', 'stop', 'carrot-jetlink', 'carrot-jetlink-hud'], check=True, timeout=45)
      update.activate()
      if (update.ROOT / 'current/SOURCE_COMMIT').read_text().strip() != manifest['source_commit']:
        raise RuntimeError('Candidate validation failed; previous release retained')
      self.report('checking')
      self.error = ''
    except (urllib.error.URLError, TimeoutError, ConnectionError):
      self.report('waiting_internet')
      self.error = 'Download unavailable; retrying'
    except Exception as exc:
      self.report('failed')
      self.error = str(exc)[:160]
    finally:
      self.next_try = time.monotonic() + 30

  def tick(self):
    if self.worker is not None and self.worker.is_alive():
      return
    if self.selected is None or not 0 <= time.monotonic() - self.received < 5:
      if self.state != 'failed':
        self.report('checking')
      return
    manifest = self.selected
    if (update.ROOT / 'current/SOURCE_COMMIT').read_text().strip() == manifest['source_commit']:
      update.atomic_json(READY, {'boot_id': BOOT_ID.read_text().strip(), 'source_commit': manifest['source_commit']})
      READY.chmod(0o644)  # The unprivileged model/HUD guards must read this.
      self.report('ready')
      self.done = True
    elif time.monotonic() >= self.next_try:
      self.worker = threading.Thread(target=self._install, args=(manifest,), daemon=False)
      self.worker.start()


class BootstrapSession:
  def __init__(self, transport, gate):
    self.transport, self.gate = transport, gate
    self.last_seq = -1

  def handle(self, msg):
    from jetlink import protocol as p
    from wifi_protocol import CAPABILITY as wifi_capability, MESSAGE as wifi_message
    if msg.msg_type == p.Msg.HELLO_REQ:
      self.last_seq = msg.seq
      self.transport.send_json(p.Msg.HELLO_RESP, msg.seq, {
        'protocol': p.VERSION, 'carrot_host': 'jetson', CAPABILITY: True,
        wifi_capability: True, 'engine_state': 'none', 'loaded': None, 'sleep_after': 0,
      })
      return
    if msg.seq <= self.last_seq:
      return
    self.last_seq = msg.seq
    if msg.msg_type == wifi_message:
      receive_wifi(msg.payload)
    elif msg.msg_type == MESSAGE:
      try:
        if not 0 < len(msg.payload) <= 65536:
          raise ValueError('Release manifest size invalid')
        self.gate.select(json.loads(bytes(msg.payload)))
      except Exception:
        # Do not report incoming data: provisioning credentials must never leak.
        self.gate.selected = None
        self.gate.report('failed')
        self.gate.error = 'Selected release verification failed'
    elif msg.msg_type == p.Msg.STATE_REQ:
      self.transport.send_json(p.Msg.STATE_RESP, msg.seq, {'carrot_update': self.gate.telemetry()})
    elif msg.msg_type == p.Msg.PING:
      self.transport.send(p.Msg.PONG, msg.seq)
    else:
      # This process cannot create an engine or accept inference/upload requests.
      self.transport.send_json(p.Msg.ERROR, msg.seq, {'error': 'update_required', 'detail': 'Boot release check pending'})


def boot():
  if runtime_ready():
    subprocess.run(['systemctl', 'start', 'carrot-jetlink', 'carrot-jetlink-hud'], check=True, timeout=45)
    return
  # Recover an interrupted transaction before making any release comparison.
  # An unrelated pending download is not activated before the current pin arrives.
  if (update.ROOT / 'updates/transaction.json').exists():
    subprocess.run(['systemctl', 'stop', 'carrot-jetlink', 'carrot-jetlink-hud'], check=True, timeout=45)
    update.activate()
  READY.unlink(missing_ok=True)
  sys.path.insert(0, str(update.ROOT / 'current/third_party/jetlink'))
  from jetlink.transport.usbbulk import UsbBulkTransport
  from jetlink.transport.base import LinkTimeout
  gate = Gate()
  transport = session = None
  while not gate.done:
    try:
      if transport is None:
        transport = UsbBulkTransport.open(timeout_ms=1000)
        session = BootstrapSession(transport, gate)
      try:
        session.handle(transport.recv(timeout=.25))
      except LinkTimeout:
        pass
      gate.tick()
      update.atomic_json(STATUS, dict(gate.telemetry(), updated=time.monotonic()))
    except Exception:
      if transport is not None:
        transport.close()
      transport = session = None
      # A disconnected USB link cannot approve starting on a remembered pin.
      gate.received = 0.
      time.sleep(1)
  if transport is not None:
    transport.close()
  subprocess.run(['systemctl', 'start', 'carrot-jetlink', 'carrot-jetlink-hud'], check=True, timeout=45)
