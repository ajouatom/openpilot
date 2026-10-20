"""Branch-owned comma AP, independent of the optional phone Internet uplink.

This low-priority control worker never touches wlan0 or starts an AP onroad.
Its secret stays in a root-only file and the existing private USB provisioning.
Returning to a branch without this helper tears down only its own AP.
"""
import argparse
import fcntl
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import re
import secrets
import signal
import subprocess
import threading
import time

from persistent_state import durable_write
from wifi_direct_netlink import INTERFACE, Wireless, profile_id
from wifi_direct_forward import Sharing

STATUS = Path('/dev/shm/carrot-jetlink-direct-wifi.json')
PRIVATE = Path('/data/carrot-jetlink-direct-wifi')
PROFILE = Path('/run/NetworkManager/system-connections/carrot-jetson-direct.nmconnection')
SUBNETS = ('172.29.77.0/24', '172.29.78.0/24', '10.203.77.0/24')


def offroad():
  try:
    p = Path('/data/params/d')
    return (p/'IsOffroad').read_bytes() == b'1' and (p/'IsOnroad').read_bytes() == b'0'
  except OSError:
    return False


def nm(*args):
  value = subprocess.run(['nmcli', '--escape', 'no', *args], capture_output=True, text=True, timeout=15,
                         env=dict(os.environ, LC_ALL='C'))
  if value.returncode:
    raise RuntimeError('Direct Wi-Fi operation failed')  # Never include NM output/secrets.
  return value.stdout.strip()


def choose_subnet(routes):
  occupied = [ipaddress.ip_network(r['dst'], strict=False) for r in routes
              if r.get('dst', 'default') != 'default' and r.get('dev') != INTERFACE]
  return next(s for s in SUBNETS if not any(ipaddress.ip_network(s).overlaps(r) for r in occupied))


def channel():
  # Query the existing scan cache only; never trigger a scan or move wlan0.
  rows = nm('-t', '-f', 'IN-USE,FREQ', 'device', 'wifi', 'list', 'ifname', 'wlan0', '--rescan', 'no').splitlines()
  freq = next((int(r.split(':')[1]) for r in rows if r.startswith('*:') and r.split(':')[1].isdigit()), 5180)
  if freq in range(2412, 2473, 5):
    return 'bg', (freq - 2407) // 5
  number = (freq - 5000) // 5
  return ('a', number) if number in (36, 40, 44, 48, 149, 153, 157, 161) else ('a', 36)


def configuration():
  PRIVATE.mkdir(mode=0o700, parents=True, exist_ok=True)
  if PRIVATE.is_symlink():
    raise ValueError('Unexpected direct Wi-Fi directory')
  os.chmod(PRIVATE, 0o700)
  path = PRIVATE/'identity.json'
  if path.is_symlink():
    raise ValueError('Unexpected direct Wi-Fi identity')
  if path.exists():
    value = json.loads(path.read_text())
    if (value.get('uuid') != profile_id() or value.get('subnet') not in SUBNETS
        or not isinstance(value.get('ssid'), str) or not re.fullmatch('Carrot-Jetson-[0-9a-f]{8}', value['ssid'])
        or not isinstance(value.get('password'), str)
        or len(value['password']) != 32 or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_' for c in value['password'])):
      raise ValueError('Invalid direct Wi-Fi identity')
    return value
  routes = json.loads(subprocess.check_output(['ip', '-j', '-4', 'route', 'show'], text=True, timeout=3))
  value = {'uuid': profile_id(), 'subnet': choose_subnet(routes),
           'ssid': 'Carrot-Jetson-' + hashlib.sha256(Path('/etc/machine-id').read_bytes()).hexdigest()[:8],
           'password': secrets.token_urlsafe(24)}
  durable_write(path, json.dumps(value).encode(), 0o600)
  return value


def render(value, band, number):
  gateway = str(ipaddress.ip_network(value['subnet']).network_address + 1)
  return ('[connection]\nid=Carrot Jetson direct\nuuid=' + value['uuid'] + '\ntype=wifi\ninterface-name=' + INTERFACE + '\nautoconnect=false\n'
          '[wifi]\nmode=ap\nssid=' + value['ssid'] + '\nband=' + band + '\nchannel=' + str(number) + '\n'
          '[wifi-security]\nkey-mgmt=wpa-psk\nproto=rsn\npairwise=ccmp\ngroup=ccmp\npsk=' + value['password'] + '\n'
          '[ipv4]\nmethod=shared\naddress1=' + gateway + '/24\nnever-default=true\n'
          '[ipv6]\nmethod=disabled\n')


class AccessPoint:
  def __init__(self, wireless, command=nm):
    self.wireless, self.command = wireless, command
    self.index = self.value = None
    self.previous_power_save = None
    self.stage = 'idle'
    self.sharing = Sharing()
    self.sharing_state = 'idle'

  def start(self):
    if not offroad():
      return False
    self.stage = 'identity'
    value = configuration()
    routes = json.loads(subprocess.check_output(['ip', '-j', '-4', 'route', 'show'], text=True, timeout=3))
    if any(r.get('dev') != INTERFACE and r.get('dst', 'default') != 'default'
           and ipaddress.ip_network(value['subnet']).overlaps(ipaddress.ip_network(r['dst'], strict=False)) for r in routes):
      raise ValueError('Direct Wi-Fi subnet conflict')
    self.stage = 'channel'
    band, number = channel()
    self.stage = 'interface'
    self.index = self.wireless.create()
    self.value = value
    self.stage = 'register_device'
    until = time.monotonic() + 3.
    while True:
      try:
        if self.command('-g', 'GENERAL.STATE', 'device', 'show', INTERFACE):
          break
      except RuntimeError:
        pass
      if time.monotonic() >= until:
        raise RuntimeError('Direct Wi-Fi device registration timeout')
      time.sleep(.2)
    self.command('device', 'set', INTERFACE, 'autoconnect', 'no', 'managed', 'yes')
    self.stage = 'profile'
    PROFILE.parent.mkdir(parents=True, exist_ok=True)
    durable_write(PROFILE, render(value, band, number).encode(), 0o600)
    self.command('connection', 'load', str(PROFILE))
    self.stage = 'activate'
    self.command('--wait', '12', 'connection', 'up', 'uuid', value['uuid'], 'ifname', INTERFACE)
    self.stage = 'power_save'
    self.previous_power_save = self.wireless.power_save(self.index)
    self.awake()
    self.share_internet()
    self.stage = 'active'
    return True

  def share_internet(self):
    if self.value is None or not offroad():
      return
    try:
      self.sharing.ensure(self.value['subnet'])
      self.sharing_state = 'configured'
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError):
      # Keep the local display connected when Internet sharing is unavailable.
      # Retry offroad; this status describes rules, not Internet reachability.
      self.sharing_state = 'unavailable'

  def awake(self):
    # Only the AP we created. Never change wlan0's Internet profile or power
    # policy. This driver otherwise stalls AP data after its station disconnects.
    if not any(r['name'] == INTERFACE and r['index'] == self.index for r in self.wireless.interfaces()):
      raise ValueError('Direct AP ownership lost')
    if self.index is not None and self.wireless.power_save(self.index) != 0:
      self.wireless.set_power_save(self.index, 0)

  def active(self):
    return self.value is not None and self.value['uuid'] in self.command('-t', '-f', 'UUID', 'connection', 'show', '--active').splitlines()

  def close(self):
    try:
      self.sharing.close()
      self.sharing_state = 'idle'
    except (OSError, RuntimeError, subprocess.SubprocessError):
      self.sharing_state = 'cleanup_failed'
    previous, self.previous_power_save = self.previous_power_save, None
    if self.index is not None and previous is not None:
      try:
        if any(r['name'] == INTERFACE and r['index'] == self.index for r in self.wireless.interfaces()):
          self.wireless.set_power_save(self.index, previous)
      except OSError:
        pass  # Still tear down our AP if the driver disappeared during close.
    try:
      if self.value is not None:
        self.command('--wait', '3', 'connection', 'down', 'uuid', self.value['uuid'])
    except (RuntimeError, subprocess.SubprocessError):
      pass
    finally:
      if self.index is not None:
        self.wireless.delete(self.index)
        self.index = None
      if self.value is not None:
        try:
          self.command('connection', 'delete', 'uuid', self.value['uuid'])
        except (RuntimeError, subprocess.SubprocessError):
          pass
      self.value = None


def ensure():
  """Nonblocking: the existing USB owner never waits for AP startup."""
  try:
    if time.monotonic() - json.loads(STATUS.read_text())['updated'] < 10:
      return
  except (OSError, ValueError, KeyError, TypeError):
    pass
  subprocess.Popen(['sudo', '-n', '/usr/bin/python3', str(Path(__file__).resolve()), '--serve'],
                   stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)


def low_priority():
  os.setpriority(os.PRIO_PROCESS, 0, 19)
  os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
  # A USB worker can inherit a big-core-only affinity. Move this independent
  # root control worker to little CPUs rather than intersecting that mask.
  os.sched_setaffinity(0, set(range(min(4, os.cpu_count() or 1))))


def serve(trial=0):
  if os.geteuid() != 0:
    raise PermissionError('Direct Wi-Fi requires its root control worker')
  lock = open('/run/carrot-jetlink-direct-wifi.lock', 'a')
  try:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
  except BlockingIOError:
    lock.close()
    return
  low_priority()
  stop = threading.Event()
  for sig in (signal.SIGTERM, signal.SIGINT):
    signal.signal(sig, lambda *_: stop.set())
  active_source = Path('/data/openpilot/tools/jetlink/wifi_direct.py')
  digest = hashlib.sha256(Path(__file__).read_bytes()).digest()
  wireless, ap = None, None
  expires, retry, state = time.monotonic() + trial if trial else float('inf'), 0., 'starting'
  try:
    wireless = Wireless()
    ap = AccessPoint(wireless)
    while not stop.is_set() and time.monotonic() < expires:
      if not trial and (not active_source.is_file() or hashlib.sha256(active_source.read_bytes()).digest() != digest):
        break  # Returning to the official branch must not leave a hidden AP.
      now = time.monotonic()
      if now >= retry:
        retry = now + 5.
        try:
          if not ap.active():
            if offroad():
              ap.close()
              state = 'active' if ap.start() else 'waiting_offroad'
            else:
              state = 'waiting_offroad'
          else:
            ap.awake()
            ap.share_internet()
            state = 'active'
        except (OSError, ValueError, RuntimeError, StopIteration, KeyError, subprocess.SubprocessError):
          ap.close()
          state, retry = 'unavailable', now + 30.
      # No SSID, password, peer identity or vehicle payload in diagnostics.
      durable_write(STATUS, json.dumps({'state': state, 'stage': ap.stage, 'interface': INTERFACE,
                                      'internet_sharing': ap.sharing_state,
                                      'power_save': 0 if ap.previous_power_save is not None else None,
                                      'updated': now}).encode(), 0o644)
      stop.wait(2.)
  finally:
    if ap is not None:
      ap.close()
    if wireless is not None:
      wireless.close()
    STATUS.unlink(missing_ok=True)
    lock.close()


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--serve', action='store_true')
  parser.add_argument('--trial', type=int, choices=range(1, 181), default=0)
  args = parser.parse_args()
  if not args.serve and not args.trial:
    parser.error('Expected --serve or a bounded --trial')
  serve(args.trial)
