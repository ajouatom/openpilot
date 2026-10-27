"""Read NetworkManager secrets only in a low-priority, private USB worker."""
import hashlib
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import sys
import time
import uuid

from wifi_protocol import HEADER, LIMIT, validate
from wifi_direct_netlink import INTERFACE as DIRECT_INTERFACE, profile_id as direct_profile_id
from openpilot.common.jetson_model import selected_release


def nm(*args):
  result = subprocess.run(['nmcli', '--escape', 'no', *args], capture_output=True,
                          text=True, timeout=2, env=dict(os.environ, LC_ALL='C'))
  if result.returncode:
    raise RuntimeError('NetworkManager query unavailable')
  return result.stdout.rstrip('\n')


def direct_client_id(uid):
  # Legacy protected Jetson workers do not reconsider their preferred profile
  # after roaming to a fallback. A new AP instance is a new client connection
  # identity, so their existing transactional importer retries it exactly once.
  # Phone association changes leave both inputs unchanged; no periodic churn.
  boot = str(uuid.UUID(Path('/proc/sys/kernel/random/boot_id').read_text().strip()))
  index = int((Path('/sys/class/net') / DIRECT_INTERFACE / 'ifindex').read_text())
  return str(uuid.uuid5(uuid.UUID(uid), f'client:{boot}:{index}'))


def collect():
  active = nm('-t', '-f', 'UUID', 'connection', 'show', '--active').splitlines()
  rows = nm('-t', '-f', 'UUID,TYPE', 'connection', 'show').splitlines()
  ids = [row.split(':')[0] for row in rows if row.endswith(':802-11-wireless')]
  ids.sort(key=lambda uid: uid not in active)
  profiles = []
  direct = None
  try:
    direct_id = direct_profile_id()
  except OSError:
    direct_id = None
  for uid in ids[:8]:
    if not re.fullmatch('[0-9a-f-]{36}', uid):
      continue
    fields = nm('--show-secrets', '-g',
      '802-11-wireless.ssid,802-11-wireless.mode,802-11-wireless.hidden,802-11-wireless-security.key-mgmt,802-11-wireless-security.psk',
      'connection', 'show', 'uuid', uid).split('\n')
    # Empty trailing fields are meaningful; query output is normalized below.
    fields += [''] * (5 - len(fields))
    if len(fields) != 5:
      continue
    ssid, mode, hidden, security, password = fields
    is_direct = mode == 'ap' and uid == direct_id and uid in active
    if mode not in ('infrastructure', '') and not is_direct:
      continue  # Only the explicitly owned, active Jetson AP is importable.
    if is_direct and nm('-g', 'connection.interface-name', 'connection', 'show', 'uuid', uid) != DIRECT_INTERFACE:
      continue
    if security in ('wpa-psk', 'sae') and not password:
      raise RuntimeError('Saved Wi-Fi secret unavailable')
    profile = dict(id=direct_client_id(uid) if is_direct else uid, ssid=ssid, hidden=hidden in ('yes', 'true'), active=uid in active,
                   security=security or 'open', password=password)
    try:
      validate(dict(version=1, owner='0' * 64, profiles=[profile]))
    except ValueError:
      continue
    if is_direct:
      direct = profile
    else:
      profiles.append(profile)
  if direct is not None:
    # Stable order/active selection prevents phone disconnect/rejoin from
    # reconnecting Jetson or changing its HUD path. Keep ordinary fallbacks.
    profiles.sort(key=lambda p: p['id'])
    for p in profiles:
      p['active'] = False
    profiles = [direct, *profiles[:7]]
  return profiles


def road_state(raw):
  # Current typed Params returns bool; older installations returned bytes.
  if type(raw) is bool:
    return raw
  if raw in (b'0', '0'):
    return False
  if raw in (b'1', '1'):
    return True
  return None


def main(fd):
  from openpilot.common.params import Params
  params = Params()
  owner = hashlib.sha256(Path('/etc/machine-id').read_bytes()).hexdigest()
  release = Path(__file__).resolve().parents[2] / 'openpilot/selfdrive/modeld/jetlink/host_release.json'
  manifest = selected_release(release)
  channel = socket.socket(fileno=fd)
  channel.setblocking(False)
  profiles = None
  refresh = 0
  next_ap = 0
  while True:
    now = time.monotonic()
    if now >= next_ap:
      try:
        from wifi_direct import ensure
        ensure()
      except Exception:
        pass  # Failure of this display side channel must not block USB.
      next_ap = now + 30
    if now >= refresh:
      try:
        profiles = collect()
        refresh = now + 30
      except Exception:
        # Do not send an empty deletion set after an access/query failure.
        profiles = None
        refresh = now + 5
    if profiles is not None:
      raw = params.get('IsOnroad')
      onroad = road_state(raw)
      value = dict(version=1, owner=owner, profiles=profiles, onroad=onroad)
      if manifest is not None:
        value['jetson_release'] = manifest
      data = json.dumps(value, separators=(',', ':')).encode()
      if len(data) <= LIMIT:
        try:
          channel.send(HEADER.pack(time.monotonic()) + data)
        except BlockingIOError:
          pass
    time.sleep(2)


if __name__ == '__main__':
  main(int(sys.argv[1]))
