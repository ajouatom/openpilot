"""Root NetworkManager worker for validated, private USB Wi-Fi provisioning."""
import json
import os
from pathlib import Path
import subprocess
import time
import uuid

from image_first_boot import atomic, nm_escape
from wifi_protocol import read_packet, validate

DIRECTORY = Path('/etc/NetworkManager/system-connections')
STATUS = Path('/dev/shm/carrot-jetlink-network-status.json')
PREFIX = 'carrot-usb-'


def nm(*args):
  result = subprocess.run(['nmcli', '--escape', 'no', *args], capture_output=True,
                          text=True, timeout=18, env=dict(os.environ, LC_ALL='C'))
  if result.returncode:
    # Command output can contain names and secrets. Never include it in errors.
    raise RuntimeError('NetworkManager operation failed')
  return result.stdout.rstrip('\n')


def render(owner, profile, priority):
  uid = str(uuid.uuid5(uuid.NAMESPACE_URL, 'carrot-usb:' + owner + ':' + profile['id']))
  text = ('[connection]\nid=' + PREFIX + uid + '\nuuid=' + uid + '\ntype=wifi\n'
          'autoconnect=true\nautoconnect-priority=' + str(priority) + '\n'
          '[wifi]\nmode=infrastructure\nssid=' + nm_escape(profile['ssid']) + '\n'
          'hidden=' + ('true' if profile['hidden'] else 'false') + '\n')
  if profile['security'] != 'open':
    text += ('[wifi-security]\nkey-mgmt=' + profile['security'] + '\npsk=' +
             nm_escape(profile['password']) + '\npsk-flags=0\n')
  text += '[ipv4]\nmethod=auto\n[ipv6]\nmethod=auto\n'
  return uid, text


def install(value, directory=DIRECTORY, command=nm):
  validate(value)
  desired = {}
  for i, profile in enumerate(value['profiles']):
    uid, content = render(value['owner'], profile, 100 - i)
    desired[PREFIX + uid + '.nmconnection'] = (uid, content)
  directory.mkdir(parents=True, exist_ok=True)
  # Install the new profiles before removing the previous comma's managed set.
  # Other administrator-created profiles are never deleted.
  for name, (uid, content) in desired.items():
    path = directory/name
    if path.is_symlink():
      raise ValueError('Unexpected network profile symlink')
    previous = path.read_text() if path.exists() else None
    if previous != content:
      atomic(path, content, 0o600)
      try:
        command('connection', 'load', str(path))
      except Exception:
        if previous is None:
          path.unlink(missing_ok=True)
        else:
          atomic(path, previous, 0o600)
        raise
  for path in directory.glob(PREFIX + '*.nmconnection'):
    if path.name not in desired:
      uid = path.name[len(PREFIX):-len('.nmconnection')]
      if str(uuid.UUID(uid)) != uid or path.is_symlink():
        raise ValueError('Unexpected managed profile path')
      # delete also unloads NetworkManager's in-memory secret/profile cache.
      command('connection', 'delete', 'uuid', uid)
      path.unlink(missing_ok=True)
  return [uid for uid, _ in desired.values()]


def connect(ids, command=nm, preferred=None):
  states = command('-t', '-f', 'DEVICE,TYPE,STATE', 'device', 'status').splitlines()
  connected = any(':wifi:connected' in row for row in states)
  active = command('-t', '-f', 'UUID', 'connection', 'show', '--active').splitlines() if preferred else []
  if connected and (preferred is None or preferred in active):
    return 'connected'
  if not ids:
    return 'waiting_for_comma_wifi'
  command('radio', 'wifi', 'on')
  # NetworkManager handles availability, password validation, and DHCP. Try
  # the comma's preferred profile first, retaining every imported fallback.
  for uid in ([preferred] if connected and preferred else ids):
    try:
      command('--wait', '12', 'connection', 'up', 'uuid', uid)
      return 'connected'
    except RuntimeError:
      continue
  return 'retrying'


def main():
  if os.geteuid() != 0:
    raise PermissionError('Network provisioning requires its root service')
  applied = None
  ids = []
  next_attempt = 0
  while True:
    packet = read_packet()
    state = 'waiting_for_usb'
    try:
      if packet is not None:
        value = packet[1]
        # Includes owner and ordering, but never persist a password-derived hash.
        identity = (value['owner'], value['profiles'])
        if identity != applied:
          ids = install(value)
          applied = identity
          next_attempt = 0
        if time.monotonic() >= next_attempt:
          preferred = next((uid for uid, p in zip(ids, value['profiles']) if p['active']), None)
          state = connect(ids, preferred=preferred)
          next_attempt = time.monotonic() + 30
        else:
          state = 'configured'
    except Exception:
      state = 'retrying'
      next_attempt = time.monotonic() + 30
    atomic(STATUS, json.dumps({'state': state, 'profiles': len(ids), 'updated': time.monotonic()}) + '\n', 0o644)
    time.sleep(2)


if __name__ == '__main__':
  main()
