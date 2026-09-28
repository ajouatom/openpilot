"""Root NetworkManager worker for validated, private USB Wi-Fi provisioning."""
import json
import os
from pathlib import Path
import subprocess
import time
import uuid
import configparser

from image_first_boot import nm_escape
from persistent_state import durable_write
from wifi_protocol import read_packet, validate

DIRECTORY = Path('/etc/NetworkManager/system-connections')
STATUS = Path('/dev/shm/carrot-jetlink-network-status.json')
PREFIX = 'carrot-usb-'


def atomic(path, text, mode=0o644):
  durable_write(path, text.encode(), mode)


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
          'autoconnect=true\nautoconnect-retries=0\nautoconnect-priority=' + str(priority) + '\n'
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
    previous = path.read_text(encoding='utf-8') if path.exists() else None
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


def connect(ids, command=nm, preferred=None, force=False):
  states = command('-t', '-f', 'DEVICE,TYPE,STATE', 'device', 'status').splitlines()
  connected = any(':wifi:connected' in row for row in states)
  active = command('-t', '-f', 'UUID', 'connection', 'show', '--active').splitlines() if preferred else []
  if not force and connected and (preferred is None or preferred in active):
    return 'connected'
  if not ids:
    return 'waiting_for_comma_wifi'
  command('radio', 'wifi', 'on')
  # NetworkManager handles availability, password validation, and DHCP. Try
  # the comma's preferred profile first, retaining every imported fallback.
  order = ([preferred] + [uid for uid in ids if uid != preferred]) if preferred else ids
  for uid in order:
    try:
      command('--wait', '12', 'connection', 'up', 'uuid', uid)
      return 'connected'
    except RuntimeError:
      continue
  return 'retrying'


def saved_ids(directory=DIRECTORY):
  """Only saved client networks; never activate a hotspot as a recovery path."""
  profiles = []
  for path in directory.glob('*.nmconnection'):
    try:
      if path.is_symlink() or path.stat().st_size > 32768:
        continue
      value = configparser.ConfigParser(interpolation=None, strict=False)
      value.read_string(path.read_text(encoding='utf-8'))
      connection = value['connection']
      if connection.get('type') not in ('wifi', '802-11-wireless'):
        continue
      wifi = value['wifi'] if 'wifi' in value else value['802-11-wireless']
      if wifi.get('mode', 'infrastructure') != 'infrastructure' or not connection.getboolean('autoconnect', fallback=True):
        continue
      uid = str(uuid.UUID(connection['uuid']))
      profiles.append((connection.getint('autoconnect-priority', fallback=0), uid))
    except (OSError, ValueError, KeyError, configparser.Error):
      continue
  return [uid for _, uid in sorted(profiles, reverse=True)]


class Worker:
  """Wi-Fi recovery does not depend on USB, a model, or an update succeeding."""
  def __init__(self, directory=DIRECTORY, command=nm, persist=lambda: None):
    self.directory, self.command, self.persist = directory, command, persist
    self.applied = None
    self.next_attempt = 0
    self.state = 'starting'
    self.count = 0
    self.recovered = False

  def rollback(self):
    journal = self.directory / '.carrot-wifi-rollback.json'
    if not journal.exists():
      return
    if journal.is_symlink() or journal.stat().st_size > 512 * 1024:
      raise ValueError('Unexpected Wi-Fi recovery journal')
    value = json.loads(journal.read_text(encoding='utf-8'))
    old, new = value['old'], value['new']
    if (not isinstance(old, dict) or not isinstance(new, list) or len(old) > 64 or len(new) > 8
        or any(not isinstance(v, str) or len(v) > 32768 for v in old.values())):
      raise ValueError('Invalid Wi-Fi recovery journal')
    for name in list(old) + new:
      uid = name[len(PREFIX):-len('.nmconnection')]
      if name != PREFIX + str(uuid.UUID(uid)) + '.nmconnection':
        raise ValueError('Unexpected Wi-Fi recovery path')
    for name in new:
      if name not in old:
        try:
          self.command('connection', 'delete', 'uuid', name[len(PREFIX):-len('.nmconnection')])
        except RuntimeError:
          pass
        (self.directory / name).unlink(missing_ok=True)
    for name, content in old.items():
      path = self.directory / name
      atomic(path, content, 0o600)
      self.command('connection', 'load', str(path))
    journal.unlink()

  def step(self, packet, now):
    if now < self.next_attempt:
      return self.state
    self.next_attempt = now + 30
    try:
      if not self.recovered:
        try:
          self.rollback()
        except (ValueError, KeyError, TypeError):
          # A damaged journal must not disable all saved-network retries.
          # Preserve it privately, then allow a fresh USB profile to recover.
          journal = self.directory / '.carrot-wifi-rollback.json'
          journal.replace(self.directory / '.carrot-wifi-rollback.invalid')
        self.recovered = True
      # Snapshot profiles before replacement. Failed credentials must not remove
      # the only previously working network, even across a service restart.
      value = packet[1] if packet is not None else None
      identity = (value['owner'], value['profiles']) if value is not None else None
      changed = value is not None and bool(value['profiles']) and identity != self.applied
      old = {p.name: p.read_text(encoding='utf-8') for p in self.directory.glob(PREFIX + '*.nmconnection') if not p.is_symlink()}
      if changed:
        try:
          journal = self.directory / '.carrot-wifi-rollback.json'
          rendered = [render(value['owner'], p, 100 - i) for i, p in enumerate(value['profiles'])]
          desired = {PREFIX + uid + '.nmconnection': content for uid, content in rendered}
          new = list(desired)
          atomic(journal, json.dumps({'old': old, 'new': new}), 0o600)
          ids = install(value, self.directory, self.command)
          preferred = next((uid for uid, p in zip(ids, value['profiles']) if p['active']), None)
          self.state = connect(ids, self.command, preferred=preferred, force=old != desired)
          active = self.command('-t', '-f', 'UUID', 'connection', 'show', '--active').splitlines()
          if self.state != 'connected' or not set(active).intersection(ids):
            raise RuntimeError('Candidate Wi-Fi not connected')
          self.persist()
          self.applied = identity
          journal.unlink()
        except Exception:
          self.rollback()
          # Restore connectivity using saved profiles, then retry USB later.
          connect(saved_ids(self.directory), self.command)
          raise
      else:
        self.state = connect(saved_ids(self.directory), self.command)
      self.count = len(saved_ids(self.directory))
    except Exception:
      self.state = 'retrying'
      self.recovered = False
    return self.state


def main():
  if os.geteuid() != 0:
    raise PermissionError('Network provisioning requires its root service')
  # One-time bridge for already installed images whose old service points into
  # a signed runtime. Install independent networking for the next start. Keep
  # this worker running even if migration is interrupted or storage is full.
  source = Path(__file__).resolve().parent
  if source.is_relative_to(Path('/opt/carrot-jetlink/releases')) and not Path('/etc/carrot-jetlink-protected.json').exists():
    try:
      from install_wifi import configure
      from install_updates import configure as configure_updates
      configure(Path('/'), source)
      configure_updates(Path('/'), source)
      subprocess.run(['systemctl', 'daemon-reload'], check=True, timeout=10,
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
      pass  # Network recovery remains available; the next start retries.
  # Protected-image persistence is installed independently of updateable model
  # code. Legacy images keep their existing on-disk NetworkManager profiles.
  def persist():
    helper = Path('/usr/lib/carrot-jetlink-storage/protected_storage.py')
    if helper.is_file():
      subprocess.run(['/usr/bin/python3', str(helper), 'save'], check=True, timeout=20,
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
  worker = Worker(persist=persist)
  while True:
    try:
      state = worker.step(read_packet(), time.monotonic())
    except Exception:
      state = 'retrying'
    atomic(STATUS, json.dumps({'state': state, 'profiles': worker.count, 'updated': time.monotonic()}) + '\n', 0o644)
    time.sleep(2)


if __name__ == '__main__':
  main()
