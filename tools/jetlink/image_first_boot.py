"""Provision a Carrot SD image from its local CARROTSETUP partition.

No default password, personal key, network secret or device identity is shipped.
This service never downloads software or builds an inference engine.
"""
import base64
import json
import os
from pathlib import Path
import re
import subprocess
import uuid

STATE = Path('/var/lib/carrot-jetlink')
SETUP = Path('/run/carrot-setup')


def validate_config(value):
  if not isinstance(value, dict) or set(value) - {'hostname', 'ssh_public_keys', 'wifi'}:
    raise ValueError('Unknown setup fields')
  hostname = value.get('hostname')
  if hostname is not None and (not isinstance(hostname, str) or not re.fullmatch(r'[a-z][a-z0-9-]{0,61}[a-z0-9]|[a-z]', hostname)):
    raise ValueError('Invalid hostname')
  keys = value.get('ssh_public_keys', [])
  if not isinstance(keys, list) or len(keys) > 16:
    raise ValueError('Invalid SSH key list')
  for key in keys:
    if not isinstance(key, str) or len(key) > 16384 or '\n' in key or '\r' in key:
      raise ValueError('Invalid SSH key')
    fields = key.split()
    if len(fields) < 2 or fields[0] not in {'ssh-ed25519', 'ssh-rsa', 'ecdsa-sha2-nistp256'}:
      raise ValueError('Only plain SSH public keys are accepted')
    raw = base64.b64decode(fields[1], validate=True)
    size = int.from_bytes(raw[:4], 'big')
    if raw[4:4+size] != fields[0].encode() or len(raw) <= size + 8:
      raise ValueError('Invalid SSH key encoding')
  wifi = value.get('wifi')
  if wifi is not None:
    if not isinstance(wifi, dict) or set(wifi) != {'ssid', 'password'}:
      raise ValueError('Wi-Fi requires ssid and password')
    if not isinstance(wifi['ssid'], str) or not 1 <= len(wifi['ssid'].encode()) <= 32:
      raise ValueError('Invalid SSID')
    password = wifi['password']
    if not isinstance(password, str) or not 8 <= len(password) <= 63:
      raise ValueError('WPA passphrase must contain 8 to 63 characters')
    if any(c in wifi['ssid'] + password for c in '\x00\r\n'):
      raise ValueError('Invalid Wi-Fi characters')
  return value


def atomic(path, text, mode=0o644):
  path.parent.mkdir(parents=True, exist_ok=True)
  temporary = path.with_name(path.name + '.new')
  fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, mode)
  with os.fdopen(fd, 'w') as output:
    output.write(text)
    output.flush()
    os.fsync(output.fileno())
  temporary.chmod(mode)
  temporary.replace(path)


def nm_escape(text):
  return text.replace('\\', '\\\\').replace('\t', '\\t').replace(' ', '\\s')


def main():
  if os.geteuid() != 0 or not Path('/etc/carrot-jetlink-image.json').is_file():
    raise RuntimeError('Only installed Carrot SD images are supported')
  STATE.mkdir(parents=True, exist_ok=True)
  if (STATE/'provisioned.json').exists():
    return
  SETUP.mkdir(parents=True, exist_ok=True)
  # The image partition number is fixed; never select an unrelated labelled USB disk.
  subprocess.run(['mount', '-o', 'rw,nosuid,nodev,noexec,umask=077', '/dev/mmcblk0p16', str(SETUP)], check=True)
  try:
    config_file = SETUP/'setup.json'
    config = validate_config(json.loads(config_file.read_text())) if config_file.exists() else {}
    machine = Path('/etc/machine-id').read_text().strip()
    if not re.fullmatch('[0-9a-f]{32}', machine):
      raise RuntimeError('Unique machine identity is not ready')
    hostname = config.get('hostname', 'carrot-jetson-' + machine[-6:])
    atomic(Path('/etc/hostname'), hostname + '\n')
    atomic(Path('/etc/hosts'), '127.0.0.1 localhost\n127.0.1.1 ' + hostname + '\n::1 localhost ip6-localhost ip6-loopback\n')
    subprocess.run(['hostname', hostname], check=True)
    keys = config.get('ssh_public_keys', [])
    if keys:
      directory = Path('/home/jetlink/.ssh')
      directory.mkdir(mode=0o700, parents=True, exist_ok=True)
      directory.chmod(0o700)
      os.chown(directory, 1000, 1000)
      atomic(directory/'authorized_keys', '\n'.join(keys) + '\n', 0o600)
      os.chown(directory/'authorized_keys', 1000, 1000)
    if config.get('wifi'):
      wifi = config['wifi']
      connection = ('[connection]\nid=carrot-setup\nuuid=' + str(uuid.uuid4()) + '\ntype=wifi\nautoconnect=true\n'
                    '[wifi]\nmode=infrastructure\nssid=' + nm_escape(wifi['ssid']) + '\n'
                    '[wifi-security]\nkey-mgmt=wpa-psk\npsk=' + nm_escape(wifi['password']) + '\n'
                    '[ipv4]\nmethod=auto\n[ipv6]\nmethod=auto\n')
      atomic(Path('/etc/NetworkManager/system-connections/carrot-setup.nmconnection'), connection, 0o600)
    subprocess.run(['ssh-keygen', '-A'], check=True)
    # No credentials in the persistent status file or journal.
    status = {'hostname': hostname, 'ssh_keys_installed': len(keys), 'wifi_configured': bool(config.get('wifi'))}
    # Allow a card first booted without keys to be configured on a later boot.
    if keys:
      atomic(STATE/'provisioned.json', json.dumps(status) + '\n', 0o600)
    atomic(SETUP/'SETUP-RESULT.json', json.dumps(status, indent=2) + '\n', 0o600)
    if config_file.exists():
      config_file.unlink()
    os.sync()
  finally:
    subprocess.run(['umount', str(SETUP)], check=True)


if __name__ == '__main__':
  main()
