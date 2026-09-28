"""Bounded, redundant identity records for the protected SD layout.

Only explicit identity files are saved, never logs, arbitrary /etc changes, or
runtime code. Checksums detect torn writes; they are not authentication. Both
partitions are private local storage, not artifacts to redistribute.
"""
import base64
import hashlib
import json
import os
from pathlib import Path
import re

LIMIT = 512 * 1024
FIXED = {'etc/hostname', 'etc/hosts',
         'home/jetlink/.ssh/authorized_keys', 'var/lib/carrot-jetlink/provisioned.json'}


def allowed(name):
  return name in FIXED or bool(re.fullmatch(
    r'etc/ssh/ssh_host_(?:rsa|ecdsa|ed25519)_key(?:\.pub)?|'
    r'etc/NetworkManager/system-connections/[a-zA-Z0-9_.-]+\.nmconnection', name))


def canonical(value):
  return json.dumps(value, sort_keys=True, separators=(',', ':')).encode()


def validate_files(files):
  if not isinstance(files, dict) or len(files) > 64:
    raise ValueError('Invalid identity record')
  for name, value in files.items():
    if not isinstance(name, str) or not allowed(name) or not isinstance(value, str):
      raise ValueError('Unexpected identity path')
    if len(base64.b64decode(value, validate=True)) > 32768:
      raise ValueError('Identity file too large')
  if len(canonical(files)) > LIMIT // 2:
    raise ValueError('Identity record too large')
  return files


def collect(root=Path('/')):
  paths = [root / name for name in FIXED]
  paths += list((root / 'etc/ssh').glob('ssh_host_*'))
  paths += list((root / 'etc/NetworkManager/system-connections').glob('*.nmconnection'))
  files = {}
  for path in paths:
    name = path.relative_to(root).as_posix()
    if allowed(name) and path.is_file() and not path.is_symlink():
      if path.stat().st_size > 32768:
        raise ValueError('Identity file too large')
      files[name] = base64.b64encode(path.read_bytes()).decode()
  return validate_files(files)


def read_record(path):
  try:
    if path.is_symlink() or path.stat().st_size > LIMIT:
      return None
    record = json.loads(path.read_bytes())
    body = record['body']
    if (body['format'] != 1 or type(body['generation']) is not int or body['generation'] < 1
        or hashlib.sha256(canonical(body)).hexdigest() != record['sha256']):
      return None
    validate_files(body['files'])
    return body
  except (OSError, ValueError, KeyError, TypeError):
    return None


def latest(directories):
  records = [record for directory in directories for slot in (0, 1)
             if (record := read_record(directory / f'identity-{slot}.json')) is not None]
  return max(records, key=lambda r: r['generation'], default=None)


def durable_write(path, data, mode=0o600):
  path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
  temporary = path.with_name(path.name + '.new')
  # No following of stale symlinks, including a previous interrupted attempt.
  fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | getattr(os, 'O_NOFOLLOW', 0), mode)
  with os.fdopen(fd, 'wb') as output:
    if os.name == 'posix':
      os.fchmod(output.fileno(), mode)
    output.write(data)
    output.flush()
    os.fsync(output.fileno())
  temporary.replace(path)
  if os.name == 'posix':
    fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
      os.fsync(fd)
    finally:
      os.close(fd)


def save(directories, files):
  validate_files(files)
  old = latest(directories)
  generation = old['generation'] if old and old['files'] == files else (old['generation'] + 1 if old else 1)
  body = dict(format=1, generation=generation, files=files)
  data = canonical(dict(body=body, sha256=hashlib.sha256(canonical(body)).hexdigest())) + b'\n'
  successes = 0
  for directory in directories:
    slots = [read_record(directory / f'identity-{slot}.json') for slot in (0, 1)]
    if any(record == body for record in slots) and all(slots):
      successes += 1
      continue  # Repeated USB packets and ordinary boots do not write flash.
    # Preserve the newest good record until its replacement is durable.
    target = min(range(2), key=lambda i: slots[i]['generation'] if slots[i] else -1)
    try:
      durable_write(directory / f'identity-{target}.json', data)
      if not any(slots):
        durable_write(directory / f'identity-{1-target}.json', data)
      successes += 1
    except OSError:
      continue  # Another partition/slot can still retain the identity.
  if not successes:
    raise OSError('No durable identity storage available')
  return generation


def restore(root, record):
  files = validate_files(record['files'])
  for name, value in files.items():
    target = root / name
    if target.is_symlink() or any(p.is_symlink() for p in target.parents if p != root and root in p.parents):
      raise ValueError('Identity target is a symlink')
    public = name in {'etc/machine-id', 'etc/hostname', 'etc/hosts'} or name.endswith('.pub')
    durable_write(target, base64.b64decode(value), 0o644 if public else 0o600)
    if name.startswith('home/jetlink/') and os.name == 'posix':
      target.parent.chmod(0o700)
      os.chown(target.parent, 1000, 1000)
      os.chown(target, 1000, 1000)
