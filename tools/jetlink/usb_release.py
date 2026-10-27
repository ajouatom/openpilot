"""Locally enabled physical USB delivery; SHA-256 is integrity, not author authentication.

Official NAS updates retain their separate signature and HTTPS policy.
USB packets cannot enable this receiver policy or replace official keys.
"""
import json
import os
from pathlib import Path
import re
import shutil
import struct
import tempfile
from urllib.parse import urlsplit

SHA256_CAPABILITY = 'carrot_usb_sha256_v1'
CHECK_CAPABILITY = 'carrot_usb_sha256_check_v1'
SHA256_POLICY = 'usb-sha256-policy.json'
SHA256_POLICY_VALUE = {'format': 1, 'integrity': 'sha256', 'transport': 'usb'}
BEGIN, CHUNK, FINISH, ACK = 0x4010, 0x4011, 0x4012, 0x4013
CHECK = 0x4014
BLOCK = 256 * 1024
HEADER = struct.Struct('<BQ')
NAMES = ('host.tar.gz', 'model.onnx')
KINDS = ('bundle', 'model')
RUNTIME = {'arch': 'aarch64', 'l4t': '36.4.7', 'tensorrt': '10.3.0'}


def validate(value):
  fields = {'format', 'delivery', 'integrity', 'profile', 'source_commit', 'runtime', 'storage_format', 'bundle', 'model'}
  if not isinstance(value, dict) or type(value.get('format')) is not int:
    raise ValueError('Invalid USB release contract')
  if value['format'] != 2 or value.get('integrity') != 'sha256':
    raise ValueError('Invalid USB integrity mode')
  if (set(value) != fields
      or value['delivery'] != 'usb' or value['runtime'] != RUNTIME or type(value['storage_format']) is not int
      or value['storage_format'] != 1 or not isinstance(value['profile'], str)
      or re.fullmatch('[a-z][a-z0-9-]{0,63}', value['profile']) is None
      or not isinstance(value['source_commit'], str) or re.fullmatch('[0-9a-f]{40}', value['source_commit']) is None):
    raise ValueError('Invalid USB release contract')
  for kind in KINDS:
    item = value[kind]
    if (not isinstance(item, dict) or set(item) != {'sha256', 'size', 'url'}
        or not isinstance(item['sha256'], str) or re.fullmatch('[0-9a-f]{64}', item['sha256']) is None
        or type(item['size']) is not int or not 0 < item['size'] <= 4 << 30 or not isinstance(item['url'], str)):
      raise ValueError('Invalid USB asset identity')
    url = urlsplit(item['url'])
    if url.scheme != 'https' or not url.hostname or url.username or url.password or url.fragment:
      raise ValueError('USB asset requires an HTTPS URL without credentials')
  return value


def sha256_enabled(root):
  from local_release import owned
  try:
    owned(Path(root), directory=True)
    updater = Path(root) / 'updater'
    policy = updater / SHA256_POLICY
    owned(updater, directory=True)
    owned(policy)
    if policy.stat().st_size > 256:
      return False
    value = json.loads(policy.read_text())
    return (isinstance(value, dict) and type(value.get('format')) is int
            and value == SHA256_POLICY_VALUE)
  except (OSError, ValueError, TypeError):
    return False


def identity(item):
  return {k: item[k] for k in ('sha256', 'size')}


def selection(value):
  return {'format': 1, 'delivery': 'local', 'profile': value['profile'], 'model': identity(value['model'])}


def record(value):
  return {'selection': selection(value), 'source_commit': value['source_commit'], 'runtime': value['runtime'],
          'storage_format': 1, 'bundle': identity(value['bundle'])}


def trusted(root, value):
  validate(value)
  if not sha256_enabled(root):
    raise ValueError('SHA-256 USB delivery requires local owner policy installation')
  return value


def registered(root, value, verify_files=True):
  from local_release import resolve, verify_assets
  directory = Path(root) / 'local-releases' / value['profile']
  if not directory.exists() and not directory.is_symlink():
    return False
  existing = resolve(root, selection(value))
  if {k: existing[k] for k in record(value)} != record(value):
    raise ValueError('USB profile already identifies another release')
  if verify_files:
    verify_assets(existing)
  return True


class Receiver:
  """One boot-gate session, fixed paths and sequential bounded writes.

  Partial files are never registered. Reconnect restarts transfer; fully verified
  registered releases can be reused. Only the locally installed USB policy grants permission.
  """
  def __init__(self, root):
    self.root = Path(root)
    self.value = None
    self.directory = None
    self.offsets = [0, 0]

  def close(self):
    if self.directory is not None:
      # Generated temp directory beneath the root-owned incoming directory only.
      shutil.rmtree(self.directory)
      self.directory = None
    self.value = None

  def begin(self, value):
    from local_release import owned
    trusted(self.root, value)  # Check local permission before allocation or cache writes.
    owned(self.root, directory=True)
    self.close()
    if registered(self.root, value):
      self.value = value
      return {'complete': True}
    parent = self.root / 'usb-incoming'
    parent.mkdir(mode=0o700, exist_ok=True)
    owned(parent, directory=True)
    required = 2 * sum(value[k]['size'] for k in KINDS) + (4 << 30)
    free = shutil.disk_usage(parent).free
    if free < required:
      raise OSError('Insufficient space for USB package and retained official release')
    # At most one receiver is owned by the boot gate. Remove its abandoned
    # staging directories from a previous power loss, never installed releases.
    for old in parent.glob('receive-*'):
      owned(old, directory=True)
      shutil.rmtree(old)
    self.directory = Path(tempfile.mkdtemp(prefix='receive-', dir=parent))
    self.value, self.offsets = value, [0, 0]
    for name in NAMES:
      (self.directory / name).touch(mode=0o600, exist_ok=False)
    return {'complete': False}

  def chunk(self, payload):
    if self.directory is None or not HEADER.size < len(payload) <= HEADER.size + BLOCK:
      raise ValueError('USB chunk without active transfer or invalid size')
    index, offset = HEADER.unpack_from(payload)
    data = memoryview(payload)[HEADER.size:]
    if index >= len(NAMES) or offset != self.offsets[index] or offset + len(data) > self.value[KINDS[index]]['size']:
      raise ValueError('USB chunk offset/length mismatch')
    with (self.directory / NAMES[index]).open('ab') as target:
      target.write(data)
    self.offsets[index] += len(data)
    return {'index': index, 'offset': self.offsets[index]}

  def finish(self):
    from local_release import owned
    from update_host import digest, atomic_json
    if self.value is None:
      raise ValueError('USB finish without manifest')
    trusted(self.root, self.value)
    if self.directory is None:
      if not registered(self.root, self.value):
        raise ValueError('USB registration disappeared')
      return {'complete': True}
    for i, name in enumerate(NAMES):
      path, item = self.directory / name, self.value[KINDS[i]]
      if self.offsets[i] != item['size'] or digest(path) != item['sha256']:
        raise ValueError('USB asset checksum/length mismatch')
      with path.open('rb') as stream:
        os.fsync(stream.fileno())
      path.chmod(0o644)
    catalog = self.root / 'local-releases'
    catalog.mkdir(mode=0o755, exist_ok=True)
    owned(catalog, directory=True)
    target = catalog / self.value['profile']
    if target.exists() or target.is_symlink():
      raise ValueError('USB profile was registered concurrently')
    atomic_json(self.directory / 'record.json', record(self.value))
    (self.directory / 'record.json').chmod(0o644)
    self.directory.chmod(0o755)
    os.sync()
    os.rename(self.directory, target)
    self.directory = None
    os.sync()
    return {'complete': True}
