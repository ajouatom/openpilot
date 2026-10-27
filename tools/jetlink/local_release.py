"""Select only experiments registered locally by root; never download them.

The USB peer supplies a profile identifier and model identity, not executable
paths, a signing key, or an installable bundle. Official releases continue to
use update_host's existing signature and HTTPS policy.
"""
import json
import os
from pathlib import Path
import re
import shutil
import stat
import tempfile

RUNTIME = {'arch': 'aarch64', 'l4t': '36.4.7', 'tensorrt': '10.3.0'}
SELECTION_FIELDS = {'format', 'delivery', 'profile', 'model'}


def validate_selection(value):
  if (not isinstance(value, dict) or set(value) != SELECTION_FIELDS or type(value['format']) is not int or
      value['format'] != 1 or value['delivery'] != 'local' or
      not isinstance(value['profile'], str) or re.fullmatch('[a-z][a-z0-9-]{0,63}', value['profile']) is None):
    raise ValueError('Invalid local experiment selection')
  model = value['model']
  if (not isinstance(model, dict) or set(model) != {'sha256', 'size'} or
      not isinstance(model['sha256'], str) or re.fullmatch('[0-9a-f]{64}', model['sha256']) is None or
      type(model['size']) is not int or not 0 < model['size'] <= 4 << 30):
    raise ValueError('Invalid local experiment model')
  return value


def owned(path, directory=False):
  info = path.lstat()
  kind = stat.S_ISDIR if directory else stat.S_ISREG
  if not kind(info.st_mode) or info.st_uid != 0 or info.st_mode & 0o022:
    raise ValueError('Local experiment must be a root-owned, non-writable regular file/directory')


def resolve(root, selection):
  """Resolve a network request against a root-installed catalog only."""
  validate_selection(selection)
  directory = Path(root) / 'local-releases' / selection['profile']
  for path in (Path(root), directory.parent, directory):
    owned(path, directory=True)
  path = directory / 'record.json'
  owned(path)
  if path.stat().st_size > 65536:
    raise ValueError('Oversized local experiment record')
  record = json.loads(path.read_text())
  if set(record) != {'selection', 'source_commit', 'runtime', 'storage_format', 'bundle'}:
    raise ValueError('Invalid local experiment record')
  if record['selection'] != selection or record['runtime'] != RUNTIME or record['storage_format'] != 1:
    raise ValueError('Local experiment is not registered for this selection')
  if re.fullmatch('[0-9a-f]{40}', str(record['source_commit'])) is None:
    raise ValueError('Invalid local source commit')
  item = record['bundle']
  if (set(item) != {'sha256', 'size'} or re.fullmatch('[0-9a-f]{64}', str(item['sha256'])) is None or
      type(item['size']) is not int or not 0 < item['size'] <= 4 << 30):
    raise ValueError('Invalid local bundle identity')
  return {**record, 'model': selection['model'], 'directory': directory}


def verify_assets(record):
  from update_host import digest
  for name, item in (('host.tar.gz', record['bundle']), ('model.onnx', record['model'])):
    path = record['directory'] / name
    owned(path)
    if path.stat().st_size != item['size'] or digest(path) != item['sha256']:
      raise ValueError('Local experiment asset is missing or corrupt: ' + name)


def stage(root, selection, progress=None):
  """Prepare a registered host/model using local files, preserving current."""
  import update_host as update
  from finalize_sd_image import extract_bundle
  record = resolve(root, selection)
  update.validate_storage_compatibility(record)
  update.check_runtime(record)
  verify_assets(record)
  releases = root / 'releases'
  releases.mkdir(exist_ok=True)
  release = releases / record['source_commit']
  if release.is_symlink():
    raise ValueError('Invalid local release directory')
  if not release.exists():
    with tempfile.TemporaryDirectory(dir=releases, prefix='local-') as temporary:
      candidate = Path(temporary)
      extract_bundle(record['directory'] / 'host.tar.gz', candidate)
      update.check_bundle(candidate, record)
      (candidate / '.bundle-sha256').write_text(record['bundle']['sha256'])
      candidate.chmod(0o755)
      os.rename(candidate, release)
  if (release / '.bundle-sha256').read_text() != record['bundle']['sha256']:
    raise ValueError('Existing local release has a different identity')
  update.check_bundle(release, record)
  models = root / 'cache/models'
  models.mkdir(parents=True, exist_ok=True)
  destination = models / (record['model']['sha256'][:16] + '.onnx')
  if destination.is_symlink():
    raise ValueError('Invalid local model cache symlink')
  if not destination.is_file() or destination.stat().st_size != record['model']['size'] or update.digest(destination) != record['model']['sha256']:
    fd, name = tempfile.mkstemp(dir=models, prefix='local-', suffix='.part')
    temporary = Path(name)
    try:
      with os.fdopen(fd, 'wb') as target, (record['directory'] / 'model.onnx').open('rb') as source:
        shutil.copyfileobj(source, target, 4 << 20)
        target.flush()
        os.fsync(target.fileno())
      temporary.chmod(0o644)
      os.replace(temporary, destination)
    finally:
      temporary.unlink(missing_ok=True)
  import pwd
  owner = pwd.getpwnam('jetlink')
  os.chown(destination, owner.pw_uid, owner.pw_gid)
  update.atomic_json(root / 'updates/pending.json', selection)
  if progress:
    progress(1.)
  os.sync()


def verify_staged(root, selection):
  import update_host as update
  record = resolve(root, selection)
  update.validate_storage_compatibility(record)
  update.check_runtime(record)
  release = root / 'releases' / record['source_commit']
  if release.is_symlink() or (release / '.bundle-sha256').read_text() != record['bundle']['sha256']:
    raise ValueError('Local release was not staged')
  update.check_bundle(release, record)
  model = root / 'cache/models' / (record['model']['sha256'][:16] + '.onnx')
  if model.is_symlink() or model.stat().st_size != record['model']['size'] or update.digest(model) != record['model']['sha256']:
    raise ValueError('Staged experiment model changed')
  return record
