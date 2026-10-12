"""Register a PC-prepared experiment in a stopped Jetson runtime (no activation)."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile

HELPERS = ('local_release.py', 'update_host.py', 'boot_update.py', 'finalize_sd_image.py',
           'wifi_protocol.py', 'hud_protocol.py', 'release-signing-public.pem')


def digest(path):
  value = hashlib.sha256()
  with path.open('rb') as source:
    while block := source.read(4 << 20):
      value.update(block)
  return value.hexdigest()


def inspect_package(package):
  manifest = json.loads((package / 'package.json').read_text())
  names = {'host.tar.gz', 'model.onnx', 'install_local_experiment.py', *HELPERS}
  if manifest.get('format') != 1 or set(manifest.get('files', {})) != names:
    raise ValueError('Unsupported local experiment package')
  for name, item in manifest['files'].items():
    path = package / name
    if path.is_symlink() or not path.is_file() or path.stat().st_size != item['size'] or digest(path) != item['sha256']:
      raise ValueError('Package file verification failed: ' + name)
  with tarfile.open(package / 'host.tar.gz') as archive:
    commit = archive.extractfile('SOURCE_COMMIT').read().decode().strip()
    prefix = 'openpilot/selfdrive/modeld/jetlink/'
    selection = json.load(archive.extractfile(prefix + 'local_host_release.json'))
    spec = json.load(archive.extractfile(prefix + 'jetson_model.json'))
    compatibility = json.load(archive.extractfile('tools/jetlink/runtime_compatibility.json'))
    if compatibility['protected_storage_format'] != 1:
      raise ValueError('Bundle does not support protected storage')
    for name in (*HELPERS, 'install_local_experiment.py'):
      if archive.extractfile('tools/jetlink/' + name).read() != (package / name).read_bytes():
        raise ValueError('Package helper differs from committed bundle: ' + name)
  if selection['model'] != {'sha256': spec['sha256'], 'size': spec['nbytes']}:
    raise ValueError('Bundle and selected model disagree')
  if manifest['files']['model.onnx'] != selection['model']:
    raise ValueError('Package has the wrong model')
  # Import verified helper code only after comparing it with the bundle.
  sys.path.insert(0, str(package))
  from local_release import RUNTIME, validate_selection
  validate_selection(selection)
  record = {'selection': selection, 'source_commit': commit, 'runtime': RUNTIME,
            'storage_format': 1, 'bundle': manifest['files']['host.tar.gz']}
  return record


def atomic_copy(source, target):
  temporary = target.with_suffix(target.suffix + '.new')
  with source.open('rb') as incoming, temporary.open('wb') as outgoing:
    shutil.copyfileobj(incoming, outgoing, 4 << 20)
    outgoing.flush()
    os.fsync(outgoing.fileno())
  temporary.chmod(0o644)
  os.replace(temporary, target)


def install(package, runtime):
  if os.geteuid() != 0:
    raise PermissionError('Run as root against the stopped/mounted Jetson DATA runtime')
  package, runtime = Path(package).resolve(), Path(runtime).resolve()
  if runtime == Path('/opt/carrot-jetlink').resolve():
    for unit in ('carrot-jetlink', 'carrot-jetlink-hud', 'carrot-jetlink-boot-gate', 'carrot-jetlink-update-stage'):
      if subprocess.run(['systemctl', 'is-active', '--quiet', unit]).returncode == 0:
        raise RuntimeError('Use recovery/offline mode; a Jetson service is still running')
  record = inspect_package(package)
  from local_release import owned
  owned(runtime, directory=True)
  updater = runtime / 'updater'
  owned(updater, directory=True)
  if not (runtime / 'boot-update-required').is_file() or not (runtime / 'protected-runtime.json').is_file():
    raise ValueError('Expected the existing protected DATA runtime with boot updates enabled')
  public_key = updater / 'release-signing-public.pem'
  owned(public_key)
  if public_key.read_bytes() != (package / public_key.name).read_bytes():
    raise ValueError('Official trust root differs; refusing to replace it')
  catalog = runtime / 'local-releases'
  catalog.mkdir(mode=0o755, exist_ok=True)
  owned(catalog, directory=True)
  target = catalog / record['selection']['profile']
  if target.exists() or target.is_symlink():
    from local_release import resolve, verify_assets
    existing = resolve(runtime, record['selection'])
    if {key: existing[key] for key in record} != record:
      raise ValueError('This profile already names another bundle; use a new profile ID')
    verify_assets(existing)
  else:
    needed = 2 * (record['selection']['model']['size'] + record['bundle']['size']) + (4 << 30)
    if shutil.disk_usage(runtime).free < needed:
      raise OSError('Insufficient DATA space for retained official files and experimental engine')
    with tempfile.TemporaryDirectory(dir=catalog, prefix='install-') as name:
      temporary = Path(name)
      for asset in ('host.tar.gz', 'model.onnx'):
        atomic_copy(package / asset, temporary / asset)
        item = record['bundle'] if asset == 'host.tar.gz' else record['selection']['model']
        if digest(temporary / asset) != item['sha256']:
          raise ValueError('Installed asset checksum mismatch')
      (temporary / 'record.json').write_text(json.dumps(record, indent=2) + '\n')
      (temporary / 'record.json').chmod(0o644)
      temporary.chmod(0o755)
      os.sync()
      os.rename(temporary, target)
  # Keep the official key, networking and boot policy. Only extend the selector.
  backup = updater / 'before-local-experiments'
  backup.mkdir(mode=0o700, exist_ok=True)
  owned(backup, directory=True)
  for name in ('local_release.py', 'update_host.py', 'boot_update.py'):
    destination = updater / name
    if destination.exists():
      owned(destination)
      if not (backup / name).exists():
        atomic_copy(destination, backup / name)
    atomic_copy(package / name, destination)
  os.sync()
  print('REGISTERED', record['selection']['profile'], record['source_commit'], flush=True)
  print('Current official release preserved. Selection occurs at the next Jetson boot.', flush=True)


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--package', type=Path, default=Path(__file__).resolve().parent)
  parser.add_argument('--runtime', type=Path, required=True, help='Mounted DATA/runtime, or stopped /opt/carrot-jetlink')
  args = parser.parse_args()
  install(args.package, args.runtime)
