"""Source injected into a known image-setup file by the offline patch publisher.

The publisher substitutes the package digest and size. The protected APP hash
binds this loader to the small payload; no arbitrary SETUP Python is executed.
"""
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import zipfile

PAYLOAD_SHA = 'PAYLOAD_SHA_PLACEHOLDER'
PAYLOAD_BYTES = 0  # PAYLOAD_BYTES_PLACEHOLDER
NAMES = {'boot_update.py', 'update_host.py', 'wifi_protocol.py', 'hud_protocol.py',
         'finalize_sd_image.py', 'release-signing-public.pem', 'install_updates.py', 'offline_bootstrap.py'}


def offline_bootstrap(root=Path('/')):
  from protected_storage import setup_mount
  if json.loads((root / 'run/carrot-storage.json').read_text()).get('state') != 'protected':
    return  # Preserve the immutable recovery runtime when DATA is unavailable.
  # Fail closed even if payload/cache is missing after an interrupted migration.
  # The complete verified helper adds the USB gate wait to these RAM-only guards.
  ready = root / 'run/carrot-offline-bootstrap-ready'
  ready.unlink(missing_ok=True)
  for name in ('carrot-jetlink', 'carrot-jetlink-hud'):
    units = root / 'run/systemd/system' / (name + '.service.d')
    units.mkdir(parents=True, exist_ok=True)
    (units / 'offline-bootstrap.conf').write_text(
      '[Unit]\nRequires=carrot-image-setup.service\nAfter=carrot-image-setup.service\n' +
      '[Service]\nExecStartPre=/usr/bin/test -f /run/carrot-offline-bootstrap-ready\n')
  subprocess.run(['systemctl', 'daemon-reload'], check=True, timeout=15)
  runtime = root / 'opt/carrot-jetlink'
  folder = runtime / 'offline-bootstrap' / PAYLOAD_SHA
  archive = folder / 'payload.zip'
  folder.mkdir(parents=True, exist_ok=True)
  def verified(path):
    try:
      with path.open('rb') as stream:
        data = stream.read(PAYLOAD_BYTES + 1)
      return len(data) == PAYLOAD_BYTES and hashlib.sha256(data).hexdigest() == PAYLOAD_SHA
    except OSError:
      return False
  if not verified(archive):
    with setup_mount() as identity:
      if identity is None:
        raise RuntimeError('Offline patch payload unavailable')
      with (identity.parent / 'carrot-boot-update.zip').open('rb') as stream:
        data = stream.read(PAYLOAD_BYTES + 1)
    if len(data) != PAYLOAD_BYTES or hashlib.sha256(data).hexdigest() != PAYLOAD_SHA:
      raise RuntimeError('Offline patch payload verification failed')
    temporary = archive.with_suffix('.new')
    with temporary.open('wb') as stream:
      stream.write(data)
      stream.flush()
      os.fsync(stream.fileno())
    temporary.replace(archive)
  data = archive.read_bytes()
  if len(data) != PAYLOAD_BYTES or hashlib.sha256(data).hexdigest() != PAYLOAD_SHA:
    raise RuntimeError('Offline patch cache verification failed')
  with zipfile.ZipFile(io.BytesIO(data)) as package:
    if set(package.namelist()) != NAMES or len(package.namelist()) != len(NAMES):
      raise RuntimeError('Unexpected offline patch contents')
    for name in sorted(NAMES):
      target = folder / name
      expected = package.read(name)
      if target.is_file() and target.read_bytes() == expected:
        continue
      temporary = target.with_suffix('.new')
      with temporary.open('wb') as stream:
        stream.write(expected)
        stream.flush()
        os.fsync(stream.fileno())
      temporary.chmod(0o644)
      temporary.replace(target)
  # A verified payload provides the same updater as the published signed host.
  # RAM guards are installed before writing the policy marker or launching it.
  import importlib.util
  spec = importlib.util.spec_from_file_location('carrot_offline_bootstrap', folder / 'offline_bootstrap.py')
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  module.guard_units(root / 'run/systemd/system')
  subprocess.run([str(runtime / 'venv/bin/python'), str(folder / 'offline_bootstrap.py')], check=True, timeout=30)


# The publisher appends this after definitions and invokes it before original main.
