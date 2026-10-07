"""Install the existing USB boot gate from an offline PC patch before services start.

Invoked by the patched, immutable image-setup helper using the runtime venv.
Never repartitions, changes identity/model files, or activates a candidate itself.
"""
import json
import os
from pathlib import Path
import subprocess


RUNTIME = Path('/opt/carrot-jetlink')
READY = Path('/run/carrot-offline-bootstrap-ready')
UNITS = Path('/run/systemd/system')


def install(source, root=Path('/')):
  from install_updates import bootstrap
  runtime, ready = root / 'opt/carrot-jetlink', root / 'run/carrot-offline-bootstrap-ready'
  if json.loads((root / 'run/carrot-storage.json').read_text()).get('state') != 'protected':
    raise RuntimeError('Protected DATA runtime is required')
  required = ('boot_update.py', 'update_host.py', 'wifi_protocol.py', 'hud_protocol.py',
              'finalize_sd_image.py', 'release-signing-public.pem')
  # Once installed, never overwrite a later updater with the PC package's version.
  marker = runtime / 'boot-update-required'
  if not marker.is_file():
    bootstrap(root, source)
  if not all((runtime / 'updater' / name).is_file() for name in required):
    raise RuntimeError('Installed boot updater is incomplete')
  temporary = ready.with_suffix('.new')
  temporary.write_text('installed\n')
  temporary.chmod(0o644)
  temporary.replace(ready)
  os.sync()


def guard_units(units=UNITS):
  """RAM-only guards also protect old runtimes that lack Python entry guards."""
  command = ('/opt/carrot-jetlink/venv/bin/python -c "import sys; ' +
             "sys.path.insert(0, '/opt/carrot-jetlink/updater'); " +
             'from boot_update import wait_for_runtime; wait_for_runtime()"')
  body = ('[Unit]\nRequires=carrot-image-setup.service carrot-jetlink-update-apply.service\n' +
          'After=carrot-image-setup.service carrot-jetlink-update-apply.service\n' +
          '[Service]\nTimeoutStartSec=0\n' +
          'ExecStartPre=/usr/bin/test -f /run/carrot-offline-bootstrap-ready\n' +
          'ExecStartPre=' + command + '\n')
  for name in ('carrot-jetlink', 'carrot-jetlink-hud'):
    folder = units / (name + '.service.d')
    folder.mkdir(parents=True, exist_ok=True)
    (folder / 'offline-bootstrap.conf').write_text(body)
  subprocess.run(['systemctl', 'daemon-reload'], check=True, timeout=15)


if __name__ == '__main__':
  if os.geteuid() != 0:
    raise PermissionError('Root image setup service required')
  install(Path(__file__).resolve().parent)
