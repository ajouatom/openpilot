"""Offline installation of the pinned, CPU-only diagnostic screen."""
from pathlib import Path
import shutil


def configure(root, source):
  root, source = Path(root), Path(source)
  repository = source.resolve().parents[1]
  target = root / 'usr/local/lib/carrot-jetlink/boot-display'
  relative = ['openpilot/__init__.py', 'tools/jetlink/boot_display.py', 'tools/jetlink/boot_status.py',
              'tools/jetlink/display_owner.py', 'openpilot/common/usbgpu_bus_lock.py',
              'openpilot/selfdrive/carrot/cluster/cluster_usb_display.py',
              'openpilot/selfdrive/carrot/cluster/cluster_config.py',
              'openpilot/selfdrive/carrot/cluster/cluster_utils.py',
              'openpilot/selfdrive/assets/fonts/KaiGenGothicKR-Bold.ttf', 'LICENSE']
  for name in relative:
    path = target / name
    path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(repository / name, path)
  vendor = 'openpilot/selfdrive/carrot/cluster/.vendor/turing-smart-screen-python-main'
  shutil.copytree(repository / vendor, target / vendor, dirs_exist_ok=True,
                  ignore=shutil.ignore_patterns('__pycache__', '*.pyc', 'log.log'))
  units = root / 'etc/systemd/system'
  units.mkdir(parents=True, exist_ok=True)
  name = 'carrot-jetlink-boot-display.service'
  (units / name).write_text('''[Unit]
Description=Jetson USB boot and recovery status without Xorg or comma
After=local-fs.target systemd-udev-trigger.service
StartLimitIntervalSec=0
[Service]
User=jetlink
SupplementaryGroups=plugdev video render
RuntimeDirectory=carrot-jetlink-display
RuntimeDirectoryMode=0700
RuntimeDirectoryPreserve=yes
WorkingDirectory=/run/carrot-jetlink-display
Environment=PYTHONDONTWRITEBYTECODE=1
Environment=PYTHONUNBUFFERED=1
Environment=OPENBLAS_NUM_THREADS=1
Environment=OMP_NUM_THREADS=1
ExecStart=/opt/carrot-jetlink/venv/bin/python /usr/local/lib/carrot-jetlink/boot-display/tools/jetlink/boot_display.py
Restart=always
RestartSec=3
Nice=19
CPUAffinity=0 1
UMask=0077
[Install]
WantedBy=multi-user.target
''')
  link = units / 'multi-user.target.wants' / name
  link.parent.mkdir(parents=True, exist_ok=True)
  link.unlink(missing_ok=True)
  link.symlink_to('../' + name)
  dropin = units / 'carrot-jetlink-hud.service.d/diagnostics.conf'
  dropin.parent.mkdir(parents=True, exist_ok=True)
  dropin.write_text('[Service]\nRuntimeDirectory=carrot-jetlink-display\n'
                   'RuntimeDirectoryMode=0700\nRuntimeDirectoryPreserve=yes\n')
