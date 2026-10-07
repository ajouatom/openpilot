"""Install the small stable updater into a live host or an unmounted image root."""
from pathlib import Path
import os
import argparse
import subprocess
import sys


def migrate_running_release(runtime=Path('/opt/carrot-jetlink')):
  """A signed runtime can migrate its own updater without per-car SSH setup.

  Published images already grant the jetlink service account passwordless sudo.
  Enable only next boot so installing never interrupts the current session.
  """
  if not (runtime / 'current/tools/jetlink/boot_update.py').is_file():
    return
  if (runtime / 'boot-update-required').is_file():
    return
  subprocess.run(['sudo', '-n', sys.executable, str(runtime / 'current/tools/jetlink/install_updates.py'),
                  '--bootstrap-only'], check=True, timeout=30)


def bootstrap(root, source, defer_this_boot=False):
  root, source = Path(root), Path(source)
  target = root / 'opt/carrot-jetlink/updater'
  target.mkdir(parents=True, exist_ok=True)
  # Dependencies first, entry point last; enable only after every file is durable.
  for name in ('finalize_sd_image.py', 'hud_protocol.py', 'release-signing-public.pem',
               'wifi_protocol.py', 'boot_update.py', 'update_host.py'):
    temporary = target / (name + '.new')
    with temporary.open('wb') as stream:
      stream.write((source / name).read_bytes())
      stream.flush()
      os.fsync(stream.fileno())
    temporary.chmod(0o644)
    temporary.replace(target / name)
  marker = target.parent / 'boot-update-required'
  temporary = marker.with_suffix('.new')
  temporary.write_text(Path('/proc/sys/kernel/random/boot_id').read_text() if defer_this_boot else 'next-boot\n')
  temporary.replace(marker)
  if hasattr(os, 'sync'):
    os.sync()


def configure(root, source):
  root = Path(root)
  bootstrap(root, source)
  systemd = root / 'etc/systemd/system'
  command = '/opt/carrot-jetlink/venv/bin/python /opt/carrot-jetlink/updater/update_host.py'
  units = {
    'carrot-jetlink-update-apply.service': f'''[Unit]
Description=Validate and activate staged Jetson release before inference
After=local-fs.target carrot-image-setup.service
Before=carrot-jetlink.service carrot-jetlink-hud.service
[Service]
Type=oneshot
ExecStart={command} activate
TimeoutStartSec=1000
RemainAfterExit=yes
[Install]
WantedBy=multi-user.target
''',
    'carrot-jetlink-update-stage.service': f'''[Unit]
Description=Stage verified Jetson updates when C4 reports offroad
After=network-online.target
[Service]
Type=oneshot
ExecStart={command} automatic
Nice=19
IOSchedulingClass=idle
CPUAffinity=0 1
TimeoutStartSec=1800
''',
    'carrot-jetlink-update-stage.timer': '''[Unit]
Description=Check for Jetson runtime updates while offroad
[Timer]
OnBootSec=2min
OnUnitActiveSec=15min
RandomizedDelaySec=30
[Install]
WantedBy=timers.target
''',
  }
  for name, body in units.items():
    (systemd / name).write_text(body)
  for name, target_name in (('carrot-jetlink-update-apply.service', 'multi-user.target'),
                            ('carrot-jetlink-update-stage.timer', 'timers.target')):
    link = systemd / (target_name + '.wants') / name
    link.parent.mkdir(parents=True, exist_ok=True)
    link.unlink(missing_ok=True)
    link.symlink_to('../' + name)


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--bootstrap-only', action='store_true', help='Migrate an installed runtime; enable from its next boot without OS writes')
  args = parser.parse_args()
  if args.bootstrap_only:
    source = Path('/opt/carrot-jetlink/current/tools/jetlink')
    if not (source / 'boot_update.py').is_file():
      raise RuntimeError('Install the boot-gated source release before enabling this policy')
    bootstrap(Path('/'), source, defer_this_boot=True)
  else:
    configure(Path('/'), Path(__file__).resolve().parent)
