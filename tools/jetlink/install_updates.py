"""Install the small stable updater into a live host or an unmounted image root."""
from pathlib import Path
import shutil


def configure(root, source):
  root = Path(root)
  target = root / 'opt/carrot-jetlink/updater'
  target.mkdir(parents=True, exist_ok=True)
  for name in ('update_host.py', 'finalize_sd_image.py', 'hud_protocol.py', 'release-signing-public.pem'):
    shutil.copyfile(source / name, target / name)
    (target / name).chmod(0o644)
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
  configure(Path('/'), Path(__file__).resolve().parent)
