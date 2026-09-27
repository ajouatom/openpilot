"""Install the validated Type-C policy on a host or an offline mounted image."""
from pathlib import Path
import shutil


def configure(root, source=None):
  root = Path(root)
  source = Path(source) if source is not None else Path(__file__).parent
  helper = root/'usr/local/lib/carrot-jetlink/usbc_host.py'
  helper.parent.mkdir(parents=True, exist_ok=True)
  shutil.copyfile(source/'usbc_host.py', helper)
  helper.chmod(0o644)
  systemd = root/'etc/systemd/system'
  systemd.mkdir(parents=True, exist_ok=True)
  name = 'carrot-jetlink-usbc-host.service'
  (systemd/name).write_text('''[Unit]
Description=Carrot Jetson USB-C host role policy
After=systemd-udev-trigger.service
Before=carrot-jetlink.service
[Service]
Type=oneshot
User=root
ExecStart=/usr/bin/python3 /usr/local/lib/carrot-jetlink/usbc_host.py
RemainAfterExit=yes
TimeoutStartSec=40
[Install]
WantedBy=multi-user.target
''')
  link = systemd/'multi-user.target.wants'/name
  link.parent.mkdir(parents=True, exist_ok=True)
  link.unlink(missing_ok=True)
  link.symlink_to('../' + name)


if __name__ == '__main__':
  configure('/')
