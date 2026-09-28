"""Install USB Wi-Fi provisioning on a live host or in an offline image."""
from pathlib import Path


def configure(root):
  systemd = Path(root)/'etc/systemd/system'
  systemd.mkdir(parents=True, exist_ok=True)
  name = 'carrot-jetlink-wifi.service'
  (systemd/name).write_text('''[Unit]
Description=Import attached comma Wi-Fi profiles over private USB channel
After=NetworkManager.service carrot-image-setup.service
Wants=NetworkManager.service
StartLimitIntervalSec=0
[Service]
Type=simple
User=root
ExecStart=/opt/carrot-jetlink/venv/bin/python /opt/carrot-jetlink/current/tools/jetlink/wifi_apply.py
Restart=always
RestartSec=5
UMask=0077
Nice=19
CPUAffinity=0 1
[Install]
WantedBy=multi-user.target
''')
  link = systemd/'multi-user.target.wants'/name
  link.parent.mkdir(parents=True, exist_ok=True)
  link.unlink(missing_ok=True)
  link.symlink_to('../' + name)


if __name__ == '__main__':
  configure(Path('/'))
