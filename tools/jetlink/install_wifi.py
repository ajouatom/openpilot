"""Install USB Wi-Fi provisioning on a live host or in an offline image."""
from pathlib import Path
import hashlib

from persistent_state import durable_write


def configure(root, source=None):
  root = Path(root)
  source = Path(source) if source is not None else Path(__file__).parent
  names = ('wifi_apply.py', 'wifi_protocol.py', 'image_first_boot.py', 'persistent_state.py')
  files = {name: (source / name).read_bytes() for name in names}
  version = hashlib.sha256(b''.join(name.encode() + b'\0' + files[name] for name in names)).hexdigest()
  base = root / 'usr/local/lib/carrot-jetlink/network'
  release = base / version
  release.mkdir(parents=True, exist_ok=True)
  for name, data in files.items():
    target = release / name
    if not target.exists() or target.read_bytes() != data:
      durable_write(target, data, 0o644)
  link = base / 'current.new'
  link.unlink(missing_ok=True)
  link.symlink_to(version)
  link.replace(base / 'current')
  systemd = root/'etc/systemd/system'
  systemd.mkdir(parents=True, exist_ok=True)
  name = 'carrot-jetlink-wifi.service'
  durable_write(systemd/name, b'''[Unit]
Description=Import attached comma Wi-Fi profiles over private USB channel
After=NetworkManager.service carrot-image-setup.service
Wants=NetworkManager.service
StartLimitIntervalSec=0
[Service]
Type=simple
User=root
ExecStart=/usr/bin/python3 /usr/local/lib/carrot-jetlink/network/current/wifi_apply.py
Restart=always
RestartSec=5
UMask=0077
Nice=19
CPUAffinity=0 1
[Install]
WantedBy=multi-user.target
''', 0o644)
  link = systemd/'multi-user.target.wants'/name
  link.parent.mkdir(parents=True, exist_ok=True)
  temporary = link.with_name(link.name + '.new')
  temporary.unlink(missing_ok=True)
  temporary.symlink_to('../' + name)
  temporary.replace(link)


if __name__ == '__main__':
  configure(Path('/'))
