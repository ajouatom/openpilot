"""Configure a prepared Jetson's boot ordering and console; retain original files for restore."""
import argparse
import base64
import json
import os
from pathlib import Path
import re
import subprocess

BACKUP = Path('/var/lib/carrot-jetlink/boot-config-before.json')
EXTLINUX = Path('/boot/extlinux/extlinux.conf')
XORG = Path('/etc/systemd/system/carrot-jetlink-xorg.service.d/boot-order.conf')
POWER = Path('/etc/systemd/system/carrot-jetlink-performance.service.d/boot-order.conf')


def boot_menu(text):
  # extlinux TIMEOUT is in tenths of a second. Keep a recovery-menu interval.
  result = re.sub(r'^TIMEOUT\s+\d+\s*$', 'TIMEOUT 10', text, flags=re.MULTILINE)
  if not re.search(r'^TIMEOUT\s+10$', result, re.MULTILINE):
    raise ValueError('No existing TIMEOUT entry; inspect this boot configuration manually')
  lines = result.splitlines()
  found = False
  for i, line in enumerate(lines):
    if line.lstrip().startswith('APPEND '):
      found = True
      if 'quiet' not in line.split():
        lines[i] = line + ' quiet'
  if not found:
    raise ValueError('No kernel APPEND line')
  return '\n'.join(lines) + '\n'


def main():
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument('--restore', action='store_true')
  args = p.parse_args()
  if os.geteuid() != 0 or not Path('/etc/nv_tegra_release').is_file():
    raise RuntimeError('Run as root on the prepared Jetson')
  if args.restore:
    records = json.loads(BACKUP.read_text())
    for name, record in records.items():
      path = Path(name)
      if record is None:
        path.unlink(missing_ok=True)
      else:
        path.write_bytes(base64.b64decode(record['data']))
        path.chmod(record['mode'])
  else:
    subprocess.run(['systemctl', 'cat', 'nvpmodel.service', 'carrot-jetlink-performance.service',
                    'carrot-jetlink-xorg.service'], check=True, stdout=subprocess.DEVNULL)
    files = {EXTLINUX: boot_menu(EXTLINUX.read_text()),
             XORG: '[Unit]\nAfter=nvpmodel.service carrot-jetlink-performance.service\nRequires=carrot-jetlink-performance.service\n',
             POWER: '[Unit]\nRequires=nvpmodel.service\n'}
    if not BACKUP.exists():
      records = {str(path): {'data': base64.b64encode(path.read_bytes()).decode(),
                            'mode': path.stat().st_mode & 0o777} if path.exists() else None for path in files}
      BACKUP.parent.mkdir(parents=True, exist_ok=True)
      with BACKUP.open('x') as f:
        json.dump(records, f, indent=2)
    for path, content in files.items():
      path.parent.mkdir(parents=True, exist_ok=True)
      temporary = path.with_suffix(path.suffix + '.carrot-tmp')
      temporary.write_text(content)
      temporary.chmod(path.stat().st_mode & 0o777 if path.exists() else 0o644)
      temporary.replace(path)
  subprocess.run(['systemctl', 'daemon-reload'], check=True)
  print('Boot configuration restored' if args.restore else 'Boot configuration installed; applies on next reboot')


if __name__ == '__main__':
  main()
