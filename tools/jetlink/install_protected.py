"""Configure a NEW offline image with a separate, prepopulated DATA partition.

Not a live migration tool. Never install this directly on a legacy expanded SD.
"""
import json
from pathlib import Path
import re
import shutil

from finalize_sd_image import write


def configure(root, source):
  root, source = Path(root).resolve(), Path(source).resolve()
  if root == Path('/') or not (root / 'etc/carrot-jetlink-image.json').is_file():
    raise ValueError('Only an offline Carrot image is supported')
  destination = root / 'usr/lib/carrot-jetlink-storage'
  destination.mkdir(parents=True, exist_ok=True)
  for name in ('protected_storage.py', 'persistent_state.py', 'wifi_apply.py', 'wifi_protocol.py', 'image_first_boot.py', 'protected_first_boot.py', 'boot_status.py'):
    shutil.copyfile(source / name, destination / name)
    (destination / name).chmod(0o644)
  write(root, '/etc/carrot-jetlink-protected.json', json.dumps(
    dict(format=1, root='/dev/mmcblk0p1', data='/dev/mmcblk0p17', setup='/dev/mmcblk0p16')) + '\n')
  write(root, '/etc/fstab', '/dev/root / ext4 ro 0 0\n/dev/mmcblk0p10 /boot/efi vfat ro,nofail 0 0\n')
  extlinux = root / 'boot/extlinux/extlinux.conf'
  lines = extlinux.read_text().splitlines()
  found = False
  for index, line in enumerate(lines):
    if re.match(r'\s*APPEND\s', line):
      if 'root=/dev/mmcblk0p1' not in line:
        raise ValueError('Unexpected boot root')
      lines[index] = re.sub(r'\s+rw(?=\s|$)', '', line) + ' ro'
      found = True
  if not found:
    raise ValueError('Missing SD boot command line')
  write(root, '/boot/extlinux/extlinux.conf', '\n'.join(lines) + '\n')
  command = '/usr/bin/python3 /usr/lib/carrot-jetlink-storage/protected_storage.py'
  write(root, '/etc/systemd/system/carrot-protected-storage.service', f'''[Unit]
Description=Read-only system, volatile writes and independent DATA recovery
DefaultDependencies=no
After=systemd-remount-fs.service
Before=local-fs-pre.target systemd-tmpfiles-setup.service systemd-journald.service
Before=carrot-image-setup.service NetworkManager.service ssh.service
[Service]
Type=oneshot
ExecStart={command} boot
RemainAfterExit=yes
TimeoutStartSec=100
[Install]
WantedBy=local-fs-pre.target
''')
  link = root / 'etc/systemd/system/local-fs-pre.target.wants/carrot-protected-storage.service'
  link.parent.mkdir(parents=True, exist_ok=True)
  link.unlink(missing_ok=True)
  link.symlink_to('../carrot-protected-storage.service')
  # Legacy grow expands APP over the remainder of the disk. It must NEVER run
  # on a split image. Fixed DATA size is intentional until a separate grow tool
  # is boot-tested; unused tail space is safer than resizing the wrong partition.
  write(root, '/etc/systemd/system/carrot-image-grow.service', '[Unit]\nDescription=Legacy APP growth disabled on protected layout\nConditionPathExists=/nonexistent-carrot-legacy-layout\n[Service]\nType=oneshot\nExecStart=/bin/true\n')
  write(root, '/etc/systemd/journald.conf.d/carrot-volatile.conf',
        '[Journal]\nStorage=volatile\nRuntimeMaxUse=32M\nRuntimeKeepFree=64M\n')
  write(root, '/etc/security/limits.d/carrot-no-core.conf', '* hard core 0\n')
  write(root, '/etc/systemd/coredump.conf.d/carrot-volatile.conf', '[Coredump]\nStorage=none\nProcessSizeMax=0\n')
  write(root, '/etc/systemd/system/carrot-image-setup.service.d/storage.conf',
        '[Unit]\nRequires=carrot-protected-storage.service\nAfter=carrot-protected-storage.service\n'
        '[Service]\nExecStart=\nExecStart=/usr/bin/python3 /usr/lib/carrot-jetlink-storage/protected_first_boot.py\n')
  for name in ('carrot-jetlink', 'carrot-jetlink-hud', 'carrot-jetlink-update-apply', 'carrot-jetlink-update-stage'):
    write(root, f'/etc/systemd/system/{name}.service.d/storage.conf',
          '[Unit]\nRequires=carrot-protected-storage.service\nAfter=carrot-protected-storage.service\n'
          '[Service]\nEnvironment=PYTHONDONTWRITEBYTECODE=1\n')
  from install_wifi import configure as configure_wifi
  configure_wifi(root)
  write(root, '/etc/systemd/system/carrot-jetlink-wifi.service.d/recovery.conf',
        '[Unit]\nRequires=carrot-protected-storage.service\nAfter=carrot-protected-storage.service carrot-image-setup.service\n'
        '[Service]\nExecStart=\nExecStart=/usr/bin/python3 /usr/lib/carrot-jetlink-storage/wifi_apply.py\n')
  write(root, '/etc/NetworkManager/conf.d/carrot-retry.conf',
        '[connection]\nconnection.autoconnect-retries=0\n')
