"""Install NVIDIA's pinned Super capsule and early-boot provisioning offline."""
import hashlib
from pathlib import Path
import shutil
import subprocess
import urllib.request

from super_bootstrap import CAPSULE_SHA

PACKAGE_URL = ('https://repo.download.nvidia.com/jetson/t234/pool/main/n/nvidia-l4t-bootloader/' +
               'nvidia-l4t-bootloader_36.4.7-20250918154033_arm64.deb')
PACKAGE_SHA = 'b7600d3c65be858e8de2b46764221e128f472480fe801b411072d874326adfc2'


def configure(root, source, work):
  root, source, work = Path(root), Path(source), Path(work)
  if root.resolve() == Path('/') or not (root / 'etc/carrot-jetlink-protected.json').is_file():
    raise ValueError('Only a new offline protected SD image is supported')
  package = work / 'nvidia-bootloader.deb'
  urllib.request.urlretrieve(PACKAGE_URL, package)
  if hashlib.sha256(package.read_bytes()).hexdigest() != PACKAGE_SHA:
    raise ValueError('NVIDIA bootloader package hash mismatch')
  extracted = work / 'nvidia-bootloader'
  subprocess.run(['dpkg-deb', '-x', str(package), str(extracted)], check=True)
  capsule = extracted / 'opt/ota_package/t23x/TEGRA_BL_3767_super.Cap'
  if hashlib.sha256(capsule.read_bytes()).hexdigest() != CAPSULE_SHA:
    raise ValueError('NVIDIA Super capsule hash mismatch')
  destination = root / 'usr/lib/carrot-jetlink-firmware'
  destination.mkdir(parents=True, exist_ok=True)
  for file in (capsule, source / 'super_bootstrap.py', source / 'power_setup.py'):
    shutil.copyfile(file, destination / file.name)
  shutil.copytree(extracted / 'usr/share/doc/nvidia-l4t-bootloader',
                  destination / 'nvidia-license', dirs_exist_ok=True)
  units = root / 'etc/systemd/system'
  vendor_unit = units / 'nv-l4t-bootloader-config.service'
  vendor_unit.unlink(missing_ok=True)
  shutil.copyfile(root / 'opt/nvidia/l4t-bootloader-config/nv-l4t-bootloader-config.service', vendor_unit)
  # NVIDIA's normal service may independently stage a version update. Retain
  # its board/ESP discovery and boot-slot verification, and serialize capsule
  # selection through our persistent transaction before GPU consumers start.
  vendor = units / 'nv-l4t-bootloader-config.service.d/carrot-super.conf'
  vendor.parent.mkdir(parents=True, exist_ok=True)
  vendor.write_text('''[Unit]
Requires=carrot-protected-storage.service
After=carrot-protected-storage.service
[Service]
ExecStart=
ExecStart=/opt/nvidia/l4t-bootloader-config/nv-l4t-bootloader-config.sh -l
ExecStart=/usr/sbin/nvbootctrl verify
''')
  name = 'carrot-super-firmware.service'
  (units / name).write_text('''[Unit]
Description=Provision and verify NVIDIA Super firmware from the boot SD
Requires=carrot-protected-storage.service nv-l4t-bootloader-config.service nvpmodel.service
After=carrot-protected-storage.service nv-l4t-bootloader-config.service nvpmodel.service
Before=carrot-jetlink-performance.service carrot-jetlink.service carrot-jetlink-xorg.service carrot-jetlink-hud.service carrot-jetlink-update-apply.service
[Service]
Type=oneshot
ExecStart=/usr/bin/python3 /usr/lib/carrot-jetlink-firmware/super_bootstrap.py
RemainAfterExit=yes
TimeoutStartSec=240
[Install]
WantedBy=multi-user.target
''')
  link = units / 'multi-user.target.wants' / name
  link.parent.mkdir(parents=True, exist_ok=True)
  link.unlink(missing_ok=True)
  link.symlink_to('../' + name)
  for consumer in ('carrot-jetlink-performance', 'carrot-jetlink', 'carrot-jetlink-xorg',
                   'carrot-jetlink-hud', 'carrot-jetlink-update-apply'):
    dropin = units / (consumer + '.service.d/super-firmware.conf')
    dropin.parent.mkdir(parents=True, exist_ok=True)
    dropin.write_text('[Unit]\nRequires=carrot-super-firmware.service\nAfter=carrot-super-firmware.service\n')
