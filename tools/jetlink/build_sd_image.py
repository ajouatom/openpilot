"""Stage a sanitized SD image on the reference Jetson; never write its physical disk.

This creates an UNFINISHED image. finalize_sd_image.py must provision and audit
the staged filesystem before it can be released or written to a spare SD card.
"""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

EXCLUDES = [
  '/home/***', '/root/***', '/proc/***', '/sys/***', '/dev/***', '/run/***',
  '/tmp/***', '/mnt/***', '/media/***', '/lost+found/***', '/boot/efi/***',
  '/swapfile', '/swap.img', '/usr/NX/***', '/var/NX/***', '/opt/ota_package/***',
  '/var/cache/***', '/var/log/***', '/var/tmp/***', '/var/crash/***', '/var/backups/***',
  '/var/spool/***', '/var/mail/***', '/var/lib/carrot-jetlink/***',
  '/var/lib/snapd/***', '/var/lib/docker/***', '/var/lib/containerd/***',
  '/var/lib/NetworkManager/***', '/var/lib/bluetooth/***', '/var/lib/private/***',
  '/var/lib/apt/lists/***', '/var/lib/systemd/random-seed', '/var/lib/dbus/machine-id',
  '/var/cudnn-local-*/***', '/var/l4t-cuda-*/***', '/var/nv-tensorrt-*/***',
  '/etc/machine-id', '/etc/hostname', '/etc/hosts', '/etc/passwd*', '/etc/shadow*',
  '/etc/group*', '/etc/gshadow*', '/etc/subuid*', '/etc/subgid*',
  '/etc/ssh/ssh_host_*', '/etc/sudoers.d/***', '/etc/ssl/private/***',
  '/etc/NetworkManager/system-connections/***', '/etc/netplan/***',
  '/etc/wpa_supplicant/***', '/etc/openvpn/***', '/etc/ipsec.secrets',
  '/etc/apt/auth.conf*', '/etc/krb5.keytab', '/etc/sssd/***', '/etc/nx*',
  '/etc/udev/rules.d/70-persistent-net.rules',
  *[f'/var/lib/{name}/***' for name in (
    'AccountsService', 'apport', 'avahi-autoipd', 'boltd', 'BrlAPI', 'colord', 'dhcp',
    'fprint', 'fwupd', 'geoclue', 'git', 'misc', 'openvpn', 'polkit-1', 'saned',
    'snmp', 'sss', 'sudo', 'tpm', 'ubiquity', 'ubuntu-advantage', 'udisks2', 'upower',
    'systemd/coredump', 'systemd/linger', 'systemd/timesync', 'systemd/timers',
    'systemd/pstore', 'systemd/rfkill')],
]


def run(*args, **kwargs):
  print('RUN', *args, flush=True)
  return subprocess.run(args, check=True, **kwargs)


def main():
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument('--work-dir', type=Path, required=True)
  p.add_argument('--gib', type=int, default=24)
  args = p.parse_args()
  assert os.geteuid() == 0 and Path('/etc/nv_tegra_release').is_file()
  work = args.work_dir.resolve()
  assert work == Path('/home/yun/jetlink-image-build')
  assert not work.exists(), 'Use a new staging directory; never overwrite an existing image'
  assert 24 <= args.gib <= 48
  source = '/dev/mmcblk0'
  layout = json.loads(subprocess.check_output(['sfdisk', '--json', source]))['partitiontable']
  assert layout['label'] == 'gpt' and layout['sectorsize'] == 512
  parts = {int(part['node'].rsplit('p', 1)[1]): part for part in layout['partitions']}
  assert set(parts) == set(range(1, 16)) and parts[1]['name'] == 'APP'
  root_source = subprocess.check_output(['findmnt', '-n', '-o', 'SOURCE', '/'], text=True).strip()
  assert root_source == '/dev/mmcblk0p1'
  locks = []
  for name in ['/var/lib/dpkg/lock-frontend', '/var/lib/dpkg/lock']:
    handle = open(name, 'a')
    fcntl.lockf(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    locks.append(handle)
  status_hash = hashlib.sha256(Path('/var/lib/dpkg/status').read_bytes()).hexdigest()
  work.mkdir(mode=0o700)
  (work/'source-layout.json').write_text(json.dumps(layout, indent=2))
  image = work/'carrot-jetson-STAGING.img'
  with image.open('xb') as output:
    output.truncate(args.gib * (1 << 30))
  assert image.is_file() and not image.is_symlink()
  run('sgdisk', '--clear', str(image))
  for number, part in parts.items():
    if number == 1:
      continue
    run('sgdisk', f'-n{number}:{part["start"]}:{part["start"]+part["size"]-1}',
        f'-t{number}:{part["type"]}', f'-c{number}:{part["name"]}', str(image))
  setup_start = parts[1]['start']
  setup_sectors = 64 * 2048
  run('sgdisk', f'-n16:{setup_start}:{setup_start+setup_sectors-1}', '-t16:0700', '-c16:CARROT_SETUP', str(image))
  run('sgdisk', f'-n1:{setup_start+setup_sectors}:{image.stat().st_size//512-2049}', '-t1:8300', '-c1:APP', str(image))
  loop = subprocess.check_output(['losetup', '--find', '--show', '--partscan', str(image)], text=True).strip()
  assert loop.startswith('/dev/loop')
  actual = subprocess.check_output(['losetup', '-n', '-O', 'BACK-FILE', loop], text=True).strip()
  assert Path(actual).resolve() == image
  root = work/'rootfs'; root.mkdir()
  setup = work/'setup'; setup.mkdir()
  mounted = []
  try:
    time.sleep(1)
    for number in [2, 3, 5, 6, 8, 9, 10, 11, 12, 13]:
      run('dd', f'if={source}p{number}', f'of={loop}p{number}', 'bs=4M', 'conv=fsync', 'status=none')
    run('mkfs.ext4', '-F', '-m', '0', '-L', 'CARROT_ROOT', loop+'p1')
    run('mkfs.vfat', '-F', '32', '-n', 'CARROTSETUP', loop+'p16')
    run('mount', loop+'p1', str(root)); mounted.append(root)
    run('mount', loop+'p16', str(setup)); mounted.append(setup)
    run('rsync', '-aHAXx', '--numeric-ids', '--info=stats2',
        *[f'--exclude={pattern}' for pattern in EXCLUDES], '/', str(root)+'/')
    assert hashlib.sha256(Path('/var/lib/dpkg/status').read_bytes()).hexdigest() == status_hash
    assert hashlib.sha256((root/'var/lib/dpkg/status').read_bytes()).hexdigest() == status_hash
    runtime = root/'opt/carrot-jetlink'; runtime.mkdir(parents=True, exist_ok=True)
    for name in ['venv', 'cache']:
      run('rsync', '-aH', '--numeric-ids', '--exclude=__pycache__', '--exclude=*.pyc',
          f'/home/yun/carrot-jetlink/{name}', str(runtime)+'/')
    (work/'stage.json').write_text(json.dumps({'state': 'UNFINISHED', 'source_dpkg_sha256': status_hash,
       'source_commit': Path('/home/yun/carrot-jetlink/carrot/SOURCE_COMMIT').read_text().strip(),
       'image_bytes': image.stat().st_size, 'root_start': setup_start+setup_sectors}, indent=2))
    run('sync')
    print('STAGING_COMPLETE', image, flush=True)
  finally:
    for mount in reversed(mounted):
      run('umount', str(mount))
    run('losetup', '-d', loop)


if __name__ == '__main__':
  main()
