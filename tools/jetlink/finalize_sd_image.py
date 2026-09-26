"""Finalize the file-backed staging image; never change the running Jetson root.

Requires a committed host bundle and zerofree. Output remains a candidate until
boot, provisioning, model and USB checks pass on the spare SD card.
"""
import argparse
import base64
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tarfile

WORK = Path('/home/yun/jetlink-image-build')
RUNTIME = '/opt/carrot-jetlink'
PRIVATE_STATE = ('AccountsService', 'apport', 'avahi-autoipd', 'bluetooth', 'boltd', 'BrlAPI',
                 'colord', 'dhcp', 'fprint', 'fwupd', 'geoclue', 'git', 'misc', 'NetworkManager',
                 'openvpn', 'polkit-1', 'private', 'saned', 'snmp', 'sss', 'sudo', 'tpm', 'ubiquity',
                 'ubuntu-advantage', 'udisks2', 'upower', 'systemd/coredump', 'systemd/linger',
                 'systemd/timesync', 'systemd/timers', 'systemd/pstore', 'systemd/rfkill')


def run(*args):
  print('RUN', *args, flush=True)
  return subprocess.run(args, check=True)


def write(root, name, content, mode=0o644):
  path = root/name.lstrip('/')
  if not path.resolve().is_relative_to(root.resolve()):
    raise ValueError('Image path escapes mount')
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(content)
  path.chmod(mode)


def clean_accounts(passwd_text, group_text):
  users = [line.split(':') for line in passwd_text.splitlines() if line]
  users = [entry for entry in users if int(entry[2]) < 1000 or entry[0] == 'nobody']
  users.append(['jetlink', 'x', '1000', '1000', 'Carrot Jetlink', '/home/jetlink', '/bin/bash'])
  names = {entry[0] for entry in users}
  groups = [line.split(':') for line in group_text.splitlines() if line]
  groups = [entry for entry in groups if int(entry[2]) < 1000 or entry[0] == 'nogroup']
  for entry in groups:
    entry[3] = ','.join(name for name in entry[3].split(',') if name in names)
    if entry[0] in {'video', 'render', 'plugdev'}:
      entry[3] = ','.join(filter(None, [entry[3], 'jetlink']))
  groups.append(['jetlink', 'x', '1000', ''])
  return ('\n'.join(':'.join(entry) for entry in users) + '\n',
          '\n'.join(':'.join(entry) for entry in groups) + '\n',
          '\n'.join(entry[0] + ':!:20000:0:99999:7:::' for entry in users) + '\n',
          '\n'.join(entry[0] + ':!::' + entry[3] for entry in groups) + '\n')


def extract_bundle(bundle, target):
  with tarfile.open(bundle, 'r:gz') as source:
    for member in source.getmembers():
      path = target/member.name
      if not path.resolve().is_relative_to(target.resolve()) or member.isdev() or member.isfifo() or member.islnk():
        raise ValueError('Unsafe host bundle member')
      if member.issym() and (Path(member.linkname).is_absolute() or not (path.parent/member.linkname).resolve().is_relative_to(target.resolve())):
        raise ValueError('Unsafe host bundle link')
    source.extractall(target)


def enable(root, name, body):
  write(root, '/etc/systemd/system/' + name, body)
  link = root/'etc/systemd/system/multi-user.target.wants'/name
  link.parent.mkdir(parents=True, exist_ok=True)
  link.unlink(missing_ok=True)
  link.symlink_to('../' + name)


def copy_user_dependencies(root):
  # Reference venv used --system-site-packages, which also exposed these four
  # packages from its owner's user-site directory. Copy only verified package
  # records, never the personal home or the unrelated torch/development stack.
  site = Path('/home/yun/.local/lib/python3.10/site-packages')
  wanted = {'typing-extensions': '4.15.0', 'cffi': '2.0.0', 'pycparser': '2.23', 'pycryptodome': '3.23.0'}
  found = {}
  destination = root/'opt/carrot-jetlink/venv/lib/python3.10/site-packages'
  for dist in importlib.metadata.distributions(path=[str(site)]):
    name = dist.metadata['Name'].lower().replace('_', '-')
    if name not in wanted:
      continue
    if dist.version != wanted[name] or dist.files is None:
      raise RuntimeError('Unreviewed reference dependency: ' + name)
    for entry in dist.files:
      source = Path(dist.locate_file(entry)).resolve()
      if '__pycache__' in source.parts or source.suffix == '.pyc':
        continue
      if not source.is_relative_to(site):
        raise RuntimeError('Package file escapes reviewed user-site directory')
      if entry.hash:
        digest = hashlib.new(entry.hash.mode, source.read_bytes()).digest()
        if base64.urlsafe_b64encode(digest).rstrip(b'=').decode() != entry.hash.value:
          raise RuntimeError('Reference package RECORD mismatch: ' + name)
      target = destination/source.relative_to(site)
      target.parent.mkdir(parents=True, exist_ok=True)
      shutil.copy2(source, target)
    found[name] = dist.version
  if found != wanted:
    raise RuntimeError('Reference user-site dependencies missing')
  write(root, '/etc/carrot-jetlink-user-dependencies.json', json.dumps(found, indent=2) + '\n')


def provision(root, setup, bundle, stage):
  for name in ['proc', 'sys', 'dev', 'run', 'tmp', 'var/tmp', 'var/log', 'var/cache', 'var/spool',
               'var/backups', 'var/crash', 'var/mail', 'var/spool/anacron', 'etc/openvpn',
               'boot/efi', 'mnt', 'media', 'root', 'home/jetlink',
               'etc/NetworkManager/system-connections', 'etc/ssl/private', 'var/lib/NetworkManager',
               'var/lib/carrot-jetlink', 'var/lib/apt/lists/partial']:
    (root/name).mkdir(parents=True, exist_ok=True)
  for name in ['tmp', 'var/tmp']:
    (root/name).chmod(0o1777)
  (root/'root').chmod(0o700)
  (root/'etc/ssl/private').chmod(0o700)
  os.chown(root/'home/jetlink', 1000, 1000)
  (root/'home/jetlink').chmod(0o750)
  for name in PRIVATE_STATE:
    path = root/'var/lib'/name
    if path.is_dir() and not path.is_symlink():
      shutil.rmtree(path)
  (root/'var/lib/NetworkManager').mkdir(mode=0o700, exist_ok=True)
  for name, content in zip(['passwd', 'group', 'shadow', 'gshadow'], clean_accounts(
      Path('/etc/passwd').read_text(), Path('/etc/group').read_text())):
    write(root, '/etc/' + name, content, 0o600 if 'shadow' in name else 0o644)
  write(root, '/etc/subuid', 'jetlink:100000:65536\n')
  write(root, '/etc/subgid', 'jetlink:100000:65536\n')
  write(root, '/etc/sudoers.d/carrot-admin', 'jetlink ALL=(ALL:ALL) NOPASSWD: ALL\n', 0o440)
  write(root, '/etc/hostname', 'carrot-jetson\n')
  write(root, '/etc/hosts', '127.0.0.1 localhost\n127.0.1.1 carrot-jetson\n::1 localhost\n')
  write(root, '/etc/machine-id', '')
  write(root, '/etc/fstab', '/dev/root / ext4 defaults 0 1\n/dev/mmcblk0p10 /boot/efi vfat defaults 0 1\n')
  delay = Path('/etc/systemd/system/systemd-update-utmp.service.d/zz-carrot-no-timestamp-delay.conf')
  if delay.exists():
    if delay.read_text() != '[Service]\nExecStartPre=\n':
      raise RuntimeError('Unexpected reference utmp override')
    write(root, str(delay), delay.read_text())
  write(root, '/etc/ssh/sshd_config.d/00-carrot-image.conf',
        'PasswordAuthentication no\nKbdInteractiveAuthentication no\nPermitRootLogin no\nPubkeyAuthentication yes\n')
  dbus = root/'var/lib/dbus/machine-id'
  dbus.parent.mkdir(parents=True, exist_ok=True)
  dbus.unlink(missing_ok=True)
  dbus.symlink_to('/etc/machine-id')
  for path in (root/'etc/apt/sources.list.d').glob('*.list'):
    if '-local' in path.name or path.name == 'docker.list':
      path.unlink()
  # Remove dead desktop/remote-login references, not NVIDIA driver or power units.
  for name in ['display-manager.service', 'nxserver.service', 'nxserver.service.d', 'gdm.service.d']:
    path = root/'etc/systemd/system'/name
    if path.is_dir() and not path.is_symlink():
      shutil.rmtree(path)
    else:
      path.unlink(missing_ok=True)
  for path in (root/'etc/systemd/system').rglob('*'):
    if path.is_symlink() and ('nxserver' in path.name or 'gdm' in path.name):
      path.unlink()
  runtime = root/RUNTIME.lstrip('/')
  releases = runtime/'releases'; releases.mkdir(exist_ok=True)
  temporary = releases/'staging'
  if temporary.exists():
    shutil.rmtree(temporary)
  temporary.mkdir()
  extract_bundle(bundle, temporary)
  commit = (temporary/'SOURCE_COMMIT').read_text().strip()
  if not re.fullmatch('[0-9a-f]{40}', commit):
    raise ValueError('Invalid committed source identity')
  release = releases/commit
  if release.exists():
    shutil.rmtree(release)
  temporary.rename(release)
  for name in ['current', 'carrot']:
    (runtime/name).unlink(missing_ok=True)
  (runtime/'current').symlink_to('releases/' + commit)
  # Keep the compatibility alias for existing static inventory tools.
  (runtime/'carrot').symlink_to('current')
  for previous in releases.iterdir():
    if previous != release:
      shutil.rmtree(previous)
  copy_user_dependencies(root)
  for path in (runtime/'venv/bin').iterdir():
    if path.is_file() and not path.is_symlink() and path.stat().st_size < 1 << 20:
      data = path.read_bytes()
      if b'\0' not in data:
        path.write_bytes(data.replace(b'/home/yun/carrot-jetlink/venv', b'/opt/carrot-jetlink/venv'))
  for path in (root/'etc/systemd/system').rglob('*'):
    if path.is_file() and not path.is_symlink():
      data = path.read_text()
      updated = data.replace('/home/yun/carrot-jetlink/carrot', RUNTIME + '/current')
      updated = updated.replace('/home/yun/carrot-jetlink', RUNTIME)
      updated = updated.replace('User=yun', 'User=jetlink').replace('xorg_auth.py yun', 'xorg_auth.py jetlink')
      updated = updated.replace(RUNTIME + '/control.sock', '/run/carrot-jetlink/control.sock')
      if path.name == 'carrot-jetlink.service':
        updated = updated.replace('[Service]', '[Service]\nRuntimeDirectory=carrot-jetlink\nRuntimeDirectoryMode=0700')
      if updated != data:
        path.write_text(updated)
  run('chown', '-R', '1000:1000', str(runtime/'cache'))
  # The vendor display library opens log.log on import. The immutable release
  # must remain root-owned; systemd creates this writable directory as jetlink.
  write(root, '/etc/systemd/system/carrot-jetlink-hud.service.d/log-directory.conf',
        '[Service]\nLogsDirectory=carrot-jetlink-hud\nLogsDirectoryMode=0700\n'
        'WorkingDirectory=/var/log/carrot-jetlink-hud\n')
  script = RUNTIME + '/current/tools/jetlink/'
  enable(root, 'carrot-image-setup.service',
         '[Unit]\nDescription=Carrot per-device SD provisioning\nAfter=local-fs.target\nBefore=ssh.service NetworkManager.service\n'
         '[Service]\nType=oneshot\nExecStart=/usr/bin/python3 ' + script + 'image_first_boot.py\nRemainAfterExit=yes\n'
         '[Install]\nWantedBy=multi-user.target\n')
  enable(root, 'carrot-image-grow.service',
         '[Unit]\nDescription=Expand Carrot SD root filesystem once\nAfter=local-fs.target\n'
         'ConditionPathExists=!/var/lib/carrot-jetlink/root-expanded.json\n'
         '[Service]\nType=oneshot\nExecStart=/usr/bin/python3 ' + script + 'image_grow_root.py\n'
         'Nice=19\nIOSchedulingClass=idle\nTimeoutStartSec=180\n[Install]\nWantedBy=multi-user.target\n')
  marker = {'format': 1, 'state': 'CANDIDATE_UNTESTED_SD', 'source_commit': commit,
            'l4t': Path('/etc/nv_tegra_release').read_text().strip(), 'root_start': stage['root_start'],
            'runtime': RUNTIME, 'hardware': 'Jetson Orin Nano Super developer kit; matching QSPI firmware required'}
  write(root, '/etc/carrot-jetlink-image.json', json.dumps(marker, indent=2) + '\n')
  (setup/'IMAGE.json').write_text(json.dumps(marker, indent=2) + '\n')
  (setup/'setup.example.json').write_text(json.dumps({'hostname': 'carrot-jetson', 'ssh_public_keys': ['REPLACE_WITH_YOUR_SSH_PUBLIC_KEY'],
       'wifi': {'ssid': 'YOUR_SSID', 'password': 'YOUR_WIFI_PASSWORD'}}, indent=2) + '\n')
  (setup/'README.txt').write_text('Carrot Jetson candidate SD image.\nCopy setup.example.json to setup.json and configure your own SSH public key.\n'
     'Remove the wifi object to use Ethernet DHCP. Login: jetlink; no default password.\n'
     'setup.json is consumed at first boot. Never redistribute a provisioned SD card.\n'
     'Requires matching Jetson Orin Nano Super QSPI firmware / L4T 36.4.7.\n'
     'First boot grows APP; allow extra time. Routine boots do not download or compile models.\n')
  # Prevent unattended image growth/provisioning from being mistaken for a hardware test.
  for pattern in ['home/yun', 'etc/ssh/ssh_host_*', 'etc/NetworkManager/system-connections/*', 'home/*/.ssh/*']:
    if list(root.glob(pattern)):
      raise RuntimeError('Private state remains: ' + pattern)
  if (root/'etc/machine-id').read_bytes():
    raise RuntimeError('Image machine ID must be empty')
  return marker


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--bundle', type=Path, required=True)
  parser.add_argument('--sha256', required=True)
  parser.add_argument('--zerofree', type=Path, required=True)
  parser.add_argument('--refresh-source', action='store_true', help='Rebuild an unprovisioned local candidate with a newer committed bundle')
  args = parser.parse_args()
  if os.geteuid() != 0 or not Path('/etc/nv_tegra_release').is_file():
    raise RuntimeError('Run on the reference Jetson as root')
  if hashlib.sha256(args.bundle.read_bytes()).hexdigest() != args.sha256:
    raise RuntimeError('Host bundle checksum mismatch')
  stage = json.loads((WORK/'stage.json').read_text())
  if args.refresh_source and stage['state'] == 'FINALIZED_CANDIDATE':
    candidate = WORK/'carrot-jetson-candidate.img'
    if candidate.is_symlink() or candidate.stat().st_size != stage['image_bytes'] or (WORK/'carrot-jetson-STAGING.img').exists():
      raise RuntimeError('Unexpected local candidate file')
    candidate.rename(WORK/'carrot-jetson-STAGING.img')
    stage['state'] = 'UNFINISHED'
    (WORK/'stage.json').write_text(json.dumps(stage, indent=2) + '\n')
  if stage['state'] != 'UNFINISHED':
    raise RuntimeError('Expected unfinished staging image')
  image = WORK/'carrot-jetson-STAGING.img'
  if image.is_symlink() or image.stat().st_size != stage['image_bytes']:
    raise RuntimeError('Invalid staging image')
  loop = subprocess.check_output(['losetup', '--find', '--show', '--partscan', str(image)], text=True).strip()
  backing = subprocess.check_output(['losetup', '-n', '-O', 'BACK-FILE', loop], text=True).strip()
  if Path(backing).resolve() != image:
    raise RuntimeError('Unexpected loop backing file')
  root = WORK/'rootfs'; setup = WORK/'setup'
  mounted = []
  try:
    run('mount', loop+'p1', str(root)); mounted.append(root)
    run('mount', loop+'p16', str(setup)); mounted.append(setup)
    if (root/'var/lib/carrot-jetlink/provisioned.json').exists() or (setup/'setup.json').exists():
      raise RuntimeError('Never refinalize or distribute a personally provisioned image')
    marker = provision(root, setup, args.bundle, stage)
    # Minimal device nodes for offline Python/library checks; no live GPU/USB is mounted.
    for name, minor in [('null', 3), ('zero', 5), ('random', 8), ('urandom', 9)]:
      path = root/'dev'/name
      if not path.exists():
        os.mknod(path, 0o020666, os.makedev(1, minor))
    run('chroot', str(root), '/usr/sbin/visudo', '-cf', '/etc/sudoers')
    run('chroot', str(root), '/opt/carrot-jetlink/venv/bin/python', '-c',
        'import numpy, onnx, usb1, tensorrt; print("IMAGE_IMPORTS_OK", tensorrt.__version__)')
    run('sync')
    for path in reversed(mounted):
      run('umount', str(path))
    mounted.clear()
    run('e2fsck', '-f', '-p', loop+'p1')
    # Clear discarded staging state from free ext4 blocks before distribution.
    run(str(args.zerofree), loop+'p1')
    run('fsck.vfat', '-n', loop+'p16')
    marker['image_bytes'] = image.stat().st_size
    stage['state'] = 'FINALIZED_CANDIDATE'
    (WORK/'stage.json').write_text(json.dumps(stage, indent=2) + '\n')
    (WORK/'candidate.json').write_text(json.dumps(marker, indent=2) + '\n')
  finally:
    for path in reversed(mounted):
      run('umount', str(path))
    run('losetup', '-d', loop)
  image.rename(WORK/'carrot-jetson-candidate.img')
  print('CANDIDATE_READY_FOR_PHYSICAL_SD_TEST', flush=True)


if __name__ == '__main__':
  main()
