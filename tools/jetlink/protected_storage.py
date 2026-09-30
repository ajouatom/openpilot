"""Early boot and identity persistence for *separately partitioned* SD images.

Never remount the system writable, format a device, or repartition a live card.
The immutable base runtime remains a recovery fallback if DATA cannot be used.
This helper is installed in the base OS, independently of updateable releases.
"""
import argparse
from contextlib import contextmanager
import json
import os
from pathlib import Path
import subprocess
import re

from persistent_state import collect, durable_write, latest, restore, save
from boot_status import stage, read_json, STAGE, persist_result

MARKER = Path('/etc/carrot-jetlink-protected.json')
DATA = Path('/run/carrot-data')
SETUP = Path('/run/carrot-identity-backup')
STATUS = Path('/run/carrot-storage.json')
RUNTIME = Path('/opt/carrot-jetlink')


def run(*args, timeout=30):
  return subprocess.run(args, check=True, capture_output=True, text=True, timeout=timeout).stdout.strip()


@contextmanager
def locked():
  import fcntl
  with Path('/run/carrot-storage.lock').open('a') as lock:
    fcntl.flock(lock, fcntl.LOCK_EX)
    yield


def configuration():
  value = json.loads(MARKER.read_text())
  if value == {'format': 2, 'layout': 'carrot-r2-sd-nvme'}:
    return portable_configuration()
  if value != {'format': 1, 'root': '/dev/mmcblk0p1', 'data': '/dev/mmcblk0p17', 'setup': '/dev/mmcblk0p16'}:
    raise ValueError('Unsupported protected SD layout')
  return value


def portable_configuration(sys_root=Path('/sys/dev/block')):
  # Resolve the mounted root through the kernel, never through a global label
  # search: an attached clone must not donate its DATA or identity partition.
  number = run('findmnt', '-n', '-o', 'MAJ:MIN', '/')
  if not re.fullmatch(r'[0-9]+:[0-9]+', number):
    raise ValueError('Root is not a block filesystem')
  partition = (sys_root / number).resolve(strict=True)
  disk = partition.parent.name
  if (not re.fullmatch(r'mmcblk[0-9]+|nvme[0-9]+n[0-9]+', disk)
      or partition.name != disk + 'p1' or (partition / 'partition').read_text().strip() != '1'):
    raise ValueError('Expected SD or NVMe APP partition 1')
  root = '/dev/' + partition.name
  if run('blkid', '-s', 'PARTUUID', '-o', 'value', root) != 'd3fb8cf2-60ea-44b1-bec1-63a7417719cf':
    raise ValueError('Unexpected portable APP identity')
  return dict(format=2, root=root, data=f'/dev/{disk}p17', setup=f'/dev/{disk}p16', efi=f'/dev/{disk}p10')


@contextmanager
def setup_mount(writable=False):
  config = configuration()
  SETUP.mkdir(mode=0o700, exist_ok=True)
  mounted = False
  try:
    options = ('rw' if writable else 'ro') + ',nosuid,nodev,noexec,umask=077'
    if config['format'] == 2:
      validate_setup(config)
    run('mount', '-t', 'vfat', '-o', options, config['setup'], str(SETUP))
    mounted = True
    yield SETUP / 'identity'
  except (OSError, RuntimeError, subprocess.SubprocessError):
    if mounted:
      raise
    yield None
  finally:
    if mounted:
      run('umount', str(SETUP))


def validate_setup(config):
  if (run('blkid', '-s', 'LABEL', '-o', 'value', config['setup']) != 'CARROTSETUP'
      or run('blkid', '-s', 'TYPE', '-o', 'value', config['setup']) != 'vfat'):
    raise RuntimeError('Unexpected SETUP partition')


def data_identity():
  return [DATA / 'identity'] if os.path.ismount(DATA) else []


def persist():
  configuration()
  with locked(), setup_mount(writable=True) as backup:
    directories = data_identity() + ([backup] if backup is not None else [])
    save(directories, collect())


def valid_runtime(path):
  """Do not execute a half-copied DATA runtime; immutable base remains usable."""
  try:
    marker = json.loads((path / 'protected-runtime.json').read_text())
    target = (path / 'current').readlink()
    # Existing updater versions persist an absolute /opt/... symlink. Before
    # the DATA bind mount, resolving that literally would inspect the factory
    # APP tree and incorrectly reject every newly activated release.
    if target.is_absolute():
      target = target.relative_to(RUNTIME)
    current = (path / target).resolve(strict=True)
    releases = (path / 'releases').resolve(strict=True)
    return (marker == {'format': 1} and current.is_relative_to(releases)
            and (current / 'tools/jetlink/server.py').is_file()
            and (current / 'SOURCE_COMMIT').is_file()
            and (path / 'venv/bin/python').is_file()
            and (path / 'cache/last-loaded.json').is_file())
  except (OSError, ValueError, RuntimeError):
    return False


def mount_volatile(root, ram, progress=lambda name: None):
  """Mount only RAM upper layers; also exercised against a real Linux loop FS."""
  ram.mkdir(mode=0o700, exist_ok=True)
  run('mount', '-t', 'tmpfs', '-o', 'mode=0700,size=768M,nosuid,nodev', 'tmpfs', str(ram))
  # Preserve PID1's already selected per-boot ID; never replace it with a
  # different identity after systemd has started. SSH keys/hostname are durable.
  machine = (root / 'etc/machine-id').read_bytes()
  names = ['etc', 'var', 'home', 'root']
  # NVIDIA's device-mode helper rewrites its 16MiB USB identification image
  # and derives per-board MACs next to its scripts. Keep those writes in RAM.
  if (root / 'opt/nvidia/l4t-usb-device-mode').is_dir():
    names.append('opt/nvidia/l4t-usb-device-mode')
  for name in names:
    progress('ram-' + name)
    key = name.replace('/', '-')
    upper, work = ram / key, ram / (key + '-work')
    target = root / name
    upper.mkdir(mode=target.stat().st_mode & 0o7777); work.mkdir()
    run('mount', '-t', 'overlay', '-o', f'lowerdir={target},upperdir={upper},workdir={work}', 'overlay', str(target))
  durable_write(root / 'etc/machine-id', machine, 0o444)
  run('mount', '-t', 'tmpfs', '-o', 'mode=1777,size=256M,nosuid,nodev', 'tmpfs', str(root / 'tmp'))
  run('mount', '-t', 'tmpfs', '-o', 'mode=0755,size=64M,nosuid,nodev,noexec', 'tmpfs', str(root / 'var/log'))
  # NVIDIA USB-device-mode creates a temporary loop mount below /mnt.
  if (root / 'mnt').is_dir():
    run('mount', '-t', 'tmpfs', '-o', 'mode=0755,size=16M,nosuid,nodev', 'tmpfs', str(root / 'mnt'))
  # Keep PVA authentication enabled; only its regenerated output is volatile.
  allowlist = root / 'lib/firmware/pva_auth_allowlist'
  if allowlist.is_file():
    output = ram / 'pva_auth_allowlist'
    output.write_bytes(allowlist.read_bytes())
    output.chmod(0o644)
    run('mount', '--bind', str(output), str(allowlist))


def boot():
  stage('root-readonly-check')
  config = configuration()
  # Fail closed on a wrong/legacy layout. Enabling this service alone on an old
  # card is deliberately unsupported; the offline builder creates partition 17.
  root_source = run('findmnt', '-n', '-o', 'SOURCE', '/')
  options = run('findmnt', '-n', '-o', 'OPTIONS', '/').split(',')
  if root_source != config['root'] or 'ro' not in options:
    raise RuntimeError('Protected boot requires the expected read-only root')
  run('blockdev', '--setro', config['root'])
  if config['format'] == 2:
    # local-fs-pre ordering creates this before fstab's optional read-only EFI
    # mount. No filesystem lookup on another disk and no persistent OS write.
    efi = Path('/run/carrot-efi')
    efi.unlink(missing_ok=True)
    efi.symlink_to(config['efi'])
  mount_volatile(Path('/'), Path('/run/carrot-volatile'), stage)
  DATA.mkdir(mode=0o700, exist_ok=True)
  state = 'base-recovery'
  stage('data-check')
  try:
    # Never fsck a mounted device. No -y, format, or automatic destructive reset.
    mounted = subprocess.run(['findmnt', '-n', '-S', config['data']], capture_output=True, timeout=5)
    if mounted.returncode != 1:
      raise RuntimeError('Unexpected existing DATA mount')
    if run('blkid', '-s', 'LABEL', '-o', 'value', config['data']) != 'CARROTDATA':
      raise RuntimeError('Unexpected DATA partition')
    check = subprocess.run(['e2fsck', '-p', config['data']], capture_output=True, timeout=60)
    if check.returncode not in (0, 1):
      raise RuntimeError('DATA requires offline repair')
    run('mount', '-t', 'ext4', '-o', 'rw,nosuid,nodev,noatime', config['data'], str(DATA))
    if not valid_runtime(DATA / 'runtime'):
      raise RuntimeError('DATA runtime incomplete')
    run('mount', '--bind', str(DATA / 'runtime'), str(RUNTIME))
    state = 'protected'
  except (OSError, ValueError, RuntimeError, subprocess.SubprocessError):
    pass  # Known baseline model + recovery network; never boot DATA code blindly.
  stage('restore-identity')
  with setup_mount() as backup:
    record = latest(data_identity() + ([backup] if backup is not None else []))
    if record is not None:
      restore(Path('/'), record)
  hostname = Path('/etc/hostname').read_text().strip()
  if re.fullmatch(r'[a-z][a-z0-9-]{0,61}[a-z0-9]|[a-z]', hostname):
    run('hostname', hostname)
  durable_write(STATUS, json.dumps({'format': 1, 'state': state, 'system_read_only': True}).encode(), 0o644)
  stage(state)


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('action', choices=('boot', 'save'))
  args = parser.parse_args()
  if os.geteuid() != 0:
    raise PermissionError('Root service required')
  if args.action == 'boot':
    with locked():
      try:
        boot()
      except Exception as error:
        previous = read_json(STAGE).get('stage', 'storage-start')
        stage(previous, error)
        raise
      finally:
        # At most one tiny result per attempted storage boot, never a log stream.
        # Preserve the original boot failure if the setup medium is unavailable.
        try:
          with setup_mount(writable=True) as backup:
            if backup is not None:
              persist_result(backup.parent, read_json(STAGE))
        except Exception:
          pass
  else:
    persist()


if __name__ == '__main__':
  main()
