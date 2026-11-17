"""First-boot Super firmware provisioning for the protected Orin Nano SD/NVMe image."""
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

sys.path.insert(0, '/usr/lib/carrot-jetlink-storage')

VERSION = (36, 4, 7)
CAPSULE = Path('/usr/lib/carrot-jetlink-firmware/TEGRA_BL_3767_super.Cap')
CAPSULE_SHA = 'fc07e537329561d520052d2e45a09c516ed430369078224971ed70f5bbe1ec2f'
STATUS = Path('/run/carrot-super-firmware.json')
STATE = Path('/run/carrot-data/super-firmware.json')
EFI = Path('/boot/efi')


def run(*args):
  return subprocess.run(args, check=True, capture_output=True, text=True, timeout=120).stdout.strip()


def atomic(path, value):
  temporary = path.with_suffix('.new')
  with temporary.open('w') as stream:
    json.dump(value, stream)
    stream.flush()
    os.fsync(stream.fileno())
  temporary.chmod(0o644)
  temporary.replace(path)
  fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
  try:
    os.fsync(fd)
  finally:
    os.close(fd)


def read_state(path):
  if not path.exists():
    return {}
  value = json.loads(path.read_text())
  if not isinstance(value, dict):
    raise ValueError('Invalid firmware transaction record')
  return value


def slots(text):
  version = re.search(r'^Current version:\s*(\d+)\.(\d+)\.(\d+)\s*$', text, re.M)
  slot = re.search(r'^Current bootloader slot:\s*([AB])\s*$', text, re.M)
  status = re.search(r'^Capsule update status:\s*([0-3])\s*$', text, re.M)
  if not all((version, slot, status)):
    raise ValueError('Cannot read NVIDIA bootloader status')
  return {'version': tuple(map(int, version.groups())), 'slot': slot[1], 'capsule_status': int(status[1])}


def decision(info, previous):
  """Only a completed subsequent boot can acknowledge a staged capsule."""
  if not info['supported']:
    return 'unsupported_board'
  if info['version'][0] != 36 or info['version'] > VERSION:
    return 'firmware_version_unsupported'
  if previous.get('device') == info['device'] and previous.get('capsule_sha256') == CAPSULE_SHA:
    if previous.get('state') in ('staging', 'reboot_pending'):
      if previous.get('boot_id') == info['boot_id']:
        return 'reboot_pending' if previous['state'] == 'reboot_pending' else 'interrupted_staging'
      if (info['capsule_status'] != 1 or info['slot'] == previous.get('slot')
          or info['version'] != VERSION or not info['super']):
        return 'firmware_update_failed'
      return 'firmware_ready'
    if previous.get('state') == 'firmware_ready' and not info['super']:
      return 'firmware_verification_failed'
  if info['version'] == VERSION and info['super']:
    return 'firmware_ready'
  return 'stage_capsule'


def inspect(root=Path('/')):
  compatible = (root / 'proc/device-tree/compatible').read_bytes().split(b'\0')
  supported = any(value in compatible for value in (
    b'nvidia,p3768-0000+p3767-0005', b'nvidia,p3768-0000+p3767-0005-super'))
  serial = (root / 'proc/device-tree/serial-number').read_bytes().rstrip(b'\0')
  if not serial:
    raise ValueError('Missing board identity')
  release = (root / 'etc/nv_tegra_release').read_text()
  if not re.search(r'# R36 \(release\), REVISION: 4\.7(?:,|\s|$)', release):
    raise ValueError('Image must use L4T 36.4.7')
  return {**slots(run('/usr/sbin/nvbootctrl', 'dump-slots-info')),
          'supported': supported, 'super': b'nvidia,p3768-0000+p3767-0005-super' in compatible,
          'device': hashlib.sha256(serial).hexdigest(),
          'boot_id': (root / 'proc/sys/kernel/random/boot_id').read_text().strip()}


def validate_storage():
  # Bind firmware writes and transaction state to the disk which booted Linux.
  from protected_storage import configuration, portable_configuration
  layout = configuration()
  actual = portable_configuration()
  expected = actual['efi']
  if (layout['root'] != actual['root'] or layout['data'] != actual['data']
      or Path(run('findmnt', '-n', '-o', 'SOURCE', str(EFI))).resolve() != Path(expected).resolve()
      or Path(run('findmnt', '-n', '-o', 'SOURCE', str(STATE.parent))).resolve() != Path(layout['data']).resolve()
      or run('blkid', '-s', 'TYPE', '-o', 'value', expected) != 'vfat'):
    raise ValueError('Boot SD/NVMe, EFI and DATA partitions do not match')
  if not Path('/sys/firmware/efi/efivars').is_dir():
    raise ValueError('UEFI runtime services are unavailable')
  control = Path('/etc/nv_boot_control.conf').read_text()
  # NVIDIA declares TEGRA_BOOT_STORAGE deprecated. Its helper leaves the image's
  # old value untouched when EFI is already mounted. Use the mounted root and
  # same-disk EFI/DATA above, retaining the vendor-generated board contract.
  if not re.search(r'^COMPATIBLE_SPEC 3767--0005--1--jetson-orin-nano-devkit-super-\s*$', control, re.M):
    raise ValueError('NVIDIA board specification does not match the Super image')


def publish(value):
  atomic(STATUS, value)
  print(json.dumps({'event': 'jetsonSuperFirmware', **value}), flush=True)


def stage_capsule(info):
  if hashlib.sha256(CAPSULE.read_bytes()).hexdigest() != CAPSULE_SHA:
    raise ValueError('NVIDIA capsule hash mismatch')
  pending = EFI / 'EFI/UpdateCapsule'
  if pending.exists() and any(pending.iterdir()):
    raise ValueError('Another firmware capsule is already pending')
  value = {**info, 'capsule_sha256': CAPSULE_SHA, 'state': 'staging'}
  atomic(STATE, value)
  publish(value)
  # Keep APP read-only. NVIDIA's helper uses the boot card's small EFI volume.
  run('mount', '-o', 'remount,rw', str(EFI))
  try:
    run('/usr/sbin/nv_bootloader_capsule_updater.sh', '-q', str(CAPSULE))
    staged = pending / CAPSULE.name
    if hashlib.sha256(staged.read_bytes()).hexdigest() != CAPSULE_SHA:
      raise ValueError('Staged capsule readback mismatch')
    indication = Path('/sys/firmware/efi/efivars/OsIndications-8be4df61-93ca-11d2-aa0d-00e098032b8c').read_bytes()
    if len(indication) != 12 or not int.from_bytes(indication[4:], 'little') & 4:
      raise ValueError('Capsule reboot request was not recorded')
    run('sync', '-f', str(EFI))
  finally:
    run('mount', '-o', 'remount,ro', str(EFI))
  value['state'] = 'reboot_pending'
  atomic(STATE, value)
  publish(value)


def main():
  import fcntl
  if os.geteuid() != 0:
    raise PermissionError('Image firmware provisioning requires root')
  with Path('/run/carrot-super-firmware.lock').open('w') as lock:
    fcntl.flock(lock, fcntl.LOCK_EX)
    try:
      if not Path('/etc/carrot-jetlink-image.json').is_file():
        raise ValueError('Expected a Carrot SD image')
      info = inspect()
      validate_storage()
      previous = read_state(STATE)
      action = decision(info, previous)
      publish({**info, 'state': action})
      if action == 'firmware_ready':
        from power_setup import run_once
        power = run_once()
        if power.get('state') == 'ready':
          atomic(STATE, {**info, 'state': action, 'capsule_sha256': CAPSULE_SHA})
          publish({**info, 'state': 'ready', 'power': power})
          return 0
        if power.get('state') == 'firmware_clock_limit' and not previous:
          action = 'stage_capsule'
        else:
          publish({**info, 'state': 'power_verification_failed', 'power': power})
          return 1
      if action == 'stage_capsule':
        stage_capsule(info)
        action = 'reboot_pending'
      if action == 'reboot_pending':
        # This image service runs before inference or HUD startup. Let the
        # independent CPU boot screen explain the automatic firmware restart.
        time.sleep(10)
        run('systemctl', '--no-block', 'reboot')
      return 1
    except (OSError, ValueError, subprocess.SubprocessError) as error:
      publish({'state': 'setup_failed', 'error': str(error)[:240]})
      return 1


if __name__ == '__main__':
  raise SystemExit(main())
