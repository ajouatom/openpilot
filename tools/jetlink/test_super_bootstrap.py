import hashlib
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import super_bootstrap as boot
from boot_display import lines


def info(**changes):
  return dict(version=(36, 4, 3), slot='A', capsule_status=0, supported=True,
              super=False, device='board1', boot_id='boot1', **changes)


def test_slot_status_parser_matches_vendor_format():
  assert boot.slots('Current version: 36.4.7\nCapsule update status: 1\n' +
                    'Current bootloader slot: B\nActive bootloader slot: B\n') == {
                      'version': (36, 4, 7), 'slot': 'B', 'capsule_status': 1}
  with pytest.raises(ValueError):
    boot.slots('Current version: 36.4.7\n')


@pytest.mark.parametrize('version,super_mode,expected', [
  ((36, 3, 0), False, 'stage_capsule'),
  ((36, 4, 7), False, 'stage_capsule'),
  ((36, 4, 7), True, 'firmware_ready'),
  ((35, 5, 0), False, 'firmware_version_unsupported'),
  ((36, 5, 0), True, 'firmware_version_unsupported'),
  ((39, 2, 0), True, 'firmware_version_unsupported'),
])
def test_version_and_variant(version, super_mode, expected):
  current = info()
  current.update(version=version, super=super_mode)
  assert boot.decision(current, {}) == expected


def pending(state='reboot_pending'):
  return {**info(), 'state': state, 'capsule_sha256': boot.CAPSULE_SHA}


def test_same_boot_restart_never_stages_twice():
  assert boot.decision(info(), pending()) == 'reboot_pending'
  assert boot.decision(info(), pending('staging')) == 'interrupted_staging'


@pytest.mark.parametrize('changes', [
  {}, {'capsule_status': 2}, {'capsule_status': 3}, {'slot': 'A'},
  {'super': False}, {'version': (36, 4, 3)},
])
def test_failed_or_incomplete_next_boot_never_retries_forever(changes):
  current = {**info(), 'boot_id': 'boot2'}
  if changes:
    current.update(version=(36, 4, 7), slot='B', capsule_status=1, super=True)
    current.update(changes)
  assert boot.decision(current, pending()) == 'firmware_update_failed'


def test_success_requires_updated_slot_version_status_and_super():
  current = {**info(), 'boot_id': 'boot2', 'version': (36, 4, 7),
             'slot': 'B', 'capsule_status': 1, 'super': True}
  assert boot.decision(current, pending()) == 'firmware_ready'


def test_state_is_bound_to_recipient_board():
  current = {**info(), 'device': 'board2'}
  assert boot.decision(current, pending()) == 'stage_capsule'
  assert boot.decision({**current, 'supported': False}, {}) == 'unsupported_board'


@pytest.mark.parametrize('disk', ['mmcblk0', 'mmcblk1', 'nvme0n1', 'nvme1n1'])
@pytest.mark.parametrize('mismatch', [None, 'efi', 'data', 'root', 'board'])
def test_firmware_uses_actual_boot_disk_with_stale_vendor_storage(tmp_path, monkeypatch, disk, mismatch):
  import protected_storage as storage
  layout = dict(format=2, root=f'/dev/{disk}p1', data=f'/dev/{disk}p17',
                setup=f'/dev/{disk}p16', efi=f'/dev/{disk}p10')
  monkeypatch.setattr(storage, 'portable_configuration', lambda: layout.copy())
  selected = layout.copy()
  if mismatch == 'root':
    selected['root'] = '/dev/otherp1'
  monkeypatch.setattr(storage, 'configuration', lambda: selected)
  original_read, original_dir = Path.read_text, Path.is_dir
  control = 'TEGRA_BOOT_STORAGE mmcblk0\nCOMPATIBLE_SPEC 3767--0005--1--jetson-orin-nano-devkit-super-\n'
  if mismatch == 'board':
    control = control.replace('0005', '0001')
  monkeypatch.setattr(Path, 'read_text', lambda path, *a, **kw:
    control if path.as_posix() == '/etc/nv_boot_control.conf' else original_read(path, *a, **kw))
  monkeypatch.setattr(Path, 'is_dir', lambda path:
    True if path.as_posix() == '/sys/firmware/efi/efivars' else original_dir(path))
  def run(*args):
    if args == ('findmnt', '-n', '-o', 'SOURCE', str(boot.EFI)):
      return '/dev/otherp10' if mismatch == 'efi' else layout['efi']
    if args == ('findmnt', '-n', '-o', 'SOURCE', str(boot.STATE.parent)):
      return '/dev/otherp17' if mismatch == 'data' else layout['data']
    if args == ('blkid', '-s', 'TYPE', '-o', 'value', layout['efi']):
      return 'vfat'
    pytest.fail(str(args))
  monkeypatch.setattr(boot, 'run', run)
  if mismatch:
    with pytest.raises(ValueError):
      boot.validate_storage()
  else:
    boot.validate_storage()


@pytest.fixture
def staging(tmp_path, monkeypatch):
  capsule = tmp_path / 'TEGRA_BL_3767_super.Cap'
  capsule.write_bytes(b'official-capsule-fixture')
  efi = tmp_path / 'efi'
  efi.mkdir()
  monkeypatch.setattr(boot, 'CAPSULE', capsule)
  monkeypatch.setattr(boot, 'CAPSULE_SHA', hashlib.sha256(capsule.read_bytes()).hexdigest())
  monkeypatch.setattr(boot, 'EFI', efi)
  monkeypatch.setattr(boot, 'STATE', tmp_path / 'state.json')
  monkeypatch.setattr(boot, 'STATUS', tmp_path / 'status.json')
  monkeypatch.setattr(boot, 'atomic', lambda path, value: path.write_text(json.dumps(value)))
  original = Path.read_bytes
  monkeypatch.setattr(Path, 'read_bytes', lambda path:
    b'\x07\x00\x00\x00\x04\x00\x00\x00\x00\x00\x00\x00'
    if path.name.startswith('OsIndications-') else original(path))
  calls = []
  def run(*args):
    calls.append(args)
    if args[0].endswith('nv_bootloader_capsule_updater.sh'):
      assert json.loads(boot.STATE.read_text())['state'] == 'staging'
      directory = efi / 'EFI/UpdateCapsule'
      directory.mkdir(parents=True)
      (directory / capsule.name).write_bytes(capsule.read_bytes())
    return ''
  monkeypatch.setattr(boot, 'run', run)
  return calls, run


def test_stage_orders_durable_intent_vendor_helper_readback_and_readonly(staging):
  calls, _ = staging
  boot.stage_capsule(info())
  assert calls == [('mount', '-o', 'remount,rw', str(boot.EFI)),
                   ('/usr/sbin/nv_bootloader_capsule_updater.sh', '-q', str(boot.CAPSULE)),
                   ('sync', '-f', str(boot.EFI)), ('mount', '-o', 'remount,ro', str(boot.EFI))]
  assert json.loads(boot.STATE.read_text())['state'] == 'reboot_pending'


def test_bad_payload_does_not_mount_or_request_update(staging):
  calls, _ = staging
  boot.CAPSULE.write_bytes(b'bad')
  with pytest.raises(ValueError, match='hash mismatch'):
    boot.stage_capsule(info())
  assert not calls and not boot.STATE.exists()


def test_existing_capsule_is_preserved(staging):
  calls, _ = staging
  folder = boot.EFI / 'EFI/UpdateCapsule'
  folder.mkdir(parents=True)
  (folder / 'other.Cap').write_bytes(b'other')
  with pytest.raises(ValueError, match='already pending'):
    boot.stage_capsule(info())
  assert not calls


def test_helper_failure_restores_readonly_and_retains_attempt(staging, monkeypatch):
  calls, original = staging
  def run(*args):
    if args[0].endswith('nv_bootloader_capsule_updater.sh'):
      raise OSError('test write failure')
    return original(*args)
  monkeypatch.setattr(boot, 'run', run)
  with pytest.raises(OSError):
    boot.stage_capsule(info())
  assert calls[-1] == ('mount', '-o', 'remount,ro', str(boot.EFI))
  assert json.loads(boot.STATE.read_text())['state'] == 'staging'


def test_boot_screen_explains_progress_and_failure():
  title, _, detail, error = lines({'firmware': {'state': 'reboot_pending'}})
  assert title[1] == 'SUPER FIRMWARE UPDATE' and '전원' in detail and not error
  title, _, detail, error = lines({'firmware': {'state': 'firmware_update_failed'}})
  assert title[1] == 'SUPER SETUP NEEDS ATTENTION' and detail == 'firmware_update_failed' and error


@pytest.mark.skipif(sys.platform != 'linux', reason='Native first-boot entry point and flock')
@pytest.mark.parametrize('power_state,expected', [('ready', 0), ('clock_apply_failed', 1)])
def test_main_verifies_power_before_allowing_consumers(tmp_path, monkeypatch, power_state, expected):
  import power_setup
  monkeypatch.setattr(boot.os, 'geteuid', lambda: 0)
  monkeypatch.setattr(boot, 'STATE', tmp_path / 'state.json')
  monkeypatch.setattr(boot, 'STATUS', tmp_path / 'status.json')
  original_open, original_is_file = Path.open, Path.is_file
  monkeypatch.setattr(Path, 'open', lambda path, *a, **kw:
    original_open(tmp_path / 'lock', *a, **kw) if path.name == 'carrot-super-firmware.lock' else original_open(path, *a, **kw))
  monkeypatch.setattr(Path, 'is_file', lambda path:
    True if path.name == 'carrot-jetlink-image.json' else original_is_file(path))
  monkeypatch.setattr(boot, 'inspect', lambda: {**info(), 'version': boot.VERSION, 'super': True})
  monkeypatch.setattr(boot, 'validate_storage', lambda: None)
  monkeypatch.setattr(power_setup, 'run_once', lambda: {'state': power_state})
  monkeypatch.setattr(boot, 'run', lambda *a: pytest.fail('Unexpected firmware or reboot command'))
  assert boot.main() == expected
  status = json.loads(boot.STATUS.read_text())
  assert status['state'] == ('ready' if expected == 0 else 'power_verification_failed')
