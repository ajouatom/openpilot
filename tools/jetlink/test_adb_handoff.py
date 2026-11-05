"""실제 USB 대신 임시 파일과 잠금으로 ADB 인계의 경계를 검사한다."""
import sys

import pytest

if sys.platform != "linux":
  pytest.skip("Linux configfs/ownership simulation", allow_module_level=True)

import fcntl
import importlib.util
import os
from pathlib import Path
import subprocess

import pytest


SOURCE = Path(__file__).with_name('adb_handoff.py')
SPEC = importlib.util.spec_from_file_location('tested_adb_handoff', SOURCE)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


@pytest.fixture
def fixture(tmp_path):
  io = MODULE.AdbHandoff(tmp_path)
  files = {
    '/TICI': '', MODULE.FLAG: '', '/data/params/d/IsOffroad': '1',
    '/data/params/d/IsOnroad': '0', '/sys/class/usbpd/usbpd0/current_dr': 'ufp',
    '/sys/kernel/config/usb_gadget/g1/UDC': MODULE.CONTROLLER,
    '/sys/kernel/config/usb_gadget/g1/idVendor': '0x04d8',
    '/sys/kernel/config/usb_gadget/g1/idProduct': '0x1234',
    '/sys/kernel/config/usb_gadget/jetlink/UDC': '',
  }
  for name, value in files.items():
    path = io.path(name)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(value)
  for name in ('ffs.adb', 'ncm.0'):
    (io.gadgets / 'g1/functions' / name).mkdir(parents=True)
  io.path('/dev/shm').mkdir(parents=True)
  return io


@pytest.fixture
def writes(monkeypatch):
  calls = []
  real_write = os.write

  def sysfs_write(fd, data):
    # 일반 파일의 덮어쓰기를 sysfs UDC 속성의 전체 값 교체처럼 모사한다.
    # 이 truncate는 검사 전용이며 실제 helper에는 추가하지 않는다.
    calls.append(data)
    os.ftruncate(fd, 0)
    os.lseek(fd, 0, os.SEEK_SET)
    return real_write(fd, data)

  monkeypatch.setattr(MODULE.os, 'write', sysfs_write)
  return calls


def test_exact_handoff_clears_only_adb_once(fixture, writes):
  io = fixture
  before = {str(p.relative_to(io.root)): p.read_bytes() for p in io.root.rglob('*') if p.is_file()}
  assert io.permitted(MODULE.CONTROLLER)
  assert io.release(MODULE.CONTROLLER)
  assert writes == [b'\n']
  assert not (io.gadgets / 'g1/UDC').read_text().strip()
  assert not io.release(MODULE.CONTROLLER)
  assert writes == [b'\n']
  for path, raw in before.items():
    if path != 'sys/kernel/config/usb_gadget/g1/UDC':
      assert (io.root / path).read_bytes() == raw


@pytest.mark.parametrize('case', [
  'default_off', 'wrong_controller', 'no_tici', 'onroad', 'unknown_road',
  'host_role', 'wrong_vendor', 'wrong_product', 'missing_function', 'extra_function',
  'function_symlink', 'flag_symlink', 'flag_hardlink', 'foreign_owner',
  'foreign_controller', 'jetlink_owned', 'g1_symlink', 'udc_symlink',
])
def test_unverified_or_in_use_state_never_writes(fixture, writes, case):
  io = fixture
  controller = MODULE.CONTROLLER
  flag = io.path(MODULE.FLAG)
  adb = io.gadgets / 'g1'
  if case == 'default_off':
    flag.unlink()
  elif case == 'wrong_controller':
    controller = 'unrecognized.dwc3'
  elif case == 'no_tici':
    io.path('/TICI').unlink()
  elif case == 'onroad':
    io.path('/data/params/d/IsOnroad').write_text('1')
  elif case == 'unknown_road':
    io.path('/data/params/d/IsOffroad').unlink()
  elif case == 'host_role':
    io.path('/sys/class/usbpd/usbpd0/current_dr').write_text('dfp')
  elif case in ('wrong_vendor', 'wrong_product'):
    (adb / ('idVendor' if case == 'wrong_vendor' else 'idProduct')).write_text('0x9999')
  elif case == 'missing_function':
    (adb / 'functions/ncm.0').rmdir()
  elif case == 'extra_function':
    (adb / 'functions/mass_storage.0').mkdir()
  elif case == 'function_symlink':
    (adb / 'functions/ffs.adb').rmdir()
    (adb / 'functions/ffs.adb').symlink_to(adb / 'functions/ncm.0', target_is_directory=True)
  elif case in ('flag_symlink', 'flag_hardlink'):
    original = flag.with_suffix('.original')
    flag.rename(original)
    if case == 'flag_symlink':
      flag.symlink_to(original)
    else:
      os.link(original, flag)
  elif case == 'foreign_owner':
    (io.gadgets / 'foreign').mkdir()
    (io.gadgets / 'foreign/UDC').write_text(MODULE.CONTROLLER)
  elif case == 'foreign_controller':
    (adb / 'UDC').write_text('another.dwc3')
  elif case == 'jetlink_owned':
    (adb / 'UDC').write_text('')
    (io.gadgets / 'jetlink/UDC').write_text(MODULE.CONTROLLER)
  elif case == 'g1_symlink':
    saved = io.root / 'saved-g1'
    adb.rename(saved)
    adb.symlink_to(saved, target_is_directory=True)
  elif case == 'udc_symlink':
    (adb / 'UDC').unlink()
    (adb / 'UDC').symlink_to(io.gadgets / 'jetlink/UDC')
  assert not io.permitted(controller)
  assert not io.release(controller)
  assert writes == []
  expected = {
    'default_off': 'opt_in_missing', 'wrong_controller': 'controller', 'no_tici': 'platform',
    'onroad': 'offroad', 'unknown_road': 'io_error', 'host_role': 'usb_role',
    'wrong_vendor': 'gadget_identity', 'wrong_product': 'gadget_identity',
    'missing_function': 'gadget_functions', 'extra_function': 'gadget_functions',
    'function_symlink': 'gadget_functions', 'flag_symlink': 'opt_in_invalid',
    'flag_hardlink': 'opt_in_invalid', 'foreign_owner': 'other_owner',
    'foreign_controller': 'controller_changed', 'jetlink_owned': 'controller_changed',
    'g1_symlink': 'gadget_identity', 'udc_symlink': 'io_error',
  }
  # Iteration may encounter the newly bound jetlink before the detached g1.
  if case == 'jetlink_owned':
    assert any(f'[adb-handoff:{code}]' in io.reason for code in ('controller_changed', 'other_owner'))
  else:
    assert f'[adb-handoff:{expected[case]}]' in io.reason
  if case == 'onroad':
    assert "IsOffroad='1', IsOnroad='1'" in io.reason
  if case == 'unknown_road':
    assert 'road state: FileNotFoundError, errno=2' in io.reason


@pytest.mark.parametrize('race', ['onroad', 'foreign_owner', 'flag_removed'])
def test_revalidation_after_open_rejects_changed_state(fixture, writes, monkeypatch, race):
  io = fixture
  real_permitted = io.permitted
  checks = []

  def changing_permitted(controller):
    checks.append(controller)
    if len(checks) == 2:
      if race == 'onroad':
        io.path('/data/params/d/IsOnroad').write_text('1')
      elif race == 'foreign_owner':
        (io.gadgets / 'foreign').mkdir()
        (io.gadgets / 'foreign/UDC').write_text(controller)
      else:
        io.path(MODULE.FLAG).unlink()
    return real_permitted(controller)

  monkeypatch.setattr(io, 'permitted', changing_permitted)
  assert not io.release(MODULE.CONTROLLER)
  assert len(checks) == 2
  assert writes == []
  assert (io.gadgets / 'g1/UDC').read_text() == MODULE.CONTROLLER
  assert '(pre-write recheck)' in io.reason


def test_existing_lock_prevents_duplicate_handoff(fixture, writes):
  io = fixture
  lock = os.open(io.path('/dev/shm/jetlink-adb-handoff.lock'), os.O_RDWR | os.O_CREAT, 0o600)
  try:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    assert not io.release(MODULE.CONTROLLER)
    assert writes == []
    assert '[adb-handoff:lock_busy]' in io.reason
  finally:
    os.close(lock)
  assert io.release(MODULE.CONTROLLER)
  assert writes == [b'\n']


@pytest.mark.parametrize('kind', ['symlink', 'hardlink'])
def test_untrusted_lock_preserves_target(fixture, writes, kind):
  io = fixture
  target = io.root / 'untouched'
  target.write_bytes(b'keep')
  lock = io.path('/dev/shm/jetlink-adb-handoff.lock')
  if kind == 'symlink':
    lock.symlink_to(target)
    with pytest.raises(OSError):
      io.release(MODULE.CONTROLLER)
  else:
    os.link(target, lock)
    assert not io.release(MODULE.CONTROLLER)
  assert target.read_bytes() == b'keep'
  assert writes == []


@pytest.mark.parametrize('case,ok,called', [
  ('default_off', False, True), ('approved', True, True),
  ('helper_rejected', False, True), ('foreign_owner', False, False),
])
def test_actual_shell_owner_loop_respects_helper_result(fixture, case, ok, called):
  io = fixture
  source = SOURCE.with_name('setup_gadget.sh').read_text()
  # 실제 소유자 루프만 실행하고 이후 mount/configfs 생성 단계는 포함하지 않는다.
  loop = 'for other in "$CONFIGFS"/usb_gadget/*/UDC; do' + source.split(
    'for other in "$CONFIGFS"/usb_gadget/*/UDC; do', 1)[1].split('\nmkdir -p "$GADGET"', 1)[0]
  reporting = 'status() {' + source.split('status() {', 1)[1].split('if [[ "${1:-}" == "--teardown" ]]', 1)[0]
  loop = loop.replace(MODULE.FLAG, str(io.path(MODULE.FLAG)))
  if case == 'default_off':
    io.path(MODULE.FLAG).unlink()
  if case == 'foreign_owner':
    (io.gadgets / 'g1').rename(io.gadgets / 'foreign')
  script = io.root / 'owner-loop.sh'
  script.write_text('''set -euo pipefail
CONFIGFS=$1
LOG=$2
UDC=$3
HELPER_RC=$4
STATUS_FILE=$CONFIGFS/handoff-status
python3() {
  printf '%s\\n' "$*" > "$LOG"
  if [[ "$HELPER_RC" != 0 ]]; then
    printf '[adb-handoff:%s] test rejection\\n' "$HELPER_RC" >&2
    return 1
  fi
  printf '\\n' > "$UDC"
}
''' + reporting + loop)
  log = io.root / 'helper-call'
  result = subprocess.run(['/bin/bash', str(script), str(io.path('/sys/kernel/config')),
                           str(log), str(io.gadgets / 'g1/UDC'),
                           {'default_off': 'opt_in_missing', 'helper_rejected': 'offroad'}.get(case, '0')],
                          env={'PATH': '/usr/bin:/bin', 'LANG': 'C', 'LC_ALL': 'C'},
                          stdin=subprocess.DEVNULL, capture_output=True, timeout=5, check=False)
  assert (result.returncode == 0) == ok
  assert log.exists() == called
  if called:
    args = log.read_text().strip().split()
    assert args[:3] == ['-I', '-S', '-B']
    assert args[-2:] == ['--udc', MODULE.CONTROLLER]
  if case in ('default_off', 'helper_rejected'):
    code = 'opt_in_missing' if case == 'default_off' else 'offroad'
    assert f'[adb-handoff:{code}] test rejection' in result.stderr.decode()
    assert io.path('/sys/kernel/config/handoff-status').read_text().strip() == f'error: [adb-handoff:{code}] test rejection'
    assert (io.gadgets / 'g1/UDC').read_text() == MODULE.CONTROLLER


@pytest.mark.parametrize('fault', ['writable', 'foreign_uid'])
def test_opt_in_marker_must_belong_to_privileged_caller(fixture, writes, monkeypatch, fault):
  io = fixture
  if fault == 'writable':
    io.path(MODULE.FLAG).chmod(0o666)
  else:
    uid = os.geteuid()
    monkeypatch.setattr(MODULE.os, 'geteuid', lambda: uid + 1)
  assert not io.release(MODULE.CONTROLLER)
  assert writes == []
  assert '[adb-handoff:opt_in_invalid]' in io.reason


@pytest.mark.parametrize('fault,code', [('short_write', 'write_failed'), ('rebound', 'still_bound')])
def test_unbind_failure_reports_exact_stage(fixture, monkeypatch, fault, code):
  monkeypatch.setattr(MODULE.os, 'write', lambda fd, data: 0 if fault == 'short_write' else 1)
  assert not fixture.release(MODULE.CONTROLLER)
  assert f'[adb-handoff:{code}]' in fixture.reason


def test_cli_reports_io_stage_without_traceback(fixture, monkeypatch, capsys):
  def failed_release(controller):
    fixture.stage = 'unbind g1 UDC'
    raise PermissionError(13, 'test')
  monkeypatch.setattr(fixture, 'release', failed_release)
  monkeypatch.setattr(MODULE, 'AdbHandoff', lambda: fixture)
  monkeypatch.setattr(MODULE.os, 'geteuid', lambda: 0)
  monkeypatch.setattr(sys, 'argv', ['adb_handoff.py', '--udc', MODULE.CONTROLLER])
  assert MODULE.main() == 1
  error = capsys.readouterr().err
  assert '[adb-handoff:io_error] unbind g1 UDC: PermissionError, errno=13' in error
  assert 'Traceback' not in error
