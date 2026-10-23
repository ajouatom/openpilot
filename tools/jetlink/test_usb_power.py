"""Exercise actual current-voter writes on fake sysfs; never host hardware."""
from pathlib import Path
from types import SimpleNamespace
import os
import subprocess

import pytest

from openpilot.selfdrive.modeld.jetlink import power
import usb_power


@pytest.fixture
def nodes(tmp_path):
  voter = tmp_path / 'USB_ICL'
  voter.mkdir()
  (voter / 'force_active').write_text('0\n')
  (voter / 'force_val').write_text('900000\n')
  return voter


@pytest.mark.parametrize('role', ['sink', 'source', 'none', 'unknown'])
@pytest.mark.parametrize('forced', ['0', '1'])
def test_attached_boot_cuts_without_changing_data_roles_or_other_supplies(nodes, monkeypatch, role, forced):
  peers = {'current_pr': role, 'current_dr': 'ufp', 'UDC': 'a600000.dwc3',
           'main_supply': 'online', 'vbus': '1'}
  for name, value in peers.items():
    (nodes.parent / name).write_text(value)
  (nodes / 'force_active').write_text(forced)
  writes = []
  original = Path.write_text
  def write(path, value, *a, **kw):
    writes.append((path.name, value.strip()))
    return original(path, value, *a, **kw)
  monkeypatch.setattr(Path, 'write_text', write)
  assert usb_power.cut_input(nodes) == 'applied'
  assert usb_power.input_status(nodes) == {'force_active': '1', 'force_val': '0', 'input_blocked': True}
  assert writes == [('force_val', '0'), ('force_active', '1')]
  assert {n: (nodes.parent / n).read_text() for n in peers} == peers
  writes.clear()
  assert usb_power.cut_input(nodes) == 'applied'
  assert writes == [('force_val', '0'), ('force_active', '1')]


@pytest.mark.parametrize('previous_value', ['900000', '0'])
def test_active_vote_reinvokes_hardware_callback_even_after_prior_failure(nodes, monkeypatch, previous_value):
  # pmic-voter.c sets force_active before its callback. Even a failed callback
  # can leave readback at 1/0. force_val alone never invokes the hardware.
  (nodes / 'force_active').write_text('1')
  (nodes / 'force_val').write_text(previous_value)
  hardware = {'input_current': 900000, 'fail_once': True, 'callbacks': 0}
  original = Path.write_text
  def write(path, value, *a, **kw):
    result = original(path, value, *a, **kw)
    if path.name == 'force_active':
      assert value.strip() == '1'  # Never release the input-current force.
      hardware['callbacks'] += 1
      if hardware['fail_once']:
        hardware['fail_once'] = False
        raise OSError('PMIC callback failed')
      hardware['input_current'] = int((nodes / 'force_val').read_text())
    return result
  monkeypatch.setattr(Path, 'write_text', write)
  with pytest.raises(OSError, match='callback'):
    usb_power.cut_input(nodes)
  assert usb_power.input_status(nodes)['input_blocked']
  assert hardware['input_current'] == 900000
  assert usb_power.cut_input(nodes) == 'applied'
  assert hardware['input_current'] == 0
  assert hardware['callbacks'] == 2


@pytest.mark.parametrize('missing', ['force_active', 'force_val'])
def test_missing_sysfs_never_creates_a_node(nodes, missing):
  node = nodes / missing
  node.unlink()
  with pytest.raises((OSError, RuntimeError)):
    usb_power.cut_input(nodes)
  assert not node.exists()
  if missing != 'force_active':
    assert (nodes / 'force_active').read_text().strip() == '0'


def test_unknown_voter_state_is_not_overwritten(nodes):
  (nodes / 'force_active').write_text('unknown')
  with pytest.raises(RuntimeError, match='unknown'):
    usb_power.cut_input(nodes)
  assert (nodes / 'force_val').read_text().strip() == '900000'


@pytest.mark.parametrize('failed_name', ['force_val', 'force_active'])
def test_write_failure_propagates_without_claiming_protection(nodes, monkeypatch, failed_name):
  original = Path.write_text
  def write(path, text, *a, **kw):
    if path.name == failed_name:
      raise PermissionError('denied')
    return original(path, text, *a, **kw)
  monkeypatch.setattr(Path, 'write_text', write)
  with pytest.raises(PermissionError):
    usb_power.cut_input(nodes)
  assert (nodes / 'force_active').read_text().strip() == '0'


@pytest.mark.parametrize('lost_write', ['force_active', 'force_val'])
def test_failed_readback_is_reported(nodes, monkeypatch, lost_write):
  original = Path.write_text
  def write(path, text, *a, **kw):
    if path.name != lost_write:
      return original(path, text, *a, **kw)
  monkeypatch.setattr(Path, 'write_text', write)
  with pytest.raises(RuntimeError, match='read back'):
    usb_power.cut_input(nodes)


def test_unconditional_cut_reapplied_each_boot_and_retained_on_restart(nodes, monkeypatch):
  calls = []
  def cut():
    calls.append(1)
    return usb_power.cut_input(nodes) == 'applied'
  monkeypatch.setattr(power, 'cut_input', cut)
  for boot in range(3):
    # Model the kernel reset with Jetson still attached. No empty-port, gear,
    # voltage, ignition or onroad input is required to protect a new boot.
    (nodes / 'force_active').write_text('0')
    (nodes / 'force_val').write_text('900000')
    for _ in range(2):
      guard = power.UsbInputPower()
      for _ in range(10):
        guard.step()
      assert guard.protected and guard.finished and not guard.error
      assert usb_power.input_status(nodes)['input_blocked']
    assert len(calls) == (boot + 1) * 2


@pytest.mark.parametrize('error', [PermissionError('sudo'), RuntimeError('voter'), subprocess.TimeoutExpired('helper', 5)])
def test_helper_failure_is_bounded_and_keeps_link_unprotected(monkeypatch, caplog, error):
  calls = []
  def fail():
    calls.append(1)
    raise error
  monkeypatch.setattr(power, 'cut_input', fail)
  guard = power.UsbInputPower()
  for _ in range(10):
    guard.step()
  assert calls == [1] and guard.finished and not guard.protected
  assert 'protection not verified' in guard.error
  assert 'could not verify protection' in caplog.text


@pytest.mark.parametrize('uid', [0, 1000])
def test_helper_invocation_is_bounded_noninteractive_and_fixed_path(monkeypatch, uid):
  monkeypatch.setattr(power.os, 'geteuid', lambda: uid, raising=False)
  calls = []
  def run(command, **kw):
    calls.append((command, kw))
    return SimpleNamespace(returncode=0, stdout='applied\n', stderr='')
  monkeypatch.setattr(power.subprocess, 'run', run)
  assert power.cut_input()
  command, kwargs = calls[0]
  assert command == ([] if uid == 0 else ['sudo', '-n']) + ['python3', str(power.HELPER)]
  assert power.HELPER.is_file()
  assert kwargs['timeout'] == 5 and not kwargs['check']


@pytest.mark.parametrize('code,stdout', [(1, ''), (0, 'unexpected'), (3, 'deferred')])
def test_helper_status_is_not_silently_treated_as_success(monkeypatch, code, stdout):
  monkeypatch.setattr(power.os, 'geteuid', lambda: 0, raising=False)
  monkeypatch.setattr(power.subprocess, 'run', lambda *a, **kw: SimpleNamespace(returncode=code, stdout=stdout, stderr=''))
  with pytest.raises(RuntimeError):
    power.cut_input()


def test_status_command_never_writes(monkeypatch, capsys):
  monkeypatch.setattr(usb_power.os, 'geteuid', lambda: 0, raising=False)
  monkeypatch.setattr(usb_power.sys, 'argv', ['usb_power.py', '--status'])
  monkeypatch.setattr(usb_power, 'input_status', lambda: {'input_blocked': True})
  monkeypatch.setattr(usb_power, 'cut_input', lambda: pytest.fail('status cannot change power'))
  assert usb_power.main() == 0
  assert 'true' in capsys.readouterr().out


def test_boot_launcher_applies_before_build_and_manager():
  source = (Path(__file__).resolve().parents[2] / 'launch_chffrplus.sh').read_text()
  boot = source[source.index('function launch {'):]
  assert boot.index('protect_jetlink_usb_input') < boot.index('agnos_init') < boot.index('bootstrap_runtime_dependencies')
  assert boot.index('protect_jetlink_usb_input') < boot.index('start_manager')
  assert boot.index('protect_jetlink_usb_input') < boot.index('flock -w')


@pytest.mark.parametrize('result', [0, 1])
def test_launcher_cut_is_bounded_and_failure_does_not_abort_native_start(tmp_path, result):
  source = (Path(__file__).resolve().parents[2] / 'launch_chffrplus.sh').read_text()
  function = 'function protect_jetlink_usb_input {' + source.split(
    'function protect_jetlink_usb_input {', 1)[1].split('\n}\n', 1)[0] + '\n}\n'
  trace = tmp_path / 'arguments'
  # Stub the entire timeout command. Never invoke sudo, Python or host sysfs.
  harness = '''
set -e
DIR=/fake/repository
timeout() { printf '%s\\n' "$@" > "$TRACE"; return "$RESULT"; }
'''
  run = subprocess.run(['bash', '-c', harness + function + '\nprotect_jetlink_usb_input\necho NATIVE_CONTINUES'],
                       env=dict(os.environ, TRACE=str(trace), RESULT=str(result)), text=True, capture_output=True)
  assert run.returncode == 0 and 'NATIVE_CONTINUES' in run.stdout
  assert trace.read_text().splitlines() == ['5s', 'sudo', '-n', 'python3', '/fake/repository/tools/jetlink/usb_power.py']
  assert ('input blocked' if result == 0 else 'protection unavailable') in run.stdout
