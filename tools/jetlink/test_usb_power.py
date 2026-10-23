"""Exercise the actual current-voter helper on fake sysfs, never host hardware."""
from pathlib import Path
from types import SimpleNamespace
import subprocess

import pytest

from openpilot.selfdrive.modeld.jetlink import power
import usb_power


@pytest.fixture
def nodes(tmp_path):
  role, voter = tmp_path / 'current_pr', tmp_path / 'USB_ICL'
  role.write_text('none\n')
  voter.mkdir()
  (voter / 'force_active').write_text('0\n')
  (voter / 'force_val').write_text('900000\n')
  return role, voter


def test_actual_helper_cuts_once_and_preserves_vote_across_new_owners(nodes):
  role, voter = nodes
  assert usb_power.cut_input(role, voter) == 'applied'
  assert (voter / 'force_val').read_text().strip() == '0'
  assert (voter / 'force_active').read_text().strip() == '1'
  for state in ('sink', 'source', 'none'):
    role.write_text(state)
    assert usb_power.cut_input(role, voter) == ('applied' if state == 'none' else 'deferred')
    assert (voter / 'force_active').read_text().strip() == '1'


@pytest.mark.parametrize('state', ['sink', 'source', '', 'unknown'])
def test_already_attached_or_unknown_supply_is_never_cut(nodes, state):
  role, voter = nodes
  role.write_text(state)
  assert usb_power.cut_input(role, voter) == 'deferred'
  assert (voter / 'force_active').read_text().strip() == '0'
  assert (voter / 'force_val').read_text().strip() == '900000'


@pytest.mark.parametrize('missing', ['role', 'force_active', 'force_val'])
def test_missing_sysfs_never_creates_a_node(nodes, missing):
  role, voter = nodes
  node = role if missing == 'role' else voter / missing
  node.unlink()
  with pytest.raises((OSError, RuntimeError)):
    usb_power.cut_input(role, voter)
  assert not node.exists()
  if missing != 'force_active':
    assert (voter / 'force_active').read_text().strip() == '0'


def test_existing_nonzero_forced_vote_is_not_overwritten(nodes):
  role, voter = nodes
  (voter / 'force_active').write_text('1')
  with pytest.raises(RuntimeError, match='existing nonzero'):
    usb_power.cut_input(role, voter)
  assert (voter / 'force_val').read_text().strip() == '900000'


@pytest.mark.parametrize('attach_read', [2, 3])
def test_attachment_race_does_not_activate_current_cut(nodes, attach_read):
  _, voter = nodes
  states = iter(['none'] * (attach_read - 1) + ['sink'])
  role = SimpleNamespace(read_text=lambda: next(states))
  assert usb_power.cut_input(role, voter) == 'deferred'
  assert (voter / 'force_active').read_text().strip() == '0'


@pytest.mark.parametrize('failed_name', ['force_val', 'force_active'])
def test_write_failure_propagates_without_claiming_protection(nodes, monkeypatch, failed_name):
  role, voter = nodes
  original = Path.write_text
  def write(path, text, *a, **kw):
    if path.name == failed_name:
      raise PermissionError('denied')
    return original(path, text, *a, **kw)
  monkeypatch.setattr(Path, 'write_text', write)
  with pytest.raises(PermissionError):
    usb_power.cut_input(role, voter)
  assert (voter / 'force_active').read_text().strip() == '0'


def test_failed_readback_is_reported(nodes, monkeypatch):
  role, voter = nodes
  original = Path.write_text
  def write(path, text, *a, **kw):
    if path.name != 'force_active':
      return original(path, text, *a, **kw)
  monkeypatch.setattr(Path, 'write_text', write)
  with pytest.raises(RuntimeError, match='read back'):
    usb_power.cut_input(role, voter)


@pytest.fixture
def policy(nodes, monkeypatch):
  role, voter = nodes
  clock = [0.]
  calls = []
  monkeypatch.setattr(power, 'POWER_ROLE', role)
  monkeypatch.setattr(power.time, 'monotonic', lambda: clock[0])
  def cut():
    calls.append(clock[0])
    return usb_power.cut_input(role, voter) == 'applied'
  monkeypatch.setattr(power, 'cut_input', cut)
  return power.UsbInputPower(), role, voter, clock, calls


def test_stable_empty_port_required_then_plug_and_retry_keep_guard(policy):
  guard, role, voter, clock, calls = policy
  guard.step()
  clock[0] = 1.
  guard.step()
  assert calls == []
  role.write_text('sink')
  clock[0] = 2.
  guard.step()
  role.write_text('none')
  clock[0] = 10.
  guard.step()
  clock[0] = 11.49
  guard.step()
  assert calls == []
  clock[0] = 11.5
  guard.step()
  for state in ('sink', 'none', 'source'):
    role.write_text(state)
    guard.step()
  assert calls == [11.5]
  assert (voter / 'force_active').read_text().strip() == '1'


@pytest.mark.parametrize('state', ['sink', 'source', 'unknown', '', None])
def test_cold_boot_with_attached_or_unknown_supply_waits(policy, state):
  guard, role, voter, clock, calls = policy
  if state is None:
    role.unlink()
  else:
    role.write_text(state)
  for now in (0., 2., 60.):
    clock[0] = now
    guard.step()
  assert calls == [] and not guard.finished
  assert (voter / 'force_active').read_text().strip() == '0'


def test_helper_attachment_race_retries_only_after_another_empty_interval(policy, monkeypatch):
  guard, _, _, clock, calls = policy
  real = power.cut_input
  results = iter([False, True])
  monkeypatch.setattr(power, 'cut_input', lambda: real() if next(results) else False)
  guard.step()
  clock[0] = 2.
  guard.step()
  assert not guard.finished and calls == []
  guard.step()
  clock[0] = 4.
  guard.step()
  assert calls == [4.] and guard.finished


@pytest.mark.parametrize('error', [PermissionError('sudo'), RuntimeError('voter'), subprocess.TimeoutExpired('helper', 5)])
def test_helper_failure_is_bounded_and_does_not_abort_daemon(policy, monkeypatch, caplog, error):
  guard, _, _, clock, _ = policy
  calls = []
  def fail():
    calls.append(1)
    raise error
  monkeypatch.setattr(power, 'cut_input', fail)
  guard.step()
  for now in (2., 3., 100.):
    clock[0] = now
    guard.step()
  assert calls == [1]
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
  if code == 3:
    assert not power.cut_input()
  else:
    with pytest.raises(RuntimeError):
      power.cut_input()


def test_zero_vote_reasserts_callback_after_previous_failure(nodes, monkeypatch):
  role, voter = nodes
  active = voter / 'force_active'
  (voter / 'force_val').write_text('0')
  active.write_text('1')
  original = Path.write_text
  attempts = []
  def write(path, value, *args, **kwargs):
    if path == active:
      attempts.append(value)
      if len(attempts) == 1:
        raise OSError('previous callback failure persists')
    return original(path, value, *args, **kwargs)
  monkeypatch.setattr(Path, 'write_text', write)
  with pytest.raises(OSError, match='callback failure'):
    usb_power.cut_input(role, voter)
  assert usb_power.cut_input(role, voter) == 'applied'
  assert attempts == ['1\n', '1\n']


@pytest.mark.parametrize('attach_read', [1, 2, 3])
def test_zero_vote_retry_never_reasserts_after_attachment(nodes, monkeypatch, attach_read):
  _, voter = nodes
  (voter / 'force_val').write_text('0')
  (voter / 'force_active').write_text('1')
  states = iter(['none'] * (attach_read - 1) + ['sink'])
  role = SimpleNamespace(read_text=lambda: next(states))
  original = Path.write_text
  calls = []
  def write(path, value, *args, **kwargs):
    if path.name == 'force_active': calls.append(value)
    return original(path, value, *args, **kwargs)
  monkeypatch.setattr(Path, 'write_text', write)
  assert usb_power.cut_input(role, voter) == 'deferred'
  assert not calls
