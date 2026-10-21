"""Priority ownership, worker replacement and failure restoration regressions."""
import json
import os
from pathlib import Path
from types import SimpleNamespace

import pytest

import usb_priority as priority
from openpilot.selfdrive.modeld.jetlink import scheduling


@pytest.fixture
def workers(tmp_path, monkeypatch):
  monkeypatch.setattr(os, 'SCHED_OTHER', 0, raising=False)
  monkeypatch.setattr(os, 'SCHED_FIFO', 1, raising=False)
  rows = {10: ['a', [0, 0]], 11: ['b', [0, 0]]}
  writes = []

  def identify(pid):
    if pid not in rows:
      raise ProcessLookupError(pid)
    return rows[pid][0]

  def put(pid, value):
    writes.append((pid, value))
    rows[pid][1] = value

  leases = priority.Leases(tmp_path / 'state.json', 'boot', identify=identify,
                           get=lambda pid: rows[pid][1], put=put)
  return leases, rows, writes


def test_renewal_expiry_and_only_observed_workers(workers):
  leases, rows, writes = workers
  leases.observe(10, 1.)
  leases.observe(10, 2.)
  leases.expire(4.)
  assert writes == [(10, [1, 2])]
  assert rows[11][1] == [0, 0]
  leases.expire(5.)
  assert writes[-1] == (10, [0, 0])
  assert leases.entries == {}


def test_new_worker_after_previous_worker_exits(workers):
  leases, rows, writes = workers
  leases.observe(10, 1.)
  del rows[10]
  leases.observe(11, 2.)
  leases.expire(4.)
  assert rows[11][1] == [1, 2]
  leases.expire(float('inf'))
  assert writes[-1] == (11, [0, 0])
  assert leases.entries == {}


def test_pid_reuse_is_not_restored(workers):
  leases, rows, writes = workers
  leases.observe(10, 1.)
  rows[10] = ['new', [1, 2]]
  leases.expire(float('inf'))
  assert writes == [(10, [1, 2])]


def test_other_scheduler_owner_not_overwritten(workers):
  leases, rows, writes = workers
  rows[11][1] = [1, 5]
  leases.observe(11, 1.)
  leases.observe(10, 1.)
  rows[10][1] = [1, 3]
  leases.observe(10, 2.)
  leases.expire(float('inf'))
  assert writes == [(10, [1, 2])]
  assert rows[10][1] == [1, 3]


def test_one_restore_failure_does_not_block_others(workers):
  leases, rows, writes = workers
  leases.observe(10, 1.)
  leases.observe(11, 1.)
  original = leases.put

  def fail_one(pid, value):
    if pid == 10:
      raise PermissionError('denied')
    original(pid, value)

  leases.put = fail_one
  with pytest.raises(RuntimeError):
    leases.expire(float('inf'))
  assert rows[11][1] == [0, 0]
  assert set(leases.entries) == {'10'}


def test_journal_precedes_priority_change_and_recovers(workers):
  leases, rows, writes = workers
  original = leases.put

  def check_journal(pid, value):
    assert str(pid) in json.loads(leases.journal.read_text())['entries']
    original(pid, value)

  leases.put = check_journal
  leases.observe(10, 1.)
  leases.entries = {}
  leases.recover()
  assert rows[10][1] == [0, 0]


def test_previous_boot_journal_does_not_touch_reused_pids(workers):
  leases, rows, writes = workers
  leases.observe(10, 1.)
  leases.boot = 'next-boot'
  leases.entries = {}
  leases.recover()
  assert writes == [(10, [1, 2])]


def test_identity_changes_before_mutation(workers):
  leases, rows, writes = workers
  tokens = iter(['old', 'new'])
  leases.identify = lambda pid: next(tokens)
  leases.observe(10, 1.)
  assert writes == []


@pytest.mark.parametrize('line,expected', [
  ('           <...>-512270 [005] .... 59186.135133: workqueue_execute_start: work struct ffffffd2ab139730: function dwc3_bh_work', 512270),
  (' kworker/u17:2-580640 [005] d... 123.45: workqueue_execute_start: work struct ffffff function=dwc3_bh_work', 580640),
  (' kworker/u9:1-15 [000] .... 123.45: workqueue_execute_start: work struct abc function=dwc3_bh_work+0x0/0x10', 15),
  (' kworker/u17:2-580640 [005] d... 123.45: workqueue_execute_start: function=other_work', None),
  (' kworker/u17:2-580640 [005] d... 123.45: workqueue_queue_work: function=dwc3_bh_work', None),
])
def test_trace_parser(line, expected):
  match = priority.EVENT.match(line)
  assert (int(match[1]) if match else None) == expected


def test_partial_trace_records(monkeypatch):
  trace = priority.Trace()
  chunks = iter([b' kworker/u17:0-50 [001] .... 1.2: workqueue_execute_start: function=dwc3_', b'bh_work\n'])
  monkeypatch.setattr(os, 'read', lambda *args: next(chunks))
  assert trace.read() == set()
  assert trace.read() == {50}


@pytest.mark.parametrize('bulk,peer', [(False, {'carrot_host': 'jetson'}), (True, {}), (True, {'carrot_host': 'mac'})])
def test_unrelated_transports_do_not_start_helper(monkeypatch, bulk, peer):
  monkeypatch.setattr(scheduling.subprocess, 'Popen', lambda *a, **kw: pytest.fail('unexpected helper'))
  helper = scheduling.UsbCompletionPriority()
  helper.start(peer, bulk_usb=bulk)
  helper.close()


def test_owner_pipe_closed_and_helper_waited(monkeypatch):
  events = []
  process = SimpleNamespace(stdin=SimpleNamespace(close=lambda: events.append('eof')),
                            wait=lambda **kw: events.append('restored') or 0)
  monkeypatch.setattr(os, 'geteuid', lambda: 1000, raising=False)
  monkeypatch.setattr(scheduling.subprocess, 'Popen', lambda cmd, **kw: events.append((cmd, kw)) or process)
  helper = scheduling.UsbCompletionPriority()
  helper.start({'carrot_host': 'jetson'}, bulk_usb=True)
  helper.start({'carrot_host': 'jetson'}, bulk_usb=True)
  helper.close()
  helper.close()
  assert len(events) == 3
  assert events[0][0][:2] == ['sudo', '-n']
  assert events[0][1]['stdin'] == scheduling.subprocess.PIPE
  assert events[1:] == ['eof', 'restored']


def test_session_cleanup_includes_helper():
  source = (Path(__file__).resolve().parents[2] / 'openpilot/selfdrive/modeld/jetlink/daemon.py').read_text()
  assert 'close_resources(usb_priority, wifi, client)' in source
  assert 'usb_priority.start(peer, bulk_usb=transport is owner)' in source


def test_failed_restoration_blocks_session_reuse():
  helper = scheduling.UsbCompletionPriority()
  helper.process = SimpleNamespace(stdin=SimpleNamespace(close=lambda: None), wait=lambda **kw: 2)
  with pytest.raises(RuntimeError, match='restoration failed'):
    helper.close()


@pytest.mark.parametrize('board,allowed', [
  (b'comma tici\0', True), (b'comma tizi\0\n', True), (b'comma mici', True),
  (b'other tici', False), (b'comma tici-extra', False), (b'', False),
])
@pytest.mark.parametrize('release,machine,eligible', [
  ('4.9.103', 'aarch64', True), ('4.9.103-custom', 'aarch64', False),
  ('4.9.104', 'aarch64', False), ('4.9.103', 'x86_64', False),
])
def test_board_and_exact_kernel_gate(monkeypatch, board, allowed, release, machine, eligible):
  monkeypatch.setattr(os, 'uname', lambda: SimpleNamespace(release=release, machine=machine), raising=False)
  monkeypatch.setattr(priority.Path, 'read_bytes', lambda _: board)
  assert priority.supported() is (allowed and eligible)


def test_unreadable_board_is_ineligible(monkeypatch):
  monkeypatch.setattr(os, 'uname', lambda: SimpleNamespace(release='4.9.103', machine='aarch64'), raising=False)
  def denied(_):
    raise PermissionError('device tree unavailable')
  monkeypatch.setattr(priority.Path, 'read_bytes', denied)
  assert not priority.supported()
