"""Run the actual embedded ownership/stop code with bounded process simulations."""
import errno
import json
import os
from pathlib import Path
import signal
import sys
import time

import pytest

pytestmark = pytest.mark.skipif(sys.platform != 'linux', reason='Linux /proc, symlink and pidfd lifecycle checks')

SCRIPT = Path(__file__).with_name('setup_mobile.sh')


@pytest.fixture
def helper(tmp_path, monkeypatch):
  state, proc, lease = (tmp_path / n for n in ('state', 'proc', 'leases'))
  state.mkdir()
  target = proc / '43210'
  target.mkdir(parents=True)
  pidfile, record = state / 'dnsmasq.pid', state / 'dnsmasq.owner'
  pidfile.write_text('43210')
  record.write_text(json.dumps({'pid': 43210, 'start': '123'}))
  (target / 'exe').symlink_to('/usr/sbin/dnsmasq')
  args = ['dnsmasq', f'--pid-file={pidfile}', '--port=0', '--conf-file=/dev/null',
          f'--dhcp-leasefile={lease / "dnsmasq.leases"}']
  (target / 'cmdline').write_bytes(b'\0'.join(a.encode() for a in args) + b'\0')
  def stat(kind='S', start='123'):
    (target / 'stat').write_text('43210 (dnsmasq) ' + ' '.join([kind] + ['0'] * 18 + [start]))
  stat()
  clock = [0.0]
  signals = []
  exit_at = [0.0]
  disposition = ['gone']
  def sleep(seconds):
    clock[0] += seconds
    if clock[0] >= exit_at[0]:
      if disposition[0] == 'gone':
        (target / 'stat').unlink(missing_ok=True)
      elif disposition[0] == 'zombie':
        stat('Z')
      elif disposition[0] == 'reused':
        stat(start='456')
  def no_pidfd(pid):
    raise OSError(errno.ENOSYS, 'AGNOS 4.9')
  monkeypatch.setattr(os, 'pidfd_open', no_pidfd)
  monkeypatch.setattr(os, 'kill', lambda pid, sig: signals.append((pid, sig)))
  monkeypatch.setattr(time, 'monotonic', lambda: clock[0])
  monkeypatch.setattr(time, 'sleep', sleep)
  monkeypatch.setattr(sys, 'argv', ['-', str(state), 'stop', str(lease)])
  code = SCRIPT.read_text().split("<<'PY'\n", 1)[1].split('\nPY\n', 1)[0]
  code = code.replace("Path('/proc')", f'Path({str(proc)!r})')
  return {'run': lambda: exec(compile(code, str(SCRIPT), 'exec'), {}), 'state': state, 'target': target,
          'record': record, 'pidfile': pidfile, 'clock': clock, 'signals': signals, 'exit_at': exit_at,
          'disposition': disposition}


@pytest.mark.parametrize('delay', [0.02, 2.1, 3.4, 4.9])
@pytest.mark.parametrize('disposition', ['gone', 'zombie', 'reused'])
def test_waits_for_exact_process_to_release_resources(helper, delay, disposition):
  helper['exit_at'][0] = delay
  helper['disposition'][0] = disposition
  helper['run']()
  assert helper['signals'] == [(43210, signal.SIGTERM)]
  assert delay <= helper['clock'][0] <= delay + .021
  assert not helper['pidfile'].exists() and not helper['record'].exists()


def test_live_process_at_deadline_keeps_ownership_and_blocks(helper):
  helper['exit_at'][0] = 100
  with pytest.raises(SystemExit, match='did not stop within five seconds'):
    helper['run']()
  assert helper['clock'][0] == pytest.approx(5.0)
  assert helper['signals'] == [(43210, signal.SIGTERM)]
  assert helper['record'].exists() and helper['pidfile'].exists()


@pytest.mark.parametrize('mismatch', ['start', 'arguments', 'executable'])
def test_foreign_process_is_never_signaled(helper, mismatch):
  if mismatch == 'start':
    helper['record'].write_text(json.dumps({'pid': 43210, 'start': '999'}))
  elif mismatch == 'arguments':
    (helper['target'] / 'cmdline').write_bytes(b'dnsmasq\0--port=53\0')
  else:
    (helper['target'] / 'exe').unlink()
    (helper['target'] / 'exe').symlink_to('/bin/other')
  with pytest.raises(SystemExit, match='refusing DHCP ownership'):
    helper['run']()
  assert not helper['signals']
  assert helper['record'].exists() and helper['pidfile'].exists()


def test_exit_between_validation_and_signal_is_harmless(helper, monkeypatch):
  def gone(pid, sig):
    raise ProcessLookupError(errno.ESRCH, 'gone')
  monkeypatch.setattr(os, 'kill', gone)
  helper['run']()
  assert not helper['pidfile'].exists() and not helper['record'].exists()


@pytest.mark.parametrize('ready', [True, False])
def test_pidfd_wait_uses_same_bound_and_closes_descriptor(helper, monkeypatch, ready):
  import select
  calls = []
  class Poll:
    def register(self, fd, event):
      calls.append(('register', fd, event))
    def poll(self, milliseconds):
      calls.append(('wait', milliseconds))
      return [(42, select.POLLIN)] if ready else []
  monkeypatch.setattr(os, 'pidfd_open', lambda pid: 42)
  monkeypatch.setattr(os, 'close', lambda fd: calls.append(('close', fd)))
  monkeypatch.setattr(signal, 'pidfd_send_signal', lambda fd, sig: calls.append(('signal', fd, sig)))
  monkeypatch.setattr(select, 'poll', Poll)
  if ready:
    helper['run']()
  else:
    with pytest.raises(SystemExit, match='five seconds'):
      helper['run']()
  assert ('wait', 5000) in calls and ('close', 42) in calls
  assert not helper['signals']
  assert helper['record'].exists() is not ready
