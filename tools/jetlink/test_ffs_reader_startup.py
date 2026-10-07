import errno
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'third_party/jetlink'))
from jetlink.transport.ffs import FfsTransport


@pytest.mark.parametrize('method', ['_widen_affinity', '_raise_reader_priority'])
def test_reader_setup_failure_notifies_consumer_instead_of_silent_timeout(monkeypatch, method):
  transport = object.__new__(FfsTransport)
  errors = []
  monkeypatch.setattr(transport, '_widen_affinity', lambda: None)
  monkeypatch.setattr(transport, '_raise_reader_priority', lambda: None)
  def fail():
    raise PermissionError(errno.EPERM, 'test scheduling permission')
  monkeypatch.setattr(transport, method, fail)
  monkeypatch.setattr(transport, '_fail', errors.append)
  monkeypatch.setattr('jetlink.transport.ffs.signal.pthread_sigmask', lambda *a: None)
  transport._read_loop()
  assert len(errors) == 1 and 'gadget reader setup failed' in errors[0] and 'permission' in errors[0]
