import gc
import sys
from itertools import count
from types import SimpleNamespace

import pytest

from openpilot.system.memory_trace import FullGCTrace
from openpilot.system import memory_trace


def test_full_collection_report_survives_rotation_and_unregisters(monkeypatch):
  monkeypatch.setattr(memory_trace.time, 'monotonic_ns', count(100).__next__)
  events = []
  sink = SimpleNamespace(event=lambda name, **fields: events.append((name, fields)))
  monkeypatch.setitem(sys.modules, 'openpilot.common.swaglog', SimpleNamespace(cloudlog=sink))
  with pytest.raises(RuntimeError), FullGCTrace('hardwared') as trace:
    gc.collect(0)
    assert trace.collections == 0
    gc.collect(2)
    assert trace.due(5.)
    trace.report(5., hw_queue=0)
    assert not trace.due(34.9)
    assert trace.due(35.)
    gc.collect(2)
    trace.report(35., hw_queue=1)
    assert events[0][0] == 'DAEMON_MEMORY'
    first, last = (item[1] for item in events)
    assert first['gen2_count'] == 1 and last['gen2_count'] == 2
    assert first['first_gen2_ns'] == last['first_gen2_ns'] == first['last_gen2_ns']
    assert first['last_gen2_ns'] < last['last_gen2_ns'] <= last['mono_ns']
    assert first['pid'] == last['pid'] and last['hw_queue'] == 1
    raise RuntimeError
  assert trace._observe not in gc.callbacks


def test_unavailable_log_does_not_stop_daemon_or_retry_every_iteration(monkeypatch):
  def fail(*args, **kwargs):
    raise OSError
  monkeypatch.setitem(sys.modules, 'openpilot.common.swaglog', SimpleNamespace(cloudlog=SimpleNamespace(event=fail)))
  with FullGCTrace('deleter') as trace:
    trace.report(20., xattr_entries=0)
    assert not trace.due(49.)


def test_later_snapshots_preserve_the_highest_fault_gc(monkeypatch):
  faults = iter((100, 9850, 10000, 10001))
  monkeypatch.setattr(memory_trace, '_minor_faults', lambda: next(faults))
  monkeypatch.setattr(memory_trace.time, 'monotonic_ns', count(100).__next__)
  trace = FullGCTrace('carrot_bluetooth')
  for _ in range(2):
    trace._observe('start', {'generation': 2})
    trace._observe('stop', {'generation': 2})
  assert trace.max_gc_faults == 9750
  assert trace.max_gc_ns == trace.first_ns < trace.last_ns
