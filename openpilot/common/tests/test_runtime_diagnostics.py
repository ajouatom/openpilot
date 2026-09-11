from types import SimpleNamespace

import pytest

from openpilot.common import runtime_diagnostics as diagnostics
from openpilot.common.utils import MovingAverage


def test_aggregation_is_bounded_and_logs_pre_error_baseline(monkeypatch, tmp_path):
  now = [0.0]
  monkeypatch.setattr(diagnostics.time, 'monotonic', lambda: now[0])
  events = []
  timing = diagnostics.RuntimeDiagnostics('modeld', lambda *a, **kw: events.append(kw))
  timing.sched_path = tmp_path / 'schedstat'
  timing.sched_prev = (1000000, 2000000, 3)
  timing.sched_path.write_text('6000000 11000000 7')
  for i in range(1000):
    timing.record(inference_ms=10 if i < 999 else 100, unavailable=float('nan'))
  assert len(timing.samples) == 1
  assert not events
  now[0] = 1.0
  timing.record(inference_ms=20)
  assert events[0]['metrics']['inference_ms'] == {'mean': 10.1, 'max': 100, 'count': 1001}
  assert events[0]['scheduler'] == {'cpu_ms': 5, 'runqueue_wait_ms': 9, 'timeslices': 4}
  assert timing.samples == {}
  now[0] = 2.0
  timing.record(inference_ms=12)
  assert events[1]['metrics']['inference_ms']['max'] == 12


def test_missing_schedstat_and_logging_failure_do_not_stop_inference(monkeypatch, tmp_path):
  now = [0.0]
  monkeypatch.setattr(diagnostics.time, 'monotonic', lambda: now[0])
  def fail(*args, **kwargs):
    assert kwargs['scheduler'] == {}
    raise OSError('log transport unavailable')
  timing = diagnostics.RuntimeDiagnostics('worker', fail)
  timing.sched_path = tmp_path / 'missing'
  now[0] = 1.0
  timing.record(work_ms=40)
  assert timing.frames == 0
  assert timing.samples == {}


def test_communication_snapshot_distinguishes_ignored_service_and_real_low_rate(monkeypatch):
  monkeypatch.setattr(diagnostics.time, 'monotonic', lambda: 10.0)
  average = MovingAverage(20)
  for _ in range(20):
    average.add_value(1 / 15)
  empty = MovingAverage(20)
  names = ['driverAssistance', 'driverMonitoringState']
  sm = SimpleNamespace(
    freq_tracker={name: SimpleNamespace(avg_dt=avg, recent_avg_dt=avg, min_freq=16, max_freq=24)
                  for name, avg in zip(names, [average, empty], strict=True)},
    recv_time=dict.fromkeys(names, 9.9), seen=dict(zip(names, [True, False], strict=True)),
    valid=dict.fromkeys(names, True), alive=dict.fromkeys(names, True), freq_ok=dict.fromkeys(names, False),
    ignore_alive=['driverMonitoringState'], ignore_valid=['driverMonitoringState'], ignore_average_freq=[],
  )
  snapshot = diagnostics.communication_snapshot(sm, names + ['notSubscribed'])
  assert snapshot['driverAssistance']['avg_hz'] == pytest.approx(15)
  assert snapshot['driverAssistance']['recv_age_ms'] == pytest.approx(100)
  assert not snapshot['driverAssistance']['ignore_freq']
  assert snapshot['driverMonitoringState']['ignore_freq']
  assert snapshot['driverMonitoringState']['avg_hz'] is None
  assert snapshot['driverMonitoringState']['recv_age_ms'] is None
  assert 'notSubscribed' not in snapshot
