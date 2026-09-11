"""Bounded timing summaries for diagnosing on-road process rate failures."""
import math
import os
from pathlib import Path
import threading
import time


class RuntimeDiagnostics:
  def __init__(self, component, emit, interval=1.0):
    self.component, self.emit, self.interval = component, emit, interval
    self.started = time.monotonic()
    self.samples = {}
    self.frames = 0
    self.sched_path = Path(f'/proc/self/task/{threading.get_native_id()}/schedstat')
    self.sched_prev = self._schedstat()
    try:
      self.schedstats_enabled = Path('/proc/sys/kernel/sched_schedstats').read_text().strip() == '1'
    except OSError:
      self.schedstats_enabled = None

  def _schedstat(self):
    try:
      return tuple(int(v) for v in self.sched_path.read_text().split()[:3])
    except (OSError, ValueError):
      return ()

  def record(self, context=None, **values):
    # No growing history or per-frame logging. One aggregate per second includes
    # healthy periods, so an uploaded segment also contains the pre-error baseline.
    self.frames += 1
    for name, value in values.items():
      if math.isfinite(value):
        total, maximum, count = self.samples.get(name, (0.0, float('-inf'), 0))
        self.samples[name] = (total + value, max(maximum, value), count + 1)
    now = time.monotonic()
    if now - self.started < self.interval:
      return
    metrics = {k: {'mean': round(s / n, 3), 'max': round(mx, 3), 'count': n}
               for k, (s, mx, n) in self.samples.items()}
    sched = self._schedstat()
    scheduler = {}
    if len(sched) == len(self.sched_prev) == 3:
      scheduler = {'cpu_ms': (sched[0] - self.sched_prev[0]) / 1e6,
                   'runqueue_wait_ms': (sched[1] - self.sched_prev[1]) / 1e6,
                   'timeslices': sched[2] - self.sched_prev[2]}
    self.sched_prev = sched
    seconds, frames = now - self.started, self.frames
    self.started, self.frames, self.samples = now, 0, {}
    try:
      self.emit('runtimeTiming', component=self.component, pid=os.getpid(), mono_time=now,
                seconds=round(seconds, 3), frames=frames, metrics=metrics, scheduler=scheduler,
                schedstats_enabled=self.schedstats_enabled, **(context or {}))
    except Exception:
      # A logging failure must not interrupt model inference or control planning.
      pass


def communication_snapshot(sm, services):
  """Capture the receiver's actual frequency filters, including ignored checks."""
  now = time.monotonic()
  result = {}
  for service in services:
    if service not in sm.freq_tracker:
      continue
    tracker = sm.freq_tracker[service]
    def hz(average):
      dt = average.get_average() if average.count else 0
      return round(1.0 / dt, 3) if dt > 0 else None
    result[service] = {
      'avg_hz': hz(tracker.avg_dt), 'recent_hz': hz(tracker.recent_avg_dt),
      'min_hz': tracker.min_freq, 'max_hz': tracker.max_freq,
      'recv_age_ms': round((now - sm.recv_time[service]) * 1000, 3) if sm.seen[service] else None,
      'valid': sm.valid[service], 'alive': sm.alive[service], 'freq_ok': sm.freq_ok[service],
      'ignore_alive': service in sm.ignore_alive, 'ignore_valid': service in sm.ignore_valid,
      'ignore_freq': service in sm.ignore_average_freq or service in sm.ignore_alive,
    }
  return result
