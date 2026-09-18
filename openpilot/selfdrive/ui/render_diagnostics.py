"""Measure render CPU work separately from elapsed time and scheduler waits."""
import time

from openpilot.common.runtime_diagnostics import RuntimeDiagnostics


def _emit(*args, **kwargs):
  from openpilot.common.swaglog import cloudlog
  cloudlog.event(*args, **kwargs)


class RenderDiagnostics:
  def __init__(self, component, emit=_emit):
    self.runtime = RuntimeDiagnostics(component, emit)
    self.values = {}
    self.started = self.cpu_started = 0

  def start(self):
    self.values.clear()
    self.started = time.monotonic_ns()
    self.cpu_started = time.thread_time_ns()

  def call(self, name, callback, *args):
    started = time.monotonic_ns()
    cpu_started = time.thread_time_ns()
    try:
      return callback(*args)
    finally:
      cpu_ms = (time.thread_time_ns() - cpu_started) * 1e-6
      elapsed_ms = (time.monotonic_ns() - started) * 1e-6
      for suffix, value in (('_ms', elapsed_ms), ('_cpu_ms', cpu_ms)):
        key = name + suffix
        self.values[key] = self.values.get(key, 0.) + value

  def finish(self):
    self.runtime.record(work_ms=(time.monotonic_ns() - self.started) * 1e-6,
                        thread_cpu_ms=(time.thread_time_ns() - self.cpu_started) * 1e-6, **self.values)
