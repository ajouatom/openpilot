"""Correlate daemon state and full collections with paced procLog samples."""
import gc
import os
import time

try:
  from resource import RUSAGE_SELF, getrusage
except ImportError:  # Windows offline tools
  getrusage = None


def _minor_faults():
  return getrusage(RUSAGE_SELF).ru_minflt if getrusage is not None else -1


class FullGCTrace:
  def __init__(self, daemon: str):
    self.daemon = daemon
    self.next_report = 0.
    self.collections = 0
    self.first_ns = self.last_ns = 0
    self.start_faults = self.max_gc_faults = -1
    self.max_gc_ns = 0

  def __enter__(self):
    gc.callbacks.append(self._observe)
    return self

  def __exit__(self, *_):
    gc.callbacks.remove(self._observe)

  def _observe(self, phase, info):
    # No logging, file reads, or explicit collection inside the GC callback.
    if info['generation'] != 2:
      return
    if phase == 'start':
      self.start_faults = _minor_faults()
    elif phase == 'stop':
      self.last_ns = time.monotonic_ns()
      if not self.collections:
        self.first_ns = self.last_ns
      self.collections += 1
      # Process-wide, including sibling threads: an indicator of COW, not proof.
      if self.start_faults >= 0:
        delta = _minor_faults() - self.start_faults
        if delta > self.max_gc_faults:
          self.max_gc_faults, self.max_gc_ns = delta, self.last_ns
        self.start_faults = -1

  def due(self, now: float) -> bool:
    return now >= self.next_report

  def report(self, now: float, **counts):
    self.next_report = now + 30.
    try:
      from openpilot.common.swaglog import cloudlog
      cloudlog.event('DAEMON_MEMORY', daemon=self.daemon, pid=os.getpid(), mono_ns=time.monotonic_ns(),
                     gen2_count=self.collections, first_gen2_ns=self.first_ns, last_gen2_ns=self.last_ns,
                     max_gc_minor_faults=self.max_gc_faults, max_gc_end_ns=self.max_gc_ns, **counts)
    except Exception:
      # Missing diagnostic output must not interrupt the daemon.
      pass
