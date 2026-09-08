"""Bounded revalidation of optional resident work after a transient slowdown."""
from collections import deque

from openpilot.selfdrive.modeld.egpu_yolo import IdleBudget, MIN_INTERVAL, RUNTIME_MARGIN


class RevalidatingBudget(IdleBudget):
  WINDOW = 30.0
  PROBE_INTERVAL = 1.0
  PROBE_RUNS = 5

  def __init__(self, prepared: IdleBudget):
    # Retain the artifact profile AND normal startup replay measurements. This
    # floor is never weakened; only later, completed-within-deadline samples age.
    self.__dict__.update(prepared.__dict__)
    self.floor = prepared.estimate
    self.samples = deque(maxlen=1200)
    self.probes = 0

  def admit(self, now, camera_pending=False, *, min_interval=MIN_INTERVAL):
    if self.settled >= 20 and not self.disabled_reason:
      while self.samples and now-self.samples[0][0] > self.WINDOW:
        self.samples.popleft()
      estimate = max(self.floor, self.samples[0][1] if self.samples else 0.)
      if estimate < self.estimate:
        self.probes = self.PROBE_RUNS
      self.estimate = estimate
    # Revalidate at 1 Hz before returning to every-frame admission. All primary
    # cadence, pending-camera, complete-job reservation and latch checks remain.
    return super().admit(now, camera_pending, min_interval=max(min_interval, self.PROBE_INTERVAL if self.probes else 0.))

  def finish(self, start, end):
    super().finish(start, end)
    measured = (end-start)*RUNTIME_MARGIN
    # Monotonic maximum queue: no scan of 30 seconds of samples per primary frame.
    while self.samples and self.samples[-1][1] <= measured:
      self.samples.pop()
    self.samples.append((end, measured))
    if self.disabled_reason:
      # An actual deadline miss is stronger evidence than an inflated margin.
      # Preserve its reservation even after supervisor-authorized latch recovery.
      self.floor = max(self.floor, self.estimate)
    elif self.probes:
      self.probes -= 1
