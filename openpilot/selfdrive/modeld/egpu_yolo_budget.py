"""Bounded revalidation of optional resident work after a transient slowdown."""
from collections import deque

from openpilot.selfdrive.modeld.egpu_yolo import IdleBudget, MIN_INTERVAL, RUNTIME_MARGIN


class RevalidatingBudget(IdleBudget):
  WINDOW = 30.0
  PROBE_INTERVAL = 1.0
  PROBE_RUNS = 5
  OVERRUN_BACKOFF = (30., 60., 120., 240., 300.)
  RECOVERY_SETTLED_FRAMES = 200

  def __init__(self, prepared: IdleBudget):
    # Retain the artifact profile AND normal startup replay measurements. This
    # floor is never weakened. Runtime measurements are revalidated separately.
    self.__dict__.update(prepared.__dict__)
    self.floor = prepared.estimate
    self.samples = deque(maxlen=1200)
    self.probes = 0
    self.overrun_estimate = 0.
    self.recover_after = 0.
    self.recovery_attempts = 0

  def admit(self, now, camera_pending=False, *, min_interval=MIN_INTERVAL):
    if self.settled >= 20 and not self.disabled_reason:
      while self.samples and now-self.samples[0][0] > self.WINDOW:
        self.samples.popleft()
      # Only a supervisor-authorized, completed deadline miss may be probed.
      # A permanent inflated floor otherwise prevents the very measurement
      # needed to establish recovery. Repeated misses back off up to 5 minutes;
      # require ten seconds of uninterrupted primary frames before lowering it.
      if (self.overrun_estimate and now >= self.recover_after and
          self.settled >= self.RECOVERY_SETTLED_FRAMES and not camera_pending):
        self.overrun_estimate = 0.
        self.probes = self.PROBE_RUNS
      estimate = max(self.floor, self.overrun_estimate, self.samples[0][1] if self.samples else 0.)
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
    if self.disabled_reason == 'overrun':
      self.overrun_estimate = max(self.overrun_estimate, self.estimate)
      delay = self.OVERRUN_BACKOFF[min(self.recovery_attempts, len(self.OVERRUN_BACKOFF)-1)]
      self.recover_after = end+delay
      self.recovery_attempts += 1
      self.probes = self.PROBE_RUNS
    elif self.probes:
      self.probes -= 1
