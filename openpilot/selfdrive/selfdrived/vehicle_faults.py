"""Escalate current controller errors; historical counters alone are not faults."""

from collections import deque
import math

CAN_CONFIRM_S = 0.5
CAN_ERROR_RATE = 100.0
CAN_SAMPLE_GAP_S = 0.5
RECOVERY_S = 2.0
SCC_WINDOW_S = 1.0
SCC_CONFIRM_S = 0.10


class VehicleFaultMonitor:
  def __init__(self):
    self.previous = {}
    self.can_since = {}
    self.can_last_bad = None
    self.can_good_since = None
    self.can_fault = False
    self.scc_samples = deque()
    self.scc_last_time = None
    self.scc_last_bad = None
    self.scc_good_since = None
    self.scc_fault = False

  def update_pandas(self, pandas, sample_time, *, valid):
    if not valid or not math.isfinite(sample_time):
      self.can_good_since = None
      self.can_since.clear()
      return self.can_fault
    growing = False
    confirmed = False
    fresh = False
    healthy = bool(pandas)
    keys = set()
    for index, panda in enumerate(pandas):
      faults = {str(f) for f in panda.faults}
      for bus in range(3):
        key = (index, bus)
        keys.add(key)
        state = getattr(panda, f'canState{bus}')
        healthy &= not (state.errorPassive or state.busOff)
        count = state.totalErrorCnt
        previous = self.previous.get(key)
        if previous is not None and sample_time <= previous[0]:
          continue
        fresh = True
        self.previous[key] = (sample_time, count, panda.uptime)
        dt = sample_time - previous[0] if previous is not None else 0.0
        if previous is None or dt > CAN_SAMPLE_GAP_S or panda.uptime < previous[2] or count < previous[1]:
          self.can_good_since = None
        current = (
          previous is not None
          and 0 < dt <= CAN_SAMPLE_GAP_S
          and panda.uptime >= previous[2]
          and count >= previous[1]
          and (count - previous[1]) / dt >= CAN_ERROR_RATE
          and (state.errorPassive or state.busOff)
          and f'interruptRateCan{bus + 1}' in faults
        )
        if current:
          growing = True
          since = self.can_since.setdefault(key, previous[0])
          confirmed |= sample_time - since >= CAN_CONFIRM_S
        else:
          self.can_since.pop(key, None)
    for key in set(self.previous) - keys:
      self.previous.pop(key, None)
      self.can_since.pop(key, None)
    if fresh:
      if growing:
        self.can_last_bad = sample_time
        self.can_good_since = None
      elif not healthy:
        self.can_good_since = None
      elif self.can_good_since is None:
        self.can_good_since = sample_time
      if confirmed:
        self.can_fault = True
      elif self.can_fault and self.can_good_since is not None and sample_time - self.can_good_since >= RECOVERY_S:
        self.can_fault = False
    return self.can_fault

  def update_scc(self, fault, sample_time, *, valid):
    if not valid or not math.isfinite(sample_time):
      self.scc_good_since = None
      self.scc_samples.clear()
      return self.scc_fault
    previous = self.scc_last_time
    if previous is not None and sample_time <= previous:
      return self.scc_fault
    self.scc_last_time = sample_time
    dt = sample_time - previous if previous is not None else 0.0
    if dt > CAN_SAMPLE_GAP_S:
      self.scc_samples.clear()
      self.scc_good_since = None
      dt = 0.0
    if fault:
      self.scc_last_bad = sample_time
      self.scc_good_since = None
    elif self.scc_good_since is None:
      self.scc_good_since = sample_time
    self.scc_samples.append((sample_time, min(dt, 0.05) if fault else 0.0))
    while self.scc_samples and sample_time - self.scc_samples[0][0] > SCC_WINDOW_S:
      self.scc_samples.popleft()
    if sum(duration for _, duration in self.scc_samples) >= SCC_CONFIRM_S - 1e-9:
      self.scc_fault = True
    elif self.scc_fault and self.scc_good_since is not None and sample_time - self.scc_good_since >= RECOVERY_S:
      self.scc_fault = False
    return self.scc_fault


def inhibit_faulted_controls(control, events):
  """AlwaysLateral must not bypass a confirmed vehicle-controller failure."""
  if any(str(event.name) in ('pandaCanFault', 'stockSccFault') for event in events):
    control.enabled = control.latActive = control.longActive = False
