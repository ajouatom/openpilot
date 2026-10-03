"""Paced, incremental PSS sampling; lightweight procLog snapshots never scan smaps."""
import time
from dataclasses import dataclass
from pathlib import Path
from typing import BinaryIO

REFRESH_SECONDS = 40.0
MAX_AGE_SECONDS = 120.0
READ_BYTES = 32 * 1024
MIN_PAUSE_SECONDS = 0.005
CPU_DUTY_FRACTION = 0.05
MIN_RSS_BYTES = 5 * 1024 * 1024


@dataclass(frozen=True)
class SmapsSample:
  pss: int = 0
  pss_anon: int = 0
  pss_shmem: int = 0
  # Conservative age: beginning of a multi-read scan, not its completion.
  mono_time: float = 0.0


class SmapsSampler:
  def __init__(self, root: Path = Path('/proc')):
    self.root = root
    self.processes: dict[int, int] = {}  # pid -> starttime ticks, protects PID reuse
    self.cache: dict[int, SmapsSample] = {}
    self.due: dict[int, float] = {}
    self.next_step = 0.0
    self.active: tuple[int, int] | None = None
    self.file: BinaryIO | None = None
    self.started = 0.0
    self.pending = b''
    self.totals = [0, 0, 0]

  def _starttime(self, pid: int) -> int:
    # Fields after the final ')' begin with field 3; starttime is field 22.
    return int((self.root / str(pid) / 'stat').read_text().rsplit(')', 1)[1].split()[19])

  def update(self, processes: dict[int, int]) -> None:
    """Publish eligibility from the ordinary stat snapshot; do no smaps IO here."""
    for pid in list(self.processes):
      if processes.get(pid) != self.processes[pid]:
        self.cache.pop(pid, None)
        self.due.pop(pid, None)
    if self.active is not None and processes.get(self.active[0]) != self.active[1]:
      self.close()
    self.processes = processes.copy()
    for pid in processes:
      self.due.setdefault(pid, 0.0)

  def get(self, pid: int, starttime: int, now: float) -> SmapsSample:
    sample = self.cache.get(pid, SmapsSample())
    if self.processes.get(pid) != starttime or now - sample.mono_time > MAX_AGE_SECONDS:
      return SmapsSample()
    return sample

  def close(self) -> None:
    if self.file is not None:
      self.file.close()
    self.file = None
    self.active = None
    self.pending = b''

  def _parse(self, data: bytes, *, eof: bool = False) -> None:
    lines = (self.pending + data).split(b'\n')
    self.pending = b'' if eof else lines.pop()
    for line in lines:
      if line.startswith((b'Pss:', b'Pss_Anon:', b'Pss_Shmem:')):
        key, amount, *_ = line.split()
        index = {b'Pss:': 0, b'Pss_Anon:': 1, b'Pss_Shmem:': 2}[key]
        self.totals[index] += int(amount) * 1024
    # Malformed input must not create an unbounded buffer.
    if len(self.pending) > READ_BYTES:
      raise ValueError('oversized smaps line')

  def step(self) -> None:
    """At most one bounded read, then a real pause; never catch up missed slots.

    The duty budget is for this sampler's thread CPU, not whole-core utilization.
    A single kernel read cannot be preempted by this userspace budget.
    """
    now = time.monotonic()
    if now < self.next_step:
      return
    cpu_start = time.thread_time()
    try:
      if self.active is None:
        pid = min(self.due, key=self.due.get) if self.due else None
        if pid is None or self.due[pid] > now:
          return
        identity = self.processes[pid]
        self.due[pid] = now + REFRESH_SECONDS
        if self._starttime(pid) != identity:
          self.cache.pop(pid, None)
          return
        self.active = (pid, identity)
        self.started = now
        self.totals = [0, 0, 0]
        # Preserve rollup-only Pss_Anon/Pss_Shmem where supported. Its kernel
        # walk is indivisible, but other processes still wait for the duty pause.
        # Older kernels (including C3's 4.9 base) use chunked per-VMA smaps.
        try:
          self.file = (self.root / str(pid) / 'smaps_rollup').open('rb', buffering=0)
        except FileNotFoundError:
          self.file = (self.root / str(pid) / 'smaps').open('rb', buffering=0)
      assert self.file is not None
      pid, identity = self.active
      if now - self.started > MAX_AGE_SECONDS:
        self.due[pid] = now + REFRESH_SECONDS
        self.close()
        return
      data = self.file.read(READ_BYTES)
      self._parse(data, eof=not data)
      if not data:
        if self._starttime(pid) == identity:
          self.cache[pid] = SmapsSample(*self.totals, self.started)
        else:
          self.cache.pop(pid, None)
        self.due[pid] = time.monotonic() + REFRESH_SECONDS
        self.close()
    except (OSError, ValueError, IndexError):
      # Exit/permission races and malformed reads must not publish partial sums.
      self.close()
    finally:
      cpu_used = max(0.0, time.thread_time() - cpu_start)
      pause = max(MIN_PAUSE_SECONDS, cpu_used * (1.0 / CPU_DUTY_FRACTION - 1.0))
      self.next_step = time.monotonic() + pause
