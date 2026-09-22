"""Low-priority display placement, including workers and CPU hotplug transitions."""
import os
import sys
import time
from pathlib import Path

LITTLE_CORES = {0, 1, 2, 3}
DISPLAY_NICE = 19


def core_online(core: int) -> bool:
  try:
    return Path(f'/sys/devices/system/cpu/cpu{core}/online').read_text().strip() == '1'
  except OSError:
    return False


def thread_ids(pid: int | str = 'self') -> list[int]:
  try:
    return [int(p.name) for p in Path(f'/proc/{pid}/task').iterdir()]
  except FileNotFoundError:
    return []  # Encoder exited before its worker sweep.


class DisplayScheduler:
  def __init__(self, onroad_core: int, *, enabled: bool):
    self.core = onroad_core
    self.enabled = enabled and sys.platform == 'linux'
    self.onroad = None
    self.next_check = 0.0

  def update(self, onroad: bool, *, force: bool = False, child_pid: int | None = None) -> None:
    if not self.enabled:
      return
    now = time.monotonic()
    if not force and onroad == self.onroad and now < self.next_check:
      return
    self.onroad = onroad
    self.next_check = now + 0.5
    use_big = onroad and core_online(self.core)
    cores = {self.core} if use_big else LITTLE_CORES
    nice = DISPLAY_NICE if use_big else 0
    workers = thread_ids()
    if child_pid is not None:
      workers.extend(thread_ids(child_pid))
    for tid in workers:
      try:
        # Lower display priority before moving onto camera/model CPUs. New
        # workers inherit this policy and are checked on the next sweep.
        if os.sched_getscheduler(tid) != os.SCHED_OTHER:
          os.sched_setscheduler(tid, os.SCHED_OTHER, os.sched_param(0))
        if use_big and os.getpriority(os.PRIO_PROCESS, tid) != nice:
          os.setpriority(os.PRIO_PROCESS, tid, nice)
        if os.sched_getaffinity(tid) != cores:
          try:
            os.sched_setaffinity(tid, cores)
          except OSError:
            # A power-save transition can offline the target after the check.
            os.sched_setaffinity(tid, LITTLE_CORES)
        if not use_big and os.getpriority(os.PRIO_PROCESS, tid) != nice:
          try:
            os.setpriority(os.PRIO_PROCESS, tid, nice)
          except PermissionError:
            # Older service limits may prohibit raising nice again. Retaining
            # low priority is safe; never leave the worker pinned to an offline CPU.
            pass
      except ProcessLookupError:
        pass  # Worker exited during enumeration.
