"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.
"""
from __future__ import annotations

import os


def all_cpus() -> set[int]:
  return set(range(os.cpu_count() or 1))


def widen_affinity() -> None:
  """Let the calling thread run on every CPU.

  A thread created inside modeld inherits the frame loop's single-core pin. If
  the inherited mask is one core, prefer every other core: simply widening did
  not help, because the balancer keeps waking a thread where it last ran. Best
  effort, and open-coded because jetlink must not import openpilot.
  """
  try:
    everything = all_cpus()
    inherited = os.sched_getaffinity(0)
    if len(inherited) == 1 and everything - inherited:
      os.sched_setaffinity(0, everything - inherited)   # off the frame-loop core
    elif everything - inherited:
      os.sched_setaffinity(0, everything)               # unpinned already; just widen
  except (OSError, AttributeError):
    # no affinity call (macOS), or a kernel that refuses
    pass


def background_thread() -> None:
  """Drop the calling thread to SCHED_OTHER 0 on every CPU.

  Threads created after config_realtime_process inherit SCHED_FIFO and the core
  pin, so even one that only waits on a condition takes the frame loop's core at
  equal priority on every wake. Call it first in any thread started here. Best
  effort.
  """
  try:
    os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
  except (OSError, AttributeError, ValueError):
    pass
  widen_affinity()
