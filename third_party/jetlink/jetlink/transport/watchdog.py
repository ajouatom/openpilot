"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.
"""
from __future__ import annotations

import threading
import time

from jetlink.transport.priority import background_thread


class WriteWatchdog:
  """One worker per link, with cancellation serialized against expiry.

  Once expiry wins, the link cannot be armed again. The callback runs outside
  the lock: unbinding a broken USB controller must not block disarm().
  """

  def __init__(self, abort):
    self._abort = abort
    self._cv = threading.Condition()
    self._deadline = None
    self._expired = False
    self._closed = False
    self.thread = threading.Thread(target=self._run, name='jetlink-write-guard', daemon=True)
    self.thread.start()

  def arm(self, budget: float) -> bool:
    with self._cv:
      if self._expired or self._closed:
        return False
      if self._deadline is not None:
        raise RuntimeError('concurrent writes on one link')
      self._deadline = time.monotonic() + budget
      self._cv.notify()
      return True

  def disarm(self) -> bool:
    with self._cv:
      self._deadline = None
      self._cv.notify()
      return not self._expired and not self._closed

  def close(self) -> None:
    with self._cv:
      self._closed = True
      self._deadline = None
      self._cv.notify()

  def _run(self) -> None:
    # This inherits the creator's policy and pin, which in modeld is the frame
    # loop's SCHED_FIFO 54 on one core.
    background_thread()
    with self._cv:
      while not self._closed:
        if self._deadline is None:
          self._cv.wait()
          continue
        remaining = self._deadline - time.monotonic()
        if remaining > 0:
          self._cv.wait(remaining)
          continue
        self._expired = True
        self._deadline = None
        break
      else:
        return
    self._abort()
