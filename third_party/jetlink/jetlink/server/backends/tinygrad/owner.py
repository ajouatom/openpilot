"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

The one thread that touches tinygrad, for the life of the process.

Measured on Metal (macOS 25.5, tinygrad 0.14): a JIT unpickled on one thread
segfaults when replayed from another, and a thread that has used Metal
segfaults in objc's autorelease-pool drain the moment it exits
(AutoreleasePoolPage::releaseUntil under _pthread_exit in the crash report).
The server loads engines on a short-lived job thread and runs frames on the
session thread, which is exactly the pattern that dies. So every tinygrad
call - build, load, run, close - is handed to this thread, and this thread
never exits: it is a daemon, and the process leaves without joining it, which
is the one way out that runs no thread-local destructor.

The handoff is a queue put and a future wait, tens of microseconds against a
frame of tens of milliseconds.
"""
from __future__ import annotations

import queue
import threading
from collections.abc import Callable
from concurrent.futures import Future
from typing import Any

_lock = threading.Lock()
_owner: _Owner | None = None


class _Owner:
  def __init__(self):
    self._q: queue.Queue = queue.Queue()
    self.thread = threading.Thread(target=self._loop, name='jetlink-tinygrad', daemon=True)
    self.thread.start()

  def call(self, fn: Callable[..., Any], *args, **kwargs) -> Any:
    if threading.current_thread() is self.thread:
      return fn(*args, **kwargs)   # already here; nested calls must not deadlock
    box: Future = Future()
    self._q.put((fn, args, kwargs, box))
    return box.result()

  def _loop(self) -> None:
    while True:
      fn, args, kwargs, box = self._q.get()
      try:
        box.set_result(fn(*args, **kwargs))
      except BaseException as e:
        box.set_exception(e)


def owner() -> _Owner:
  global _owner
  with _lock:
    if _owner is None:
      _owner = _Owner()
    return _owner


def on_owner(fn: Callable[..., Any], *args, **kwargs) -> Any:
  """Run `fn` on the tinygrad thread and return its result, or raise what it raised."""
  return owner().call(fn, *args, **kwargs)
