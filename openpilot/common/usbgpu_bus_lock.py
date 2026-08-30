from __future__ import annotations

import contextlib
import os
import threading
from collections.abc import Iterator

try:
  import fcntl
except ImportError:  # pragma: no cover - Windows development fallback
  fcntl = None  # type: ignore[assignment]


USBGPU_BUS_LOCK_PATH = "/tmp/carrot_usbgpu_bus.lock"

_process_lock = threading.RLock()
_thread_state = threading.local()
_lock_fd: int | None = None
_lock_pid: int | None = None


def _shared_lock_fd() -> int:
  global _lock_fd, _lock_pid

  pid = os.getpid()
  if _lock_fd is None or _lock_pid != pid:
    if _lock_fd is not None:
      with contextlib.suppress(OSError):
        os.close(_lock_fd)
    _lock_fd = os.open(USBGPU_BUS_LOCK_PATH, os.O_CREAT | os.O_RDWR, 0o666)
    _lock_pid = pid
  return _lock_fd


@contextlib.contextmanager
def usbgpu_bus_lock() -> Iterator[None]:
  """Serialize eGPU and external-display libusb calls across processes.

  The lock is reentrant for nested eGPU transactions such as a vendor-control
  setup followed by a bulk transfer. On non-POSIX development hosts it still
  serializes threads, while TICI uses flock for interprocess exclusion.
  """
  with _process_lock:
    depth = getattr(_thread_state, "depth", 0)
    if depth == 0 and fcntl is not None:
      fcntl.flock(_shared_lock_fd(), fcntl.LOCK_EX)
    _thread_state.depth = depth + 1
    try:
      yield
    finally:
      next_depth = _thread_state.depth - 1
      _thread_state.depth = next_depth
      if next_depth == 0 and fcntl is not None:
        fcntl.flock(_shared_lock_fd(), fcntl.LOCK_UN)
