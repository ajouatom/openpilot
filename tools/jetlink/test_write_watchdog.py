import queue
import sys
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'third_party/jetlink'))
from jetlink.transport import watchdog


class ObservedCondition(threading.Condition):
  def __init__(self):
    super().__init__()
    self.waits = queue.Queue()
    self.notifications = 0

  def wait(self, timeout=None):
    self.waits.put(timeout)
    return super().wait(timeout)

  def notify(self, n=1):
    self.notifications += 1
    return super().notify(n)


@pytest.fixture
def running_guard(monkeypatch):
  now = [0.]
  aborted = threading.Event()
  monkeypatch.setattr(watchdog, 'background_thread', lambda: None)
  monkeypatch.setattr(watchdog, 'threading', SimpleNamespace(Condition=ObservedCondition, Thread=threading.Thread))
  monkeypatch.setattr(watchdog, 'time', SimpleNamespace(monotonic=lambda: now[0]))
  guard = watchdog.WriteWatchdog(aborted.set)
  assert guard._cv.waits.get(timeout=2) is None
  try:
    yield guard, now, aborted
  finally:
    guard.close()
    guard.thread.join(timeout=2)
    assert not guard.thread.is_alive()


def wake_at(guard, now, stamp):
  with guard._cv:
    now[0] = stamp
    # Drive virtual time, without counting this test-only wake as production IO.
    threading.Condition.notify(guard._cv)


def test_many_completed_writes_share_the_earliest_watchdog_wake(running_guard):
  guard, now, aborted = running_guard
  assert guard.arm(15)
  assert guard._cv.waits.get(timeout=2) == 15
  notifications = guard._cv.notifications
  assert guard.disarm()
  for index in range(1000):
    now[0] = (index + 1) / 1000
    assert guard.arm(15)
    assert guard.disarm()
  assert guard._cv.notifications == notifications
  assert not aborted.is_set()
  # Cancellation still prevents expiry at the old wake.
  wake_at(guard, now, 15)
  assert guard._cv.waits.get(timeout=2) is None
  assert not aborted.is_set()


def test_earlier_budget_interrupts_existing_later_wait(running_guard):
  guard, now, aborted = running_guard
  assert guard.arm(15)
  assert guard._cv.waits.get(timeout=2) == 15
  assert guard.disarm()
  assert guard.arm(2)
  assert guard._cv.waits.get(timeout=2) == 2
  wake_at(guard, now, 2)
  assert aborted.wait(2)
  assert not guard.arm(1)
  assert not guard.disarm()


def test_old_deadline_does_not_expire_a_rearmed_write(running_guard):
  guard, now, aborted = running_guard
  assert guard.arm(1)
  assert guard._cv.waits.get(timeout=2) == 1
  assert guard.disarm()
  now[0] = .25
  assert guard.arm(2)
  wake_at(guard, now, 1)
  assert guard._cv.waits.get(timeout=2) == 1.25
  assert not aborted.is_set()
  wake_at(guard, now, 2.25)
  assert aborted.wait(2)


def test_idle_guard_rearms_and_spurious_wake_keeps_deadline(running_guard):
  guard, now, aborted = running_guard
  assert guard.arm(1)
  assert guard._cv.waits.get(timeout=2) == 1
  assert guard.disarm()
  wake_at(guard, now, 1)
  assert guard._cv.waits.get(timeout=2) is None
  assert guard.arm(2)
  assert guard._cv.waits.get(timeout=2) == 2
  wake_at(guard, now, 1.5)
  assert guard._cv.waits.get(timeout=2) == 1.5
  assert not aborted.is_set()
  wake_at(guard, now, 3)
  assert aborted.wait(2)


def test_concurrent_write_rejected_and_close_wakes_future_timer(running_guard):
  guard, _, aborted = running_guard
  assert guard.arm(100)
  assert guard._cv.waits.get(timeout=2) == 100
  with pytest.raises(RuntimeError, match='concurrent writes'):
    guard.arm(1)
  guard.close()
  guard.thread.join(timeout=2)
  assert not guard.thread.is_alive()
  assert not aborted.is_set()
  assert not guard.arm(1)
  assert not guard.disarm()


def test_expiry_callback_does_not_hold_cancellation_lock(monkeypatch):
  entered, release = threading.Event(), threading.Event()
  monkeypatch.setattr(watchdog, 'background_thread', lambda: None)
  def abort():
    entered.set()
    release.wait(2)
  guard = watchdog.WriteWatchdog(abort)
  try:
    assert guard.arm(.02)
    assert entered.wait(2)
    assert not guard.disarm()
    assert not guard.arm(1)
    guard.close()
  finally:
    release.set()
    guard.close()
    guard.thread.join(timeout=2)
  assert not guard.thread.is_alive()
