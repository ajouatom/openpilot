import threading
import time

from openpilot.common.usbgpu_bus_lock import usbgpu_bus_lock


def test_usbgpu_bus_lock_is_reentrant():
  with usbgpu_bus_lock():
    with usbgpu_bus_lock():
      pass


def test_usbgpu_bus_lock_serializes_threads():
  entered = threading.Event()
  release = threading.Event()
  acquired = []

  def holder():
    with usbgpu_bus_lock():
      entered.set()
      release.wait(timeout=1.0)

  def waiter():
    entered.wait(timeout=1.0)
    with usbgpu_bus_lock():
      acquired.append(True)

  holder_thread = threading.Thread(target=holder)
  waiter_thread = threading.Thread(target=waiter)
  holder_thread.start()
  waiter_thread.start()
  assert entered.wait(timeout=1.0)
  time.sleep(0.01)
  assert acquired == []
  release.set()
  holder_thread.join(timeout=1.0)
  waiter_thread.join(timeout=1.0)
  assert acquired == [True]
