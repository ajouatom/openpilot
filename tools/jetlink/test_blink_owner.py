from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from blink_owner import Owner, GRACE, current


def test_lease_rejects_invalid_or_unbounded_deadlines():
  for record in ({}, None, {'until':float('nan')}, {'until':float('inf')}, {'until':99}, {'until':True}):
    assert not current(record, 10.)
  assert current({'until':10.5}, 10.)


@pytest.mark.skipif(sys.platform != 'linux', reason='Real fcntl ownership')
@pytest.mark.parametrize('first', ['S3 BLE', 'CANable 2.0', 'PEAK PCAN-USB'])
def test_first_ready_owns_and_departure_waits_for_old_receiver_expiry(tmp_path, first):
  clock = [10.]
  a, b = Owner(first, tmp_path, lambda: clock[0]), Owner('other', tmp_path, lambda: clock[0])
  assert not a.acquire(False)
  assert a.acquire(True)
  assert not b.acquire(True)
  clock[0] += .1
  assert a.acquire(True)
  a.release()
  assert not b.acquire(True)
  clock[0] += GRACE+.001
  assert b.acquire(True)
  assert not a.acquire(True)


@pytest.mark.skipif(sys.platform != 'linux', reason='Real fcntl ownership')
def test_stalled_owner_cannot_resume_after_other_adapter_takes_over(tmp_path):
  clock = [10.]
  a, b = Owner('S3', tmp_path, lambda: clock[0]), Owner('CANable', tmp_path, lambda: clock[0])
  assert a.acquire(True)
  clock[0] += GRACE+.01
  assert b.acquire(True)
  assert not a.acquire(True)
  assert not a.held and b.held
  # A's cleanup must not erase B's newer lease.
  a.release()
  clock[0] += .1
  assert b.acquire(True)


def _competing_owner(directory, start, results, name):
  owner = Owner(name, directory)
  start.wait(5)
  results.put(owner.acquire(True))


@pytest.mark.skipif(sys.platform != 'linux', reason='Real fcntl ownership')
def test_simultaneous_processes_choose_exactly_one_output(tmp_path):
  import multiprocessing
  context = multiprocessing.get_context('fork')
  start, results = context.Event(), context.Queue()
  processes = [context.Process(target=_competing_owner, args=(tmp_path, start, results, name))
               for name in ('S3 BLE', 'CANable 2.0', 'PEAK PCAN-USB')]
  try:
    for process in processes:
      process.start()
    start.set()
    assert sum(results.get(timeout=5) for _ in processes) == 1
    for process in processes:
      process.join(5)
      assert process.exitcode == 0
  finally:
    for process in processes:
      if process.is_alive():
        process.kill()
        process.join()
