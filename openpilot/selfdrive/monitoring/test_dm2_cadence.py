import bisect

import pytest

from openpilot.selfdrive.monitoring import dm2_cadence


class Clock:
  def __init__(self, now=100.0, overshoot=0.0):
    self.now = now
    self.overshoot = overshoot

  def monotonic(self):
    return self.now

  def sleep(self, seconds):
    assert seconds > 0
    self.now += seconds + self.overshoot


def simulate(monkeypatch, disabled_seconds, processing=0.003, overshoot=0.0):
  clock = Clock(overshoot=overshoot)
  monkeypatch.setattr(dm2_cadence, 'time', clock)
  rate = dm2_cadence.DmRatekeeper(0.05)
  enabled_at = clock.now + disabled_seconds
  # Twenty camera results per second with alternating late/early inference.
  # Long gaps cause a timeout followed by an early result on the next poll.
  arrivals = [enabled_at + 0.05 * i + (0.028 if i % 2 else 0.0) for i in range(2000)]
  publications = []
  pos = 0
  while clock.now < enabled_at + 60:
    if pos < len(arrivals) and arrivals[pos] <= clock.now:
      pos = bisect.bisect_right(arrivals, clock.now)
    elif pos < len(arrivals) and arrivals[pos] <= clock.now + 0.05:
      clock.now = arrivals[pos]
      pos += 1
    else:
      clock.now += 0.05
    clock.now += processing
    publications.append(clock.now)
    rate.keep_time()
  return enabled_at, publications


@pytest.mark.parametrize('disabled_seconds', [0, 60, 780, 3600])
@pytest.mark.parametrize('overshoot', [0.0, 0.001])
def test_camera_start_after_prolonged_absence_does_not_catch_up(monkeypatch, disabled_seconds, overshoot):
  enabled_at, times = simulate(monkeypatch, disabled_seconds, overshoot=overshoot)
  # Check every rolling second, including the transition, not just total Hz.
  for t in times:
    if t >= times[0] + 1:
      count = bisect.bisect_right(times, t) - bisect.bisect_right(times, t - 1)
      assert 16 <= count <= 24
  recovered = [t for t in times if enabled_at + 10 <= t < enabled_at + 60]
  assert 16 <= len(recovered) / 50 <= 20.1


@pytest.mark.parametrize('stall', [0.1, 1.0, 30.0])
def test_scheduler_stall_is_not_repaid_with_fast_iterations(monkeypatch, stall):
  clock = Clock()
  monkeypatch.setattr(dm2_cadence, 'time', clock)
  rate = dm2_cadence.DmRatekeeper(0.05)
  for _ in range(100):
    clock.now += 0.003
    rate.keep_time()
  clock.now += stall
  rate.keep_time()
  resumed = clock.now
  for i in range(1, 101):
    clock.now += 0.003
    rate.keep_time()
    assert clock.now == pytest.approx(resumed + i * 0.05)
  assert rate.frame == 201


def test_persistent_overload_is_not_hidden(monkeypatch):
  clock = Clock()
  monkeypatch.setattr(dm2_cadence, 'time', clock)
  rate = dm2_cadence.DmRatekeeper(0.05)
  start = clock.now
  for _ in range(100):
    clock.now += 0.1
    rate.keep_time()
  # A genuinely slow producer stays below the existing 16 Hz health limit.
  assert 100 / (clock.now - start) == pytest.approx(10)
