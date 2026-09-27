import pytest

from opendbc.car.hyundai.section_average import SECTION_END_DEBOUNCE_S, SectionAverage


def _drive(avg, speeds_kph, dt=0.05, start_time=0.0, start_distance=0.0, limit=80):
  """Integrate a speed profile; the allowance caps each step like the section cap does."""
  now, distance = start_time, start_distance
  for requested in speeds_kph:
    avg.update(True, limit, now, distance)
    allowance = avg.allowance_kph(now, distance)
    speed = min(requested, max(limit, allowance))
    now += dt
    distance += speed / 3.6 * dt
  return now, distance


def _unlocked(limit=80):
  avg = SectionAverage()
  avg.update(True, limit, 0.0, 0.0)
  avg.unlock()
  return avg


def test_locked_section_never_allows_more_than_the_limit():
  avg = SectionAverage()
  now, distance = _drive(avg, [30] * int(60 / 0.05))
  assert avg.bank_seconds(now, distance) > 30
  assert avg.allowance_kph(now, distance) == 0.0


def test_no_bank_means_no_allowance():
  avg = _unlocked()
  assert avg.allowance_kph(10.0, 80 / 3.6 * 10.0) == 0.0


def test_congestion_banks_time_and_allows_more_than_the_limit():
  avg = _unlocked()
  now, distance = _drive(avg, [30] * int(60 / 0.05))         # one minute at 30 km/h
  assert avg.bank_seconds(now, distance) == pytest.approx(60 - 500 / (80 / 3.6), rel=0.01)
  assert avg.allowance_kph(now, distance) > 80


def test_running_average_never_exceeds_the_limit_while_spending_the_bank():
  avg = _unlocked()
  now, distance = _drive(avg, [20] * int(90 / 0.05))         # congestion
  for _ in range(int(600 / 0.05)):                           # then the driver asks for 130
    avg.update(True, 80, now, distance)
    allowance = avg.allowance_kph(now, distance)
    if allowance == 0.0:
      break                                                   # bank spent: back to the normal cap
    now += 0.05
    distance += min(130, max(80, allowance)) / 3.6 * 0.05
    assert distance / now * 3.6 <= 80 + 0.5
  assert avg.bank_seconds(now, distance) <= 1.0


def test_lock_and_section_change_restore_the_cap():
  avg = _unlocked()
  now, distance = _drive(avg, [30] * int(60 / 0.05))
  avg.lock()
  assert avg.allowance_kph(now, distance) == 0.0
  avg.unlock()
  avg.update(True, 100, now, distance)                        # new limit: a new section, locked again
  assert not avg.unlocked and (avg.limit_kph, avg.start_time) == (100, now)


def test_short_dropout_keeps_the_section_but_a_real_exit_resets():
  avg = _unlocked()
  avg.update(False, 0, 10.0, 100.0)
  avg.update(True, 80, 10.0 + SECTION_END_DEBOUNCE_S / 2, 110.0)
  assert avg.active and avg.unlocked and avg.start_time == 0.0
  avg.update(False, 0, 20.0, 200.0)
  avg.update(False, 0, 20.0 + SECTION_END_DEBOUNCE_S, 210.0)
  assert not avg.active and not avg.unlocked


def test_unlock_needs_an_active_section():
  avg = SectionAverage()
  avg.unlock()
  assert not avg.unlocked
