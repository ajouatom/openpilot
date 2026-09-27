import pytest

from openpilot.selfdrive.carrot.section_average import SECTION_END_DEBOUNCE_S, SectionAverage


def _drive(avg, speeds_kph, dt=0.05, margin=0, start_time=0.0, start_distance=0.0, limit=80):
  """Integrate a speed profile; the allowance caps each step like carrot_serv does."""
  now, distance = start_time, start_distance
  for requested in speeds_kph:
    avg.update(True, limit, now, distance)
    allowance = avg.allowance_kph(now, distance, margin)
    speed = min(requested, max(limit, allowance))
    now += dt
    distance += speed / 3.6 * dt
  return now, distance


def test_no_bank_means_no_allowance():
  avg = SectionAverage()
  avg.update(True, 80, 0.0, 0.0)
  assert avg.allowance_kph(10.0, 80 / 3.6 * 10.0, 0) == 0.0


def test_congestion_banks_time_and_allows_more_than_the_limit():
  avg = SectionAverage()
  now, distance = _drive(avg, [30] * int(60 / 0.05))         # one minute at 30 km/h
  assert avg.bank_seconds(now, distance, 0) == pytest.approx(60 - 500 / (80 / 3.6), rel=0.01)
  assert avg.allowance_kph(now, distance, 0) > 80


def test_running_average_never_exceeds_the_target_while_spending_the_bank():
  avg = SectionAverage()
  now, distance = _drive(avg, [20] * int(90 / 0.05))         # congestion
  for _ in range(int(600 / 0.05)):                           # then the driver asks for 130
    avg.update(True, 80, now, distance)
    speed = min(130, max(80, avg.allowance_kph(now, distance, 0)))
    if avg.allowance_kph(now, distance, 0) == 0.0:
      break                                                   # bank spent: back to the normal cap
    now += 0.05
    distance += speed / 3.6 * 0.05
    assert distance / now * 3.6 <= 80 + 0.5
  assert avg.bank_seconds(now, distance, 0) <= 1.0


def test_margin_lowers_the_target_average():
  avg = SectionAverage()
  now, distance = _drive(avg, [77] * int(60 / 0.05))
  assert avg.allowance_kph(now, distance, 0) > 0.0          # 77 < 80 banks time
  assert avg.allowance_kph(now, distance, 3) == 0.0         # 77 = 80 - 3 banks nothing


def test_short_dropout_keeps_the_section_but_a_real_exit_resets():
  avg = SectionAverage()
  avg.update(True, 80, 0.0, 0.0)
  avg.update(False, 0, 10.0, 100.0)
  avg.update(True, 80, 10.0 + SECTION_END_DEBOUNCE_S / 2, 110.0)
  assert avg.active and avg.start_time == 0.0
  avg.update(False, 0, 20.0, 200.0)
  avg.update(False, 0, 20.0 + SECTION_END_DEBOUNCE_S, 210.0)
  assert not avg.active


def test_limit_change_restarts_the_section():
  avg = SectionAverage()
  avg.update(True, 80, 0.0, 0.0)
  avg.update(True, 100, 30.0, 500.0)
  assert (avg.limit_kph, avg.start_time, avg.start_distance) == (100, 30.0, 500.0)
