import math
from itertools import pairwise

import pytest

from opendbc.car.hyundai.steering_handover import SteeringHandover


class Trace:
  def __init__(self, mode=3):
    self.controller = SteeringHandover()
    self.time = 0.0
    self.args = dict(mode=mode, baseline=25.0, minimum=25.0, maximum=250.0,
                     driver=300.0, threshold=250.0, pressed=True, target_error=0.5,
                     command_error=0.5, speed=15.0, wheelbase=3.0, steer_ratio=14.26,
                     active=True, valid=True)

  def step(self, **kwargs):
    self.time += 0.01
    return self.controller.update(now=self.time, **(self.args | kwargs))

  def run(self, frames, **kwargs):
    return [self.step(**kwargs) for _ in range(frames)]


@pytest.mark.parametrize("mode", [0, -1, 4, 100])
def test_default_and_unknown_modes_are_exact_passthrough(mode):
  trace = Trace(mode)
  for tick in range(200):
    baseline = tick * 1.234
    assert trace.step(baseline=baseline, valid=False, driver=float("nan")) == baseline
  assert trace.step(baseline=0, active=False) == 0


@pytest.mark.parametrize("mode", [1, 3])
def test_offer_is_bounded_and_stops_without_driver_response(mode):
  trace = Trace(mode)
  caps = trace.run(200)
  assert max(caps) == pytest.approx(80)
  assert max(b - a for a, b in pairwise(caps)) <= 1.100001
  assert caps[-1] == 25
  assert trace.controller.state == "blocked"
  assert max(trace.run(500)) == 25  # no repeated nudges while held


@pytest.mark.parametrize("strength,ceiling", [(1.2, 80), (1.8, 55), (2.5, 35), (3.5, 25)])
def test_stronger_driver_force_reduces_offer(strength, ceiling):
  trace = Trace(1)
  assert max(trace.run(150, driver=250 * strength)) == pytest.approx(ceiling)
  assert trace.controller.effort == pytest.approx(strength)  # not clipped at 2


@pytest.mark.parametrize("veto", [dict(driver=-300), dict(driver=500), dict(target_error=5),
                                  dict(command_error=-5), dict(valid=False), dict(active=False, baseline=0)])
def test_offer_yields_to_reversal_force_error_and_invalidity(veto):
  trace = Trace(1)
  assert max(trace.run(115)) > 70
  before = trace.controller.cap
  first = trace.step(**veto)
  assert first <= before - 19.999
  assert trace.run(8, **veto)[-1] == veto.get("baseline", 25)


def test_alternating_force_never_looks_like_release_or_offer():
  trace = Trace(3)
  for tick in range(500):
    assert trace.step(driver=300 * (-1) ** tick) == 25
  assert trace.controller.effort > 1


def test_threshold_noise_does_not_chatter_offers():
  trace = Trace(1)
  states = []
  for tick in range(300):
    cap = trace.step(driver=300 + 15 * math.sin(tick * 1.2))
    assert 25 <= cap <= 80
    states.append(trace.controller.state)
  assert sum(a != b for a, b in pairwise(states)) <= 3


@pytest.mark.parametrize("mode", [2, 3])
def test_abrupt_release_has_confirmation_and_bounded_ramp(mode):
  trace = Trace(mode)
  trace.run(150)
  caps = trace.run(20, driver=0, pressed=False)
  first = next(i for i, cap in enumerate(caps) if cap > 25)
  assert 10 <= first <= 13
  assert caps[first] <= 29.500001
  assert max(b - a for a, b in pairwise(caps)) <= 4.500001
  assert trace.run(70, driver=0, pressed=False)[-1] == 250
  # Even after full rapid recovery, renewed driver effort cancels the addition.
  assert trace.run(15, driver=-350, pressed=True)[-1] == 25


@pytest.mark.parametrize("duration", [1, 6, 9])
def test_zero_crossing_does_not_trigger_rapid_recovery(duration):
  trace = Trace(2)
  trace.run(100)
  assert max(trace.run(duration, driver=0, pressed=False)) == 25
  assert max(trace.run(100, driver=-350)) == 25


def test_long_zero_crossing_is_ambiguous_but_opposition_cancels():
  trace = Trace(2)
  trace.run(100)
  assert trace.run(16, driver=0, pressed=False)[-1] > 25
  assert trace.run(10, driver=-350)[-1] == 25


def test_slow_unloading_and_partial_release_do_not_trigger_fast_path():
  trace = Trace(2)
  trace.run(100)
  for tick in range(100):
    driver = max(0, 300 - tick * 3)
    assert trace.step(driver=driver, pressed=driver > 250) == 25
  assert max(trace.run(50, driver=0, pressed=False)) == 25


@pytest.mark.parametrize("overrides", [dict(target_error=3), dict(command_error=3), dict(valid=False),
                                       dict(target_error=1.5, speed=45)])
def test_rapid_release_cannot_bypass_error_speed_and_validity_gates(overrides):
  trace = Trace(2)
  trace.run(100)
  assert max(trace.run(100, driver=0, pressed=False, **overrides)) == 25


def test_mode_one_holds_offer_until_legacy_catches_up():
  trace = Trace(1)
  trace.run(80)
  caps = trace.run(30, driver=0, pressed=False)
  assert trace.controller.state == "handoff"
  assert 25 < caps[-1] <= 80
  assert trace.run(20, driver=0, pressed=False, baseline=90)[-1] == 90


def test_combined_transfers_offer_to_rapid_without_adding_gains():
  trace = Trace(3)
  trace.run(80)
  assert trace.controller.state == "offering"
  caps = trace.run(50, driver=0, pressed=False)
  assert trace.controller.state == "rapid"
  assert max(b - a for a, b in pairwise(caps)) <= 4.500001
  assert caps[-1] > 80


@pytest.mark.parametrize("bad", [dict(driver=float("nan")), dict(threshold=0), dict(steer_ratio=0), dict(valid=False)])
def test_bad_input_clears_release_history(bad):
  trace = Trace(2)
  trace.run(100)
  trace.step(**bad)
  assert max(trace.run(100, driver=0, pressed=False)) == 25


def test_timing_gap_and_mode_change_require_new_evidence():
  trace = Trace(2)
  trace.run(100)
  trace.time += 0.1
  assert max(trace.run(100, driver=0, pressed=False)) == 25
  trace.run(100)
  assert max(trace.run(100, mode=3, driver=0, pressed=False)) == 25
  assert trace.step(mode=0, baseline=27.25) == 27.25


def test_invalidity_cannot_rearm_an_interrupted_offer():
  trace = Trace(1)
  trace.run(115)
  assert trace.controller.state == "offering"
  trace.run(50, valid=False)
  assert max(trace.run(200)) == 25
  trace.run(100, driver=0, pressed=False)
  assert max(trace.run(200)) > 25


def test_legacy_full_authority_is_never_lowered_by_offer_logic():
  trace = Trace(3)
  assert min(trace.run(300, baseline=250)) == 250
