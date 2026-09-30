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


@pytest.mark.parametrize("veto", [dict(driver=-300), dict(driver=500),
                                  dict(valid=False), dict(active=False, baseline=0)])
def test_offer_yields_to_reversal_force_and_invalidity(veto):
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
  trace.run(220)
  caps = trace.run(20, driver=0, pressed=False)
  first = next(i for i, cap in enumerate(caps) if cap > 25)
  assert 3 <= first <= 4  # limited capture precedes confirmed low-force recovery
  assert caps[first] <= 26.100001
  assert max(b - a for a, b in pairwise(caps)) <= 4.500001
  assert trace.run(70, driver=0, pressed=False)[-1] == 250
  # Even after full rapid recovery, renewed driver effort cancels the addition.
  assert trace.run(15, driver=-350, pressed=True)[-1] == 25


@pytest.mark.parametrize("duration", [1, 2, 3])
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


def test_slow_unloading_does_not_trigger_fast_path():
  trace = Trace(2)
  trace.run(100)
  for tick in range(600):
    driver = max(0, 300 - tick * 0.5)
    assert trace.step(driver=driver, pressed=driver > 250) == 25
  assert max(trace.run(50, driver=0, pressed=False)) == 25


@pytest.mark.parametrize("overrides", [dict(valid=False), dict(driver=float("nan")), dict(threshold=0)])
def test_rapid_release_cannot_bypass_validity_gates(overrides):
  trace = Trace(2)
  trace.run(100)
  assert max(trace.run(100, **(dict(driver=0, pressed=False) | overrides))) == 25


def test_mode_one_holds_offer_until_legacy_catches_up():
  trace = Trace(1)
  trace.run(80)
  caps = trace.run(30, driver=0, pressed=False)
  assert trace.controller.state == "returning"
  assert 25 < caps[-1] <= 80
  assert trace.run(60, driver=0, pressed=False, baseline=90)[-1] == 90


def test_combined_transfers_offer_to_rapid_without_adding_gains():
  trace = Trace(3)
  trace.run(80)
  assert trace.controller.state == "offering"
  caps = trace.run(50, driver=0, pressed=False)
  assert trace.controller.state == "recover"
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
  trace.run(90)
  assert trace.controller.state == "offering"
  trace.run(50, valid=False)
  assert max(trace.run(200)) == 25
  trace.run(100, driver=0, pressed=False)
  assert max(trace.run(200)) > 25


def test_legacy_full_authority_is_never_lowered_by_offer_logic():
  trace = Trace(3)
  assert min(trace.run(300, baseline=250)) == 250


def test_small_error_growth_during_decreasing_effort_does_not_abort_offer():
  trace = Trace(1)
  trace.run(65, driver=400)
  before = trace.controller.cap
  caps = [trace.step(driver=400 - i, target_error=0.5 + i * 0.05) for i in range(20)]
  assert trace.controller.state == "offering"
  assert caps[-1] > before
  assert min(b - a for a, b in pairwise(caps)) >= -1.100001


@pytest.mark.parametrize("field", ["target_error", "command_error"])
def test_error_alone_fades_offer_gradually_but_renewed_force_yields_fast(field):
  trace = Trace(1)
  trace.run(90)
  before = trace.controller.cap
  assert trace.step(**{field: 10}) == pytest.approx(before - 1.1)
  before = trace.controller.cap
  assert trace.step(driver=-350, **{field: 10}) == pytest.approx(max(25, before - 20))


def test_converging_larger_error_can_qualify_but_stable_large_error_cannot():
  trace = Trace(1)
  caps = [trace.step(target_error=5 - i * 0.015, command_error=5 - i * 0.015) for i in range(90)]
  assert caps[-1] > 25
  trace = Trace(1)
  assert max(trace.run(200, target_error=5, command_error=5)) == 25


def test_large_error_with_decreasing_force_pauses_increase_instead_of_veto():
  trace = Trace(1)
  trace.run(55, driver=400)
  before = trace.controller.cap
  # A larger growing error gives a target ceiling above this early offer, but
  # no converging evidence: hold instead of adding authority or dropping to 25.
  after = trace.step(driver=390, target_error=3.2, command_error=3.2)
  assert after == before
  assert trace.controller.state == "offering"


@pytest.mark.parametrize("field", ["target_error", "command_error"])
def test_release_error_changes_rate_without_blocking_recovery(field):
  caps = []
  for error in (0.5, 6, 15):
    trace = Trace(2)
    trace.run(100, **{field: error})
    values = trace.run(25, driver=0, pressed=False, **{field: error})
    assert values[3] > 25
    assert all(0 <= b - a <= 4.500001 for a, b in pairwise(values))
    caps.append(values[-1])
    assert trace.run(250, driver=0, pressed=False, **{field: error})[-1] == 250
  assert caps[0] > caps[1] > caps[2] > 25


def test_release_rate_tightens_at_speed():
  caps = []
  for speed in (15, 45):
    trace = Trace(2)
    trace.run(100, target_error=2, speed=speed)
    caps.append(trace.run(30, driver=0, pressed=False, target_error=2, speed=speed)[-1])
  assert caps[0] > caps[1] > 25


def test_partial_release_has_only_a_bounded_capture_then_withdraws():
  trace = Trace(2)
  trace.run(100)
  caps = trace.run(150, driver=190, pressed=False)
  assert 25 < max(caps) <= 45
  assert caps[-1] == 25
  assert trace.controller.state == "blocked"


def test_legacy_ceiling_cannot_override_active_capture_or_recovery():
  trace = Trace(2)
  trace.run(100)
  trace.run(4, driver=0, pressed=False, target_error=12)
  before = trace.controller.cap
  caps = trace.run(20, driver=0, pressed=False, target_error=12, baseline=250)
  assert caps[0] - before <= 1.100001
  assert caps[-1] < 80
  assert trace.run(300, driver=0, pressed=False, baseline=250)[-1] == 250
  assert trace.controller.state == "waiting"


def test_invalidity_during_owned_low_cap_does_not_jump_to_full_legacy():
  trace = Trace(2)
  trace.run(100)
  trace.run(10, driver=0, pressed=False)
  before = trace.controller.cap
  assert trace.step(valid=False, baseline=250) <= before
  assert trace.controller.effort is None
  caps = trace.run(50, driver=0, pressed=False, baseline=250)
  assert max(b - a for a, b in pairwise(caps)) <= 1.100001


def test_hard_opposition_also_interrupts_soft_withdrawal():
  trace = Trace(1)
  trace.run(115)
  assert trace.controller.state == "withdrawing"
  before = trace.controller.cap
  assert trace.step(driver=-350) == pytest.approx(max(25, before - 20))


def test_confirmed_recovery_yields_to_force_before_pressed_flag():
  trace = Trace(2)
  trace.run(100)
  trace.run(30, driver=0, pressed=False)
  before = trace.controller.cap
  assert trace.step(driver=220, pressed=False) == pytest.approx(before - 20)
