import pytest

from openpilot.selfdrive.carrot.t_follow import ramp_t_follow


DT_MDL = 0.05


def _apply_for_seconds(current: float, target: float, seconds: float, decel_extra: float = 0.0) -> float:
  for _ in range(round(seconds / DT_MDL)):
    current = ramp_t_follow(target, current, decel_extra, DT_MDL)
  return current


def test_gap_one_to_four_is_felt_within_two_seconds():
  assert _apply_for_seconds(1.1, 1.6, 1.7) == pytest.approx(1.6)


def test_gap_increase_is_faster_while_already_decelerating():
  assert _apply_for_seconds(1.1, 1.6, 0.85, decel_extra=0.03) == pytest.approx(1.6)


def test_gap_reduction_remains_immediate():
  assert ramp_t_follow(1.1, 1.6, 0.0, DT_MDL) == pytest.approx(1.1)
