import pytest

from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  A_CHANGE_COST,
  get_jlead_a_change_cost,
)


@pytest.mark.parametrize(("factor", "expected_min_cost"), [
  (0.0, 200.0),
  (0.25, 155.0),
  (0.5, 110.0),
  (0.75, 65.0),
  (1.0, 20.0),
])
def test_jlead_factor_scales_cost_reduction(factor, expected_min_cost):
  assert get_jlead_a_change_cost(2.0, factor) == pytest.approx(expected_min_cost)


def test_zero_factor_keeps_full_smoothing_cost():
  for j_lead in (-3.0, -1.0, 0.0, 1.0, 3.0):
    assert get_jlead_a_change_cost(j_lead, 0.0) == pytest.approx(A_CHANGE_COST)


def test_jlead_cost_preserves_thresholds_and_symmetry():
  assert get_jlead_a_change_cost(0.3, 0.5) == pytest.approx(A_CHANGE_COST)
  assert get_jlead_a_change_cost(1.15, 0.5) == pytest.approx(155.0)
  assert get_jlead_a_change_cost(-1.15, 0.5) == pytest.approx(155.0)


def test_jlead_factor_is_clipped_to_setting_range():
  assert get_jlead_a_change_cost(2.0, -1.0) == pytest.approx(A_CHANGE_COST)
  assert get_jlead_a_change_cost(2.0, 2.0) == pytest.approx(20.0)
