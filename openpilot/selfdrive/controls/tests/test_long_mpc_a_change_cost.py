from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  A_CHANGE_COST,
  get_a_change_cost,
)


def test_active_accel_change_cost_is_fixed():
  assert get_a_change_cost(True, 0.0) == A_CHANGE_COST
  assert get_a_change_cost(True, 500.0) == A_CHANGE_COST


def test_starting_cost_is_preserved_only_without_previous_accel_constraint():
  assert get_a_change_cost(False, 10.0) == 10.0


def test_lead_response_scales_cost_inside_mpc():
  assert get_a_change_cost(True, 500.0, 0.40) == 80.0
  assert get_a_change_cost(True, 500.0, 0.18) == 36.0
  assert get_a_change_cost(True, 500.0, 0.05) == 10.0


def test_response_factor_cannot_raise_or_invert_cost():
  assert get_a_change_cost(True, 10.0, 2.0) == A_CHANGE_COST
  assert get_a_change_cost(True, 10.0, -1.0) == 0.0
