from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  A_CHANGE_COST,
  get_a_change_cost,
)


def test_active_accel_change_cost_is_fixed():
  assert get_a_change_cost(True, 0.0) == A_CHANGE_COST
  assert get_a_change_cost(True, 500.0) == A_CHANGE_COST


def test_starting_cost_is_preserved_only_without_previous_accel_constraint():
  assert get_a_change_cost(False, 10.0) == 10.0
