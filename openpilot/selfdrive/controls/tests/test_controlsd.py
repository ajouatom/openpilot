from openpilot.selfdrive.controls.controlsd import lateral_control_allowed


def test_lateral_control_allows_capable_cars_at_standstill():
  assert lateral_control_allowed(True, False, True, False, False, True, True)


def test_lateral_control_blocks_cars_without_standstill_support():
  assert not lateral_control_allowed(True, False, True, False, False, True, False)


def test_lateral_control_preserves_fault_gate():
  assert not lateral_control_allowed(True, True, True, True, False, False, True)
