from types import SimpleNamespace as NS

from openpilot.selfdrive.carrot.carrot_controls import CarrotControls


def test_suspend_threshold_reads_remain_live_and_do_not_change_timers():
  control = CarrotControls.__new__(CarrotControls)
  control.lat_suspend_active = False
  control.lat_suspend_enter_t = control.lat_suspend_hold_t = 0.
  reads = []
  setting = [40]
  control.params = NS(get_int=lambda key: (reads.append(key), setting[0])[1])
  state = NS(steeringPressed=False, steeringAngleDeg=50.)
  for _ in range(100):
    assert control.lat_suspend_control(state, True)
  assert not reads
  state.steeringPressed = True
  for _ in range(50):
    assert control.lat_suspend_control(state, True)
  setting[0] = 60
  assert control.lat_suspend_control(state, True)
  assert control.lat_suspend_enter_t == 0.
  setting[0] = 40
  for _ in range(100):
    control.lat_suspend_control(state, True)
  assert control.lat_suspend_active
  count = len(reads)
  for _ in range(100):
    assert not control.lat_suspend_control(state, True)
  assert len(reads) == count
  state.steeringPressed = False
  state.steeringAngleDeg = 0.
  assert control.lat_suspend_control(state, True)
  assert not control.lat_suspend_active
