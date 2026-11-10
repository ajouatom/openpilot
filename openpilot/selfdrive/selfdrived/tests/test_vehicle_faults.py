from types import SimpleNamespace as NS

import pytest

from openpilot.cereal import car, log
from openpilot.selfdrive.selfdrived.vehicle_faults import VehicleFaultMonitor, inhibit_faulted_controls
from openpilot.selfdrive.selfdrived.tests.test_model_startup_events import Events, state_machine


def panda(count=0, *, passive=True, fault=True, uptime=10):
  p = log.PandaState.new_message()
  p.uptime = uptime
  p.faults = ['interruptRateCan2'] if fault else []
  p.canState1.errorPassive = passive
  p.canState1.totalErrorCnt = count
  return p


def test_current_can_error_storm_blocks_after_confirmation_and_requires_healthy_recovery():
  monitor = VehicleFaultMonitor()
  for i in range(6):
    assert monitor.update_pandas([panda(i * 1000)], 10 + i * 0.1, valid=True) == (i == 5)
  # Repeated reads cannot add confirmation or advance recovery.
  for _ in range(100):
    assert monitor.update_pandas([panda(5000)], 10.5, valid=True)
  assert monitor.update_pandas([panda(5000, passive=False)], 20, valid=True)
  for i in range(1, 20):
    assert monitor.update_pandas([panda(5000, passive=False)], 20 + i * 0.1, valid=True)
  assert not monitor.update_pandas([panda(5000, passive=False)], 22.1, valid=True)


def test_passive_bus_does_not_recover_just_because_counter_stops():
  m = VehicleFaultMonitor()
  for i in range(7):
    m.update_pandas([panda(i * 1000)], 1 + i * .1, valid=True)
  for i in range(40):
    assert m.update_pandas([panda(6000)], 2 + i * .1, valid=True)


@pytest.mark.parametrize('passive,fault,increment', [(False, True, 1000), (True, False, 1000), (True, True, 0), (True, True, 1)])
def test_historical_fault_or_small_counter_growth_does_not_disable(passive, fault, increment):
  monitor = VehicleFaultMonitor()
  for i in range(30):
    assert not monitor.update_pandas([panda(5000000 + i * increment, passive=passive, fault=fault)], 10 + i * 0.1, valid=True)


def test_counter_reset_and_sample_gap_cannot_complete_confirmation():
  m = VehicleFaultMonitor()
  for t, c in [(1.0, 100), (1.1, 1100), (1.2, 2100), (1.3, 0), (1.4, 1000), (2.0, 2000), (2.1, 3000)]:
    assert not m.update_pandas([panda(c)], t, valid=True)


def test_invalid_stream_does_not_clear_confirmed_fault():
  m = VehicleFaultMonitor()
  for i in range(7):
    m.update_pandas([panda(i * 1000)], 1 + i * 0.1, valid=True)
  assert m.can_fault
  assert m.update_pandas([], 100.0, valid=False)
  assert m.update_pandas([panda(6000)], 101.0, valid=True)


def test_short_fca_fault_is_not_accumulated_forever_but_recurring_fault_is():
  m = VehicleFaultMonitor()
  for i in range(150):
    assert not m.update_scc(i < 6, 1 + i * 0.01, valid=True)
  m = VehicleFaultMonitor()
  for i in range(70):
    m.update_scc(i < 6 or 60 <= i < 66, 1 + i * 0.01, valid=True)
  assert m.scc_fault
  assert m.update_scc(False, 20.0, valid=True)  # a missing interval is not recovery
  for i in range(1, 210):
    m.update_scc(False, 20 + i * 0.01, valid=True)
  assert not m.scc_fault


@pytest.mark.parametrize('name', ['pandaCanFault', 'stockSccFault'])
def test_confirmed_fault_disables_state_machine_and_always_lateral(name):
  event = getattr(log.OnroadEvent.EventName, name)
  events = Events([event])
  machine = state_machine()
  machine.state = log.SelfdriveState.OpenpilotState.enabled
  assert machine.update(events) == (False, False)
  assert events.contains('NO_ENTRY')
  # Exercise the actual controlsd admission helper on serialized event values.
  control = car.CarControl.new_message(enabled=True, latActive=True, longActive=True)
  message = log.Event.new_message()
  message.init('onroadEvents', 1)
  message.onroadEvents[0].name = event
  inhibit_faulted_controls(control, message.onroadEvents)
  assert not (control.enabled or control.latActive or control.longActive)
  events.add(log.OnroadEvent.EventName.buttonEnable)
  assert machine.update(events) == (False, False)


def test_unrelated_events_keep_existing_actuator_admission():
  control = NS(enabled=True, latActive=True, longActive=True)
  inhibit_faulted_controls(control, [NS(name='gasPressedOverride')])
  assert control.enabled and control.latActive and control.longActive
