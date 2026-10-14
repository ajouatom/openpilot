import pytest

from openpilot.cereal import car, log
from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
from openpilot.selfdrive.selfdrived.events import AlertStatus, EmptyAlert, ET, Events
from openpilot.selfdrive.selfdrived.state import StateMachine

EventName = log.OnroadEvent.EventName
State = log.SelfdriveState.OpenpilotState


def callback_args(brand="hyundai", control="torque", flags=0, timer=300):
  return [car.CarParams(brand=brand, steerControlType=control, flags=flags), car.CarState(),
          None, False, timer, log.LongitudinalPersonality.standard]


def tick(events, event, args=None):
  events.clear()
  if event is not None:
    events.add(event)
  return events.create_alerts([ET.SOFT_DISABLE, ET.WARNING], args or callback_args())


@pytest.mark.parametrize("brand,control,flags", [
  ("hyundai", "torque", 0), ("hyundai", "angle", 0xffffffff),
  ("toyota", "torque", 0), ("ford", "angle", 0), ("volkswagen", "torque", 0),
])
@pytest.mark.parametrize("event", [EventName.steerTempUnavailable, EventName.steerTempUnavailableSilent])
def test_all_platforms_wait_twelve_completed_cycles(brand, control, flags, event):
  events = Events()
  args = callback_args(brand, control, flags)
  for cycle in range(14):
    alerts = tick(events, event, args)
    assert bool(alerts) == (cycle >= 12)
    assert event in events.names
    assert events.contains(ET.SOFT_DISABLE if event == EventName.steerTempUnavailable else ET.WARNING)


def test_short_fault_does_not_reach_alert_manager():
  events, manager = Events(), AlertManager()
  for frame in range(230):
    alerts = tick(events, EventName.steerTempUnavailable if frame < 7 else None)
    manager.add_many(frame, alerts)
    manager.process_alerts(frame, set())
    assert manager.current_alert is EmptyAlert


def test_repeated_short_faults_accumulate_without_alerting_while_clear():
  events = Events()
  for _ in range(7):
    assert not tick(events, EventName.steerTempUnavailable)
  for _ in range(3):
    assert not tick(events, None)
  for cycle in range(7):
    assert bool(tick(events, EventName.steerTempUnavailable)) == (cycle >= 5)
  assert not tick(events, None)
  assert tick(events, EventName.steerTempUnavailable)


@pytest.mark.parametrize("healthy_cycles,immediate", [(99, True), (100, False)])
def test_full_second_without_temporary_event_resets_delay(healthy_cycles, immediate):
  events = Events()
  for _ in range(13):
    tick(events, EventName.steerTempUnavailable)
  for _ in range(healthy_cycles):
    assert not tick(events, None)
  assert bool(tick(events, EventName.steerTempUnavailable)) == immediate


def test_silent_and_audible_variants_share_history():
  events = Events()
  for cycle in range(14):
    event = EventName.steerTempUnavailable if cycle % 2 else EventName.steerTempUnavailableSilent
    assert bool(tick(events, event)) == (cycle >= 12)


def test_repeated_rendering_and_duplicate_events_do_not_advance_time():
  events = Events()
  events.add(EventName.steerTempUnavailable)
  events.add(EventName.steerTempUnavailable)
  for _ in range(100):
    assert not events.create_alerts([ET.SOFT_DISABLE], callback_args())
  for cycle in range(12):
    assert bool(tick(events, EventName.steerTempUnavailable)) == (cycle == 11)


def test_no_entry_and_serialized_event_remain_immediate():
  events = Events()
  assert not tick(events, EventName.steerTempUnavailable)
  assert events.contains(ET.NO_ENTRY)
  assert events.create_alerts([ET.NO_ENTRY], callback_args())
  message, = events.to_msg()
  assert message.noEntry and message.softDisable


def test_critical_takeover_bypasses_delay():
  events = Events()
  alert, = tick(events, EventName.steerTempUnavailable, callback_args(timer=49))
  assert alert.alert_status == AlertStatus.critical
  assert alert.audible_alert == car.CarControl.HUDControl.AudibleAlert.warningImmediate


@pytest.mark.parametrize("event,kind", [
  (EventName.steerUnavailable, ET.IMMEDIATE_DISABLE),
  (EventName.canError, ET.IMMEDIATE_DISABLE),
  (EventName.vehicleSensorsInvalid, ET.IMMEDIATE_DISABLE),
  (EventName.overheat, ET.SOFT_DISABLE),
])
def test_other_faults_remain_immediate(event, kind):
  events = Events()
  events.add(event)
  assert events.create_alerts([kind], callback_args())


def test_short_fault_keeps_existing_state_machine_recovery():
  events, machine = Events(), StateMachine()
  machine.state = State.enabled
  for cycle in range(7):
    assert not tick(events, EventName.steerTempUnavailable)
    enabled, active = machine.update(events)
    assert enabled and active and machine.state == State.softDisabling
    assert machine.soft_disable_timer == 300 - cycle
  tick(events, None)
  assert machine.update(events) == (True, True)
  assert machine.state == State.enabled


def test_persistent_fault_does_not_extend_soft_disable_deadline():
  events, machine = Events(), StateMachine()
  machine.state = State.enabled
  for cycle in range(301):
    tick(events, EventName.steerTempUnavailable)
    machine.update(events)
    assert machine.state == (State.softDisabling if cycle < 300 else State.disabled)
