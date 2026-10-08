from openpilot.cereal import car, log
from openpilot.selfdrive.car.car_specific import (
  CarSpecificEvents,
  TESLA_STANDSTILL_STEER_FAULT_DEBOUNCE_FRAMES,
  TESLA_STANDSTILL_STEER_FAULT_SPEED,
)


EventName = log.OnroadEvent.EventName


def make_events(brand="tesla"):
  CP = car.CarParams.new_message(brand=brand)
  events = CarSpecificEvents(CP)
  events.silent_steer_warning = 20
  return events


def update_steer_fault(events, *, standstill=True, temporary=True, permanent=False, v_ego=0.0):
  CS = car.CarState.new_message(
    standstill=standstill,
    vEgo=v_ego,
    steerFaultTemporary=temporary,
    steerFaultPermanent=permanent,
  )
  return events.create_common_events(CS, car.CarState.new_message(), pcm_enable=False)


def test_tesla_standstill_temporary_fault_is_debounced():
  events = make_events()

  for _ in range(TESLA_STANDSTILL_STEER_FAULT_DEBOUNCE_FRAMES - 1):
    names = update_steer_fault(events).names
    assert EventName.steerTempUnavailable not in names
    assert EventName.steerTempUnavailableSilent not in names

  assert EventName.steerTempUnavailableSilent in update_steer_fault(events).names


def test_tesla_standstill_debounce_resets_after_fault_clears():
  events = make_events()

  for _ in range(TESLA_STANDSTILL_STEER_FAULT_DEBOUNCE_FRAMES - 1):
    update_steer_fault(events)
  update_steer_fault(events, temporary=False)

  assert EventName.steerTempUnavailableSilent not in update_steer_fault(events).names


def test_moving_tesla_temporary_fault_is_not_debounced():
  events = make_events()

  assert EventName.steerTempUnavailableSilent in update_steer_fault(
    events, standstill=False, v_ego=TESLA_STANDSTILL_STEER_FAULT_SPEED + 0.1
  ).names


def test_tesla_near_stop_temporary_fault_is_debounced_before_standstill_latches():
  events = make_events()

  for _ in range(TESLA_STANDSTILL_STEER_FAULT_DEBOUNCE_FRAMES - 1):
    names = update_steer_fault(
      events, standstill=False, v_ego=TESLA_STANDSTILL_STEER_FAULT_SPEED - 0.1
    ).names
    assert EventName.steerTempUnavailable not in names
    assert EventName.steerTempUnavailableSilent not in names


def test_other_vehicle_temporary_fault_is_not_debounced():
  events = make_events(brand="ford")

  assert EventName.steerTempUnavailableSilent in update_steer_fault(events).names


def test_tesla_permanent_fault_is_not_debounced():
  events = make_events()

  assert EventName.steerUnavailable in update_steer_fault(events, permanent=True).names
