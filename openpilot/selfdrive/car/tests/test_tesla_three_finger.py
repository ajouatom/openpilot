import pytest

from opendbc.can import CANPacker
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.tests.test_auto_turn_signals import N, params
from openpilot.cereal import car
from openpilot.common.params import Params
from openpilot.selfdrive.car import car_specific


@pytest.fixture
def gesture(monkeypatch, tmp_path):
  store = Params(str(tmp_path / "params"))
  monkeypatch.setattr(car_specific, "Params", lambda: store)

  def make(brand="tesla"):
    events = car_specific.CarSpecificEvents(car.CarParams.new_message(brand=brand))

    def update(buttons=()):
      state = car.CarState.new_message(buttonEvents=[
        {"type": str(button.type), "pressed": button.pressed} for button in buttons
      ])
      events.update(state, car.CarState.new_message(), car.CarControl.new_message())

    return update

  return store, make


@pytest.mark.parametrize("initial", [None, False, True])
@pytest.mark.parametrize("experimental", [False, True])
@pytest.mark.parametrize("confirmed", [False, True])
def test_tesla_touch_can_toggles_only_always_lateral(gesture, initial, experimental, confirmed):
  store, make = gesture
  if initial is not None:
    store.put_bool("AlwaysLateral", initial)
  store.put_bool("ExperimentalMode", experimental)
  store.put_bool("ExperimentalModeConfirmed", confirmed)
  update = make()
  ci = CarInterface(params(enabled=False))
  packer = CANPacker("tesla_model3_vehicle")
  start = bool(initial)
  expected = [start, start, start, not start, not start, not start, not start, start, start]
  for index, (touches, enabled) in enumerate(zip([0, 1, 2, 3, 3, 3, 2, 3, 0], expected, strict=True)):
    state = ci.update([(N + index * 100_000_000, [
      packer.make_can_msg("UI_status2", 1, {"UI_activeTouchPoints": touches}),
    ])])
    update(state.buttonEvents)
    assert store.get_bool("AlwaysLateral") == enabled
    assert store.get_bool("ExperimentalMode") == experimental
    assert store.get_bool("ExperimentalModeConfirmed") == confirmed


def test_duplicate_press_and_unrelated_events_do_not_retoggle(gesture):
  store, make = gesture
  update = make()
  press = car.CarState.ButtonEvent.new_message(type="lkas", pressed=True)
  release = car.CarState.ButtonEvent.new_message(type="lkas", pressed=False)
  other = car.CarState.ButtonEvent.new_message(type="cancel", pressed=False)
  update([press])
  assert store.get_bool("AlwaysLateral")
  for buttons in ([press], [], [other], [press]):
    update(buttons)
    assert store.get_bool("AlwaysLateral")
  update([release])
  update([press])
  assert not store.get_bool("AlwaysLateral")


@pytest.mark.parametrize("brand", ["hyundai", "ford"])
def test_other_brands_do_not_use_tesla_shortcut(gesture, brand):
  store, make = gesture
  update = make(brand)
  update([car.CarState.ButtonEvent.new_message(type="lkas", pressed=True)])
  assert store.get("AlwaysLateral") is None


def test_shortcut_reaches_live_control_without_bypassing_interlocks(gesture):
  from openpilot.selfdrive.controls.tests.test_lateral_startup_control import control_fixture

  store, make = gesture
  update = make()
  control, step = control_fixture()
  control.CP = control.CP.as_builder()
  control.CP.brand = "tesla"
  control.params.get_bool = store.get_bool
  state = control.sm["carState"]
  control.sm["selfdriveState"].enabled = control.sm["selfdriveState"].active = False
  assert not step()[0].latActive
  press = car.CarState.ButtonEvent.new_message(type="lkas", pressed=True)
  update([press])
  assert step()[0].latActive
  state.latEnabled = False
  assert not step()[0].latActive
  state.latEnabled = True
  state.steerFaultPermanent = True
  assert not step()[0].latActive
  state.steerFaultPermanent = False
  state.gearShifter = "park"
  assert not step()[0].latActive
  state.gearShifter = "drive"
  assert step()[0].latActive
  update([car.CarState.ButtonEvent.new_message(type="lkas", pressed=False)])
  update([press])
  assert not step()[0].latActive
  control.sm["selfdriveState"].enabled = control.sm["selfdriveState"].active = True
  assert step()[0].latActive
