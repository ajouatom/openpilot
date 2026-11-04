import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.cereal import car
from openpilot.selfdrive.car import cruise
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.values import Buttons, HyundaiFlags


@pytest.fixture
def setup_driver(monkeypatch):
  def setup(auto_engage=0):
    class Settings:
      def __init__(self, *args):
        pass

      def get_int(self, key):
        return {"AutoEngage": auto_engage, "CruiseButtonLongDelay": 40,
                "CruiseSpeedUnit": 10, "CruiseSpeedUnitBasic": 1, "CancelButtonMode": 1}.get(key, 0)

      def get_float(self, key):
        return float(self.get_int(key))

      def get_bool(self, key):
        return False

    monkeypatch.setattr(cruise, "Params", Settings)
    monkeypatch.setattr(cruise, "CommandReader", lambda *args: SimpleNamespace(read=lambda **kwargs: None))
    cp = car.CarParams(brand="hyundai", flags=int(HyundaiFlags.CANFD | HyundaiFlags.CAMERA_SCC),
                       openpilotLongitudinalControl=True, pcmCruise=False)
    state = CarState.__new__(CarState)
    state.CP = cp
    state.main_enabled = auto_engage == 2
    state.manual_main_off_latched = False
    state.MainMode_ACC = False
    helper = cruise.VCruiseCarrot(cp)
    cs = car.CarState(canValid=True, gearShifter="drive", vEgo=20., vEgoCluster=20.)
    sm = type("Messages", (dict,), {"alive": {"longitudinalPlan": False, "radarState": False,
                                             "drivingModelData": False}, "seen": {"carrotMan": False}})(
      carControl=car.CarControl(enabled=False, latActive=False, longActive=False))

    def tick(stock_main, *, main_release=False, button=Buttons.NONE, event=None):
      state.MainMode_ACC = stock_main
      state._update_canfd_main_enabled(button, main_release)
      cs.cruiseState.available = state.main_enabled  # Initialized/ready card path.
      cs.buttonEvents = [] if event is None else [car.CarState.ButtonEvent(type=event[0], pressed=event[1])]
      helper.update_v_cruise(cs, sm, True)
      cs.latEnabled = helper._lat_enabled  # Same publication as card.update.
      return cs

    return state, helper, tick
  return setup


@pytest.mark.parametrize("auto_engage", (0, 1, 2))
def test_stock_main_feedback_does_not_create_a_driver_enable(setup_driver, auto_engage):
  state, helper, tick = setup_driver(auto_engage)
  for stock_main in [False] * 5 + [True] * 20 + [False] * 5 + [True] * 20:
    cs = tick(stock_main)
    assert state.main_enabled == (auto_engage == 2)
    assert cs.latEnabled == (auto_engage > 0)
    assert helper._activate_cruise == 0


def test_driver_lfa_toggle_remains_off_through_stock_main_changes(setup_driver):
  _, _, tick = setup_driver()
  assert not tick(True).latEnabled
  for expected in (True, False, True, False):
    tick(True, event=("lfaButton", True))
    assert tick(True, event=("lfaButton", False)).latEnabled == expected
    for main in (False, True, True):
      assert tick(main).latEnabled == expected


def test_driver_main_button_can_arm_then_disable_while_oem_main_stays_on(setup_driver):
  state, _, tick = setup_driver()
  assert not tick(True).latEnabled
  assert tick(True, main_release=True).latEnabled
  tick(True, main_release=True)
  assert not state.main_enabled
  # Explicit LFA off must survive subsequent OEM MAIN feedback as well.
  tick(True, event=("lfaButton", True))
  assert not tick(True, event=("lfaButton", False)).latEnabled
  for main in (True, False, True):
    assert not tick(main).latEnabled


@pytest.mark.parametrize("button,event", [(Buttons.RES_ACCEL, "accelCruise"), (Buttons.SET_DECEL, "decelCruise")])
def test_driver_set_resume_can_enable_after_automatic_stock_main(setup_driver, button, event):
  state, _, tick = setup_driver()
  assert not tick(True).latEnabled
  tick(True, button=button, event=(event, True))
  assert tick(True, event=(event, False)).latEnabled
  assert state.main_enabled


@pytest.mark.parametrize("always_lateral", (False, True))
def test_production_lateral_gate_remains_closed_without_driver_intent(setup_driver, always_lateral):
  _, _, tick = setup_driver()
  source = Path(__file__).resolve().parents[2] / "controls/controlsd.py"
  node = next(node for node in ast.parse(source.read_text(encoding="utf-8")).body
              if isinstance(node, ast.FunctionDef) and node.name == "lateral_control_allowed")
  scope = {}
  exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), "exec"), scope)
  for main in (False, True, True, False, True):
    cs = tick(main)
    assert not scope["lateral_control_allowed"](False, always_lateral, cs.latEnabled,
                                                False, False, False, False, False)
