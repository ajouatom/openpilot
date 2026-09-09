import ast
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.cereal import car


def run_lateral_gate(*, brand="tesla", supported=True, speed=0.0, stopped=True, min_speed=0.0,
                     active=True, always_lateral=False, lat_enabled=True, temporary_fault=False,
                     permanent_fault=False, gear="drive"):
  # Execute the production state_control path through its lateral gating call,
  # without starting hardware, IPC, model inference, or actuator controllers.
  path = Path(__file__).parents[1] / "controlsd.py"
  tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
  tree.body = [node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.ClassDef))]
  namespace = {
    "math": math, "car": car, "MIN_LATERAL_CONTROL_SPEED": 0.3,
    "resolve_vehicle_model_steer_ratio": lambda *_args: 15.0,
  }
  exec(compile(tree, str(path), "exec"), namespace)
  controls = namespace["Controls"].__new__(namespace["Controls"])
  controls.CP = SimpleNamespace(brand=brand, steerAtStandstill=supported, minSteerSpeed=min_speed,
                               lateralTuning=SimpleNamespace(which=lambda: "angle"))
  controls.params = SimpleNamespace(get_float=lambda _key: 0.0, get_bool=lambda _key: always_lateral)
  controls.is_vw_meb = False
  controls.VM = SimpleNamespace(update_params=lambda *_args: None, calc_curvature=lambda *_args: 0.0)
  state = car.CarState.new_message(vEgo=speed, standstill=stopped, gearShifter=gear, latEnabled=lat_enabled,
                                   steerFaultTemporary=temporary_fault, steerFaultPermanent=permanent_fault)
  controls.sm = {
    "carState": state,
    "liveParameters": SimpleNamespace(stiffnessFactor=1.0, steerRatio=15.0, angleOffsetDeg=0.0, roll=0.0),
    "longitudinalPlan": SimpleNamespace(), "modelV2": SimpleNamespace(),
    "selfdriveState": SimpleNamespace(enabled=active, active=active),
  }

  class ReachedLateralGate(Exception):
    pass

  result = []

  def capture(_state, lat_active):
    result.append(lat_active)
    raise ReachedLateralGate

  controls.carrot_controls = SimpleNamespace(lat_suspend_control=capture)
  with pytest.raises(ReachedLateralGate):
    controls.state_control()
  return result[0]


@pytest.mark.parametrize("active,always_lateral", [(True, False), (False, True)])
def test_tesla_can_steer_at_true_standstill(active, always_lateral):
  assert run_lateral_gate(active=active, always_lateral=always_lateral)


@pytest.mark.parametrize("brand", ["tesla", "ford", "volkswagen", "psa", "hyundai"])
@pytest.mark.parametrize("speed,min_speed", [(0.1, 0.0), (0.25, 0.0), (4.9, 5.0), (5.0, 5.0)])
def test_moving_cars_still_obey_minimum_lateral_speed(brand, speed, min_speed):
  assert not run_lateral_gate(brand=brand, speed=speed, min_speed=min_speed, stopped=False)


@pytest.mark.parametrize("brand", ["ford", "volkswagen", "psa", "hyundai"])
def test_standstill_exception_does_not_change_other_brands(brand):
  assert not run_lateral_gate(brand=brand)


@pytest.mark.parametrize("override", [
  {"supported": False}, {"active": False}, {"lat_enabled": False},
  {"temporary_fault": True}, {"permanent_fault": True},
  {"active": False, "always_lateral": True, "gear": "park"},
  {"active": False, "always_lateral": True, "gear": "reverse"},
  {"active": False, "always_lateral": True, "gear": "neutral"},
])
def test_standstill_preserves_engagement_fault_and_gear_gates(override):
  assert not run_lateral_gate(**override)


@pytest.mark.parametrize("brand", ["tesla", "ford", "volkswagen", "psa", "hyundai"])
def test_lateral_control_remains_available_above_minimum_speed(brand):
  assert run_lateral_gate(brand=brand, speed=5.1, min_speed=5.0, stopped=False)
