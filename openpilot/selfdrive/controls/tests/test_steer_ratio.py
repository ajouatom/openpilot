import ast
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.lib.steer_ratio import resolve_vehicle_model_steer_ratio
from openpilot.selfdrive.controls.lib.lateral_readiness import LateralStartupGate, lateral_vehicle_parameters


@pytest.mark.parametrize("invalid_rate", [-math.inf, math.nan, 0.0, 29.0, 201.0, math.inf])
def test_invalid_steer_ratio_rate_uses_full_live_ratio(invalid_rate):
  assert resolve_vehicle_model_steer_ratio(14.75, invalid_rate, 0.0) == pytest.approx(14.75)


@pytest.mark.parametrize(("rate", "expected"), [(30.0, 4.425), (80.0, 11.8), (100.0, 14.75), (200.0, 29.5)])
def test_valid_steer_ratio_rate_scales_live_ratio(rate, expected):
  assert resolve_vehicle_model_steer_ratio(14.75, rate, 0.0) == pytest.approx(expected)


def test_custom_steer_ratio_overrides_live_ratio_rate():
  assert resolve_vehicle_model_steer_ratio(14.75, 80.0, 150.0) == pytest.approx(15.0)


@pytest.mark.parametrize("invalid_custom", [-math.inf, math.nan, -10.0, 0.0, 10.0, math.inf])
def test_invalid_custom_ratio_uses_live_scaling(invalid_custom):
  assert resolve_vehicle_model_steer_ratio(14.75, 80.0, invalid_custom) == pytest.approx(11.8)


@pytest.mark.parametrize("is_vw_meb", [False, True])
def test_controlsd_applies_live_setting_changes_to_vehicle_model(is_vw_meb):
  # Execute the production state_control prefix through VM.update_params.
  # This covers the actual setting reads and wiring without hardware/IPC.
  path = Path(__file__).parents[1] / "controlsd.py"
  tree = ast.parse(path.read_text(encoding="utf-8"))
  controls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "Controls")
  method = next(n for n in controls.body if isinstance(n, ast.FunctionDef) and n.name == "state_control")
  end = next(i for i, n in enumerate(method.body) if isinstance(n, ast.Expr)
             and isinstance(n.value, ast.Call) and ast.unparse(n.value.func) == "self.VM.update_params")
  method.body = method.body[:end + 1]
  module = ast.Module(body=[method], type_ignores=[])
  namespace = {"resolve_vehicle_model_steer_ratio": resolve_vehicle_model_steer_ratio,
               "lateral_vehicle_parameters": lateral_vehicle_parameters}
  exec(compile(module, str(path), "exec"), namespace)
  settings = {"CustomSR": 0.0, "SteerRatioRate": 100.0}
  live = SimpleNamespace(stiffnessFactor=0.69, steerRatio=14.97, valid=True, sensorValid=True,
                         posenetValid=True, angleOffsetDeg=0.0, roll=0.0)
  class Messages(dict):
    seen = {'liveParameters': True}

    def all_checks(self, _services):
      return True
  updates = []
  instance = SimpleNamespace(is_vw_meb=is_vw_meb,
                             lateral_startup=LateralStartupGate(ready=True),
                             CP=SimpleNamespace(steerRatio=15.0),
                             sm=Messages(carState=SimpleNamespace(), liveParameters=live),
                             params=SimpleNamespace(get_float=lambda key: settings[key]),
                             VM=SimpleNamespace(update_params=lambda stiff, sr: updates.append((stiff, sr))))
  for custom, rate, learned, expected in [(0, 100, 14.97, 14.97), (159, 30, 14.97, 15.9),
                                         (200, 30, 14.97, 20.0), (0, 30, 14.97, 4.491),
                                         (0, 100, 15.1, 15.1), (0, 0, 15.1, 15.1)]:
    settings.update(CustomSR=custom, SteerRatioRate=rate)
    live.steerRatio = learned
    namespace["state_control"](instance)
    assert updates[-1] == pytest.approx((0.69, expected))
