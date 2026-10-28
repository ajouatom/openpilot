import ast
import math
import importlib
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from opendbc.car import gen_empty_fingerprint, scale_tire_stiffness
from opendbc.car import interfaces
from opendbc.car.hyundai import interface, hyundaicanfd
from opendbc.car.values import PLATFORMS
from opendbc.car.hyundai.values import CAR
from opendbc.car.vehicle_model import VehicleModel
from openpilot.cereal import car, log


@pytest.fixture
def geometry_params(monkeypatch):
  values = {}

  class Params:
    def get_int(self, key):
      return int(values.get(key, 0))

    def get_float(self, key):
      return float(values.get(key, 0))

    def get_bool(self, key):
      return bool(values.get(key, False))

    def get(self, key):
      return values.get(key)

  for module in (interface, interfaces, hyundaicanfd):
    monkeypatch.setattr(module, 'Params', Params)
  return values, Params()


def make_cp():
  return interface.CarInterface.get_params(CAR.KIA_K8_HEV_1ST_GEN, gen_empty_fingerprint(), [], False, False, False)


@pytest.mark.parametrize('stored', [None, '', 'bad', 0, -1, 30, 70, 100, float('inf')])
def test_invalid_geometry_keeps_original_default(geometry_params, stored):
  values, _ = geometry_params
  values['CenterToFrontRatioEnabled'] = True
  if stored is not None:
    values['CenterToFrontRatio'] = stored
  cp = make_cp()
  assert cp.centerToFront == pytest.approx(cp.wheelbase * 0.4)


@pytest.mark.parametrize('percent', [31, 40, 50, 60, 69])
def test_real_car_params_recompute_axle_stiffness_and_preserve_other_tuning(geometry_params, percent):
  values, _ = geometry_params
  baseline = make_cp()
  values['CenterToFrontRatioEnabled'] = True
  values['CenterToFrontRatio'] = percent
  cp = make_cp()
  assert cp.centerToFront == pytest.approx(cp.wheelbase * percent / 100)
  front, rear = scale_tire_stiffness(cp.mass, cp.wheelbase, cp.centerToFront, cp.tireStiffnessFactor)
  assert cp.tireStiffnessFront == pytest.approx(front)
  assert cp.tireStiffnessRear == pytest.approx(rear)
  expected = baseline.to_dict()
  actual = cp.to_dict()
  for key in ('centerToFront', 'tireStiffnessFront', 'tireStiffnessRear'):
    expected.pop(key)
    actual.pop(key)
  assert actual == expected
  # Existing objects keep one coherent geometry until CarParams is rebuilt.
  assert baseline.centerToFront == pytest.approx(baseline.wheelbase * 0.4)
  before, after = VehicleModel(baseline), VehicleModel(cp)
  for speed in (3., 10., 20., 30.):
    assert after.get_steer_from_curvature(0.002, speed, 0.02) == pytest.approx(
      before.get_steer_from_curvature(0.002, speed, 0.02), rel=1e-5)
  assert cp.wheelbase - cp.centerToFront == pytest.approx(cp.wheelbase * (1 - percent / 100))


@pytest.mark.parametrize('previous_ratio,current_ratio,previous_wheelbase,reused', [
  (0.4, 0.4, 2.895, True),
  (0.6, 0.6, 2.895, True),
  (0.4, 0.6, 2.895, False),
  (0.6, 0.4, 2.895, False),
  (0.4, 0.4, 2.7, False),
])
def test_parameter_learner_reuses_cache_only_for_same_geometry(previous_ratio, current_ratio, previous_wheelbase, reused):
  # Run the complete production cache loader with real serialized schema;
  # native Kalman/IPC construction is not needed to check restart behavior.
  root = Path(__file__).resolve().parents[4]
  path = root / 'openpilot/selfdrive/locationd/paramsd.py'
  tree = ast.parse(path.read_text(encoding='utf-8'))
  node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'retrieve_initial_vehicle_params')
  namespace = {'Params': object, 'car': car, 'log': log, 'np': np, 'math': math,
               'cloudlog': SimpleNamespace(error=lambda *a: None, info=lambda *a: None)}
  exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), namespace)
  cp = car.CarParams.new_message(carFingerprint='KIA_K8_HEV_1ST_GEN', steerRatio=13.27,
                                wheelbase=2.895, centerToFront=2.895 * current_ratio)
  previous = car.CarParams.new_message(carFingerprint=cp.carFingerprint, steerRatio=13.27,
                                      wheelbase=previous_wheelbase, centerToFront=previous_wheelbase * previous_ratio)
  event = log.Event.new_message()
  lp = event.init('liveParameters')
  lp.steerRatio, lp.stiffnessFactor, lp.angleOffsetAverageDeg = 17.0, 1.2, 0.75
  stored = {'LiveParametersV2': event.to_bytes(), 'CarParamsPrevRoute': previous.to_bytes()}
  removed = []
  params = SimpleNamespace(get=stored.get, remove=removed.append)
  sr, stiffness, offset, _ = namespace['retrieve_initial_vehicle_params'](params, cp, False, False)
  assert sr == pytest.approx(17.0 if reused else cp.steerRatio)
  assert offset == pytest.approx(0.75 if reused else 0.0)
  assert stiffness == 1.0
  assert removed == ([] if reused else ['LiveParametersV2'])


BRAND_CANDIDATES = {}
for candidate in PLATFORMS.values():
  BRAND_CANDIDATES.setdefault(type(candidate).__module__.split('.')[-2], candidate)


@pytest.mark.parametrize('brand,candidate', sorted(BRAND_CANDIDATES.items()))
def test_shared_override_applies_after_each_brand_setup_and_restores_defaults(monkeypatch, geometry_params, brand, candidate):
  values, params = geometry_params
  if candidate not in interfaces.get_torque_params():
    pytest.skip('Existing platform has no torque-data entry for CarParams construction')
  module = importlib.import_module(f'opendbc.car.{brand}.interface')
  # Replace settings IO only; exercise each brand's real interface and schema.
  for name, loaded in list(sys.modules.items()):
    if name.startswith('opendbc.car.') and hasattr(loaded, 'Params'):
      monkeypatch.setattr(loaded, 'Params', lambda: params)

  def build():
    return module.CarInterface.get_params(candidate, gen_empty_fingerprint(), [], False, False, False)
  original = build()
  values['CenterToFrontRatio'] = 60
  assert build().to_dict() == original.to_dict()  # Disabled ignores a stored value.
  values['CenterToFrontRatioEnabled'] = True
  changed = build()
  assert changed.centerToFront == pytest.approx(changed.wheelbase * 0.6)
  front, rear = scale_tire_stiffness(changed.mass, changed.wheelbase, changed.centerToFront, changed.tireStiffnessFactor)
  assert changed.tireStiffnessFront == pytest.approx(front)
  assert changed.tireStiffnessRear == pytest.approx(rear)
  original_dict, changed_dict = original.to_dict(), changed.to_dict()
  for key in ('centerToFront', 'tireStiffnessFront', 'tireStiffnessRear'):
    original_dict.pop(key)
    changed_dict.pop(key)
  assert changed_dict == original_dict
  values['CenterToFrontRatio'] = 100
  assert build().to_dict() == original.to_dict()  # Invalid input restores each brand's value, not 0.4.
  values['CenterToFrontRatio'] = 60
  values['CenterToFrontRatioEnabled'] = False
  assert build().to_dict() == original.to_dict()
