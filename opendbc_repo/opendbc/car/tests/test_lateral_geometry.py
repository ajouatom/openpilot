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
from opendbc.car.lateral_geometry import LateralGeometry, requested_center_to_front_ratio
from openpilot.common.lateral_geometry_params import migrate_lateral_geometry_params
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
  if stored is not None:
    values['CenterToFrontRatio'] = stored
  cp = make_cp()
  assert requested_center_to_front_ratio(geometry_params[1], cp) == pytest.approx(0.4)


@pytest.mark.parametrize('percent', [31, 40, 50, 60, 69])
def test_live_geometry_recomputes_stiffness_without_mutating_vehicle_baseline(geometry_params, percent):
  values, params = geometry_params
  baseline = make_cp()
  values['CenterToFrontRatio'] = percent
  assert make_cp().to_dict() == baseline.to_dict()
  geometry = LateralGeometry(baseline)
  geometry.update(requested_center_to_front_ratio(params, baseline))
  assert geometry.center_to_front == pytest.approx(baseline.wheelbase * percent / 100)
  front, rear = scale_tire_stiffness(baseline.mass, baseline.wheelbase, geometry.center_to_front, baseline.tireStiffnessFactor)
  assert geometry.stiffness_front == pytest.approx(front)
  assert geometry.stiffness_rear == pytest.approx(rear)
  before, after = VehicleModel(baseline), VehicleModel(baseline)
  before.update_params(0.8, 17.0)
  after.update_params(0.8, 17.0)
  after.update_geometry(geometry.ratio)
  assert after.aF == pytest.approx(geometry.center_to_front)
  assert after.cF == pytest.approx(front * 0.8)
  assert after.sR == 17.0
  for speed in (3., 10., 20., 30.):
    assert after.get_steer_from_curvature(0.002, speed, 0.02) == pytest.approx(
      before.get_steer_from_curvature(0.002, speed, 0.02), rel=1e-5)
  values['CenterToFrontRatio'] = 0
  after.update_geometry(requested_center_to_front_ratio(params, baseline))
  assert after.aF == pytest.approx(baseline.centerToFront)
  assert after.cF_orig == pytest.approx(baseline.tireStiffnessFront)


@pytest.mark.parametrize('previous_ratio,current_ratio,previous_wheelbase,reused,cached_ratio,target_ratio', [
  (0.4, 0.4, 2.895, True, 0, None),
  (0.6, 0.6, 2.895, True, 0, None),
  (0.4, 0.6, 2.895, False, 0, None),
  (0.6, 0.4, 2.895, False, 0, None),
  (0.4, 0.4, 2.7, False, 0, None),
  (0.4, 0.4, 2.895, True, 0.6, 0.6),
  (0.4, 0.4, 2.895, False, 0.5, 0.6),
  (0.4, 0.4, 2.895, False, 0.6, None),
])
def test_parameter_learner_reuses_cache_only_for_same_geometry(previous_ratio, current_ratio, previous_wheelbase, reused, cached_ratio, target_ratio):
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
  lp.centerToFrontRatio = cached_ratio
  lp.steerRatio, lp.stiffnessFactor, lp.angleOffsetAverageDeg = 17.0, 1.2, 0.75
  stored = {'LiveParametersV2': event.to_bytes(), 'CarParamsPrevRoute': previous.to_bytes()}
  removed = []
  params = SimpleNamespace(get=stored.get, remove=removed.append)
  sr, stiffness, offset, _ = namespace['retrieve_initial_vehicle_params'](params, cp, False, False, target_ratio)
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
  assert build().to_dict() == original.to_dict()  # CP preserves each brand's immutable baseline.
  geometry = LateralGeometry(original)
  geometry.update(requested_center_to_front_ratio(params, original))
  assert geometry.ratio == pytest.approx(0.6)
  front, rear = scale_tire_stiffness(original.mass, original.wheelbase, geometry.center_to_front, original.tireStiffnessFactor)
  assert geometry.stiffness_front == pytest.approx(front)
  assert geometry.stiffness_rear == pytest.approx(rear)
  for value in (100, 0):
    values['CenterToFrontRatio'] = value
    geometry.update(requested_center_to_front_ratio(params, original))
    assert geometry.center_to_front == pytest.approx(original.centerToFront)
    assert geometry.stiffness_front == pytest.approx(original.tireStiffnessFront)


@pytest.mark.parametrize('invalid', [0, -1, 0.1, 0.9, float('nan'), float('inf')])
def test_legacy_and_invalid_live_messages_hold_last_geometry(geometry_params, invalid):
  vm = VehicleModel(make_cp())
  vm.update_geometry(0.6)
  vm.update_geometry(invalid)
  assert vm.geometry.ratio == pytest.approx(0.6)


@pytest.mark.parametrize('old_enable,stored,expected', [(None, 55, 55), (0, 60, 0), (1, 60, 60), (0, 40, 0)])
def test_retired_switch_migration_is_idempotent(old_enable, stored, expected):
  values = {'CenterToFrontRatio': stored}
  if old_enable is not None:
    values['CenterToFrontRatioEnabled'] = old_enable
  params = SimpleNamespace(get=values.get, get_bool=lambda k: bool(values.get(k)),
                           put_int=lambda k, v: values.update({k: v}), remove=lambda k: values.pop(k, None))
  migrate_lateral_geometry_params(params)
  assert values == {'CenterToFrontRatio': expected}
  values['CenterToFrontRatio'] = 57
  migrate_lateral_geometry_params(params)
  assert values['CenterToFrontRatio'] == 57


def test_retired_switch_migrates_before_manager_defaults():
  source = Path(__file__).resolve().parents[4] / 'openpilot/system/manager/manager.py'
  text = source.read_text(encoding='utf-8')
  assert text.index('  migrate_lateral_geometry_params(params)') < text.index('  # set unset params to their default value')
