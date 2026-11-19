"""Manual choice changes only the vehicle and explicitly requested preset parameters."""

from types import SimpleNamespace as NS

import pytest

from openpilot.common.params import Params
from openpilot.common.vehicle_selection import (
  CAR_LABELS,
  IONIQ5,
  PENDING,
  PORTER,
  PRESETS,
  finish_pending_selection,
  retire_automatic_switch,
  select_manual_vehicle,
  selected_vehicle_index,
)


@pytest.fixture
def store(tmp_path):
  p = Params(str(tmp_path / 'params'))
  p.put_bool('IsOffroad', True)
  for key, value in {
    'CustomSR': 202,
    'CustomSteerMax': 409,
    'AutoEngage': 2,
    'PorterAutoSetMain': True,
    'PorterAutoCancelBelow30': True,
    'PorterGapUse': True,
    'PorterGap2': 80,
    'CalibrationParams': b'camera',
    'LiveTorqueParameters': b'torque',
  }.items():
    p.put(key, value)
  return p


def preserved(p):
  return {
    key: p.get(key)
    for key in (
      'CustomSteerMax',
      'AutoEngage',
      'PorterAutoSetMain',
      'PorterAutoCancelBelow30',
      'PorterGapUse',
      'PorterGap2',
      'CalibrationParams',
      'LiveTorqueParameters',
    )
  }


def test_repeated_explicit_choices_apply_vehicle_presets_and_keep_other_values(store):
  before = preserved(store)
  for index, vehicle in ((0, PORTER), (1, IONIQ5), (0, PORTER)):
    select_manual_vehicle(store, index)
    assert store.get('CarSelected3') == CAR_LABELS[vehicle]
    assert store.get('CarName') == vehicle
    assert {key: store.get_int(key) for key in PRESETS[vehicle]} == PRESETS[vehicle]
    assert selected_vehicle_index(store) == index
    assert preserved(store) == before
    assert not store.get(PENDING)


@pytest.mark.parametrize('onroad,offroad', [(True, False), (True, True), (False, False)])
def test_only_confirmed_offroad_choice_is_allowed(store, onroad, offroad):
  store.put_bool('IsOnroad', onroad)
  store.put_bool('IsOffroad', offroad)
  with pytest.raises(RuntimeError):
    select_manual_vehicle(store, 1)
  assert not store.get(PENDING)


def test_interrupted_selection_finishes_without_touching_other_manual_values(store):
  before = preserved(store)
  store.put(PENDING, {'version': 1, 'vehicle': IONIQ5})
  store.put_int('HyundaiCameraSCC', 2)
  finish_pending_selection(store)
  assert store.get('CarName') == IONIQ5
  assert {key: store.get_int(key) for key in PRESETS[IONIQ5]} == PRESETS[IONIQ5]
  assert preserved(store) == before


@pytest.mark.parametrize('value', [{'version': 2, 'vehicle': IONIQ5}, {'version': 1, 'vehicle': 'OTHER'}, {'vehicle': IONIQ5}])
def test_invalid_pending_choice_never_applies_a_profile(store, value):
  store.put(PENDING, value)
  with pytest.raises(ValueError):
    finish_pending_selection(store)
  assert not store.get('CarName')


def test_retired_auto_settings_cannot_rewrite_presets_or_start_a_handoff(store):
  before = preserved(store)
  store.put_bool('AutoVehicleSwitch', True)
  store.put_bool('AutoVehicleSession', True)
  store.put_bool('AutoVehicleBranchPending', True)
  store.put('AutoVehicleBranchHandoff', {'vehicle': IONIQ5})
  store.put('AutoVehiclePorterBase', {'saved': 'keep as data'})
  retire_automatic_switch(store)
  assert not store.get_bool('AutoVehicleSwitch')
  assert not store.get_bool('AutoVehicleSession')
  assert not store.get('AutoVehicleBranchHandoff')
  assert store.get('AutoVehiclePorterBase') == {'saved': 'keep as data'}
  assert preserved(store) == before


@pytest.mark.parametrize('index,vehicle', [(0, PORTER), (1, IONIQ5)])
def test_real_manual_car_lookup_skips_active_queries_despite_stale_auto_flags(store, monkeypatch, index, vehicle):
  from opendbc.car import car_helpers
  from opendbc.car.structs import CarParams
  from opendbc.car.vin import VIN_UNKNOWN

  select_manual_vehicle(store, index)
  store.put_bool('AutoVehicleSession', True)
  store.put_bool('AutoVehicleBranchPending', True)
  calls = []

  def fingerprint(*args, **kw):
    assert kw['skip_fw_query'] is True
    calls.append(True)
    return None, {0: {}, 1: {}, 2: {}}, VIN_UNKNOWN, [], CarParams.FingerprintSource.can, False

  class Interface:
    @staticmethod
    def get_params(candidate, *args, **kw):
      return NS(carFingerprint=candidate)

    def __init__(self, cp):
      self.CP = cp

  monkeypatch.setattr(car_helpers, 'Params', lambda: store)
  monkeypatch.setattr(car_helpers, 'fingerprint', fingerprint)
  monkeypatch.setattr(car_helpers, 'interfaces', {vehicle: Interface})
  assert car_helpers.get_selected_car_platform(CAR_LABELS[vehicle]) == vehicle
  result = car_helpers.get_car(lambda **kw: [], lambda _: None, lambda _: None, False, False)
  assert result.CP.carFingerprint == vehicle
  assert result.CP.fingerprintSource == CarParams.FingerprintSource.fixed
  assert calls == [True]


def test_other_manual_selection_is_not_replaced_by_a_two_vehicle_default(store):
  store.put('CarSelected3', 'Another selected car')
  assert selected_vehicle_index(store) == -1
  finish_pending_selection(store)
  assert store.get('CarSelected3') == 'Another selected car'


def test_ioniq_params_are_identical_with_porter_options_on_or_off(store, monkeypatch):
  from collections import defaultdict
  from opendbc.car import interfaces
  from opendbc.car.hyundai import interface, hyundaicanfd
  from opendbc.car.hyundai.values import CAR

  select_manual_vehicle(store, 1)
  for module in (interface, hyundaicanfd, interfaces):
    monkeypatch.setattr(module, 'Params', lambda: store)
  fingerprint = defaultdict(dict, {0: {0x1CF: 8}, 1: {0x50: 16}, 2: {}})
  results = []
  for enabled in (False, True):
    for key in ('PorterAutoSetMain', 'PorterAutoCancelBelow30', 'PorterGapUse'):
      store.put_bool(key, enabled)
    results.append(interface.CarInterface.get_params(CAR.HYUNDAI_IONIQ_5, fingerprint, [], True, False, False).to_dict())
  assert results[0] == results[1]


@pytest.mark.parametrize('values', [(True, False, True), (False, True, False), (False, False, False)])
def test_porter_feature_toggles_survive_both_vehicle_choices(store, values):
  keys = ('PorterAutoSetMain', 'PorterAutoCancelBelow30', 'PorterGapUse')
  for key, value in zip(keys, values, strict=True):
    store.put_bool(key, value)
  for index in (0, 1, 1, 0):
    select_manual_vehicle(store, index)
    assert tuple(store.get_bool(key) for key in keys) == values


def test_manual_ratio_adjustment_is_not_overwritten_on_startup_without_a_new_choice(store):
  select_manual_vehicle(store, 1)
  assert store.get_int('CustomSR') == 140
  store.put_int('CustomSR', 155)
  retire_automatic_switch(store)
  finish_pending_selection(store)
  assert store.get_int('CustomSR') == 155
  select_manual_vehicle(store, 0)
  assert store.get_int('CustomSR') == 202
