from types import SimpleNamespace

import pytest

from openpilot.common import auto_vehicle as av
from openpilot.common import auto_vehicle_branch as branches
from openpilot.common.params import Params
from opendbc.car import car_helpers
from opendbc.car.hyundai.porter_fingerprint import PORTER_BUS0, PORTER_CAMERA
from opendbc.car.structs import CarParams
from opendbc.car.vin import VIN_UNKNOWN


@pytest.fixture
def setup(tmp_path, monkeypatch):
  monkeypatch.setattr(branches, 'await_correct_branch', lambda *args: None)
  params = Params(str(tmp_path / 'params'))
  for key in av.IONIQ_SETTINGS:
    params.put(key, params.get_default_value(key))
  params.put_bool('PorterAutoSetMain', True)
  params.put('CarName', av.PORTER)
  params.put('CarSelected3', av.CAR_LABELS[av.PORTER])
  params.put_bool('AutoVehicleSession', True)
  monkeypatch.setattr(car_helpers, 'Params', lambda: params)
  for key in ('FINGERPRINT', 'SKIP_FW_QUERY', 'DISABLE_FW_CACHE'):
    monkeypatch.delenv(key, raising=False)
  calls = []

  class Interface:
    @staticmethod
    def get_params(candidate, fingerprints, car_fw, alpha_long_allowed, is_release, docs):
      calls.append((candidate, {k: params.get(k) for k in av.IONIQ_SETTINGS}))
      return SimpleNamespace(carFingerprint=candidate)

    def __init__(self, cp):
      self.CP = cp

  monkeypatch.setattr(car_helpers, 'interfaces', {c: Interface for c in (av.PORTER, av.IONIQ5, 'MOCK')})
  return params, calls


def detect(monkeypatch, candidate, fingerprint, exact=True):
  def fingerprint_call(*args, **kwargs):
    assert args[-1] is None  # Cached firmware from the other car is bypassed.
    assert not kwargs['skip_fw_query']  # A stored manual selection cannot win.
    return candidate, fingerprint, VIN_UNKNOWN, [], CarParams.FingerprintSource.fw, exact

  monkeypatch.setattr(car_helpers, 'fingerprint', fingerprint_call)
  return car_helpers.get_car(lambda **kw: [], lambda messages: None, lambda enabled: None,
                            False, False, cached_params=object()).CP


def test_porter_ioniq_porter_switch_happens_before_interface_construction(setup, monkeypatch):
  params, calls = setup
  base = {k: params.get(k) for k in av.IONIQ_SETTINGS}
  cp = detect(monkeypatch, av.IONIQ5, {0: {}, 1: {}, 2: {0x50: 16}})
  assert cp.carFingerprint == av.IONIQ5
  assert calls[-1] == (av.IONIQ5, av.IONIQ_SETTINGS)
  assert params.get('CarSelected3') == av.CAR_LABELS[av.IONIQ5]
  cp = detect(monkeypatch, None, {0: PORTER_BUS0, 1: {}, 2: PORTER_CAMERA})
  assert cp.carFingerprint == av.PORTER
  assert calls[-1] == (av.PORTER, base)
  assert params.get('CarSelected3') == av.CAR_LABELS[av.PORTER]


@pytest.mark.parametrize(('candidate', 'fingerprint', 'exact'), [
  (None, {0: {}, 1: {}, 2: {}}, True),
  (av.IONIQ5, {2: {0x50: 16}}, False),
  (av.IONIQ5, {0: {}, 1: {}, 2: {}}, True),
  ('HYUNDAI_IONIQ_5_PE', {2: {0x50: 16}}, True),
])
def test_unresolved_identity_cannot_use_stale_manual_porter_or_change_settings(setup, monkeypatch, candidate, fingerprint, exact):
  params, calls = setup
  before = {k: params.get(k) for k in av.IONIQ_SETTINGS}
  cp = detect(monkeypatch, candidate, fingerprint, exact)
  assert cp.carFingerprint == 'MOCK'
  assert calls[-1] == ('MOCK', before)
  assert params.get('CarSelected3') == av.CAR_LABELS[av.PORTER]


def test_fixed_environment_does_not_force_a_vehicle_in_auto_mode(setup, monkeypatch):
  _, calls = setup
  monkeypatch.setenv('FINGERPRINT', av.IONIQ5)
  cp = detect(monkeypatch, av.IONIQ5, {2: {0x50: 16}})
  assert cp.carFingerprint == 'MOCK'
  assert calls[-1][0] == 'MOCK'


def test_card_restart_uses_session_identity_without_reapplying_or_querying(setup, monkeypatch):
  params, calls = setup
  params.put('AutoVehicleResolved', av.IONIQ5)
  params.put_bool('AutoVehicleReady', True)
  # This edit is intentionally after startup; it cannot redirect a restart.
  params.put('CarSelected3', av.CAR_LABELS[av.PORTER])
  params.put_int('CustomSR', 150)

  def fingerprint_call(*args, **kwargs):
    assert kwargs['skip_fw_query']
    return None, {0: {}, 1: {}, 2: {0x50: 16}}, VIN_UNKNOWN, [], CarParams.FingerprintSource.can, True

  monkeypatch.setattr(car_helpers, 'fingerprint', fingerprint_call)
  cp = car_helpers.get_car(lambda **kw: [], lambda m: None, lambda e: None, False, False).CP
  assert cp.carFingerprint == av.IONIQ5
  assert params.get_int('CustomSR') == 150
  assert calls[-1][0] == av.IONIQ5


def test_real_hyundai_params_equal_manual_settings_and_porter_profile_survives(setup, monkeypatch):
  from opendbc.car.hyundai import interface
  from opendbc.car.hyundai.values import CAR, HyundaiFlags

  params, _ = setup
  monkeypatch.setattr(interface, 'Params', lambda: params)
  porter_fp = {0: dict(PORTER_BUS0), 1: {}, 2: dict(PORTER_CAMERA)}
  original = interface.CarInterface.get_params(CAR.HYUNDAI_PORTER_II_EV, porter_fp, [], False, False, docs=False)
  data = av.prepare_settings(params)
  av.apply_vehicle_settings(params, av.IONIQ5, data)
  ioniq_fp = {0: {}, 1: {}, 2: {0x50: 16}}
  automatic = interface.CarInterface.get_params(CAR.HYUNDAI_IONIQ_5, ioniq_fp, [], False, False, docs=False)
  assert automatic.flags & HyundaiFlags.CANFD_HDA2
  assert automatic.flags & HyundaiFlags.CAMERA_SCC
  assert all(c.safetyParam != 15489 for c in automatic.safetyConfigs)
  for key, value in av.IONIQ_SETTINGS.items():
    params.put(key, value)
  manual = interface.CarInterface.get_params(CAR.HYUNDAI_IONIQ_5, ioniq_fp, [], False, False, docs=False)
  assert automatic.to_dict() == manual.to_dict()
  av.apply_vehicle_settings(params, av.PORTER, av.prepare_settings(params))
  restored = interface.CarInterface.get_params(CAR.HYUNDAI_PORTER_II_EV, porter_fp, [], False, False, docs=False)
  assert restored.to_dict() == original.to_dict()


@pytest.mark.parametrize('vehicle', [av.PORTER, av.IONIQ5])
def test_branch_mismatch_blocks_before_presets_and_interface(setup, monkeypatch, vehicle):
  params, calls = setup
  before = {k: params.get(k) for k in av.IONIQ_SETTINGS}
  class Pending(Exception):
    pass
  def await_branch(*args):
    assert args[1] == vehicle
    raise Pending
  monkeypatch.setattr(branches, 'await_correct_branch', await_branch)
  with pytest.raises(Pending):
    if vehicle == av.PORTER:
      detect(monkeypatch, None, {0: PORTER_BUS0, 2: PORTER_CAMERA})
    else:
      detect(monkeypatch, av.IONIQ5, {2: {0x50: 16}})
  assert calls == []
  assert {k: params.get(k) for k in av.IONIQ_SETTINGS} == before
  assert not params.get_bool('AutoVehicleReady')


@pytest.mark.parametrize('journal', [True, False])
def test_handoff_card_restart_never_requeries_firmware_or_constructs_interface(setup, monkeypatch, journal):
  params, calls = setup
  monkeypatch.setattr(branches, 'boot_id', lambda: 'same-boot')
  params.put_bool('AutoVehicleBranchPending', True)
  key = 'AutoVehicleBranchHandoff' if journal else 'AutoVehicleBranchRequest'
  params.put(key, {'boot': 'same-boot', 'vehicle': av.IONIQ5})
  class Pending(Exception):
    pass
  def await_branch(*args):
    raise Pending
  def forbidden_query(*args, **kwargs):
    raise AssertionError('firmware query restarted during noOutput handoff')
  monkeypatch.setattr(branches, 'await_correct_branch', await_branch)
  monkeypatch.setattr(car_helpers, 'fingerprint', forbidden_query)
  with pytest.raises(Pending):
    car_helpers.get_car(lambda **kw: [], lambda m: None, lambda e: None, False, False)
  assert calls == []
