from collections import defaultdict

import pytest

from opendbc.car.hyundai import hyundaicanfd, interface
from opendbc.car import interfaces
from opendbc.car.hyundai.values import CAR, HyundaiSafetyFlags
from opendbc.car.structs import CarParams


class Config:
  def get_int(self, key):
    return {'HyundaiCameraSCC': 2, 'EnableRadarTracks': 3, 'CanfdHDA2': 1}.get(key, 0)

  def get_bool(self, key):
    return False

  def put_int(self, key, value):
    assert key == 'LongitudinalPersonalityMax'


@pytest.mark.parametrize('candidate', [CAR.HYUNDAI_IONIQ_5, CAR.HYUNDAI_IONIQ_5_PE, CAR.KIA_EV6, CAR.HYUNDAI_PORTER_II_EV])
@pytest.mark.parametrize('angle', [False, True])
def test_real_interface_selects_strict_profile_only_for_torque_ioniq5(monkeypatch, candidate, angle):
  config = Config()
  for module in (interface, hyundaicanfd, interfaces):
    monkeypatch.setattr(module, 'Params', lambda: config)
  fingerprint = defaultdict(dict, {0: {0x1CF: 8}, 1: {0x50: 16}, 2: {}})
  if angle:
    fingerprint[2][0xCB] = 24
  cp = interface.CarInterface.get_params(candidate, fingerprint, [], True, False, False)
  strict = candidate == CAR.HYUNDAI_IONIQ_5 and not angle
  assert bool(cp.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_STRICT_CONTROL) == (
    strict if cp.safetyConfigs[-1].safetyModel == CarParams.SafetyModel.hyundaiCanfd else False)
  if strict:
    assert cp.steerControlType == CarParams.SteerControlType.torque
    assert cp.safetyConfigs[-1].safetyParam == 8221


@pytest.mark.parametrize('camera,hda2', [(0, 0), (0, 1), (1, 1), (2, 0)])
def test_unverified_ioniq_wiring_keeps_its_existing_profile(monkeypatch, camera, hda2):
  config = Config()
  config.get_int = lambda key: {'HyundaiCameraSCC': camera, 'CanfdHDA2': hda2, 'EnableRadarTracks': 3}.get(key, 0)
  for module in (interface, hyundaicanfd, interfaces):
    monkeypatch.setattr(module, 'Params', lambda: config)
  cp = interface.CarInterface.get_params(CAR.HYUNDAI_IONIQ_5, defaultdict(dict), [], True, False, False)
  assert not cp.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_STRICT_CONTROL
