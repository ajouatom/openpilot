import copy

import pytest

from openpilot.common import auto_vehicle as av
from openpilot.common.params import Params


@pytest.fixture
def params(tmp_path):
  p = Params(str(tmp_path / 'params'))
  for key in av.IONIQ_SETTINGS:
    p.put(key, p.get_default_value(key))
  p.put('CarName', av.PORTER)
  p.put('CarSelected3', av.CAR_LABELS[av.PORTER])
  p.put_bool('PorterAutoSetMain', True)
  p.put_bool('PorterAutoCancelBelow30', True)
  p.put_bool('PorterGapUse', True)
  for key in av.RECALIBRATION_KEYS:
    p.put(key, {'test': 'learned'} if key == 'LiveParameters' else b'old calibration/learning')
  return p


def snapshot(params):
  return {(k.decode() if isinstance(k, bytes) else k): params.get(k) for k in params.all_keys()}


def test_only_requested_settings_and_recalibration_change_and_porter_returns_exactly(params):
  params.put('CalibrationParams', b'untouched camera calibration')
  params.put('LiveTorqueParameters', b'untouched learned torque')
  params.put_int('PorterGap1', 45)
  params.put_int('TFollowGap1', 65)
  params.put_int('AlwaysLateral', 1)
  before = snapshot(params)
  av.apply_vehicle_settings(params, av.IONIQ5, av.prepare_settings(params))
  after = snapshot(params)
  changed = {k for k in before if before[k] != after[k]}
  assert changed <= set(av.IONIQ_SETTINGS) | set(av.RECALIBRATION_KEYS) | {'AutoVehiclePorterBase', 'CarName', 'CarSelected3'}
  assert all(params.get(key) is None for key in av.RECALIBRATION_KEYS)
  assert {k: params.get(k) for k in av.IONIQ_SETTINGS} == av.IONIQ_SETTINGS
  av.apply_vehicle_settings(params, av.PORTER, av.prepare_settings(params))
  restored = snapshot(params)
  exempt = {'AutoVehiclePorterBase'} | set(av.RECALIBRATION_KEYS)
  assert {k: restored[k] for k in before if k not in exempt} == {
    k: v for k, v in before.items() if k not in exempt
  }


def test_ioniq_values_never_overwrite_porter_baseline(params):
  base = {k: params.get(k) for k in av.IONIQ_SETTINGS}
  for _ in range(3):
    av.apply_vehicle_settings(params, av.IONIQ5, av.prepare_settings(params))
    assert params.get('AutoVehiclePorterBase')['porter'] == base
  av.apply_vehicle_settings(params, av.PORTER, av.prepare_settings(params))
  assert {k: params.get(k) for k in av.IONIQ_SETTINGS} == base
  params.put_int('CustomSR', 155)
  av.apply_vehicle_settings(params, av.IONIQ5, av.prepare_settings(params))
  av.apply_vehicle_settings(params, av.PORTER, av.prepare_settings(params))
  assert params.get_int('CustomSR') == 155


def test_missing_porter_baseline_cannot_assume_current_ioniq_values(params):
  params.put('CarName', av.IONIQ5)
  params.put('CarSelected3', av.CAR_LABELS[av.IONIQ5])
  before = snapshot(params)
  with pytest.raises(ValueError, match='Save the Porter baseline'):
    av.apply_vehicle_settings(params, av.IONIQ5, av.prepare_settings(params))
  assert snapshot(params) == before


@pytest.mark.parametrize('failure_key', list(av.IONIQ_SETTINGS) + ['CarSelected3', 'CarName'])
def test_failed_write_rolls_back_all_seven_and_vehicle_identity(params, failure_key):
  data = av.prepare_settings(params)
  before = snapshot(params)

  class FailOnce:
    failed = False

    def __getattr__(self, name):
      return getattr(params, name)

    def put(self, key, value):
      if key == failure_key and not self.failed:
        self.failed = True
        raise OSError('simulated disk failure')
      params.put(key, value)

  with pytest.raises(OSError):
    av.apply_vehicle_settings(FailOnce(), av.IONIQ5, data)
  assert snapshot(params) == before


def test_power_loss_recovers_before_next_snapshot(params):
  data = av.prepare_settings(params)
  before = snapshot(params)

  class PowerLoss:
    def __getattr__(self, name):
      return getattr(params, name)

    def put(self, key, value):
      if key == 'CustomSR':
        raise SystemExit('simulated power loss')
      params.put(key, value)

  with pytest.raises(SystemExit):
    av.apply_vehicle_settings(PowerLoss(), av.IONIQ5, data)
  assert params.get_int('HyundaiCameraSCC') == 2
  assert 'pending' in params.get('AutoVehiclePorterBase')
  av.prepare_settings(params)
  assert snapshot(params) == before


@pytest.mark.parametrize('bad', [[], {'version': 2}, {'version': 1, 'porter': {'CustomSR': 140}, 'active': av.PORTER}])
def test_corrupt_baseline_never_applies_partial_defaults(params, bad):
  params.put('AutoVehiclePorterBase', bad)
  before = snapshot(params)
  with pytest.raises(ValueError):
    av.prepare_settings(params)
  assert snapshot(params) == before


def test_unknown_vehicle_does_not_write_settings(params):
  data = av.prepare_settings(params)
  before = snapshot(params)
  with pytest.raises(ValueError):
    av.apply_vehicle_settings(params, 'HYUNDAI_IONIQ_5_PE', copy.deepcopy(data))
  assert snapshot(params) == before


def test_live_ready_session_cannot_apply_another_vehicle(params):
  data = av.prepare_settings(params)
  params.put_bool('AutoVehicleReady', True)
  before = snapshot(params)
  with pytest.raises(RuntimeError):
    av.apply_vehicle_settings(params, av.IONIQ5, data)
  assert snapshot(params) == before


def test_both_vehicle_changes_reset_but_same_car_restarts_keep_calibration(params):
  original = {key: params.get(key) for key in av.RECALIBRATION_KEYS}
  assert not av.apply_vehicle_settings(params, av.PORTER, av.prepare_settings(params))
  assert {key: params.get(key) for key in av.RECALIBRATION_KEYS} == original
  assert av.apply_vehicle_settings(params, av.IONIQ5, av.prepare_settings(params))
  assert all(params.get(key) is None for key in av.RECALIBRATION_KEYS)
  for key, value in original.items():
    params.put(key, value)
  assert not av.apply_vehicle_settings(params, av.IONIQ5, av.prepare_settings(params))
  assert {key: params.get(key) for key in av.RECALIBRATION_KEYS} == original
  assert av.apply_vehicle_settings(params, av.PORTER, av.prepare_settings(params))
  assert all(params.get(key) is None for key in av.RECALIBRATION_KEYS)


def test_failure_during_recalibration_restores_all_previous_values(params):
  data = av.prepare_settings(params)
  before = snapshot(params)

  class FailReset:
    def __getattr__(self, key):
      return getattr(params, key)

    def remove(self, key):
      if key == 'LiveTorqueParameters':
        raise OSError('reset failed')
      params.remove(key)

  with pytest.raises(OSError):
    av.apply_vehicle_settings(FailReset(), av.IONIQ5, data)
  assert snapshot(params) == before


def test_power_loss_after_reset_recovers_old_calibration_before_retry(params):
  data = av.prepare_settings(params)
  before = snapshot(params)

  class PowerLoss:
    def __getattr__(self, key):
      return getattr(params, key)

    def put(self, key, value):
      if key == 'CarName':
        raise SystemExit('power loss after clearing calibration')
      params.put(key, value)

  with pytest.raises(SystemExit):
    av.apply_vehicle_settings(PowerLoss(), av.IONIQ5, data)
  assert all(params.get(key) is None for key in av.RECALIBRATION_KEYS)
  data = av.prepare_settings(params)
  assert snapshot(params) == before
  av.apply_vehicle_settings(params, av.IONIQ5, data)
  assert all(params.get(key) is None for key in av.RECALIBRATION_KEYS)
  assert 'pending' not in params.get('AutoVehiclePorterBase')


def test_calibrator_starts_uncalibrated_after_switch_and_reuses_same_car(params, monkeypatch):
  from openpilot.cereal import log, messaging
  from openpilot.selfdrive.locationd import calibrationd

  monkeypatch.setattr(calibrationd, 'Params', lambda: params)
  msg = messaging.new_message('liveCalibration')
  msg.liveCalibration.validBlocks = 20
  msg.liveCalibration.rpyCalib = [0., 0., 0.]
  msg.liveCalibration.wideFromDeviceEuler = [0., 0., 0.]
  msg.liveCalibration.height = [1.22]
  params.put('CalibrationParams', msg.to_bytes())
  assert calibrationd.Calibrator(param_put=True).valid_blocks == 20
  av.apply_vehicle_settings(params, av.PORTER, av.prepare_settings(params))
  assert calibrationd.Calibrator(param_put=True).valid_blocks == 20
  av.apply_vehicle_settings(params, av.IONIQ5, av.prepare_settings(params))
  calibrator = calibrationd.Calibrator(param_put=True)
  assert calibrator.valid_blocks == 0
  assert calibrator.cal_status == log.LiveCalibrationData.Status.uncalibrated
