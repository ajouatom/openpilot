from unittest.mock import Mock

import pytest

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.hyundai import carstate, hyundaicanfd
from opendbc.car.hyundai.values import CAR, DBC, HyundaiFlags


@pytest.fixture
def params(monkeypatch):
  params = Mock()
  params.get_int.return_value = 0
  params.get_bool.return_value = False
  params.get.return_value = '{0: {}, 1: {}, 2: {}}'
  monkeypatch.setattr(carstate, 'Params', lambda: params)
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: params)
  return params


def make_state(flags=0):
  return carstate.CarState(structs.CarParams(carFingerprint=CAR.HYUNDAI_SONATA, flags=int(flags), safetyConfigs=[{}]))


@pytest.mark.parametrize('setting, flags, eligible', [
  (0, 0, True), (1, 0, False), (2, 0, False), (3, 0, False),
  (0, HyundaiFlags.CAMERA_SCC, False),
])
def test_setting_and_effective_configuration(params, setting, flags, eligible):
  params.get_int.side_effect = lambda key: setting if key == 'HyundaiCameraSCC' else 0
  state = make_state(flags)
  assert bool(state.camera_scc_hint_enabled) == eligible
  assert not state.camera_scc_hint
  params.put_bool.assert_called_once_with('HyundaiCameraSccHint', False)
  parser = CANParser(DBC[CAR.HYUNDAI_SONATA][Bus.pt], [], 2)
  parser.seen_addresses.add(0x421)
  state._update_camera_scc_hint(parser, False)
  assert state.camera_scc_hint == eligible
  assert params.put_bool_nonblocking.call_count == int(eligible)


@pytest.mark.parametrize('canfd, dbc, address, size', [
  (False, 'hyundai_kia_generic', 0x421, 8),
  (True, 'hyundai_canfd_generated', 0x1A0, 32),
])
@pytest.mark.parametrize('camera_bus', [2, 6])
def test_onroad_reception_after_startup_does_not_add_validity_checks(params, canfd, dbc, address, size, camera_bus):
  state = make_state()
  state.controls_ready_count = carstate.READY_COUNT_OK + 1
  parser = CANParser(dbc, [], camera_bus)
  parser.controls_ready = True
  parsers = {Bus.cam: parser}

  # A matching ID on the powertrain bus, or the other platform's ID, is insufficient.
  other_address = 0x421 if canfd else 0x1A0
  parser.update([(1_000_000_000, [(address, bytes(size), camera_bus - 2), (other_address, bytes(8), camera_bus)])])
  state.monitor_fingerprint(parsers, canfd)
  assert not state.camera_scc_hint

  parser.update([(1_010_000_000, [(address, bytes(size), camera_bus)])])
  state.monitor_fingerprint(parsers, canfd)
  assert state.camera_scc_hint
  params.put_bool_nonblocking.assert_called_once_with('HyundaiCameraSccHint', True)
  assert not parser.addresses
  assert not parser.message_states
  assert parser.can_valid

  # Reception history remains useful when CAN is later disconnected; no repeated writes.
  parser.update([(10_000_000_000, [])])
  state.monitor_fingerprint(parsers, canfd)
  params.put_bool_nonblocking.assert_called_once()
  make_state()
  assert params.put_bool.call_count == 2  # A restarted card clears the previous hint.


def test_missing_dbc_message_does_not_raise_hint(params):
  state = make_state()
  parser = CANParser('hyundai_kia_generic', [], 2)
  parser.seen_addresses.add(0x1A0)
  state._update_camera_scc_hint(parser, True)
  assert not state.camera_scc_hint
  params.put_bool_nonblocking.assert_not_called()
