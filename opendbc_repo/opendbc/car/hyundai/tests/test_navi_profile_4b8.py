from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, structs
from opendbc.car.hyundai import carstate, hyundaicanfd
from opendbc.car.hyundai.values import CAR, HyundaiFlags


@pytest.fixture(params=[CAR.GENESIS_GV80, CAR.HYUNDAI_IONIQ_5, CAR.KIA_EV4, CAR.KIA_PV5])
def navi(request, monkeypatch):
  params = Mock()
  params.get_int.return_value = 0
  params.get_bool.side_effect = lambda key: key == 'ControlsReady'
  params.get.return_value = '{0: {}, 1: {}, 2: {}}'
  monkeypatch.setattr(carstate, 'Params', lambda: params)
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: params)

  def make(hda2=False):
    flags = HyundaiFlags.CANFD | (HyundaiFlags.CANFD_HDA2 if hda2 else 0)
    cp = structs.CarParams(carFingerprint=request.param, flags=int(flags), safetyConfigs=[{}])
    state = carstate.CarState(cp)
    state.is_metric = True
    state.vehicleNaviCanControl = 1
    parsers = state.get_can_parsers(cp)
    return state, parsers
  return make


def receive(state, parsers, time_s, frames=()):
  for parser in parsers.values():
    parser.update([int(time_s * 1e9), list(frames)])
  state.monitor_fingerprint(parsers, True)


def output(state, parsers, speed=0., warning=False):
  ret = SimpleNamespace(speedLimit=speed, vEgo=0.)
  state._update_vehicle_navi_events(parsers[Bus.pt], ret, warning, parsers.get(Bus.alt))
  return ret


@pytest.mark.parametrize('hda2', [False, True])
def test_recorded_4b8_bump_is_received_on_every_supported_navi_variant(navi, hda2):
  state, parsers = navi(hda2)
  parser = parsers[Bus.pt]
  assert parser.message_states[0x4b8].ignore_alive
  # Actual GV80 0000000b/3: type 16, bump 6, distance 317 m, path 8.
  # Send before the normal startup cache handoff, then publish without a repeat.
  receive(state, parsers, 1., [(0x4b8, bytes.fromhex('060000003d410804'), parser.bus)])
  state.controls_ready_count = 122
  receive(state, parsers, 1.2)
  ret = output(state, parsers)
  assert ret.vehicleNaviAvailable and ret.vehicleNaviActive
  assert ret.speedBumpDistance == 317.
  state.totalDistance = 100.
  receive(state, parsers, 1.3)
  assert output(state, parsers).speedBumpDistance == 217.
  assert len(state.vehicleNaviEvents) == 1


@pytest.mark.parametrize('size', [0, 4, 7, 12, 24])
def test_same_address_with_other_length_is_not_a_profile(navi, size):
  state, parsers = navi()
  state.controls_ready_count = 122
  payload = bytes.fromhex('060000003d410804').ljust(size, b'\x00')[:size]
  receive(state, parsers, 1., [(0x4b8, payload, parsers[Bus.pt].bus)])
  ret = output(state, parsers)
  assert not ret.vehicleNaviAvailable and not ret.vehicleNaviActive
  assert not state.vehicleNaviEvents


def test_absent_or_other_bus_profile_does_not_enable_navigation(navi):
  state, parsers = navi()
  state.controls_ready_count = 122
  receive(state, parsers, 1.)
  assert not output(state, parsers).vehicleNaviAvailable
  receive(state, parsers, 1.1, [(0x4b8, bytes.fromhex('060000003d410804'), (parsers[Bus.pt].bus + 1) % 3)])
  assert not output(state, parsers).vehicleNaviAvailable
  # Optional parsing also accepts the first legitimate message after startup.
  receive(state, parsers, 10., [(0x4b8, bytes.fromhex('060000003d410804'), parsers[Bus.pt].bus)])
  assert output(state, parsers).speedBumpDistance == 317.


@pytest.mark.parametrize('mode', [0, 1, 2, 3])
def test_alternate_profile_keeps_existing_mode_admission(navi, mode):
  state, parsers = navi()
  state.vehicleNaviCanControl = mode
  state.controls_ready_count = 122
  receive(state, parsers, 1., [(0x4b8, bytes.fromhex('060000003d410804'), parsers[Bus.pt].bus)])
  assert output(state, parsers).vehicleNaviActive == (mode == 1)


@pytest.mark.parametrize('alternate_newer', [False, True])
def test_both_profile_addresses_follow_timestamp_order(alternate_newer):
  from opendbc.car.hyundai.tests.test_speed_camera import _car_state
  state = _car_state()
  state.vehicleNaviCanControl = 1
  parser = CANParser('hyundai_canfd_generated', [('NEW_MSG_4BE', float('nan')), ('NEW_MSG_4B8', float('nan'))], 0)
  packer = CANPacker('hyundai_canfd_generated')
  for name, value, timestamp in [('NEW_MSG_4BE', 0xd7, 1 if alternate_newer else 2),
                                 ('NEW_MSG_4B8', 0xb7, 2 if alternate_newer else 1)]:
    parser.update([timestamp * 1_000_000_000, [packer.make_can_msg(name, 0, {'PROLONG_VALUE': value, 'PROLONG_PROFILE_TYPE': 16})]])
  parser._last_update_nanos = 2_000_000_000
  state.navi_profile_4be = parser.vl['NEW_MSG_4BE']
  state.navi_profile_4b8 = parser.vl['NEW_MSG_4B8']
  ret = SimpleNamespace(speedLimit=50., vEgo=0.)
  state._update_vehicle_navi_events(parser, ret, True)
  assert state.vehicleNaviSpeedZoneSpeed == (50 if alternate_newer else 60)


def test_duplicate_addresses_do_not_queue_two_bumps(navi):
  state, parsers = navi()
  if state.canfd_wrapped_navi or state.canfd_ev4_navi:
    primary = CANPacker('hyundai_canfd_generated').make_can_msg(state.navi_profile_msg, parsers[Bus.pt].bus,
                                                              {'PROLONG_VALUE': 6, 'PROLONG_OFFSET': 317, 'PROLONG_PROFILE_TYPE': 16})
  else:
    primary = (0x4be, bytes.fromhex('060000003d410804'), parsers[Bus.pt].bus)
  state.controls_ready_count = 122
  receive(state, parsers, 1., [primary, (0x4b8, bytes.fromhex('060000003d410804'), parsers[Bus.pt].bus)])
  assert output(state, parsers).speedBumpDistance == 317.
  assert len(state.vehicleNaviEvents) == 1


def test_alternate_bump_obeys_route_path_and_timeout(navi):
  state, parsers = navi()
  state.vehicleNaviCanControl = 3
  state.controls_ready_count = 122
  bus = parsers[Bus.pt].bus
  packer = CANPacker('hyundai_canfd_generated')
  # Calculated route, path 9: the recorded path-8 bump must not be admitted.

  def route(path):
    raw = (path << 13) | (1 << 22) | (3 << 24)
    return packer.make_can_msg(state.navi_segment_msg, bus,
                               {f'BYTE_{i+1}': (raw >> (8*i)) & 255 for i in range(8)})
  bump = (0x4b8, bytes.fromhex('060000003d410804'), bus)
  receive(state, parsers, 1., [route(9), bump])
  assert output(state, parsers).speedBumpDistance == 0.
  receive(state, parsers, 1.1, [route(8), bump])
  # PV5 intentionally has no route-proof receiver; keep its existing exclusion.
  assert output(state, parsers).speedBumpDistance == (0. if state.canfd_wrapped_navi else 317.)
  receive(state, parsers, 3.2)
  assert output(state, parsers).speedBumpDistance == 0.


@pytest.mark.parametrize('value,profile_type', [(4, 16), (9, 16), (6, 15)])
def test_unrecognized_alternate_profile_does_not_create_event(navi, value, profile_type):
  state, parsers = navi()
  state.controls_ready_count = 122
  frame = CANPacker('hyundai_canfd_generated').make_can_msg('NEW_MSG_4B8', parsers[Bus.pt].bus,
             {'PROLONG_VALUE': value, 'PROLONG_OFFSET': 317, 'PROLONG_PROFILE_TYPE': profile_type})
  receive(state, parsers, 1., [frame])
  assert not output(state, parsers).vehicleNaviActive
  assert not state.vehicleNaviEvents


@pytest.mark.parametrize('warning', [False, True])
def test_alternate_camera_keeps_pv5_current_warning_requirement(navi, warning):
  state, parsers = navi()
  state.vehicleSpeedCameraDistanceTime = 6.
  state.controls_ready_count = 122
  frame = CANPacker('hyundai_canfd_generated').make_can_msg('NEW_MSG_4B8', parsers[Bus.pt].bus,
             {'PROLONG_VALUE': 0xb1, 'PROLONG_OFFSET': 100, 'PROLONG_PROFILE_TYPE': 16})
  receive(state, parsers, 1., [frame])
  ret = output(state, parsers, speed=50., warning=warning)
  assert ret.vehicleNaviActive == (warning or not state.canfd_wrapped_navi)
  assert state.vehicleNaviCameraTarget == (100. if ret.vehicleNaviActive else None)
