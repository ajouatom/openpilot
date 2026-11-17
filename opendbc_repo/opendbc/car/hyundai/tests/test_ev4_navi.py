from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.hyundai import carstate, hyundaicanfd
from opendbc.car.hyundai.values import CAR, HyundaiFlags


@pytest.fixture(params=[False, True], ids=['hda1', 'hda2'])
def ev4(request, monkeypatch):
  params = Mock()
  params.get_int.return_value = 0
  params.get_bool.side_effect = lambda key: key == 'ControlsReady'
  params.get.return_value = '{0: {}, 1: {}, 2: {}}'
  monkeypatch.setattr(carstate, 'Params', lambda: params)
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: params)
  flags = HyundaiFlags.CANFD | HyundaiFlags.EV | HyundaiFlags.CANFD_CAMERA_SCC
  if request.param:
    flags |= HyundaiFlags.CANFD_HDA2
  cp = structs.CarParams(carFingerprint=CAR.KIA_EV4, flags=int(flags), safetyConfigs=[{}])
  state = carstate.CarState(cp)
  state.is_metric = True
  state.vehicleNaviCanControl = 1
  parsers = state.get_can_parsers(cp)
  for parser in parsers.values():
    parser.controls_ready = True
  return state, parsers, CANPacker('hyundai_canfd_generated')


def receive(state, parsers, time_s, frames=()):
  for parser in parsers.values():
    parser.update([int(time_s * 1e9), list(frames)])
  state.monitor_fingerprint(parsers, True)


def output(state, parser, speed=0.):
  ret = SimpleNamespace(speedLimit=speed, vEgo=0.)
  state._update_vehicle_navi_events(parser, ret, False)
  return ret


def test_ev4_logged_camera_is_cached_before_fingerprint_registration(ev4):
  state, parsers, _ = ev4
  parser = parsers[Bus.pt]
  # EV4 0000030c/8, camera preview 60 km/h at 2000 m.
  receive(state, parsers, 1., [(0x93, bytes.fromhex('89090cffffffffffd1000000d0e70814ffffffffffffffff'), parser.bus)])
  state.controls_ready_count = 122
  receive(state, parsers, 1.2)
  assert state.navi_profile_4be is parser.vl[carstate.CANFD_NAVI_PROFILE_MSG]
  ret = output(state, parser)
  assert ret.vehicleNaviAvailable and ret.vehicleNaviActive
  assert ret.speedLimit == 60.
  assert state.vehicleNaviCameraTarget == 2000.


@pytest.mark.parametrize('mode,route,expected', [(0, 1, False), (1, 0, True), (2, 0, False), (2, 1, True), (3, 1, True)])
def test_ev4_bump_uses_existing_mode_and_route_admission(ev4, mode, route, expected):
  state, parsers, packer = ev4
  parser = parsers[Bus.pt]
  state.vehicleNaviCanControl = mode
  state.controls_ready_count = 122
  raw = (8 << 13) | (route << 22) | (3 << 24)
  frames = [
    packer.make_can_msg(carstate.CANFD_NAVI_SEGMENT_MSG, parser.bus,
                       {f'BYTE_{i+1}': (raw >> (8*i)) & 255 for i in range(8)}),
    packer.make_can_msg(carstate.CANFD_NAVI_PROFILE_MSG, parser.bus,
                       {'PROLONG_VALUE': 6, 'PROLONG_OFFSET': 150, 'PROLONG_PATH_INDEX': 8, 'PROLONG_PROFILE_TYPE': 16}),
  ]
  receive(state, parsers, 1., frames)
  ret = output(state, parser)
  assert ret.vehicleNaviActive == expected
  assert ret.speedBumpDistance == (150 if expected else 0)


def test_ev4_padding_does_not_refresh_route_or_keep_bump_after_timeout(ev4):
  state, parsers, packer = ev4
  parser = parsers[Bus.pt]
  state.vehicleNaviCanControl = 2
  state.controls_ready_count = 122
  raw = (8 << 13) | (1 << 22) | (3 << 24)
  receive(state, parsers, 1., [packer.make_can_msg(carstate.CANFD_NAVI_SEGMENT_MSG, parser.bus,
                             {f'BYTE_{i+1}': (raw >> (8*i)) & 255 for i in range(8)})])
  output(state, parser)
  state._add_vehicle_navi_event('bump', 0, 6, 100.)
  padding = (0x99, bytes.fromhex('502627ffffffffffffffffffffffffffffffffffffffffff'), parser.bus)
  receive(state, parsers, 1.1, [padding])
  assert output(state, parser).speedBumpDistance == 100.
  assert state.vehicleNaviSegmentTimestamp == 1_000_000_000
  receive(state, parsers, 3.1, [padding])
  assert output(state, parser).speedBumpDistance == 0.
  assert state.vehicleNaviRouteState == 0


def test_ev4_unvalidated_position_cannot_activate_section(ev4):
  state, parsers, _ = ev4
  parser = parsers[Bus.pt]
  state.controls_ready_count = 122
  # Real EV4 0x092 payload. The proposed legacy-position mapping decodes an
  # impossible 384 km/h average and would incorrectly establish a section cap.
  receive(state, parsers, 1., [(0x92, bytes.fromhex('24da2affffffffffd40c0070113dffff'), parser.bus)])
  assert not output(state, parser, 100.).vehicleNaviSectionActive


def test_ev4_logged_hda_marks_availability_without_profile(ev4):
  state, parsers, _ = ev4
  parser = parsers[Bus.pt]
  state.controls_ready_count = 122
  receive(state, parsers, 1., [(0x364, bytes.fromhex('a4517e0000000000416e9a0900000100'), parser.bus)])
  ret = output(state, parser)
  assert ret.vehicleNaviAvailable
  assert not ret.vehicleNaviActive
  assert state.hda_info_4a3['SPEED_LIMIT'] == 110
  assert state.hda_info_4a3['CountryCode'] == 410


def test_ev4_logged_bump_uses_profile_distance(ev4):
  state, parsers, _ = ev4
  parser = parsers[Bus.pt]
  state.controls_ready_count = 122
  receive(state, parsers, 1., [(0x93, bytes.fromhex('7b04ecffffffffff060000005de40804ffffffffffffffff'), parser.bus)])
  ret = output(state, parser)
  assert ret.vehicleNaviActive
  assert ret.speedBumpDistance == 1117.


def test_ev4_missing_optional_navigation_does_not_add_timeouts(ev4):
  state, parsers, packer = ev4
  parser = parsers[Bus.pt]
  for frame in range(1, 301):
    receive(state, parsers, frame * .02, [packer.make_can_msg('CRUISE_BUTTONS', parser.bus, {'COUNTER': frame % 16})])
  assert parser.can_valid
  assert not output(state, parser).vehicleNaviAvailable
