from types import SimpleNamespace

import pytest

from opendbc.can import CANParser, CANPacker
from opendbc.can.parser import get_raw_value
from opendbc.car import structs
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from openpilot.cereal import log


T = 1_000_000_000


@pytest.fixture
def cluster(monkeypatch):
  hyundaicanfd.create_ccnc_messages.__dict__.clear()
  keys = {'CcncClusterDisplay': 0, 'LaneLineCheck': 0}
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: SimpleNamespace(get_int=lambda key: keys.get(key, 0), get=lambda key: '0'))
  packer = CANPacker('hyundai_canfd_generated')
  stock_packer = CANPacker('hyundai_canfd_generated')
  parser = CANParser('hyundai_canfd_generated', [('ADRV_0x161', 20)], 2)
  definition = packer.dbc.name_to_msg['ADRV_0x161']
  cp = SimpleNamespace(flags=HyundaiFlags.CANFD | HyundaiFlags.CAMERA_SCC, carFingerprint=CAR.KIA_PV5, notCar=False)
  state = SimpleNamespace(
    cp_cam=parser, adrv_0x161=parser.vl[0x161], out=structs.CarState(canValid=True), modelV2=log.ModelDataV2.new_message(),
    lfahda_cluster=None, cruise_buttons_msg=None, adrv_0x200=None, adrv_0x1ea=None, ccnc_0x162=None,
    is_metric=True, paddle_button_prev=0, softHoldActive=0, trailer_connected=False,
  )
  control = structs.CarControl(enabled=True, latActive=True)
  control.hudControl.leftLaneVisible = control.hudControl.rightLaneVisible = True

  def receive(left=2, right=8, stamp=T, counter=1, bus=2):
    packet = stock_packer.make_can_msg('ADRV_0x161', bus, {
      'COUNTER': counter, 'LANELINE_LEFT': left, 'LANELINE_RIGHT': right,
      'LANELINE_LEFT_POSITION': 12, 'LANELINE_RIGHT_POSITION': 14, 'LANELINE_CURVATURE': 7,
    })
    parser.update([(stamp, [packet])])
    return packet

  def send(now=T, frame=5):
    packet, = hyundaicanfd.create_ccnc_messages(
      cp, packer, SimpleNamespace(ECAN=0, CAM=2), frame, control, state, control.hudControl,
      0, False, False, 0, False, 0, 0, now_nanos=now)
    assert packet[0] == 0x161 and packet[2] == 0
    values = {k: get_raw_value(packet[1], sig) for k, sig in definition.sigs.items()}
    assert values['CHECKSUM'] == hyundaicanfd.hkg_can_fd_checksum(0x161, None, bytearray(packet[1]))
    return values, packet

  yield SimpleNamespace(cp=cp, state=state, control=control, parser=parser, packer=packer, keys=keys, receive=receive, send=send)
  hyundaicanfd.create_ccnc_messages.__dict__.clear()


@pytest.mark.parametrize('display', (0, 1))
@pytest.mark.parametrize('active', (False, True))
@pytest.mark.parametrize('left,right,expected', ((2, 8, (2, 8)), (8, 2, (8, 2)), (8, 8, (8, 8)),
                                               (4, 8, (4, 8)), (0, 1, (0, 1)), (6, 2, (6, 2)), (15, 8, (2, 8))))
def test_preserves_original_display_without_inventing_unobserved_codes(cluster, display, active, left, right, expected):
  cluster.keys['CcncClusterDisplay'] = display
  cluster.control.enabled = cluster.control.latActive = active
  cluster.receive(left, right)
  before = cluster.state.adrv_0x161.copy()
  values, _ = cluster.send()
  assert (values['LANELINE_LEFT'], values['LANELINE_RIGHT']) == expected
  assert values['COUNTER'] == 2
  assert cluster.state.adrv_0x161 == before
  assert (cluster.state.out.leftLaneLine, cluster.state.out.rightLaneLine) == (0, 0)


@pytest.mark.parametrize('side', ('left', 'right'))
@pytest.mark.parametrize('reason,expected', (('departure', 4), ('departure_hidden', 1), ('blindspot', 4),
                                           ('boundary', 4), ('available', 6), ('trailer', 2), ('stock_warning', 4)))
def test_existing_warning_and_availability_priority(cluster, side, reason, expected):
  cluster.receive(4 if reason == 'stock_warning' else 8, 4 if reason == 'stock_warning' else 8)
  frame = 55 if reason == 'departure_hidden' else 5
  if reason.startswith('departure'):
    setattr(cluster.control.hudControl, side + 'LaneDepart', True)
  elif reason == 'blindspot':
    setattr(cluster.state.out, side + 'Blindspot', True)
  elif reason == 'boundary':
    cluster.keys['LaneLineCheck'] = 1
    setattr(cluster.state.out, side + 'LaneLine', 11)
  elif reason in ('available', 'stock_warning'):
    setattr(cluster.state.modelV2.meta, 'laneChangeAvailable' + side.title(), True)
  elif reason == 'trailer':
    cluster.state.trailer_connected = True
  values, _ = cluster.send(frame=frame)
  assert values['LANELINE_' + side.upper()] == expected
  if reason != 'trailer':
    assert values['LANELINE_' + ('RIGHT' if side == 'left' else 'LEFT')] == (4 if reason == 'stock_warning' else 8)


@pytest.mark.parametrize('age,expected', ((0, 8), (250_000_000, 8), (250_000_001, 2), (-1, 2)))
def test_actual_controller_clock_expires_source_even_when_all_can_stops(cluster, age, expected):
  cluster.receive()
  # No parser update: its receive clock freezes, but controller time advances.
  assert cluster.send(now=T + age)[0]['LANELINE_RIGHT'] == expected


@pytest.mark.parametrize('bus', (0, 1, 128, 130))
def test_other_buses_and_transmit_echo_cannot_supply_stock_pattern(cluster, bus):
  cluster.receive(bus=bus)
  assert cluster.send()[0]['LANELINE_RIGHT'] == 2


def test_bad_crc_cannot_refresh_last_good_observation(cluster):
  address, data, bus = cluster.receive()
  bad = (address, bytes([data[0] ^ 1]) + data[1:], bus)
  cluster.parser.update([(T + 200_000_000, [bad])])
  assert cluster.send(now=T + 200_000_000)[0]['LANELINE_RIGHT'] == 8
  assert cluster.send(now=T + 250_000_001)[0]['LANELINE_RIGHT'] == 2


@pytest.mark.parametrize('fault', ('can_invalid', 'short', 'checksum_disabled', 'counter_disabled', 'wrong_camera', 'wrong_template'))
def test_invalid_source_falls_back(cluster, fault):
  cluster.receive()
  if fault == 'can_invalid':
    cluster.state.out.canValid = False
  elif fault == 'short':
    cluster.parser.dat[0x161] = cluster.parser.dat[0x161][:-1]
  elif fault in ('checksum_disabled', 'counter_disabled'):
    setattr(cluster.parser.message_states[0x161], 'ignore_' + fault.split('_')[0], True)
  elif fault == 'wrong_camera':
    cluster.parser.bus = 1
  else:
    cluster.state.adrv_0x161 = cluster.state.adrv_0x161.copy()
  assert cluster.send()[0]['LANELINE_RIGHT'] == 2


@pytest.mark.parametrize('delta', (0, -50_000_000))
def test_ambiguous_and_out_of_order_samples_cannot_restore_dashed(cluster, delta):
  cluster.receive(2, 2)
  cluster.receive(8, 8, stamp=T + delta, counter=2)
  assert cluster.send()[0]['LANELINE_RIGHT'] == 2
  cluster.receive(2, 8, stamp=T + 50_000_000, counter=3)
  assert cluster.send(now=T + 50_000_000)[0]['LANELINE_RIGHT'] == 8


@pytest.mark.parametrize('car', [c for c in CAR if c != CAR.KIA_PV5 and c.config.flags & HyundaiFlags.CANFD] + [None])
@pytest.mark.parametrize('display', (0, 1))
def test_other_canfd_vehicles_keep_identical_packets(cluster, monkeypatch, car, display):
  cluster.cp.carFingerprint = car
  cluster.keys['CcncClusterDisplay'] = display
  cluster.receive()
  cluster.state.out.leftLaneLine, cluster.state.out.rightLaneLine = 11, 10
  counters = cluster.packer.counters.copy()
  actual = cluster.send()[1]
  cluster.packer.counters = counters
  monkeypatch.setattr(hyundaicanfd, '_pv5_stock_lane_lines', lambda *args: None)
  assert actual == cluster.send()[1]


def test_not_car_and_missing_controller_time_keep_legacy(cluster):
  cluster.receive()
  cluster.cp.notCar = True
  assert cluster.send()[0]['LANELINE_RIGHT'] == 2
  cluster.cp.notCar = False
  assert cluster.send(now=None)[0]['LANELINE_RIGHT'] == 2
