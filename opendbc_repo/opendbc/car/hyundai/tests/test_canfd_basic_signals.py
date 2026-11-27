from types import SimpleNamespace

import pytest

from opendbc.can import CANParser, CANPacker
from opendbc.car.hyundai.canfd_signals import SteeringRateSign, fresh_values, front_bsd
from opendbc.car.hyundai.rear_gap import tbt_bsd_inputs
from opendbc.car.hyundai.tests.test_rear_gap import inputs


def parser(name, addr, size, **values):
  return SimpleNamespace(bus_timeout=False, vl={name: values}, dat={addr: bytes(size)},
                         ts_nanos={name: {'COUNTER': 1_000_000_000}})


@pytest.mark.parametrize('fault', ['old', 'future', 'size', 'bus', 'missing'])
def test_optional_signal_requires_fresh_exact_frame(fault):
  p = parser('IMU_01_10MS', 0x4A, 32, IMU_YawRtVal=2.0)
  if fault == 'old':
    p.ts_nanos['IMU_01_10MS']['COUNTER'] -= 150_000_001
  elif fault == 'future':
    p.ts_nanos['IMU_01_10MS']['COUNTER'] += 1
  elif fault == 'size':
    p.dat[0x4A] = bytes(24)
  elif fault == 'bus':
    p.bus_timeout = True
  else:
    p.ts_nanos.clear()
  assert fresh_values(p, 'IMU_01_10MS', 0x4A, 32, 1_000_000_000) is None


def test_front_bsd_on_separate_camera_bus_and_loss():
  _, rear, _ = inputs()
  front = parser('BLINDSPOTS_FRONT_CORNER_1', 0x1E5, 16, INDICATOR_LEFT=0, INDICATOR_RIGHT=1)
  assert tbt_bsd_inputs(rear, 1_000_000_000, front) == (True, True)
  front.vl['BLINDSPOTS_FRONT_CORNER_1']['INDICATOR_RIGHT'] = 0
  assert tbt_bsd_inputs(rear, 1_000_000_000, front) == (True, False)
  front.ts_nanos.clear()
  assert front_bsd(front, 1_000_000_000) == (True, True)
  assert tbt_bsd_inputs(rear, 1_000_000_000, front) == (True, True)


def test_shared_unsigned_rate_uses_same_sensor_direction_and_resets_after_gap():
  p = parser('STEERING_SENSORS', 0x125, 16, STEERING_ANGLE=0, STEERING_RATE=20)
  sign = SteeringRateSign()
  assert sign.update(p, 1_000_000_000) == 0
  for now, angle, expected in [(1_010_000_000, -1, 20), (1_020_000_000, 1, -20), (1_030_000_000, 1, -20)]:
    p.ts_nanos['STEERING_SENSORS']['COUNTER'] = now
    p.vl['STEERING_SENSORS']['STEERING_ANGLE'] = angle
    assert sign.update(p, now) == expected
  assert sign.update(p, 2_000_000_000) == 0
  p.ts_nanos['STEERING_SENSORS']['COUNTER'] = 2_000_000_000
  assert sign.update(p, 2_000_000_000) == 0


@pytest.mark.parametrize('byte,expected', [(0, 0), (1, 0), (2, 0), (4, 1), (8, 0), (128, 0)])
def test_brake_lamp_decodes_only_observed_bit(byte, expected):
  p = CANParser('hyundai_canfd_generated', [('BRAKE_LIGHT_412', 0)], 0)
  p.update([(1_000_000_000, [(0x412, bytes(6) + bytes([byte, 0]), 0)])])
  assert p.vl['BRAKE_LIGHT_412']['STOP_LAMP'] == expected


def test_imu_and_bsd_real_dbc_decoding():
  packer = CANPacker('hyundai_canfd_generated')
  names = [('IMU_01_10MS', {'IMU_YawRtVal': -10}),
           ('BLINDSPOTS_FRONT_CORNER_1', {'INDICATOR_LEFT': 1, 'INDICATOR_RIGHT': 0})]
  p = CANParser('hyundai_canfd_generated', [(name, 0) for name, _ in names], 0)
  p.update([(1_000_000_000, [packer.make_can_msg(name, 0, values) for name, values in names])])
  assert p.vl['IMU_01_10MS']['IMU_YawRtVal'] == pytest.approx(-10)
  assert front_bsd(p, 1_000_000_000) == (True, False)
