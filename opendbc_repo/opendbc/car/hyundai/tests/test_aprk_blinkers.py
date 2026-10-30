from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.can.parser import get_raw_value
from opendbc.car import structs
from opendbc.car.hyundai import carcontroller
from opendbc.car.hyundai.carcontroller import CarController

D, P = structs.CarState.GearShifter.drive, structs.CarState.GearShifter.park


class Crossing:
  """LaneCrossing stand-in: reports `side` once, on the first allowed call after it is set."""
  def __init__(self):
    self.side = 0

  def update(self, md, t, allowed, torque=None):
    side, self.side = (self.side, 0) if allowed else (0, self.side)
    return side


def make_cc(control=True, test=0, spas=0):
  return SimpleNamespace(blinker_control=control, blinker_test=test, aprk_state=0, aprk_frame=0, lane_crossing=Crossing(),
                         blinker_side=0, blinker_pulse_end=0, frame=0, yellow_until=[0, 0], CAN=SimpleNamespace(ECAN=0, ACAN=1),
                         packer=CANPacker('hyundai_canfd_generated'), spas_steer_test=spas, spas_angle=0., aprk_buses=())


def step(cc, frames, gear, kph, cross=0, lamp=False, tunnel=0, roundabout=False, left_line=10, right_line=10):
  cc.lane_crossing.side = cross or cc.lane_crossing.side
  CS = SimpleNamespace(out=SimpleNamespace(gearShifter=gear, vEgo=kph / 3.6, leftBlinker=lamp, rightBlinker=False, steeringTorque=0,
                                           naviTunnel=bool(tunnel), naviRoundabout=roundabout,
                                           leftLaneLine=left_line, rightLaneLine=right_line))
  sent = []
  for _ in range(frames):
    sent += CarController.aprk_blinkers(cc, None, CS)
    cc.frame += 1
  return sent


def spas2(cc, sent):
  sig = cc.packer.dbc.name_to_msg['SPAS2'].sigs['BLINKER_CONTROL']
  return [get_raw_value(m[1], sig) for m in sent if m[0] == 0x16A]


def test_aprk_paused_only_around_a_crossing_kick():
  cc = make_cc()
  assert step(cc, 50, D, 60) == [] and cc.aprk_state == 0  # on but nobody crossing: parking ECU untouched
  sent = step(cc, 10, D, 5, cross=3)  # any speed in D
  uds = [m[1] for m in sent if m[0] == carcontroller.APRK_ADDR]
  assert uds == [carcontroller.APRK_EXT_SESSION, carcontroller.APRK_COM_DISABLE] and cc.aprk_state == 2
  assert set(spas2(cc, step(cc, 10, D, 0))) == {3}
  sent = step(cc, carcontroller.BLINKER_PULSE_FRAMES, D, 60)
  assert [m[1] for m in sent if m[0] == carcontroller.APRK_ADDR][-1:] == [carcontroller.APRK_DEFAULT_SESSION]
  assert cc.aprk_state == 0 and step(cc, 100, D, 60) == []  # restored after the kick
  step(cc, 10, D, 60, cross=4)
  assert [m[1] for m in step(cc, 1, P, 60)] == [carcontroller.APRK_DEFAULT_SESSION]  # out of D: restored


def test_kick_is_short_and_follows_the_crossing_side():
  cc = make_cc()
  kick = spas2(cc, step(cc, carcontroller.BLINKER_PULSE_FRAMES, D, 60, cross=4))
  assert kick and set(kick) == {4}
  assert spas2(cc, step(cc, 300, D, 60)) == []
  assert set(spas2(cc, step(cc, 10, D, 60, cross=3))) == {3}


def test_no_kick_while_the_lamp_is_on_or_setting_off():
  cc = make_cc()
  assert step(cc, 100, D, 60, cross=3, lamp=True) == []  # the driver already signalled
  cc = make_cc(control=False)
  assert step(cc, 100, D, 60, cross=3) == []


def test_menu_test_blinks_at_standstill_in_p():
  cc = make_cc(control=False, test=2)
  sent = step(cc, 20, P, 0)
  assert cc.aprk_state == 2 and set(spas2(cc, sent)) == {4}
  cc.blinker_test = 0
  assert [m[1] for m in step(cc, 1, P, 0)] == [carcontroller.APRK_DEFAULT_SESSION] and cc.aprk_state == 0


def test_no_kick_in_a_tunnel():
  # CarState.naviTunnel: lane changes are banned there; the crossing stays pending until out of the tunnel
  cc = make_cc()
  assert 3 not in spas2(cc, step(cc, 50, D, 80, cross=3, tunnel=1))
  assert 3 in spas2(cc, step(cc, 50, D, 80, tunnel=0))


def test_no_kick_across_a_yellow_line():
  # 10-30 seg 1: kicked left over a yellow dashed centre line (20); the color drops to 0 while crossing
  cc = make_cc()
  step(cc, 50, D, 30, left_line=20)
  assert 3 not in spas2(cc, step(cc, 50, D, 30, cross=3, left_line=0))   # yellow 0.5 s ago: no kick, crossing consumed
  assert 4 in spas2(cc, step(cc, 50, D, 30, cross=4, left_line=20))      # the white right line still kicks
  step(cc, 300, D, 30, left_line=0)
  assert 3 in spas2(cc, step(cc, 50, D, 30, cross=3, left_line=0))       # yellow over 2 s ago


def test_no_kick_after_a_roundabout_sign():
  # CarState.naviRoundabout (0x162 SIGNS=24, 20 s, < 60 km/h): same ban as a tunnel
  cc = make_cc()
  assert 4 not in spas2(cc, step(cc, 50, D, 30, cross=4, roundabout=True))
  assert 4 in spas2(cc, step(cc, 50, D, 30))


def test_spas_steer_test_parked_only():
  cc = make_cc(control=False, spas=90)
  CS = lambda gear, held=False: SimpleNamespace(out=SimpleNamespace(gearShifter=gear, standstill=True, steeringPressed=held,
                                                                     steeringAngleDeg=-10., leftBlinker=False,
                                                                     rightBlinker=False, steeringTorque=0, leftLaneLine=0,
                                                                     rightLaneLine=0))
  sent = []
  for _ in range(120):
    sent += CarController.aprk_blinkers(cc, None, CS(P))
    cc.frame += 1
  # APRK asked on both buses
  assert [(m[1], m[2]) for m in sent if m[0] == carcontroller.APRK_ADDR][:4] == [
    (carcontroller.APRK_EXT_SESSION, 0), (carcontroller.APRK_EXT_SESSION, 1), (carcontroller.APRK_COM_DISABLE, 0),
    (carcontroller.APRK_COM_DISABLE, 1)]
  msg = cc.packer.dbc.name_to_msg['SPAS1']
  spas1 = [m[1] for m in sent if m[0] == 0x165]
  modes = [get_raw_value(d, msg.sigs['NEW_SIGNAL_2']) for d in spas1]
  assert modes[:carcontroller.SPAS_STEER_ASK_FRAMES] == [4] * carcontroller.SPAS_STEER_ASK_FRAMES and set(modes[10:]) == {5}
  angles = [int.from_bytes(d[12:14], 'little', signed=True) * 0.1 for d in spas1]
  assert abs(angles[0] - -9.) < 0.11 and abs(angles[-1] - 90.) < 0.11  # ramps from the wheel at 100 deg/s
  assert all(d[3] == 0x4b and d[11] in (0x10, 0x14) and not any(d[4:11]) and not any(d[14:]) for d in spas1)
  # hands on the wheel: APRK back on both buses, no more SPAS1
  sent = CarController.aprk_blinkers(cc, None, CS(P, held=True))
  assert [(m[1], m[2]) for m in sent] == [(carcontroller.APRK_DEFAULT_SESSION, 0), (carcontroller.APRK_DEFAULT_SESSION, 1)]
  cc.frame += 1
  assert CarController.aprk_blinkers(cc, None, CS(D)) == []
