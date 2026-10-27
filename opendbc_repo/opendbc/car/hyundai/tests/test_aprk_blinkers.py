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

  def update(self, md, t, allowed):
    side, self.side = (self.side, 0) if allowed else (0, self.side)
    return side


def make_cc(control=True, test=0):
  return SimpleNamespace(blinker_control=control, blinker_test=test, aprk_state=0, aprk_frame=0, lane_crossing=Crossing(),
                         blinker_side=0, blinker_pulse_end=0, frame=0, CAN=SimpleNamespace(ECAN=0),
                         packer=CANPacker('hyundai_canfd_generated'))


def step(cc, frames, gear, kph, cross=0, lamp=False):
  cc.lane_crossing.side = cross or cc.lane_crossing.side
  CS = SimpleNamespace(out=SimpleNamespace(gearShifter=gear, vEgo=kph / 3.6, leftBlinker=lamp, rightBlinker=False))
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
