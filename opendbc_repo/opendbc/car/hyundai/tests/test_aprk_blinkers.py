from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.can.parser import get_raw_value
from opendbc.car import structs
from opendbc.car.hyundai import carcontroller
from opendbc.car.hyundai.carcontroller import CarController

D, P = structs.CarState.GearShifter.drive, structs.CarState.GearShifter.park


def step(cc, frames, gear, kph, left=False):
  CS = SimpleNamespace(out=SimpleNamespace(gearShifter=gear, vEgo=kph / 3.6))
  CC = SimpleNamespace(leftBlinker=left, rightBlinker=False)
  sent = []
  for _ in range(frames):
    sent += CarController.aprk_blinkers(cc, CC, CS)
    cc.frame += 1
  return sent


def test_aprk_silenced_only_in_d_above_30_and_restored():
  packer = CANPacker('hyundai_canfd_generated')
  cc = SimpleNamespace(blinker_control=True, aprk_state=0, aprk_frame=0, frame=0, CAN=SimpleNamespace(ECAN=0), packer=packer)
  assert step(cc, 50, D, 25) == [] and cc.aprk_state == 0  # below 30 km/h: parking ECU untouched
  sent = step(cc, 10, D, 35)
  uds = [m[1] for m in sent if m[0] == carcontroller.APRK_ADDR]
  assert uds == [carcontroller.APRK_EXT_SESSION, carcontroller.APRK_COM_DISABLE] and cc.aprk_state == 2
  spas2 = [m for m in step(cc, 10, D, 25, left=True) if m[0] == 0x16A]  # 25 km/h: hysteresis keeps it silenced
  sig = packer.dbc.name_to_msg['SPAS2'].sigs['BLINKER_CONTROL']
  assert len(spas2) == 5 and all(get_raw_value(m[1], sig) == 3 for m in spas2)
  assert [m[1] for m in step(cc, 1, D, 15)] == [carcontroller.APRK_DEFAULT_SESSION] and cc.aprk_state == 0  # below 20: restored
  step(cc, 10, D, 35)
  assert [m[1] for m in step(cc, 1, P, 35)] == [carcontroller.APRK_DEFAULT_SESSION]  # out of D: restored
  cc.blinker_control = False
  assert step(cc, 10, D, 50) == []
