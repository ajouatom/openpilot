from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.can.parser import get_raw_value
from opendbc.car import structs
from opendbc.car.hyundai import carcontroller
from opendbc.car.hyundai.carcontroller import CarController

D, P = structs.CarState.GearShifter.drive, structs.CarState.GearShifter.park


def step(cc, frames, gear, kph, desire=0):
  md = SimpleNamespace(meta=SimpleNamespace(desire=SimpleNamespace(raw=desire), desireState=[]))
  CS = SimpleNamespace(out=SimpleNamespace(gearShifter=gear, vEgo=kph / 3.6), modelV2=md)
  CC = SimpleNamespace(leftBlinker=desire == 3, rightBlinker=desire == 4)
  sent = []
  for _ in range(frames):
    sent += CarController.aprk_blinkers(cc, CC, CS)
    cc.frame += 1
  return sent


def test_aprk_paused_only_around_a_kick():
  packer = CANPacker('hyundai_canfd_generated')
  cc = SimpleNamespace(blinker_control=True, blinker_test=0, aprk_state=0, aprk_frame=0, blinker_desire=0, blinker_pulse_end=0,
                       frame=0, CAN=SimpleNamespace(ECAN=0), packer=packer)
  assert step(cc, 50, D, 60) == [] and cc.aprk_state == 0  # on but no lane change: parking ECU untouched
  assert step(cc, 10, D, 25, desire=3) == []  # below 30 km/h: no kick
  step(cc, 10, D, 60)
  sent = step(cc, 10, D, 60, desire=3)
  uds = [m[1] for m in sent if m[0] == carcontroller.APRK_ADDR]
  assert uds == [carcontroller.APRK_EXT_SESSION, carcontroller.APRK_COM_DISABLE] and cc.aprk_state == 2
  spas2 = [m for m in step(cc, 10, D, 25, desire=3) if m[0] == 0x16A]  # 25 km/h: hysteresis keeps it paused
  sig = packer.dbc.name_to_msg['SPAS2'].sigs['BLINKER_CONTROL']
  assert len(spas2) == 10 and all(get_raw_value(m[1], sig) == 3 for m in spas2)
  sent = step(cc, carcontroller.BLINKER_PULSE_FRAMES, D, 60, desire=3)
  assert [m[1] for m in sent if m[0] == carcontroller.APRK_ADDR][-1:] == [carcontroller.APRK_DEFAULT_SESSION]
  assert cc.aprk_state == 0 and step(cc, 100, D, 60, desire=3) == []  # restored after the kick, same change stays quiet
  step(cc, 10, D, 60)
  step(cc, 10, D, 60, desire=4)
  assert [m[1] for m in step(cc, 1, P, 60, desire=4)] == [carcontroller.APRK_DEFAULT_SESSION]  # out of D: restored
  cc.blinker_control = False
  step(cc, 10, D, 60)
  assert step(cc, 10, D, 60, desire=3) == []


def test_menu_test_blinks_at_standstill_in_p():
  packer = CANPacker('hyundai_canfd_generated')
  cc = SimpleNamespace(blinker_control=False, blinker_test=2, aprk_state=0, aprk_frame=0, blinker_desire=0, blinker_pulse_end=0, frame=0, CAN=SimpleNamespace(ECAN=0), packer=packer)
  sig = packer.dbc.name_to_msg['SPAS2'].sigs['BLINKER_CONTROL']
  spas2 = [m for m in step(cc, 20, P, 0) if m[0] == 0x16A]
  assert cc.aprk_state == 2 and spas2 and all(get_raw_value(m[1], sig) == 4 for m in spas2)
  cc.blinker_test = 0
  assert [m[1] for m in step(cc, 1, P, 0)] == [carcontroller.APRK_DEFAULT_SESSION] and cc.aprk_state == 0


def test_lane_change_kicks_once_then_stops():
  # the whole-change hold read back as the driver's blinker and never let the lamp go off (EV4 10-26)
  packer = CANPacker('hyundai_canfd_generated')
  cc = SimpleNamespace(blinker_control=True, blinker_test=0, aprk_state=0, aprk_frame=0, blinker_desire=0, blinker_pulse_end=0,
                       frame=0, CAN=SimpleNamespace(ECAN=0), packer=packer)
  sig = packer.dbc.name_to_msg['SPAS2'].sigs['BLINKER_CONTROL']
  step(cc, 10, D, 60)
  values = lambda sent: [get_raw_value(m[1], sig) for m in sent if m[0] == 0x16A]
  kick = values(step(cc, carcontroller.BLINKER_PULSE_FRAMES, D, 60, desire=4))
  assert kick and set(kick) == {4}
  assert values(step(cc, 300, D, 60, desire=4)) == []  # same change: no second kick, parking ECU back
  step(cc, 10, D, 60)
  assert set(values(step(cc, 10, D, 60, desire=3))) == {3}  # next change kicks again (after the ~50 ms pause request)
