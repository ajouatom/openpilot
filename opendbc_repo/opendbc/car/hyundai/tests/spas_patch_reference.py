"""Reference from owner-supplied f8448120 + 04dd4119, Joongyu, 2026-10-26.

The original method below is kept independently for wire-sequence comparison.
"""
from opendbc.car import CanData, make_tester_present_msg, structs
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.common.conversions import Conversions as CV

APRK_ADDR = 0x7B1
APRK_EXT_SESSION = bytes([0x02, 0x10, 0x03, 0, 0, 0, 0, 0])
APRK_COM_DISABLE = bytes([0x03, 0x28, 0x83, 0x01, 0, 0, 0, 0])
APRK_DEFAULT_SESSION = bytes([0x02, 0x10, 0x01, 0, 0, 0, 0, 0])
APRK_ON_KPH, APRK_OFF_KPH = 30., 20.


def aprk_blinkers(self, CC, CS):
  # shortcut: replies are not checked; if APRK ignores the requests it keeps sending SPAS2 and the cluster sees both
  ret = []
  drive = CS.out.gearShifter == structs.CarState.GearShifter.drive
  fast = CS.out.vEgo * CV.MS_TO_KPH > (APRK_OFF_KPH if self.aprk_state else APRK_ON_KPH)
  want = self.blinker_test in (1, 2) or (self.blinker_control and drive and fast)
  if not want:
    if self.aprk_state:
      ret.append(CanData(APRK_ADDR, APRK_DEFAULT_SESSION, self.CAN.ECAN))
    self.aprk_state = 0
  elif self.aprk_state == 0:
    ret.append(CanData(APRK_ADDR, APRK_EXT_SESSION, self.CAN.ECAN))
    self.aprk_state, self.aprk_frame = 1, self.frame
  elif self.aprk_state == 1:
    if self.frame - self.aprk_frame >= 5:  # 50 ms for the session change
      ret.append(CanData(APRK_ADDR, APRK_COM_DISABLE, self.CAN.ECAN))
      self.aprk_state = 2
  else:
    if self.frame % 100 == 0:
      ret.append(make_tester_present_msg(APRK_ADDR, self.CAN.ECAN, suppress_response=True))
    if self.frame % 2 == 0:  # APRK's own SPAS rate, 50 Hz
      left, right = CC.leftBlinker or self.blinker_test == 1, CC.rightBlinker or self.blinker_test == 2
      ret.extend(hyundaicanfd.create_spas_messages(self.packer, self.CAN, self.frame, left, right))
  return ret
