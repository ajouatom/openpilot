from openpilot.common.numpy_fast import clip
from openpilot.selfdrive.car import CanBusBase
from openpilot.selfdrive.car.hyundai.values import HyundaiFlags


class CanBus(CanBusBase):
  def __init__(self, CP, hda2=None, fingerprint=None) -> None:
    super().__init__(CP, fingerprint)

    if hda2 is None:
      assert CP is not None
      hda2 = CP.flags & HyundaiFlags.CANFD_HDA2.value

    # On the CAN-FD platforms, the LKAS camera is on both A-CAN and E-CAN. HDA2 cars
    # have a different harness than the HDA1 and non-HDA variants in order to split
    # a different bus, since the steering is done by different ECUs.
    self._a, self._e = 1, 0
    if hda2:
      self._a, self._e = 0, 1

    self._a += self.offset
    self._e += self.offset
    self._cam = 2 + self.offset

  @property
  def ECAN(self):
    return self._e

  @property
  def ACAN(self):
    return self._a

  @property
  def CAM(self):
    return self._cam


def create_steering_messages(packer, CP, CAN, enabled, lat_active, apply_steer):

  ret = []

  values = {
    "LKA_MODE": 2,
    "LKA_ICON": 2 if enabled else 1,
    "TORQUE_REQUEST": apply_steer,
    "LKA_ASSIST": 0,
    "STEER_REQ": 1 if lat_active else 0,
    "STEER_MODE": 0,
    "HAS_LANE_SAFETY": 0,  # hide LKAS settings
    "NEW_SIGNAL_1": 0,
    "NEW_SIGNAL_2": 0,
  }

  if CP.flags & HyundaiFlags.CANFD_HDA2:
    hda2_lkas_msg = "LKAS_ALT" if CP.flags & HyundaiFlags.CANFD_HDA2_ALT_STEERING else "LKAS"
    if CP.openpilotLongitudinalControl:
      ret.append(packer.make_can_msg("LFA", CAN.ECAN, values))
    ret.append(packer.make_can_msg(hda2_lkas_msg, CAN.ACAN, values))
  else:
    ret.append(packer.make_can_msg("LFA", CAN.ECAN, values))

  return ret

def create_suppress_lfa(packer, CAN, hda2_lfa_block_msg, hda2_alt_steering):
  suppress_msg = "CAM_0x362" if hda2_alt_steering else "CAM_0x2a4"
  msg_bytes = 32 if hda2_alt_steering else 24

  values = {f"BYTE{i}": hda2_lfa_block_msg[f"BYTE{i}"] for i in range(3, msg_bytes) if i != 7}
  values["COUNTER"] = hda2_lfa_block_msg["COUNTER"]
  values["SET_ME_0"] = 0
  values["SET_ME_0_2"] = 0
  values["LEFT_LANE_LINE"] = 0
  values["RIGHT_LANE_LINE"] = 0
  return packer.make_can_msg(suppress_msg, CAN.ACAN, values)

def create_buttons(packer, CP, CAN, cnt, btn):
  values = {
    "COUNTER": cnt,
    "SET_ME_1": 1,
    "CRUISE_BUTTONS": btn,
  }

  bus = CAN.ECAN if CP.flags & HyundaiFlags.CANFD_HDA2 else CAN.CAM
  return packer.make_can_msg("CRUISE_BUTTONS", bus, values)

def create_acc_cancel(packer, CP, CAN, cruise_info_copy):
  # TODO: why do we copy different values here?
  if CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value:
    values = {s: cruise_info_copy[s] for s in [
      "COUNTER",
      "CHECKSUM",
      "NEW_SIGNAL_1",
      "MainMode_ACC",
      "ACCMode",
      "ZEROS_9",
      "CRUISE_STANDSTILL",
      "ZEROS_5",
      "DISTANCE_SETTING",
      "VSetDis",
    ]}
  else:
    values = {s: cruise_info_copy[s] for s in [
      "COUNTER",
      "CHECKSUM",
      "ACCMode",
      "VSetDis",
      "CRUISE_STANDSTILL",
    ]}
  values.update({
    "ACCMode": 4,
    "aReqRaw": 0.0,
    "aReqValue": 0.0,
  })
  return packer.make_can_msg("SCC_CONTROL", CAN.ECAN, values)

def create_lfahda_cluster(packer, CAN, enabled):
  values = {
    "HDA_ICON": 1 if enabled else 0,
    "LFA_ICON": 2 if enabled else 0,
  }
  return packer.make_can_msg("LFAHDA_CLUSTER", CAN.ECAN, values)


def create_acc_control(packer, CAN, enabled, accel_last, accel, stopping, gas_override, set_speed, personality):
  jerk = 5
  jn = jerk / 50
  if not enabled or gas_override:
    a_val, a_raw = 0, 0
  else:
    a_raw = accel
    a_val = clip(accel, accel_last - jn, accel_last + jn)

  values = {
    "ACCMode": 0 if not enabled else (2 if gas_override else 1),
    "MainMode_ACC": 1,
    "StopReq": 1 if stopping else 0,
    "aReqValue": a_val,
    "aReqRaw": a_raw,
    "VSetDis": set_speed,
    "JerkLowerLimit": jerk if enabled else 1,
    "JerkUpperLimit": 3.0,

    "ACC_ObjDist": 1,
    "ObjValid": 0,
    "OBJ_STATUS": 2,
    "SET_ME_2": 0x4,
    "SET_ME_3": 0x3,
    "SET_ME_TMP_64": 0x64,
    "DISTANCE_SETTING": personality + 1,
  }

  return packer.make_can_msg("SCC_CONTROL", CAN.ECAN, values)


def create_spas_messages(packer, CAN, frame, left_blink, right_blink):
  ret = []

  values = {
  }
  ret.append(packer.make_can_msg("SPAS1", CAN.ECAN, values))

  blink = 0
  if left_blink:
    blink = 3
  elif right_blink:
    blink = 4
  values = {
    "BLINKER_CONTROL": blink,
  }
  ret.append(packer.make_can_msg("SPAS2", CAN.ECAN, values))

  return ret


def create_adrv_messages(packer, CAN, frame):
  # messages needed to car happy after disabling
  # the ADAS Driving ECU to do longitudinal control

  ret = []

  values = {
  }
  ret.append(packer.make_can_msg("ADRV_0x51", CAN.ACAN, values))

  if frame % 2 == 0:
    values = {
      'AEB_SETTING': 0x1,  # show AEB disabled icon
      'SET_ME_2': 0x2,
      'SET_ME_FF': 0xff,
      'SET_ME_FC': 0xfc,
      'SET_ME_9': 0x9,
    }
    ret.append(packer.make_can_msg("ADRV_0x160", CAN.ECAN, values))

  if frame % 5 == 0:
    values = {
      'SET_ME_1C': 0x1c,
      'SET_ME_FF': 0xff,
      'SET_ME_TMP_F': 0xf,
      'SET_ME_TMP_F_2': 0xf,
    }
    ret.append(packer.make_can_msg("ADRV_0x1ea", CAN.ECAN, values))

    values = {
      'SET_ME_E1': 0xe1,
      'SET_ME_3A': 0x3a,
    }
    ret.append(packer.make_can_msg("ADRV_0x200", CAN.ECAN, values))

  if frame % 20 == 0:
    values = {
      'SET_ME_15': 0x15,
    }
    ret.append(packer.make_can_msg("ADRV_0x345", CAN.ECAN, values))

  if frame % 100 == 0:
    values = {
      'SET_ME_22': 0x22,
      'SET_ME_41': 0x41,
    }
    ret.append(packer.make_can_msg("ADRV_0x1da", CAN.ECAN, values))

  return ret




import random

# 각 채널별 데이터를 파싱하여 구성
channel_data_db = {
    1: [
        b'\x84\xbe\x40\x00\x10\x60\x34\x00\x1a\x00\x05\x05\x00\x00\x00\x00',
        b'\xa0\x16\x41\x00\x10\x60\x34\x00\x1a\x00\x05\x05\x00\x00\x00\x00',
        b'\xed\xfe\x42\x00\x10\x60\x34\x00\x1a\x00\x05\x05\x00\x00\x00\x00',
        b'\x21\xd3\x43\x00\x13\x60\x33\x00\x1a\x00\x05\x05\x00\x00\x00\x00',
        b'\xbe\xbb\x44\x00\x13\x60\x33\x00\x1a\x00\x05\x05\x00\x00\x00\x00',
        b'\xf1\x4b\x55\x00\x12\x60\x33\x00\x1a\x00\x05\x05\x00\x00\x00\x00',
        b'\xdd\xd8\x56\x00\x13\x60\x33\x00\x1a\x00\x05\x05\x00\x00\x00\x00',
        b'\xf9\x70\x57\x00\x13\x60\x33\x00\x1a\x00\x05\x05\x00\x00\x00\x00',
        b'\x82\x72\x58\x00\x12\x60\x33\x00\x1a\x00\x05\x05\x00\x00\x00\x00',
        b'\xa6\xda\x59\x00\x12\x60\x33\x00\x1a\x00\x05\x05\x00\x00\x00\x00',
        b'\xd9\xc4\x69\x00\x13\x60\x33\x00\x1a\x00\x05\x05\x00\x00\x00\x00',
        b'\x7c\xa9\x6a\x00\x10\x60\x34\x00\x1a\x00\x05\x05\x00\x00\x00\x00',
        b'\xb0\x84\x6b\x00\x13\x60\x33\x00\x1a\x00\x05\x05\x00\x00\x00\x00',
        b'\xc7\x69\x6c\x00\x10\x60\x34\x00\x1a\x00\x05\x05\x00\x00\x00\x00',
        b'\xe3\xc1\x6d\x00\x10\x60\x34\x00\x1a\x00\x05\x05\x00\x00\x00\x00',
        b'\xae\x29\x6e\x00\x10\x60\x34\x00\x1a\x00\x05\x05\x00\x00\x00\x00',
    ],
    2: [
        b'\xdf\x02\x8b\x00\x22\x60\x35\x00\x1b\x00\x05\x05\x00\x00\x00\x00',
        b'\x40\x6a\x8c\x00\x22\x60\x35\x00\x1b\x00\x05\x05\x00\x00\x00\x00',
        b'\x05\xb9\x8d\x00\x23\x60\x35\x00\x1b\x00\x05\x05\x00\x00\x00\x00',
        b'\x48\x51\x8e\x00\x23\x60\x35\x00\x1b\x00\x05\x05\x00\x00\x00\x00',
        b'\x6c\xf9\x8f\x00\x23\x60\x35\x00\x1b\x00\x05\x05\x00\x00\x00\x00',
        b'\x40\x2b\x90\x00\x20\x60\x36\x00\x1b\x00\x05\x05\x00\x00\x00\x00',
        b'\xd5\x4d\x9f\x00\x20\x60\x37\x00\x1c\x00\x05\x05\x00\x00\x00\x00',
        b'\xd1\x51\xa0\x00\x20\x60\x37\x00\x1c\x00\x05\x05\x00\x00\x00\x00',
        b'\xf5\xf9\xa1\x00\x20\x60\x37\x00\x1c\x00\x05\x05\x00\x00\x00\x00',
        b'\xb8\x11\xa2\x00\x20\x60\x37\x00\x1c\x00\x05\x05\x00\x00\x00\x00',
        b'\xfd\xc2\xa3\x00\x21\x60\x37\x00\x1c\x00\x05\x05\x00\x00\x00\x00',
        b'\x14\xf2\xb3\x00\x21\x60\x38\x00\x1c\x00\x05\x05\x00\x00\x00\x00',
        b'\x8b\x9a\xb4\x00\x21\x60\x38\x00\x1c\x00\x05\x05\x00\x00\x00\x00',
        b'\x0c\xbf\xb5\x00\x22\x60\x38\x00\x1c\x00\x05\x05\x00\x00\x00\x00',
        b'\x20\x2c\xb6\x00\x23\x60\x38\x00\x1c\x00\x05\x05\x00\x00\x00\x00',
        b'\x04\x84\xb7\x00\x23\x60\x38\x00\x1c\x00\x05\x05\x00\x00\x00\x00',
    ],
    3: [
        b'\xf4\x12\x20\x00\x33\x60\x37\x00\x1c\x00\x05\x05\x00\x00\x00\x00',
        b'\xd0\xba\x21\x00\x33\x60\x37\x00\x1c\x00\x05\x05\x00\x00\x00\x00',
        b'\x9d\x52\x22\x00\x33\x60\x37\x00\x1c\x00\x05\x05\x00\x00\x00\x00',
        b'\xb9\xfa\x23\x00\x33\x60\x37\x00\x1c\x00\x05\x05\x00\x00\x00\x00',
        b'\x26\x92\x24\x00\x33\x60\x37\x00\x1c\x00\x05\x05\x00\x00\x00\x00',
        b'\x02\x3a\x25\x00\x33\x60\x37\x00\x1c\x00\x05\x05\x00\x00\x00\x00',
        b'\x4f\xd2\x26\x00\x33\x60\x37\x00\x1c\x00\x05\x05\x00\x00\x00\x00',
        b'\xe6\x7c\x36\x00\x30\x60\x37\x00\x1c\x00\x05\x05\x00\x00\x00\x00',
        b'\xee\x46\x37\x00\x33\x60\x36\x00\x1b\x00\x05\x05\x00\x00\x00\x00',
        b'\xd8\xad\x38\x00\x30\x60\x37\x00\x1c\x00\x05\x05\x00\x00\x00\x00',
        b'\xfc\x05\x39\x00\x30\x60\x37\x00\x1c\x00\x05\x05\x00\x00\x00\x00',
        b'\x9d\x7f\x3a\x00\x33\x60\x36\x00\x1b\x00\x05\x05\x00\x00\x00\x00',
        b'\xb9\xd7\x3b\x00\x33\x60\x36\x00\x1b\x00\x05\x05\x00\x00\x00\x00',
        b'\x26\xbf\x3c\x00\x33\x60\x36\x00\x1b\x00\x05\x05\x00\x00\x00\x00',
        b'\x97\x73\x4d\x00\x30\x60\x36\x00\x1b\x00\x05\x05\x00\x00\x00\x00',
        b'\xda\x9b\x4e\x00\x30\x60\x36\x00\x1b\x00\x05\x05\x00\x00\x00\x00',
    ],
    4: [
        b'\x5c\x3c\xd6\x00\x43\x60\x39\x00\x1d\x00\x05\x05\x00\x00\x00\x00',
        b'\x78\x94\xd7\x00\x43\x60\x39\x00\x1d\x00\x05\x05\x00\x00\x00\x00',
        b'\x5e\x65\xd8\x00\x40\x60\x3a\x00\x1d\x00\x05\x05\x00\x00\x00\x00',
        b'\x7a\xcd\xd9\x00\x40\x60\x3a\x00\x1d\x00\x05\x05\x00\x00\x00\x00',
        b'\x37\x25\xda\x00\x40\x60\x3a\x00\x1d\x00\x05\x05\x00\x00\x00\x00',
        b'\x2f\x05\xdb\x00\x43\x60\x39\x00\x1d\x00\x05\x05\x00\x00\x00\x00',
        b'\x15\xc8\xea\x00\x43\x60\x39\x00\x1d\x00\x05\x05\x00\x00\x00\x00',
        b'\x31\x60\xeb\x00\x43\x60\x39\x00\x1d\x00\x05\x05\x00\x00\x00\x00',
        b'\xae\x08\xec\x00\x43\x60\x39\x00\x1d\x00\x05\x05\x00\x00\x00\x00',
        b'\x8a\xa0\xed\x00\x43\x60\x39\x00\x1d\x00\x05\x05\x00\x00\x00\x00',
        b'\xa6\x33\xee\x00\x42\x60\x39\x00\x1d\x00\x05\x05\x00\x00\x00\x00',
        b'\x0f\x9d\xfe\x00\x41\x60\x39\x00\x1d\x00\x05\x05\x00\x00\x00\x00',
        b'\x4a\x4e\xff\x00\x40\x60\x39\x00\x1d\x00\x05\x05\x00\x00\x00\x00',
        b'\x17\xd6\x00\x00\x40\x60\x39\x00\x1d\x00\x05\x05\x00\x00\x00\x00',
        b'\x33\x7e\x01\x00\x40\x60\x39\x00\x1d\x00\x05\x05\x00\x00\x00\x00',
        b'\x7e\x96\x02\x00\x40\x60\x39\x00\x1d\x00\x05\x05\x00\x00\x00\x00',
    ]
}

# 채널 번호가 주어지면 해당 채널의 데이터 중 하나를 랜덤으로 선택하여 반환하는 함수
def get_random_data(channel):
    if channel in channel_data_db:
        return random.choice(channel_data_db[channel])
    else:
        return None

def alt_cruise_buttons(packer, CP, CAN, buttons, cruise_btns_msg):
  if cruise_btns_msg is not None:
  #  print("send alt_cruise_buttons")
    return alt_cruise_buttons2(packer, CP, CAN, buttons, cruise_btns_msg)
  ## CRUISE_BUTTONS_ALT
  return [426, 0, get_random_data(int(buttons)), CAN.ECAN]

def alt_cruise_buttons2(packer, CP, CAN, buttons, cruise_btns_msg):
  #if cruise_btns_msg is None:
  #  return None
  
  print("alt_cruise1=", values)
  values = {key: value[0] for key, value in cruise_btns_msg.items()}
  values["CRUISE_BUTTONS"] = buttons
  print("alt_cruise2=", values)
  return packer.make_can_msg("CRUISE_BUTTONS_ALT", CAN.ECAN, values)
