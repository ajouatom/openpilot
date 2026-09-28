"""Receive-profile wheel touch, independent of vehicle name and ADAS forwarding."""
import math

from opendbc.car.crc import CRC8J1850, mk_crc8_fun

TOUCH_ADDR = 0x2AF
TOUCH_MSG = 'STEER_TOUCH_2AF'
TOUCH_TIMEOUT_NS = 250_000_000
_crc = mk_crc8_fun(CRC8J1850)


def touch_checksum(data: bytes) -> int:
  # Empirical receive profile: polynomial 0x1D, zero initial register, residual
  # 0x32 over bytes 1..7. Verified on independent Ioniq 5 PE logs; this is not
  # an OEM protocol specification or the existing transmit checksum function.
  return _crc(data) ^ 0x32


class HyundaiSteeringTouch:
  def __init__(self):
    self.last_timestamp = 0
    self.last_counter = None
    self.frame_valid = False

  def update(self, cp) -> dict:
    # Scope the address to the named DBC message on original ECAN. A matching
    # numeric ID in another vehicle protocol is not touch evidence.
    message = cp.dbc.name_to_msg.get(TOUCH_MSG)
    if message is None or message.address != TOUCH_ADDR or message.size != 8:
      return {}
    # Observe even after the startup fingerprint window. Register only once a
    # real frame was seen; absence/dropout must not create a CAN-validity fault.
    if TOUCH_ADDR not in cp.addresses and TOUCH_ADDR in cp.seen_addresses:
      cp._add_message(TOUCH_MSG, math.nan)
    timestamp = cp.ts_nanos.get(TOUCH_MSG, {}).get('TOUCH_DETECT', 0)
    data = cp.dat.get(TOUCH_ADDR, b'')
    now = cp._last_update_nanos
    fresh = timestamp > 0 and 0 <= now - timestamp <= TOUCH_TIMEOUT_NS and not cp.bus_timeout
    if timestamp != self.last_timestamp:
      previous_timestamp = self.last_timestamp
      previous_counter = self.last_counter
      self.last_timestamp = timestamp
      layout_ok = (len(data) == 8 and data[1] & 0x0f == 0 and data[1] >> 4 < 15 and
                   data[2] <= 4 and data[3] == 1 and data[6:] == b'\x01\x00')
      integrity = layout_ok and data[0] == touch_checksum(data[1:])
      counter = data[1] >> 4 if integrity else None
      self.frame_valid = (fresh and integrity and previous_counter is not None and
                          0 < timestamp - previous_timestamp <= TOUCH_TIMEOUT_NS and
                          counter == (previous_counter + 1) % 15)
      self.last_counter = counter if fresh else None
    if not fresh:
      self.frame_valid = False
      self.last_counter = None
    valid = fresh and self.frame_valid
    return dict(available=timestamp > 0, valid=valid, touched=bool(valid and data[2] >= 1),
                sampleMonoTime=timestamp, rawStatus=data[2] if len(data) == 8 else 0,
                rawTouch1=data[4] if len(data) == 8 else 0, rawTouch2=data[5] if len(data) == 8 else 0)
