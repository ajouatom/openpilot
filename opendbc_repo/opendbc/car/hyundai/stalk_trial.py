"""Bounded parked reproduction of joongyu01's counter-aware lever replay."""
from opendbc.car import CanData


def crc(body):
  value = 0
  for byte in body:
    value ^= byte
    for _ in range(8):
      value = ((value << 1) ^ (0x1d if value & 0x80 else 0)) & 255
  return value ^ 0x65


def replay(original, direction, bus=0):
  if (len(original) != 8 or original[0] != crc(original[1:]) or direction not in ('left', 'right') or
      original[3] & 0x40 or original[4] & 0x51):
    raise ValueError('fresh neutral lever required')
  data = bytearray(original)
  data[1] = (((data[1] >> 4) + 1) % 16 << 4) | (data[1] & 15)
  data[3] = (data[3] & ~0x40) | (0x40 if direction == 'left' else 0)
  data[4] = (data[4] & ~0x51) | (1 if direction == 'right' else 0)
  data[0] = crc(data[1:])
  return CanData(0x3C1, bytes(data), bus)
