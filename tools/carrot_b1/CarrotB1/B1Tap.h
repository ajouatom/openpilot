#pragma once
#include <stdint.h>
#include <string.h>

namespace b1 {
inline uint8_t crc(const uint8_t *data) {
  uint8_t value = 0;
  for (int i = 1; i < 8; i++) {
    value ^= data[i];
    for (int bit = 0; bit < 8; bit++)
      value = (value << 1) ^ ((value & 0x80) ? 0x1d : 0);
  }
  return value ^ 0x65;
}

enum class Direction { Left, Right };
struct TapFrame { uint32_t offsetUs; uint8_t data[8]; };

// Return zero for an invalid seed. Caller supplies space for seven frames.
// Pure encoder: no TWAI transmit or scheduling side effects.
inline int encode(const uint8_t (&original)[8], Direction dir, unsigned counter,
                  TapFrame (&frames)[7]) {
  if (counter >= 15 || crc(original) != original[0] ||
      (original[3] & 0x40) || (original[4] & 0x51)) return 0;
  constexpr uint32_t lt[] = {0, 37063, 86550, 217974, 257611, 297840};
  constexpr uint32_t rt[] = {0, 36767, 76723, 113991, 137291, 167372, 210180};
  bool left = dir == Direction::Left;
  int count = left ? 6 : 7;
  for (int i = 0; i < count; i++) {
    bool pressed = i < (left ? 3 : 4);
    frames[i].offsetUs = left ? lt[i] : rt[i];
    auto *data = frames[i].data;
    memcpy(data, original, 8);
    data[1] = (((counter + (pressed ? 1 : 2)) % 15) << 4) | (original[1] & 15);
    if (pressed) {
      if (left) data[3] |= 0x40;
      data[4] |= left ? 0x10 : 0x41;
    }
    data[0] = crc(data);
  }
  return count;
}
}
