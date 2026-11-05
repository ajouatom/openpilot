#pragma once

// Raw Comma Pedal protocol helpers. Vehicle limits are supplied by its safety mode.
typedef struct {
  uint16_t min_track1;
  uint16_t max_track1;
  uint16_t min_track2;
  uint16_t max_track2;
  int track2_ratio;
  int track2_tolerance;
  uint16_t driver_threshold1;
  uint16_t driver_threshold2;
} PedalInterceptorLimits;

static uint16_t pedal_interceptor_track(const CANPacket_t *msg, int offset) {
  return ((uint16_t)GET_BYTE(msg, offset) << 8U) | GET_BYTE(msg, offset + 1);
}

static uint8_t pedal_interceptor_checksum(const CANPacket_t *msg) {
  uint8_t crc = 0xFFU;
  for (int i = 4; i >= 0; i--) {
    crc ^= GET_BYTE(msg, i);
    for (int j = 0; j < 8; j++) {
      crc = (crc & 0x80U) ? (uint8_t)((crc << 1U) ^ 0xD5U) : (uint8_t)(crc << 1U);
    }
  }
  return crc;
}

static bool pedal_interceptor_command_checks(const CANPacket_t *msg, const PedalInterceptorLimits limits) {
  const uint16_t track1 = pedal_interceptor_track(msg, 0);
  const uint16_t track2 = pedal_interceptor_track(msg, 2);
  const bool enabled = GET_BIT(msg, 39U);
  const int expected_track2 = (int)limits.min_track2 + limits.track2_ratio * ((int)track1 - (int)limits.min_track1);
  const int track2_error = (int)track2 - expected_track2;
  return ((GET_BYTE(msg, 4) & 0x70U) != 0U) ||
         (GET_BYTE(msg, 5) != pedal_interceptor_checksum(msg)) ||
         (enabled && ((track1 < limits.min_track1) || (track1 > limits.max_track1) ||
                      (track2 < limits.min_track2) || (track2 > limits.max_track2) ||
                      (track2_error > limits.track2_tolerance) || (track2_error < -limits.track2_tolerance))) ||
         (!enabled && ((track1 != 0U) || (track2 != 0U))) ||
         longitudinal_interceptor_checks(msg);
}
