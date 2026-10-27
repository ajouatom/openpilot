#pragma once

static bool tesla_auto_turn_signals = false;
static bool tesla_turn_template_valid = false;
static uint8_t tesla_turn_template[8] = {0U};
static uint32_t tesla_turn_template_ts = 0U;
static uint8_t tesla_turn_direction = 0U;
static uint16_t tesla_turn_count = 0U;
static uint32_t tesla_turn_started = 0U;
static uint8_t tesla_turn_cancel_count = 0U;
static uint32_t tesla_turn_cancel_started = 0U;

static uint8_t tesla_turn_checksum(const CANPacket_t *msg) {
  uint8_t checksum = 0xECU;
  for (int i = 0; i < 7; i++) {
    checksum += GET_BYTE(msg, i);
  }
  return checksum;
}

static void tesla_turn_rx(const CANPacket_t *msg) {
  if ((GET_ADDR(msg) == 0x3E9) && (GET_BUS(msg) == 1) && (GET_LEN(msg) == 8) &&
      !msg->extended && !msg->returned && !msg->rejected &&
      ((GET_BYTE(msg, 1) & 3U) == 0U) && (tesla_turn_checksum(msg) == GET_BYTE(msg, 7))) {
    for (int i = 0; i < 8; i++) {
      tesla_turn_template[i] = GET_BYTE(msg, i);
    }
    tesla_turn_template_valid = true;
    tesla_turn_template_ts = microsecond_timer_get();
  }
}

static bool tesla_turn_tx(const CANPacket_t *msg) {
  const uint32_t now = microsecond_timer_get();
  const uint8_t request = GET_BYTE(msg, 1) & 3U;
  const uint8_t reason = (GET_BYTE(msg, 2) >> 1) & 15U;
  const bool active = (request == 1U) || (request == 2U);
  bool valid = tesla_auto_turn_signals && !msg->extended && (GET_BUS(msg) == 1) && (GET_LEN(msg) == 8) &&
               tesla_turn_template_valid && (get_ts_elapsed(now, tesla_turn_template_ts) <= 1500000U) &&
               (tesla_turn_checksum(msg) == GET_BYTE(msg, 7)) &&
               ((active && (reason == 8U)) || ((request == 3U) && (reason == 4U)));
  for (int i = 0; i < 7; i++) {
    const uint8_t mask = (i == 1) ? 0xFCU : (i == 2) ? 0xE1U : (i == 6) ? 0x0FU : 0xFFU;
    valid &= (GET_BYTE(msg, i) & mask) == (tesla_turn_template[i] & mask);
  }
  valid &= (GET_BYTE(msg, 6) >> 4) == (((tesla_turn_template[6] >> 4) + 1U) & 15U);
  if (active) {
    valid &= (tesla_turn_direction == 0U) ||
             ((tesla_turn_direction == request) && (tesla_turn_count < 1200U) &&
              (get_ts_elapsed(now, tesla_turn_started) <= 60000000U));
  } else {
    valid &= (tesla_turn_direction != 0U) ||
             ((tesla_turn_cancel_count > 0U) && (tesla_turn_cancel_count < 3U) &&
              (get_ts_elapsed(now, tesla_turn_cancel_started) <= 5000000U));
  }
  if (valid) {
    tesla_turn_template_valid = false;
    if (active) {
      if (tesla_turn_direction == 0U) {
        tesla_turn_started = now;
      }
      tesla_turn_direction = request;
      tesla_turn_count++;
      tesla_turn_cancel_count = 0U;
    } else {
      if (tesla_turn_cancel_count == 0U) {
        tesla_turn_cancel_started = now;
      }
      tesla_turn_cancel_count++;
      tesla_turn_direction = 0U;
      tesla_turn_count = 0U;
    }
  }
  return valid;
}
