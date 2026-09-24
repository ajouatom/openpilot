#pragma once

// Host button intent only. Never transmit the host's sparse 0x10B payload.
// Overlay fresh intent on the original vehicle frame in the forwarding hook.
static uint8_t hyundai_alt2_request;
static bool hyundai_alt2_blocked;
static bool hyundai_alt2_driver_pressed;
static uint32_t hyundai_alt2_request_us;
static uint32_t hyundai_alt2_press_us;

static void hyundai_alt2_reset(void) {
  hyundai_alt2_request = 0U;
  hyundai_alt2_blocked = false;
  hyundai_alt2_driver_pressed = false;
  hyundai_alt2_request_us = 0U;
  hyundai_alt2_press_us = 0U;
}

static bool hyundai_alt2_checksum_valid(const CANPacket_t *pkt) {
  uint32_t received = GET_BYTE(pkt, 0) | (GET_BYTE(pkt, 1) << 8U);
  return received == hyundai_common_canfd_compute_checksum(pkt);
}

static bool hyundai_alt2_set_request(const CANPacket_t *pkt, uint32_t now) {
  if (!hyundai_camera_scc || !hyundai_longitudinal || (GET_BUS(pkt) != 2) ||
      (GET_LEN(pkt) != 16U) || !hyundai_alt2_checksum_valid(pkt)) {
    return false;
  }
  uint8_t buttons = GET_BYTE(pkt, 10);
  // Only the existing automatic LFA, SET and MAIN requests are supported.
  if ((buttons != 0U) && (buttons != 0x80U) && (buttons != 2U) && (buttons != 8U)) {
    return false;
  }
  if (buttons == 0U) {
    hyundai_alt2_request = 0U;
    hyundai_alt2_blocked = hyundai_alt2_driver_pressed;
  } else if (!hyundai_alt2_blocked) {
    if (hyundai_alt2_request == 0U) {
      hyundai_alt2_press_us = now;
    } else if ((buttons != hyundai_alt2_request) ||
               ((now - hyundai_alt2_request_us) >= 120000U) ||
               ((now - hyundai_alt2_press_us) >= 200000U)) {
      // A release is required between different/stale/overlong requests.
      hyundai_alt2_blocked = true;
    }
    hyundai_alt2_request = buttons;
    hyundai_alt2_request_us = now;
  }
  return true;
}

static void hyundai_alt2_overlay(CANPacket_t *pkt, uint32_t now) {
  if (!hyundai_camera_scc || !hyundai_longitudinal || (GET_BUS(pkt) != 0) || (GET_LEN(pkt) != 16U)) {
    return;
  }
  // Physical input always wins, including CANCEL and unrecognized buttons.
  hyundai_alt2_driver_pressed = (GET_BYTE(pkt, 10) & 0x8FU) != 0U;
  if (hyundai_alt2_driver_pressed || !hyundai_alt2_checksum_valid(pkt)) {
    hyundai_alt2_blocked = true;
    return;
  }
  if ((hyundai_alt2_request == 0U) || hyundai_alt2_blocked) {
    return;
  }
  if (((now - hyundai_alt2_request_us) >= 120000U) || ((now - hyundai_alt2_press_us) >= 200000U)) {
    hyundai_alt2_blocked = true;
    return;
  }
  pkt->data[10] = (pkt->data[10] & 0x70U) | hyundai_alt2_request;
  hyundai_canfd_update_checksum(pkt);
}
