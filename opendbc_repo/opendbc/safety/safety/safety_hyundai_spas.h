#pragma once

// Opt-in SPAS lease; admits no steering, parking or acceleration payload fields.
static bool hyundai_spas_enabled;
static int hyundai_spas_bus;
static uint8_t hyundai_spas_phase;
static uint32_t hyundai_spas_phase_at, hyundai_spas_started, hyundai_spas_alive;
static uint32_t hyundai_spas_rx_at[2], hyundai_spas_tx_at[2];
static uint8_t hyundai_spas_rx_counter[2], hyundai_spas_rx_count[2], hyundai_spas_tx_counter[2];
static bool hyundai_spas_tx_seen[2];
static CanMsg hyundai_spas_tx_list[128];

static void hyundai_spas_init(bool enabled, int bus) {
  hyundai_spas_enabled = enabled;
  hyundai_spas_bus = bus;
  hyundai_spas_phase = 0U;
  hyundai_spas_phase_at = hyundai_spas_started = hyundai_spas_alive = 0U;
  for (int i = 0; i < 2; i++) {
    hyundai_spas_rx_at[i] = hyundai_spas_tx_at[i] = 0U;
    hyundai_spas_rx_counter[i] = hyundai_spas_rx_count[i] = hyundai_spas_tx_counter[i] = 0U;
    hyundai_spas_tx_seen[i] = false;
  }
}

static bool hyundai_spas_crc(const CANPacket_t *pkt) {
  return GET_BYTES(pkt, 0, 2) == hyundai_common_canfd_compute_checksum(pkt);
}

static bool hyundai_spas_stock_ready(uint32_t now) {
  bool ready = true;
  for (int i = 0; i < 2; i++) {
    ready = ready && (hyundai_spas_rx_count[i] >= 10U) && (get_ts_elapsed(now, hyundai_spas_rx_at[i]) < 100000U);
  }
  return ready;
}

static void hyundai_spas_rx(const CANPacket_t *pkt, uint32_t now) {
  if (hyundai_spas_enabled && (GET_BUS(pkt) == hyundai_spas_bus)) {
    int addr = GET_ADDR(pkt);
    if ((addr == 0x165) || (addr == 0x16A)) {
      int idx = (addr == 0x165) ? 0 : 1;
      if ((GET_LEN(pkt) == ((idx == 0) ? 24 : 32)) && hyundai_spas_crc(pkt)) {
        uint8_t count = GET_BYTE(pkt, 2);
        bool advancing = (get_ts_elapsed(now, hyundai_spas_rx_at[idx]) < 100000U) &&
                         (count == ((hyundai_spas_rx_counter[idx] + 1U) & 0xFFU));
        hyundai_spas_rx_count[idx] = advancing ? MIN(hyundai_spas_rx_count[idx] + 1U, 20U) : 1U;
        hyundai_spas_rx_at[idx] = now;
        hyundai_spas_rx_counter[idx] = count;
      } else {
        hyundai_spas_rx_count[idx] = 0U;
      }
      if (hyundai_spas_phase == 5U) {
        hyundai_spas_phase = 0U;
      }
    } else if ((addr == 0x7B9) && (GET_LEN(pkt) == 8) && (GET_BYTE(pkt, 0) >= 2U) && (GET_BYTE(pkt, 0) <= 7U)) {
      if (get_ts_elapsed(now, hyundai_spas_phase_at) < 400000U) {
        if ((hyundai_spas_phase == 1U) && (GET_BYTE(pkt, 1) == 0x50U) && (GET_BYTE(pkt, 2) == 3U)) {
          hyundai_spas_phase = 2U;
        } else if ((hyundai_spas_phase == 3U) && (GET_BYTE(pkt, 1) == 0x68U) && (GET_BYTE(pkt, 2) == 3U)) {
          hyundai_spas_phase = 4U;
          hyundai_spas_phase_at = now;
        } else if (GET_BYTE(pkt, 1) == 0x7FU) {
          hyundai_spas_phase = 0U;
        } else {
        }
      }
    }
  }
}

static bool hyundai_spas_tx(const CANPacket_t *pkt, uint32_t now) {
  bool ok = hyundai_spas_enabled && (GET_BUS(pkt) == hyundai_spas_bus) && !relay_malfunction;
  int addr = GET_ADDR(pkt);
  if ((addr == 0x7B1) && ok) {
    ok = (GET_LEN(pkt) == 8) && (GET_BYTES(pkt, 4, 4) == 0U);
    uint32_t cmd = GET_BYTES(pkt, 0, 4);
    if (ok && ((cmd == 0x01002803U) || (cmd == 0x00011002U))) {
      // Recovery remains possible after a lost lease or ECU rejection.
      hyundai_spas_phase = 0U;
    } else if (ok && (cmd == 0x00031002U)) {
      ok = (hyundai_spas_phase == 0U) && hyundai_spas_stock_ready(now);
      if (ok) {
        hyundai_spas_phase = 1U;
        hyundai_spas_phase_at = hyundai_spas_started = hyundai_spas_alive = now;
        for (int i = 0; i < 2; i++) {
          hyundai_spas_tx_seen[i] = false;
        }
      }
    } else if (ok && (cmd == 0x01032803U)) {
      ok = (hyundai_spas_phase == 2U) && (get_ts_elapsed(now, hyundai_spas_phase_at) < 400000U);
      if (ok) {
        hyundai_spas_phase = 3U;
        hyundai_spas_phase_at = now;
      }
    } else if (ok && (cmd == 0x00803E02U)) {
      ok = (hyundai_spas_phase == 5U) && (get_ts_elapsed(now, hyundai_spas_alive) < 1500000U) &&
           (get_ts_elapsed(now, hyundai_spas_started) < 300000000U) &&
           (get_ts_elapsed(now, hyundai_spas_alive) >= 500000U);
      if (ok) {
        hyundai_spas_alive = now;
      }
    } else {
      ok = false;
    }
  } else if (ok && ((addr == 0x165) || (addr == 0x16A))) {
    int idx = (addr == 0x165) ? 0 : 1;
    bool quiet = (get_ts_elapsed(now, hyundai_spas_rx_at[0]) >= 100000U) &&
                 (get_ts_elapsed(now, hyundai_spas_rx_at[1]) >= 100000U);
    if ((hyundai_spas_phase == 4U) && quiet && (get_ts_elapsed(now, hyundai_spas_phase_at) >= 100000U) &&
        (get_ts_elapsed(now, hyundai_spas_phase_at) < 500000U)) {
      hyundai_spas_phase = 5U;
    }
    ok = (hyundai_spas_phase == 5U) && quiet && (GET_LEN(pkt) == ((idx == 0) ? 24 : 32)) &&
         (get_ts_elapsed(now, hyundai_spas_started) < 300000000U) &&
         (get_ts_elapsed(now, hyundai_spas_alive) < 1500000U);
    if (ok) {
      ok = hyundai_spas_crc(pkt);
      for (int i = 3; i < GET_LEN(pkt); i++) {
        uint8_t b = GET_BYTE(pkt, i);
        bool indicator = (idx == 1) && (i == 16) && ((b == 0U) || (b == 0x60U) || (b == 0x80U));
        ok = ok && ((b == 0U) || indicator);
      }
      if (hyundai_spas_tx_seen[idx]) {
        ok = ok && (get_ts_elapsed(now, hyundai_spas_tx_at[idx]) >= 15000U) &&
             (get_ts_elapsed(now, hyundai_spas_tx_at[idx]) < 200000U) &&
             (GET_BYTE(pkt, 2) == ((hyundai_spas_tx_counter[idx] + 1U) & 0xFFU));
      }
      if (ok) {
        hyundai_spas_tx_seen[idx] = true;
        hyundai_spas_tx_at[idx] = now;
        hyundai_spas_tx_counter[idx] = GET_BYTE(pkt, 2);
      }
    }
  } else {
    ok = false;
  }
  return ok;
}
