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
// Separate ten-second parked comparison. Normal driving still needs the
// acknowledged ECU handoff. This path never authorizes diagnostic commands.
static uint32_t hyundai_spas_park_at[3], hyundai_spas_trial_at;
static uint8_t hyundai_spas_park_counter[2], hyundai_spas_park_count[3], hyundai_spas_trial_direction;
static bool hyundai_spas_park_value[3], hyundai_spas_trial_used, hyundai_spas_stock_idle;
static bool hyundai_spas_patch_candidate, hyundai_spas_patch_present;
static uint8_t hyundai_stalk_latest[8], hyundai_stalk_original[8];
static uint32_t hyundai_stalk_tx_at;

static void hyundai_spas_init(bool enabled, int bus) {
  hyundai_spas_enabled = enabled;
  hyundai_spas_bus = bus;
  hyundai_spas_phase = 0U;
  hyundai_spas_phase_at = hyundai_spas_started = hyundai_spas_alive = 0U;
  hyundai_spas_trial_at = 0U;
  hyundai_spas_trial_used = hyundai_spas_stock_idle = false;
  hyundai_spas_trial_direction = 0U;
  hyundai_spas_patch_candidate = hyundai_spas_patch_present = false;
  for (int i = 0; i < 3; i++) {
    hyundai_spas_park_at[i] = hyundai_spas_park_count[i] = 0U;
    hyundai_spas_park_value[i] = false;
  }
  hyundai_spas_park_counter[0] = hyundai_spas_park_counter[1] = 0U;
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

static bool hyundai_spas_park_guard(uint32_t now) {
  bool ready = !controls_allowed && !vehicle_moving && !gas_pressed && (hyundai_spas_bus == 0);
  for (int i = 0; i < 3; i++) {
    ready = ready && hyundai_spas_park_value[i] && (hyundai_spas_park_count[i] >= ((i == 2) ? 3U : 10U)) &&
            (get_ts_elapsed(now, hyundai_spas_park_at[i]) < ((i == 2) ? 500000U : 100000U));
  }
  // Corrupt checked frames never reach the vehicle RX hook. Consult the
  // common validator as well, instead of retaining the previous good sample.
  for (int i = 0; i < current_safety_config.rx_checks_len; i++) {
    const RxCheck *check = &current_safety_config.rx_checks[i];
    const CanMsgCheck *msg = &check->msg[check->status.index];
    if ((msg->bus == hyundai_spas_bus) && ((msg->addr == 0x35) || (msg->addr == 0xA0))) {
      ready = ready && check->status.msg_seen && check->status.valid_checksum && check->status.valid_quality_flag &&
              (check->status.wrong_counters < MAX_WRONG_COUNTERS) && !check->status.lagging &&
              (get_ts_elapsed(now, check->status.last_timestamp) < 100000U);
    }
  }
  return ready;
}

static bool hyundai_spas_park_ready(uint32_t now) {
  return hyundai_spas_park_guard(now) && hyundai_spas_stock_idle && hyundai_spas_stock_ready(now);
}

static void hyundai_spas_rx(const CANPacket_t *pkt, uint32_t now) {
  if (hyundai_spas_enabled && (GET_BUS(pkt) == hyundai_spas_bus)) {
    int addr = GET_ADDR(pkt);
    if ((addr == 0xA0) || (addr == 0x35)) {
      int idx = (addr == 0xA0) ? 0 : 1;
      bool valid = (GET_LEN(pkt) == ((idx == 0) ? 24 : 32)) && hyundai_spas_crc(pkt);
      if (valid) {
        uint8_t count = GET_BYTE(pkt, 2);
        bool advancing = (get_ts_elapsed(now, hyundai_spas_park_at[idx]) < 100000U) &&
                         (count == ((hyundai_spas_park_counter[idx] + 1U) & 0xFFU));
        hyundai_spas_park_count[idx] = advancing ? MIN(hyundai_spas_park_count[idx] + 1U, 20U) : 1U;
        hyundai_spas_park_at[idx] = now;
        hyundai_spas_park_counter[idx] = count;
        if (idx == 0) {
          hyundai_spas_park_value[idx] = (GET_BYTES(pkt, 8, 2) & 0x3FFFU) == 0U &&
            (GET_BYTES(pkt, 10, 2) & 0x3FFFU) == 0U && (GET_BYTES(pkt, 12, 2) & 0x3FFFU) == 0U &&
            (GET_BYTES(pkt, 14, 2) & 0x3FFFU) == 0U;
        } else {
          // Known EV ACCELERATOR layout: P=0, pedal=0. Other layouts do not arm.
          hyundai_spas_park_value[idx] = ((GET_BYTE(pkt, 24) & 7U) == 0U) && (GET_BYTE(pkt, 5) == 0U);
        }
      } else {
        hyundai_spas_park_count[idx] = 0U;
      }
    } else if (addr == 0x3C1) {
      uint8_t crc = 0U;
      for (int i = 1; i < GET_LEN(pkt); i++) {
        crc ^= GET_BYTE(pkt, i);
        for (int b = 0; b < 8; b++) {
          crc = (crc << 1) ^ (((crc & 0x80U) != 0U) ? 0x1DU : 0U);
        }
      }
      if ((GET_LEN(pkt) == 8) && (GET_BYTE(pkt, 0) == (crc ^ 0x65U))) {
        hyundai_spas_park_count[2] = (get_ts_elapsed(now, hyundai_spas_park_at[2]) < 500000U) ? MIN(hyundai_spas_park_count[2] + 1U, 10U) : 1U;
        hyundai_spas_park_at[2] = now;
        for (int i = 0; i < 8; i++) { hyundai_stalk_latest[i] = GET_BYTE(pkt, i); }
        hyundai_spas_park_value[2] = ((GET_BYTE(pkt, 3) & 0x40U) == 0U) && ((GET_BYTE(pkt, 4) & 0x51U) == 0U);
      } else {
        hyundai_spas_park_count[2] = 0U;
      }
    }
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
      if (idx == 1) {
        hyundai_spas_stock_idle = (GET_LEN(pkt) == 32) && hyundai_spas_crc(pkt) && ((GET_BYTE(pkt, 16) & 0xE0U) == 0U);
      }
      if (hyundai_spas_phase == 5U) {
        hyundai_spas_phase = 0U;
      }
    } else if ((addr == 0x7B9) && (GET_LEN(pkt) == 8) && (GET_BYTE(pkt, 0) >= 2U) && (GET_BYTE(pkt, 0) <= 7U)) {
      if ((hyundai_spas_phase <= 5U) && (get_ts_elapsed(now, hyundai_spas_phase_at) < 400000U)) {
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
    if ((hyundai_spas_phase == 6U) && (!hyundai_spas_park_ready(now) || (get_ts_elapsed(now, hyundai_spas_trial_at) >= 10000000U))) {
      hyundai_spas_phase = 7U;  // revoked, cooldown cannot be refreshed by TX
    }
    if ((hyundai_spas_phase == 8U) && (!hyundai_spas_park_guard(now) ||
        (get_ts_elapsed(now, hyundai_spas_trial_at) >= 10000000U) || (get_ts_elapsed(now, hyundai_spas_alive) >= 1500000U))) {
      hyundai_spas_phase = 7U;
    }
    if (hyundai_spas_phase == 9U) {
      bool unchanged = true;
      for (int i = 0; i < 8; i++) { unchanged = unchanged && (hyundai_stalk_latest[i] == hyundai_stalk_original[i]); }
      if (!unchanged || !hyundai_spas_park_guard(now) ||
          (get_ts_elapsed(now, hyundai_spas_trial_at) >= 10000000U) ||
          (get_ts_elapsed(now, hyundai_stalk_tx_at) >= 150000U)) {
        hyundai_spas_phase = 7U;
      }
    }
    if ((hyundai_spas_phase == 7U) && (get_ts_elapsed(now, hyundai_spas_trial_at) >= 30000000U)) {
      hyundai_spas_phase = 0U;
    }
  }
}

// Parked 0x3C1 comparison only. No APRK/SPAS or longitudinal commands.
static bool hyundai_stalk_tx(const CANPacket_t *pkt, uint32_t now) {
  bool ready = hyundai_spas_enabled && !relay_malfunction && (GET_BUS(pkt) == 0) &&
               (GET_LEN(pkt) == 8) && hyundai_spas_park_guard(now);
  if (hyundai_spas_phase == 9U) {
    for (int i = 0; i < 8; i++) { ready = ready && (hyundai_stalk_latest[i] == hyundai_stalk_original[i]); }
    ready = ready && (get_ts_elapsed(now, hyundai_spas_trial_at) < 10000000U) &&
            (get_ts_elapsed(now, hyundai_stalk_tx_at) < 150000U);
    if (!ready) { hyundai_spas_phase = 7U; return false; }
  } else {
    ready = ready && ((hyundai_spas_phase == 0U) || (hyundai_spas_phase == 7U)) &&
            (!hyundai_spas_trial_used || (get_ts_elapsed(now, hyundai_spas_trial_at) >= 30000000U));
  }
  if (!ready) { return false; }
  bool left = (GET_BYTE(pkt, 3) & 0x40U) != 0U;
  bool right = (GET_BYTE(pkt, 4) & 1U) != 0U;
  bool ok = left != right;
  uint8_t crc = 0U;
  for (int i = 1; i < 8; i++) {
    uint8_t expected = hyundai_stalk_latest[i];
    if (i == 1) { expected = ((((expected >> 4) + 1U) & 15U) << 4) | (expected & 15U); }
    if (i == 3) { expected = (expected & ~0x40U) | (left ? 0x40U : 0U); }
    if (i == 4) { expected = (expected & ~0x51U) | (right ? 1U : 0U); }
    ok = ok && (GET_BYTE(pkt, i) == expected);
    crc ^= GET_BYTE(pkt, i);
    for (int b = 0; b < 8; b++) { crc = (crc << 1) ^ (((crc & 0x80U) != 0U) ? 0x1DU : 0U); }
  }
  ok = ok && (GET_BYTE(pkt, 0) == (crc ^ 0x65U));
  if (hyundai_spas_phase == 9U) {
    ok = ok && (get_ts_elapsed(now, hyundai_stalk_tx_at) >= 40000U) &&
         (hyundai_spas_trial_direction == (left ? 1U : 2U));
  }
  if (ok) {
    if (hyundai_spas_phase != 9U) {
      for (int i = 0; i < 8; i++) { hyundai_stalk_original[i] = hyundai_stalk_latest[i]; }
      hyundai_spas_phase = 9U;
      hyundai_spas_trial_at = now;
      hyundai_spas_trial_used = true;
      hyundai_spas_trial_direction = left ? 1U : 2U;
    }
    hyundai_stalk_tx_at = now;
  }
  return ok;
}

static bool hyundai_spas_tx(const CANPacket_t *pkt, uint32_t now) {
  bool ok = hyundai_spas_enabled && (GET_BUS(pkt) == hyundai_spas_bus) && !relay_malfunction;
  int addr = GET_ADDR(pkt);
  if ((addr == 0x7B1) && ok) {
    ok = (GET_LEN(pkt) == 8) && (GET_BYTES(pkt, 4, 4) == 0U);
    uint32_t cmd = GET_BYTES(pkt, 0, 4);
    if (((hyundai_spas_phase == 6U) || (hyundai_spas_phase == 7U)) && !(ok && (cmd == 0x00011002U))) {
      return false;
    }
    if (ok && ((cmd == 0x01002803U) || (cmd == 0x00011002U))) {
      // Exact default-session recovery remains possible after parked revocation.
      hyundai_spas_phase = (hyundai_spas_phase >= 6U) ? 7U : 0U;
      hyundai_spas_patch_candidate = false;
    } else if (ok && (cmd == 0x00031002U)) {
      ok = (hyundai_spas_phase == 0U) && hyundai_spas_stock_ready(now);
      if (ok) {
        hyundai_spas_phase = 1U;
        hyundai_spas_phase_at = hyundai_spas_started = hyundai_spas_alive = now;
        hyundai_spas_patch_candidate = hyundai_spas_park_ready(now) &&
          (!hyundai_spas_trial_used || (get_ts_elapsed(now, hyundai_spas_trial_at) >= 30000000U));
        for (int i = 0; i < 2; i++) {
          hyundai_spas_tx_seen[i] = false;
        }
      }
    } else if (ok && (cmd == 0x01832803U)) {
      // Supplied patch's response-suppressed command is confined to a parked
      // ten-second lease. It does not relax the normal acknowledged handoff.
      ok = (hyundai_spas_phase <= 2U) && hyundai_spas_patch_candidate && hyundai_spas_park_ready(now) &&
           (get_ts_elapsed(now, hyundai_spas_started) >= 40000U) &&
           (get_ts_elapsed(now, hyundai_spas_started) < 400000U);
      if (ok) {
        hyundai_spas_phase = 8U;
        hyundai_spas_trial_at = hyundai_spas_started;
        hyundai_spas_trial_used = true;
        hyundai_spas_trial_direction = 0U;
        hyundai_spas_patch_candidate = hyundai_spas_patch_present = false;
        hyundai_spas_tx_seen[0] = hyundai_spas_tx_seen[1] = false;
      }
    } else if (ok && (cmd == 0x01032803U)) {
      ok = (hyundai_spas_phase == 2U) && (get_ts_elapsed(now, hyundai_spas_phase_at) < 400000U);
      if (ok) {
        hyundai_spas_phase = 3U;
        hyundai_spas_phase_at = now;
      }
    } else if (ok && (cmd == 0x00803E02U)) {
      bool patch = hyundai_spas_phase == 8U;
      ok = patch ? (hyundai_spas_park_guard(now) && (get_ts_elapsed(now, hyundai_spas_trial_at) < 10000000U) &&
           (get_ts_elapsed(now, hyundai_spas_alive) < 1500000U) &&
           (!hyundai_spas_patch_present || (get_ts_elapsed(now, hyundai_spas_alive) >= 500000U))) :
           ((hyundai_spas_phase == 5U) && (get_ts_elapsed(now, hyundai_spas_alive) < 1500000U) &&
           (get_ts_elapsed(now, hyundai_spas_started) < 300000000U) &&
           (get_ts_elapsed(now, hyundai_spas_alive) >= 500000U));
      if (ok) {
        hyundai_spas_alive = now;
        hyundai_spas_patch_present = patch;
      }
    } else {
      ok = false;
    }
  } else if (ok && ((addr == 0x165) || (addr == 0x16A))) {
    int idx = (addr == 0x165) ? 0 : 1;
    bool cooldown = !hyundai_spas_trial_used || (get_ts_elapsed(now, hyundai_spas_trial_at) >= 30000000U);
    if (((hyundai_spas_phase == 0U) || (hyundai_spas_phase == 7U)) && cooldown &&
        (idx == 0) && hyundai_spas_park_ready(now)) {
      // Do not let a malformed arm packet consume or start the trial.
      bool arm = (GET_LEN(pkt) == 24) && hyundai_spas_crc(pkt) &&
                 (GET_BYTE(pkt, 2) == ((hyundai_spas_rx_counter[0] + 1U) & 0xFFU));
      for (int i = 3; i < GET_LEN(pkt); i++) {
        arm = arm && (GET_BYTE(pkt, i) == 0U);
      }
      if (arm) {
        hyundai_spas_phase = 6U;
        hyundai_spas_trial_at = now;
        hyundai_spas_trial_used = true;
        hyundai_spas_trial_direction = 0U;
        hyundai_spas_tx_seen[0] = hyundai_spas_tx_seen[1] = false;
      }
    }
    bool patch = hyundai_spas_phase == 8U;
    bool trial = (hyundai_spas_phase == 6U) || patch;
    bool park_ready = patch ? (hyundai_spas_park_guard(now) && (get_ts_elapsed(now, hyundai_spas_alive) < 1500000U)) : hyundai_spas_park_ready(now);
    if (trial && (!park_ready || (get_ts_elapsed(now, hyundai_spas_trial_at) >= 10000000U))) {
      hyundai_spas_phase = 7U;
      return false;
    }
    bool quiet = (get_ts_elapsed(now, hyundai_spas_rx_at[0]) >= 100000U) &&
                 (get_ts_elapsed(now, hyundai_spas_rx_at[1]) >= 100000U);
    if ((hyundai_spas_phase == 4U) && quiet && (get_ts_elapsed(now, hyundai_spas_phase_at) >= 100000U) &&
        (get_ts_elapsed(now, hyundai_spas_phase_at) < 500000U)) {
      hyundai_spas_phase = 5U;
    }
    ok = (trial || ((hyundai_spas_phase == 5U) && quiet &&
         (get_ts_elapsed(now, hyundai_spas_started) < 300000000U) &&
         (get_ts_elapsed(now, hyundai_spas_alive) < 1500000U))) && (GET_LEN(pkt) == ((idx == 0) ? 24 : 32));
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
      if (trial && (idx == 1)) {
        uint8_t direction = GET_BYTE(pkt, 16);
        ok = ok && ((direction == 0U) || (hyundai_spas_trial_direction == 0U) || (direction == hyundai_spas_trial_direction));
        if (ok && (direction != 0U)) {
          hyundai_spas_trial_direction = direction;
        }
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
