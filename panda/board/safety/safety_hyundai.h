#include "safety_hyundai_common.h"

const SteeringLimits HYUNDAI_STEERING_LIMITS = {
  .max_steer = 509,
  .max_rt_delta = 300, //112,
  .max_rt_interval = 250000,
  .max_rate_up = 30,
  .max_rate_down = 30,
  .driver_torque_allowance = 350,
  .driver_torque_factor = 2,
  .type = TorqueDriverLimited,

  // the EPS faults when the steering angle is above a certain threshold for too long. to prevent this,
  // we allow setting CF_Lkas_ActToi bit to 0 while maintaining the requested torque value for two consecutive frames
  .min_valid_request_frames = 89,
  .max_invalid_request_frames = 2,
  .min_valid_request_rt_interval = 810000,  // 810ms; a ~10% buffer on cutting every 90 frames
  .has_steer_req_tolerance = true,
};

const int HYUNDAI_MAX_ACCEL = 250;  // 1/100 m/s2
const int HYUNDAI_MIN_ACCEL = -400; // -350; // 1/100 m/s2

const CanMsg HYUNDAI_TX_MSGS[] = {
  {593, 2, 8},                              // MDPS12, Bus 2
  {832, 0, 8},  // LKAS11 Bus 0
  {1265, 0, 4}, // CLU11 Bus 0
  {1157, 0, 4}, // LFAHDA_MFC Bus 0
};

const CanMsg HYUNDAI_LONG_TX_MSGS[] = {
  {593, 2, 8},  // MDPS12, Bus 2
  {832, 0, 8},  // LKAS11 Bus 0
  {832, 1, 8},  // LKAS11 Bus 1
  {1265, 0, 4}, // CLU11 Bus 0
  {1265, 1, 4}, // CLU11 Bus 1
  {1157, 0, 4}, // LFAHDA_MFC Bus 0
  {1056, 0, 8}, // SCC11 Bus 0
  {1057, 0, 8}, // SCC12 Bus 0
  {1290, 0, 8}, // SCC13 Bus 0
  {905, 0, 8},  // SCC14 Bus 0
  {1186, 0, 2}, // FRT_RADAR11 Bus 0
  {909, 0, 8},  // FCA11 Bus 0
  {1155, 0, 8}, // FCA12 Bus 0
  {2000, 0, 8}, // radar UDS TX addr Bus 0 (for radar disable)
};

const CanMsg HYUNDAI_CAMERA_SCC_TX_MSGS[] = {
  {593, 2, 8},                              // MDPS12, Bus 2
  {832, 0, 8},  // LKAS11 Bus 0
  {1265, 2, 4}, // CLU11 Bus 2
  {1157, 0, 4}, // LFAHDA_MFC Bus 0
};

AddrCheckStruct hyundai_addr_checks[] = {
  {.msg = {{608, 0, 8, .check_checksum = true, .max_counter = 3U, .expected_timestep = 10000U},
           {881, 0, 8, .expected_timestep = 10000U}, { 0 }}},
  {.msg = {{902, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{916, 0, 8, .check_checksum = true, .max_counter = 7U, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{1057, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},
};
#define HYUNDAI_ADDR_CHECK_LEN (sizeof(hyundai_addr_checks) / sizeof(hyundai_addr_checks[0]))

AddrCheckStruct hyundai_cam_scc_addr_checks[] = {
  {.msg = {{608, 0, 8, .check_checksum = true, .max_counter = 3U, .expected_timestep = 10000U},
           {881, 0, 8, .expected_timestep = 10000U}, { 0 }}},
  {.msg = {{902, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{916, 0, 8, .check_checksum = true, .max_counter = 7U, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{1057, 2, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},
};
#define HYUNDAI_CAM_SCC_ADDR_CHECK_LEN (sizeof(hyundai_cam_scc_addr_checks) / sizeof(hyundai_cam_scc_addr_checks[0]))

AddrCheckStruct hyundai_long_addr_checks[] = {
  {.msg = {{608, 0, 8, .check_checksum = true, .max_counter = 3U, .expected_timestep = 10000U},
           {881, 0, 8, .expected_timestep = 10000U}, { 0 }}},
  {.msg = {{902, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{916, 0, 8, .check_checksum = true, .max_counter = 7U, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{1265, 0, 4, .check_checksum = false, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},
};
#define HYUNDAI_LONG_ADDR_CHECK_LEN (sizeof(hyundai_long_addr_checks) / sizeof(hyundai_long_addr_checks[0]))

// older hyundai models have less checks due to missing counters and checksums
AddrCheckStruct hyundai_legacy_addr_checks[] = {
  {.msg = {{608, 0, 8, .check_checksum = true, .max_counter = 3U, .expected_timestep = 10000U},
           {881, 0, 8, .expected_timestep = 10000U}, { 0 }}},
  {.msg = {{902, 0, 8, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{916, 0, 8, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{1057, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},
};
#define HYUNDAI_LEGACY_ADDR_CHECK_LEN (sizeof(hyundai_legacy_addr_checks) / sizeof(hyundai_legacy_addr_checks[0]))

AddrCheckStruct hyundai_legacy_long_addr_checks[] = {
  {.msg = {{608, 0, 8, .check_checksum = true, .max_counter = 3U, .expected_timestep = 10000U},
           {881, 0, 8, .expected_timestep = 10000U}, { 0 }}},
  {.msg = {{902, 0, 8, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{916, 0, 8, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{1265, 0, 4, .check_checksum = false, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},
};
#define HYUNDAI_LEGACY_LONG_ADDR_CHECK_LEN (sizeof(hyundai_legacy_long_addr_checks) / sizeof(hyundai_legacy_long_addr_checks[0]))

bool hyundai_legacy = false;

int     busSCC = -1;
int     busLKAS11 = -1;
int     cruiseEngaged = 0;

bool apilot_connected_prev = false;
uint32_t LKAS11_lastTxTime = 0;
uint32_t LKAS11_maxTxDiffTime = 0;
bool LKAS11_forwarding = true;
uint32_t last_ts_mdps12_from_op = 0; // from neokii


addr_checks hyundai_rx_checks = {hyundai_addr_checks, HYUNDAI_ADDR_CHECK_LEN};


static uint8_t hyundai_get_counter(CANPacket_t *to_push) {
  int addr = GET_ADDR(to_push);

  uint8_t cnt;
  if (addr == 608) {
    cnt = (GET_BYTE(to_push, 7) >> 4) & 0x3U;
  } else if (addr == 902) {
    cnt = ((GET_BYTE(to_push, 3) >> 6) << 2) | (GET_BYTE(to_push, 1) >> 6);
  } else if (addr == 916) {
    cnt = (GET_BYTE(to_push, 1) >> 5) & 0x7U;
  } else if (addr == 1057) {
    cnt = GET_BYTE(to_push, 7) & 0xFU;
  } else if (addr == 1265) {
    cnt = (GET_BYTE(to_push, 3) >> 4) & 0xFU;
  } else {
    cnt = 0;
  }
  return cnt;
}

static uint8_t hyundai_get_checksum(CANPacket_t *to_push) {
  int addr = GET_ADDR(to_push);

  uint8_t chksum;
  if (addr == 608) {
    chksum = GET_BYTE(to_push, 7) & 0xFU;
  } else if (addr == 902) {
    chksum = ((GET_BYTE(to_push, 7) >> 6) << 2) | (GET_BYTE(to_push, 5) >> 6);
  } else if (addr == 916) {
    chksum = GET_BYTE(to_push, 6) & 0xFU;
  } else if (addr == 1057) {
    chksum = GET_BYTE(to_push, 7) >> 4;
  } else {
    chksum = 0;
  }
  return chksum;
}

static uint8_t hyundai_compute_checksum(CANPacket_t *to_push) {
  int addr = GET_ADDR(to_push);

  uint8_t chksum = 0;
  if (addr == 902) {
    // count the bits
    for (int i = 0; i < 8; i++) {
      uint8_t b = GET_BYTE(to_push, i);
      for (int j = 0; j < 8; j++) {
        uint8_t bit = 0;
        // exclude checksum and counter
        if (((i != 1) || (j < 6)) && ((i != 3) || (j < 6)) && ((i != 5) || (j < 6)) && ((i != 7) || (j < 6))) {
          bit = (b >> (uint8_t)j) & 1U;
        }
        chksum += bit;
      }
    }
    chksum = (chksum ^ 9U) & 15U;
  } else {
    // sum of nibbles
    for (int i = 0; i < 8; i++) {
      if ((addr == 916) && (i == 7)) {
        continue; // exclude
      }
      uint8_t b = GET_BYTE(to_push, i);
      if (((addr == 608) && (i == 7)) || ((addr == 916) && (i == 6)) || ((addr == 1057) && (i == 7))) {
        b &= (addr == 1057) ? 0x0FU : 0xF0U; // remove checksum
      }
      chksum += (b % 16U) + (b / 16U);
    }
    chksum = (16U - (chksum %  16U)) % 16U;
  }

  return chksum;
}

static int hyundai_rx_hook(CANPacket_t *to_push) {

  bool valid = addr_safety_check(to_push, &hyundai_rx_checks,
                                 hyundai_get_checksum, hyundai_compute_checksum,
                                 hyundai_get_counter);

  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  // SCC12 is on bus 2 for camera-based SCC cars, bus 0 on all others
  if (valid && (addr == 1057) && (((bus == 0 || bus == 2) && !hyundai_camera_scc) || ((bus == 2) && hyundai_camera_scc))) {
    // 2 bits: 13-14
    int cruise_engaged = (GET_BYTES_04(to_push) >> 13) & 0x3U;
    if (cruiseEngaged != cruise_engaged) {
        puts("CruiseState...: "); puth2(cruise_engaged); puts("\n");
        cruiseEngaged = cruise_engaged;
    }
    hyundai_common_cruise_state_check2(cruise_engaged);
  }
  // SCC11 is on bus 2
  if (valid && (addr == 1056) && (((bus == 0 || bus == 2) && !hyundai_camera_scc) || ((bus == 2) && hyundai_camera_scc))) {
      // 1 bits: 0
      int cruise_engaged = (GET_BYTES_04(to_push) >> 0) & 0x1U;
      static int cruise_engaged_pre = 0;
      if (cruise_engaged_pre != cruise_engaged) {
          puts("CruiseSet...: "); puth2(cruise_engaged); puts("\n");
          cruise_engaged_pre = cruise_engaged;
      }
      hyundai_common_cruise_state_check(cruise_engaged);
  }

  if (valid && (bus == 0)) {
    if (addr == 593) {
      int torque_driver_new = ((GET_BYTES_04(to_push) & 0x7ffU) * 0.79) - 808; // scale down new driver torque signal to match previous one
      // update array of samples
      update_sample(&torque_driver, torque_driver_new);
    }

    // ACC steering wheel buttons
    if (addr == 1265) {
      int cruise_button = GET_BYTE(to_push, 0) & 0x7U;
      int main_button = GET_BIT(to_push, 3U);
      hyundai_common_cruise_buttons_check(cruise_button, main_button);
    }

    // gas press, different for EV, hybrid, and ICE models
    if ((addr == 881) && hyundai_ev_gas_signal) {
      gas_pressed = (((GET_BYTE(to_push, 4) & 0x7FU) << 1) | GET_BYTE(to_push, 3) >> 7) != 0U;
    } else if ((addr == 881) && hyundai_hybrid_gas_signal) {
      gas_pressed = GET_BYTE(to_push, 7) != 0U;
    } else if ((addr == 608) && !hyundai_ev_gas_signal && !hyundai_hybrid_gas_signal) {
      gas_pressed = (GET_BYTE(to_push, 7) >> 6) != 0U;
    } else {
    }

    // sample wheel speed, averaging opposite corners
    if (addr == 902) {
      uint32_t hyundai_speed = (GET_BYTES_04(to_push) & 0x3FFFU) + ((GET_BYTES_48(to_push) >> 16) & 0x3FFFU);  // FL + RR
      hyundai_speed /= 2;
      vehicle_moving = hyundai_speed > HYUNDAI_STANDSTILL_THRSLD;
    }

    if (addr == 916) {
      brake_pressed = GET_BIT(to_push, 55U) != 0U;
    }

    gas_pressed = brake_pressed = false;

    bool stock_ecu_detected = (addr == 832);

    // If openpilot is controlling longitudinal we need to ensure the radar is turned off
    // Enforce by checking we don't see SCC12
    if (hyundai_longitudinal && (addr == 1057)) {
      stock_ecu_detected = true;
    }
    generic_rx_checks(stock_ecu_detected);
  }
  return valid;
}

static int hyundai_tx_hook(CANPacket_t *to_send, bool longitudinal_allowed) {

  int tx = 1;
  int addr = GET_ADDR(to_send);

  if (hyundai_longitudinal) {
    tx = msg_allowed(to_send, HYUNDAI_LONG_TX_MSGS, sizeof(HYUNDAI_LONG_TX_MSGS)/sizeof(HYUNDAI_LONG_TX_MSGS[0]));
  } else if (hyundai_camera_scc) {
    tx = msg_allowed(to_send, HYUNDAI_CAMERA_SCC_TX_MSGS, sizeof(HYUNDAI_CAMERA_SCC_TX_MSGS)/sizeof(HYUNDAI_CAMERA_SCC_TX_MSGS[0]));
  } else {
    tx = msg_allowed(to_send, HYUNDAI_TX_MSGS, sizeof(HYUNDAI_TX_MSGS)/sizeof(HYUNDAI_TX_MSGS[0]));
  }

  if (addr == 1056 && hyundai_auto_engage) {
      int mainModeACC = GET_BYTE(to_send, 0) & 0x1U;

      if (mainModeACC == 1) {
          controls_allowed = 1;
          hyundai_auto_engage = 0;
          LKAS11_forwarding = true;
      }
  }

  // FCA11: Block any potential actuation
  if (addr == 909) {
    int CR_VSM_DecCmd = GET_BYTE(to_send, 1);
    int FCA_CmdAct = GET_BIT(to_send, 20U);
    int CF_VSM_DecCmdAct = GET_BIT(to_send, 31U);

    if ((CR_VSM_DecCmd != 0) || (FCA_CmdAct != 0) || (CF_VSM_DecCmdAct != 0)) {
      tx = 0;
      puts("violation[FCA11, 909]\n");
    }
  }

  // ACCEL: safety check
  if (addr == 1057) {
    int desired_accel_raw = (((GET_BYTE(to_send, 4) & 0x7U) << 8) | GET_BYTE(to_send, 3)) - 1023U;
    int desired_accel_val = ((GET_BYTE(to_send, 5) << 3) | (GET_BYTE(to_send, 4) >> 5)) - 1023U;

    //int aeb_decel_cmd = GET_BYTE(to_send, 2);
    //int aeb_req = GET_BIT(to_send, 54U);

    bool violation = 0;

    if (!longitudinal_allowed) {
      if ((desired_accel_raw != 0) || (desired_accel_val != 0)) {
          violation = 1;
      }
    }
    violation |= max_limit_check(desired_accel_raw, HYUNDAI_MAX_ACCEL, HYUNDAI_MIN_ACCEL);  
    violation |= max_limit_check(desired_accel_val, HYUNDAI_MAX_ACCEL, HYUNDAI_MIN_ACCEL);

    //violation |= (aeb_decel_cmd != 0);
    //violation |= (aeb_req != 0);

    if (violation) {
        puts("violation[1057]: raw = ");
        puth(desired_accel_raw); puts(", val="); puth(desired_accel_val); puts("\n");
      tx = 0;
    }
  }

  // LKA STEER: safety check
  if (addr == 832) {
    int desired_torque = ((GET_BYTES_04(to_send) >> 16) & 0x7ffU) - 1024U;
    bool steer_req = GET_BIT(to_send, 27U) != 0U;

    if (steer_torque_cmd_checks(desired_torque, steer_req, HYUNDAI_STEERING_LIMITS)) {
      //tx = 0;
      LKAS11_forwarding = false;// true;
      puts("violation[LKAS11, 832]\n");
    }
    else LKAS11_forwarding = false;
  }

  // UDS: Only tester present ("\x02\x3E\x80\x00\x00\x00\x00\x00") allowed on diagnostics address
  if (addr == 2000) {
    if ((GET_BYTES_04(to_send) != 0x00803E02U) || (GET_BYTES_48(to_send) != 0x0U)) {
      tx = 0;
    }
  }

  // BUTTONS: used for resume spamming and cruise cancellation
  if ((addr == 1265) && !hyundai_longitudinal) {
    int button = GET_BYTE(to_send, 0) & 0x7U;

    bool allowed_resume = (button == 1) && controls_allowed;
    bool allowed_cancel = (button == 4) && cruise_engaged_prev;
    bool allowed_set = (button == 2) && controls_allowed;
    bool allowed_gap = (button == 3) && controls_allowed;
    if (!(allowed_resume || allowed_cancel || allowed_set || allowed_gap)) {
      tx = 0;
    }
  }

  apilot_connected = true;
  if (addr == 832) {
      LKAS11_lastTxTime = microsecond_timer_get();
  }
  else if (addr == 593) {
      last_ts_mdps12_from_op = microsecond_timer_get();
  }

  return tx;
}

static int hyundai_fwd_hook(int bus_num, CANPacket_t *to_fwd) {

  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);

  int is_lkas11_msg = (addr == 832);
  int is_lfahda_mfc_msg = (addr == 1157);
  int is_scc_msg = (addr == 1056) || (addr == 1057) || (addr == 1290) || (addr == 905);
  //int is_fca_msg = (addr == 909) || (addr == 1155);
  uint32_t now = microsecond_timer_get();

  //int is_clu11_msg = (addr == 1265);
  //int is_mdps12_msg = (addr = 593);
  //int is_ems11_msg = (addr == 790);
  // forward cam to ccan and viceversa, except lkas cmd
  if (apilot_connected != apilot_connected_prev) {
      puts("[hyundai_fwd_hook] apilot_connected="); puth2(apilot_connected); puts("\n");
      apilot_connected_prev = apilot_connected;
  }
  if (bus_num == 0) {
    bus_fwd = 2;

    // from neokii
    if (addr == 593) {
        if (now - last_ts_mdps12_from_op < 200000) {
            bus_fwd = -1;
        }
    }
  }
  if (bus_num == 2) {

      //int block_msg = is_lkas11_msg || is_lfahda_mfc_msg || is_scc_msg;
      int block_msg = is_lfahda_mfc_msg || is_scc_msg;
      block_msg |= (LKAS11_forwarding) ? 0 : is_lkas11_msg;  // LKAS메시지에 불량이 있으면 TX를 안함.. 여기서 그냥 포워딩해버리자 => 시험..

      if (apilot_connected) {
          if (!block_msg) {
              bus_fwd = 0;
          }
      }
      else {
          bus_fwd = 0;
          if (is_lkas11_msg) {
              LKAS11_lastTxTime = microsecond_timer_get();
              LKAS11_maxTxDiffTime = 0;
          }
      }
  }
  if (apilot_connected) {
      //uint32_t now = microsecond_timer_get();
      uint32_t diff = now - LKAS11_lastTxTime;
      if (diff > LKAS11_maxTxDiffTime)
      {
          LKAS11_maxTxDiffTime = diff;
          puts("diff="); puth(diff); puts("\n");
      }
      if (diff > 0x15000) {
          apilot_connected = false;  // Neokii코드 참조: 오픈파일럿이 죽거나 재부팅하면,,,, 강제로 끊어줌.
          puts("apilot may be reboot...\n");
          controls_allowed = false;
      }
  }

  return bus_fwd;
}

static const addr_checks* hyundai_init(int16_t param) {
  controls_allowed = false;
  hyundai_common_init(param);
  hyundai_legacy = false;

  if (hyundai_camera_scc) {
    hyundai_longitudinal = false;
  }

  if (hyundai_longitudinal) {
    hyundai_rx_checks = (addr_checks){hyundai_long_addr_checks, HYUNDAI_LONG_ADDR_CHECK_LEN};
  } else if (hyundai_camera_scc) {
    hyundai_rx_checks = (addr_checks){hyundai_cam_scc_addr_checks, HYUNDAI_CAM_SCC_ADDR_CHECK_LEN};
  } else {
    hyundai_rx_checks = (addr_checks){hyundai_addr_checks, HYUNDAI_ADDR_CHECK_LEN};
  }
  return &hyundai_rx_checks;
}

static const addr_checks* hyundai_legacy_init(int16_t param) {
  controls_allowed = false;
  hyundai_common_init(param);
  hyundai_legacy = true;
  //hyundai_longitudinal = false;
  hyundai_camera_scc = false;
  if (hyundai_longitudinal) {
      hyundai_rx_checks = (addr_checks){ hyundai_legacy_long_addr_checks, HYUNDAI_LEGACY_LONG_ADDR_CHECK_LEN };
  }
  else {
      hyundai_rx_checks = (addr_checks){ hyundai_legacy_addr_checks, HYUNDAI_LEGACY_ADDR_CHECK_LEN };
  }
  return &hyundai_rx_checks;
}

const safety_hooks hyundai_hooks = {
  .init = hyundai_init,
  .rx = hyundai_rx_hook,
  .tx = hyundai_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = hyundai_fwd_hook,
};

const safety_hooks hyundai_legacy_hooks = {
  .init = hyundai_legacy_init,
  .rx = hyundai_rx_hook,
  .tx = hyundai_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = hyundai_fwd_hook,
};
