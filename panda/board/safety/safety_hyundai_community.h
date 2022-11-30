int LKAS11_op = 0;
int MDPS12_op = 0;
int CLU11_op = 0;
int SCC12_op = 0;
int SCC12_car = 0;
int EMS11_op = 0;
int MDPS_bus = -1;
int SCC_bus = -1;

const CanMsg HYUNDAI_COMMUNITY_TX_MSGS[] = {
  {593, 2, 8},                              // MDPS12, Bus 2
//  {897, 0, 8},                              // MDPS11, Bus 0
  {790, 1, 8},                              // EMS11, Bus 1
  {832, 0, 8}, {832, 1, 8},                 // LKAS11, Bus 0, 1
  {1056, 0, 8},                             // SCC11, Bus 0
  {1057, 0, 8},                             // SCC12, Bus 0
  {1290, 0, 8},                             // SCC13, Bus 0
  {905, 0, 8},                              // SCC14, Bus 0
  {909, 0, 8},                              // FCA11 Bus 0
  {1155, 0, 8},                             // FCA12 Bus 0
  {1157, 0, 4},                             // LFAHDA_MFC, Bus 0
  {1186, 0, 8},                             // FRT_RADAR11, Bus 0
  {1265, 0, 4}, {1265, 1, 4}, {1265, 2, 4}, // CLU11, Bus 0, 1, 2
//  {912, 0, 7}, {912,1, 7},                  // SPAS11, Bus 0, 1
//  {1268, 0, 8}, {1268,1, 8},                // SPAS12, Bus 0, 1
//  {2000, 0, 8},                             // radar UDS TX addr Bus 0 (for radar disable)
 };

// older hyundai models have less checks due to missing counters and checksums
AddrCheckStruct hyundai_community_addr_checks[] = {
  {.msg = {{608, 0, 8, .check_checksum = true, .max_counter = 3U, .expected_timestep = 10000U},     // EMS16
           {881, 0, 8, .expected_timestep = 10000U}, { 0 }}},                                       // E_EMS11
  {.msg = {{902, 0, 8, .expected_timestep = 20000U}, { 0 }, { 0 }}},                                // WHL_SPD11
//  {.msg = {{916, 0, 8, .expected_timestep = 20000U}}},                                              // TCS13
//  {.msg = {{1057, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},  // SCC12
};

#define HYUNDAI_COMMUNITY_ADDR_CHECK_LEN (sizeof(hyundai_community_addr_checks) / sizeof(hyundai_community_addr_checks[0]))

addr_checks hyundai_community_rx_checks = {hyundai_community_addr_checks, HYUNDAI_COMMUNITY_ADDR_CHECK_LEN};

static int hyundai_community_rx_hook(CANPacket_t *to_push) {
  int addr = GET_ADDR(to_push);
  int bus = GET_BUS(to_push);
  bool valid = addr_safety_check(to_push, &hyundai_community_rx_checks, hyundai_get_checksum,
                                 hyundai_compute_checksum, hyundai_get_counter);
  if (!valid){
    puts("  CAN RX invalid: "); puth(addr); puts("\n");
  }
  if (bus == 1 && Lcan_bus1) {
    valid = false;
  }

  // check if we have a LCAN on Bus1
  if (bus == 1 && (addr == 1296 || addr == 524)) {
    Lcan_bus1_cnt = 500;
    if (Fwd_bus1 || !Lcan_bus1) {
      Lcan_bus1 = true;
      Fwd_bus1 = false;
      puts("  LCAN on bus1: forwarding disabled\n");
    }
  }

  // check if LKAS11 on Bus0
  if (addr == 832) {
    if (bus == 0 && Fwd_bus2) {
      Fwd_bus2 = false;
      LKAS11_bus0_cnt = 20;
      puts("  LKAS11 on bus0: forwarding disabled\n");
    }
    if (bus == 2) {
      if (LKAS11_bus0_cnt > 0) {
        LKAS11_bus0_cnt--;
        } else if (!Fwd_bus2) {
          Fwd_bus2 = true;
          puts("  LKAS11 on bus2: forwarding enabled\n");
      }
      if (Lcan_bus1_cnt > 0) {
        Lcan_bus1_cnt--;
        } else if (Lcan_bus1) {
          Lcan_bus1 = false;
          puts("  Lcan not on bus1\n");
      }
    }
  }

  // check MDPS12 or MDPS11 on Bus
  if ((addr == 593 || addr == 897) && MDPS_bus != bus) {
    if (bus != 1 || (!Lcan_bus1 || Fwd_obd)) {
      MDPS_bus = bus;
      if (bus == 1 && !Fwd_obd) {
        puts("  MDPS on bus1\n");
        if (!Fwd_bus1 && !Lcan_bus1) {
          Fwd_bus1 = true;
          puts("  bus1 forwarding enabled\n");
          }
        } else if (bus == 1) {
          puts("  MDPS on obd bus\n");
      }
    }
  }

  // check SCC11 or SCC12 on Bus
  if ((addr == 1056 || addr == 1057) && SCC_bus != bus) {
    if (bus != 1 || !Lcan_bus1) {
      SCC_bus = bus;
      if (bus == 1) {
        puts("  SCC on bus1\n");
        if (!Fwd_bus1) {
          Fwd_bus1 = true;
          puts("  bus1 forwarding enabled\n");
        }
      }
      if (bus == 2) {
        puts("  SCC bus = bus2\n");
      }
    }
  }

  // MDPS12
  if (valid) {
    if (addr == 593 && bus == MDPS_bus) {
      int torque_driver_new = ((GET_BYTES_04(to_push) & 0x7ff) * 0.79) - 808; // scale down new driver torque signal to match previous one
      // update array of samples
      update_sample(&torque_driver, torque_driver_new);
    }

    // for cars without long control ( SCC11 )
    if (addr == 1056 && !SCC12_op) {
      // 2 bits: 13-14
      int cruise_engaged = GET_BYTES_04(to_push) & 0x1; // ACC main_on signal
      if (cruise_engaged && !cruise_engaged_prev) {
        controls_allowed = 1;
        puts("  SCC w/o long control: controls allowed\n");
      }
      if (!cruise_engaged) {
        if (controls_allowed) {
          puts("  SCC w/o long control: controls not allowed\n");}
        controls_allowed = 0;
      }
      cruise_engaged_prev = cruise_engaged;
    }

    // cruise control for car without SCC ( EMS16 )
    if (addr == 608 && bus == 0 && SCC_bus == -1 && !SCC12_op) {
      // bit 25
      int cruise_engaged = (GET_BYTES_04(to_push) >> 25 & 0x1); // ACC main_on signal
      if (cruise_engaged && !cruise_engaged_prev) {
        controls_allowed = 1;
        puts("  non-SCC w/ long control: controls allowed\n");
      }
      if (!cruise_engaged) {
        if (controls_allowed) {
          puts("  non-SCC w/ long control: controls not allowed\n");}
        controls_allowed = 0;
      }
      cruise_engaged_prev = cruise_engaged;
    }

    // sample wheel speed, averaging opposite corners ( WHL_SPD11 )
    if (addr == 902) {
      int hyundai_speed = (GET_BYTES_04(to_push) & 0x3FFFU) + ((GET_BYTES_48(to_push) >> 16) & 0x3FFFU);  // FL + RR
      hyundai_speed /= 2;
      vehicle_moving = hyundai_speed > HYUNDAI_STANDSTILL_THRSLD;
    }
    generic_rx_checks((addr == 832 && bus == 0)); // LKAS11
  }
  return valid;
}

static int hyundai_community_tx_hook(CANPacket_t *to_send, bool longitudinal_allowed) {
  UNUSED(longitudinal_allowed);

  int tx = 1;
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);

  if (!msg_allowed(to_send, HYUNDAI_COMMUNITY_TX_MSGS, sizeof(HYUNDAI_COMMUNITY_TX_MSGS)/sizeof(HYUNDAI_COMMUNITY_TX_MSGS[0]))) {
    tx = 0;
    puts("  CAN TX not allowed: "); puth(addr); puts(", "); puth(bus); puts("\n");
  }

  if (relay_malfunction) {
    tx = 0;
    puts("  CAN TX not allowed LKAS on bus0\n");
  }

  // LKA STEER: safety check
  if (addr == 832) {
    LKAS11_op = 20;
    int desired_torque = ((GET_BYTES_04(to_send) >> 16) & 0x7ff) - 1024;
    uint32_t ts = microsecond_timer_get();
    bool violation = 0;

    if (controls_allowed) {
      // *** global torque limit check ***
      bool torque_check = 0;
      violation |= torque_check = max_limit_check(desired_torque, HYUNDAI_MAX_STEER, -HYUNDAI_MAX_STEER);
      if (torque_check) {
        puts("  LKAS TX not allowed: torque limit check failed!\n");}

      // *** torque rate limit check ***
      bool torque_rate_check = 0;
      violation |= torque_rate_check = driver_limit_check(desired_torque, desired_torque_last, &torque_driver,
        HYUNDAI_MAX_STEER, HYUNDAI_MAX_RATE_UP, HYUNDAI_MAX_RATE_DOWN,
        HYUNDAI_DRIVER_TORQUE_ALLOWANCE, HYUNDAI_DRIVER_TORQUE_FACTOR);
      if (torque_rate_check) {
        puts("  LKAS TX not allowed: torque rate limit check failed!\n");}

      // used next time
      desired_torque_last = desired_torque;

      // *** torque real time rate limit check ***
      bool torque_rt_check = 0;
      violation |= torque_rt_check = rt_rate_limit_check(desired_torque, rt_torque_last, HYUNDAI_MAX_RT_DELTA);
      if (torque_rt_check) {
        puts("  LKAS TX not allowed: torque real time rate limit check failed!\n");}

      // every RT_INTERVAL set the new limits
      uint32_t ts_elapsed = get_ts_elapsed(ts, ts_last);
      if (ts_elapsed > HYUNDAI_RT_INTERVAL) {
        rt_torque_last = desired_torque;
        ts_last = ts;
      }
    }

    // no torque if controls is not allowed
    if (!controls_allowed && (desired_torque != 0)) {
      violation = 1;
      puts("  LKAS torque not allowed: controls not allowed!\n");
    }

    // reset to 0 if either controls is not allowed or there's a violation
    if (!controls_allowed) { // a reset worsen the issue of Panda blocking some valid LKAS messages
      desired_torque_last = 0;
      rt_torque_last = 0;
      ts_last = ts;
    }

    if (violation) {
      tx = 0;
    }
  }

  // FORCE CANCEL: safety check only relevant when spamming the cancel button.
  // ensuring that only the cancel button press is sent (VAL 4) when controls are off.
  // This avoids unintended engagements while still allowing resume spam
  //allow clu11 to be sent to MDPS if MDPS is not on bus0
  if (addr == 1265 && !controls_allowed && (bus != MDPS_bus && MDPS_bus == 1)) {
    if ((GET_BYTES_04(to_send) & 0x7) != 4) {
      tx = 0;
    }
  }

  if (addr == 593) { MDPS12_op = 20; }
  if (addr == 1265 && bus == 1) { CLU11_op = 20; } // only count mesage created for MDPS
  if (addr == 1057) { SCC12_op = 20; if (SCC12_car > 0) { SCC12_car -= 1; }}
  if (addr == 790) { EMS11_op = 20; }

  // 1 allows the message through
  return tx;
}

static int hyundai_community_fwd_hook(int bus_num, CANPacket_t *to_fwd) {
  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);
  int fwd_to_bus1 = -1;
  if (Fwd_bus1 || Fwd_obd){fwd_to_bus1 = 1;}

  // forward cam to ccan and viceversa, except lkas cmd
  if (Fwd_bus2) {
    if (bus_num == 0) {
      if (!CLU11_op || addr != 1265 || MDPS_bus == 0) {
        if (!MDPS12_op || addr != 593) {
          if (!EMS11_op || addr != 790) {
            bus_fwd = fwd_to_bus1 == 1 ? 12 : 2;
          } else {
            bus_fwd = 2;  // OP create EMS11 for MDPS
            EMS11_op -= 1;
          }
        } else {
          bus_fwd = fwd_to_bus1;  // OP create MDPS for LKAS
          MDPS12_op -= 1;
        }
      } else {
        bus_fwd = 2; // OP create CLU12 for MDPS
        CLU11_op -= 1;
      }
    }
    if (bus_num == 1 && (Fwd_bus1 || Fwd_obd)) {
      if (!MDPS12_op || addr != 593) {
        if (!SCC12_op || (addr != 1056 && addr != 1057 && addr != 1290 && addr != 905)) {
          bus_fwd = 20;
        } else {
          bus_fwd = 2;  // OP create SCC11 SCC12 SCC13 SCC14 for Car
          SCC12_op -= 1;
        }
      } else {
        bus_fwd = 0;  // OP create MDPS for LKAS
        MDPS12_op -= 1;
      }
    }
    if (bus_num == 2) {
      if (!LKAS11_op || (addr != 832 && addr != 1157)) {
        if (!SCC12_op || (addr != 1056 && addr != 1057 && addr != 1290 && addr != 905)) {
          bus_fwd = fwd_to_bus1 == 1 ? 10 : 0;
        } else {
          bus_fwd = fwd_to_bus1;  // OP create SCC12 for Car
          SCC12_op -= 1;
        }
      } else if (MDPS_bus == 0) {
        bus_fwd = fwd_to_bus1; // OP create LKAS and LFA for Car
        LKAS11_op -= 1;
      } else {
        LKAS11_op -= 1; // OP create LKAS and LFA for Car and MDPS
      }
    }
  } else {
    if (bus_num == 0) {
      bus_fwd = fwd_to_bus1;
    }
    if (bus_num == 1 && (Fwd_bus1 || Fwd_obd)) {
      bus_fwd = 0;
    }
  }
  return bus_fwd;
}

static const addr_checks* hyundai_community_init(int16_t param) {
  UNUSED(param);
  controls_allowed = false;
  relay_malfunction_reset();

  if (current_board->has_obd && Fwd_obd) {
    current_board->set_can_mode(CAN_MODE_OBD_CAN2);
    puts("  MDPS or SCC on OBD2 CAN: setting can mode obd\n");
  }

  hyundai_community_rx_checks = (addr_checks){hyundai_community_addr_checks, HYUNDAI_COMMUNITY_ADDR_CHECK_LEN};
  return &hyundai_community_rx_checks;
}

const safety_hooks hyundai_community_hooks = {
  .init = hyundai_community_init,
  .rx = hyundai_community_rx_hook,
  .tx = hyundai_community_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = hyundai_community_fwd_hook,
};
