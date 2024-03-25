void default_rx_hook(const CANPacket_t *to_push) {
  UNUSED(to_push);
}

// *** no output safety mode ***

static safety_config nooutput_init(uint16_t param) {
  UNUSED(param);
  return (safety_config){NULL, 0, NULL, 0};
}

static bool nooutput_tx_hook(const CANPacket_t *to_send) {
  UNUSED(to_send);
  return false;
}

static int default_fwd_hook(int bus_num, int addr) {
  UNUSED(bus_num);
  UNUSED(addr);
  return -1;
}

const safety_hooks nooutput_hooks = {
  .init = nooutput_init,
  .rx = default_rx_hook,
  .tx = nooutput_tx_hook,
  .fwd = default_fwd_hook,
};

// *** all output safety mode ***

// Enables passthrough mode where relay is open and bus 0 gets forwarded to bus 2 and vice versa
const uint16_t ALLOUTPUT_PARAM_PASSTHROUGH = 1;
bool alloutput_passthrough = false;

static safety_config alloutput_init(uint16_t param) {
  controls_allowed = true;
  alloutput_passthrough = GET_FLAG(param, ALLOUTPUT_PARAM_PASSTHROUGH);
  print("panda::: allooutput_init$$$$$$$$$$$= "); putui((uint32_t)alloutput_passthrough); print("\n");
  return (safety_config){NULL, 0, NULL, 0};
}

static bool alloutput_tx_hook(const CANPacket_t *to_send) {
  UNUSED(to_send);
  return true;
}

uint32_t last_ts_lkas_msg_acan = 0;
bool lkas_msg_acan_active = false;

int addr_list1[128];
int addr_list_count1 = 0;

static int alloutput_fwd_hook(int bus_num, int addr) {
  int bus_fwd = -1;
  //UNUSED(addr);
  uint32_t now = microsecond_timer_get();

  if (alloutput_passthrough) {
    if (bus_num == 0) {
      bus_fwd = 2;
      if (addr == 272 || addr == 80) { // || addr == 81) { // || addr == 866 || addr == 676) {
          last_ts_lkas_msg_acan = now;
          lkas_msg_acan_active = true;
          print("blocking\n");
          bus_fwd = -1;
      }
    }
    if (bus_num == 2) {
        int i;
        for (i = 0; i < addr_list_count1; i++) {
            if (addr_list1[i] == addr) {
                break;
            }
        }
        if (i == addr_list_count1) {
            addr_list1[addr_list_count1] = addr;
            addr_list_count1++;
            print("bus222_list33=");
            for (int j = 0; j < addr_list_count1; j++) { putui((uint32_t)addr_list1[j]); print(","); }
            print("\n");
        }

      bus_fwd = 0;
      if (addr == 272 || addr == 80) { // || addr == 81) { // || addr == 866 || addr == 676) {
          if (now - last_ts_lkas_msg_acan < 200000) {
              bus_fwd = -1;
          }
          else lkas_msg_acan_active = false;
      }
    }
  }

  return bus_fwd;
}

const safety_hooks alloutput_hooks = {
  .init = alloutput_init,
  .rx = default_rx_hook,
  .tx = alloutput_tx_hook,
  .fwd = alloutput_fwd_hook,
};
