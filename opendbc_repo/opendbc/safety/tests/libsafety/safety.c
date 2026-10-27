#include <stdbool.h>

bool safety_tx_buffered_for_fwd = false;

#include "fake_stm.h"
#include "can.h"

//int safety_tx_hook(CANPacket_t *to_send) { return 1; }

#include "faults.h"
#define safety_fwd_hook safety_fwd_hook_packet
#include "safety.h"
#undef safety_fwd_hook

// Python's legacy forwarding tests pass a bus/address rather than a packet.
int safety_fwd_hook(int bus_num, int addr) {
  CANPacket_t packet = {.bus = bus_num, .addr = addr, .data_len_code = 8U};
  return safety_fwd_hook_packet(&packet);
}

#include "drivers/can_common.h"

// libsafety stuff
#include "safety_helpers.h"
