#include <stdbool.h>

#include "fake_stm.h"
#include "can.h"

//int safety_tx_hook(CANPacket_t *to_send) { return 1; }

#include "faults.h"
#include "safety.h"
#include "drivers/can_common.h"

// Defined by panda's board/drivers/can_common.h; the CAN-FD forwarding
// buffer flag the Hyundai safety sets. Not in this copy of can_common.h.
bool safety_tx_buffered_for_fwd = false;

// libsafety stuff
#include "safety_helpers.h"
