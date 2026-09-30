// Reuse the production-hook harness, with format and relay fault injection.
#include "hyundai_canfd_alt_buttons.c"

EXPORT int cluster_test_packet(int address, int bus, int length, uint8_t *data, uint32_t now,
                               bool tx, bool extended, bool relay_fault) {
  timer.CNT = now;
  relay_malfunction = relay_fault;
  CANPacket_t pkt = packet(address, bus, length, data);
  pkt.extended = extended;
  safety_tx_buffered_for_fwd = false;
  int result;
  if (tx) {
    bool allowed = safety_tx_hook(&pkt);
    result = (allowed ? 1 : 0) | (safety_tx_buffered_for_fwd ? 2 : 0);
  } else {
    result = safety_fwd_hook(&pkt);
  }
  memcpy(data, pkt.data, length);
  return result;
}
