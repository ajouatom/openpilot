// Native harness for the production safety TX/forwarding hooks.
#include <stdbool.h>
#include <string.h>
#include "fake_stm.h"
#include "can.h"
void putui(uint32_t value) { (void)value; }
bool safety_tx_buffered_for_fwd = false;
#include "faults.h"
#include "safety.h"

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT
#endif

static CANPacket_t packet(int address, int bus, int length, const uint8_t *data) {
  CANPacket_t pkt = {0};
  pkt.addr = address;
  pkt.bus = bus;
  pkt.fd = 1U;
  for (unsigned int i = 0; i < 16U; i++) {
    if (dlc_to_len[i] == length) pkt.data_len_code = i;
  }
  memcpy(pkt.data, data, length);
  return pkt;
}

EXPORT void alt2_test_init(int param) {
  set_safety_hooks(SAFETY_HYUNDAI_CANFD, param);
  relay_malfunction = false;
}

EXPORT int alt2_test_tx(int address, int bus, int length, const uint8_t *data, uint32_t now) {
  timer.CNT = now;
  CANPacket_t pkt = packet(address, bus, length, data);
  safety_tx_buffered_for_fwd = false;
  bool allowed = safety_tx_hook(&pkt);
  return (allowed ? 1 : 0) | (safety_tx_buffered_for_fwd ? 2 : 0);
}

EXPORT int alt2_test_fwd(int address, int bus, int length, uint8_t *data, uint32_t now) {
  timer.CNT = now;
  CANPacket_t pkt = packet(address, bus, length, data);
  int result = safety_fwd_hook(&pkt);
  memcpy(data, pkt.data, length);
  return result;
}
