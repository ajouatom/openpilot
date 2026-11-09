// PC-only adapter around the complete production safety hooks.
#include "libsafety/safety.c"

int canfd_test_forward(CANPacket_t *pkt) { return safety_fwd_hook_packet(pkt); }
void canfd_test_set_tick(uint32_t value) { safety_mode_cnt = value; }
int canfd_test_queue_count(int addr, int bus) {
  CanfdBufferedFwd *queue = canfd_bfwd_find(addr, bus);
  return queue == NULL ? -1 : queue->count;
}
bool canfd_test_inputs_valid(void) { return hyundai_canfd_inputs_valid(); }
