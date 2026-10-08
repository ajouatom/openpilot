// Exercise the production TX/forwarding hooks with independent timer clocks.
#include "hyundai_canfd_cluster.c"

EXPORT void fwd_timer_set_tick(uint32_t tick) {
  safety_mode_cnt = tick;
}
