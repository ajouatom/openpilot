// cc -std=c11 -Wall -Wextra -Werror test_status_led.c -o test_status_led && ./test_status_led
#include <assert.h>
#include "../firmware/main/status_led_policy.h"

int main(void) {
  status_led_state_t s = {0};
  assert(status_led_color(s, 100000) == 0x000003);
  assert(status_led_color(s, 600000) == 0);
  s.wifi_lost = true;
  assert(status_led_color(s, 100000) == 0x030000);
  s.wifi_ready = true;
  assert(status_led_color(s, 100000) == 0x030000); // USB absent
  s.usb_present = true;
  assert(status_led_color(s, 100000) == 0x000300);
  s.last_frame = 1000000;
  assert(status_led_color(s, 1050000) == 0);
  assert(status_led_color(s, 1200000) == 0x000300);
  assert(status_led_color(s, 3050000) == 0x000300); // stale frame is not streaming
  s.usb_error = true;
  assert(status_led_color(s, 1200000) == 0);
  assert(status_led_color(s, 1450000) == 0x030000);
  s.setup = true;
  assert(status_led_color(s, 1100000) == 0x030003);
  assert(status_led_color(s, 1600000) == 0);
  // No state may accidentally emit full-brightness RGB values.
  for (unsigned flags = 0; flags < 32; flags++) {
    s = (status_led_state_t){flags&1, flags&2, flags&4, flags&8, flags&16, 1000000};
    for (int64_t t = 0; t < 4000000; t += 10000) {
      uint32_t color = status_led_color(s, t);
      assert((color >> 16) <= 3 && ((color >> 8) & 255) <= 3 && (color & 255) <= 3);
    }
  }
  return 0;
}
