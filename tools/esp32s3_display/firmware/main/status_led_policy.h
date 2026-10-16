#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  bool wifi_ready, wifi_lost, setup, usb_present, usb_error;
  int64_t last_frame;
} status_led_state_t;

// RGB, limited to 3/255 per channel. Association/IP is distinct from frame delivery.
static inline uint32_t status_led_color(status_led_state_t s, int64_t now) {
  unsigned ms = (unsigned)(now / 1000);
  if (s.setup) return ms % 1000 < 500 ? 0x030003 : 0;
  if (!s.wifi_ready) return ms % 1000 < 500 ? (s.wifi_lost ? 0x030000 : 0x000003) : 0;
  if (!s.usb_present || s.usb_error) return ms % 1000 < 200 || (ms % 1000 >= 400 && ms % 1000 < 600) ? 0x030000 : 0;
  if (s.last_frame > 0 && now >= s.last_frame && now - s.last_frame < 2000000) return ms % 1000 < 100 ? 0 : 0x000300;
  return 0x000300;
}
