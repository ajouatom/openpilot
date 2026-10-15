#pragma once
#include <stdbool.h>
#include <stdint.h>

void status_led_start(void);
void status_led_wifi(bool ready, bool disconnected);
void status_led_setup(bool active);
void status_led_usb(bool present, bool error);
void status_led_frame(void);
