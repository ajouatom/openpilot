#pragma once
#include <stddef.h>
#include <stdint.h>
void display_command(uint8_t out[512], uint8_t command, uint32_t value);
void display_decode(const uint8_t in[512], uint8_t out[504]);
uint8_t *display_setup_image(const char *ssid, const char *password, size_t *length);
