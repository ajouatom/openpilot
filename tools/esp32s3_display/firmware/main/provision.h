#pragma once
#include <stdbool.h>
#include <stdint.h>
extern uint8_t pairing_key[32], device_id[6];
extern char device_hex[13];
void provision_start(void);
bool provision_setup(char *ssid, char *password);
