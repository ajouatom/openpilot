#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#define FRAME_CAPACITY (1024 * 1024)
#define FRAME_SLOTS 3
typedef struct {
  uint8_t *data;
  uint32_t length, sequence, session;
  bool expect_reply;
  int64_t received;
} network_frame_t;
extern network_frame_t network_frames[FRAME_SLOTS];
extern QueueHandle_t network_free, network_ready;
void network_start(void);
bool network_current(const network_frame_t *frame);
void network_complete(int slot, uint32_t status, uint32_t usb_us, const uint8_t *response, size_t length);
void digest(const uint8_t key[32], const void *a, size_t alen, const void *b, size_t blen, uint8_t out[32]);
bool secure_equal(const uint8_t *a, const uint8_t *b, size_t length);
