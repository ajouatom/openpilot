#include <stdio.h>
#include <assert.h>
#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "usb/usb_host.h"
#include "soc/usb_dwc_struct.h"
#include "display_status.h"
#include "provision.h"
#include "esp_system.h"
#include <stdlib.h>
#include "network.h"

static const char *TAG = "S3_USB";
static QueueHandle_t event_queue;
static usb_host_client_handle_t client;
static usb_device_handle_t opened[128];
static int64_t last_frame;
static bool dark = true;
static bool setup_shown;
static uint8_t brightness = 80;
static uint8_t response_data[512];
static size_t response_length;
static int transfer_timeout_ms = 1000;
static bool transfer_done;
static bool last_empty_response;
static usb_device_handle_t claimed_device;
static bool streaming;
static uint8_t response_command;
static uint32_t response_stamp;

static void transfer_cb(usb_transfer_t *transfer) { transfer_done = true; }

static bool exchange(usb_device_handle_t dev, uint8_t endpoint, const uint8_t *data, int size) {
  last_empty_response = false;
  usb_transfer_t *transfer = NULL;
  if (usb_host_transfer_alloc(size, 0, &transfer) != ESP_OK) return false;
  transfer->device_handle = dev;
  transfer->bEndpointAddress = endpoint;
  transfer->callback = transfer_cb;
  transfer->num_bytes = size;
  if (data) memcpy(transfer->data_buffer, data, size);
  transfer_done = false;
  esp_err_t err = usb_host_transfer_submit(transfer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "SUBMIT ep=%02x error=%s", endpoint, esp_err_to_name(err));
    usb_host_transfer_free(transfer);
    return false;
  }
  int64_t deadline = esp_timer_get_time() + transfer_timeout_ms * 1000LL;
  while (!transfer_done && esp_timer_get_time() < deadline) {
    usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
  }
  if (!transfer_done) {
    ESP_LOGE(TAG, "TRANSFER_TIMEOUT ep=%02x", endpoint);
    usb_host_endpoint_halt(dev, endpoint);
    usb_host_endpoint_flush(dev, endpoint);
    deadline = esp_timer_get_time() + 1000000;
    while (!transfer_done && esp_timer_get_time() < deadline) {
      usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
    }
    if (transfer_done) {
      usb_host_transfer_free(transfer);
      usb_host_endpoint_clear(dev, endpoint);
    }
    else esp_restart(); // Never reuse an outstanding USB DMA transfer.
    return false;
  }
  bool ok = transfer->status == USB_TRANSFER_STATUS_COMPLETED;
  if (!(endpoint & 0x80)) ok = ok && transfer->actual_num_bytes == size;
  if (endpoint & 0x80) {
    if (!streaming) {
      ESP_LOGI(TAG, "RESPONSE status=%d bytes=%d", transfer->status, transfer->actual_num_bytes);
      ESP_LOG_BUFFER_HEX_LEVEL(TAG, transfer->data_buffer, transfer->actual_num_bytes < 32 ? transfer->actual_num_bytes : 32, ESP_LOG_INFO);
    }
    last_empty_response = ok && transfer->actual_num_bytes == 0;
    ok = ok && transfer->actual_num_bytes > 8 &&
         (transfer->data_buffer[1] == 0xc8 || transfer->data_buffer[8] == 0xc8);
    if (ok) {
      response_command = transfer->data_buffer[0];
      memcpy(&response_stamp, transfer->data_buffer + 2, 4);
      response_length = transfer->actual_num_bytes;
      memcpy(response_data, transfer->data_buffer, response_length);
    }
  } else if (!ok) {
    ESP_LOGE(TAG, "OUT status=%d actual=%d requested=%d", transfer->status, transfer->actual_num_bytes, size);
  }
  usb_host_transfer_free(transfer);
  return ok;
}

static bool read_response(usb_device_handle_t dev) {
  for (int attempt = 0; attempt < 3; attempt++) {
    if (exchange(dev, 0x81, NULL, 512)) return true;
    if (!last_empty_response) return false;
  }
  return false;
}

static bool send_payload(const uint8_t *data, size_t size) {
  for (size_t offset=0; offset<size;) {
    size_t count=size-offset; if (count>32768) count=32768;
    if (!exchange(claimed_device, 0x01, data+offset, count)) return false;
    offset+=count;
  }
  return true;
}

static void local_brightness(uint8_t value) {
  uint8_t packet[512]; display_command(packet, 14, value);
  if (send_payload(packet, sizeof(packet))) {
    transfer_timeout_ms=50; read_response(claimed_device); transfer_timeout_ms=1000;
  }
}

static void claim_display(usb_device_handle_t dev) {
  if (claimed_device) return;
  if (usb_host_interface_claim(client, dev, 0, 0) != ESP_OK) return;
  claimed_device=dev; setup_shown=false; dark=true; last_frame=0;
  uint8_t sync[512]; display_command(sync, 10, 0);
  if (send_payload(sync, sizeof(sync))) read_response(dev);
  local_brightness(0);
}

static void event_cb(const usb_host_client_event_msg_t *event, void *arg) {
  if (xQueueSend(event_queue, event, 0) != pdTRUE) {
    ESP_LOGE(TAG, "Device event queue overflow");
  }
}

static void library_task(void *arg) {
  while (1) {
    uint32_t flags = 0;
    esp_err_t err = usb_host_lib_handle_events(pdMS_TO_TICKS(1000), &flags);
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
      ESP_LOGE(TAG, "USB library error: %s", esp_err_to_name(err));
    }
  }
}

static void describe_device(uint8_t address) {
  if (address >= 128 || opened[address]) return;
  usb_device_handle_t dev = NULL;
  esp_err_t err = usb_host_device_open(client, address, &dev);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "OPEN address=%u error=%s", address, esp_err_to_name(err));
    return;
  }
  opened[address] = dev;
  const usb_device_desc_t *desc = NULL;
  const usb_config_desc_t *cfg;
  usb_device_info_t info;
  if (usb_host_get_device_descriptor(dev, &desc) == ESP_OK) {
    ESP_LOGI(TAG, "DEVICE address=%u vid=%04x pid=%04x class=%02x bcdUSB=%04x ep0=%u", address,
             desc->idVendor, desc->idProduct, desc->bDeviceClass, desc->bcdUSB, desc->bMaxPacketSize0);
    if (desc->idVendor == 0x1cbe) ESP_LOGI(TAG, "TURZX_DISPLAY_ENUMERATED");
  }
  if (usb_host_device_info(dev, &info) == ESP_OK) {
    ESP_LOGI(TAG, "LINK address=%u speed_enum=%d (0=low,1=full) configuration=%u", address,
             (int)info.speed, info.bConfigurationValue);
  }
  if (usb_host_get_active_config_descriptor(dev, &cfg) == ESP_OK) {
    ESP_LOGI(TAG, "CONFIG total=%u interfaces=%u max_power_mA=%u", cfg->wTotalLength,
             cfg->bNumInterfaces, cfg->bMaxPower * 2);
    const uint8_t *raw = (const uint8_t *)cfg;
    for (unsigned i = 0; i + 2 <= cfg->wTotalLength;) {
      unsigned n = raw[i];
      if (n < 2 || i + n > cfg->wTotalLength) break;
      if (raw[i+1] == 4 && n >= 9) {
        ESP_LOGI(TAG, "INTERFACE number=%u alt=%u endpoints=%u class=%02x subclass=%02x protocol=%02x",
                 raw[i+2], raw[i+3], raw[i+4], raw[i+5], raw[i+6], raw[i+7]);
      } else if (raw[i+1] == 5 && n >= 7) {
        ESP_LOGI(TAG, "ENDPOINT address=%02x type=%u max_packet=%u interval=%u",
                 raw[i+2], raw[i+3] & 3, raw[i+4] | (raw[i+5] << 8), raw[i+6]);
      }
      i += n;
    }
  }
  if (desc && desc->idVendor == 0x1cbe && desc->idProduct == 0x0092) claim_display(dev);
}

static void usb_task(void *arg) {
  ESP_LOGI(TAG, "Carrot S3 display bridge v1; no OTA; external powered USB hub required");
  ESP_LOGI(TAG, "External hub support=%d", CONFIG_USB_HOST_HUBS_SUPPORTED);
  event_queue = xQueueCreate(16, sizeof(usb_host_client_event_msg_t));
  assert(event_queue);
  const usb_host_config_t host_cfg = {.skip_phy_setup = false, .intr_flags = ESP_INTR_FLAG_LEVEL1};
  ESP_ERROR_CHECK(usb_host_install(&host_cfg));
  const usb_host_client_config_t client_cfg = {
    .is_synchronous = false, .max_num_event_msg = 16,
    .async = {.client_event_callback = event_cb, .callback_arg = NULL},
  };
  ESP_ERROR_CHECK(usb_host_client_register(&client_cfg, &client));
  assert(xTaskCreate(library_task, "usb_library", 4096, NULL, 5, NULL) == pdPASS);
  network_start();
  uint32_t good_frames=0, bad_frames=0;
  int64_t report_at = 0;
  while (1) {
    esp_err_t err = usb_host_client_handle_events(client, pdMS_TO_TICKS(1));
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) ESP_LOGE(TAG, "Client: %s", esp_err_to_name(err));
    usb_host_client_event_msg_t event;
    while (xQueueReceive(event_queue, &event, 0) == pdTRUE) {
      if (event.event == USB_HOST_CLIENT_EVENT_NEW_DEV) describe_device(event.new_dev.address);
      else if (event.event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
        for (int i = 1; i < 128; i++) {
          if (opened[i] == event.dev_gone.dev_hdl) {
            ESP_LOGW(TAG, "DISCONNECT address=%d", i);
            if (claimed_device == opened[i]) {
              usb_host_interface_release(client, claimed_device, 0);
              claimed_device = NULL; setup_shown=false; dark=true;
            }
            usb_host_device_close(client, opened[i]);
            opened[i] = NULL;
          }
        }
      }
    }
    char setup_ssid[32], setup_password[17];
    bool setup=provision_setup(setup_ssid, setup_password);
    if (claimed_device && setup && !setup_shown) {
      size_t length=0;
      uint8_t *image=display_setup_image(setup_ssid, setup_password, &length);
      if (image) {
        local_brightness(80);
        setup_shown=send_payload(image, length) && read_response(claimed_device);
        free(image); dark=false;
      }
    }
    if (!setup && setup_shown) {
      if (claimed_device) local_brightness(0);
      setup_shown=false; dark=true;
    }
    int slot;
    if (xQueueReceive(network_ready, &slot, 0) == pdTRUE) {
      network_frame_t *frame=&network_frames[slot];
      streaming=true;
      int64_t start=esp_timer_get_time();
      bool ok=claimed_device && network_current(frame) && !setup;
      uint8_t plain[504]; display_decode(frame->data, plain);
      uint8_t command=plain[0];
      uint32_t stamp; memcpy(&stamp, plain+4, 4);
      bool image=command==101 || command==102;
      response_length=0;
      if (ok && image && dark) local_brightness(brightness);
      if (ok) ok=send_payload(frame->data, frame->length);
      if (ok) {
        transfer_timeout_ms=frame->expect_reply ? 1000 : 50;
        bool received=read_response(claimed_device);
        transfer_timeout_ms=1000;
        // Discard late optional command replies, never mistake them for a frame ACK.
        for (int i=0; received && response_command!=command && i<3; i++) received=read_response(claimed_device);
        if (frame->expect_reply) ok=received && response_command==command && (!image || response_stamp==stamp);
        if (!received) response_length=0;
      }
      if (ok && image) { last_frame=esp_timer_get_time(); dark=false; }
      if (ok && command==14) { brightness=plain[8]; dark=brightness==0; }
      if (ok) good_frames++; else bad_frames++;
      network_complete(slot, ok ? 0 : 1, esp_timer_get_time()-start, response_data, response_length);
      streaming=false;
    }
    if (claimed_device && !dark && !setup && esp_timer_get_time()-last_frame>2000000) {
      local_brightness(0); dark=true;
    }
    int64_t now = esp_timer_get_time();
    if (now >= report_at) {
      usb_host_lib_info_t info;
      if (usb_host_lib_info(&info) == ESP_OK) ESP_LOGI(TAG, "ALIVE devices=%d clients=%d", info.num_devices, info.num_clients);
      usb_dwc_hprt_reg_t port = {.val = USB_DWC.hprt_reg.val};
      ESP_LOGI(TAG, "PORT connected=%u enabled=%u line=%u controller_power=%u raw=%08" PRIx32,
               (unsigned)port.prtconnsts, (unsigned)port.prtena,
               (unsigned)port.prtlnsts, (unsigned)port.prtpwr, port.val);
      ESP_LOGI(TAG, "STREAM_TOTAL acked=%" PRIu32 " failed=%" PRIu32, good_frames, bad_frames);
      report_at = now + 5000000;
    }
  }
}

void app_main(void) {
  assert(xTaskCreate(usb_task, "display_usb", 16384, NULL, 1, NULL) == pdPASS);
}
