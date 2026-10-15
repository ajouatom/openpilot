#include "status_led.h"
#include "status_led_policy.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static status_led_state_t state;

void status_led_wifi(bool ready, bool disconnected) {
  taskENTER_CRITICAL(&lock);
  state.wifi_ready = ready;
  if (ready) state.wifi_lost = false;
  else if (disconnected) state.wifi_lost = true;
  if (!ready) state.last_frame = 0;
  taskEXIT_CRITICAL(&lock);
}
void status_led_setup(bool active) {
  taskENTER_CRITICAL(&lock); state.setup = active; taskEXIT_CRITICAL(&lock);
}
void status_led_usb(bool present, bool error) {
  taskENTER_CRITICAL(&lock); state.usb_present = present; state.usb_error = error; taskEXIT_CRITICAL(&lock);
}
void status_led_frame(void) {
  taskENTER_CRITICAL(&lock); state.last_frame = esp_timer_get_time(); taskEXIT_CRITICAL(&lock);
}

static void led_task(void *arg) {
  rmt_channel_handle_t channel = NULL;
  rmt_encoder_handle_t encoder = NULL;
  rmt_tx_channel_config_t config = {
    .clk_src = RMT_CLK_SRC_DEFAULT, .gpio_num = CONFIG_S3_STATUS_LED_GPIO,
    .mem_block_symbols = 64, .resolution_hz = 10000000, .trans_queue_depth = 1,
  };
  rmt_copy_encoder_config_t copy = {};
  esp_err_t err = rmt_new_tx_channel(&config, &channel);
  if (err == ESP_OK) err = rmt_new_copy_encoder(&copy, &encoder);
  if (err == ESP_OK) err = rmt_enable(channel);
  if (err != ESP_OK) {
    ESP_LOGW("S3_LED", "Status LED unavailable: %s", esp_err_to_name(err));
    if (encoder) rmt_del_encoder(encoder);
    if (channel) rmt_del_channel(channel);
    vTaskDelete(NULL);
    return;
  }
  ESP_LOGI("S3_LED", "RGB status LED GPIO=%d brightness=3/255", CONFIG_S3_STATUS_LED_GPIO);
  uint32_t previous = UINT32_MAX;
  while (1) {
    taskENTER_CRITICAL(&lock); status_led_state_t snapshot = state; taskEXIT_CRITICAL(&lock);
    uint32_t rgb = status_led_color(snapshot, esp_timer_get_time());
    if (rgb != previous) {
      uint32_t grb = ((rgb & 0x00ff00) << 8) | ((rgb & 0xff0000) >> 8) | (rgb & 0xff);
      rmt_symbol_word_t symbols[25];
      for (int i = 0; i < 24; i++) {
        bool one = (grb >> (23-i)) & 1;
        symbols[i] = (rmt_symbol_word_t){.level0=1, .duration0=one ? 9 : 3, .level1=0, .duration1=one ? 3 : 9};
      }
      symbols[24] = (rmt_symbol_word_t){.level0=0, .duration0=500, .level1=0, .duration1=500};
      rmt_transmit_config_t tx = {.loop_count=0};
      err = rmt_transmit(channel, encoder, symbols, sizeof(symbols), &tx);
      if (err == ESP_OK) err = rmt_tx_wait_all_done(channel, 20);
      if (err != ESP_OK) {
        // Do not reuse the stack-backed symbols until the channel is stopped.
        rmt_disable(channel);
        ESP_LOGW("S3_LED", "Status LED stopped: %s", esp_err_to_name(err));
        rmt_del_encoder(encoder); rmt_del_channel(channel); vTaskDelete(NULL); return;
      }
      previous = rgb;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void status_led_start(void) {
  if (xTaskCreate(led_task, "status_led", 3072, NULL, 1, NULL) != pdPASS)
    ESP_LOGW("S3_LED", "Status LED task unavailable");
}
