#include <assert.h>
#include <string.h>
#include <unistd.h>
#include "network.h"
#include "provision.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "mbedtls/md.h"
#include "lwip/sockets.h"
#include "lwip/tcp.h"

network_frame_t network_frames[FRAME_SLOTS];
QueueHandle_t network_free, network_ready;
static SemaphoreHandle_t socket_lock;
static int active_socket = -1;
static uint32_t session_id;
static uint8_t active_key[32];

void digest(const uint8_t key[32], const void *a, size_t alen, const void *b, size_t blen, uint8_t out[32]) {
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  assert(mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1) == 0);
  assert(mbedtls_md_hmac_starts(&ctx, key, 32) == 0);
  assert(mbedtls_md_hmac_update(&ctx, a, alen) == 0);
  if (blen) assert(mbedtls_md_hmac_update(&ctx, b, blen) == 0);
  assert(mbedtls_md_hmac_finish(&ctx, out) == 0);
  mbedtls_md_free(&ctx);
}

bool secure_equal(const uint8_t *a, const uint8_t *b, size_t length) {
  unsigned difference = 0;
  for (size_t i = 0; i < length; i++) difference |= a[i] ^ b[i];
  return difference == 0;
}

static bool receive_all(int sock, void *buffer, size_t count) {
  uint8_t *p = buffer;
  while (count) {
    int n = recv(sock, p, count, 0);
    if (n <= 0) return false;
    p += n; count -= n;
  }
  return true;
}

static bool send_all(int sock, const void *buffer, size_t count) {
  const uint8_t *p = buffer;
  while (count) {
    int n = send(sock, p, count, 0);
    if (n <= 0) return false;
    p += n; count -= n;
  }
  return true;
}

bool network_current(const network_frame_t *frame) {
  xSemaphoreTake(socket_lock, portMAX_DELAY);
  bool current = active_socket >= 0 && frame->session == session_id;
  xSemaphoreGive(socket_lock);
  return current && esp_timer_get_time() - frame->received < 500000;
}

void network_complete(int slot, uint32_t status, uint32_t usb_us, const uint8_t *response, size_t length) {
  network_frame_t *frame = &network_frames[slot];
  uint32_t reply[6] = {htonl(0x53334131), htonl(frame->sequence), htonl(length), htonl(status), htonl(usb_us), 0};
  uint8_t signature[32];
  xSemaphoreTake(socket_lock, portMAX_DELAY);
  if (active_socket >= 0 && frame->session == session_id) {
    digest(active_key, reply, sizeof(reply), response, length, signature);
    if (!send_all(active_socket, reply, sizeof(reply)) || !send_all(active_socket, signature, 32) ||
        !send_all(active_socket, response, length)) shutdown(active_socket, SHUT_RDWR);
  }
  xSemaphoreGive(socket_lock);
  xQueueSend(network_free, &slot, portMAX_DELAY);
}

static int listener(int type) {
  int sock = socket(AF_INET, type, IPPROTO_IP), yes = 1;
  assert(sock >= 0);
  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
  struct sockaddr_in address = {.sin_family=AF_INET, .sin_port=htons(3333), .sin_addr.s_addr=htonl(INADDR_ANY)};
  assert(bind(sock, (struct sockaddr *)&address, sizeof(address)) == 0);
  return sock;
}

static void discovery_task(void *arg) {
  int sock = listener(SOCK_DGRAM);
  uint8_t packet[128], signature[32];
  while (1) {
    struct sockaddr_in peer;
    socklen_t size = sizeof(peer);
    int n = recvfrom(sock, packet, sizeof(packet), 0, (struct sockaddr *)&peer, &size);
    if (n != 58 || memcmp(packet, "S3D1", 4) || memcmp(packet + 4, device_id, 6)) continue;
    digest(pairing_key, packet, 26, NULL, 0, signature);
    if (!secure_equal(signature, packet + 26, 32)) continue;
    memcpy(packet, "S3R1", 4);
    digest(pairing_key, packet, 26, NULL, 0, packet + 26);
    sendto(sock, packet, 58, 0, (struct sockaddr *)&peer, size);
  }
}

static bool authenticate(int sock, uint8_t session_key[32]) {
  uint8_t hello[42], response[64], signature[32], transcript[80];
  memcpy(hello, "S3H1", 4); memcpy(hello + 4, device_id, 6);
  esp_fill_random(hello + 10, 32);
  if (!send_all(sock, hello, sizeof(hello)) || !receive_all(sock, response, sizeof(response))) return false;
  memcpy(transcript, "client", 6); memcpy(transcript + 6, hello, 42); memcpy(transcript + 48, response, 32);
  digest(pairing_key, transcript, 80, NULL, 0, signature);
  if (!secure_equal(signature, response + 32, 32)) return false;
  memcpy(transcript, "server", 6);
  digest(pairing_key, transcript, 80, NULL, 0, signature);
  if (!send_all(sock, signature, 32)) return false;
  // "session" is seven bytes; hash prefix and the 74-byte transcript separately.
  digest(pairing_key, "session", 7, transcript + 6, 74, session_key);
  return true;
}

static void server_task(void *arg) {
  int server = listener(SOCK_STREAM);
  assert(listen(server, 1) == 0);
  while (1) {
    int sock = accept(server, NULL, NULL);
    if (sock < 0) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
    int yes = 1;
    struct timeval timeout = {.tv_sec=2};
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
    uint8_t key[32];
    if (!authenticate(sock, key)) { close(sock); vTaskDelay(pdMS_TO_TICKS(100)); continue; }
    xSemaphoreTake(socket_lock, portMAX_DELAY);
    active_socket = sock;
    uint32_t session = ++session_id;
    memcpy(active_key, key, 32);
    xSemaphoreGive(socket_lock);
    ESP_LOGI("S3", "Authenticated display connection");
    uint32_t previous_sequence = 0;
    while (1) {
      uint32_t header[6];
      uint8_t signature[32], expected[32];
      if (!receive_all(sock, header, sizeof(header)) || !receive_all(sock, signature, 32)) break;
      uint32_t length = ntohl(header[2]), sequence = ntohl(header[1]);
      if (ntohl(header[0]) != 0x53335531 || length < 512 || length > FRAME_CAPACITY ||
          sequence != previous_sequence + 1 || ntohl(header[3]) > 1 || header[4] || header[5]) break;
      int slot;
      if (xQueueReceive(network_free, &slot, pdMS_TO_TICKS(2000)) != pdTRUE) break;
      network_frame_t *frame = &network_frames[slot];
      frame->length=length; frame->sequence=sequence; frame->session=session; frame->expect_reply=ntohl(header[3]);
      bool ok = receive_all(sock, frame->data, length);
      if (ok) {
        digest(key, header, sizeof(header), frame->data, length, expected);
        ok = secure_equal(signature, expected, 32) && frame->data[510] == 0xa1 && frame->data[511] == 0x1a;
      }
      if (!ok) { xQueueSend(network_free, &slot, portMAX_DELAY); break; }
      frame->received = esp_timer_get_time();
      previous_sequence = sequence;
      xQueueSend(network_ready, &slot, portMAX_DELAY);
    }
    xSemaphoreTake(socket_lock, portMAX_DELAY);
    active_socket = -1;
    memset(active_key, 0, sizeof(active_key));
    shutdown(sock, SHUT_RDWR); close(sock);
    xSemaphoreGive(socket_lock);
    memset(key, 0, sizeof(key));
    ESP_LOGI("S3", "Display connection closed; waiting for reconnect");
  }
}

void network_start(void) {
  network_free = xQueueCreate(FRAME_SLOTS, sizeof(int));
  network_ready = xQueueCreate(FRAME_SLOTS, sizeof(int));
  socket_lock = xSemaphoreCreateMutex();
  assert(network_free && network_ready && socket_lock);
  for (int i=0; i<FRAME_SLOTS; i++) {
    network_frames[i].data = heap_caps_malloc(FRAME_CAPACITY, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(network_frames[i].data);
    xQueueSend(network_free, &i, portMAX_DELAY);
  }
  provision_start();
  assert(xTaskCreate(discovery_task, "discovery", 4096, NULL, 3, NULL) == pdPASS);
  assert(xTaskCreate(server_task, "display_tcp", 8192, NULL, 4, NULL) == pdPASS);
}
