#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "provision.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "lwip/sockets.h"
#include "cJSON.h"
#include "status_led.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include <inttypes.h>

uint8_t pairing_key[32], device_id[6];
char device_hex[13];
typedef struct {
  uint32_t version;
  uint8_t key[32];
  char ap_password[17], ssid[33], password[64];
} registration_t;
static registration_t registration;
static SemaphoreHandle_t config_lock;
static EventGroupHandle_t wifi_events;
static nvs_handle_t storage;
static bool apply_pending, setup_requested;
static int64_t setup_until;
static int64_t apply_after;
static char ap_name[32], csrf[33];
static unsigned disconnect_count;
static unsigned last_disconnect_reason;
static esp_netif_t *sta_netif;

static void hex(const uint8_t *data, size_t length, char *out) {
  for (size_t i=0; i<length; i++) sprintf(out + 2*i, "%02x", data[i]);
}

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
  if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
    wifi_event_sta_disconnected_t *event = data;
    xEventGroupClearBits(wifi_events, 1);
    status_led_wifi(false, true);
    xSemaphoreTake(config_lock, portMAX_DELAY);
    last_disconnect_reason = event->reason;
    unsigned count = ++disconnect_count;
    xSemaphoreGive(config_lock);
    ESP_LOGW("S3", "WIFI_DISCONNECTED reason=%u rssi=%d count=%u", event->reason, event->rssi, count);
  }
  if (base == IP_EVENT && id == IP_EVENT_STA_LOST_IP) {
    xEventGroupClearBits(wifi_events, 1);
    status_led_wifi(false, true);
    ESP_LOGW("S3", "WIFI_IP_LOST");
  }
  if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = data;
    xEventGroupSetBits(wifi_events, 1);
    status_led_wifi(true, false);
    ESP_LOGI("S3", "WIFI_IP=" IPSTR, IP2STR(&event->ip_info.ip));
  }
}

static void uart_status(void) {
  wifi_ap_record_t ap = {0};
  esp_netif_ip_info_t ip = {0};
  bool associated = esp_wifi_sta_get_ap_info(&ap) == ESP_OK;
  if (sta_netif) esp_netif_get_ip_info(sta_netif, &ip);
  xSemaphoreTake(config_lock, portMAX_DELAY);
  unsigned count = disconnect_count, reason = last_disconnect_reason;
  xSemaphoreGive(config_lock);
  printf("S3JSON {\"status\":true,\"uptime_ms\":%" PRId64 ",\"reset_reason\":%d,"
         "\"associated\":%s,\"ip_ready\":%s,\"ip\":\"" IPSTR "\",\"rssi\":%d,\"channel\":%u,"
         "\"bssid\":\"%02x:%02x:%02x:%02x:%02x:%02x\","
         "\"disconnect_count\":%u,\"last_disconnect_reason\":%u,\"free_internal\":%u,\"min_internal\":%u}\n",
         esp_timer_get_time()/1000, (int)esp_reset_reason(), associated ? "true" : "false",
         xEventGroupGetBits(wifi_events)&1 ? "true" : "false", IP2STR(&ip.ip), ap.rssi, ap.primary,
         ap.bssid[0], ap.bssid[1], ap.bssid[2], ap.bssid[3], ap.bssid[4], ap.bssid[5],
         count, reason, (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
         (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
}

static bool store_wifi(cJSON *json) {
  const cJSON *ssid = cJSON_GetObjectItemCaseSensitive(json, "ssid");
  const cJSON *password = cJSON_GetObjectItemCaseSensitive(json, "password");
  if (!cJSON_IsString(ssid) || !cJSON_IsString(password)) return false;
  size_t s = strlen(ssid->valuestring), p = strlen(password->valuestring);
  if (!s || s > 32 || p < 8 || p > 63) return false;
  xSemaphoreTake(config_lock, portMAX_DELAY);
  registration_t next = registration;
  memset(next.ssid, 0, sizeof(next.ssid)); memset(next.password, 0, sizeof(next.password));
  memcpy(next.ssid, ssid->valuestring, s); memcpy(next.password, password->valuestring, p);
  bool ok = nvs_set_blob(storage, "registration", &next, sizeof(next)) == ESP_OK && nvs_commit(storage) == ESP_OK;
  if (ok) { registration = next; apply_pending = true; apply_after=esp_timer_get_time()+2000000; }
  memset(&next, 0, sizeof(next));
  xSemaphoreGive(config_lock);
  return ok;
}

static void pairing_json(char *out, size_t capacity) {
  char key[65];
  hex(pairing_key, 32, key);
  snprintf(out, capacity, "{\"version\":1,\"device\":\"%s\",\"key\":\"%s\"}", device_hex, key);
  memset(key, 0, sizeof(key));
}

bool provision_setup(char *ssid, char *password) {
  xSemaphoreTake(config_lock, portMAX_DELAY);
  bool active = setup_until > esp_timer_get_time();
  if (active && ssid && password) {
    strcpy(ssid, ap_name); strcpy(password, registration.ap_password);
  }
  xSemaphoreGive(config_lock);
  return active;
}

// Serve setup only on the physical AP interface, never on the vehicle LAN.
static bool setup_client(httpd_req_t *req) {
  struct sockaddr_storage local;
  socklen_t size = sizeof(local);
  if (!provision_setup(NULL, NULL) || getsockname(httpd_req_to_sockfd(req), (struct sockaddr *)&local, &size)) return false;
  if (local.ss_family==AF_INET) return ((struct sockaddr_in *)&local)->sin_addr.s_addr==inet_addr("192.168.4.1");
  if (local.ss_family==AF_INET6) {
    const uint8_t mapped[16]={0,0,0,0,0,0,0,0,0,0,255,255,192,168,4,1};
    return !memcmp(&((struct sockaddr_in6 *)&local)->sin6_addr, mapped, sizeof(mapped));
  }
  return false;
}

static esp_err_t portal_redirect(httpd_req_t *req, httpd_err_code_t error) {
  if (!setup_client(req)) return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Setup closed");
  httpd_resp_set_status(req, "302 Found");
  httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_sendstr(req, "Open Carrot HUD setup");
}

static void dns_task(void *arg) {
  int sock=socket(AF_INET, SOCK_DGRAM, 0);
  assert(sock>=0);
  struct sockaddr_in address={.sin_family=AF_INET,.sin_port=htons(53),.sin_addr.s_addr=inet_addr("192.168.4.1")};
  assert(bind(sock,(struct sockaddr *)&address,sizeof(address))==0);
  while (1) {
    uint8_t packet[512]; struct sockaddr_in peer; socklen_t size=sizeof(peer);
    int n=recvfrom(sock,packet,sizeof(packet),0,(struct sockaddr *)&peer,&size);
    if (n<17 || !provision_setup(NULL,NULL) || (packet[2]&0xf8) || packet[4] || packet[5]!=1) continue;
    size_t end=12;
    while (end<(size_t)n && packet[end] && packet[end]<=63 && end+packet[end]+1<(size_t)n) end+=packet[end]+1;
    if (end+5>(size_t)n || packet[end] || end+21>sizeof(packet)) continue;
    end++;
    bool answer=packet[end]==0 && packet[end+1]==1 && packet[end+2]==0 && packet[end+3]==1;
    end+=4;
    packet[2]=0x81; packet[3]=0x80; packet[6]=0; packet[7]=answer; memset(packet+8,0,4);
    if (answer) {
      const uint8_t record[]={0xc0,0x0c,0,1,0,1,0,0,0,0,0,4,192,168,4,1};
      memcpy(packet+end,record,sizeof(record)); end+=sizeof(record);
    }
    sendto(sock,packet,end,0,(struct sockaddr *)&peer,size);
  }
}

static esp_err_t setup_page(httpd_req_t *req) {
  if (!setup_client(req)) return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Setup closed");
  char page[3100];
  snprintf(page, sizeof(page),
    "<!doctype html><html lang=ko><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>Carrot S3 HUD</title><style>body{font:18px sans-serif;max-width:32em;margin:2em auto;padding:1em}"
    "input,button{font:inherit;display:block;box-sizing:border-box;width:100%%;padding:.7em;margin:.7em 0}</style>"
    "<h1>Carrot S3 HUD</h1><p>2.4GHz 핫스팟 또는 공유기 정보를 입력하세요.</p>"
    "<form id=f><label>Wi-Fi 이름<input id=s required maxlength=32 autocomplete=off></label>"
    "<label>Wi-Fi 비밀번호<input id=p type=password required minlength=8 maxlength=63 autocomplete=new-password></label>"
    "<button>Wi-Fi 저장</button></form><p id=m></p><a id=a hidden>처음 C4 연결: 등록 파일 받기</a>"
    "<p>이미 연결한 C4는 Wi-Fi만 저장하면 됩니다. 처음 연결할 때만 아래 파일을 받아 C4에 등록하세요. 파일은 다른 사람에게 공유하지 마세요.</p>"
    "<script>f.onsubmit=async e=>{e.preventDefault();try{let r=await fetch('/configure',{method:'POST',"
    "headers:{'Content-Type':'application/json','X-Setup-Token':'%s'},body:JSON.stringify({ssid:s.value,password:p.value})});"
    "if(!r.ok)throw Error();let b=await r.blob();a.href=URL.createObjectURL(b);"
    "a.onclick=()=>{a.download='carrot-s3-'+Date.now()+'.json'};a.hidden=false;"
    "p.value='';m.textContent='Wi-Fi를 저장했습니다. 기존 C4 등록 파일은 그대로 쓰세요. 폰을 핫스팟으로 되돌리면 연결됩니다. 설정 Wi-Fi는 15초 후 닫힙니다.';}"
    "catch(e){m.textContent='저장하지 못했습니다. 입력값과 연결을 확인하세요.'}}</script></html>", csrf);
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_set_hdr(req, "X-Frame-Options", "DENY");
  return httpd_resp_sendstr(req, page);
}

static esp_err_t configure_page(httpd_req_t *req) {
  char token[40];
  if (!setup_client(req) || httpd_req_get_hdr_value_str(req, "X-Setup-Token", token, sizeof(token)) != ESP_OK ||
      strcmp(token, csrf)) return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Setup closed");
  if (!req->content_len || req->content_len > 768) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid input");
  char body[769] = {0};
  size_t offset = 0;
  while (offset < req->content_len) {
    int n = httpd_req_recv(req, body + offset, req->content_len - offset);
    if (n <= 0) return ESP_FAIL;
    offset += n;
  }
  cJSON *json = cJSON_Parse(body);
  bool ok = json && store_wifi(json);
  cJSON_Delete(json); memset(body, 0, sizeof(body));
  if (!ok) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid Wi-Fi input or storage failure");
  char pairing[160];
  pairing_json(pairing, sizeof(pairing));
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=carrot-s3.json");
  esp_err_t result = httpd_resp_sendstr(req, pairing);
  xSemaphoreTake(config_lock, portMAX_DELAY); setup_until=esp_timer_get_time()+15000000; xSemaphoreGive(config_lock);
  memset(pairing, 0, sizeof(pairing));
  return result;
}

static void uart_task(void *arg) {
  char line[1024];
  size_t used = 0;
  bool overflow = false;
  while (1) {
    uint8_t c;
    if (uart_read_bytes(UART_NUM_0, &c, 1, pdMS_TO_TICKS(1000)) != 1) { used=0; overflow=false; continue; }
    if (c != '\n') {
      if (used < sizeof(line)-1) line[used++] = c; else overflow=true;
      continue;
    }
    line[used] = 0;
    if (!overflow && !strcmp(line, "S3HELLO")) printf("S3JSON {\"version\":1,\"device\":\"%s\"}\n", device_hex);
    else if (!overflow && !strcmp(line, "S3STATUS")) uart_status();
    else if (!overflow && !strncmp(line, "S3CONFIG ", 9)) {
      cJSON *json = cJSON_Parse(line + 9);
      bool ok = json && store_wifi(json);
      cJSON_Delete(json);
      if (ok) { xSemaphoreTake(config_lock, portMAX_DELAY); setup_until=0; xSemaphoreGive(config_lock); }
      printf("S3JSON %s\n", ok ? "{\"saved\":true}" : "{\"error\":true}");
    } else if (!overflow && !strcmp(line, "S3PAIR")) {
      char pairing[160]; pairing_json(pairing, sizeof(pairing));
      printf("S3JSON %s\n", pairing); memset(pairing, 0, sizeof(pairing));
    } else if (!overflow && !strcmp(line, "S3SETUP")) {
      xSemaphoreTake(config_lock, portMAX_DELAY); setup_requested=true; xSemaphoreGive(config_lock);
      printf("S3JSON {\"setup\":true}\n");
    }
    memset(line, 0, sizeof(line)); used=0; overflow=false;
  }
}

static void manager_task(void *arg) {
  int64_t button_since=0, next_connect=0;
  bool button_handled=false, ap_active=false;
  unsigned backoff=1;
  while (1) {
    int64_t now = esp_timer_get_time();
    if (!gpio_get_level(GPIO_NUM_0)) {
      if (!button_since) button_since=now;
      if (!button_handled && now-button_since>3000000) {
        xSemaphoreTake(config_lock, portMAX_DELAY); setup_requested=true; xSemaphoreGive(config_lock);
        button_handled=true;
      }
    } else { button_since=0; button_handled=false; }
    xSemaphoreTake(config_lock, portMAX_DELAY);
    if (setup_requested) { setup_until=now+600000000; setup_requested=false; }
    bool setup=setup_until>now;
    bool apply=apply_pending && now>=apply_after;
    if (apply) apply_pending=false;
    registration_t current=registration;
    xSemaphoreGive(config_lock);
    if (setup != ap_active) {
      ESP_ERROR_CHECK(esp_wifi_set_mode(setup ? WIFI_MODE_APSTA : WIFI_MODE_STA));
      ap_active=setup;
      status_led_setup(setup);
      ESP_LOGI("S3", "Phone setup %s", setup ? "opened for 10 minutes" : "closed");
    }
    if (apply) {
      esp_wifi_disconnect();
      xEventGroupClearBits(wifi_events, 1);
      wifi_config_t config={0};
      memcpy(config.sta.ssid, current.ssid, strlen(current.ssid));
      memcpy(config.sta.password, current.password, strlen(current.password));
      // Several APs can share the registered SSID. Fast scan stops at the
      // first match, which can select a weak AP before reaching a nearer one.
      config.sta.scan_method=WIFI_ALL_CHANNEL_SCAN;
      config.sta.sort_method=WIFI_CONNECT_AP_BY_SIGNAL;
      config.sta.threshold.authmode=WIFI_AUTH_WPA2_PSK;
      config.sta.pmf_cfg.capable=true;
      ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &config));
      memset(&config, 0, sizeof(config)); next_connect=now; backoff=1;
    }
    if (current.ssid[0] && !(xEventGroupGetBits(wifi_events)&1) && now>=next_connect) {
      esp_wifi_connect(); next_connect=now+backoff*1000000LL;
      if (backoff<30) backoff=backoff*2>30 ? 30 : backoff*2;
    } else if (xEventGroupGetBits(wifi_events)&1) { backoff=1; next_connect=now+1000000; }
    memset(&current, 0, sizeof(current));
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

void provision_start(void) {
  config_lock=xSemaphoreCreateMutex(); wifi_events=xEventGroupCreate();
  assert(config_lock && wifi_events);
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(nvs_open("carrot_s3", NVS_READWRITE, &storage));
  size_t size=sizeof(registration);
  esp_err_t result=nvs_get_blob(storage, "registration", &registration, &size);
  if (result == ESP_ERR_NVS_NOT_FOUND) {
    memset(&registration, 0, sizeof(registration)); registration.version=1;
    esp_fill_random(registration.key, sizeof(registration.key));
    strcpy(registration.ap_password, "12345678");
    ESP_ERROR_CHECK(nvs_set_blob(storage, "registration", &registration, sizeof(registration)));
    ESP_ERROR_CHECK(nvs_commit(storage));
  } else { ESP_ERROR_CHECK(result); assert(size==sizeof(registration) && registration.version==1); }
  memcpy(pairing_key, registration.key, 32);
  ESP_ERROR_CHECK(esp_read_mac(device_id, ESP_MAC_WIFI_STA)); hex(device_id, 6, device_hex);
  // Deliberately simple setup credentials, as requested. Normal LAN pairing stays random.
  strcpy(ap_name, "CARROT-HUD");
  strcpy(registration.ap_password, "12345678");
  uint8_t random[16]; esp_fill_random(random, 16); hex(random, 16, csrf);
  ESP_ERROR_CHECK(esp_netif_init()); ESP_ERROR_CHECK(esp_event_loop_create_default());
  sta_netif=esp_netif_create_default_wifi_sta();
  esp_netif_t *ap_netif=esp_netif_create_default_wifi_ap();
  esp_netif_dns_info_t dns={.ip.type=ESP_IPADDR_TYPE_V4,.ip.u_addr.ip4.addr=inet_addr("192.168.4.1")};
  ESP_ERROR_CHECK(esp_netif_set_dns_info(ap_netif,ESP_NETIF_DNS_MAIN,&dns));
  wifi_init_config_t init=WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&init)); ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
  wifi_config_t ap={0};
  strcpy((char *)ap.ap.ssid, ap_name); strcpy((char *)ap.ap.password, registration.ap_password);
  ap.ap.ssid_len=strlen(ap_name); ap.ap.authmode=WIFI_AUTH_WPA2_PSK; ap.ap.max_connection=2;
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
  if (!registration.ssid[0]) { setup_until=esp_timer_get_time()+600000000; setup_requested=true; }
  else ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start()); ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  apply_pending=registration.ssid[0]!=0;
  gpio_config_t button={.pin_bit_mask=1ULL<<GPIO_NUM_0, .mode=GPIO_MODE_INPUT, .pull_up_en=GPIO_PULLUP_ENABLE};
  ESP_ERROR_CHECK(gpio_config(&button));
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 2048, 0, 0, NULL, 0));
  httpd_config_t web=HTTPD_DEFAULT_CONFIG(); web.stack_size=8192; web.max_open_sockets=3; web.lru_purge_enable=true;
  httpd_handle_t server;
  ESP_ERROR_CHECK(httpd_start(&server, &web));
  httpd_uri_t page={.uri="/", .method=HTTP_GET, .handler=setup_page};
  httpd_uri_t configure={.uri="/configure", .method=HTTP_POST, .handler=configure_page};
  ESP_ERROR_CHECK(httpd_register_uri_handler(server, &page)); ESP_ERROR_CHECK(httpd_register_uri_handler(server, &configure));
  ESP_ERROR_CHECK(httpd_register_err_handler(server,HTTPD_404_NOT_FOUND,portal_redirect));
  assert(xTaskCreate(dns_task,"setup_dns",4096,NULL,3,NULL)==pdPASS);
  assert(xTaskCreate(uart_task, "provision_uart", 6144, NULL, 3, NULL)==pdPASS);
  assert(xTaskCreate(manager_task, "wifi_reconnect", 6144, NULL, 3, NULL)==pdPASS);
  ESP_LOGI("S3", "Registration ready; hold BOOT 3 seconds for phone setup");
}
