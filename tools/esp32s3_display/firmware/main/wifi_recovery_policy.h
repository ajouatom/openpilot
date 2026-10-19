#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef enum { WIFI_WAIT, WIFI_CONNECT, WIFI_REJOIN } wifi_action_t;
typedef struct {
  int64_t next_attempt, waiting_since;
  unsigned backoff, phase;
} wifi_recovery_t;

// One owner (the Wi-Fi manager) calls this with snapshots of event state.
// A successful association must not leave us calling connect() forever while
// DHCP has no address. Neither scan/authentication nor DHCP waits are unbounded.
static inline wifi_action_t wifi_recovery_poll(wifi_recovery_t *s, int64_t now,
                                               bool ready, bool associated, bool joining) {
  if (ready) {
    *s = (wifi_recovery_t){.next_attempt=now+1000000, .backoff=1};
    return WIFI_WAIT;
  }
  unsigned phase = associated ? 2 : joining ? 1 : 0;
  if (phase != s->phase) { s->phase=phase; s->waiting_since=now; }
  if (phase) {
    int64_t limit = associated ? 20000000 : 30000000;
    if (now-s->waiting_since < limit) return WIFI_WAIT;
    *s = (wifi_recovery_t){.next_attempt=now+1000000, .backoff=1};
    return WIFI_REJOIN;
  }
  if (now < s->next_attempt) return WIFI_WAIT;
  if (!s->backoff) s->backoff=1;
  s->next_attempt=now+s->backoff*1000000LL;
  s->backoff=s->backoff < 4 ? s->backoff*2 : 5;
  return WIFI_CONNECT;
}
