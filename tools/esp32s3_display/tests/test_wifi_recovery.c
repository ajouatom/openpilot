#include <assert.h>
#include "../firmware/main/wifi_recovery_policy.h"

int main(void) {
  wifi_recovery_t s={0};
  assert(wifi_recovery_poll(&s,0,false,false,false)==WIFI_CONNECT);
  assert(wifi_recovery_poll(&s,250000,false,false,true)==WIFI_WAIT);
  // No repeated connect() while an all-channel scan/auth attempt is active.
  assert(wifi_recovery_poll(&s,30000000,false,false,true)==WIFI_WAIT);
  assert(wifi_recovery_poll(&s,30250000,false,false,true)==WIFI_REJOIN);
  assert(wifi_recovery_poll(&s,31000000,false,false,false)==WIFI_WAIT);
  assert(wifi_recovery_poll(&s,31250000,false,false,false)==WIFI_CONNECT);
  // IP acquisition gets its own deadline after association.
  assert(wifi_recovery_poll(&s,32000000,false,true,false)==WIFI_WAIT);
  assert(wifi_recovery_poll(&s,51999999,false,true,false)==WIFI_WAIT);
  assert(wifi_recovery_poll(&s,52000000,false,true,false)==WIFI_REJOIN);
  assert(wifi_recovery_poll(&s,53000000,false,false,false)==WIFI_CONNECT);
  assert(wifi_recovery_poll(&s,54000000,true,true,false)==WIFI_WAIT);
  // Ready operation, a lost IP on the same AP, and recovery without a reset.
  assert(wifi_recovery_poll(&s,100000000,true,true,false)==WIFI_WAIT);
  assert(wifi_recovery_poll(&s,101000000,false,true,false)==WIFI_WAIT);
  assert(wifi_recovery_poll(&s,121000000,false,true,false)==WIFI_REJOIN);
  // Repeated AP-not-found outcomes back off, but never exceed five seconds.
  s=(wifi_recovery_t){0};
  int64_t now=0;
  for (unsigned i=0;i<1000;i++) {
    assert(wifi_recovery_poll(&s,now,false,false,false)==WIFI_CONNECT);
    assert(s.next_attempt>now && s.next_attempt-now<=5000000);
    now=s.next_attempt;
  }
  // A long healthy connection cannot inherit an old timeout on later loss.
  assert(wifi_recovery_poll(&s,now,true,true,false)==WIFI_WAIT);
  assert(wifi_recovery_poll(&s,now+1000000,false,false,false)==WIFI_CONNECT);
  assert(wifi_recovery_poll(&s,now+1250000,false,false,true)==WIFI_WAIT);
  assert(wifi_recovery_poll(&s,now+2000000,false,true,false)==WIFI_WAIT);
  assert(wifi_recovery_poll(&s,now+21000000,false,true,false)==WIFI_WAIT);
  assert(wifi_recovery_poll(&s,now+22000000,false,true,false)==WIFI_REJOIN);
  return 0;
}
