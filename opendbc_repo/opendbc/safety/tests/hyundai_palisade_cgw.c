// Host test of the actual production safety hooks. Build from the repository root:
// gcc -O2 -Ipanda/board -Ipanda -Iopendbc_repo/opendbc/safety \
//   opendbc_repo/opendbc/safety/tests/hyundai_palisade_cgw.c -o /tmp/palisade-cgw && /tmp/palisade-cgw
#include <assert.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "fake_stm.h"
#include "can.h"
bool safety_tx_buffered_for_fwd = false;
#include "faults.h"
#include "safety.h"

static unsigned int checks = 0;
#define CHECK(expr) do { checks++; assert(expr); } while (0)

static CANPacket_t msg(int addr, uint64_t data) {
  CANPacket_t p = {0};
  p.addr = addr;
  p.data_len_code = 8;
  for (int i = 0; i < 8; i++) p.data[i] = (data >> (8 * i)) & 0xFFU;
  return p;
}

static void init(int param) {
  CHECK(set_safety_hooks(SAFETY_HYUNDAI, param) == 0);
  controls_allowed = false;
  relay_malfunction = false;
}

static CANPacket_t wheels(int wheel, int speed, int counter) {
  CANPacket_t p = msg(0x386, 0);
  if (wheel >= 0) {
    p.data[2 * wheel] = speed & 0xFF;
    p.data[2 * wheel + 1] = (speed >> 8) & 0x3F;
  }
  p.data[1] |= (counter & 3) << 6;
  p.data[3] |= ((counter >> 2) & 3) << 6;
  uint8_t crc = hyundai_compute_checksum(&p);
  p.data[5] |= (crc & 3) << 6;
  p.data[7] |= (crc >> 2) << 6;
  return p;
}

static void ready(uint32_t now, uint64_t stock, int counter) {
  timer.CNT = now;
  CANPacket_t c = msg(0x541, stock), g = msg(0x367, 0), w = wheels(-1, 0, counter);
  CHECK(safety_rx_hook(&c));
  CHECK(safety_rx_hook(&g));
  CHECK(safety_rx_hook(&w));
}

static bool tx(uint32_t now, uint64_t data) {
  timer.CNT = now;
  CANPacket_t p = msg(0x541, data);
  return safety_tx_hook(&p);
}

int main(void) {
  // lamp bit + lever side in byte 5 (2 left, 1 right), as the stock CGW1 sends while flashing
  const uint64_t left = (1ULL << 19) | (2ULL << 41), right = (1ULL << 62) | (1ULL << 41);
  const uint64_t mask = (3ULL << 19) | (3ULL << 62) | (3ULL << 41);
  const uint64_t stock = 0x123456789ABCDEF0ULL & ~(mask | (3ULL << 33) | (3ULL << 41));
  // Flag, all classic config variants, first-frame at timestamp zero, and both directions.
  for (int param = 0; param < 16; param++) {
    init(param); ready(0, stock, 0); CHECK(!tx(0, stock | left));
    init(param | 4096); ready(0, stock, 0); CHECK(tx(0, stock | left));
    CHECK(!tx(49999, stock | right)); CHECK(tx(50000, stock | right));
  }
  // the off half of a flash (lever only, lamp 0) is allowed; the other side's lamp or a lamp without lever is not
  for (int l = 0; l < 4; l++) for (int r = 0; r < 4; r++) for (int s = 0; s < 4; s++) {
    init(4096); ready(0, stock, 0);
    CHECK(tx(0, stock | ((uint64_t)l << 19) | ((uint64_t)r << 62) | ((uint64_t)s << 41)) ==
          (((s == 2) && (l <= 1) && (r == 0)) || ((s == 1) && (l == 0) && (r <= 1))));
  }
  // Every unrelated bit must match, including fields currently zero and currently one.
  for (int bit = 0; bit < 64; bit++) if (!(mask & (1ULL << bit))) {
    init(4096); ready(0, stock, 0); CHECK(!tx(0, (stock | left) ^ (1ULL << bit)));
  }
  for (int field = 0; field < 3; field++) for (int value = 1; value < 4; value++) {
    init(4096);
    uint64_t source = stock | ((uint64_t)value << (field == 0 ? 19 : (field == 1 ? 62 : 33)));
    ready(0, source, 0);
    CHECK(!tx(0, (source & ~mask) | left));
  }
  for (int gear = 1; gear < 16; gear++) {
    init(4096); ready(0, stock, 0);
    CANPacket_t g = msg(0x367, (uint64_t)gear << 32);
    CHECK(safety_rx_hook(&g)); CHECK(!tx(0, stock | left));
  }
  for (int wheel = 0; wheel < 4; wheel++) {
    init(4096); ready(0, stock, 0);
    CANPacket_t w = wheels(wheel, 1, 1);
    CHECK(safety_rx_hook(&w)); CHECK(!tx(0, stock | left));
  }
  // Missing, stale, wrong-bus and malformed RX for each required input.
  const int addrs[] = {0x541, 0x367, 0x386};
  for (int missing = 0; missing < 3; missing++) for (int kind = 0; kind < 6; kind++) {
    init(4096);
    for (int i = 0; i < 3; i++) {
      CANPacket_t p = i == 2 ? wheels(-1, 0, 0) : msg(addrs[i], i == 0 ? stock : 0);
      timer.CNT = 250001;
      if (i == missing) {
        if (kind == 0) continue;
        if (kind == 1) timer.CNT = 0;
        if (kind == 2) p.bus = 2;
        if (kind == 3) p.data_len_code = 7;
        if (kind == 4) p.extended = 1;
        if (kind == 5) p.fd = 1;
      }
      safety_rx_hook(&p);
    }
    CHECK(!tx(250001, stock | left));
  }
  init(4096); ready(0, stock, 0); CHECK(tx(250000, stock | left));
  init(4096); ready(0, stock, 0); CHECK(!tx(250001, stock | left));
  // Latest template replaces the previous one.
  init(4096); ready(0, stock, 0); ready(1, stock ^ 1ULL, 1);
  CHECK(!tx(1, stock | left)); CHECK(tx(1, (stock ^ 1ULL) | left));
  for (int kind = 0; kind < 7; kind++) {
    init(4096); ready(0, stock, 0);
    CANPacket_t p = msg(0x541, stock | left);
    if (kind == 0) p.bus = 1;
    if (kind == 1) p.bus = 2;
    if (kind == 2) p.data_len_code = 7;
    if (kind == 3) p.extended = 1;
    if (kind == 4) p.fd = 1;
    if (kind == 5) controls_allowed = true;
    if (kind == 6) relay_malfunction = true;
    CHECK(!safety_tx_hook(&p));
  }
  // Exactly 60 accepted 20-Hz frames, then latched cutoff while requests continue.
  for (int wrap = 0; wrap < 2; wrap++) {
    init(4096);
    uint32_t start = wrap ? 0xFFFF0000U : 0U;
    for (int i = 0; i <= 100; i++) {
      uint32_t now = start + i * 50000U;
      ready(now, stock, i % 16);
      CHECK(tx(now, stock | left) == (i < 60));
    }
    ready(start + 5999999U, stock, 5); CHECK(!tx(start + 5999999U, stock | left));
    ready(start + 6999999U, stock, 6); CHECK(tx(start + 6999999U, stock | right));
    init(4096); ready(0, stock, 0); CHECK(tx(0, stock | left));
  }
  // Bad checksums cannot supply a wheel sample.
  init(4096);
  CANPacket_t c = msg(0x541, stock), g = msg(0x367, 0), w = wheels(-1, 0, 0);
  timer.CNT = 0; CHECK(safety_rx_hook(&c)); CHECK(safety_rx_hook(&g));
  w.data[5] ^= 0x40U; CHECK(!safety_rx_hook(&w)); CHECK(!tx(0, stock | left));
  for (int switches = 1; switches < 4; switches++) {
    init(4096); ready(0, stock | ((uint64_t)switches << 41), 0);
    CHECK(!tx(0, stock | ((uint64_t)switches << 41) | left));
  }
  // Init clears cached RX and rate/trial state, also after switching safety modes.
  init(4096); ready(0, stock, 0); CHECK(tx(0, stock | left));
  init(4096); CHECK(!tx(0, stock | left)); ready(0, stock, 0); CHECK(tx(0, stock | left));
  init(0); ready(0, stock, 0); CHECK(!tx(0, stock | left));
  CHECK(set_safety_hooks(SAFETY_HYUNDAI_LEGACY, 4096) == 0);
  CHECK(!tx(0, stock | left));
  // CGW forwarding is unchanged even immediately after accepted host TX.
  init(4096); ready(0, stock, 0); CHECK(tx(0, stock | left));
  for (int bus = 0; bus < 3; bus++) {
    CANPacket_t p = msg(0x541, stock); p.bus = bus;
    CHECK(safety_fwd_hook(&p) == (bus == 0 ? 2 : (bus == 2 ? 0 : -1)));
    CHECK(memcmp(p.data, c.data, 8) == 0);
  }
  printf("Palisade CGW production safety: %u checks passed\n", checks);
  return 0;
}
