#pragma once

// Follow stock RX rather than host scheduling. Latest-value storage avoids
// FIFO backlog when vehicle and host rates differ. This bounds HOST freshness,
// not the age of the source snapshot used by the host.
#define HYUNDAI_CANFD_CLUSTER_MAX_AGE_US 150000U

typedef struct {
  int addr;
  unsigned int len;
  bool valid;
  uint32_t updated_us;
  uint8_t data[32];
} HyundaiCanfdCluster;

static HyundaiCanfdCluster hyundai_canfd_cluster[] = {
  {.addr = 0x161, .len = 32U},
  {.addr = 0x162, .len = 32U},
  {.addr = 0x1E0, .len = 16U},
  {.addr = 0x1EA, .len = 32U},
  {.addr = 0x200, .len = 8U},
};

static HyundaiCanfdCluster* hyundai_canfd_cluster_find(int addr) {
  for (unsigned int i = 0U; i < sizeof(hyundai_canfd_cluster) / sizeof(hyundai_canfd_cluster[0]); i++) {
    if (hyundai_canfd_cluster[i].addr == addr) {
      return &hyundai_canfd_cluster[i];
    }
  }
  return NULL;
}

static void hyundai_canfd_cluster_reset(void) {
  for (unsigned int i = 0U; i < sizeof(hyundai_canfd_cluster) / sizeof(hyundai_canfd_cluster[0]); i++) {
    hyundai_canfd_cluster[i].valid = false;
    hyundai_canfd_cluster[i].updated_us = 0U;
  }
}

static bool hyundai_canfd_cluster_valid(const HyundaiCanfdCluster *st, const CANPacket_t *pkt) {
  return (GET_LEN(pkt) == st->len) && (pkt->extended == 0U) &&
         (hyundai_canfd_get_checksum(pkt) == hyundai_common_canfd_compute_checksum(pkt));
}

static bool hyundai_canfd_cluster_store(HyundaiCanfdCluster *st, const CANPacket_t *pkt, uint32_t now) {
  if (!hyundai_canfd_cluster_valid(st, pkt)) {
    st->valid = false;
    return false;
  }
  for (unsigned int i = 0U; i < st->len; i++) {
    st->data[i] = pkt->data[i];
  }
  st->updated_us = now;
  st->valid = true;
  return true;
}

static void hyundai_canfd_cluster_forward(HyundaiCanfdCluster *st, CANPacket_t *pkt, uint32_t now) {
  if (!st->valid) {
    return;
  }
  if (((now - st->updated_us) >= HYUNDAI_CANFD_CLUSTER_MAX_AGE_US) ||
      !hyundai_canfd_cluster_valid(st, pkt)) {
    // Never repair an invalid original using a cached body. Stock fallback
    // does not change its CRC or any other byte; rearm on a new host update.
    st->valid = false;
    return;
  }
  // All five DBC layouts have COUNTER at byte 2, including 8-byte 0x200.
  // The generic helper instead treats 8-byte packets as button messages.
  const uint8_t counter = pkt->data[2];
  for (unsigned int i = 0U; i < st->len; i++) {
    pkt->data[i] = st->data[i];
  }
  pkt->data[2] = counter;
  hyundai_canfd_update_checksum(pkt);
}
