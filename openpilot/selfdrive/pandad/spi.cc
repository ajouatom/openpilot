#include <sys/file.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include <algorithm>
#include <cassert>
#include <cinttypes>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <iomanip>
#include <mutex>
#include <sstream>

#include "common/util.h"
#include "common/timing.h"
#include "common/swaglog.h"
#include "panda/board/comms_definitions.h"
#include "selfdrive/pandad/panda_comms.h"


#define SPI_SYNC 0x5AU
#define SPI_HACK 0x79U
#define SPI_DACK 0x85U
#define SPI_NACK 0x1FU
#define SPI_CHECKSUM_START 0xABU


enum SpiError {
  NACK = -2,
  ACK_TIMEOUT = -3,
};

enum class SpiFailurePhase : uint8_t {
  NONE,
  HEADER_IO,
  HACK_NACK,
  HACK_TIMEOUT,
  DATA_IO,
  DACK_NACK,
  DACK_TIMEOUT,
  RX_LENGTH,
  RX_IO,
  RX_CHECKSUM,
};

static const char *spi_failure_phase_name(SpiFailurePhase phase) {
  switch (phase) {
    case SpiFailurePhase::NONE: return "none";
    case SpiFailurePhase::HEADER_IO: return "header_io";
    case SpiFailurePhase::HACK_NACK: return "hack_nack";
    case SpiFailurePhase::HACK_TIMEOUT: return "hack_timeout";
    case SpiFailurePhase::DATA_IO: return "data_io";
    case SpiFailurePhase::DACK_NACK: return "dack_nack";
    case SpiFailurePhase::DACK_TIMEOUT: return "dack_timeout";
    case SpiFailurePhase::RX_LENGTH: return "rx_length";
    case SpiFailurePhase::RX_IO: return "rx_io";
    case SpiFailurePhase::RX_CHECKSUM: return "rx_checksum";
  }
  return "unknown";
}

const unsigned int SPI_ACK_TIMEOUT = 500; // milliseconds
const std::string SPI_DEVICE = "/dev/spidev0.0";
// TODO: fix SPI turnaround synchronization at the protocol level.
constexpr uint64_t SPI_TURNAROUND_NS = 400000ULL;
constexpr uint64_t SPI_SAFETY_MODE_TURNAROUND_NS = 1000000ULL;
static uint64_t spi_last_bus_activity_ns = 0;       // protected by hw_lock
static uint64_t spi_inter_transaction_ns = SPI_TURNAROUND_NS;  // protected by hw_lock

static void wait_for_spi_turnaround(uint64_t start_ns, uint64_t turnaround_ns = SPI_TURNAROUND_NS) {
  while ((nanos_since_boot() - start_ns) < turnaround_ns) {}
}

struct SpiAttemptTiming {
  uint64_t lock_us = 0U;
  uint64_t turnaround_us = 0U;
  uint64_t hack_us = 0U;
  uint64_t dack_us = 0U;
  uint64_t recovery_us = 0U;
  uint64_t total_us = 0U;
  uint32_t recovery_restarts = 0U;
  SpiFailurePhase failure_phase = SpiFailurePhase::NONE;
};

struct SpiPhaseDiagStats {
  uint32_t count = 0U;
  uint32_t slow_count = 0U;
  uint32_t retry_count = 0U;
  uint32_t nack_count = 0U;
  uint32_t hack_nack_count = 0U;
  uint32_t dack_nack_count = 0U;
  uint32_t ack_timeout_count = 0U;
  uint32_t host_checksum_count = 0U;
  uint32_t other_failure_count = 0U;
  uint32_t recovery_count = 0U;
  uint32_t recovery_restart_count = 0U;
  uint32_t max_attempts = 0U;
  uint64_t total_sum_us = 0U;
  uint64_t total_max_us = 0U;
  uint64_t lock_max_us = 0U;
  uint64_t turnaround_max_us = 0U;
  uint64_t hack_max_us = 0U;
  uint64_t dack_max_us = 0U;
  uint64_t recovery_max_us = 0U;
};

static thread_local SpiAttemptTiming spi_attempt_timing;
static std::mutex spi_phase_diag_lock;
static SpiPhaseDiagStats spi_phase_diag[2];

static int spi_phase_diag_index(uint8_t endpoint) {
  if (endpoint == 0x03U) return 0;
  if (endpoint == 0x81U) return 1;
  return -1;
}

static void record_spi_phase_diag(uint8_t endpoint, uint64_t total_us, uint32_t attempts,
                                  uint32_t nacks, uint32_t hack_nacks, uint32_t dack_nacks,
                                  uint32_t ack_timeouts, uint32_t host_checksums,
                                  uint32_t other_failures, uint32_t recoveries,
                                  uint32_t recovery_restarts,
                                  uint64_t lock_max_us, uint64_t turnaround_max_us,
                                  uint64_t hack_max_us, uint64_t dack_max_us,
                                  uint64_t recovery_max_us) {
  int idx = spi_phase_diag_index(endpoint);
  if (idx < 0) return;

  bool emit = false;
  SpiPhaseDiagStats snapshot = {};
  {
    std::lock_guard lk(spi_phase_diag_lock);
    SpiPhaseDiagStats &st = spi_phase_diag[idx];
    st.count++;
    st.slow_count += total_us > 5000U;
    st.retry_count += attempts > 1U;
    st.nack_count += nacks;
    st.hack_nack_count += hack_nacks;
    st.dack_nack_count += dack_nacks;
    st.ack_timeout_count += ack_timeouts;
    st.host_checksum_count += host_checksums;
    st.other_failure_count += other_failures;
    st.recovery_count += recoveries;
    st.recovery_restart_count += recovery_restarts;
    st.max_attempts = std::max(st.max_attempts, attempts);
    st.total_sum_us += total_us;
    st.total_max_us = std::max(st.total_max_us, total_us);
    st.lock_max_us = std::max(st.lock_max_us, lock_max_us);
    st.turnaround_max_us = std::max(st.turnaround_max_us, turnaround_max_us);
    st.hack_max_us = std::max(st.hack_max_us, hack_max_us);
    st.dack_max_us = std::max(st.dack_max_us, dack_max_us);
    st.recovery_max_us = std::max(st.recovery_max_us, recovery_max_us);

    if (st.count >= 100U) {
      snapshot = st;
      st = {};
      emit = true;
    }
  }

  if (emit) {
    LOGW("spi_phase_diag: endpoint=0x%x, total_avg_us=%" PRIu64 ", total_max_us=%" PRIu64
         ", lock_max_us=%" PRIu64 ", turnaround_max_us=%" PRIu64 ", hack_max_us=%" PRIu64
         ", dack_max_us=%" PRIu64 ", recovery_max_us=%" PRIu64
         ", slow_over_5ms=%u, retries=%u, nacks=%u, ack_timeouts=%u, max_attempts=%u"
         ", hack_nacks=%u, dack_nacks=%u, host_checksums=%u, other_failures=%u"
         ", recoveries=%u, recovery_restarts=%u",
         endpoint, snapshot.total_sum_us / snapshot.count, snapshot.total_max_us,
         snapshot.lock_max_us, snapshot.turnaround_max_us, snapshot.hack_max_us,
         snapshot.dack_max_us, snapshot.recovery_max_us, snapshot.slow_count,
         snapshot.retry_count, snapshot.nack_count, snapshot.ack_timeout_count,
         snapshot.max_attempts, snapshot.hack_nack_count, snapshot.dack_nack_count,
         snapshot.host_checksum_count, snapshot.other_failure_count,
         snapshot.recovery_count, snapshot.recovery_restart_count);
  }
}

static int spi_lock_priority(uint8_t endpoint) {
  if (endpoint == 0x03U) return 2;  // CAN TX
  if (endpoint == 0x81U) return 1;  // CAN RX
  return 0;                         // control and status
}

class SpiPriorityArbiter {
public:
  void lock(int priority) {
    std::unique_lock lk(lock_);
    waiters_[priority]++;
    cv_.wait(lk, [this, priority]() {
      if (active_) return false;
      for (int i = priority + 1; i < 3; i++) {
        if (waiters_[i] > 0U) return false;
      }
      return true;
    });
    waiters_[priority]--;
    active_ = true;
  }

  void unlock() {
    {
      std::lock_guard lk(lock_);
      active_ = false;
    }
    cv_.notify_all();
  }

private:
  std::mutex lock_;
  std::condition_variable cv_;
  uint32_t waiters_[3] = {0U, 0U, 0U};
  bool active_ = false;
};

static SpiPriorityArbiter spi_priority_arbiter;

class LockEx {
public:
  LockEx(int fd_, std::recursive_mutex &m_, uint8_t endpoint) : fd(fd_), m(m_) {
    spi_priority_arbiter.lock(spi_lock_priority(endpoint));
    m.lock();
    flock(fd, LOCK_EX);
  }

  ~LockEx() {
    flock(fd, LOCK_UN);
    m.unlock();
    spi_priority_arbiter.unlock();
  }

private:
  int fd;
  std::recursive_mutex &m;
};

#define SPILOG(fn, fmt, ...) do {  \
      fn(fmt, ## __VA_ARGS__);     \
      fn("  %d / 0x%x / %d / %d / tx: %s", \
         xfer_count, header.endpoint, header.tx_len, header.max_rx_len, \
         util::hexdump(tx_buf, std::min((int)header.tx_len, 8)).c_str()); \
      } while (0)

PandaSpiHandle::PandaSpiHandle(std::string serial) {
  int ret;
  const int uid_len = 12;
  uint8_t uid[uid_len] = {0};

  uint32_t spi_mode = SPI_MODE_0;
  uint8_t spi_bits_per_word = 8;

  // 50MHz is the max of the 845. note that some older
  // revs of the comma three may not support this speed
  uint32_t spi_speed = 50000000;
  try {
    if (!util::file_exists(SPI_DEVICE)) {
      throw std::runtime_error("Error connecting to panda: SPI device not found");
    }

    spi_fd = open(SPI_DEVICE.c_str(), O_RDWR);
    if (spi_fd < 0) {
      LOGE("failed opening SPI device %d", spi_fd);
      throw std::runtime_error("Error connecting to panda: failed to open SPI device");
    }

    // SPI settings
    util::safe_ioctl(spi_fd, SPI_IOC_WR_MODE, &spi_mode, "failed setting SPI mode");
    util::safe_ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed, "failed setting SPI speed");
    util::safe_ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word, "failed setting SPI bits per word");

    // get hw UID/serial
    ret = control_read(0xc3, 0, 0, uid, uid_len, 100);
    if (ret == uid_len) {
      std::stringstream stream;
      for (int i = 0; i < uid_len; i++) {
        stream << std::hex << std::setw(2) << std::setfill('0') << int(uid[i]);
      }
      hw_serial = stream.str();
    } else {
      LOGD("failed to get serial %d", ret);
      throw std::runtime_error("Error connecting to panda: failed to get serial");
    }

    if (!serial.empty() && (serial != hw_serial)) {
      throw std::runtime_error("Error connecting to panda: serial mismatch");
    }

  } catch (...) {
    cleanup();
    throw;
  }
  return;
}

PandaSpiHandle::~PandaSpiHandle() {
  std::lock_guard lk(hw_lock);
  cleanup();
}

void PandaSpiHandle::cleanup() {
  if (spi_fd != -1) {
    close(spi_fd);
    spi_fd = -1;
  }
}



int PandaSpiHandle::control_write(uint8_t request, uint16_t param1, uint16_t param2, unsigned int timeout) {
  ControlPacket_t packet = {
    .request = request,
    .param1 = param1,
    .param2 = param2,
    .length = 0
  };
  return spi_transfer_retry(0, (uint8_t *) &packet, sizeof(packet), NULL, 0, timeout);
}

int PandaSpiHandle::control_read(uint8_t request, uint16_t param1, uint16_t param2, unsigned char *data, uint16_t length, unsigned int timeout) {
  ControlPacket_t packet = {
    .request = request,
    .param1 = param1,
    .param2 = param2,
    .length = length
  };
  return spi_transfer_retry(0, (uint8_t *) &packet, sizeof(packet), data, length, timeout);
}

int PandaSpiHandle::bulk_write(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout) {
  return bulk_transfer(endpoint, data, length, NULL, 0, timeout);
}
int PandaSpiHandle::bulk_read(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout) {
  return bulk_transfer(endpoint, NULL, 0, data, length, timeout);
}

int PandaSpiHandle::bulk_transfer(uint8_t endpoint, uint8_t *tx_data, uint16_t tx_len, uint8_t *rx_data, uint16_t rx_len, unsigned int timeout) {
  const int xfer_size = SPI_BUF_SIZE - 0x40;

  int ret = 0;
  uint16_t length = (tx_data != NULL) ? tx_len : rx_len;
  for (int i = 0; i < (int)std::ceil((float)length / xfer_size); i++) {
    int d;
    if (tx_data != NULL) {
      int len = std::min(xfer_size, tx_len - (xfer_size * i));
      d = spi_transfer_retry(endpoint, tx_data + (xfer_size * i), len, NULL, 0, timeout);
    } else {
      uint16_t to_read = std::min(xfer_size, rx_len - ret);
      d = spi_transfer_retry(endpoint, NULL, 0, rx_data + (xfer_size * i), to_read, timeout);
    }

    if (d < 0) {
      SPILOG(LOGE, "SPI: bulk transfer failed with %d", d);
      comms_healthy = false;
      return d;
    }

    ret += d;
    if ((rx_data != NULL) && d < xfer_size) {
      break;
    }
  }

  return ret;
}

std::vector<std::string> PandaSpiHandle::list() {
  try {
    PandaSpiHandle sh("");
    return {sh.hw_serial};
  } catch (std::exception &e) {
    // no panda on SPI
  }
  return {};
}

void add_checksum(uint8_t *data, int data_len) {
  data[data_len] = SPI_CHECKSUM_START;
  for (int i=0; i < data_len; i++) {
    data[data_len] ^= data[i];
  }
}

bool check_checksum(uint8_t *data, int data_len) {
  uint8_t checksum = SPI_CHECKSUM_START;
  for (uint16_t i = 0U; i < data_len; i++) {
    checksum ^= data[i];
  }
  return checksum == 0U;
}


int PandaSpiHandle::spi_transfer_retry(uint8_t endpoint, uint8_t *tx_data, uint16_t tx_len, uint8_t *rx_data, uint16_t max_rx_len, unsigned int timeout) {
  int ret;
  int nack_count = 0;
  int timeout_count = 0;
  bool timed_out = false;
  double start_time = millis_since_boot();
  const uint64_t diag_start_ns = nanos_since_boot();
  uint32_t attempts = 0U;
  uint32_t total_nacks = 0U;
  uint32_t total_hack_nacks = 0U;
  uint32_t total_dack_nacks = 0U;
  uint32_t total_ack_timeouts = 0U;
  uint32_t total_host_checksums = 0U;
  uint32_t total_other_failures = 0U;
  uint32_t total_recoveries = 0U;
  uint32_t total_recovery_restarts = 0U;
  uint64_t lock_max_us = 0U;
  uint64_t turnaround_max_us = 0U;
  uint64_t hack_max_us = 0U;
  uint64_t dack_max_us = 0U;
  uint64_t recovery_max_us = 0U;
  SpiAttemptTiming first_failure_timing = {};
  SpiAttemptTiming last_failure_timing = {};

  do {
    ret = spi_transfer(endpoint, tx_data, tx_len, rx_data, max_rx_len, timeout);
    attempts++;
    total_nacks += ret == SpiError::NACK;
    total_ack_timeouts += ret == SpiError::ACK_TIMEOUT;
    total_hack_nacks += spi_attempt_timing.failure_phase == SpiFailurePhase::HACK_NACK;
    total_dack_nacks += spi_attempt_timing.failure_phase == SpiFailurePhase::DACK_NACK;
    total_host_checksums += spi_attempt_timing.failure_phase == SpiFailurePhase::RX_CHECKSUM;
    if (ret < 0) {
      if (total_recoveries == 0U) {
        first_failure_timing = spi_attempt_timing;
      }
      last_failure_timing = spi_attempt_timing;
      total_recoveries++;
    }
    total_recovery_restarts += spi_attempt_timing.recovery_restarts;
    total_other_failures += (ret < 0) &&
                            (spi_attempt_timing.failure_phase != SpiFailurePhase::HACK_NACK) &&
                            (spi_attempt_timing.failure_phase != SpiFailurePhase::DACK_NACK) &&
                            (spi_attempt_timing.failure_phase != SpiFailurePhase::HACK_TIMEOUT) &&
                            (spi_attempt_timing.failure_phase != SpiFailurePhase::DACK_TIMEOUT) &&
                            (spi_attempt_timing.failure_phase != SpiFailurePhase::RX_CHECKSUM);
    lock_max_us = std::max(lock_max_us, spi_attempt_timing.lock_us);
    turnaround_max_us = std::max(turnaround_max_us, spi_attempt_timing.turnaround_us);
    hack_max_us = std::max(hack_max_us, spi_attempt_timing.hack_us);
    dack_max_us = std::max(dack_max_us, spi_attempt_timing.dack_us);
    recovery_max_us = std::max(recovery_max_us, spi_attempt_timing.recovery_us);

    if (ret < 0) {
      timed_out = (timeout != 0) && (timeout_count > 5);
      timeout_count += ret == SpiError::ACK_TIMEOUT;

      // give other threads a chance to run
      std::this_thread::yield();

      if (ret == SpiError::NACK) {
        // prevent busy waiting while the panda is NACK'ing
        // due to full TX buffers
        nack_count += 1;
        if (nack_count > 3) {
          SPILOG(LOGD, "NACK sleep %d", nack_count);
          usleep(std::clamp(nack_count*10, 200, 2000));
        }
      }
    }
  } while (ret < 0 && connected && !timed_out);

  record_spi_phase_diag(endpoint, (nanos_since_boot() - diag_start_ns) / 1000U,
                        attempts, total_nacks, total_hack_nacks, total_dack_nacks,
                        total_ack_timeouts, total_host_checksums, total_other_failures,
                        total_recoveries, total_recovery_restarts,
                        lock_max_us, turnaround_max_us, hack_max_us,
                        dack_max_us, recovery_max_us);

  // Log after the retry sequence so diagnostics never delay a recovery attempt.
  if (total_recoveries > 0U) {
    LOGW("spi_failure_diag: endpoint=0x%x, attempts=%u, final_ret=%d"
         ", hack_nacks=%u, dack_nacks=%u, ack_timeouts=%u, host_checksums=%u"
         ", other_failures=%u, first_phase=%s, last_phase=%s"
         ", first_lock_us=%" PRIu64 ", first_turnaround_us=%" PRIu64
         ", first_hack_us=%" PRIu64 ", first_dack_us=%" PRIu64
         ", first_recovery_us=%" PRIu64
         ", last_lock_us=%" PRIu64 ", last_turnaround_us=%" PRIu64
         ", last_hack_us=%" PRIu64 ", last_dack_us=%" PRIu64
         ", last_recovery_us=%" PRIu64 ", recovery_restarts=%u",
         endpoint, attempts, ret, total_hack_nacks, total_dack_nacks,
         total_ack_timeouts, total_host_checksums, total_other_failures,
         spi_failure_phase_name(first_failure_timing.failure_phase),
         spi_failure_phase_name(last_failure_timing.failure_phase),
         first_failure_timing.lock_us, first_failure_timing.turnaround_us,
         first_failure_timing.hack_us, first_failure_timing.dack_us,
         first_failure_timing.recovery_us,
         last_failure_timing.lock_us, last_failure_timing.turnaround_us,
         last_failure_timing.hack_us, last_failure_timing.dack_us,
         last_failure_timing.recovery_us, total_recovery_restarts);
  }

  if (ret < 0) {
    SPILOG(LOGE, "transfer failed, after %d tries, %.2fms", timeout_count, millis_since_boot() - start_time);
  }

  return ret;
}

int PandaSpiHandle::wait_for_ack(uint8_t ack, uint8_t tx, unsigned int timeout, unsigned int length) {
  double start_millis = millis_since_boot();
  if (timeout == 0) {
    timeout = SPI_ACK_TIMEOUT;
  }
  timeout = std::clamp(timeout, 20U, SPI_ACK_TIMEOUT);

  spi_ioc_transfer transfer = {
    .tx_buf = (uint64_t)tx_buf,
    .rx_buf = (uint64_t)rx_buf,
    .len = length,
  };
  memset(tx_buf, tx, length);

  while (true) {
    int ret = lltransfer(transfer);
    if (ret < 0) {
      SPILOG(LOGE, "SPI: failed to send ACK request");
      return ret;
    }

    if (rx_buf[0] == ack) {
      break;
    } else if (rx_buf[0] == SPI_NACK) {
      SPILOG(LOGD, "SPI: got NACK, waiting for 0x%x", ack);
      return SpiError::NACK;
    }

    // handle timeout
    if (millis_since_boot() - start_millis > timeout) {
      SPILOG(LOGW, "SPI: timed out waiting for ACK, waiting for 0x%x", ack);
      return SpiError::ACK_TIMEOUT;
    }
  }

  return 0;
}

int PandaSpiHandle::lltransfer(spi_ioc_transfer &t) {
  static const double err_prob = std::stod(util::getenv("SPI_ERR_PROB", "-1"));

  if (err_prob > 0) {
    if ((static_cast<double>(rand()) / RAND_MAX) < err_prob) {
      printf("transfer len error\n");
      t.len = rand() % SPI_BUF_SIZE;
    }
    if ((static_cast<double>(rand()) / RAND_MAX) < err_prob && t.tx_buf != (uint64_t)NULL) {
      printf("corrupting TX\n");
      for (int i = 0; i < t.len; i++) {
        if ((static_cast<double>(rand()) / RAND_MAX) > 0.9) {
          ((uint8_t*)t.tx_buf)[i] = (uint8_t)(rand() % 256);
        }
      }
    }
  }

  int ret = util::safe_ioctl(spi_fd, SPI_IOC_MESSAGE(1), &t);

  if (err_prob > 0) {
    if ((static_cast<double>(rand()) / RAND_MAX) < err_prob && t.rx_buf != (uint64_t)NULL) {
      printf("corrupting RX\n");
      for (int i = 0; i < t.len; i++) {
        if ((static_cast<double>(rand()) / RAND_MAX) > 0.9) {
          ((uint8_t*)t.rx_buf)[i] = (uint8_t)(rand() % 256);
        }
      }
    }
  }

  return ret;
}

int PandaSpiHandle::spi_transfer(uint8_t endpoint, uint8_t *tx_data, uint16_t tx_len, uint8_t *rx_data, uint16_t max_rx_len, unsigned int timeout) {
  int ret;
  uint16_t rx_data_len;
  const uint64_t attempt_start_ns = nanos_since_boot();
  uint64_t phase_start_ns = 0U;
  spi_attempt_timing = {};
  const bool safety_mode_control = (endpoint == 0U) && (tx_data != nullptr) &&
                                   (tx_len >= sizeof(ControlPacket_t)) && (tx_data[0] == 0xdcU);
  LockEx lock(spi_fd, hw_lock, endpoint);
  spi_attempt_timing.lock_us = (nanos_since_boot() - attempt_start_ns) / 1000U;

  // needs to be less, since we need to have space for the checksum
  assert(tx_len < SPI_BUF_SIZE);
  assert(max_rx_len < SPI_BUF_SIZE);

  phase_start_ns = nanos_since_boot();
  wait_for_spi_turnaround(spi_last_bus_activity_ns, spi_inter_transaction_ns);
  spi_attempt_timing.turnaround_us += (nanos_since_boot() - phase_start_ns) / 1000U;

  xfer_count++;
  header = {
    .sync = SPI_SYNC,
    .endpoint = endpoint,
    .tx_len = tx_len,
    .max_rx_len = max_rx_len
  };

  spi_ioc_transfer transfer = {
    .tx_buf = (uint64_t)tx_buf,
    .rx_buf = (uint64_t)rx_buf
  };

  // Send header
  memcpy(tx_buf, &header, sizeof(header));
  add_checksum(tx_buf, sizeof(header));
  transfer.len = sizeof(header) + 1;
  ret = lltransfer(transfer);
  if (ret < 0) {
    spi_attempt_timing.failure_phase = SpiFailurePhase::HEADER_IO;
    SPILOG(LOGE, "SPI: failed to send header");
    goto fail;
  }

  // Wait for (N)ACK
  phase_start_ns = nanos_since_boot();
  ret = wait_for_ack(SPI_HACK, 0x11, timeout, 1);
  spi_attempt_timing.hack_us = (nanos_since_boot() - phase_start_ns) / 1000U;
  if (ret < 0) {
    spi_attempt_timing.failure_phase = ret == SpiError::NACK ? SpiFailurePhase::HACK_NACK :
                                       ret == SpiError::ACK_TIMEOUT ? SpiFailurePhase::HACK_TIMEOUT :
                                       SpiFailurePhase::HEADER_IO;
    goto fail;
  }
  phase_start_ns = nanos_since_boot();
  wait_for_spi_turnaround(nanos_since_boot());
  spi_attempt_timing.turnaround_us += (nanos_since_boot() - phase_start_ns) / 1000U;

  // Send data
  if (tx_data != NULL) {
    memcpy(tx_buf, tx_data, tx_len);
  }
  add_checksum(tx_buf, tx_len);
  transfer.len = tx_len + 1;
  ret = lltransfer(transfer);
  if (ret < 0) {
    spi_attempt_timing.failure_phase = SpiFailurePhase::DATA_IO;
    SPILOG(LOGE, "SPI: failed to send data");
    goto fail;
  }

  // Wait for (N)ACK
  phase_start_ns = nanos_since_boot();
  ret = wait_for_ack(SPI_DACK, 0x13, timeout, 3);
  spi_attempt_timing.dack_us = (nanos_since_boot() - phase_start_ns) / 1000U;
  if (ret < 0) {
    spi_attempt_timing.failure_phase = ret == SpiError::NACK ? SpiFailurePhase::DACK_NACK :
                                       ret == SpiError::ACK_TIMEOUT ? SpiFailurePhase::DACK_TIMEOUT :
                                       SpiFailurePhase::DATA_IO;
    goto fail;
  }

  // Read data
  rx_data_len = *(uint16_t *)(rx_buf+1);
  if (rx_data_len >= SPI_BUF_SIZE) {
    spi_attempt_timing.failure_phase = SpiFailurePhase::RX_LENGTH;
    SPILOG(LOGE, "SPI: RX data len larger than buf size %d", rx_data_len);
    goto fail;
  }

  transfer.len = rx_data_len + 1;
  transfer.rx_buf = (uint64_t)(rx_buf + 2 + 1);
  ret = lltransfer(transfer);
  if (ret < 0) {
    spi_attempt_timing.failure_phase = SpiFailurePhase::RX_IO;
    SPILOG(LOGE, "SPI: failed to read rx data");
    goto fail;
  }
  if (!check_checksum(rx_buf, rx_data_len + 4)) {
    spi_attempt_timing.failure_phase = SpiFailurePhase::RX_CHECKSUM;
    SPILOG(LOGE, "SPI: bad checksum");
    goto fail;
  }

  if (rx_data != NULL) {
    memcpy(rx_data, rx_buf + 3, rx_data_len);
  }

  spi_last_bus_activity_ns = nanos_since_boot();
  // Safety mode changes reinitialize the Panda CAN controllers. The dedicated
  // CAN RX thread can already be waiting on this transfer, so keep the shared
  // SPI bus idle long enough for the Panda header DMA to be armed again.
  spi_inter_transaction_ns = safety_mode_control ? SPI_SAFETY_MODE_TURNAROUND_NS : SPI_TURNAROUND_NS;
  spi_attempt_timing.total_us = (spi_last_bus_activity_ns - attempt_start_ns) / 1000U;
  return rx_data_len;

fail:
  // ensure slave is in a consistent state
  // and ready for the next transfer
  phase_start_ns = nanos_since_boot();
  int nack_cnt = 0;
  while (nack_cnt < 3) {
    if (wait_for_ack(SPI_NACK, 0x14, 1, SPI_BUF_SIZE/2) == 0) {
      nack_cnt += 1;
    } else {
      nack_cnt = 0;
      spi_attempt_timing.recovery_restarts++;
    }
  }
  spi_attempt_timing.recovery_us = (nanos_since_boot() - phase_start_ns) / 1000U;

  spi_last_bus_activity_ns = nanos_since_boot();
  spi_inter_transaction_ns = SPI_TURNAROUND_NS;
  spi_attempt_timing.total_us = (spi_last_bus_activity_ns - attempt_start_ns) / 1000U;
  if (ret >= 0) ret = -1;
  return ret;
}
