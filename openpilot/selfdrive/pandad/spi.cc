#include <sys/file.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include <cassert>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iomanip>
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
  HEADER_NACK = -2,
  ACK_TIMEOUT = -3,
  RECOVERY_FAILED = -4,
  DATA_NACK = -5,
};

const unsigned int SPI_ACK_MIN_TIMEOUT = 20; // milliseconds
const unsigned int SPI_ACK_TIMEOUT = 50; // milliseconds
// A blocked SPI transfer also blocks the 100 Hz CAN path. Transient NACK bursts
// normally recover within a few milliseconds, so reconnect on sustained faults.
const unsigned int SPI_TRANSFER_RETRY_TIMEOUT = 100; // milliseconds
const int SPI_MAX_NACK_RETRIES = 8;
const int SPI_MAX_ACK_TIMEOUTS = 3;
const unsigned int SPI_RECOVERY_TIMEOUT = 50; // milliseconds
const int SPI_RECOVERY_MAX_ATTEMPTS = 12;
const double SPI_HOST_DIAGNOSTIC_INTERVAL = 5000.0; // milliseconds
const std::string SPI_DEVICE = "/dev/spidev0.0";

class LockEx {
public:
  LockEx(int fd_, std::recursive_mutex &m_) : fd(fd_), m(m_) {
    const double mutex_start = millis_since_boot();
    m.lock();
    mutex_wait_ms = millis_since_boot() - mutex_start;

    const double flock_start = millis_since_boot();
    flock_result = flock(fd, LOCK_EX);
    flock_errno = flock_result < 0 ? errno : 0;
    flock_wait_ms = millis_since_boot() - flock_start;
  }

  ~LockEx() {
    if (flock_result == 0) {
      flock(fd, LOCK_UN);
    }
    m.unlock();
  }

  double mutex_wait() const { return mutex_wait_ms; }
  double flock_wait() const { return flock_wait_ms; }
  int flock_ret() const { return flock_result; }
  int flock_error() const { return flock_errno; }

private:
  int fd;
  std::recursive_mutex &m;
  double mutex_wait_ms = 0.0;
  double flock_wait_ms = 0.0;
  int flock_result = -1;
  int flock_errno = 0;
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
  uint32_t spi_speed = 50000000U;
  host_diagnostics.requested_speed_hz = spi_speed;
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

    uint32_t actual_mode = 0;
    uint32_t actual_speed = 0;
    uint8_t actual_bits_per_word = 0;
    const int mode_ret = util::safe_ioctl(spi_fd, SPI_IOC_RD_MODE, &actual_mode);
    const int speed_ret = util::safe_ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &actual_speed);
    const int bits_ret = util::safe_ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &actual_bits_per_word);
    host_diagnostics.actual_mode = actual_mode;
    host_diagnostics.actual_speed_hz = actual_speed;
    host_diagnostics.actual_bits_per_word = actual_bits_per_word;
    LOGW("Panda SPI configuration: device=%s requested_speed=%u actual_speed=%u mode=0x%x bits=%u readback_ret=%d/%d/%d",
         SPI_DEVICE.c_str(), spi_speed, actual_speed, actual_mode, (unsigned)actual_bits_per_word,
         mode_ret, speed_ret, bits_ret);

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
    log_host_diagnostics(true);
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
  if (!connected) {
    return -1;
  }

  int ret = -1;
  int header_nack_count = 0;
  int data_nack_count = 0;
  int timeout_count = 0;
  const double start_time = millis_since_boot();
  const double transfer_deadline = start_time + SPI_TRANSFER_RETRY_TIMEOUT;
  bool retry_exhausted = false;

  do {
    ret = spi_transfer(endpoint, tx_data, tx_len, rx_data, max_rx_len, timeout, transfer_deadline);

    if (ret < 0) {
      header_nack_count += ret == SpiError::HEADER_NACK;
      data_nack_count += ret == SpiError::DATA_NACK;
      timeout_count += ret == SpiError::ACK_TIMEOUT;
      const int nack_count = header_nack_count + data_nack_count;
      const double elapsed = millis_since_boot() - start_time;
      retry_exhausted = (ret == SpiError::RECOVERY_FAILED) ||
                        (elapsed >= SPI_TRANSFER_RETRY_TIMEOUT) ||
                        (nack_count >= SPI_MAX_NACK_RETRIES) ||
                        (timeout_count >= SPI_MAX_ACK_TIMEOUTS);

      if (!retry_exhausted) {
        // give other threads a chance to run
        std::this_thread::yield();

        if ((ret == SpiError::HEADER_NACK) || (ret == SpiError::DATA_NACK)) {
          // Prevent busy waiting on a persistent protocol NACK.
          if (nack_count > 3) {
            SPILOG(LOGD, "NACK sleep %d", nack_count);
            usleep(std::clamp(nack_count*10, 200, 2000));
          }
        }
      }
    }
  } while (ret < 0 && connected && !retry_exhausted);

  if ((ret >= 0) && ((header_nack_count != 0) || (data_nack_count != 0) || (timeout_count != 0))) {
    LOGW_100("SPI transfer recovered: endpoint=0x%x tx=%u rx=%u header_nacks=%d data_nacks=%d timeouts=%d elapsed=%.2fms",
             endpoint, tx_len, max_rx_len, header_nack_count, data_nack_count, timeout_count,
             millis_since_boot() - start_time);
  }

  if (ret < 0) {
    const double elapsed = millis_since_boot() - start_time;
    LOGE("SPI transfer failed: ret=%d, endpoint=0x%x, tx=%u, rx=%u, header_nacks=%d, data_nacks=%d, timeouts=%d, elapsed=%.2fms",
         ret, endpoint, tx_len, max_rx_len, header_nack_count, data_nack_count, timeout_count, elapsed);

    // Let native pandad exit. The Python wrapper will reopen the SPI device and
    // configure Panda and safety from a known state instead of draining stale
    // sendcan messages after a multi-second stall.
    comms_healthy = false;
    connected = false;
  }

  return ret;
}

int PandaSpiHandle::wait_for_ack(uint8_t ack, uint8_t tx, unsigned int timeout, unsigned int length, double deadline) {
  const double start_millis = millis_since_boot();
  if (timeout == 0) {
    timeout = SPI_ACK_TIMEOUT;
  }
  timeout = std::clamp(timeout, SPI_ACK_MIN_TIMEOUT, SPI_ACK_TIMEOUT);
  const double ack_deadline = std::min(start_millis + timeout, deadline);

  spi_ioc_transfer transfer = {
    .tx_buf = (uint64_t)tx_buf,
    .rx_buf = (uint64_t)rx_buf,
    .len = length,
  };
  memset(tx_buf, tx, length);
  transfer_phase = (ack == SPI_HACK) ? TransferPhase::HEADER_ACK :
                   (ack == SPI_DACK) ? TransferPhase::DATA_ACK : TransferPhase::RECOVERY;
  uint32_t poll_count = 0;
  uint8_t last_rx[3] = {0U, 0U, 0U};

  while (true) {
    if (millis_since_boot() >= ack_deadline) {
      SPILOG(LOGW, "SPI ACK timeout before poll: expected=0x%x request=0x%x polls=%u last=%02x/%02x/%02x elapsed=%.2fms",
             ack, tx, poll_count, last_rx[0], last_rx[1], last_rx[2], millis_since_boot() - start_millis);
      return SpiError::ACK_TIMEOUT;
    }

    int ret = lltransfer(transfer);
    if (ret < 0) {
      SPILOG(LOGE, "SPI: failed to send ACK request");
      return ret;
    }
    poll_count += 1U;
    for (uint32_t i = 0U; i < std::min(length, 3U); i++) {
      last_rx[i] = rx_buf[i];
    }

    if (rx_buf[0] == ack) {
      break;
    } else if (rx_buf[0] == SPI_NACK) {
      const int nack_error = ack == SPI_HACK ? SpiError::HEADER_NACK : SpiError::DATA_NACK;
      SPILOG(LOGD, "SPI got %s NACK: expected=0x%x request=0x%x polls=%u rx=%02x/%02x/%02x elapsed=%.2fms",
             ack == SPI_HACK ? "header" : "data", ack, tx, poll_count,
             last_rx[0], last_rx[1], last_rx[2], millis_since_boot() - start_millis);
      return nack_error;
    }

    // handle timeout
    if (millis_since_boot() >= ack_deadline) {
      SPILOG(LOGW, "SPI ACK timeout: expected=0x%x request=0x%x polls=%u last=%02x/%02x/%02x elapsed=%.2fms",
             ack, tx, poll_count, last_rx[0], last_rx[1], last_rx[2], millis_since_boot() - start_millis);
      return SpiError::ACK_TIMEOUT;
    }
  }

  return 0;
}

const char *PandaSpiHandle::phase_name(TransferPhase phase) {
  switch (phase) {
    case TransferPhase::IDLE: return "idle";
    case TransferPhase::HEADER: return "header";
    case TransferPhase::HEADER_ACK: return "header_ack";
    case TransferPhase::DATA: return "data";
    case TransferPhase::DATA_ACK: return "data_ack";
    case TransferPhase::RESPONSE: return "response";
    case TransferPhase::RECOVERY: return "recovery";
  }
  return "unknown";
}

void PandaSpiHandle::record_lock_diagnostics(uint8_t endpoint, double mutex_wait_ms, double flock_wait_ms,
                                             int flock_ret, int flock_errno) {
  auto update_slow_counts = [](double elapsed, uint64_t &over_1ms, uint64_t &over_5ms,
                               uint64_t &over_20ms, uint64_t &over_100ms) {
    over_1ms += elapsed >= 1.0;
    over_5ms += elapsed >= 5.0;
    over_20ms += elapsed >= 20.0;
    over_100ms += elapsed >= 100.0;
  };

  host_diagnostics.mutex_wait_count += 1U;
  host_diagnostics.mutex_total_ms += mutex_wait_ms;
  update_slow_counts(mutex_wait_ms, host_diagnostics.mutex_over_1ms_count,
                     host_diagnostics.mutex_over_5ms_count, host_diagnostics.mutex_over_20ms_count,
                     host_diagnostics.mutex_over_100ms_count);
  if (mutex_wait_ms > host_diagnostics.mutex_max_ms) {
    host_diagnostics.mutex_max_ms = mutex_wait_ms;
    host_diagnostics.mutex_max_endpoint = endpoint;
  }

  host_diagnostics.flock_wait_count += 1U;
  host_diagnostics.flock_error_count += flock_ret < 0;
  host_diagnostics.flock_total_ms += flock_wait_ms;
  update_slow_counts(flock_wait_ms, host_diagnostics.flock_over_1ms_count,
                     host_diagnostics.flock_over_5ms_count, host_diagnostics.flock_over_20ms_count,
                     host_diagnostics.flock_over_100ms_count);
  if (flock_wait_ms > host_diagnostics.flock_max_ms) {
    host_diagnostics.flock_max_ms = flock_wait_ms;
    host_diagnostics.flock_max_endpoint = endpoint;
  }

  if ((flock_ret < 0) || (mutex_wait_ms >= 5.0) || (flock_wait_ms >= 5.0)) {
    LOGW("Panda SPI lock event: serial=%s endpoint=0x%x mutex=%.2fms flock=%.2fms flock_ret=%d errno=%d(%s)",
         hw_serial.c_str(), endpoint, mutex_wait_ms, flock_wait_ms, flock_ret, flock_errno,
         flock_errno != 0 ? strerror(flock_errno) : "none");
  }
}

void PandaSpiHandle::log_host_diagnostics(bool force) {
  if (host_diagnostics.ioctl_count == 0U) {
    return;
  }

  const double now = millis_since_boot();
  if (!force && (host_diagnostics.last_log_ms != 0.0) &&
      ((now - host_diagnostics.last_log_ms) < SPI_HOST_DIAGNOSTIC_INTERVAL)) {
    return;
  }
  host_diagnostics.last_log_ms = now;

  const double ioctl_average = host_diagnostics.ioctl_total_ms / host_diagnostics.ioctl_count;
  const double mutex_average = host_diagnostics.mutex_wait_count != 0U ?
                               host_diagnostics.mutex_total_ms / host_diagnostics.mutex_wait_count : 0.0;
  const double flock_average = host_diagnostics.flock_wait_count != 0U ?
                               host_diagnostics.flock_total_ms / host_diagnostics.flock_wait_count : 0.0;
  LOGW("Panda SPI host diagnostic: serial=%s speed=%u/%u mode=0x%x bits=%u "
       "ioctl=%llu err=%llu short=%llu slow1/5/20/100=%llu/%llu/%llu/%llu avg/max=%.3f/%.2fms "
       "max_ctx=0x%x/%s/%u/ret%d mutex=%llu slow=%llu/%llu/%llu/%llu avg/max=%.3f/%.2fms@0x%x "
       "flock=%llu err=%llu slow=%llu/%llu/%llu/%llu avg/max=%.3f/%.2fms@0x%x",
       hw_serial.c_str(), host_diagnostics.requested_speed_hz, host_diagnostics.actual_speed_hz,
       host_diagnostics.actual_mode, (unsigned)host_diagnostics.actual_bits_per_word,
       (unsigned long long)host_diagnostics.ioctl_count,
       (unsigned long long)host_diagnostics.ioctl_error_count,
       (unsigned long long)host_diagnostics.ioctl_short_count,
       (unsigned long long)host_diagnostics.ioctl_over_1ms_count,
       (unsigned long long)host_diagnostics.ioctl_over_5ms_count,
       (unsigned long long)host_diagnostics.ioctl_over_20ms_count,
       (unsigned long long)host_diagnostics.ioctl_over_100ms_count,
       ioctl_average, host_diagnostics.ioctl_max_ms, host_diagnostics.ioctl_max_endpoint,
       phase_name(host_diagnostics.ioctl_max_phase), host_diagnostics.ioctl_max_len,
       host_diagnostics.ioctl_max_ret,
       (unsigned long long)host_diagnostics.mutex_wait_count,
       (unsigned long long)host_diagnostics.mutex_over_1ms_count,
       (unsigned long long)host_diagnostics.mutex_over_5ms_count,
       (unsigned long long)host_diagnostics.mutex_over_20ms_count,
       (unsigned long long)host_diagnostics.mutex_over_100ms_count,
       mutex_average, host_diagnostics.mutex_max_ms, host_diagnostics.mutex_max_endpoint,
       (unsigned long long)host_diagnostics.flock_wait_count,
       (unsigned long long)host_diagnostics.flock_error_count,
       (unsigned long long)host_diagnostics.flock_over_1ms_count,
       (unsigned long long)host_diagnostics.flock_over_5ms_count,
       (unsigned long long)host_diagnostics.flock_over_20ms_count,
       (unsigned long long)host_diagnostics.flock_over_100ms_count,
       flock_average, host_diagnostics.flock_max_ms, host_diagnostics.flock_max_endpoint);
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

  const double ioctl_start = millis_since_boot();
  int ret = util::safe_ioctl(spi_fd, SPI_IOC_MESSAGE(1), &t);
  const int ioctl_errno = ret < 0 ? errno : 0;
  const double ioctl_elapsed = millis_since_boot() - ioctl_start;

  host_diagnostics.ioctl_count += 1U;
  host_diagnostics.ioctl_error_count += ret < 0;
  host_diagnostics.ioctl_short_count += (ret >= 0) && (ret != static_cast<int>(t.len));
  host_diagnostics.ioctl_over_1ms_count += ioctl_elapsed >= 1.0;
  host_diagnostics.ioctl_over_5ms_count += ioctl_elapsed >= 5.0;
  host_diagnostics.ioctl_over_20ms_count += ioctl_elapsed >= 20.0;
  host_diagnostics.ioctl_over_100ms_count += ioctl_elapsed >= 100.0;
  host_diagnostics.ioctl_total_ms += ioctl_elapsed;
  if (ioctl_elapsed > host_diagnostics.ioctl_max_ms) {
    host_diagnostics.ioctl_max_ms = ioctl_elapsed;
    host_diagnostics.ioctl_max_endpoint = header.endpoint;
    host_diagnostics.ioctl_max_phase = transfer_phase;
    host_diagnostics.ioctl_max_len = t.len;
    host_diagnostics.ioctl_max_ret = ret;
  }

  if ((ret < 0) || (ret != static_cast<int>(t.len)) || (ioctl_elapsed >= 5.0)) {
    const uint8_t *rx = reinterpret_cast<const uint8_t *>(t.rx_buf);
    const uint8_t rx0 = (rx != nullptr) && (t.len > 0U) ? rx[0] : 0U;
    const uint8_t rx1 = (rx != nullptr) && (t.len > 1U) ? rx[1] : 0U;
    const uint8_t rx2 = (rx != nullptr) && (t.len > 2U) ? rx[2] : 0U;
    LOGW("Panda SPI ioctl event: serial=%s endpoint=0x%x phase=%s len=%u ret=%d errno=%d(%s) "
         "elapsed=%.2fms speed=%u delay=%u bits=%u cs_change=%u rx=%02x/%02x/%02x",
         hw_serial.c_str(), header.endpoint, phase_name(transfer_phase), t.len, ret, ioctl_errno,
         ioctl_errno != 0 ? strerror(ioctl_errno) : "none", ioctl_elapsed, t.speed_hz,
         (unsigned)t.delay_usecs, (unsigned)t.bits_per_word, (unsigned)t.cs_change, rx0, rx1, rx2);
  }

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

  log_host_diagnostics(false);

  return ret;
}

int PandaSpiHandle::spi_transfer(uint8_t endpoint, uint8_t *tx_data, uint16_t tx_len, uint8_t *rx_data, uint16_t max_rx_len,
                                unsigned int timeout, double deadline) {
  int ret;
  uint16_t rx_data_len;
  LockEx lock(spi_fd, hw_lock);
  record_lock_diagnostics(endpoint, lock.mutex_wait(), lock.flock_wait(), lock.flock_ret(), lock.flock_error());

  // needs to be less, since we need to have space for the checksum
  assert(tx_len < SPI_BUF_SIZE);
  assert(max_rx_len < SPI_BUF_SIZE);

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
  transfer_phase = TransferPhase::HEADER;
  ret = lltransfer(transfer);
  if (ret < 0) {
    SPILOG(LOGE, "SPI: failed to send header");
    goto fail;
  }

  // Wait for (N)ACK
  ret = wait_for_ack(SPI_HACK, 0x11, timeout, 1, deadline);
  if (ret < 0) {
    goto fail;
  }

  // Send data
  if (tx_data != NULL) {
    memcpy(tx_buf, tx_data, tx_len);
  }
  add_checksum(tx_buf, tx_len);
  transfer.len = tx_len + 1;
  transfer_phase = TransferPhase::DATA;
  ret = lltransfer(transfer);
  if (ret < 0) {
    SPILOG(LOGE, "SPI: failed to send data");
    goto fail;
  }

  // Wait for (N)ACK
  ret = wait_for_ack(SPI_DACK, 0x13, timeout, 3, deadline);
  if (ret < 0) {
    goto fail;
  }

  // Read data
  rx_data_len = *(uint16_t *)(rx_buf+1);
  if (rx_data_len >= SPI_BUF_SIZE) {
    SPILOG(LOGE, "SPI: RX data len larger than buf size %d", rx_data_len);
    goto fail;
  }

  transfer.len = rx_data_len + 1;
  transfer.rx_buf = (uint64_t)(rx_buf + 2 + 1);
  transfer_phase = TransferPhase::RESPONSE;
  ret = lltransfer(transfer);
  if (ret < 0) {
    SPILOG(LOGE, "SPI: failed to read rx data");
    goto fail;
  }
  if (!check_checksum(rx_buf, rx_data_len + 4)) {
    SPILOG(LOGE, "SPI response checksum failed: response_len=%u rx=%s", rx_data_len,
           util::hexdump(rx_buf, std::min((int)rx_data_len + 4, 16)).c_str());
    goto fail;
  }

  if (rx_data != NULL) {
    memcpy(rx_data, rx_buf + 3, rx_data_len);
  }

  transfer_phase = TransferPhase::IDLE;
  return rx_data_len;

fail:
  // ensure slave is in a consistent state
  // and ready for the next transfer
  int nack_cnt = 0;
  int recovery_attempts = 0;
  const double recovery_start_time = millis_since_boot();
  const double recovery_deadline = std::min(recovery_start_time + SPI_RECOVERY_TIMEOUT, deadline);
  while ((nack_cnt < 3) &&
         (recovery_attempts < SPI_RECOVERY_MAX_ATTEMPTS) &&
         (millis_since_boot() < recovery_deadline)) {
    recovery_attempts += 1;
    if (wait_for_ack(SPI_NACK, 0x14, 1, SPI_BUF_SIZE/2, recovery_deadline) == 0) {
      nack_cnt += 1;
    } else {
      nack_cnt = 0;
    }
  }
  if (nack_cnt < 3) {
    if (recovery_attempts == 0) {
      LOGE("SPI recovery skipped: transfer deadline expired before recovery (elapsed=%.2fms deadline_overrun=%.2fms)",
           millis_since_boot() - recovery_start_time, millis_since_boot() - deadline);
    } else {
      LOGE("SPI recovery failed after %d attempts (%.2fms)", recovery_attempts,
           millis_since_boot() - recovery_start_time);
    }
    ret = SpiError::RECOVERY_FAILED;
  }

  transfer_phase = TransferPhase::IDLE;
  if (ret >= 0) ret = -1;
  return ret;
}
