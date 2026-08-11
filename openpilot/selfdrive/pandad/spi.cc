#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/random.h>
#include <linux/spi/spidev.h>
#include <unistd.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <thread>
#include <vector>

#include "common/util.h"
#include "common/timing.h"
#include "common/swaglog.h"
#include "panda/board/comms_definitions.h"
#include "selfdrive/pandad/panda_comms.h"
#include "selfdrive/pandad/spi_v3_transport.h"
#include "selfdrive/pandad/spi_version.h"


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
  CAN_TX_FULL = -6,
};

const unsigned int SPI_ACK_MIN_TIMEOUT = 20; // milliseconds
const unsigned int SPI_ACK_TIMEOUT = 50; // milliseconds
// A blocked SPI transfer also blocks the 100 Hz CAN path. Transient NACK bursts
// normally recover within a few milliseconds, so reconnect on sustained faults.
const unsigned int SPI_TRANSFER_RETRY_TIMEOUT = 100; // milliseconds
const unsigned int SPI_CAN_TX_RETRY_DELAY = 1000; // microseconds
const int SPI_MAX_NACK_RETRIES = 8;
const int SPI_MAX_ACK_TIMEOUTS = 3;
const unsigned int SPI_RECOVERY_TIMEOUT = 50; // milliseconds
const int SPI_RECOVERY_MAX_ATTEMPTS = 12;
const std::string SPI_DEVICE = "/dev/spidev0.0";
// The Panda SPI slave switches RX DMA phases from its TX-complete interrupt.
// Do not clock the next phase until that interrupt has had time to re-arm RX.
// Protocol v3 removes this phase transition; the guard remains for v2 devices.
static uint64_t spi_last_bus_activity_ns = 0;  // protected by hw_lock

static void wait_for_spi_turnaround(uint64_t start_ns) {
  while ((nanos_since_boot() - start_ns) < 400000) {}
}

static uint32_t new_spi_session_id() {
  uint32_t session_id = 0U;
  if (getrandom(&session_id, sizeof(session_id), GRND_NONBLOCK) !=
      static_cast<ssize_t>(sizeof(session_id))) {
    session_id = static_cast<uint32_t>(nanos_since_boot()) ^
                 (static_cast<uint32_t>(getpid()) << 16U);
  }
  // Zero is valid on the wire, but reserving it makes uninitialized sessions
  // obvious in firmware diagnostics.
  return session_id != 0U ? session_id : 1U;
}

class LockEx {
public:
  LockEx(int fd_, std::recursive_mutex &m_) : fd(fd_), m(m_) {
    m.lock();
    flock(fd, LOCK_EX);
  }

  ~LockEx() {
    flock(fd, LOCK_UN);
    m.unlock();
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

    if (!negotiate_protocol()) {
      // Very old Panda firmware did not expose stable VERSION discovery. It
      // can only speak v2, so preserve the historical control-read fallback.
      protocol_version_ = 2U;
      LOGW("SPI VERSION discovery failed, falling back to protocol v2");
    }
    LOGW("using Panda SPI protocol v%d", protocol_version_);

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
  v3_transport.reset();
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
  const size_t default_xfer_size = SPI_BUF_SIZE - 0x40;
  const int xfer_size = static_cast<int>(
    (protocol_version_ == static_cast<uint8_t>(panda::spi::ProtocolSelection::V3) && tx_data != nullptr) ?
      panda::spi_v3::bulk_write_chunk_size(endpoint, default_xfer_size) : default_xfer_size);

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

  if (protocol_version_ == static_cast<uint8_t>(panda::spi::ProtocolSelection::V3)) {
    return spi_transfer_v3(endpoint, tx_data, tx_len, rx_data, max_rx_len, timeout);
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

    // A data NACK on endpoint 3 is Panda CAN TX backpressure (or a bad data
    // checksum). Panda NACKs before enqueueing this buffer, so keep retrying the
    // same sendcan batch within the bounded SPI transfer deadline instead of
    // silently dropping it after the USB timeout.
    if (ret == SpiError::CAN_TX_FULL) {
      if (millis_since_boot() < transfer_deadline) {
        // Back off outside hw_lock so CAN RX and heartbeat traffic can use the
        // bus while Panda's CAN TX queue drains.
        usleep(SPI_CAN_TX_RETRY_DELAY);
        std::this_thread::yield();
        if (millis_since_boot() < transfer_deadline) {
          continue;
        }
      }
      ret = SpiError::DATA_NACK;
    }

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

int PandaSpiHandle::spi_transfer_v3(uint8_t endpoint, uint8_t *tx_data, uint16_t tx_len,
                                    uint8_t *rx_data, uint16_t max_rx_len,
                                    unsigned int timeout) {
  if (v3_transport == nullptr) {
    comms_healthy = false;
    connected = false;
    return static_cast<int>(panda::spi_v3::TransportError::InvalidArgument);
  }

  LockEx lock(spi_fd, hw_lock);
  xfer_count++;
  header = {
    .sync = SPI_SYNC,
    .endpoint = endpoint,
    .tx_len = tx_len,
    .max_rx_len = max_rx_len,
  };

  const panda::spi_v3::TransferResult result =
    v3_transport->transfer(endpoint, tx_data, tx_len, max_rx_len, timeout);
  if (!result.ok()) {
    LOGE("SPI v3 transfer failed: error=%d, status=%d, endpoint=0x%x, tx=%u, rx=%u",
         static_cast<int>(result.error), static_cast<int>(result.remote_status),
         endpoint, tx_len, max_rx_len);
    comms_healthy = false;
    connected = false;
    return static_cast<int>(result.error);
  }

  if (rx_data != nullptr && !result.payload.empty()) {
    memcpy(rx_data, result.payload.data(), result.payload.size());
  }
  return static_cast<int>(result.payload.size());
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

  while (true) {
    if (millis_since_boot() >= ack_deadline) {
      SPILOG(LOGW, "SPI: timed out waiting for ACK, waiting for 0x%x", ack);
      return SpiError::ACK_TIMEOUT;
    }

    int ret = lltransfer(transfer);
    if (ret < 0) {
      SPILOG(LOGE, "SPI: failed to send ACK request");
      return ret;
    }

    if (rx_buf[0] == ack) {
      break;
    } else if (rx_buf[0] == SPI_NACK) {
      const int nack_error = ack == SPI_HACK ? SpiError::HEADER_NACK : SpiError::DATA_NACK;
      SPILOG(LOGD, "SPI: got %s NACK, waiting for 0x%x", ack == SPI_HACK ? "header" : "data", ack);
      return nack_error;
    }

    // handle timeout
    if (millis_since_boot() >= ack_deadline) {
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

int PandaSpiHandle::spi_transfer(uint8_t endpoint, uint8_t *tx_data, uint16_t tx_len, uint8_t *rx_data, uint16_t max_rx_len,
                                unsigned int timeout, double deadline) {
  int ret;
  uint16_t rx_data_len;
  LockEx lock(spi_fd, hw_lock);

  // needs to be less, since we need to have space for the checksum
  assert(tx_len < SPI_BUF_SIZE);
  assert(max_rx_len < SPI_BUF_SIZE);

  wait_for_spi_turnaround(spi_last_bus_activity_ns);

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
    SPILOG(LOGE, "SPI: failed to send header");
    goto fail;
  }

  // Wait for (N)ACK
  ret = wait_for_ack(SPI_HACK, 0x11, timeout, 1, deadline);
  if (ret < 0) {
    goto fail;
  }
  wait_for_spi_turnaround(nanos_since_boot());

  // Send data
  if (tx_data != NULL) {
    memcpy(tx_buf, tx_data, tx_len);
  }
  add_checksum(tx_buf, tx_len);
  transfer.len = tx_len + 1;
  ret = lltransfer(transfer);
  if (ret < 0) {
    SPILOG(LOGE, "SPI: failed to send data");
    goto fail;
  }

  // Wait for (N)ACK
  ret = wait_for_ack(SPI_DACK, 0x13, timeout, 3, deadline);
  if ((ret == SpiError::DATA_NACK) && (endpoint == 3U) && (tx_len > 0U)) {
    // The next lock owner enforces the remaining NACK-to-header turnaround.
    spi_last_bus_activity_ns = nanos_since_boot();
    return SpiError::CAN_TX_FULL;
  }
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
  ret = lltransfer(transfer);
  if (ret < 0) {
    SPILOG(LOGE, "SPI: failed to read rx data");
    goto fail;
  }
  if (!check_checksum(rx_buf, rx_data_len + 4)) {
    SPILOG(LOGE, "SPI: bad checksum");
    goto fail;
  }

  if (rx_data != NULL) {
    memcpy(rx_data, rx_buf + 3, rx_data_len);
  }

  spi_last_bus_activity_ns = nanos_since_boot();
  return rx_data_len;

fail:
  // ensure slave is in a consistent state
  // and ready for the next transfer
  int nack_cnt = 0;
  int recovery_attempts = 0;
  const double recovery_start_time = millis_since_boot();
  const double recovery_deadline = std::min(recovery_start_time + SPI_RECOVERY_TIMEOUT, deadline);
  // The NACK or response that sent us here also completes from Panda's TX ISR.
  // Let it restore HEADER RX before clocking recovery junk.
  wait_for_spi_turnaround(nanos_since_boot());
  while ((nack_cnt < 3) &&
         (recovery_attempts < SPI_RECOVERY_MAX_ATTEMPTS) &&
         (millis_since_boot() < recovery_deadline)) {
    recovery_attempts += 1;
    const int recovery_ret = wait_for_ack(SPI_NACK, 0x14, 1, SPI_BUF_SIZE/2, recovery_deadline);
    wait_for_spi_turnaround(nanos_since_boot());
    if (recovery_ret == 0) {
      nack_cnt += 1;
    } else {
      nack_cnt = 0;
    }
  }
  if (nack_cnt < 3) {
    LOGE("SPI recovery failed after %d attempts (%.2fms)", recovery_attempts,
         millis_since_boot() - recovery_start_time);
    ret = SpiError::RECOVERY_FAILED;
  }

  if (ret >= 0) ret = -1;
  spi_last_bus_activity_ns = nanos_since_boot();
  return ret;
}

bool PandaSpiHandle::negotiate_protocol() {
  LockEx lock(spi_fd, hw_lock);
  panda::spi::VersionStreamDecoder decoder;

  const auto accept_version = [this](const std::optional<panda::spi::VersionInfo> &info) {
    if (!info.has_value()) {
      return false;
    }

    // VERSION retains the legacy v2 TX-complete re-arm. Keep the process-wide
    // flock until its terminal quiet time has elapsed, including when another
    // process is waiting to use spidev with its own activity timestamp.
    wait_for_spi_turnaround(spi_last_bus_activity_ns);
    const panda::spi::ProtocolSelection selection = panda::spi::select_protocol(*info);
    if (selection == panda::spi::ProtocolSelection::Unsupported) {
      throw std::runtime_error("unsupported Panda SPI protocol/device combination");
    }
    protocol_version_ = static_cast<uint8_t>(selection);
    if (selection == panda::spi::ProtocolSelection::V3) {
      v3_transport = std::make_unique<panda::spi_v3::Transport>(
        [this](const uint8_t *tx, uint8_t *rx, size_t size) {
          if (size > SPI_BUF_SIZE) {
            return -1;
          }
          memcpy(tx_buf, tx, size);
          memset(rx_buf, 0xcd, size);
          spi_ioc_transfer transfer = {};
          transfer.tx_buf = (uint64_t)tx_buf;
          transfer.rx_buf = (uint64_t)rx_buf;
          transfer.len = static_cast<uint32_t>(size);
          const int result = lltransfer(transfer);
          memcpy(rx, rx_buf, size);
          return result;
        }, new_spi_session_id());
    }
    return true;
  };

  // VERSION is deliberately outside both framed protocols. Its response can
  // begin while the request itself is still being clocked, so preserve those
  // MISO bytes in the same decoder used for every subsequent poll.
  wait_for_spi_turnaround(spi_last_bus_activity_ns);
  memcpy(tx_buf, panda::spi::kVersionRequest.data(), panda::spi::kVersionRequest.size());
  memset(rx_buf, 0xcd, panda::spi::kVersionRequest.size());
  spi_ioc_transfer request_transfer = {};
  request_transfer.tx_buf = (uint64_t)tx_buf;
  request_transfer.rx_buf = (uint64_t)rx_buf;
  request_transfer.len = static_cast<uint32_t>(panda::spi::kVersionRequest.size());
  if (lltransfer(request_transfer) < 0) {
    spi_last_bus_activity_ns = nanos_since_boot();
    wait_for_spi_turnaround(spi_last_bus_activity_ns);
    return false;
  }
  spi_last_bus_activity_ns = nanos_since_boot();
  if (accept_version(decoder.feed(rx_buf, panda::spi::kVersionRequest.size()))) {
    return true;
  }

  // A legacy v2 Panda rearms HEADER RX as soon as its fixed VERSION TX DMA
  // completes. A wide filler transfer can therefore clock past the response
  // CRC and inject a fake header. Poll one byte per CS and stop on the exact
  // byte that completes the packet. The bounded budget can also drain one
  // maximum v3 response abandoned by a dead previous host, plus enough bytes
  // for Panda to service and return this VERSION request.
  constexpr auto version_poll_pacing = std::chrono::microseconds(100);
  for (size_t poll = 0U; poll < panda::spi::kVersionMaxPollBytes; ++poll) {
    std::this_thread::sleep_for(version_poll_pacing);
    wait_for_spi_turnaround(spi_last_bus_activity_ns);

    tx_buf[0] = 0xcdU;
    rx_buf[0] = 0xcdU;
    spi_ioc_transfer poll_transfer = {};
    poll_transfer.tx_buf = (uint64_t)tx_buf;
    poll_transfer.rx_buf = (uint64_t)rx_buf;
    poll_transfer.len = 1U;
    if (lltransfer(poll_transfer) < 0) {
      spi_last_bus_activity_ns = nanos_since_boot();
      wait_for_spi_turnaround(spi_last_bus_activity_ns);
      return false;
    }
    spi_last_bus_activity_ns = nanos_since_boot();
    if (accept_version(decoder.feed(rx_buf, 1U))) {
      return true;
    }
  }
  wait_for_spi_turnaround(spi_last_bus_activity_ns);
  return false;
}
