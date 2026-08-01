#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

struct libusb_context;
struct libusb_device_handle;

#define TIMEOUT 0
#define SPI_BUF_SIZE 2048


class PandaCommsHandle {
public:
  PandaCommsHandle() {}
  virtual ~PandaCommsHandle() {}
  virtual void cleanup() = 0;

  std::string hw_serial;
  std::atomic<bool> connected = true;
  std::atomic<bool> comms_healthy = true;

  virtual int control_write(uint8_t request, uint16_t param1, uint16_t param2, unsigned int timeout=TIMEOUT) = 0;
  virtual int control_read(uint8_t request, uint16_t param1, uint16_t param2, unsigned char *data, uint16_t length, unsigned int timeout=TIMEOUT) = 0;
  virtual int bulk_write(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT) = 0;
  virtual int bulk_read(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT) = 0;
};

class PandaUsbHandle : public PandaCommsHandle {
public:
  PandaUsbHandle(std::string serial);
  ~PandaUsbHandle();

  int control_write(uint8_t request, uint16_t param1, uint16_t param2, unsigned int timeout=TIMEOUT);
  int control_read(uint8_t request, uint16_t param1, uint16_t param2, unsigned char *data, uint16_t length, unsigned int timeout=TIMEOUT);
  int bulk_write(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT);
  int bulk_read(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT);
  void cleanup();

  static std::vector<std::string> list();

private:
  libusb_context *ctx = nullptr;
  libusb_device_handle *dev_handle = nullptr;
  std::recursive_mutex hw_lock;

  void handle_usb_issue(int err, const char func[]);
};

class PandaSpiHandle : public PandaCommsHandle {
public:
  PandaSpiHandle(std::string serial);
  ~PandaSpiHandle();

  int control_write(uint8_t request, uint16_t param1, uint16_t param2, unsigned int timeout=TIMEOUT);
  int control_read(uint8_t request, uint16_t param1, uint16_t param2, unsigned char *data, uint16_t length, unsigned int timeout=TIMEOUT);
  int bulk_write(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT);
  int bulk_read(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT);
  void cleanup();

  static std::vector<std::string> list();

private:
  enum class TransferPhase : uint8_t {
    IDLE = 0,
    HEADER,
    HEADER_ACK,
    DATA,
    DATA_ACK,
    RESPONSE,
    RECOVERY,
  };

  struct HostDiagnostics {
    uint32_t requested_speed_hz = 0;
    uint32_t actual_speed_hz = 0;
    uint32_t actual_mode = 0;
    uint8_t actual_bits_per_word = 0;

    uint64_t ioctl_count = 0;
    uint64_t ioctl_error_count = 0;
    uint64_t ioctl_short_count = 0;
    uint64_t ioctl_over_1ms_count = 0;
    uint64_t ioctl_over_5ms_count = 0;
    uint64_t ioctl_over_20ms_count = 0;
    uint64_t ioctl_over_100ms_count = 0;
    double ioctl_total_ms = 0.0;
    double ioctl_max_ms = 0.0;
    uint8_t ioctl_max_endpoint = 0;
    TransferPhase ioctl_max_phase = TransferPhase::IDLE;
    uint32_t ioctl_max_len = 0;
    int ioctl_max_ret = 0;

    uint64_t mutex_wait_count = 0;
    uint64_t mutex_over_1ms_count = 0;
    uint64_t mutex_over_5ms_count = 0;
    uint64_t mutex_over_20ms_count = 0;
    uint64_t mutex_over_100ms_count = 0;
    double mutex_total_ms = 0.0;
    double mutex_max_ms = 0.0;
    uint8_t mutex_max_endpoint = 0;

    uint64_t flock_wait_count = 0;
    uint64_t flock_error_count = 0;
    uint64_t flock_over_1ms_count = 0;
    uint64_t flock_over_5ms_count = 0;
    uint64_t flock_over_20ms_count = 0;
    uint64_t flock_over_100ms_count = 0;
    double flock_total_ms = 0.0;
    double flock_max_ms = 0.0;
    uint8_t flock_max_endpoint = 0;

    double last_log_ms = 0.0;
  };

  int spi_fd = -1;
  uint8_t tx_buf[SPI_BUF_SIZE];
  uint8_t rx_buf[SPI_BUF_SIZE];
  inline static std::recursive_mutex hw_lock;
  TransferPhase transfer_phase = TransferPhase::IDLE;
  HostDiagnostics host_diagnostics;

  struct __attribute__((packed)) spi_header {
    uint8_t sync;
    uint8_t endpoint;
    uint16_t tx_len;
    uint16_t max_rx_len;
  };

  int wait_for_ack(uint8_t ack, uint8_t tx, unsigned int timeout, unsigned int length, double deadline);
  int bulk_transfer(uint8_t endpoint, uint8_t *tx_data, uint16_t tx_len, uint8_t *rx_data, uint16_t rx_len, unsigned int timeout);
  int spi_transfer(uint8_t endpoint, uint8_t *tx_data, uint16_t tx_len, uint8_t *rx_data, uint16_t max_rx_len,
                   unsigned int timeout, double deadline);
  int spi_transfer_retry(uint8_t endpoint, uint8_t *tx_data, uint16_t tx_len, uint8_t *rx_data, uint16_t max_rx_len, unsigned int timeout);
  int lltransfer(struct spi_ioc_transfer &t);
  void record_lock_diagnostics(uint8_t endpoint, double mutex_wait_ms, double flock_wait_ms, int flock_ret, int flock_errno);
  void log_host_diagnostics(bool force=false);
  static const char *phase_name(TransferPhase phase);

  spi_header header{};
  uint32_t xfer_count = 0;
};
