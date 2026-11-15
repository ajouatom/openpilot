#include "kernel.h"
#include "openpilot-panda-spi-linux/src/bridge.rs.h"
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <limits>
#include <linux/spi/spidev.h>
#include <sched.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace panda_spi_kernel {
namespace {
Call status(int result, int fd, unsigned long request, const void *argument) {
  return {result, errno, fd, static_cast<uint64_t>(request), reinterpret_cast<uint64_t>(argument)};
}
}
Handle::~Handle() { close(); }
bool Handle::exists() const { struct stat info{}; return stat(path_.c_str(), &info) != -1; }
int Handle::open() {
  if (fd_ >= 0) { errno = EALREADY; return -1; }
  fd_ = ::open(path_.c_str(), O_RDWR);
  return fd_;
}
Call Handle::open_call() { const int result = open(); return status(result, fd_, 0, nullptr); }
OptionCall Handle::read_option(uint8_t option) {
  uint8_t byte = 0;
  uint32_t word = 0;
  unsigned long request;
  void *argument;
  if (option == 0) { request = SPI_IOC_RD_MODE; argument = &byte; }
  else if (option == 1) { request = SPI_IOC_RD_MAX_SPEED_HZ; argument = &word; }
  else if (option == 2) { request = SPI_IOC_RD_BITS_PER_WORD; argument = &byte; }
  else { errno = EINVAL; return {status(-1, fd_, 0, nullptr), 0}; }
  const int result = ioctl(fd_, request, argument);
  return {status(result, fd_, request, argument), option == 1 ? word : byte};
}
Call Handle::configure(uint8_t option, uint32_t value) {
  unsigned long request;
  if (option == 0) request = SPI_IOC_WR_MODE;
  else if (option == 1) request = SPI_IOC_WR_MAX_SPEED_HZ;
  else if (option == 2) request = SPI_IOC_WR_BITS_PER_WORD;
  else { errno = EINVAL; return status(-1, fd_, 0, nullptr); }
  if (option == 2) {
    uint8_t byte = static_cast<uint8_t>(value);
    const int result = ioctl(fd_, request, &byte);
    return status(result, fd_, request, &byte);
  }
  const int result = ioctl(fd_, request, &value);
  return status(result, fd_, request, &value);
}
Call Handle::transfer(rust::Slice<const uint8_t> tx, rust::Slice<uint8_t> rx) {
  if (tx.size() != rx.size() || tx.size() > std::numeric_limits<uint32_t>::max()) {
    errno = EINVAL; return status(-1, fd_, SPI_IOC_MESSAGE(1), nullptr);
  }
  spi_ioc_transfer transfer{};
  static_assert(sizeof(spi_ioc_transfer) == 32);
  transfer.tx_buf = reinterpret_cast<uint64_t>(tx.data());
  transfer.rx_buf = reinterpret_cast<uint64_t>(rx.data());
  transfer.len = static_cast<uint32_t>(tx.size());
  const int result = ioctl(fd_, SPI_IOC_MESSAGE(1), &transfer);
  return status(result, fd_, SPI_IOC_MESSAGE(1), &transfer);
}
Call Handle::transfer_at_speed(rust::Slice<const uint8_t> tx, rust::Slice<uint8_t> rx, uint32_t speed, uint8_t bits) {
  if (tx.size() != rx.size() || tx.size() > std::numeric_limits<uint32_t>::max()) {
    errno = EINVAL; return status(-1, fd_, SPI_IOC_MESSAGE(1), nullptr);
  }
  spi_ioc_transfer transfer{};
  transfer.tx_buf = reinterpret_cast<uint64_t>(tx.data());
  transfer.rx_buf = reinterpret_cast<uint64_t>(rx.data());
  transfer.len = static_cast<uint32_t>(tx.size());
  transfer.speed_hz = speed;
  transfer.bits_per_word = bits;
  const int result = ioctl(fd_, SPI_IOC_MESSAGE(1), &transfer);
  return status(result, fd_, SPI_IOC_MESSAGE(1), &transfer);
}
Call Handle::read_bytes(rust::Slice<uint8_t> rx) {
  if (rx.size() > static_cast<size_t>(std::numeric_limits<int>::max())) { errno = EINVAL; return status(-1, fd_, 0, nullptr); }
  const int result = static_cast<int>(::read(fd_, rx.data(), rx.size()));
  return status(result, fd_, 0, rx.data());
}
Call Handle::write_bytes(rust::Slice<const uint8_t> tx) {
  if (tx.size() > static_cast<size_t>(std::numeric_limits<int>::max())) { errno = EINVAL; return status(-1, fd_, 0, nullptr); }
  const int result = static_cast<int>(::write(fd_, tx.data(), tx.size()));
  return status(result, fd_, 0, tx.data());
}
Call Handle::firmware_transfer(uint8_t endpoint, rust::Slice<const uint8_t> tx, rust::Slice<uint8_t> rx, bool disconnect) {
  struct FirmwareTransfer {
    uint64_t rx_buf;
    uint64_t tx_buf;
    uint32_t tx_length;
    uint32_t rx_length_max;
    uint32_t timeout;
    uint8_t endpoint;
    uint8_t expect_disconnect;
  };
  static_assert(sizeof(FirmwareTransfer) == 32);
  static_assert(offsetof(FirmwareTransfer, timeout) == 24);
  static_assert(offsetof(FirmwareTransfer, endpoint) == 28);
  if (tx.size() > std::numeric_limits<uint32_t>::max() || rx.size() > static_cast<size_t>(std::numeric_limits<int>::max())) {
    errno = EINVAL; return status(-1, fd_, SPI_IOC_RD_LSB_FIRST, nullptr);
  }
  FirmwareTransfer transfer{};
  transfer.rx_buf = reinterpret_cast<uint64_t>(rx.data());
  transfer.tx_buf = reinterpret_cast<uint64_t>(tx.data());
  transfer.tx_length = static_cast<uint32_t>(tx.size());
  transfer.rx_length_max = static_cast<uint32_t>(rx.size());
  transfer.endpoint = endpoint;
  transfer.expect_disconnect = disconnect;
  const int result = ioctl(fd_, SPI_IOC_RD_LSB_FIRST, &transfer);
  return status(result, fd_, SPI_IOC_RD_LSB_FIRST, &transfer);
}
Call Handle::flock_call(bool exclusive) {
  const int result = ::flock(fd_, exclusive ? LOCK_EX : LOCK_UN);
  return status(result, fd_, 0, nullptr);
}
void Handle::flock(bool exclusive) { ::flock(fd_, exclusive ? LOCK_EX : LOCK_UN); }
void Handle::close() { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }
std::unique_ptr<Handle> create(const std::string &path) { return std::make_unique<Handle>(path); }
// SPI timing only measures intervals (turnaround, ACK and recovery deadlines),
// so the monotonic clock serves: the comma never suspends while pandad runs.
// The kernel (4.9) reads CLOCK_BOOTTIME by a system call but CLOCK_MONOTONIC
// through the vDSO, and the turnaround and ACK loops read the clock thousands
// of times a second (strace: ~5,000 clock_gettime calls/s across the CAN threads).
uint64_t now_ns() { timespec time{}; clock_gettime(CLOCK_MONOTONIC, &time); return static_cast<uint64_t>(time.tv_sec) * 1000000000ULL + time.tv_nsec; }
void yield_now() { sched_yield(); }
void sleep_us(uint32_t micros) { usleep(micros); }
void sleep_until_ns(uint64_t deadline) {
  const timespec target{static_cast<time_t>(deadline / 1000000000ULL),
                        static_cast<long>(deadline % 1000000000ULL)};
  // Retry the same absolute target on EINTR. Other errors fall back to Rust's
  // clock-checked precision loop, preserving the minimum quiet interval.
  while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &target, nullptr) == EINTR) {}
}
Call set_scheduler(int32_t policy, int32_t priority) {
  sched_param parameter{};
  parameter.sched_priority = priority;
  return status(sched_setscheduler(0, policy, &parameter), -1, 0, nullptr);
}
std::unique_ptr<std::string> errno_description(int error) { return std::make_unique<std::string>(strerror(error)); }
double parse_probability(const std::string &value) { return std::stod(value); }
uint32_t random() { static_assert(RAND_MAX == 2147483647); return rand(); }
void diagnostic_print(rust::Str text) { fwrite(text.data(), 1, text.size(), stdout); }
}
