#pragma once

#include <cassert>
#include <cerrno>
#include <condition_variable>
#include <exception>
#include <functional>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <utility>
#include <sys/eventfd.h>
#include <unistd.h>

// Only the kernel fence wait runs on this worker. Buffer ownership, recovery,
// exposure, timestamp synchronization and publishing stay on camerad's thread.
class AsyncFrameWait {
public:
  AsyncFrameWait() {
    fd_ = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
    if (fd_ < 0) throw std::system_error(errno, std::generic_category(), "camera wait eventfd");
    try {
      worker_ = std::thread([this] { run(); });
    } catch (...) {
      close(fd_);
      throw;
    }
  }
  ~AsyncFrameWait() {
    {
      std::lock_guard lock(mutex_);
      stopping_ = true;
    }
    ready_.notify_one();
    worker_.join();  // The camera and sync objects outlive the in-flight wait.
    close(fd_);
  }
  AsyncFrameWait(const AsyncFrameWait &) = delete;
  AsyncFrameWait &operator=(const AsyncFrameWait &) = delete;

  int fd() const { return fd_; }
  bool busy() const { return busy_; }  // owner thread only

  void submit(std::function<bool()> wait) {
    assert(!busy_);
    {
      std::lock_guard lock(mutex_);
      work_ = std::move(wait);
      busy_ = true;
    }
    ready_.notify_one();
  }

  std::optional<bool> take() {
    uint64_t value;
    ssize_t ret;
    do { ret = read(fd_, &value, sizeof(value)); } while (ret < 0 && errno == EINTR);
    if (ret < 0 && errno == EAGAIN) return std::nullopt;
    if (ret != static_cast<ssize_t>(sizeof(value))) throw std::runtime_error("camera wait eventfd read");
    std::lock_guard lock(mutex_);
    busy_ = false;
    if (error_) std::rethrow_exception(std::exchange(error_, nullptr));
    return result_;
  }

private:
  void run() {
    while (true) {
      std::function<bool()> work;
      {
        std::unique_lock lock(mutex_);
        ready_.wait(lock, [this] { return stopping_ || work_; });
        if (stopping_) return;
        work = std::move(work_);
        work_ = nullptr;
      }
      bool result = false;
      std::exception_ptr error;
      try { result = work(); } catch (...) { error = std::current_exception(); }
      {
        std::lock_guard lock(mutex_);
        result_ = result;
        error_ = error;
      }
      uint64_t value = 1;
      ssize_t ret;
      do { ret = write(fd_, &value, sizeof(value)); } while (ret < 0 && errno == EINTR);
      if (ret != static_cast<ssize_t>(sizeof(value))) std::terminate();
    }
  }

  int fd_ = -1;
  bool busy_ = false;
  bool stopping_ = false;
  bool result_ = false;
  std::exception_ptr error_;
  std::function<bool()> work_;
  std::mutex mutex_;
  std::condition_variable ready_;
  std::thread worker_;
};
