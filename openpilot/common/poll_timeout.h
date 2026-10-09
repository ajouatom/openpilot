#pragma once

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <poll.h>

// poll's relative timeout starts over after EINTR. Messaging wakeups can arrive
// faster than that timeout forever, so retain the original monotonic deadline.
inline int poll_with_timeout(struct pollfd *fds, nfds_t count, int timeout_ms) {
  using clock = std::chrono::steady_clock;
  const auto deadline = clock::now() + std::chrono::milliseconds(std::max(timeout_ms, 0));
  int remaining_ms = timeout_ms;
  while (true) {
    int ret = poll(fds, count, remaining_ms);
    if (ret >= 0 || errno != EINTR) return ret;
    if (timeout_ms >= 0) {
      const auto remaining = deadline - clock::now();
      if (remaining <= clock::duration::zero()) return 0;
      remaining_ms = std::chrono::ceil<std::chrono::milliseconds>(remaining).count();
    }
  }
}
