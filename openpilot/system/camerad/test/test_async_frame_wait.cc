#undef NDEBUG
#include <cassert>
#include <cstdio>

#include <atomic>
#include <chrono>
#include <future>
#include <poll.h>

#include "system/camerad/cameras/async_frame_wait.h"

void test_pending_fence_does_not_block_owner() {
  AsyncFrameWait wait;
  std::promise<void> release;
  auto fence = release.get_future();
  wait.submit([&] { fence.wait(); return false; });
  // Hold the driver fence indefinitely. The owner can service road events and
  // check completion without entering that kernel wait or publishing driver data.
  for (int frame = 0; frame < 20; ++frame) {
    assert(wait.busy());
    assert(!wait.take().has_value());
  }
  release.set_value();
  pollfd fd = {.fd = wait.fd(), .events = POLLIN, .revents = 0};
  assert(poll(&fd, 1, 1000) == 1);
  auto result = wait.take();
  assert(result.has_value());
  assert(!*result);
  assert(!wait.busy());
  assert(!wait.take().has_value());
  // Recovery uses the same worker and returns the real successful fence result.
  wait.submit([] { return true; });
  assert(poll(&fd, 1, 1000) == 1);
  assert(wait.take() == true);
}

void test_one_result_per_request() {
  AsyncFrameWait wait;
  pollfd fd = {.fd = wait.fd(), .events = POLLIN, .revents = 0};
  for (int frame = 0; frame < 500; ++frame) {
    wait.submit([frame] { return frame % 3 != 0; });
    assert(poll(&fd, 1, 1000) == 1);
    assert(wait.take() == (frame % 3 != 0));
    assert(!wait.take().has_value());
  }
}

void test_shutdown_joins_wait() {
  std::atomic<bool> finished{false};
  {
    AsyncFrameWait wait;
    std::promise<void> entered;
    auto started = entered.get_future();
    wait.submit([&] {
      entered.set_value();
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
      finished = true;
      return true;
    });
    started.wait();
  }
  assert(finished);
}

void test_exceptions_reach_owner() {
  AsyncFrameWait wait;
  wait.submit([]() -> bool { throw std::runtime_error("fence error"); });
  pollfd fd = {.fd = wait.fd(), .events = POLLIN, .revents = 0};
  assert(poll(&fd, 1, 1000) == 1);
  bool threw = false;
  try { wait.take(); } catch (const std::runtime_error &e) { threw = std::string(e.what()) == "fence error"; }
  assert(threw);
  assert(!wait.busy());
}

int main() {
  test_pending_fence_does_not_block_owner();
  test_one_result_per_request();
  test_shutdown_joins_wait();
  test_exceptions_reach_owner();
  puts("4 native regression cases passed");
}
