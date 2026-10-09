#undef NDEBUG
#include <cassert>
#include <cstdio>

#include <atomic>
#include <csignal>
#include <pthread.h>
#include <thread>
#include <unistd.h>

#include "common/poll_timeout.h"

void test_signal_storm_respects_deadline() {
  struct sigaction action = {}, previous = {};
  action.sa_handler = [](int) {};
  sigemptyset(&action.sa_mask);
  assert(sigaction(SIGUSR2, &action, &previous) == 0);
  const auto owner = pthread_self();
  std::atomic<bool> stop{false};
  std::thread sender([&] {
    // Finite even with the old implementation, which would wait until this ends.
    for (int i = 0; i < 150 && !stop; ++i) {
      pthread_kill(owner, SIGUSR2);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  });
  const auto start = std::chrono::steady_clock::now();
  const int ret = poll_with_timeout(nullptr, 0, 40);
  const auto elapsed = std::chrono::steady_clock::now() - start;
  stop = true;
  sender.join();
  sigaction(SIGUSR2, &previous, nullptr);
  assert(ret == 0);
  assert(elapsed >= std::chrono::milliseconds(35));
  assert(elapsed < std::chrono::milliseconds(200));
}

void test_nonblocking_and_invalid_fd() {
  int pipes[2];
  assert(pipe(pipes) == 0);
  pollfd fd = {.fd = pipes[0], .events = POLLIN, .revents = 0};
  assert(poll_with_timeout(&fd, 1, 0) == 0);
  char value = 1;
  assert(write(pipes[1], &value, 1) == 1);
  assert(poll_with_timeout(&fd, 1, 0) == 1);
  assert((fd.revents & POLLIN) != 0);
  close(pipes[0]);
  assert(poll_with_timeout(&fd, 1, 10) == 1);
  assert((fd.revents & POLLNVAL) != 0);
  close(pipes[1]);
}

void test_indefinite_wait_returns_ready() {
  int pipes[2];
  assert(pipe(pipes) == 0);
  std::thread sender([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    char value = 1;
    assert(write(pipes[1], &value, 1) == 1);
  });
  pollfd fd = {.fd = pipes[0], .events = POLLIN, .revents = 0};
  const int ret = poll_with_timeout(&fd, 1, -1);
  sender.join();
  assert(ret == 1);
  close(pipes[0]);
  close(pipes[1]);
}

int main() {
  test_signal_storm_respects_deadline();
  test_nonblocking_and_invalid_fd();
  test_indefinite_wait_returns_ready();
  puts("3 native regression cases passed");
}
