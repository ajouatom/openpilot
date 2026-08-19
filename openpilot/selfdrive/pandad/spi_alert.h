#pragma once

#include <algorithm>
#include <cstdint>
#include <deque>

class PandaSpiAlertTracker {
public:
  static constexpr uint64_t ONROAD_ARM_DELAY_MS = 5000U;
  static constexpr uint64_t RECOVERED_BURST_WINDOW_MS = 10000U;
  static constexpr uint32_t RECOVERED_BURST_THRESHOLD = 3U;
  static constexpr uint64_t CONFIRM_DELAY_MS = 1000U;

  void update_onroad(bool is_onroad, uint64_t now_ms) {
    if (!is_onroad) {
      reset();
      return;
    }

    if (!is_onroad_) {
      reset();
      is_onroad_ = true;
      onroad_since_ms_ = now_ms;
    }
  }

  bool observe(uint64_t now_ms, uint64_t event_count, bool terminal_failure) {
    if (!armed(now_ms) || capture_requested_ || event_count == 0U) {
      return false;
    }

    prune_recovered_events(now_ms);
    if (!terminal_failure) {
      const uint64_t capped_count = std::min<uint64_t>(event_count, RECOVERED_BURST_THRESHOLD);
      for (uint64_t i = 0U; i < capped_count; ++i) {
        recovered_event_times_ms_.push_back(now_ms);
      }
      if (recovered_event_times_ms_.size() < RECOVERED_BURST_THRESHOLD) {
        return false;
      }
    }

    if (!pending_) {
      pending_ = true;
      pending_since_ms_ = now_ms;
    }
    return true;
  }

  bool ready(uint64_t now_ms) const {
    return armed(now_ms) && pending_ && !capture_requested_ &&
           now_ms - pending_since_ms_ >= CONFIRM_DELAY_MS;
  }

  void mark_capture_requested() {
    capture_requested_ = true;
    pending_ = false;
    recovered_event_times_ms_.clear();
  }

private:
  bool armed(uint64_t now_ms) const {
    return is_onroad_ && now_ms - onroad_since_ms_ >= ONROAD_ARM_DELAY_MS;
  }

  void prune_recovered_events(uint64_t now_ms) {
    while (!recovered_event_times_ms_.empty() &&
           now_ms - recovered_event_times_ms_.front() > RECOVERED_BURST_WINDOW_MS) {
      recovered_event_times_ms_.pop_front();
    }
  }

  void reset() {
    is_onroad_ = false;
    onroad_since_ms_ = 0U;
    pending_ = false;
    pending_since_ms_ = 0U;
    capture_requested_ = false;
    recovered_event_times_ms_.clear();
  }

  bool is_onroad_ = false;
  uint64_t onroad_since_ms_ = 0U;
  bool pending_ = false;
  uint64_t pending_since_ms_ = 0U;
  bool capture_requested_ = false;
  std::deque<uint64_t> recovered_event_times_ms_;
};
