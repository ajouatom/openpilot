#pragma once

#include <cstddef>
#include <cstdint>
#include <utility>

namespace pandad_scheduler {

constexpr uint64_t STATE_PERIOD_FRAMES = 10U;
constexpr uint64_t SERIAL_LOG_PHASE = 0U;
constexpr uint64_t PERIPHERAL_STATE_PHASE = 2U;

struct StateOperationTarget {
  size_t panda_index;
  size_t stage;
};

// Run the same stage for every Panda before advancing to the next stage. This
// keeps multi-Panda service fair and places the later health/heartbeat stages
// closest to the publish boundary.
constexpr StateOperationTarget state_operation_target(size_t operation, size_t panda_count) {
  return {operation % panda_count, operation / panda_count};
}

// Evenly distribute slow Panda transactions across the 100 Hz CAN loop. The
// returned half-open range contains the operations assigned to this frame.
constexpr std::pair<size_t, size_t> operation_range(uint64_t frame,
                                                    size_t operation_count) {
  const uint64_t phase = frame % STATE_PERIOD_FRAMES;
  const size_t begin = (phase * operation_count) / STATE_PERIOD_FRAMES;
  const size_t end = ((phase + 1U) * operation_count) / STATE_PERIOD_FRAMES;
  return {begin, end};
}

constexpr bool state_publish_due(uint64_t frame) {
  return (frame % STATE_PERIOD_FRAMES) == (STATE_PERIOD_FRAMES - 1U);
}

constexpr bool state_cycle_start(uint64_t frame) {
  return (frame % STATE_PERIOD_FRAMES) == 0U;
}

constexpr bool serial_log_poll_due(uint64_t frame) {
  return (frame % STATE_PERIOD_FRAMES) == SERIAL_LOG_PHASE;
}

constexpr bool peripheral_state_due(uint64_t frame) {
  return (frame % 50U) == PERIPHERAL_STATE_PHASE;
}

} // namespace pandad_scheduler
