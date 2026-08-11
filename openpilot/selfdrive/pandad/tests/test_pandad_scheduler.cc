#define CATCH_CONFIG_MAIN

#include <algorithm>
#include <vector>

#include "catch2/catch.hpp"
#include "selfdrive/pandad/pandad_scheduler.h"

TEST_CASE("state operations cover one cycle without duplication") {
  for (const size_t operation_count : {5U, 10U, 15U, 25U}) {
    std::vector<size_t> visited;
    size_t max_operations_per_frame = 0U;

    for (uint64_t frame = 0U; frame < pandad_scheduler::STATE_PERIOD_FRAMES;
         frame++) {
      const auto [begin, end] =
          pandad_scheduler::operation_range(frame, operation_count);
      REQUIRE(begin <= end);
      REQUIRE(end <= operation_count);
      max_operations_per_frame =
          std::max(max_operations_per_frame, end - begin);
      for (size_t operation = begin; operation < end; operation++) {
        visited.push_back(operation);
      }
    }

    REQUIRE(visited.size() == operation_count);
    for (size_t operation = 0U; operation < operation_count; operation++) {
      REQUIRE(visited[operation] == operation);
    }
    REQUIRE(max_operations_per_frame <= (operation_count + 9U) / 10U);
  }
}

TEST_CASE("one and two Panda state work stays bounded") {
  std::vector<uint64_t> one_panda_phases;
  for (uint64_t frame = 0U; frame < pandad_scheduler::STATE_PERIOD_FRAMES;
       frame++) {
    const auto [begin, end] = pandad_scheduler::operation_range(frame, 5U);
    if (begin != end) {
      one_panda_phases.push_back(frame);
    }
  }
  REQUIRE((one_panda_phases == std::vector<uint64_t>{1U, 3U, 5U, 7U, 9U}));

  for (uint64_t frame = 0U; frame < pandad_scheduler::STATE_PERIOD_FRAMES;
       frame++) {
    const auto [begin, end] = pandad_scheduler::operation_range(frame, 10U);
    REQUIRE(end - begin == 1U);
  }
}

TEST_CASE("multi-Panda operations are round-robin and stage ordered") {
  constexpr size_t stages_per_panda = 5U;
  for (size_t panda_count = 1U; panda_count <= 3U; panda_count++) {
    std::vector<std::vector<size_t>> visits(
        stages_per_panda, std::vector<size_t>(panda_count, 0U));
    for (size_t operation = 0U; operation < panda_count * stages_per_panda;
         operation++) {
      const auto target =
          pandad_scheduler::state_operation_target(operation, panda_count);
      REQUIRE(target.panda_index == operation % panda_count);
      REQUIRE(target.stage == operation / panda_count);
      REQUIRE(target.stage < stages_per_panda);
      visits[target.stage][target.panda_index]++;
    }

    for (const auto &stage_visits : visits) {
      for (const size_t count : stage_visits) {
        REQUIRE(count == 1U);
      }
    }
  }
}

TEST_CASE("slow service phases are stable") {
  for (uint64_t frame = 0U; frame < 100U; frame++) {
    REQUIRE(pandad_scheduler::state_cycle_start(frame) ==
            ((frame % 10U) == 0U));
    REQUIRE(pandad_scheduler::state_publish_due(frame) ==
            ((frame % 10U) == 9U));
    REQUIRE(pandad_scheduler::serial_log_poll_due(frame) ==
            ((frame % 10U) == 0U));
    REQUIRE(pandad_scheduler::peripheral_state_due(frame) ==
            ((frame % 50U) == 2U));
  }
}
