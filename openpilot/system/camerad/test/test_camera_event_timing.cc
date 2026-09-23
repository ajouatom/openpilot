#define CATCH_CONFIG_MAIN
#include "catch2/catch.hpp"

#include "system/camerad/cameras/camera_event_timing.h"

TEST_CASE("normal camera cadence does not log") {
  CameraEventTiming timing;
  for (uint64_t sof = 1000000000; sof < 10000000000; sof += 50000000) {
    REQUIRE_FALSE(timing.observe(sof, sof + 30000000).report);
  }
}

TEST_CASE("long SOF interval and late userspace arrival remain distinguishable") {
  CameraEventTiming timing;
  timing.observe(1000000000, 1048000000);

  // Ioniq 5 594--7: consecutive frame/request IDs with a 102.044 ms SOF gap.
  auto gap = timing.observe(1102044320, 1146931320);
  REQUIRE(gap.report);
  REQUIRE(gap.sof_delta_ns == 102044320);
  REQUIRE(gap.event_age_ns == 44887000);

  CameraEventTiming late;
  late.observe(1000000000, 1030000000);
  auto queued = late.observe(1050000000, 1140000000);
  REQUIRE(queued.report);
  REQUIRE(queued.sof_delta_ns == 50000000);
  REQUIRE(queued.event_age_ns == 90000000);
}

TEST_CASE("persistent stalls have bounded logging per camera") {
  CameraEventTiming timing;
  REQUIRE(timing.observe(1000000000, 1100000000).report);
  for (uint64_t i = 1; i < 20; ++i) {
    REQUIRE_FALSE(timing.observe(1000000000 + i * 50000000, 1100000000 + i * 50000000).report);
  }
  auto next = timing.observe(2000000000, 2100000000);
  REQUIRE(next.report);
  REQUIRE(next.suppressed == 19);
  REQUIRE_FALSE(timing.observe(2050000000, 2080000000).report);
}

TEST_CASE("startup and repeated or backward SOF cannot underflow the delta") {
  CameraEventTiming timing;
  REQUIRE(timing.observe(1000000000000, 1000030000000).sof_delta_ns == 0);
  auto repeated = timing.observe(1000000000000, 1000040000000);
  REQUIRE(repeated.sof_delta_ns == 0);
  REQUIRE_FALSE(repeated.report);
  auto backwards = timing.observe(999999000000, 1000050000000);
  REQUIRE(backwards.sof_delta_ns == 0);
  REQUIRE_FALSE(backwards.report);
  auto future = timing.observe(1000050000000, 1000040000000);
  REQUIRE(future.event_age_ns == 0);
  REQUIRE_FALSE(future.report);
}
