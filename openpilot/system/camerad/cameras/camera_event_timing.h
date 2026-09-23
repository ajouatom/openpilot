#pragma once

#include <cstdint>

// Passive diagnostics only: BOOT_TS is generated in the kernel SOF handling
// path, not an independent sensor timestamp. These thresholds never reject a
// frame or change camera/model validity.
class CameraEventTiming {
public:
  struct Sample {
    uint64_t sof_delta_ns;
    uint64_t event_age_ns;
    uint64_t suppressed;
    bool report;
  };

  Sample observe(uint64_t sof, uint64_t received) {
    const uint64_t delta = last_sof && sof > last_sof ? sof - last_sof : 0;
    const uint64_t age = received > sof ? received - sof : 0;
    last_sof = sof;

    // At 20 Hz, 75 ms catches a long SOF interval or delayed event handling.
    const bool anomalous = delta > 75000000ULL || age > 75000000ULL;
    const bool report = anomalous && (!last_report || received - last_report >= 1000000000ULL);
    Sample sample{delta, age, suppressed, report};
    if (report) {
      last_report = received;
      suppressed = 0;
    } else if (anomalous) {
      ++suppressed;
    }
    return sample;
  }

private:
  uint64_t last_sof = 0;
  uint64_t last_report = 0;
  uint64_t suppressed = 0;
};
