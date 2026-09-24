#include <array>
#include <iostream>
#include <map>
#include <stdexcept>

#include "system/camerad/sensors/sensor.h"

// Model the documented group hold / frame-boundary latch, not sensor physics.
// Try a frame boundary after every byte write in the real generated payload.
static void check(bool ok, const char *message) {
  if (!ok) throw std::runtime_error(message);
}

int main() {
  OS04C10 sensor;
  const std::array<int, 18> exposures = {2, 255, 256, 511, 512, 2047, 2048, 2252,
    2273, 2298, 2299, 2304, 2308, 2309, 2344, 2349, 2351, 2352};
  int cases = 0;
  for (int old_exp : exposures) {
    for (int new_exp : exposures) {
      for (int gain : {0, 2, 40}) {
        std::map<uint32_t, uint32_t> active = {{0x3501, uint32_t(old_exp >> 8)},
          {0x3502, uint32_t(old_exp & 255)}, {0x3508, 0}, {0x3509, 0x80},
          {0x350c, 0}, {0x350d, 0x80}};
        std::map<uint32_t, uint32_t> held;
        bool holding = false, ended = false, launched = false;
        for (auto reg : sensor.getExposureRegisters(new_exp, gain, true)) {
          if (reg.reg_addr == 0x3208) {
            if (reg.reg_data == 0x00) {
              check(!holding && !ended, "duplicate hold");
              holding = true;
            } else if (reg.reg_data == 0x10) {
              check(holding, "end without hold");
              holding = false; ended = true;
            } else if (reg.reg_data == 0xa0) {
              check(ended && !holding && held.size() == 6, "incomplete exposure/gain launch");
              // Delayed launch commits the recorded set before this latch point.
              for (auto [address, value] : held) active[address] = value;
              launched = true;
            } else {
              throw std::runtime_error("unexpected group command");
            }
          } else if (holding) {
            held[reg.reg_addr] = reg.reg_data;
          } else {
            active[reg.reg_addr] = reg.reg_data;
          }
          const int observed = (active[0x3501] << 8) | active[0x3502];
          check(observed >= sensor.exposure_time_min && observed <= sensor.exposure_time_max,
                "a latch can observe out-of-range exposure");
          check(observed == old_exp || observed == new_exp, "a latch can observe a torn exposure");
          check(active[0x3508] == active[0x350c] && active[0x3509] == active[0x350d],
                "gain channels can latch different updates");
        }
        check((active[0x3501] << 8 | active[0x3502]) == uint32_t(new_exp), "exposure command never committed");
        const uint32_t expected_gain = gain == 0 ? 0x080 : (gain == 2 ? 0x090 : 0x440);
        check((active[0x3508] << 8 | active[0x3509]) == expected_gain, "gain command never committed");
        check(!holding && (!ended || launched), "unfinished grouped update");
        ++cases;
      }
    }
  }
  std::cout << cases << " exposure/gain transitions passed all byte-boundary latch checks\n";
}
