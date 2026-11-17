#include <cassert>
#include <initializer_list>
#include "CarrotB1/B1Tap.h"

int main() {
  for (auto direction : {b1::Direction::Left, b1::Direction::Right}) {
    const uint8_t report = direction == b1::Direction::Left ? 1 : 4;
    const uint8_t lamp = direction == b1::Direction::Left ? 1 : 2;
    const uint8_t opposite = direction == b1::Direction::Left ? 2 : 1;
    const uint32_t end = direction == b1::Direction::Left ? 347840 : 260180;
    b1::TapFeedback echo;
    assert(echo.accepts(0, 0, direction, false, 0, end));
    assert(!echo.accepts(report, lamp, direction, false, 20000, end));
    assert(echo.accepts(report, lamp, direction, true, 40000, end));
    // Recorded 0x413 retains its lever bit through successive dark flash halves.
    for (uint32_t time = 400000; time <= 2800000; time += 400000) {
      assert(echo.accepts(report, time % 800000 ? 0 : lamp, direction, true, time, end));
    }
    assert(echo.accepts(0, 0, direction, true, 3000000, end));
    assert(!echo.accepts(report, lamp, direction, true, 3100000, end));
    assert(!echo.accepts(0, opposite, direction, true, 3100000, end));
    echo = {};
    assert(!echo.accepts(report, lamp, direction, true, end + 1, end));
    echo = {};
    assert(echo.accepts(report, 0, direction, true, 40000, end));
    assert(echo.accepts(0, 0, direction, true, 80000, end));
    assert(echo.accepts(report, lamp, direction, true, 120000, end));
    assert(!echo.accepts(report == 1 ? 4 : 1, 0, direction, true, 150000, end));
    assert(!echo.accepts(5, lamp, direction, true, 150000, end));
    assert(!echo.accepts(report, 3, direction, true, 150000, end));
  }
}
