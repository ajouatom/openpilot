#include "system/camerad/cameras/camera_common.h"

#include <cassert>

#include "common/params.h"
#include "common/util.h"

int main(int argc, char *argv[]) {
  // Trial: separate camera request handling from card's FIFO53 work on core6.
  // Keep normal scheduling below planner/radard on core5; UI uses cores0..3.
  // Camera IRQ placement in Tici.set_power_save must use the same core.
  int ret = util::set_core_affinity({5});
  assert(ret == 0 || Params().getBool("IsOffroad")); // failure ok while offroad due to offlining cores

  camerad_thread();
  return 0;
}
