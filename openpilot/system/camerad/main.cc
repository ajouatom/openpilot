#include "system/camerad/cameras/camera_common.h"

#include <cassert>

#include "common/params.h"
#include "common/util.h"

int main(int argc, char *argv[]) {
  // AGNOS isolates cores6..7. Keep camera work off the general-purpose core5,
  // where background reclaim and planner/radard can delay ready frames.
  // Short FIFO53 control/state work shares core6; card runs on core5.
  // Camera keeps normal priority; onroad UI shares core6 at nice19.
  // Camera IRQ placement in Tici.set_power_save must use the same core.
  int ret = util::set_core_affinity({6});
  assert(ret == 0 || Params().getBool("IsOffroad")); // failure ok while offroad due to offlining cores

  camerad_thread();
  return 0;
}
