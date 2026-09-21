#!/usr/bin/env python3
import gc
import os

from openpilot.system.hardware import TICI
from openpilot.common.realtime import set_core_affinity
from openpilot.selfdrive.ui.carrot_ui_sched import ensure_ui_sched_other
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.layouts.main import MainLayout
from openpilot.selfdrive.ui.mici.layouts.main import MiciMainLayout
from openpilot.selfdrive.ui.ui_state import ui_state

BIG_UI = gui_app.big_ui()


def main():
  cores = {0, 1, 2, 3}
  # Keep rendering off the camera/planner core5 and model core7. The UI must
  # remain SCHED_OTHER so realtime sensor, localization and CAN work takes
  # precedence on the little cores. See docs/camera_core5_trial.md.
  # GC는 계속 끈다 — 기존 config_realtime_process가 하던 GC pause(프레임
  # 히치) 방지는 유지해야 한다.
  gc.disable()
  # TICI offroad power-save는 big core4~7을 offline한다 — always_run UI는
  # 항상 online인 core0에서 부트스트랩한다. 렌더 루프에서는 power-save에서도
  # online인 core0~3으로 best-effort re-affine한다 (실패 시 다음 프레임 재시도).
  set_core_affinity([0])
  # SCHED_OTHER 계약 명시 적용 + readback 검증 (실패는 fail-stop)
  ensure_ui_sched_other()

  gui_app.init_window("UI")
  if BIG_UI:
    MainLayout()
  else:
    MiciMainLayout()

  for should_render in gui_app.render():
    ui_state.update()
    if should_render:
      # Keep the render thread on the little cores across power-save transitions.
      if TICI and os.sched_getaffinity(0) != cores:
        try:
          set_core_affinity(list(cores))
        except OSError:
          pass


if __name__ == "__main__":
  main()
