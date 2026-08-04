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
  cores = {5, }
  # UI는 상시 SCHED_OTHER — 비RT UI는 core5의 radard(FIFO51)를 절대
  # 선점하지 못하므로 주행 프로세스가 항상 우선한다 (기존 FIFO51 UI는
  # radard와 같은 우선순위로 core5를 점유해 20Hz cadence를 위협). FIFO 승격 재도입
  # 금지, core7은 modeld+plannerd+dmonitoringmodeld 전용이라 UI 재배치도 금지.
  # GC는 계속 끈다 — 기존 config_realtime_process가 하던 GC pause(프레임
  # 히치) 방지는 유지해야 한다.
  gc.disable()
  # TICI offroad power-save는 big core4~7을 offline한다 — always_run UI는
  # 항상 online인 core0에서 부트스트랩하고, onroad에서 core5가 online되면
  # 아래 render loop가 best-effort로 re-affine한다 (실패는 삼키고 다음
  # 프레임에 재시도).
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
      # reaffine after power save offlines our core
      if TICI and os.sched_getaffinity(0) != cores:
        try:
          set_core_affinity(list(cores))
        except OSError:
          pass


if __name__ == "__main__":
  main()
