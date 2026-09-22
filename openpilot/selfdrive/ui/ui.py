#!/usr/bin/env python3
import gc

from openpilot.system.hardware import TICI
from openpilot.common.realtime import set_core_affinity
from openpilot.common.display_scheduling import DisplayScheduler
from openpilot.selfdrive.ui.carrot_ui_sched import ensure_ui_sched_other
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.layouts.main import MainLayout
from openpilot.selfdrive.ui.mici.layouts.main import MiciMainLayout
from openpilot.selfdrive.ui.ui_state import ui_state

BIG_UI = gui_app.big_ui()


def main():
  # Share core6 only onroad, below camera and realtime control work.
  scheduler = DisplayScheduler(6, enabled=TICI)
  # GC는 계속 끈다 — 기존 config_realtime_process가 하던 GC pause(프레임
  # 히치) 방지는 유지해야 한다.
  gc.disable()
  # Offroad power-save offlines cores4..7; bootstrap on always-online core0.
  set_core_affinity([0])
  # SCHED_OTHER 계약 명시 적용 + readback 검증 (실패는 fail-stop)
  ensure_ui_sched_other()
  scheduler.update(False, force=True)

  gui_app.init_window("UI")
  if BIG_UI:
    MainLayout()
  else:
    MiciMainLayout()

  for _ in gui_app.render():
    ui_state.update()
    scheduler.update(ui_state.started)


if __name__ == "__main__":
  main()
