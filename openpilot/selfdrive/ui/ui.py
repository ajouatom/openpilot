#!/usr/bin/env python3
import gc
import os

from openpilot.system.hardware import HARDWARE, TICI
from openpilot.common.realtime import set_core_affinity
from openpilot.common.display_scheduling import DisplayScheduler
from openpilot.selfdrive.ui.carrot_ui_sched import ensure_ui_sched_other
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.layouts.main import MainLayout
from openpilot.selfdrive.ui.mici.layouts.main import MiciMainLayout
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.impact_dashcam import ImpactDashcamPrompt

BIG_UI = gui_app.big_ui()


def main():
  # C3/C3X can also use little cores rather than waiting only on core6.
  # C4 retains core6; workers stay SCHED_OTHER/nice19.  The render thread
  # additionally gets a bounded onroad RT slice (see display_scheduling) so
  # background bursts cannot push frames past their 50 ms deadline.
  scheduler = DisplayScheduler(6, enabled=TICI,
                               include_little=TICI and HARDWARE.get_device_type() in ('tici', 'tizi'),
                               rt_budget=TICI,
                               schedtune_boost=TICI and os.getenv('CARROT_UI_SCHEDTUNE', '0') == '1')
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
  impact_prompt = ImpactDashcamPrompt()

  for rendered in gui_app.render():
    ui_state.update()
    scheduler.update(ui_state.started)
    if rendered:
      impact_prompt.render()

  scheduler.update(False, force=True)  # drop the RT slice and restore offroad placement


if __name__ == "__main__":
  main()
