"""carrot 전용: UI 스케줄러 계약 — 상시 SCHED_OTHER를 명시 적용하고 검증한다.

UI 목표는 SCHED_OTHER/core5 (offroad는 core0 부트스트랩 — ui.py 소관) — 비RT
UI는 core5의 radard(FIFO51)를 절대 선점하지 못하므로 주행 프로세스가
항상 우선한다. 기존 FIFO51 UI는 radar와 같은 우선순위로 core5를
점유해 상시 렌더 부하(DebugPlot 등)에서 20Hz cadence를 위협했다. core7은
modeld(FIFO54)+plannerd(FIFO51)+dmonitoringmodeld(FIFO5) 전용이라 UI 재배치 금지이고, 이
branch에는 FIFO 승격/복구 경로 자체가 없어야 한다 — 재도입 금지.

정상 manager launch는 SCHED_OTHER를 상속하므로 사실상 no-op 검증이지만,
계약을 명시 적용(drop)하고 readback해 어떤 경로로든 RT로 시작된 UI가 그대로
실행되는 것을 막는다 (시작 시 1회 — 매 프레임 syscall 금지).
"""
import os
import sys

from openpilot.common.realtime import drop_realtime
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware import PC

_VERIFY_ATTEMPTS = 2  # bounded 재시도 — 무한/매 프레임 syscall 폭주 금지


def ensure_ui_sched_other() -> None:
  """UI 메인 스레드를 SCHED_OTHER로 명시 강등하고 readback으로 검증한다.

  검증 실패는 fail-stop — RT policy UI가 core5의 radar를 굶기며 계속
  실행되는 것보다 manager 재시작(restart_if_crash)이 낫다 (fail-closed).
  readback이 SCHED_OTHER가 아니면 성공 로그를 내지 않는다 (false success 금지)."""
  if sys.platform != "linux" or PC:
    return  # RT 스케줄링이 없는 환경 — 계약 자체가 불필요
  policy = None
  for _ in range(_VERIFY_ATTEMPTS):
    try:
      drop_realtime()
      policy = os.sched_getscheduler(0)
    except OSError:
      continue
    if policy == os.SCHED_OTHER:
      cloudlog.info("UISCHED: UI SCHED_OTHER verified (core0 bootstrap)")
      return
  cloudlog.critical(f"UISCHED: UI could not be verified SCHED_OTHER (policy={policy}); fail-stop")
  raise RuntimeError("UI must run SCHED_OTHER (radar preemption contract)")
