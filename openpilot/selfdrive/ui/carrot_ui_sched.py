"""carrot 전용: UI 스케줄러 계약 — 상시 SCHED_OTHER를 명시 적용하고 검증한다.

UI 목표는 SCHED_OTHER/core0~3 (core0 부트스트랩 — ui.py 소관)이다.
센서·위치 추정·CAN의 실시간 작업이 UI보다 우선해야 한다. 카메라·플래너의
core5와 모델의 core7에는 UI를 배치하지 않는다. FIFO 승격/복구 재도입 금지.

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

  검증 실패는 fail-stop — RT policy UI가 센서·위치 추정을 굶기며 계속
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
  raise RuntimeError("UI must run SCHED_OTHER (realtime preemption contract)")
