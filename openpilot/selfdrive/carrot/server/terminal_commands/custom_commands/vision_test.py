from __future__ import annotations

from ...services import vision_test
from ..registry import register_command


def _print_help() -> None:
  print("사용법: carrot vision <start|status|logs|stop> [--lines N]")
  print("  start   카메라 확인 시작")
  print("  status  현재 준비 상태 확인")
  print("  logs    최근 확인 로그 표시")
  print("  stop    카메라 확인 중지")


@register_command(
  name="vision",
  summary="주차 상태에서 당근 비전 카메라를 확인합니다.",
  usage="carrot vision <start|status|logs|stop> [--lines N]",
)
def run(args: list[str]) -> int:
  if not args or args[0].lower() in {"help", "-h", "--help"}:
    _print_help()
    return 0

  aliases = {"on": "start", "off": "stop", "log": "logs"}
  action = aliases.get(args[0].lower(), args[0].lower())
  return vision_test.run_command([action, *args[1:]])


@register_command(
  name="vision_on",
  summary="Legacy alias for carrot vision start.",
  usage="carrot vision start",
  hidden=True,
)
def run_legacy_on(args: list[str]) -> int:
  return vision_test.run_command(["start", *args])


@register_command(
  name="vision_off",
  summary="Legacy alias for carrot vision stop.",
  usage="carrot vision stop",
  hidden=True,
)
def run_legacy_off(args: list[str]) -> int:
  return vision_test.run_command(["stop", *args])
