from __future__ import annotations

from ...services import quiet
from ..registry import register_command


@register_command(
  name="madmax",
  summary="예약된 터미널 별칭 명령입니다.",
  usage="madmax",
  hidden=True,
)
def run(args: list[str]) -> int:
  if args:
    print("사용법: madmax")
    return 2

  print(f"madmax {'on' if quiet.toggle() else 'off'}")
  return 0
