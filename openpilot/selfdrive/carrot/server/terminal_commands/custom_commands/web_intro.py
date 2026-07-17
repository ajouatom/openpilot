from __future__ import annotations

from ..registry import register_command


# The web terminal consumes this line before xterm renders it. Keeping the
# action in terminal output means the command cannot write device state and the
# browser still opens the real, already-loaded intro implementation.
WEB_INTRO_ACTION = "[[CARROT_WEB_ACTION:web-intro]]"


@register_command(
  name="web-intro",
  summary="실제 설치 인트로를 저장 없는 미리보기로 엽니다.",
  usage="carrot web-intro",
)
def run(args: list[str]) -> int:
  if args:
    print("사용법: carrot web-intro")
    return 2

  print("[web-intro] 실제 인트로를 미리보기로 엽니다. 변경사항은 저장되지 않습니다.")
  print(WEB_INTRO_ACTION)
  return 0
