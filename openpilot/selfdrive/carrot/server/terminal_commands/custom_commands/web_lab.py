from __future__ import annotations

from ...services.web_capabilities import resolve_web_capabilities, set_web_capability_enabled
from ...services.web_settings import read_web_settings
from ..registry import register_command


WEB_LAB_CAPABILITY = "web_lab"


def _print_status() -> None:
  enabled = resolve_web_capabilities(read_web_settings())[WEB_LAB_CAPABILITY]
  print(f"[web-lab] {'on' if enabled else 'off'}")


@register_command(
  name="web-lab",
  summary="Carrot Web 실험 기능의 잠금을 관리합니다.",
  usage="carrot web-lab <on|off|status>",
)
def run(args: list[str]) -> int:
  action = str(args[0] if args else "status").strip().lower()
  if action in {"status", "-s"}:
    _print_status()
    return 0
  if action in {"on", "enable"}:
    set_web_capability_enabled(WEB_LAB_CAPABILITY, True)
    print("[web-lab] on - 실험 기능 잠금이 해제되었습니다.")
    return 0
  if action in {"off", "disable"}:
    set_web_capability_enabled(WEB_LAB_CAPABILITY, False)
    print("[web-lab] off - 실험 기능을 끄고 잠갔습니다.")
    return 0

  print("사용법: carrot web-lab <on|off|status>")
  return 2
