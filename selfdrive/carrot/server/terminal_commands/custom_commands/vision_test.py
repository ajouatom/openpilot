from __future__ import annotations

from ...services import vision_test
from ..registry import register_command


@register_command(
  name="vision_test",
  summary="Run an offroad Carrot Vision camera and WebRTC test.",
  usage=":vision_test [start|status|logs|stop] [--lines N]",
)
def run(args: list[str]) -> int:
  return vision_test.run_command(args)
