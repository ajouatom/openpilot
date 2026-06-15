from __future__ import annotations

from ...services import vision_test
from ..registry import register_command


@register_command(
  name="vision_on",
  summary="Start the offroad Carrot Vision camera and WebRTC test.",
  usage=":vision_on",
)
def run_on(args: list[str]) -> int:
  return vision_test.start_test()


@register_command(
  name="vision_off",
  summary="Stop the Carrot Vision camera and WebRTC test.",
  usage=":vision_off",
)
def run_off(args: list[str]) -> int:
  return vision_test.stop_test()
