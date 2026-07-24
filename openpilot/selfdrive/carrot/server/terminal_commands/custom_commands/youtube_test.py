from __future__ import annotations

from ...services import youtube_test
from ..registry import register_command


@register_command(
  name="youtube-test",
  summary="Run the YouTube camera pipeline while the device is offroad.",
  usage="carrot youtube-test [verify|start|status|logs|stop] [--lines N]",
)
def run(args: list[str]) -> int:
  return youtube_test.run_command(args)
