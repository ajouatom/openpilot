from __future__ import annotations

from ..registry import get_command, iter_commands, register_command


@register_command(
  name="help",
  summary="사용 가능한 Carrot 명령과 사용법을 보여줍니다.",
  usage="carrot help [command]",
)
def run(args: list[str]) -> int:
  if args:
    command = get_command(args[0])
    if command is None:
      print(f"[terminal] unknown command: {args[0]}")
      return 2
    print(f"{command.usage}\n  {command.summary}")
    return 0

  print("Carrot 명령")
  for command in iter_commands():
    print(f"  carrot {command.name:<14} {command.summary}")
  print("\n자세한 사용법: carrot help <명령>")
  return 0
