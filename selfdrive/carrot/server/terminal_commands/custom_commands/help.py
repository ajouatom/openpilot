from __future__ import annotations

from ..bridge import META_COMMAND_PREFIX
from ..registry import get_command, iter_commands, register_command


@register_command(
  name="help",
  summary="List web terminal meta commands or show command usage.",
  usage=":help [command]",
)
def run(args: list[str]) -> int:
  if args:
    command = get_command(args[0])
    if command is None:
      print(f"[terminal] unknown command: {args[0]}")
      return 2
    print(f"{command.usage}\n  {command.summary}")
    return 0

  print("Web terminal meta commands")
  for command in iter_commands():
    print(f"  {META_COMMAND_PREFIX}{command.name:<18} {command.summary}")
  print("\nRun :help <command> for usage. Other input is sent to the shell unchanged.")
  return 0
