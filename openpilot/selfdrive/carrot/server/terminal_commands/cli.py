from __future__ import annotations

import argparse
import shlex
import sys

from .custom_commands import load_commands
from .registry import get_command


def _parse_command_line(command_line: str) -> list[str] | None:
  try:
    return shlex.split(command_line)
  except ValueError as exc:
    print(f"[terminal] parse error: {exc}", file=sys.stderr)
    return None


def main(argv: list[str] | None = None) -> int:
  parser = argparse.ArgumentParser(add_help=False)
  parser.add_argument("--line", default="help")
  options = parser.parse_args(argv)

  load_commands()
  parts = _parse_command_line(options.line)
  if parts is None:
    return 2
  if not parts:
    parts = ["help"]

  command = get_command(parts[0])
  if command is None:
    print(f"[terminal] unknown command: {parts[0]}", file=sys.stderr)
    print("[terminal] run :help to list available commands", file=sys.stderr)
    return 2

  try:
    result = command.handler(parts[1:])
  except Exception as exc:
    print(f"[terminal] {command.name} failed: {exc}", file=sys.stderr)
    return 1
  return int(result or 0)


if __name__ == "__main__":
  raise SystemExit(main())
