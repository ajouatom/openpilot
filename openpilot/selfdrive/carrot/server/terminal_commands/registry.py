from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Callable, Iterable


CommandHandler = Callable[[list[str]], int | None]
_COMMAND_NAME_RE = re.compile(r"^[a-z][a-z0-9_]*$")


@dataclass(frozen=True)
class TerminalCommand:
  name: str
  summary: str
  usage: str
  handler: CommandHandler


_commands: dict[str, TerminalCommand] = {}


def register_command(*, name: str, summary: str, usage: str) -> Callable[[CommandHandler], CommandHandler]:
  normalized = str(name or "").strip().lower()
  if not _COMMAND_NAME_RE.fullmatch(normalized):
    raise ValueError(f"invalid terminal command name: {name!r}")

  def decorator(handler: CommandHandler) -> CommandHandler:
    if normalized in _commands:
      raise ValueError(f"duplicate terminal command: {normalized}")
    _commands[normalized] = TerminalCommand(
      name=normalized,
      summary=str(summary or "").strip(),
      usage=str(usage or "").strip(),
      handler=handler,
    )
    return handler

  return decorator


def get_command(name: str) -> TerminalCommand | None:
  return _commands.get(str(name or "").strip().lower())


def iter_commands() -> Iterable[TerminalCommand]:
  return sorted(_commands.values(), key=lambda command: command.name)
