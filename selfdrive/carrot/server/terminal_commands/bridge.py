from __future__ import annotations

import shlex


META_COMMAND_PREFIX = ":"
_CLI_MODULE = "selfdrive.carrot.server.terminal_commands.cli"


def translate_meta_command(line: str) -> str | None:
  """Translate a web-terminal-only meta command into the fixed CLI bridge."""
  stripped = str(line or "").strip()
  if not stripped.startswith(META_COMMAND_PREFIX):
    return None

  command_line = stripped[len(META_COMMAND_PREFIX):].strip() or "help"
  return shlex.join(["python3", "-m", _CLI_MODULE, "--line", command_line])
