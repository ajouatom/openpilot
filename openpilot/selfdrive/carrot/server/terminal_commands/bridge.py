from __future__ import annotations

import os
import shlex
from pathlib import Path


# Double-colon so it never collides with a real shell command or a program's
# own `:` commands (vim `:qa!`, etc.). A single line starting with `::` is not a
# valid shell command, so the web terminal can safely claim it for meta commands
# while everything else is passed through to the shell/program unchanged.
META_COMMAND_PREFIX = "::"
_CLI_MODULE = "selfdrive.carrot.server.terminal_commands.cli"
_OPENPILOT_PACKAGE_ROOT = Path(__file__).resolve().parents[4]


def _pythonpath() -> str:
  paths = [str(_OPENPILOT_PACKAGE_ROOT)]
  existing = os.environ.get("PYTHONPATH", "").strip()
  if existing:
    paths.append(existing)
  return os.pathsep.join(paths)


def translate_meta_command(line: str) -> str | None:
  """Translate a web-terminal-only meta command into the fixed CLI bridge."""
  stripped = str(line or "").strip()
  if not stripped.startswith(META_COMMAND_PREFIX):
    return None

  command_line = stripped[len(META_COMMAND_PREFIX):].strip() or "help"
  return shlex.join(["env", f"PYTHONPATH={_pythonpath()}", "python3", "-m", _CLI_MODULE, "--line", command_line])
