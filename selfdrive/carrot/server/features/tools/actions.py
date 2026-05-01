"""Shared tool action policy.

Keep dispatcher behavior in dispatcher.py, but keep action names and the
free-form shell command policy in one small module.
"""
from __future__ import annotations

from typing import Iterable, List, Optional, Tuple


KNOWN_TOOL_ACTIONS = {
  "backup_settings",
  "delete_all_logs",
  "delete_all_videos",
  "git_branch_list",
  "git_checkout",
  "git_log",
  "git_pull",
  "git_remote_add",
  "git_remote_set",
  "git_reset",
  "git_reset_repo_checkout",
  "git_reset_repo_fetch",
  "git_sync",
  "install_required",
  "reboot",
  "rebuild_all",
  "reset_calib",
  "send_tmux_log",
  "server_tmux_log",
  "shell_cmd",
}

READ_ONLY_SHELL_COMMANDS = {"cat", "df", "echo", "free", "git", "ls", "uptime"}
READ_ONLY_GIT_COMMANDS = {
  "branch",
  "diff",
  "log",
  "remote",
  "rev-parse",
  "show",
  "show-ref",
  "status",
}


def normalize_action(action: object) -> str:
  return str(action or "").strip()


def is_known_action(action: object) -> bool:
  return normalize_action(action) in KNOWN_TOOL_ACTIONS


def validate_action(action: object) -> Optional[Tuple[str, str]]:
  normalized = normalize_action(action)
  if normalized:
    if normalized in KNOWN_TOOL_ACTIONS:
      return None
    return f"unknown action: {normalized}", "UNKNOWN_TOOL_ACTION"
  return "missing action", "MISSING_TOOL_ACTION"


def validate_shell_argv(argv: Iterable[str]) -> Optional[Tuple[str, str, str]]:
  parts: List[str] = [str(part) for part in argv]
  if not parts:
    return "empty cmd", "EMPTY_CMD", ""

  top = parts[0]
  if top not in READ_ONLY_SHELL_COMMANDS:
    return f"not allowed: {top}", "CMD_NOT_ALLOWED", top

  if top != "git":
    return None

  if len(parts) < 2:
    return "missing git subcommand", "GIT_CMD_NOT_ALLOWED", ""

  subcommand = parts[1]
  if subcommand not in READ_ONLY_GIT_COMMANDS:
    return f"git subcommand not allowed: {subcommand}", "GIT_CMD_NOT_ALLOWED", subcommand

  return None
