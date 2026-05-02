"""GitHub SSH key helpers for the web Device developer panel."""
from __future__ import annotations

import re
from typing import Any, Dict

from aiohttp import ClientSession, ClientTimeout

from .params import HAS_PARAMS, Params, get_param_value, set_param_value


GITHUB_USERNAME_RE = re.compile(r"^[A-Za-z0-9](?:[A-Za-z0-9-]{0,37}[A-Za-z0-9])?$")


class SshKeyError(RuntimeError):
  def __init__(self, message: str, status: int = 400) -> None:
    super().__init__(message)
    self.status = status


def get_ssh_key_status() -> Dict[str, Any]:
  username = get_param_value("GithubUsername", "")
  keys = get_param_value("GithubSshKeys", "")
  return {
    "username": username,
    "has_keys": bool(str(keys).strip()),
  }


def clear_ssh_keys() -> Dict[str, Any]:
  if HAS_PARAMS:
    params = Params()
    for key in ("GithubUsername", "GithubSshKeys"):
      try:
        params.remove(key)
      except Exception:
        set_param_value(key, "")
  else:
    set_param_value("GithubUsername", "")
    set_param_value("GithubSshKeys", "")

  return {"username": "", "has_keys": False}


async def add_github_ssh_keys(session: ClientSession, username: str) -> Dict[str, Any]:
  username = str(username or "").strip()
  if not GITHUB_USERNAME_RE.match(username):
    raise SshKeyError("invalid username", 400)

  url = f"https://github.com/{username}.keys"
  try:
    async with session.get(url, timeout=ClientTimeout(total=10)) as resp:
      text = await resp.text()
      if resp.status == 404:
        raise SshKeyError(f"Username '{username}' doesn't exist on GitHub", 404)
      if resp.status < 200 or resp.status >= 300:
        raise SshKeyError(f"GitHub request failed: HTTP {resp.status}", 502)
  except TimeoutError as exc:
    raise SshKeyError("Request timed out", 504) from exc

  keys = "\n".join(line.strip() for line in text.splitlines() if line.strip())
  if not keys:
    raise SshKeyError(f"Username '{username}' has no keys on GitHub", 404)

  set_param_value("GithubUsername", username)
  set_param_value("GithubSshKeys", keys)
  return {"username": username, "has_keys": True}
