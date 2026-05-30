"""GitHub SSH key helpers for the web Device developer panel."""
from __future__ import annotations

import base64
import hashlib
import re
import time
from typing import Any, Dict

from aiohttp import ClientSession, ClientTimeout

from .params import HAS_PARAMS, Params, get_param_value, set_param_value


GITHUB_USERNAME_RE = re.compile(r"^[A-Za-z0-9](?:[A-Za-z0-9-]{0,37}[A-Za-z0-9])?$")
SSH_KEY_TYPES = {
  "ssh-ed25519",
  "ssh-rsa",
  "ecdsa-sha2-nistp256",
  "ecdsa-sha2-nistp384",
  "ecdsa-sha2-nistp521",
  "sk-ssh-ed25519@openssh.com",
  "sk-ecdsa-sha2-nistp256@openssh.com",
}


class SshKeyError(RuntimeError):
  def __init__(self, message: str, status: int = 400) -> None:
    super().__init__(message)
    self.status = status


def _fingerprint_key_blob(key_blob: str) -> str:
  try:
    raw = base64.b64decode(key_blob.encode("ascii"), validate=True)
    digest = base64.b64encode(hashlib.sha256(raw).digest()).decode("ascii").rstrip("=")
    return f"SHA256:{digest}"
  except Exception:
    return ""


def _summarize_ssh_keys(keys: str) -> list[Dict[str, str]]:
  summaries = []
  for line in str(keys or "").splitlines():
    line = line.strip()
    if not line or line.startswith("#"):
      continue
    parts = line.split()
    if len(parts) < 2 or parts[0] not in SSH_KEY_TYPES:
      continue
    fingerprint = _fingerprint_key_blob(parts[1])
    summaries.append({
      "type": parts[0].replace("ssh-", ""),
      "fingerprint": fingerprint,
    })
  return summaries


def get_ssh_key_status() -> Dict[str, Any]:
  username = get_param_value("GithubUsername", "")
  keys = get_param_value("GithubSshKeys", "")
  key_summaries = _summarize_ssh_keys(keys)
  return {
    "username": username,
    "has_keys": bool(key_summaries),
    "key_count": len(key_summaries),
    "fingerprints": key_summaries[:8],
    "updated_at": get_param_value("GithubSshKeysUpdatedAt", ""),
  }


def clear_ssh_keys() -> Dict[str, Any]:
  if HAS_PARAMS:
    params = Params()
    for key in ("GithubUsername", "GithubSshKeys", "GithubSshKeysUpdatedAt"):
      try:
        params.remove(key)
      except Exception:
        set_param_value(key, "")
  else:
    set_param_value("GithubUsername", "")
    set_param_value("GithubSshKeys", "")
    set_param_value("GithubSshKeysUpdatedAt", "")

  return get_ssh_key_status()


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
  set_param_value("GithubSshKeysUpdatedAt", str(int(time.time() * 1000)))
  return get_ssh_key_status()


async def refresh_github_ssh_keys(session: ClientSession) -> Dict[str, Any]:
  username = get_param_value("GithubUsername", "")
  if not str(username or "").strip():
    raise SshKeyError("GitHub username is not configured", 400)
  return await add_github_ssh_keys(session, username)
