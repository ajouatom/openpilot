from __future__ import annotations

import asyncio
import os
import platform
import re
import shutil
import stat
import urllib.request
from dataclasses import dataclass, field

from ..config import CARROT_DATA_DIR


TRYCLOUDFLARE_RE = re.compile(r"https://[a-zA-Z0-9.-]+\.trycloudflare\.com")
CLOUDFLARED_DOWNLOADS = {
  "x86_64": "https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64",
  "amd64": "https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64",
  "aarch64": "https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-arm64",
  "arm64": "https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-arm64",
  "armv7l": "https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-arm",
}


@dataclass
class TunnelHandle:
  url: str
  process: asyncio.subprocess.Process | None = None
  log: list[str] = field(default_factory=list)
  drain_tasks: list[asyncio.Task] = field(default_factory=list)

  async def stop(self) -> None:
    for task in self.drain_tasks:
      task.cancel()
    if self.process is None:
      return
    if self.process.returncode is not None:
      return
    self.process.terminate()
    try:
      await asyncio.wait_for(self.process.wait(), timeout=4.0)
    except Exception:
      try:
        self.process.kill()
      except Exception:
        pass
      try:
        await asyncio.wait_for(self.process.wait(), timeout=2.0)
      except Exception:
        pass


def cloudflared_path() -> str:
  configured = os.environ.get("CARROT_CLOUDFLARED_BIN", "").strip()
  if configured:
    return configured
  path_binary = shutil.which("cloudflared")
  if path_binary:
    return path_binary
  local_binary = _local_cloudflared_path()
  if os.path.isfile(local_binary) and os.access(local_binary, os.X_OK):
    return local_binary
  return ""


def _local_cloudflared_path() -> str:
  return os.path.join(CARROT_DATA_DIR, "bin", "cloudflared")


def _cloudflared_download_url() -> str:
  if platform.system().lower() != "linux":
    return ""
  return CLOUDFLARED_DOWNLOADS.get(platform.machine().lower(), "")


def auto_install_enabled() -> bool:
  value = os.environ.get("CARROT_SUPPORT_AUTO_INSTALL_CLOUDFLARED", "1").strip().lower()
  return value not in {"0", "false", "no", "off"}


def cloudflared_status() -> dict[str, str | bool]:
  path = cloudflared_path()
  download_url = _cloudflared_download_url()
  return {
    "installed": bool(path),
    "path": path,
    "local_path": _local_cloudflared_path(),
    "auto_install": auto_install_enabled(),
    "download_supported": bool(download_url),
    "download_url": download_url,
    "platform": f"{platform.system()} {platform.machine()}",
  }


def _download_cloudflared() -> str:
  url = _cloudflared_download_url()
  if not url:
    raise RuntimeError(f"cloudflared not found and auto-install is unsupported on {platform.system()} {platform.machine()}")
  target = _local_cloudflared_path()
  os.makedirs(os.path.dirname(target), exist_ok=True)
  tmp_path = f"{target}.tmp"
  try:
    with urllib.request.urlopen(url, timeout=60) as response, open(tmp_path, "wb") as out:
      shutil.copyfileobj(response, out)
    os.chmod(tmp_path, stat.S_IRUSR | stat.S_IWUSR | stat.S_IXUSR | stat.S_IRGRP | stat.S_IXGRP | stat.S_IROTH | stat.S_IXOTH)
    os.replace(tmp_path, target)
  except Exception as exc:
    try:
      if os.path.exists(tmp_path):
        os.remove(tmp_path)
    except Exception:
      pass
    raise RuntimeError(f"cloudflared download failed: {exc}") from exc
  return target


async def resolve_cloudflared_path() -> str:
  binary = cloudflared_path()
  if binary:
    return binary
  if not auto_install_enabled():
    raise RuntimeError("cloudflared not found")
  return await asyncio.to_thread(_download_cloudflared)


async def _read_line(stream: asyncio.StreamReader, label: str) -> tuple[str, str]:
  raw = await stream.readline()
  if not raw:
    return label, ""
  return label, raw.decode("utf-8", errors="replace").rstrip()


async def _drain_stream(stream: asyncio.StreamReader, label: str, log: list[str]) -> None:
  while True:
    line = await stream.readline()
    if not line:
      return
    text = line.decode("utf-8", errors="replace").rstrip()
    if text:
      log.append(f"{label}: {text}")
      del log[:-80]


async def start_quick_tunnel(local_url: str, timeout_s: float = 20.0) -> TunnelHandle:
  fake_url = os.environ.get("CARROT_SUPPORT_FAKE_TUNNEL_URL", "").strip()
  if fake_url:
    return TunnelHandle(url=fake_url)

  binary = await resolve_cloudflared_path()

  process = await asyncio.create_subprocess_exec(
    binary,
    "tunnel",
    "--url",
    local_url,
    stdout=asyncio.subprocess.PIPE,
    stderr=asyncio.subprocess.PIPE,
  )
  log: list[str] = []
  streams = []
  if process.stdout is not None:
    streams.append(_read_line(process.stdout, "stdout"))
  if process.stderr is not None:
    streams.append(_read_line(process.stderr, "stderr"))

  deadline = asyncio.get_running_loop().time() + timeout_s
  pending = {asyncio.create_task(coro) for coro in streams}
  url = ""
  try:
    while pending and asyncio.get_running_loop().time() < deadline:
      timeout = max(0.1, deadline - asyncio.get_running_loop().time())
      done, pending = await asyncio.wait(pending, timeout=timeout, return_when=asyncio.FIRST_COMPLETED)
      if not done:
        break
      for task in done:
        label, line = task.result()
        if line:
          log.append(f"{label}: {line}")
          match = TRYCLOUDFLARE_RE.search(line)
          if match:
            url = match.group(0)
            break
        stream = process.stdout if label == "stdout" else process.stderr
        if stream is not None:
          pending.add(asyncio.create_task(_read_line(stream, label)))
      if url:
        break
      if process.returncode is not None:
        break
    if not url:
      detail = "; ".join(log[-6:])
      suffix = f": {detail}" if detail else ""
      raise RuntimeError(f"cloudflared did not provide a trycloudflare URL{suffix}")
  except Exception:
    for task in pending:
      task.cancel()
    handle = TunnelHandle(url="", process=process, log=log)
    await handle.stop()
    raise

  for task in pending:
    task.cancel()
  handle = TunnelHandle(url=url, process=process, log=log)
  if process.stdout is not None:
    handle.drain_tasks.append(asyncio.create_task(_drain_stream(process.stdout, "stdout", handle.log)))
  if process.stderr is not None:
    handle.drain_tasks.append(asyncio.create_task(_drain_stream(process.stderr, "stderr", handle.log)))
  return handle
