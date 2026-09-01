#!/usr/bin/env python3
"""Download and select the optional Carrot USB-eGPU driving model.

The large ONNX is deliberately kept outside the git checkout.  A small remote
manifest selects the model, while verified files live on persistent device
storage across source updates.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import socket
import sys
import tempfile
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Callable
from urllib.parse import urljoin, urlparse
from urllib.request import Request, urlopen

from openpilot.selfdrive.modeld.big_model_status import BigModelStatusReporter


DEFAULT_MANIFEST_URL = "https://upload.shind0.synology.me/models/comma4-big-tgc/manifest.json"
TGC_MODEL = {
  "model_id": "comma-pr38739-tgc-a2e422ee-1791d594",
  "filename": "big_driving_supercombo.onnx",
  "size": 765_950_064,
  "sha256": "1791d5940b2c048d0639813426dd2cf1d6f2a6727ed51e17c8bcea8bbe754123",
  "url": "https://upload.shind0.synology.me/models/comma4-big-tgc/big_driving_supercombo.onnx",
}
MAX_MANIFEST_SIZE = 64 * 1024
MAX_MODEL_SIZE = 4 * 1024 * 1024 * 1024
DOWNLOAD_CHUNK_SIZE = 4 * 1024 * 1024
DOWNLOAD_HEADROOM = 256 * 1024 * 1024
STATE_FILENAME = "state.json"
MODEL_NAME_RE = re.compile(r"^[A-Za-z0-9._-]+$")
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


@dataclass(frozen=True)
class BigModelManifest:
  model_id: str
  filename: str
  size: int
  sha256: str
  url: str

  @classmethod
  def from_dict(cls, value: Any, manifest_url: str = DEFAULT_MANIFEST_URL) -> BigModelManifest:
    if not isinstance(value, dict):
      raise ValueError("model manifest must be a JSON object")

    model_id = value.get("model_id")
    filename = value.get("filename")
    size = value.get("size")
    sha256 = value.get("sha256")
    model_url = value.get("url")
    if not isinstance(model_id, str) or not MODEL_NAME_RE.fullmatch(model_id):
      raise ValueError("invalid model_id")
    if not isinstance(filename, str) or not MODEL_NAME_RE.fullmatch(filename) or not filename.endswith(".onnx"):
      raise ValueError("invalid model filename")
    if not isinstance(size, int) or isinstance(size, bool) or not 0 < size <= MAX_MODEL_SIZE:
      raise ValueError("invalid model size")
    if not isinstance(sha256, str) or not SHA256_RE.fullmatch(sha256):
      raise ValueError("invalid model sha256")
    if not isinstance(model_url, str) or not model_url:
      raise ValueError("invalid model URL")

    resolved_url = urljoin(manifest_url, model_url)
    if urlparse(resolved_url).scheme != "https":
      raise ValueError("model URL must use HTTPS")
    return cls(model_id=model_id, filename=filename, size=size, sha256=sha256, url=resolved_url)

  @property
  def cache_filename(self) -> str:
    return f"{Path(self.filename).stem}-{self.sha256[:16]}.onnx"


def model_cache_dir() -> Path:
  if cache_override := os.getenv("CARROT_BIG_MODEL_DIR"):
    return Path(cache_override)
  if Path("/TICI").is_file():
    return Path("/data/media/0/carrot/models")
  return Path.home() / ".comma" / "models"


def state_path(cache_dir: Path | None = None) -> Path:
  return (cache_dir or model_cache_dir()) / STATE_FILENAME


def model_path(manifest: BigModelManifest, cache_dir: Path | None = None) -> Path:
  return (cache_dir or model_cache_dir()) / manifest.cache_filename


def _read_json(path: Path) -> Any:
  with path.open(encoding="utf-8") as f:
    return json.load(f)


def read_state(cache_dir: Path | None = None) -> dict[str, BigModelManifest | None]:
  path = state_path(cache_dir)
  try:
    raw = _read_json(path)
    if not isinstance(raw, dict):
      raise ValueError("invalid state")
    return {
      "active": BigModelManifest.from_dict(raw["active"]) if raw.get("active") is not None else None,
      "previous": BigModelManifest.from_dict(raw["previous"]) if raw.get("previous") is not None else None,
    }
  except (FileNotFoundError, KeyError, TypeError, ValueError, json.JSONDecodeError):
    return {"active": None, "previous": None}


def _write_state(active: BigModelManifest, previous: BigModelManifest | None, cache_dir: Path) -> None:
  cache_dir.mkdir(parents=True, exist_ok=True)
  value = {"active": asdict(active), "previous": asdict(previous) if previous is not None else None}
  fd, tmp_name = tempfile.mkstemp(prefix=".state-", suffix=".json", dir=cache_dir)
  try:
    with os.fdopen(fd, "w", encoding="utf-8") as f:
      json.dump(value, f, indent=2, sort_keys=True)
      f.write("\n")
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp_name, state_path(cache_dir))
  finally:
    try:
      os.unlink(tmp_name)
    except FileNotFoundError:
      pass


def fetch_manifest(manifest_url: str = DEFAULT_MANIFEST_URL, timeout: float = 15.0) -> BigModelManifest:
  # carrot-tgc intentionally pins commaai/openpilot#38739 (TGC). Keep the
  # environment/CLI override path below so a different manifest can still be
  # tested explicitly without changing this branch.
  if manifest_url == DEFAULT_MANIFEST_URL:
    return BigModelManifest.from_dict(TGC_MODEL, manifest_url)

  req = Request(manifest_url, headers={"Accept": "application/json", "User-Agent": "carrot-modeld/1"})
  with urlopen(req, timeout=timeout) as response:
    data = response.read(MAX_MANIFEST_SIZE + 1)
  if len(data) > MAX_MANIFEST_SIZE:
    raise ValueError("model manifest is too large")
  return BigModelManifest.from_dict(json.loads(data), manifest_url)


def wait_for_manifest_network(manifest_url: str, timeout: float = 60.0,
                              connect_timeout: float = 2.0, poll_interval: float = 2.0) -> bool:
  parsed = urlparse(manifest_url)
  if parsed.scheme != "https" or parsed.hostname is None:
    raise ValueError("model manifest URL must use HTTPS and include a host")

  address = (parsed.hostname, parsed.port or 443)
  deadline = time.monotonic() + max(0.0, timeout)
  while True:
    remaining = max(0.0, deadline - time.monotonic())
    try:
      connection = socket.create_connection(address, timeout=max(0.1, min(connect_timeout, remaining or 0.1)))
      connection.close()
      return True
    except OSError:
      remaining = deadline - time.monotonic()
      if remaining <= 0.0:
        return False
      time.sleep(min(poll_interval, remaining))


def _sha256(path: Path) -> str:
  digest = hashlib.sha256()
  with path.open("rb") as f:
    while chunk := f.read(DOWNLOAD_CHUNK_SIZE):
      digest.update(chunk)
  return digest.hexdigest()


def _download_model(manifest: BigModelManifest, cache_dir: Path, timeout: float = 30.0,
                    progress_callback: Callable[[BigModelManifest, int, int], None] | None = None,
                    phase_callback: Callable[[str, BigModelManifest], None] | None = None) -> Path:
  cache_dir.mkdir(parents=True, exist_ok=True)
  final_path = model_path(manifest, cache_dir)
  partial_path = final_path.with_suffix(final_path.suffix + ".part")

  if final_path.is_file() and final_path.stat().st_size == manifest.size:
    if phase_callback is not None:
      phase_callback("verifying", manifest)
    if _sha256(final_path) == manifest.sha256:
      return final_path
    final_path.unlink()

  offset = partial_path.stat().st_size if partial_path.is_file() else 0
  if offset > manifest.size:
    partial_path.unlink()
    offset = 0
  elif offset == manifest.size:
    if phase_callback is not None:
      phase_callback("verifying", manifest)
    if _sha256(partial_path) == manifest.sha256:
      os.replace(partial_path, final_path)
      return final_path
    partial_path.unlink()
    offset = 0

  required = manifest.size - offset + DOWNLOAD_HEADROOM
  if shutil.disk_usage(cache_dir).free < required:
    raise OSError(f"not enough free space for big model (need {required} bytes)")

  headers = {"Accept-Encoding": "identity", "User-Agent": "carrot-modeld/1"}
  if offset:
    headers["Range"] = f"bytes={offset}-"
  req = Request(manifest.url, headers=headers)
  if progress_callback is not None:
    progress_callback(manifest, offset, manifest.size)
  with urlopen(req, timeout=timeout) as response:
    status = getattr(response, "status", response.getcode())
    append = offset > 0 and status == 206
    if append:
      content_range = response.headers.get("Content-Range", "")
      if not content_range.startswith(f"bytes {offset}-"):
        raise OSError(f"invalid resume response: {content_range!r}")
    elif status != 200:
      raise OSError(f"unexpected model download status: {status}")

    mode = "ab" if append else "wb"
    with partial_path.open(mode) as f:
      while chunk := response.read(DOWNLOAD_CHUNK_SIZE):
        f.write(chunk)
        offset += len(chunk)
        if progress_callback is not None:
          progress_callback(manifest, offset, manifest.size)
      f.flush()
      os.fsync(f.fileno())

  actual_size = partial_path.stat().st_size
  if actual_size != manifest.size:
    raise OSError(f"model size mismatch: expected {manifest.size}, got {actual_size}")
  if phase_callback is not None:
    phase_callback("verifying", manifest)
  actual_sha256 = _sha256(partial_path)
  if actual_sha256 != manifest.sha256:
    partial_path.unlink()
    raise OSError(f"model sha256 mismatch: expected {manifest.sha256}, got {actual_sha256}")
  os.replace(partial_path, final_path)
  return final_path


def ensure_big_model(manifest_url: str = DEFAULT_MANIFEST_URL, cache_dir: Path | None = None,
                     progress_callback: Callable[[BigModelManifest, int, int], None] | None = None,
                     phase_callback: Callable[[str, BigModelManifest], None] | None = None) -> tuple[Path, bool]:
  cache_dir = cache_dir or model_cache_dir()
  manifest = fetch_manifest(manifest_url)
  state = read_state(cache_dir)
  active = state["active"]
  path = model_path(manifest, cache_dir)
  if active is not None and active.sha256 == manifest.sha256 and path.is_file() and path.stat().st_size == manifest.size:
    return path, False

  path = _download_model(manifest, cache_dir, progress_callback=progress_callback, phase_callback=phase_callback)
  changed = active is None or active.sha256 != manifest.sha256
  if changed:
    previous = active if active is not None and model_path(active, cache_dir).is_file() else state["previous"]
    _write_state(manifest, previous, cache_dir)
    keep = {manifest.cache_filename}
    if previous is not None:
      keep.add(previous.cache_filename)
    for old_model in cache_dir.glob("big_driving_supercombo-*.onnx"):
      if old_model.name not in keep:
        old_model.unlink()
  return path, changed


def active_manifest(cache_dir: Path | None = None) -> BigModelManifest | None:
  cache_dir = cache_dir or model_cache_dir()
  active = read_state(cache_dir)["active"]
  if active is None:
    return None
  path = model_path(active, cache_dir)
  return active if path.is_file() and path.stat().st_size == active.size else None


def active_model_path(cache_dir: Path | None = None) -> Path | None:
  cache_dir = cache_dir or model_cache_dir()
  active = active_manifest(cache_dir)
  return model_path(active, cache_dir) if active is not None else None


def active_model_compiled() -> bool:
  if active_model_path() is None:
    return False
  from openpilot.common.file_chunker import get_manifest_path
  from openpilot.selfdrive.modeld.helpers import modeld_pkl_path
  return Path(get_manifest_path(Path(modeld_pkl_path(usbgpu=True)))).is_file()


def remember_usbgpu_connection(params, present: bool, compiled_model: bool) -> bool:
  """Persist hardware-proven eGPU history for updates while it is powered off."""
  remembered = params.get_bool("UsbGpuHardwareSeen")
  known = remembered or present or compiled_model
  if known and not remembered:
    params.put_bool("UsbGpuHardwareSeen", True)
  return known


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--ensure-if-egpu", action="store_true")
  parser.add_argument("--active-sha", action="store_true")
  parser.add_argument("--active-path", action="store_true")
  parser.add_argument("--manifest-url", default=os.getenv("CARROT_BIG_MODEL_MANIFEST", DEFAULT_MANIFEST_URL))
  parser.add_argument("--network-wait-seconds", type=float, default=0.0)
  args = parser.parse_args()

  if args.network_wait_seconds < 0.0:
    parser.error("--network-wait-seconds must be non-negative")

  if args.ensure_if_egpu:
    from openpilot.common.params import Params
    from openpilot.selfdrive.modeld.helpers import usbgpu_compiled, usbgpu_present
    present = usbgpu_present()
    if remember_usbgpu_connection(Params(), present, usbgpu_compiled()):
      cache_dir = model_cache_dir()
      reporter = BigModelStatusReporter(cache_dir)
      try:
        reporter.update("checking", detail="checking model catalog")
        if active_manifest() is None and args.network_wait_seconds > 0.0:
          print(f"waiting up to {args.network_wait_seconds:g}s for the big model server")
          reporter.update("checking", detail="waiting for network")
          if not wait_for_manifest_network(args.manifest_url, args.network_wait_seconds):
            raise TimeoutError(f"big model server unavailable after {args.network_wait_seconds:g}s")
        def phase(state: str, manifest: BigModelManifest) -> None:
          reporter.update(state, model_id=manifest.model_id, sha256=manifest.sha256,
                          downloaded_bytes=manifest.size, total_bytes=manifest.size)

        path, changed = ensure_big_model(args.manifest_url, cache_dir,
                                         progress_callback=reporter.download_progress,
                                         phase_callback=phase)
        manifest = active_manifest(cache_dir)
        reporter.update("compiled" if active_model_compiled() else "ready",
                        model_id=manifest.model_id if manifest is not None else None,
                        sha256=manifest.sha256 if manifest is not None else None,
                        downloaded_bytes=manifest.size if manifest is not None else None,
                        total_bytes=manifest.size if manifest is not None else None)
        print(f"big model {'updated' if changed else 'ready'}: {path}")
      except Exception as e:
        # Model delivery must never prevent the normal internal-GPU build.
        reporter.update("error", detail=str(e))
        print(f"big model update skipped: {e}", file=sys.stderr)
    return 0

  manifest = active_manifest()
  if args.active_sha:
    print(manifest.sha256 if manifest is not None else "")
  elif args.active_path:
    path = active_model_path()
    print(path if path is not None else "")
  else:
    parser.error("select an action")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
