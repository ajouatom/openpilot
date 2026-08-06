#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections.abc import Callable
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import time
from typing import BinaryIO
from urllib.parse import urlparse
from urllib.request import Request, urlopen


MODELS_DIR = Path(__file__).resolve().parent / "models"
RDF_MODEL_PATH = MODELS_DIR / "driving_supercombo.onnx"
RDF_MODEL_MANIFEST_PATH = MODELS_DIR / "driving_supercombo.onnx.json"
DOWNLOAD_CHUNK_SIZE = 1024 * 1024


class ModelDownloadError(RuntimeError):
  pass


@dataclass(frozen=True)
class ModelManifest:
  url: str
  size: int
  sha256: str
  metadata: dict


def load_model_manifest(path: Path = RDF_MODEL_MANIFEST_PATH) -> ModelManifest:
  data = json.loads(path.read_text(encoding="utf-8"))
  if data.get("version") != 1:
    raise ModelDownloadError(f"unsupported model manifest version: {data.get('version')!r}")

  url = data.get("url")
  size = data.get("size")
  sha256 = str(data.get("sha256", "")).lower()
  if not isinstance(url, str) or urlparse(url).scheme != "https":
    raise ModelDownloadError("model manifest URL must use HTTPS")
  if not isinstance(size, int) or size <= 0:
    raise ModelDownloadError("model manifest size must be a positive integer")
  if len(sha256) != 64 or any(c not in "0123456789abcdef" for c in sha256):
    raise ModelDownloadError("model manifest SHA-256 is invalid")

  metadata = {k: v for k, v in data.items() if k not in ("version", "url", "size", "sha256")}
  return ModelManifest(url=url, size=size, sha256=sha256, metadata=metadata)


def sha256_file(path: Path) -> str:
  digest = hashlib.sha256()
  with path.open("rb") as f:
    while chunk := f.read(DOWNLOAD_CHUNK_SIZE):
      digest.update(chunk)
  return digest.hexdigest()


def model_matches(path: Path, manifest: ModelManifest) -> bool:
  return path.is_file() and path.stat().st_size == manifest.size and sha256_file(path) == manifest.sha256


def _download_once(url: str, destination: Path, expected_size: int,
                   progress: Callable[[int, int], None] | None,
                   urlopen_fn: Callable[..., BinaryIO]) -> None:
  request = Request(url, headers={"User-Agent": "carrot-openpilot-model-downloader/1"})
  downloaded = 0
  with urlopen_fn(request, timeout=60) as response, destination.open("wb") as output:
    while chunk := response.read(DOWNLOAD_CHUNK_SIZE):
      output.write(chunk)
      downloaded += len(chunk)
      if progress is not None:
        progress(downloaded, expected_size)
  if downloaded != expected_size:
    raise ModelDownloadError(f"model size mismatch: expected {expected_size}, downloaded {downloaded}")


def ensure_rdf_model(model_path: Path = RDF_MODEL_PATH,
                     manifest_path: Path = RDF_MODEL_MANIFEST_PATH,
                     progress: Callable[[int, int], None] | None = None,
                     urlopen_fn: Callable[..., BinaryIO] = urlopen,
                     attempts: int = 3,
                     sleep_fn: Callable[[float], None] = time.sleep) -> bool:
  manifest = load_model_manifest(manifest_path)
  if model_matches(model_path, manifest):
    return False
  if attempts < 1:
    raise ValueError("attempts must be at least 1")

  download_url = os.getenv("RDF_MODEL_URL", manifest.url)
  if urlparse(download_url).scheme != "https":
    raise ModelDownloadError("RDF_MODEL_URL must use HTTPS")

  model_path.parent.mkdir(parents=True, exist_ok=True)
  temporary_path = model_path.with_name(f".{model_path.name}.download-{os.getpid()}")
  last_error: Exception | None = None
  for attempt in range(attempts):
    try:
      temporary_path.unlink(missing_ok=True)
      _download_once(download_url, temporary_path, manifest.size, progress, urlopen_fn)
      actual_sha256 = sha256_file(temporary_path)
      if actual_sha256 != manifest.sha256:
        raise ModelDownloadError(f"model SHA-256 mismatch: expected {manifest.sha256}, got {actual_sha256}")
      os.replace(temporary_path, model_path)
      return True
    except Exception as exc:
      last_error = exc
      temporary_path.unlink(missing_ok=True)
      if attempt + 1 < attempts:
        sleep_fn(2 ** attempt)

  raise ModelDownloadError(f"failed to download RDF driving model after {attempts} attempts") from last_error


def main() -> None:
  parser = argparse.ArgumentParser(description="Download and verify the RDF driving model")
  parser.add_argument("--check", action="store_true", help="only verify the existing model")
  args = parser.parse_args()
  manifest = load_model_manifest()
  if args.check:
    if not model_matches(RDF_MODEL_PATH, manifest):
      raise SystemExit("RDF driving model is missing or invalid")
    print(f"RDF driving model OK: {RDF_MODEL_PATH}")
    return

  def print_progress(downloaded: int, total: int) -> None:
    print(f"\rDownloading RDF driving model: {downloaded * 100 // total:3d}%", end="", flush=True)

  downloaded = ensure_rdf_model(progress=print_progress)
  if downloaded:
    print()
    print(f"Downloaded RDF driving model: {RDF_MODEL_PATH}")
  else:
    print(f"RDF driving model already verified: {RDF_MODEL_PATH}")


if __name__ == "__main__":
  main()
