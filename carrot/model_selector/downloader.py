"""Download ONNX model files with size + SHA256 verification."""
from __future__ import annotations

import hashlib
import os
import re
import tempfile
import urllib.parse
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from .config import (
    ALLOWED_ONNX_FILES,
    ALLOWED_URL_PREFIX,
    MODEL_ID_REGEX,
    MODELS_TMP_DIR,
)
from .manifest import FileSpec, ModelEntry


class DownloadError(Exception):
    pass


ProgressFn = Callable[[str, int, int], None]  # (filename, bytes_done, bytes_total)


def _validate_model_id(model_id: str) -> None:
    if not re.fullmatch(MODEL_ID_REGEX, model_id or ""):
        raise DownloadError(f"invalid model id: {model_id!r}")
    if ".." in model_id or "/" in model_id or "\\" in model_id:
        raise DownloadError(f"disallowed character in model id: {model_id!r}")


def _validate_url(url: str) -> None:
    if not url.startswith(ALLOWED_URL_PREFIX):
        raise DownloadError(f"URL prefix not allowed: {url}")


def _build_url(base_url: str, filename: str) -> str:
    if filename not in ALLOWED_ONNX_FILES:
        raise DownloadError(f"filename not in allowlist: {filename}")
    sep = "" if base_url.endswith("/") else "/"
    url = f"{base_url}{sep}{filename}"
    _validate_url(url)
    # Percent-encode the path (spaces, unicode) — raw.githubusercontent.com
    # serves files whose names contain spaces (e.g. "Op Modelv12") but urllib
    # rejects URLs with literal control characters.
    parts = urllib.parse.urlsplit(url)
    quoted_path = urllib.parse.quote(parts.path, safe="/%")
    return urllib.parse.urlunsplit(parts._replace(path=quoted_path))


@dataclass
class DownloadResult:
    path: Path
    size: int
    sha256: str


def _download_one(
    url: str,
    dst: Path,
    spec: FileSpec,
    on_progress: ProgressFn | None,
) -> DownloadResult:
    total = spec.size
    hasher = hashlib.sha256()
    # write to a .part file, rename on success
    dst.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_str = tempfile.mkstemp(prefix=dst.name + ".", suffix=".part", dir=dst.parent)
    tmp_path = Path(tmp_str)
    done = 0
    try:
        with os.fdopen(fd, "wb") as out, urllib.request.urlopen(url, timeout=60) as resp:
            chunk_size = 1 << 20  # 1 MiB
            while True:
                chunk = resp.read(chunk_size)
                if not chunk:
                    break
                out.write(chunk)
                hasher.update(chunk)
                done += len(chunk)
                if on_progress is not None:
                    on_progress(spec.name, done, total)
                if done > total:
                    raise DownloadError(
                        f"{spec.name}: body exceeds declared size ({done} > {total})"
                    )
        if done != total:
            raise DownloadError(
                f"{spec.name}: size mismatch (got {done}, want {total})"
            )
        digest = hasher.hexdigest().lower()
        if digest != spec.sha256.lower():
            raise DownloadError(
                f"{spec.name}: sha256 mismatch (got {digest}, want {spec.sha256})"
            )
        os.replace(tmp_path, dst)
    except Exception:
        try:
            tmp_path.unlink(missing_ok=True)
        except Exception:
            pass
        raise
    return DownloadResult(path=dst, size=done, sha256=hasher.hexdigest().lower())


def download_model(
    entry: ModelEntry,
    on_progress: ProgressFn | None = None,
    tmp_dir: Path = MODELS_TMP_DIR,
) -> dict[str, DownloadResult]:
    """Download every file of `entry` into a fresh tmp_dir, verifying
    size + SHA256 of each.  Raises `DownloadError` on any failure.
    """
    _validate_model_id(entry.id)
    _validate_url(entry.base_url + "/")

    # Start with a clean tmp_dir so a partial previous download cannot be
    # mistaken for the current one.
    if tmp_dir.exists():
        for child in tmp_dir.iterdir():
            try:
                if child.is_dir():
                    continue
                child.unlink()
            except OSError:
                pass
    tmp_dir.mkdir(parents=True, exist_ok=True)

    # Either a single supercombo onnx (new architecture) or the legacy
    # vision + (on_policy or policy) set is required; fail fast otherwise.
    names = set(entry.files.keys())
    if "driving_supercombo.onnx" not in names:
        if "driving_vision.onnx" not in names:
            raise DownloadError(
                "manifest entry missing driving_supercombo.onnx or driving_vision.onnx"
            )
        if "driving_on_policy.onnx" not in names and "driving_policy.onnx" not in names:
            raise DownloadError(
                "manifest entry needs driving_on_policy.onnx or driving_policy.onnx"
            )

    results: dict[str, DownloadResult] = {}
    for filename in sorted(names):
        spec = entry.files[filename]
        url = _build_url(entry.base_url, filename)
        dst = tmp_dir / filename
        results[filename] = _download_one(url, dst, spec, on_progress)
    return results
