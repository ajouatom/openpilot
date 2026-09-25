"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

Getting a large model's ONNX by its git-lfs oid.

comma overwrites one file per model, so the commit is the only name a big
model's ONNX has. GitHub's raw host serves the LFS pointer for any commit it
holds, merged or not, and that pointer carries the oid and size the comma will
ask a jetlink server for. The bytes themselves are on comma's LFS servers,
which is GitLab and not GitHub: each is asked in turn because which one has an
object varies with the model's age.

This mirrors the fork's openpilot/sunnypilot/accelerators/jetlink/lfs.py: same
URLs, same endpoint order, same verify rules, no openpilot imports.
"""
from __future__ import annotations

import hashlib
import json
import logging
import shutil
import urllib.request
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

from jetlink.registry.catalog import NetworkError, RegistryError, VerifyError, is_sha256

log = logging.getLogger('jetlink.registry')

POINTER_URL = 'https://raw.githubusercontent.com/commaai/openpilot/{ref}/openpilot/selfdrive/modeld/models/big_driving_supercombo.onnx'
LFS_ENDPOINTS = (
  'https://gitlab.com/commaai/openpilot-lfs.git/info/lfs',      # every object, older and PR-branch models included
  'https://huggingface.co/commaai/openpilot-lfs.git/info/lfs',  # where comma is moving them; the current ones
)
LFS_MEDIA_TYPE = 'application/vnd.git-lfs+json'
POINTER_TIMEOUT = 10.0
CONNECT_TIMEOUT = 30.0
# A pointer is 134 bytes. Anything larger is the ONNX itself, served by a host
# that resolved the LFS filter for us, and reading a gigabyte to find that out
# is not on.
POINTER_MAX = 4096
CHUNK = 4 << 20
FREE_SLACK = 64 << 20

ProgressFn = Callable[[float], None]
StopFn = Callable[[], bool]


@dataclass(frozen=True)
class Pointer:
  oid: str    # the ONNX SHA-256
  size: int


def parse_pointer_text(text: str) -> Pointer | None:
  """The oid and size in a git-lfs pointer's text, or None if it is not one."""
  if not isinstance(text, str) or len(text.encode('utf-8', 'replace')) > POINTER_MAX:
    return None
  oid = None
  size = None
  for line in text.splitlines():
    key, _, value = line.partition(' ')
    if key == 'oid':
      oid = value.removeprefix('sha256:').strip()
    elif key == 'size':
      try:
        size = int(value)
      except ValueError:
        return None
  if not is_sha256(oid) or size is None or size <= 0:
    return None
  return Pointer(oid, size)


def fetch_pointer(ref: str, timeout: float = POINTER_TIMEOUT, opener=None) -> Pointer:
  """The oid and size of the ONNX at a comma commit."""
  opener = opener or urllib.request.urlopen
  url = POINTER_URL.format(ref=ref)
  try:
    with opener(url, timeout=timeout) as response:
      text = response.read(POINTER_MAX).decode('utf-8', 'replace')
  except (OSError, ValueError) as e:
    raise NetworkError(f"could not fetch the lfs pointer at {url}: {e}") from e
  pointer = parse_pointer_text(text)
  if pointer is None:
    raise RegistryError(f"{ref[:10]} did not serve an lfs pointer")
  return pointer


def lfs_resolve(endpoint: str, pointer: Pointer, timeout: float = CONNECT_TIMEOUT, opener=None) -> str | None:
  """Ask one LFS server for a download href, or None if it does not have it.

  A server that is down is not different from a server that lacks the object:
  either way the caller moves to the next one, so nothing raises here.
  """
  opener = opener or urllib.request.urlopen
  body = json.dumps({
    'operation': 'download',
    'transfers': ['basic'],
    'objects': [{'oid': pointer.oid, 'size': pointer.size}],
  }).encode()
  request = urllib.request.Request(f"{endpoint}/objects/batch", data=body, method='POST',
                                   headers={'Accept': LFS_MEDIA_TYPE, 'Content-Type': LFS_MEDIA_TYPE})
  try:
    with opener(request, timeout=timeout) as response:
      payload = json.loads(response.read().decode())
  except (OSError, ValueError) as e:
    log.warning("lfs batch failed at %s: %s", endpoint, e)
    return None

  for obj in (payload.get('objects') or []) if isinstance(payload, dict) else []:
    if not isinstance(obj, dict) or obj.get('oid') != pointer.oid:
      continue
    if 'error' in obj:
      log.warning("%s has no %s (%s)", endpoint, pointer.oid[:16], (obj['error'] or {}).get('message'))
      return None
    href = ((obj.get('actions') or {}).get('download') or {}).get('href')
    if href:
      return str(href)
  return None


def lfs_download(href: str, pointer: Pointer, dest: Path, progress: ProgressFn | None = None,
                 should_stop: StopFn | None = None, opener=None) -> Path:
  """Stream to a .part file, hashing as we go, and only then take the name.

  A half-written model must never sit where the next start would hand it to a
  backend to build from.
  """
  opener = opener or urllib.request.urlopen
  dest = Path(dest)
  dest.parent.mkdir(parents=True, exist_ok=True)
  free = shutil.disk_usage(dest.parent).free
  if free < pointer.size + FREE_SLACK:
    raise RegistryError(f"need {pointer.size >> 20} MB for the model, {free >> 20} MB free")

  part = dest.with_name(dest.name + '.part')
  digest = hashlib.sha256()
  written = 0
  # Whole percent only: a gigabyte at 4 MB a chunk would call this a few
  # hundred times and the callback may write a param or a socket line.
  reported = -1
  try:
    with opener(href, timeout=CONNECT_TIMEOUT) as response, open(part, 'wb') as out:
      while True:
        if should_stop is not None and should_stop():
          raise RegistryError('download cancelled')
        chunk = response.read(CHUNK)
        if not chunk:
          break
        out.write(chunk)
        digest.update(chunk)
        written += len(chunk)
        if progress is not None and pointer.size:
          percent = int(100 * written / pointer.size)
          if percent != reported:
            reported = percent
            progress(min(1.0, written / pointer.size))
  except RegistryError:
    part.unlink(missing_ok=True)
    raise
  except Exception as e:
    part.unlink(missing_ok=True)
    raise NetworkError(f"could not download {pointer.oid[:16]}: {e}") from e

  if written != pointer.size:
    part.unlink(missing_ok=True)
    raise VerifyError(f"{pointer.oid[:16]} is {written} bytes, expected {pointer.size}")
  if digest.hexdigest() != pointer.oid:
    part.unlink(missing_ok=True)
    raise VerifyError(f"downloaded bytes hash to {digest.hexdigest()[:16]}, expected {pointer.oid[:16]}")

  part.replace(dest)
  if progress is not None:
    progress(1.0)
  return dest
