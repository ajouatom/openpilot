"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

sunnypilot's big-model catalog: which large models exist and which commit each one is.

The catalog is the same JSON the comma's model manager caches for its chestnut
slot, so a model picked here is a model the comma can ask for. A bundle's
artifacts are tinygrad pkls for a GPU we do not have; the only field that
matters is `ref`, the comma openpilot commit the bundle was compiled from,
because that commit's ONNX is what a jetlink server runs.

Stdlib only, and every network call takes an `opener` so tests stay offline.
"""
from __future__ import annotations

import json
import re
import urllib.request
from dataclasses import dataclass

CATALOG_URL = 'https://raw.githubusercontent.com/sunnypilot/sunnypilot-models/refs/heads/gh-pages/docs/driving_models_chestnut_v25.json'
# The selector version the fork requires (REQUIRED_JSON_VERSION on the comma).
# It is a string in the JSON; bundles at any other version describe fields we
# would misread.
REQUIRED_SELECTOR_VERSION = 19
DEFAULT_BIG_MODEL_REF = 'f877d7a0ccc3cce943c76e285214c020cd65c899'
CATALOG_TIMEOUT = 10.0

_REF = re.compile(r'[0-9a-f]{40}')
_SHA256 = re.compile(r'[0-9a-f]{64}')


class RegistryError(Exception):
  """Anything the registry refuses to do. Exit code 1 in the CLI."""


class NetworkError(RegistryError):
  """A server could not be reached or did not answer sensibly. Exit code 2."""


class VerifyError(RegistryError):
  """Bytes arrived, but not the bytes that were asked for. Exit code 3."""


@dataclass(frozen=True)
class CatalogModel:
  name: str
  short_name: str
  ref: str
  build_time: str
  index: int


def is_ref(value: str) -> bool:
  """A comma openpilot commit: 40 lowercase hex characters."""
  return isinstance(value, str) and _REF.fullmatch(value) is not None


def is_sha256(value: str) -> bool:
  """A model identity: 64 lowercase hex characters, which is also the LFS oid."""
  return isinstance(value, str) and _SHA256.fullmatch(value) is not None


def parse_catalog(data: dict) -> list[CatalogModel]:
  """The big-model bundles, newest first. No IO, and no bundle can raise.

  A malformed entry in a catalog served to every comma must cost that entry and
  nothing else, so each bundle is parsed in its own try.
  """
  found: dict[str, CatalogModel] = {}
  bundles = (data or {}).get('bundles') or []
  if not isinstance(bundles, list):
    return []
  for bundle in bundles:
    try:
      ref = bundle.get('ref')
      if not is_ref(ref) or ref in found:
        continue
      if int(bundle.get('minimum_selector_version', 0)) != REQUIRED_SELECTOR_VERSION or not bundle.get('is_big'):
        continue
      found[ref] = CatalogModel(name=str(bundle.get('display_name') or ref[:10]), short_name=str(bundle.get('short_name') or ''),
                                ref=ref, build_time=str(bundle.get('build_time') or ''), index=int(bundle.get('index', 0)))
    except (AttributeError, TypeError, ValueError):
      continue
  return sorted(found.values(), key=lambda m: m.index, reverse=True)


def fetch_catalog(url: str = CATALOG_URL, timeout: float = CATALOG_TIMEOUT, opener=None) -> dict:
  """The catalog JSON. Every failure, transport or content, is a NetworkError."""
  opener = opener or urllib.request.urlopen
  try:
    with opener(url, timeout=timeout) as response:
      data = json.loads(response.read().decode())
  except (OSError, ValueError) as e:
    raise NetworkError(f"could not fetch the model catalog from {url}: {e}") from e
  if not isinstance(data, dict):
    raise NetworkError(f"{url} did not serve a JSON object")
  return data
