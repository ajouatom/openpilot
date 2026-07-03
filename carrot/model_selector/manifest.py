"""Fetch and verify the Carrot model manifest.

Primary source is ``models_v4.json`` (full catalog incl. new-architecture
models), with fallback to the frozen legacy ``models.json``.  Entries are
version-gated before filename validation and unparseable entries are skipped,
so future manifest additions can never break this selector's whole list
(the v3 selector had that flaw — see config.MODELS_JSON_URL).

Mirrors the canonical-JSON Ed25519 verification done in c3-ms
``model_manager.cc`` so the same signing tooling produces compatible bundles.
"""
from __future__ import annotations

import base64
import json
import math
import sys
import time
import urllib.parse
import urllib.request
from dataclasses import dataclass, field

from .config import ALLOWED_ONNX_FILES, MODELS_JSON_FALLBACK_URL, MODELS_JSON_URL
from .keys import MODEL_SELECTOR_VERSION, MODEL_SIGNING_KEYS


class ManifestError(Exception):
    pass


# ----------------------------------------------------------------------------
# Canonical JSON — keys sorted, no whitespace, only integer-valued numbers.
# ----------------------------------------------------------------------------

_ESCAPE = {
    ord('"'): '\\"',
    ord('\\'): '\\\\',
    ord('\b'): '\\b',
    ord('\f'): '\\f',
    ord('\n'): '\\n',
    ord('\r'): '\\r',
    ord('\t'): '\\t',
}


def _encode_string(s: str) -> str:
    out = ['"']
    for ch in s:
        code = ord(ch)
        esc = _ESCAPE.get(code)
        if esc is not None:
            out.append(esc)
        elif code <= 0x1F:
            out.append(f"\\u{code:04x}")
        else:
            out.append(ch)
    out.append('"')
    return "".join(out)


def to_canonical_json(value) -> str:
    if isinstance(value, dict):
        items = sorted(value.items(), key=lambda kv: kv[0])
        return "{" + ",".join(
            f"{_encode_string(str(k))}:{to_canonical_json(v)}" for k, v in items
        ) + "}"
    if isinstance(value, list):
        return "[" + ",".join(to_canonical_json(v) for v in value) + "]"
    if isinstance(value, str):
        return _encode_string(value)
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        if not math.isfinite(value) or not value.is_integer():
            raise ManifestError("non-integer float in canonical JSON")
        return str(int(value))
    if value is None:
        return "null"
    raise ManifestError(f"unsupported canonical JSON value: {type(value).__name__}")


# ----------------------------------------------------------------------------
# Ed25519 verification
# ----------------------------------------------------------------------------

def _verify_ed25519(public_key_b64: str, signature: bytes, message: bytes) -> bool:
    from cryptography.exceptions import InvalidSignature
    from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey
    try:
        pk = Ed25519PublicKey.from_public_bytes(base64.b64decode(public_key_b64))
        pk.verify(signature, message)
        return True
    except (InvalidSignature, ValueError):
        return False


# ----------------------------------------------------------------------------
# Manifest data model
# ----------------------------------------------------------------------------

@dataclass
class FileSpec:
    name: str
    size: int
    sha256: str


@dataclass
class ModelEntry:
    id: str
    name: str
    base_url: str
    added_at: str
    files: dict[str, FileSpec]
    minimum_selector_version: int = 0
    raw: dict = field(default_factory=dict)

    @property
    def onnx_filenames(self) -> list[str]:
        return sorted(self.files.keys())


def _min_selector_version(raw: dict) -> int | None:
    """Entry's minimum selector version; None when unparseable.

    Callers treat None as fail-closed (skip the entry): an entry whose
    version requirement can't be read must not be shown to selectors it
    might be incompatible with.
    """
    min_ver = raw.get("minimum_selector_version")
    if min_ver is None:
        min_ver = raw.get("minimumSelectorVersion", 0)
    try:
        return int(min_ver)
    except (TypeError, ValueError):
        return None


def _parse_model(raw: dict) -> ModelEntry:
    files: dict[str, FileSpec] = {}
    for fname, info in (raw.get("files") or {}).items():
        if fname not in ALLOWED_ONNX_FILES:
            raise ManifestError(f"disallowed filename in manifest: {fname}")
        files[fname] = FileSpec(
            name=fname,
            size=int(info["size"]),
            sha256=str(info["sha256"]).lower(),
        )
    base_url = raw.get("base_url") or raw.get("baseUrl")
    if not base_url:
        raise ManifestError("missing base_url")
    added_at = raw.get("added_at") or raw.get("addedAt") or ""
    return ModelEntry(
        id=str(raw["id"]),
        name=str(raw.get("name") or raw["id"]),
        base_url=str(base_url),
        added_at=str(added_at),
        files=files,
        minimum_selector_version=_min_selector_version(raw) or 0,
        raw=raw,
    )


def _fetch_and_verify_one(url: str, timeout: float = 20.0) -> list[ModelEntry]:
    """Download one manifest URL, verify its Ed25519 signature, and return the
    parsed model list.  Raises `ManifestError` on any failure.
    """
    # Append cache-busting query to bypass raw.githubusercontent.com CDN.
    sep = "&" if urllib.parse.urlparse(url).query else "?"
    fetch_url = f"{url}{sep}t={int(time.time())}"
    req = urllib.request.Request(fetch_url, headers={
        "Cache-Control": "no-cache",
        "Pragma": "no-cache",
    })
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            body = resp.read()
    except Exception as e:
        raise ManifestError(f"failed to fetch manifest: {e}") from e

    try:
        doc = json.loads(body)
    except (json.JSONDecodeError, UnicodeDecodeError) as e:
        raise ManifestError(f"manifest is not valid JSON: {e}") from e

    if not isinstance(doc, dict):
        raise ManifestError("manifest root must be an object")

    key_id = doc.pop("key_id", None)
    signature_b64 = doc.pop("signature", None)
    if not key_id or not signature_b64:
        raise ManifestError("manifest missing key_id or signature")

    public_key_b64 = MODEL_SIGNING_KEYS.get(key_id)
    if public_key_b64 is None:
        raise ManifestError(f"unknown signing key_id: {key_id}")

    try:
        signature = base64.b64decode(signature_b64)
    except Exception as e:
        raise ManifestError(f"signature is not valid base64: {e}") from e
    if len(signature) != 64:
        raise ManifestError(f"invalid signature length: {len(signature)}")

    canonical = to_canonical_json(doc).encode("utf-8")
    if not _verify_ed25519(public_key_b64, signature, canonical):
        raise ManifestError("manifest signature verification failed")

    models_raw = doc.get("models") or []
    if not isinstance(models_raw, list):
        raise ManifestError("manifest 'models' must be a list")

    out: list[ModelEntry] = []
    for raw in models_raw:
        if not isinstance(raw, dict):
            continue
        # Version-gate BEFORE parsing: entries for newer selectors may use
        # filenames this version doesn't know, and must not fail the whole list.
        # Unreadable version requirement → fail closed (skip).
        min_ver = _min_selector_version(raw)
        if min_ver is None or min_ver > MODEL_SELECTOR_VERSION:
            continue
        try:
            entry = _parse_model(raw)
        except (ManifestError, KeyError, ValueError, TypeError):
            # Forward compatibility: skip entries this selector can't handle
            # instead of failing the entire (signature-verified) list.
            continue
        out.append(entry)
    return out


def fetch_and_verify(url: str | None = None, timeout: float = 20.0) -> list[ModelEntry]:
    """Fetch the model manifest and return the verified, parsed model list.

    With no explicit `url`, tries the full catalog (`models_v4.json`) first
    and falls back to the frozen legacy manifest (`models.json`).  Raises the
    primary `ManifestError` only if every candidate fails.
    """
    urls = (url,) if url else (MODELS_JSON_URL, MODELS_JSON_FALLBACK_URL)
    errors: list[ManifestError] = []
    for u in urls:
        try:
            return _fetch_and_verify_one(u, timeout)
        except ManifestError as e:
            errors.append(e)
            if len(urls) > 1:
                # A quietly-degraded list (fallback after a broken primary)
                # must at least be diagnosable from the web server log.
                print(f"model_selector: manifest fetch failed for {u}: {e}",
                      file=sys.stderr)
    raise errors[0]
