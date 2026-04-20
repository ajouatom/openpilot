"""Fetch and verify the Carrot model manifest (models.json).

Mirrors the canonical-JSON Ed25519 verification done in c3-ms
``model_manager.cc`` so the same signing tooling produces compatible bundles.
"""
from __future__ import annotations

import base64
import json
import math
import urllib.request
from dataclasses import dataclass, field

from .config import ALLOWED_ONNX_FILES, MODELS_JSON_URL
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
    return ModelEntry(
        id=str(raw["id"]),
        name=str(raw.get("name") or raw["id"]),
        base_url=str(raw["baseUrl"]),
        added_at=str(raw.get("addedAt", "")),
        files=files,
        minimum_selector_version=int(raw.get("minimumSelectorVersion", 0)),
        raw=raw,
    )


def fetch_and_verify(url: str = MODELS_JSON_URL, timeout: float = 20.0) -> list[ModelEntry]:
    """Download `models.json`, verify its Ed25519 signature, and return the
    parsed model list.  Raises `ManifestError` on any failure.
    """
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            body = resp.read()
    except Exception as e:
        raise ManifestError(f"failed to fetch manifest: {e}") from e

    try:
        doc = json.loads(body)
    except json.JSONDecodeError as e:
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
        try:
            entry = _parse_model(raw)
        except (KeyError, ValueError, TypeError) as e:
            raise ManifestError(f"invalid model entry: {e}") from e

        if entry.minimum_selector_version > MODEL_SELECTOR_VERSION:
            # Skip entries that require a newer selector.
            continue
        out.append(entry)
    return out
