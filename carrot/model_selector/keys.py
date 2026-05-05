"""Ed25519 public keys for verifying the carrot model manifest.

Raw 32-byte public keys, base64-encoded.  Keep in sync with the signing tooling
that produces `models.json`.
"""
from __future__ import annotations

MODEL_SIGNING_KEYS: dict[str, str] = {
    # Current key — generated 2025-01
    "key_2025_01": "yFPR4om9LyYvQjzRzSiyyso9wc2bP1egmg/PjKa79fg=",
}

# Bumped when incompatible selector changes ship; manifests declare
# `minimumSelectorVersion` and are rejected if this constant is lower.
MODEL_SELECTOR_VERSION = 3
