"""Validate an installed custom model directory.

A directory is considered valid when it contains either:
  - new architecture (supercombo/lebowski): a single unified
    driving_tinygrad.pkl (metadata + model JIT + warp JITs bundled inside), or
  - legacy split architecture:
      driving_vision_tinygrad.pkl + driving_vision_metadata.pkl (required)
      driving_on_policy_* or driving_policy_* (either acceptable)
    The off-policy pair is optional and only activates the 3-model architecture.
"""
from __future__ import annotations

from pathlib import Path

from .config import SUPERCOMBO_PKL_NAME


def _pair_exists(base: Path, stem: str) -> bool:
    pkl = base / f"{stem}_tinygrad.pkl"
    meta = base / f"{stem}_metadata.pkl"
    return pkl.exists() and meta.exists() and pkl.stat().st_size > 0 and meta.stat().st_size > 0


def has_vision(base: Path) -> bool:
    return _pair_exists(base, "driving_vision")


def has_on_policy(base: Path) -> bool:
    return _pair_exists(base, "driving_on_policy")


def has_policy(base: Path) -> bool:
    return _pair_exists(base, "driving_policy")


def has_off_policy(base: Path) -> bool:
    return _pair_exists(base, "driving_off_policy")


def has_supercombo(base: Path) -> bool:
    pkl = base / SUPERCOMBO_PKL_NAME
    return pkl.exists() and pkl.stat().st_size > 0


def is_valid_legacy_model_dir(base: Path) -> bool:
    if not base.exists() or not base.is_dir():
        return False
    return has_vision(base) and (has_on_policy(base) or has_policy(base))


def is_valid_supercombo_model_dir(base: Path) -> bool:
    if not base.exists() or not base.is_dir():
        return False
    return has_supercombo(base)


def is_valid_model_dir(base: Path) -> bool:
    return is_valid_supercombo_model_dir(base) or is_valid_legacy_model_dir(base)


def describe(base: Path) -> str:
    """Human-readable status string for logs / API responses."""
    if not base.exists():
        return f"{base}: missing"
    if has_supercombo(base):
        return f"{base}: supercombo"
    parts = []
    parts.append("vision" if has_vision(base) else "NO_VISION")
    if has_on_policy(base):
        parts.append("on_policy")
    elif has_policy(base):
        parts.append("policy")
    else:
        parts.append("NO_POLICY")
    if has_off_policy(base):
        parts.append("off_policy")
    return f"{base}: " + "+".join(parts)
