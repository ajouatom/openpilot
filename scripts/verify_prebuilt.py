#!/usr/bin/env python3
"""Validate shipped binaries and their inputs before bypassing source compilation."""
import argparse
import hashlib
import json
import platform
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def verify(params_only=False):
  manifest = json.loads((ROOT / "prebuilt").read_text())
  if manifest["format"] != 1 or platform.machine() != manifest["architecture"]:
    raise ValueError("prebuilt architecture/format mismatch")
  if list(sys.version_info[:2]) != manifest["python"]:
    raise ValueError("prebuilt Python ABI mismatch")
  if Path("/VERSION").read_text().strip() != manifest["agnos"]:
    raise ValueError("prebuilt AGNOS version mismatch")
  files = manifest["files"]
  names = manifest["params_files"] if params_only else files
  for name in names:
    path = ROOT / name
    if not path.resolve().is_relative_to(ROOT) or not path.is_file():
      raise ValueError(f"missing/unsafe prebuilt file: {name}")
    if hashlib.file_digest(path.open("rb"), "sha256").hexdigest() != files[name]["sha256"]:
      raise ValueError(f"prebuilt input/artifact changed: {name}")
    if files[name]["executable"] and not (path.stat().st_mode & 0o111):
      raise ValueError(f"prebuilt executable bit missing: {name}")
  return manifest

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--params-only", action="store_true")
  args = parser.parse_args()
  try:
    verify(args.params_only)
  except (OSError, ValueError, KeyError, TypeError) as exc:
    print(f"Prebuilt validation failed: {exc}", file=sys.stderr)
    raise SystemExit(1)
