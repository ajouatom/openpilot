#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path


CARROT_DIR = Path(__file__).resolve().parent
BUNDLE_DIR = CARROT_DIR / "cluster"
OPENPILOT_ROOT = CARROT_DIR.parents[1]

for path in (OPENPILOT_ROOT, BUNDLE_DIR):
    path_text = str(path)
    if path_text not in sys.path:
        sys.path.insert(0, path_text)


def main() -> None:
    args = sys.argv[1:]
    if "--input" not in args:
        args = ["--input", "live", *args]
    sys.argv = [sys.argv[0], *args]

    from main import main as cluster_main

    cluster_main()


if __name__ == "__main__":
    main()
