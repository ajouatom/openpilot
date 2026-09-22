#!/usr/bin/env python3
from __future__ import annotations

import locale
import os
import sys
from pathlib import Path


CARROT_DIR = Path(__file__).resolve().parent
BUNDLE_DIR = CARROT_DIR / "cluster"
OPENPILOT_ROOT = CARROT_DIR.parents[2]

for path in (OPENPILOT_ROOT, BUNDLE_DIR):
    path_text = str(path)
    if path_text not in sys.path:
        sys.path.insert(0, path_text)

def configure_cluster_locale() -> None:
    for candidate in ("C.UTF-8", "C"):
        try:
            locale.setlocale(locale.LC_ALL, candidate)
        except locale.Error:
            continue
        os.environ["LC_ALL"] = candidate
        os.environ["LC_CTYPE"] = candidate
        os.environ["LANG"] = candidate
        return


def configure_cluster_scheduling() -> None:
    from openpilot.common.realtime import drop_realtime
    from openpilot.common.display_scheduling import DisplayScheduler
    from openpilot.system.hardware import TICI

    drop_realtime()
    # Bootstrap on always-online CPUs. The live render loop handles transitions.
    DisplayScheduler(7, enabled=TICI).update(False, force=True)


def main(*, exit_on_error: bool = True) -> None:
    configure_cluster_locale()
    args = sys.argv[1:]
    if "--input" not in args:
        args = ["--input", "live", *args]
    sys.argv = [sys.argv[0], *args]

    configure_cluster_scheduling()
    from main import main as cluster_main

    cluster_main(exit_on_error=exit_on_error)


if __name__ == "__main__":
    main()
