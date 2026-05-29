#!/usr/bin/env python3
from __future__ import annotations

import locale
import os
import sys
from pathlib import Path


CARROT_DIR = Path(__file__).resolve().parent
BUNDLE_DIR = CARROT_DIR / "cluster"
OPENPILOT_ROOT = CARROT_DIR.parents[1]

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


def configure_cluster_realtime() -> None:
    realtime_enabled = os.environ.get("CLUSTER_REALTIME", "0").strip().lower() in ("1", "true", "yes", "on")
    if not realtime_enabled:
        return

    try:
        from openpilot.common.realtime import config_realtime_process

        cores_text = os.environ.get("CLUSTER_REALTIME_CORES", "0,1,2,3")
        cores = [int(core.strip()) for core in cores_text.split(",") if core.strip()]
        priority = int(os.environ.get("CLUSTER_REALTIME_PRIORITY", "55"))
        config_realtime_process(cores, priority)
        print(f"[cluster_run] realtime enabled cores={cores} priority={priority}", flush=True)
    except Exception as exc:
        print(f"[cluster_run] failed to configure realtime process: {exc}", flush=True)


def main() -> None:
    configure_cluster_locale()
    args = sys.argv[1:]
    if "--input" not in args:
        args = ["--input", "live", *args]
    sys.argv = [sys.argv[0], *args]

    configure_cluster_realtime()
    from main import main as cluster_main

    cluster_main()


if __name__ == "__main__":
    main()
