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

from cluster_config import (
    CLUSTER_CORE_MODE_ALL,
    CLUSTER_CORE_MODE_PARAM,
    CLUSTER_PRIORITY_PARAM,
    normalize_cluster_core_mode,
    normalize_cluster_priority,
)

DEFAULT_REALTIME_CORES = [1, 2, 3, 4]
REALTIME_CORES_ENV = "CLUSTER_REALTIME_CORES"
REALTIME_PRIORITY_ENV = "CLUSTER_REALTIME_PRIORITY"


def _all_cpu_cores() -> list[int]:
    cpu_count = os.cpu_count() or 1
    return list(range(cpu_count))


def _cores_for_mode(core_mode: int) -> list[int]:
    if core_mode == CLUSTER_CORE_MODE_ALL:
        return _all_cpu_cores()
    return DEFAULT_REALTIME_CORES[:]


def _parse_realtime_cores(text: str) -> list[int]:
    normalized = text.strip().lower()
    if normalized in ("all", "*"):
        return _all_cpu_cores()
    return [int(core.strip()) for core in text.split(",") if core.strip()]


def _read_int_param(param_name: str, default: int) -> int:
    try:
        from openpilot.common.params import Params

        return int(Params().get_int(param_name))
    except Exception:
        return default


def _resolved_realtime_cores() -> list[int]:
    cores_text = os.environ.get(REALTIME_CORES_ENV)
    if cores_text:
        return _parse_realtime_cores(cores_text)
    core_mode = normalize_cluster_core_mode(_read_int_param(CLUSTER_CORE_MODE_PARAM, 0))
    return _cores_for_mode(core_mode)


def _resolved_realtime_priority() -> int:
    priority_text = os.environ.get(REALTIME_PRIORITY_ENV)
    if priority_text:
        return normalize_cluster_priority(priority_text)
    return normalize_cluster_priority(_read_int_param(CLUSTER_PRIORITY_PARAM, 10))


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

        cores = _resolved_realtime_cores()
        priority = _resolved_realtime_priority()
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
