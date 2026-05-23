#!/usr/bin/env python3
from __future__ import annotations

import sys
import time
from pathlib import Path

from openpilot.common.params import Params


CARROT_DIR = Path(__file__).resolve().parent
CLUSTER_DIR = CARROT_DIR / "cluster"
OPENPILOT_ROOT = CARROT_DIR.parents[1]
HUD_PARAM = "ClusterHud"
RETRY_INTERVAL_S = 5.0


def _ensure_cluster_paths() -> None:
    for path in (OPENPILOT_ROOT, CLUSTER_DIR):
        path_text = str(path)
        if path_text not in sys.path:
            sys.path.insert(0, path_text)


def _read_hud_mode(params: Params) -> int:
    try:
        return int(params.get_int(HUD_PARAM))
    except Exception as exc:
        print(f"[cluster_autorun] failed to read {HUD_PARAM}: {exc}", flush=True)
        return 0


def _cluster_args() -> list[str]:
    return [
        "--input",
        "live",
        "--output",
        "usb",
        "--usb-codec",
        "jpeg",
        "--usb-jpeg-quality",
        "68",
    ]


def _run_cluster_once() -> None:
    from selfdrive.carrot import cluster_run

    previous_argv = sys.argv[:]
    try:
        sys.argv = [previous_argv[0], *_cluster_args()]
        cluster_run.main()
    finally:
        sys.argv = previous_argv


def main() -> None:
    _ensure_cluster_paths()
    from cluster_usb_display import find_supported_usb_product, product_id_for_hud_mode, product_label

    params = Params()
    hud_mode = _read_hud_mode(params)
    expected_product_id = product_id_for_hud_mode(hud_mode)
    if expected_product_id is None:
        print(f"[cluster_autorun] {HUD_PARAM}={hud_mode}; HUD disabled", flush=True)
        return

    found_product_id = find_supported_usb_product(expected_product_id)
    if found_product_id is None:
        print(
            f"[cluster_autorun] {product_label(expected_product_id)} not found at startup; "
            "not retrying until the HUD setting or manager is restarted",
            flush=True,
        )
        return

    print(f"[cluster_autorun] found {product_label(found_product_id)}; starting cluster HUD", flush=True)
    while True:
        hud_mode = _read_hud_mode(params)
        expected_product_id = product_id_for_hud_mode(hud_mode)
        if expected_product_id is None:
            print(f"[cluster_autorun] {HUD_PARAM}={hud_mode}; stopping cluster HUD", flush=True)
            return

        if find_supported_usb_product(expected_product_id) is None:
            print(
                f"[cluster_autorun] {product_label(expected_product_id)} disconnected; "
                f"retrying in {RETRY_INTERVAL_S:.0f}s",
                flush=True,
            )
            time.sleep(RETRY_INTERVAL_S)
            continue

        try:
            _run_cluster_once()
            print(
                f"[cluster_autorun] cluster HUD exited; retrying in {RETRY_INTERVAL_S:.0f}s",
                flush=True,
            )
        except Exception as exc:
            print(
                f"[cluster_autorun] cluster HUD failed: {exc}; retrying in {RETRY_INTERVAL_S:.0f}s",
                flush=True,
            )
        time.sleep(RETRY_INTERVAL_S)


if __name__ == "__main__":
    main()
