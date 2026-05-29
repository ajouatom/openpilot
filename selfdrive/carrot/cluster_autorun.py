#!/usr/bin/env python3
from __future__ import annotations

import os
import select
import socket
import sys
import time
import traceback
from pathlib import Path

from openpilot.common.params import Params


CARROT_DIR = Path(__file__).resolve().parent
CLUSTER_DIR = CARROT_DIR / "cluster"
OPENPILOT_ROOT = CARROT_DIR.parents[1]
HUD_PARAM = "ClusterHud"
RETRY_INTERVAL_S = 5.0
HUD_CHECK_INTERVAL_S = 5.0
USB_FALLBACK_SCAN_INTERVAL_S = 60.0
NETLINK_KOBJECT_UEVENT = 15
AUTORUN_FPS_ENV = "CLUSTER_AUTORUN_FPS"
AUTORUN_DEFAULT_ENV = {
    "CLUSTER_REALTIME": "0",
}


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


def _cluster_args(hud_mode: int) -> list[str]:
    args = [
        "--input",
        "live",
        "--output",
        "usb",
        "--usb-codec",
        "jpeg",
        "--usb-jpeg-quality",
        "68",
        "--live-no-can",
        "--cluster-hud-mode",
        str(hud_mode),
    ]
    fps = os.environ.get(AUTORUN_FPS_ENV, "").strip()
    if fps:
        args.extend(["--fps", fps])
    return args


def _run_cluster_once(hud_mode: int) -> None:
    from selfdrive.carrot import cluster_run

    previous_argv = sys.argv[:]
    previous_env = {key: os.environ.get(key) for key in AUTORUN_DEFAULT_ENV}
    try:
        for key, value in AUTORUN_DEFAULT_ENV.items():
            os.environ.setdefault(key, value)
        sys.argv = [previous_argv[0], *_cluster_args(hud_mode)]
        cluster_run.main()
    finally:
        sys.argv = previous_argv
        for key, value in previous_env.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value


def _open_usb_uevent_socket() -> socket.socket | None:
    if not hasattr(socket, "AF_NETLINK"):
        return None

    last_error: OSError | None = None
    for port_id in (0, os.getpid()):
        sock: socket.socket | None = None
        try:
            sock = socket.socket(
                socket.AF_NETLINK,
                socket.SOCK_DGRAM,
                getattr(socket, "NETLINK_KOBJECT_UEVENT", NETLINK_KOBJECT_UEVENT),
            )
            sock.bind((port_id, 1))
            sock.setblocking(False)
            return sock
        except OSError as exc:
            last_error = exc
            try:
                if sock is not None:
                    sock.close()
            except Exception:
                pass

    print(f"[cluster_autorun] USB event monitor unavailable: {last_error}", flush=True)
    return None


def _decode_uevent(payload: bytes) -> dict[str, str]:
    event: dict[str, str] = {}
    for part in payload.decode("utf-8", errors="replace").split("\0"):
        if not part:
            continue
        if "=" in part:
            key, value = part.split("=", 1)
            event[key] = value
        elif "@" in part:
            event.setdefault("ACTION", part.split("@", 1)[0])
    return event


def _parse_hex_int(value: str | None) -> int | None:
    if not value:
        return None
    try:
        return int(value, 16)
    except ValueError:
        return None


def _usb_uevent_matches(payload: bytes, expected_product_id: int) -> bool:
    from cluster_usb_display import TURZX_USB_VENDOR_ID

    event = _decode_uevent(payload)
    if event.get("SUBSYSTEM") != "usb":
        return False

    action = event.get("ACTION")
    if action not in ("add", "bind", "change", "move"):
        return False

    product = event.get("PRODUCT")
    if product:
        parts = product.split("/")
        if len(parts) >= 2:
            vendor_id = _parse_hex_int(parts[0])
            product_id = _parse_hex_int(parts[1])
            return vendor_id == TURZX_USB_VENDOR_ID and product_id == expected_product_id

    vendor_id = _parse_hex_int(event.get("ID_VENDOR_ID"))
    product_id = _parse_hex_int(event.get("ID_MODEL_ID"))
    if vendor_id is not None or product_id is not None:
        return vendor_id == TURZX_USB_VENDOR_ID and product_id == expected_product_id

    return event.get("DEVTYPE") == "usb_device"


def _wait_for_usb_uevent(sock: socket.socket | None, timeout_s: float, expected_product_id: int) -> bool:
    if timeout_s <= 0:
        return False
    if sock is None:
        time.sleep(timeout_s)
        return False

    try:
        readable, _, _ = select.select([sock], [], [], timeout_s)
    except (OSError, ValueError) as exc:
        print(f"[cluster_autorun] USB event wait failed: {exc}", flush=True)
        time.sleep(timeout_s)
        return False

    if not readable:
        return False

    matched = False
    while True:
        try:
            payload = sock.recv(8192)
        except BlockingIOError:
            return matched
        except OSError as exc:
            print(f"[cluster_autorun] USB event read failed: {exc}", flush=True)
            return matched
        matched = _usb_uevent_matches(payload, expected_product_id) or matched


def _wait_for_supported_usb_device(params: Params, expected_product_id: int, reason: str) -> int | None:
    from cluster_usb_display import find_supported_usb_product, product_id_for_hud_mode, product_label

    print(
        f"[cluster_autorun] {product_label(expected_product_id)} {reason}; "
        "waiting for USB event",
        flush=True,
    )
    usb_events = _open_usb_uevent_socket()
    if usb_events is None:
        print(
            f"[cluster_autorun] falling back to USB scan every {USB_FALLBACK_SCAN_INTERVAL_S:.0f}s",
            flush=True,
        )
    else:
        print(
            f"[cluster_autorun] fallback USB scan every {USB_FALLBACK_SCAN_INTERVAL_S:.0f}s",
            flush=True,
        )

    next_hud_check = time.monotonic()
    next_fallback_scan = time.monotonic() + USB_FALLBACK_SCAN_INTERVAL_S
    try:
        while True:
            now = time.monotonic()
            if now >= next_hud_check:
                hud_mode = _read_hud_mode(params)
                current_product_id = product_id_for_hud_mode(hud_mode)
                if current_product_id is None:
                    print(f"[cluster_autorun] {HUD_PARAM}={hud_mode}; stopping cluster HUD", flush=True)
                    return None
                if current_product_id != expected_product_id:
                    expected_product_id = current_product_id
                    next_fallback_scan = now
                next_hud_check = now + HUD_CHECK_INTERVAL_S

            now = time.monotonic()
            if now >= next_fallback_scan:
                found_product_id = find_supported_usb_product(expected_product_id)
                if found_product_id is not None:
                    return found_product_id
                next_fallback_scan = now + USB_FALLBACK_SCAN_INTERVAL_S

            wait_s = max(0.1, min(next_hud_check, next_fallback_scan) - time.monotonic())
            if _wait_for_usb_uevent(usb_events, wait_s, expected_product_id):
                found_product_id = find_supported_usb_product(expected_product_id)
                if found_product_id is not None:
                    return found_product_id
    finally:
        if usb_events is not None:
            usb_events.close()


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
        found_product_id = _wait_for_supported_usb_device(
            params,
            expected_product_id,
            "not found at startup",
        )
        if found_product_id is None:
            return

    print(f"[cluster_autorun] found {product_label(found_product_id)}; starting cluster HUD", flush=True)
    while True:
        hud_mode = _read_hud_mode(params)
        expected_product_id = product_id_for_hud_mode(hud_mode)
        if expected_product_id is None:
            print(f"[cluster_autorun] {HUD_PARAM}={hud_mode}; stopping cluster HUD", flush=True)
            return

        if find_supported_usb_product(expected_product_id) is None:
            found_product_id = _wait_for_supported_usb_device(
                params,
                expected_product_id,
                "disconnected",
            )
            if found_product_id is None:
                return
            print(f"[cluster_autorun] found {product_label(found_product_id)}; starting cluster HUD", flush=True)

        try:
            _run_cluster_once(hud_mode)
            next_hud_mode = _read_hud_mode(params)
            if next_hud_mode != hud_mode:
                print(
                    f"[cluster_autorun] {HUD_PARAM} changed from {hud_mode} to {next_hud_mode}; rechecking",
                    flush=True,
                )
                continue
            print(
                f"[cluster_autorun] cluster HUD exited; retrying in {RETRY_INTERVAL_S:.0f}s",
                flush=True,
            )
        except Exception as exc:
            print(
                f"[cluster_autorun] cluster HUD failed: {exc}; retrying in {RETRY_INTERVAL_S:.0f}s",
                flush=True,
            )
            traceback.print_exc()
        time.sleep(RETRY_INTERVAL_S)


if __name__ == "__main__":
    main()
