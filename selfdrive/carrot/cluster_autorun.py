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
HUD_ENCODER_PARAM = "ClusterHudEncoder"
RETRY_INTERVAL_S = 5.0
HUD_CHECK_INTERVAL_S = 5.0
USB_FALLBACK_SCAN_INTERVAL_S = 60.0
NETLINK_KOBJECT_UEVENT = 15
AUTORUN_FPS_ENV = "CLUSTER_AUTORUN_FPS"
AUTORUN_DEFAULT_ENV = {
    "CLUSTER_REALTIME": "1",
}
ENCODER_AUTO = 0
ENCODER_JPEG = 1
ENCODER_HARDWARE = 2
ENCODER_SOFTWARE = 3
ENCODER_NAMES = {
    ENCODER_AUTO: "auto",
    ENCODER_JPEG: "jpeg",
    ENCODER_HARDWARE: "hardware",
    ENCODER_SOFTWARE: "software",
}


def _ensure_cluster_paths() -> None:
    for path in (OPENPILOT_ROOT, CLUSTER_DIR):
        path_text = str(path)
        if path_text not in sys.path:
            sys.path.insert(0, path_text)


def _apply_autorun_defaults() -> None:
    for key, value in AUTORUN_DEFAULT_ENV.items():
        os.environ.setdefault(key, value)


def _cluster_realtime_enabled() -> bool:
    return os.environ.get("CLUSTER_REALTIME", "0").strip().lower() in ("1", "true", "yes", "on")


def _cluster_realtime_cores() -> list[int]:
    cores_text = os.environ.get("CLUSTER_REALTIME_CORES", "0,1,2,3")
    return [int(core.strip()) for core in cores_text.split(",") if core.strip()]


def _set_current_process_affinity(cores: list[int]) -> list[int]:
    if sys.platform != "linux" or not hasattr(os, "sched_setaffinity"):
        return []
    os.sched_setaffinity(0, cores)
    return sorted(os.sched_getaffinity(0))


def _configure_autorun_affinity() -> None:
    if not _cluster_realtime_enabled():
        return
    try:
        cores = _cluster_realtime_cores()
        allowed_cores = _set_current_process_affinity(cores)
        print(f"[cluster_autorun] affinity enabled cores={allowed_cores or cores}", flush=True)
    except Exception as exc:
        print(f"[cluster_autorun] failed to set core affinity: {exc}", flush=True)


def _read_hud_mode(params: Params) -> int:
    try:
        return int(params.get_int(HUD_PARAM))
    except Exception as exc:
        print(f"[cluster_autorun] failed to read {HUD_PARAM}: {exc}", flush=True)
        return 0


def _read_encoder_mode(params: Params) -> int:
    try:
        encoder_mode = int(params.get_int(HUD_ENCODER_PARAM))
    except Exception as exc:
        print(f"[cluster_autorun] failed to read {HUD_ENCODER_PARAM}: {exc}", flush=True)
        return ENCODER_AUTO
    if encoder_mode not in ENCODER_NAMES:
        print(
            f"[cluster_autorun] unsupported {HUD_ENCODER_PARAM}={encoder_mode}; using auto",
            flush=True,
        )
        return ENCODER_AUTO
    return encoder_mode


def _encoder_sequence(encoder_mode: int) -> list[int]:
    if encoder_mode == ENCODER_AUTO:
        return [ENCODER_HARDWARE, ENCODER_SOFTWARE, ENCODER_JPEG]
    return [encoder_mode]


def _encoder_args(encoder_mode: int) -> list[str]:
    if encoder_mode == ENCODER_HARDWARE:
        return ["--usb-codec", "h264", "--usb-h264-backend", "native"]
    if encoder_mode == ENCODER_SOFTWARE:
        return [
            "--usb-codec",
            "h264",
            "--usb-h264-backend",
            "ffmpeg",
            "--usb-h264-ffmpeg-encoder",
            "libx264",
        ]
    return ["--usb-codec", "jpeg", "--usb-jpeg-quality", "68"]


def _cluster_args(hud_mode: int, configured_encoder_mode: int, active_encoder_mode: int) -> list[str]:
    args = [
        "--input",
        "live",
        "--output",
        "usb",
        *_encoder_args(active_encoder_mode),
        "--live-no-can",
        "--cluster-hud-mode",
        str(hud_mode),
        "--cluster-hud-encoder",
        str(configured_encoder_mode),
    ]
    fps = os.environ.get(AUTORUN_FPS_ENV, "").strip()
    if fps:
        args.extend(["--fps", fps])
    return args


def _run_cluster_once(hud_mode: int, encoder_mode: int) -> None:
    from selfdrive.carrot import cluster_run

    previous_argv = sys.argv[:]
    try:
        sequence = _encoder_sequence(encoder_mode)
        for index, active_encoder_mode in enumerate(sequence):
            print(
                f"[cluster_autorun] starting HUD encoder "
                f"{ENCODER_NAMES[active_encoder_mode]} "
                f"(setting={encoder_mode}:{ENCODER_NAMES[encoder_mode]})",
                flush=True,
            )
            try:
                sys.argv = [
                    previous_argv[0],
                    *_cluster_args(hud_mode, encoder_mode, active_encoder_mode),
                ]
                cluster_run.main()
                return
            except Exception:
                if encoder_mode != ENCODER_AUTO or index == len(sequence) - 1:
                    raise
                next_encoder_mode = sequence[index + 1]
                print(
                    f"[cluster_autorun] HUD encoder {ENCODER_NAMES[active_encoder_mode]} failed; "
                    f"falling back to {ENCODER_NAMES[next_encoder_mode]}",
                    flush=True,
                )
                traceback.print_exc()
    finally:
        sys.argv = previous_argv


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
    _apply_autorun_defaults()
    _configure_autorun_affinity()
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
        encoder_mode = _read_encoder_mode(params)
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
            _run_cluster_once(hud_mode, encoder_mode)
            next_hud_mode = _read_hud_mode(params)
            next_encoder_mode = _read_encoder_mode(params)
            if next_hud_mode != hud_mode or next_encoder_mode != encoder_mode:
                print(
                    f"[cluster_autorun] HUD setting changed "
                    f"mode {hud_mode}->{next_hud_mode}, "
                    f"encoder {encoder_mode}->{next_encoder_mode}; rechecking",
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
