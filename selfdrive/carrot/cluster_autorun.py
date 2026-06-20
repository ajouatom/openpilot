#!/usr/bin/env python3
from __future__ import annotations

import locale
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
HUD_DEBUG_PARAM = "ClusterHudDebug"
HUD_ENCODER_PARAM = "ClusterHudEncoder"
HUD_LIVE_FPS_PARAM = "ClusterHudLiveFps"
HUD_CORE_MODE_PARAM = "ClusterHudCoreMode"
HUD_PRIORITY_PARAM = "ClusterHudPriority"
IS_ONROAD_PARAM = "IsOnroad"
RETRY_INTERVAL_S = 5.0
HUD_CHECK_INTERVAL_S = 5.0
USB_FALLBACK_SCAN_INTERVAL_S = 5.0
USB_OFF_DIM_INTERVAL_S = 30.0
NETLINK_KOBJECT_UEVENT = 15
AUTORUN_FPS_ENV = "CLUSTER_AUTORUN_FPS"
REALTIME_CORES_ENV = "CLUSTER_REALTIME_CORES"
REALTIME_PRIORITY_ENV = "CLUSTER_REALTIME_PRIORITY"
AUTORUN_DEFAULT_ENV = {
    "CLUSTER_REALTIME": "1",
}
DEFAULT_REALTIME_CORES = [1, 2, 3, 4]
DEFAULT_REALTIME_PRIORITY = 10
CORE_MODE_DEDICATED = 0
CORE_MODE_ALL = 1
EXPLICIT_REALTIME_CORES_ENV = REALTIME_CORES_ENV in os.environ
EXPLICIT_REALTIME_PRIORITY_ENV = REALTIME_PRIORITY_ENV in os.environ
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
INITIAL_ALLOWED_CORES = (
    sorted(os.sched_getaffinity(0))
    if sys.platform == "linux" and hasattr(os, "sched_getaffinity")
    else list(range(os.cpu_count() or 1))
)


def _configure_autorun_locale() -> None:
    for candidate in ("C.UTF-8", "C"):
        try:
            locale.setlocale(locale.LC_ALL, candidate)
        except locale.Error:
            continue
        os.environ["LC_ALL"] = candidate
        os.environ["LC_CTYPE"] = candidate
        os.environ["LANG"] = candidate
        return


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


def _normalize_core_mode(value: object) -> int:
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("all", "all-cores", "all_cores"):
            return CORE_MODE_ALL
        if normalized in ("dedicated", "default", "cluster", "1,2,3,4"):
            return CORE_MODE_DEDICATED
        try:
            value = int(normalized)
        except ValueError:
            return CORE_MODE_DEDICATED
    try:
        mode = int(value)
    except (TypeError, ValueError):
        return CORE_MODE_DEDICATED
    if mode == CORE_MODE_ALL:
        return CORE_MODE_ALL
    return CORE_MODE_DEDICATED


def _normalize_priority(value: object) -> int:
    if isinstance(value, str):
        normalized = value.strip()
        try:
            value = int(normalized)
        except ValueError:
            return DEFAULT_REALTIME_PRIORITY
    try:
        priority = int(value)
    except (TypeError, ValueError):
        return DEFAULT_REALTIME_PRIORITY
    if priority < 1:
        return DEFAULT_REALTIME_PRIORITY
    return min(99, priority)


def _all_realtime_cores() -> list[int]:
    return INITIAL_ALLOWED_CORES[:] or list(range(os.cpu_count() or 1))


def _cores_for_core_mode(core_mode: int) -> list[int]:
    if core_mode == CORE_MODE_ALL:
        return _all_realtime_cores()
    return DEFAULT_REALTIME_CORES[:]


def _parse_realtime_cores(text: str) -> list[int]:
    normalized = text.strip().lower()
    if normalized in ("all", "*"):
        return _all_realtime_cores()
    return [int(core.strip()) for core in text.split(",") if core.strip()]


def _apply_realtime_setting_env(core_mode: int, priority: int) -> None:
    if not EXPLICIT_REALTIME_CORES_ENV:
        os.environ[REALTIME_CORES_ENV] = ",".join(str(core) for core in _cores_for_core_mode(core_mode))
    if not EXPLICIT_REALTIME_PRIORITY_ENV:
        os.environ[REALTIME_PRIORITY_ENV] = str(priority)


def _cluster_realtime_cores() -> list[int]:
    cores_text = os.environ.get(REALTIME_CORES_ENV)
    if cores_text:
        return _parse_realtime_cores(cores_text)
    return DEFAULT_REALTIME_CORES[:]


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


def _read_hud_debug_mode(params: Params) -> int:
    try:
        return int(params.get_int(HUD_DEBUG_PARAM))
    except Exception as exc:
        print(f"[cluster_autorun] failed to read {HUD_DEBUG_PARAM}: {exc}", flush=True)
        return 0


def _read_is_onroad(params: Params) -> bool:
    try:
        return bool(params.get_bool(IS_ONROAD_PARAM))
    except Exception as exc:
        print(f"[cluster_autorun] failed to read {IS_ONROAD_PARAM}: {exc}", flush=True)
        return False


def _hud_output_allowed(params: Params) -> bool:
    return _read_hud_debug_mode(params) >= 1 or _read_is_onroad(params)


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


def _read_live_fps_mode(params: Params) -> int:
    try:
        return int(params.get_int(HUD_LIVE_FPS_PARAM))
    except Exception as exc:
        print(f"[cluster_autorun] failed to read {HUD_LIVE_FPS_PARAM}: {exc}", flush=True)
        return 0


def _read_core_mode(params: Params) -> int:
    try:
        return _normalize_core_mode(params.get_int(HUD_CORE_MODE_PARAM))
    except Exception as exc:
        print(f"[cluster_autorun] failed to read {HUD_CORE_MODE_PARAM}: {exc}", flush=True)
        return CORE_MODE_DEDICATED


def _read_priority(params: Params) -> int:
    try:
        return _normalize_priority(params.get_int(HUD_PRIORITY_PARAM))
    except Exception as exc:
        print(f"[cluster_autorun] failed to read {HUD_PRIORITY_PARAM}: {exc}", flush=True)
        return DEFAULT_REALTIME_PRIORITY


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


def _cluster_args(
    hud_mode: int,
    configured_encoder_mode: int,
    active_encoder_mode: int,
    core_mode: int,
    priority: int,
) -> list[str]:
    args = [
        "--input",
        "live",
        "--output",
        "usb",
        *_encoder_args(active_encoder_mode),
        "--cluster-hud-mode",
        str(hud_mode),
        "--cluster-hud-encoder",
        str(configured_encoder_mode),
        "--cluster-hud-core-mode",
        str(core_mode),
        "--cluster-hud-priority",
        str(priority),
    ]
    fps = os.environ.get(AUTORUN_FPS_ENV, "").strip()
    if fps:
        args.extend(["--fps", fps])
    return args


def _run_cluster_once(hud_mode: int, encoder_mode: int, core_mode: int, priority: int) -> None:
    from selfdrive.carrot import cluster_run

    def run_cluster_entry() -> None:
        try:
            cluster_run.main(exit_on_error=False)
        except SystemExit as exc:
            if exc.code in (None, 0):
                return
            raise RuntimeError(f"cluster_run exited with {exc.code}") from exc

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
                    *_cluster_args(hud_mode, encoder_mode, active_encoder_mode, core_mode, priority),
                ]
                run_cluster_entry()
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
                if not _hud_output_allowed(params):
                    print(
                        f"[cluster_autorun] {HUD_DEBUG_PARAM}=0 and {IS_ONROAD_PARAM}=0; "
                        "stopping HUD output while waiting for USB",
                        flush=True,
                    )
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


def _turn_off_supported_usb_device(expected_product_id: int, reason: str) -> bool:
    _configure_autorun_locale()

    from cluster_usb_display import TuringUsbDisplay, find_supported_usb_product, product_label

    if find_supported_usb_product(expected_product_id) is None:
        return False

    display = TuringUsbDisplay(brightness=0, display_fps=0, expected_product_id=expected_product_id)
    try:
        display.open()
        print(
            f"[cluster_autorun] sent brightness 0 to {product_label(expected_product_id)} ({reason})",
            flush=True,
        )
        return True
    except Exception as exc:
        print(
            f"[cluster_autorun] failed to turn off {product_label(expected_product_id)} ({reason}): {exc}",
            flush=True,
        )
        return False
    finally:
        display.close()


def _wait_for_hud_output_allowed(params: Params, expected_product_id: int) -> int | None:
    from cluster_usb_display import product_id_for_hud_mode

    print(
        f"[cluster_autorun] {HUD_DEBUG_PARAM}=0 and {IS_ONROAD_PARAM}=0; "
        "keeping external HUD output off",
        flush=True,
    )
    next_hud_check = time.monotonic()
    next_off_dim = time.monotonic()
    while True:
        now = time.monotonic()
        if now >= next_hud_check:
            hud_mode = _read_hud_mode(params)
            current_product_id = product_id_for_hud_mode(hud_mode)
            if current_product_id is None:
                print(f"[cluster_autorun] {HUD_PARAM}={hud_mode}; stopping cluster HUD", flush=True)
                return None
            expected_product_id = current_product_id
            if _hud_output_allowed(params):
                return expected_product_id
            next_hud_check = now + HUD_CHECK_INTERVAL_S

        now = time.monotonic()
        if now >= next_off_dim:
            _turn_off_supported_usb_device(expected_product_id, "output disabled")
            next_off_dim = now + USB_OFF_DIM_INTERVAL_S

        time.sleep(max(0.1, min(next_hud_check, next_off_dim) - time.monotonic()))


def main() -> None:
    _configure_autorun_locale()
    _ensure_cluster_paths()
    _apply_autorun_defaults()
    from cluster_usb_display import find_supported_usb_product, product_id_for_hud_mode, product_label

    params = Params()
    while True:
        core_mode = _read_core_mode(params)
        priority = _read_priority(params)
        _apply_realtime_setting_env(core_mode, priority)
        _configure_autorun_affinity()
        hud_mode = _read_hud_mode(params)
        encoder_mode = _read_encoder_mode(params)
        live_fps_mode = _read_live_fps_mode(params)
        expected_product_id = product_id_for_hud_mode(hud_mode)
        if expected_product_id is None:
            print(f"[cluster_autorun] {HUD_PARAM}={hud_mode}; stopping cluster HUD", flush=True)
            return

        if not _hud_output_allowed(params):
            expected_product_id = _wait_for_hud_output_allowed(params, expected_product_id)
            if expected_product_id is None:
                return
            continue

        if find_supported_usb_product(expected_product_id) is None:
            found_product_id = _wait_for_supported_usb_device(
                params,
                expected_product_id,
                "not found or disconnected",
            )
            if found_product_id is None:
                continue
            print(f"[cluster_autorun] found {product_label(found_product_id)}; starting cluster HUD", flush=True)
        else:
            print(f"[cluster_autorun] found {product_label(expected_product_id)}; starting cluster HUD", flush=True)

        try:
            _run_cluster_once(hud_mode, encoder_mode, core_mode, priority)
            next_hud_mode = _read_hud_mode(params)
            next_encoder_mode = _read_encoder_mode(params)
            next_live_fps_mode = _read_live_fps_mode(params)
            next_core_mode = _read_core_mode(params)
            next_priority = _read_priority(params)
            if (
                next_hud_mode != hud_mode
                or next_encoder_mode != encoder_mode
                or next_live_fps_mode != live_fps_mode
                or next_core_mode != core_mode
                or next_priority != priority
            ):
                print(
                    f"[cluster_autorun] HUD setting changed "
                    f"mode {hud_mode}->{next_hud_mode}, "
                    f"encoder {encoder_mode}->{next_encoder_mode}, "
                    f"live_fps {live_fps_mode}->{next_live_fps_mode}, "
                    f"core_mode {core_mode}->{next_core_mode}, "
                    f"priority {priority}->{next_priority}; rechecking",
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
