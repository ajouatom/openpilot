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
from openpilot.system.hardware import TICI


CARROT_DIR = Path(__file__).resolve().parent
CLUSTER_DIR = CARROT_DIR / "cluster"
OPENPILOT_ROOT = CARROT_DIR.parents[1]
HUD_PARAM = "ClusterHud"
HUD_DEBUG_PARAM = "ClusterHudDebug"
HUD_ENCODER_PARAM = "ClusterHudEncoder"
HUD_LIVE_FPS_PARAM = "ClusterHudLiveFps"
HUD_ORIENTATION_PARAM = "ClusterHudOrientation"
HUD_CORE_MODE_PARAM = "ClusterHudCoreMode"
HUD_PRIORITY_PARAM = "ClusterHudPriority"
IS_ONROAD_PARAM = "IsOnroad"
RETRY_INTERVAL_S = 5.0
HUD_CHECK_INTERVAL_S = 0.1
USB_FALLBACK_SCAN_INTERVAL_S = 5.0
USB_OFF_DIM_INTERVAL_S = 30.0
USBGPU_DISCOVERY_GRACE_S = 3.0
USBGPU_STARTUP_TIMEOUT_S = 60.0
USBGPU_STARTUP_POLL_S = 0.1
USBGPU_DISPLAY_STABILIZE_S = 10.0
USBGPU_DISPLAY_FPS = 5
USBGPU_HARDWARE_SEEN_PARAM = "UsbGpuHardwareSeen"
USBGPU_LOADING_PARAM = "UsbGpuLoading"
USBGPU_ACTIVE_PARAM = "UsbGpuActive"
USBGPU_STARTUP_FAILED_PARAM = "UsbGpuStartupFailed"
NETLINK_KOBJECT_UEVENT = 15
AUTORUN_FPS_ENV = "CLUSTER_AUTORUN_FPS"
REALTIME_CORES_ENV = "CLUSTER_REALTIME_CORES"
REALTIME_PRIORITY_ENV = "CLUSTER_REALTIME_PRIORITY"
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
USB_DISCONNECT_TEXT = ("usb display disconnected", "no such device", "device has been disconnected")
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


def _is_usb_disconnect_error(exc: BaseException) -> bool:
    """Recognize an expected hot-unplug through the wrapped pipeline errors."""
    current: BaseException | None = exc
    seen: set[int] = set()
    while current is not None and id(current) not in seen:
        seen.add(id(current))
        message = str(current).lower()
        if any(marker in message for marker in USB_DISCONNECT_TEXT):
            return True
        current = current.__cause__ or current.__context__
    return False


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
    try:
        cores = _cluster_realtime_cores()
        allowed_cores = _set_current_process_affinity(cores)
        print(f"[cluster_autorun] affinity configured cores={allowed_cores or cores}", flush=True)
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


def _read_orientation(params: Params) -> int | None:
    try:
        orientation = int(params.get_int(HUD_ORIENTATION_PARAM))
    except Exception as exc:
        print(f"[cluster_autorun] failed to read {HUD_ORIENTATION_PARAM}: {exc}", flush=True)
        return None
    return orientation if orientation in (0, 2) else None


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
        return [ENCODER_HARDWARE, ENCODER_SOFTWARE, ENCODER_JPEG] if TICI else [ENCODER_SOFTWARE, ENCODER_JPEG]
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
    output_mode: str = "usb",
    usbgpu_active: bool = False,
) -> list[str]:
    args = [
        "--input",
        "live",
        "--output",
        output_mode,
        "--cluster-hud-mode",
        str(hud_mode),
        "--cluster-hud-encoder",
        str(configured_encoder_mode),
        "--cluster-hud-core-mode",
        str(core_mode),
        "--cluster-hud-priority",
        str(priority),
    ]
    if output_mode in ("usb", "both"):
        # Standalone carrot_navi owns TCP 7714; live input consumes its carrotNavi cereal service.
        args[4:4] = _encoder_args(active_encoder_mode)
        if usbgpu_active:
            args.extend(["--usb-display-fps", str(USBGPU_DISPLAY_FPS)])
    fps = os.environ.get(AUTORUN_FPS_ENV, "").strip()
    if output_mode in ("usb", "both") and usbgpu_active:
        # Cap rendering/encoding as well as the TURZX controller setting. The
        # latter alone does not reduce H264 uploads on the shared USB bus.
        args.extend(["--fps", str(USBGPU_DISPLAY_FPS)])
    elif fps:
        args.extend(["--fps", fps])
    return args


def _run_cluster_once(
    hud_mode: int,
    encoder_mode: int,
    core_mode: int,
    priority: int,
    output_mode: str = "usb",
    usbgpu_active: bool = False,
) -> None:
    from selfdrive.carrot import cluster_run
    from cluster_h264_pipeline import H264PipelineInitializationError

    def run_cluster_entry() -> None:
        try:
            cluster_run.main(exit_on_error=False)
        except SystemExit as exc:
            if exc.code in (None, 0):
                return
            raise RuntimeError(f"cluster_run exited with {exc.code}") from exc

    previous_argv = sys.argv[:]
    try:
        sequence = _encoder_sequence(encoder_mode) if output_mode in ("usb", "both") else [encoder_mode]
        for index, active_encoder_mode in enumerate(sequence):
            if output_mode in ("usb", "both"):
                print(
                    f"[cluster_autorun] starting HUD encoder "
                    f"{ENCODER_NAMES[active_encoder_mode]} "
                    f"(setting={encoder_mode}:{ENCODER_NAMES[encoder_mode]})",
                    flush=True,
                )
            else:
                print("[cluster_autorun] starting HUD window fallback", flush=True)
            try:
                sys.argv = [
                    previous_argv[0],
                    *_cluster_args(
                        hud_mode,
                        encoder_mode,
                        active_encoder_mode,
                        core_mode,
                        priority,
                        output_mode,
                        usbgpu_active,
                    ),
                ]
                run_cluster_entry()
                return
            except H264PipelineInitializationError:
                if output_mode not in ("usb", "both") or encoder_mode != ENCODER_AUTO or index == len(sequence) - 1:
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


def _usbgpu_startup_expected() -> bool:
    try:
        from openpilot.selfdrive.modeld.helpers import usbgpu_compiled, usbgpu_present

        return usbgpu_present() and usbgpu_compiled()
    except Exception as exc:
        print(f"[cluster_autorun] could not inspect eGPU startup state: {exc}", flush=True)
        return False


def _wait_for_usbgpu_startup(params: Params) -> None:
    """Keep the shared USB display idle until initial eGPU transfers finish."""
    remembered = params.get_bool(USBGPU_HARDWARE_SEEN_PARAM)
    expected = _usbgpu_startup_expected()
    discovery_deadline = time.monotonic() + USBGPU_DISCOVERY_GRACE_S
    while remembered and not expected and time.monotonic() < discovery_deadline:
        if (
            params.get_bool(USBGPU_LOADING_PARAM)
            or params.get_bool(USBGPU_ACTIVE_PARAM)
            or params.get_bool(USBGPU_STARTUP_FAILED_PARAM)
        ):
            expected = True
            break
        time.sleep(USBGPU_STARTUP_POLL_S)
        expected = _usbgpu_startup_expected()

    if not expected:
        return

    print("[cluster_autorun] waiting for eGPU startup before opening USB display", flush=True)
    startup_deadline = time.monotonic() + USBGPU_STARTUP_TIMEOUT_S
    while time.monotonic() < startup_deadline:
        loading = params.get_bool(USBGPU_LOADING_PARAM)
        active = params.get_bool(USBGPU_ACTIVE_PARAM)
        failed = params.get_bool(USBGPU_STARTUP_FAILED_PARAM)
        if not loading and active:
            print(
                f"[cluster_autorun] eGPU startup complete; keeping USB display idle for "
                f"{USBGPU_DISPLAY_STABILIZE_S:.0f}s",
                flush=True,
            )
            stabilize_deadline = time.monotonic() + USBGPU_DISPLAY_STABILIZE_S
            while time.monotonic() < stabilize_deadline:
                if not params.get_bool(USBGPU_ACTIVE_PARAM):
                    print(
                        "[cluster_autorun] eGPU became inactive during stabilization; "
                        "starting USB display",
                        flush=True,
                    )
                    return
                time.sleep(min(USBGPU_STARTUP_POLL_S, stabilize_deadline - time.monotonic()))
            print(
                f"[cluster_autorun] eGPU stable; starting USB display at {USBGPU_DISPLAY_FPS} fps",
                flush=True,
            )
            return
        if not loading and failed:
            print("[cluster_autorun] eGPU startup failed; starting USB display after fallback", flush=True)
            return
        time.sleep(USBGPU_STARTUP_POLL_S)

    print(
        f"[cluster_autorun] eGPU startup wait timed out after {USBGPU_STARTUP_TIMEOUT_S:.0f}s; "
        "starting USB display",
        flush=True,
    )


def main() -> None:
    _configure_autorun_locale()
    _ensure_cluster_paths()
    from cluster_usb_display import find_supported_usb_product, product_id_for_hud_mode, product_label

    params = Params()
    params.put_bool_nonblocking("ClusterHudConnected", False)
    while True:
        core_mode = _read_core_mode(params)
        priority = _read_priority(params)
        _apply_realtime_setting_env(core_mode, priority)
        _configure_autorun_affinity()
        hud_mode = _read_hud_mode(params)
        encoder_mode = _read_encoder_mode(params)
        live_fps_mode = _read_live_fps_mode(params)
        orientation = _read_orientation(params)
        expected_product_id = product_id_for_hud_mode(hud_mode)
        if expected_product_id is None:
            print(f"[cluster_autorun] {HUD_PARAM}={hud_mode}; stopping cluster HUD", flush=True)
            return

        if not _hud_output_allowed(params):
            expected_product_id = _wait_for_hud_output_allowed(params, expected_product_id)
            if expected_product_id is None:
                return
            continue

        _wait_for_usbgpu_startup(params)

        if find_supported_usb_product(expected_product_id) is None:
            if not TICI:
                print(
                    f"[cluster_autorun] {product_label(expected_product_id)} not found on PC; "
                    "starting window-only HUD",
                    flush=True,
                )
                try:
                    _run_cluster_once(hud_mode, encoder_mode, core_mode, priority, output_mode="window")
                    continue
                except Exception as exc:
                    print(
                        f"[cluster_autorun] cluster HUD window failed: {exc}; "
                        f"retrying in {RETRY_INTERVAL_S:.0f}s",
                        flush=True,
                    )
                    traceback.print_exc()
                    time.sleep(RETRY_INTERVAL_S)
                    continue
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
            _run_cluster_once(
                hud_mode,
                encoder_mode,
                core_mode,
                priority,
                usbgpu_active=params.get_bool(USBGPU_ACTIVE_PARAM),
            )
            next_hud_mode = _read_hud_mode(params)
            next_encoder_mode = _read_encoder_mode(params)
            next_live_fps_mode = _read_live_fps_mode(params)
            next_orientation = _read_orientation(params)
            next_core_mode = _read_core_mode(params)
            next_priority = _read_priority(params)
            if (
                next_hud_mode != hud_mode
                or next_encoder_mode != encoder_mode
                or next_live_fps_mode != live_fps_mode
                or (next_orientation is not None and next_orientation != orientation)
                or next_core_mode != core_mode
                or next_priority != priority
            ):
                print(
                    f"[cluster_autorun] HUD setting changed "
                    f"mode {hud_mode}->{next_hud_mode}, "
                    f"encoder {encoder_mode}->{next_encoder_mode}, "
                    f"live_fps {live_fps_mode}->{next_live_fps_mode}, "
                    f"orientation {orientation}->{next_orientation}, "
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
            if _is_usb_disconnect_error(exc):
                print(
                    f"[cluster_autorun] USB display disconnected; retrying in {RETRY_INTERVAL_S:.0f}s",
                    flush=True,
                )
            else:
                print(
                    f"[cluster_autorun] cluster HUD failed: {exc}; retrying in {RETRY_INTERVAL_S:.0f}s",
                    flush=True,
                )
                traceback.print_exc()
        time.sleep(RETRY_INTERVAL_S)


if __name__ == "__main__":
    main()
