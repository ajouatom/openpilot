from __future__ import annotations

import sys
import os
import time
from io import BytesIO
from pathlib import Path
from typing import Any

from cluster_utils import clamp


VENDOR_ROOT = Path(__file__).resolve().parent / ".vendor" / "turing-smart-screen-python-main"
VENDOR_LIBRARY = VENDOR_ROOT / "library"
TURZX_USB_VENDOR_ID = 0x1CBE
TURZX_USB_PRODUCT_IDS = {
    0x0092: "TURZX 9.2",
    0x0123: "TURZX 12.3",
}
HUD_MODE_PRODUCT_IDS = {
    1: 0x0092,
    2: 0x0123,
}
MAX_CONSECUTIVE_FRAME_ERRORS = 3
USB_COMMAND_TIMEOUT_MS = 2000
USB_FRAME_TIMEOUT_MS = 2000
USB_COMMAND_GAP_S = 0.2
TURZX_BRIGHTNESS_COMMAND_MAX = 102
_LIBUSB_DLL_DIR_HANDLE = None


def product_id_for_hud_mode(hud_mode: int) -> int | None:
    try:
        return HUD_MODE_PRODUCT_IDS.get(int(hud_mode))
    except Exception:
        return None


def product_label(product_id: int | None) -> str:
    if product_id is None:
        return "TURZX USB"
    return TURZX_USB_PRODUCT_IDS.get(product_id, f"TURZX USB pid=0x{product_id:04x}")


def _add_libusb_search_path_once() -> None:
    global _LIBUSB_DLL_DIR_HANDLE

    libusb = VENDOR_ROOT / "external" / "libusb-1.0" / "libusb-1.0.dll"
    if not libusb.exists():
        return

    dll_dir = str(libusb.parent)
    path_entries = os.environ.get("PATH", "").split(os.pathsep)
    if dll_dir not in path_entries:
        os.environ["PATH"] = dll_dir + os.pathsep + os.environ.get("PATH", "")
    if hasattr(os, "add_dll_directory") and _LIBUSB_DLL_DIR_HANDLE is None:
        _LIBUSB_DLL_DIR_HANDLE = os.add_dll_directory(dll_dir)


def find_supported_usb_product(expected_product_id: int | None = None) -> int | None:
    if not VENDOR_LIBRARY.exists():
        print(f"TURZX vendor library not found: {VENDOR_LIBRARY}", flush=True)
        return None

    _add_libusb_search_path_once()
    try:
        import usb.core  # type: ignore
    except Exception as exc:
        print(f"TURZX USB scan unavailable: {exc}", flush=True)
        return None

    product_ids = [expected_product_id] if expected_product_id is not None else list(TURZX_USB_PRODUCT_IDS)
    for product_id in product_ids:
        try:
            dev = usb.core.find(idVendor=TURZX_USB_VENDOR_ID, idProduct=product_id)
        except Exception as exc:
            print(f"TURZX USB scan failed for pid=0x{product_id:04x}: {exc}", flush=True)
            return None
        if dev is not None:
            return product_id
    return None


class TuringUsbDisplay:
    def __init__(
        self,
        brightness: int = 80,
        display_fps: int = 60,
        jpeg_quality: int = 82,
        jpeg_encoder: str = "auto",
        fast_write: bool = False,
        wait_for_frame_ack: bool = False,
        frame_drain_attempts: int = 2,
        frame_drain_timeout_ms: int = 2,
        fast_frame_drain_attempts: int = 3,
        fast_frame_drain_timeout_ms: int = 2,
    ) -> None:
        self.brightness = int(clamp(brightness, 0, 100))
        self.display_fps = int(clamp(display_fps, 0, 255))
        self.jpeg_quality = int(clamp(jpeg_quality, 1, 95))
        self.jpeg_encoder = jpeg_encoder
        self.fast_write = fast_write
        self.wait_for_frame_ack = wait_for_frame_ack
        self.frame_drain_attempts = max(0, int(frame_drain_attempts))
        self.frame_drain_timeout_ms = max(0, int(frame_drain_timeout_ms))
        self.fast_frame_drain_attempts = max(0, int(fast_frame_drain_attempts))
        self.fast_frame_drain_timeout_ms = max(0, int(fast_frame_drain_timeout_ms))
        self.dev = None
        self.dev_pid: int | None = None
        self.landscape_width = 1920
        self.landscape_height = 480
        self._send_image = None
        self._send_jpeg = None
        self._find_usb_device = None
        self._product_id = None
        self._build_command_packet_header = None
        self._encrypt_command_packet = None
        self._cmd_upload_jpeg = 101
        self._cmd_upload_png = 102
        self._ep_out = None
        self._ep_in = None
        self._dll_dir_handle = None
        self._frame_error_count = 0
        self._turbojpeg = None
        self._turbojpeg_unavailable = False
        self._jpeg_buffer = BytesIO()
        self.profile_enabled = os.environ.get("CLUSTER_PROFILE_USB") == "1"
        self._profile_samples: list[tuple[str, float]] = []

    def set_profile_enabled(self, enabled: bool) -> None:
        self.profile_enabled = enabled

    def clear_profile_samples(self) -> None:
        self._profile_samples.clear()

    def profile_samples(self) -> tuple[tuple[str, float], ...]:
        return tuple(self._profile_samples)

    def _profile_start(self) -> float:
        return time.perf_counter() if self.profile_enabled else 0.0

    def _profile_add(self, name: str, start_time: float) -> None:
        if self.profile_enabled:
            self._profile_samples.append((name, (time.perf_counter() - start_time) * 1000.0))

    def open(self) -> None:
        if not VENDOR_LIBRARY.exists():
            raise RuntimeError(f"TURZX vendor library not found: {VENDOR_LIBRARY}")
        self._add_libusb_search_path()
        if str(VENDOR_ROOT) not in sys.path:
            sys.path.insert(0, str(VENDOR_ROOT))

        from library.lcd.lcd_comm_turing_usb import (  # type: ignore
            CMD_UPLOAD_JPEG,
            CMD_UPLOAD_PNG,
            PRODUCT_ID,
            build_command_packet_header,
            encrypt_command_packet,
            find_usb_device,
            send_image,
            send_jpeg,
        )

        self._send_image = send_image
        self._send_jpeg = send_jpeg
        self._find_usb_device = find_usb_device
        self._product_id = PRODUCT_ID
        self._build_command_packet_header = build_command_packet_header
        self._encrypt_command_packet = encrypt_command_packet
        self._cmd_upload_jpeg = CMD_UPLOAD_JPEG
        self._cmd_upload_png = CMD_UPLOAD_PNG
        self._connect_device()
        try:
            self._initialize_device()
        except RuntimeError:
            print("USB display did not respond during init; resetting device once...")
            self._reset_and_reconnect()
            self._initialize_device()

    def close(self) -> None:
        if self.dev is None:
            self._ep_out = None
            self._ep_in = None
            return

        try:
            self._send_brightness(0, "brightness-off")
        except Exception as exc:
            print(f"Warning: TURZX USB brightness-off command skipped during close: {exc}", flush=True)

        try:
            import usb.util

            usb.util.dispose_resources(self.dev)
        except Exception:
            pass

        self.dev = None
        self.dev_pid = None
        self._ep_out = None
        self._ep_in = None
        self._frame_error_count = 0

    def set_brightness(self, brightness: int, *, force: bool = False) -> bool:
        next_brightness = int(clamp(brightness, 0, 100))
        if next_brightness == self.brightness and not force:
            return False
        self.brightness = next_brightness
        if self.dev is not None:
            self._send_brightness(self.brightness, "brightness")
            return True
        return False

    def _connect_device(self) -> None:
        self.dev, self.dev_pid = self._find_usb_device()
        self._cache_out_endpoint()
        portrait_width, portrait_height = self._product_id[self.dev_pid]
        self.landscape_width = portrait_height
        self.landscape_height = portrait_width

    def _initialize_device(self) -> None:
        if self.dev is None:
            raise RuntimeError("USB display is not open")

        self._send_command(10, "sync")
        time.sleep(USB_COMMAND_GAP_S)
        if self.display_fps > 0:
            self._send_optional_command(15, "frame-rate", {8: self.display_fps})
        self._send_brightness(self.brightness, "brightness")

    def _send_brightness(self, brightness: int, name: str) -> None:
        value = int(clamp(brightness, 0, 100) / 100 * TURZX_BRIGHTNESS_COMMAND_MAX)
        self._send_optional_command(
            14,
            name,
            {8: value},
            log=False,
            no_ack_gap_s=0.0,
            no_ack_drain_attempts=0,
        )

    def _send_command(
        self,
        command_id: int,
        name: str,
        fields: dict[int, int] | None = None,
        *,
        expect_response: bool = True,
        log: bool = True,
        no_ack_gap_s: float = USB_COMMAND_GAP_S,
        no_ack_drain_attempts: int = 5,
    ) -> bytes:
        if self._build_command_packet_header is None or self._encrypt_command_packet is None:
            raise RuntimeError("USB command helpers are not initialized")
        packet = self._build_command_packet_header(command_id)
        if fields:
            for index, value in fields.items():
                packet[index] = value & 0xFF
        if log:
            print(f"Sending {name} command (ID {command_id})...")
        payload = self._encrypt_command_packet(packet)
        if not expect_response:
            self._write_payload_no_ack(
                payload,
                f"TURZX USB {name} command write failed",
                timeout_ms=USB_COMMAND_TIMEOUT_MS,
            )
            if no_ack_gap_s > 0.0:
                time.sleep(no_ack_gap_s)
            self._drain_input(attempts=no_ack_drain_attempts)
            return b""
        return self._write_payload_checked(
            payload,
            f"TURZX USB {name} command timed out",
            timeout_ms=USB_COMMAND_TIMEOUT_MS,
        )

    def _send_optional_command(
        self,
        command_id: int,
        name: str,
        fields: dict[int, int] | None = None,
        *,
        log: bool = True,
        no_ack_gap_s: float = USB_COMMAND_GAP_S,
        no_ack_drain_attempts: int = 5,
    ) -> None:
        try:
            self._send_command(
                command_id,
                name,
                fields,
                expect_response=False,
                log=log,
                no_ack_gap_s=no_ack_gap_s,
                no_ack_drain_attempts=no_ack_drain_attempts,
            )
        except RuntimeError as exc:
            print(f"Warning: optional TURZX USB {name} command skipped: {exc}")

    def _reset_and_reconnect(self) -> None:
        import usb.util

        if self.dev is not None:
            try:
                self.dev.reset()
            except Exception as exc:
                print(f"USB reset failed: {exc}")
            try:
                usb.util.dispose_resources(self.dev)
            except Exception:
                pass
        time.sleep(1.5)
        self._connect_device()

    def _add_libusb_search_path(self) -> None:
        libusb = VENDOR_ROOT / "external" / "libusb-1.0" / "libusb-1.0.dll"
        if not libusb.exists():
            return

        dll_dir = str(libusb.parent)
        path_entries = os.environ.get("PATH", "").split(os.pathsep)
        if dll_dir not in path_entries:
            os.environ["PATH"] = dll_dir + os.pathsep + os.environ.get("PATH", "")
        if hasattr(os, "add_dll_directory") and self._dll_dir_handle is None:
            self._dll_dir_handle = os.add_dll_directory(dll_dir)

    def send_png(self, frame: bytes) -> None:
        if self.dev is None or self._send_image is None:
            raise RuntimeError("USB display is not open")
        try:
            self._send_frame(self._cmd_upload_png, frame)
        except Exception as exc:
            self._handle_frame_error(exc)

    def send_jpeg(self, frame: bytes) -> None:
        if self.dev is None or self._send_jpeg is None:
            raise RuntimeError("USB display is not open")
        try:
            self._send_frame(self._cmd_upload_jpeg, frame)
        except Exception as exc:
            self._handle_frame_error(exc)

    def encode_jpeg(self, rgba: Any, width: int, height: int) -> bytes:
        if self.jpeg_encoder == "turbojpeg" or (
            self.jpeg_encoder == "auto" and not self._turbojpeg_unavailable
        ):
            try:
                return self._encode_jpeg_turbojpeg(rgba, width, height)
            except ImportError:
                if self.jpeg_encoder == "turbojpeg":
                    raise
                self._turbojpeg_unavailable = True
            except Exception:
                if self.jpeg_encoder == "turbojpeg":
                    raise
                self._turbojpeg_unavailable = True
        return self._encode_jpeg_pillow(rgba, width, height)

    def _encode_jpeg_turbojpeg(self, rgba: Any, width: int, height: int) -> bytes:
        profile_stage = self._profile_start()
        import numpy as np
        import turbojpeg  # type: ignore

        self._profile_add("usb.encode.turbojpeg_import", profile_stage)

        profile_stage = self._profile_start()
        rgba_array = np.frombuffer(rgba, dtype=np.uint8).reshape((height, width, 4))
        self._profile_add("usb.encode.turbojpeg_rgba_view", profile_stage)

        profile_stage = self._profile_start()
        jpeg = self._turbojpeg_encode_array(turbojpeg, rgba_array)
        self._profile_add("usb.encode.turbojpeg_encode", profile_stage)
        return jpeg

    def _turbojpeg_encode_array(self, turbojpeg_module, rgba_array) -> bytes:
        if hasattr(turbojpeg_module, "TurboJPEG"):
            if self._turbojpeg is None:
                self._turbojpeg = turbojpeg_module.TurboJPEG()
            pixel_format = getattr(turbojpeg_module, "TJPF_RGBA", None)
            jpeg_subsample = getattr(turbojpeg_module, "TJSAMP_420", None)
            kwargs = {"quality": int(self.jpeg_quality)}
            if pixel_format is not None:
                kwargs["pixel_format"] = pixel_format
            if jpeg_subsample is not None:
                kwargs["jpeg_subsample"] = jpeg_subsample
            return self._turbojpeg.encode(rgba_array, **kwargs)

        compress = getattr(turbojpeg_module, "compress", None)
        if compress is not None:
            kwargs = {"quality": int(self.jpeg_quality)}
            if hasattr(turbojpeg_module, "PF"):
                kwargs["pixelformat"] = turbojpeg_module.PF.RGBA
            if hasattr(turbojpeg_module, "SAMP"):
                kwargs["subsamp"] = turbojpeg_module.SAMP.Y420
            return compress(rgba_array, **kwargs)

        raise RuntimeError("unsupported turbojpeg Python API")

    def _encode_jpeg_pillow(self, rgba: Any, width: int, height: int) -> bytes:
        from PIL import Image

        profile_stage = self._profile_start()
        image = Image.frombuffer("RGB", (width, height), rgba, "raw", "RGBX", 0, 1)
        self._profile_add("usb.encode.rgba_to_rgbx_view", profile_stage)
        buffer = self._jpeg_buffer
        buffer.seek(0)
        buffer.truncate(0)
        profile_stage = self._profile_start()
        image.save(
            buffer,
            format="JPEG",
            quality=self.jpeg_quality,
            optimize=False,
            progressive=False,
            subsampling=2,
        )
        self._profile_add("usb.encode.jpeg_save", profile_stage)
        profile_stage = self._profile_start()
        jpeg = buffer.getvalue()
        self._profile_add("usb.encode.getvalue", profile_stage)
        return jpeg

    def _cache_out_endpoint(self) -> None:
        import usb.util

        cfg = self.dev.get_active_configuration()
        intf = usb.util.find_descriptor(cfg, bInterfaceNumber=0)
        if intf is None:
            raise RuntimeError("USB interface 0 not found")
        self._ep_out = usb.util.find_descriptor(
            intf,
            custom_match=lambda endpoint: usb.util.endpoint_direction(
                endpoint.bEndpointAddress
            )
            == usb.util.ENDPOINT_OUT,
        )
        if self._ep_out is None:
            raise RuntimeError("Could not find USB OUT endpoint")
        self._ep_in = usb.util.find_descriptor(
            intf,
            custom_match=lambda endpoint: usb.util.endpoint_direction(
                endpoint.bEndpointAddress
            )
            == usb.util.ENDPOINT_IN,
        )
        if self._ep_in is None:
            raise RuntimeError("Could not find USB IN endpoint")

    def _drain_input(self, attempts: int = 3, timeout_ms: int = 20) -> None:
        if self._ep_in is None or attempts <= 0:
            return
        for _ in range(attempts):
            try:
                self._ep_in.read(512, timeout_ms)
            except Exception:
                return

    def _clear_endpoint_halt(self) -> None:
        if self.dev is None:
            return
        for endpoint in (self._ep_out, self._ep_in):
            if endpoint is None:
                continue
            try:
                self.dev.clear_halt(endpoint.bEndpointAddress)
            except Exception:
                pass

    def _write_payload_checked(self, payload: bytes, error_message: str, timeout_ms: int) -> bytes:
        if self._ep_out is None or self._ep_in is None:
            raise RuntimeError("USB endpoints are not open")
        profile_stage = self._profile_start()
        self._clear_endpoint_halt()
        self._drain_input()
        self._profile_add("usb.write_checked.prepare", profile_stage)
        try:
            profile_stage = self._profile_start()
            self._ep_out.write(payload, timeout_ms)
            self._profile_add("usb.write_checked.write", profile_stage)
            profile_stage = self._profile_start()
            response = bytes(self._ep_in.read(512, timeout_ms))
            self._profile_add("usb.write_checked.read_ack", profile_stage)
            return response
        except Exception as exc:
            raise RuntimeError(error_message) from exc

    def _write_payload_no_ack(self, payload: bytes, error_message: str, timeout_ms: int) -> None:
        if self._ep_out is None:
            raise RuntimeError("USB OUT endpoint is not open")
        profile_stage = self._profile_start()
        self._clear_endpoint_halt()
        self._drain_input()
        self._profile_add("usb.write_no_ack.prepare", profile_stage)
        try:
            profile_stage = self._profile_start()
            self._ep_out.write(payload, timeout_ms)
            self._profile_add("usb.write_no_ack.write", profile_stage)
        except Exception as exc:
            raise RuntimeError(error_message) from exc

    def _build_frame_payload(self, command_id: int, frame: bytes) -> bytes:
        if self._build_command_packet_header is None or self._encrypt_command_packet is None:
            raise RuntimeError("USB command helpers are not initialized")

        frame_size = len(frame)
        profile_stage = self._profile_start()
        cmd_packet = self._build_command_packet_header(command_id)
        cmd_packet[8] = (frame_size >> 24) & 0xFF
        cmd_packet[9] = (frame_size >> 16) & 0xFF
        cmd_packet[10] = (frame_size >> 8) & 0xFF
        cmd_packet[11] = frame_size & 0xFF
        payload = self._encrypt_command_packet(cmd_packet) + frame
        self._profile_add("usb.frame.build_payload", profile_stage)
        return payload

    def _send_frame(self, command_id: int, frame: bytes) -> None:
        if self.wait_for_frame_ack:
            response = (
                self._send_frame_fast(command_id, frame)
                if self.fast_write
                else self._send_frame_ack(command_id, frame)
            )
            self._check_frame_response(response)
        else:
            self._send_frame_no_ack(command_id, frame, drain_input=not self.fast_write)
            self._frame_error_count = 0

    def _send_frame_ack(self, command_id: int, frame: bytes) -> bytes:
        return self._write_payload_checked(
            self._build_frame_payload(command_id, frame),
            "TURZX USB frame upload timed out",
            timeout_ms=USB_FRAME_TIMEOUT_MS,
        )

    def _send_frame_fast(self, command_id: int, frame: bytes) -> bytes:
        if self._ep_out is None:
            raise RuntimeError("USB OUT endpoint is not open")

        profile_stage = self._profile_start()
        self._clear_endpoint_halt()
        self._drain_input()
        self._profile_add("usb.frame_fast.prepare", profile_stage)
        profile_stage = self._profile_start()
        payload = self._build_frame_payload(command_id, frame)
        self._profile_add("usb.frame_fast.payload", profile_stage)
        profile_stage = self._profile_start()
        self._ep_out.write(payload, USB_FRAME_TIMEOUT_MS)
        self._profile_add("usb.frame_fast.write", profile_stage)
        profile_stage = self._profile_start()
        response = bytes(self._ep_in.read(512, USB_FRAME_TIMEOUT_MS))
        self._profile_add("usb.frame_fast.read_ack", profile_stage)
        return response

    def _send_frame_no_ack(self, command_id: int, frame: bytes, *, drain_input: bool) -> None:
        if self._ep_out is None:
            raise RuntimeError("USB OUT endpoint is not open")

        profile_stage = self._profile_start()
        if drain_input:
            self._drain_input(
                attempts=self.frame_drain_attempts,
                timeout_ms=self.frame_drain_timeout_ms,
            )
        else:
            self._drain_input(
                attempts=self.fast_frame_drain_attempts,
                timeout_ms=self.fast_frame_drain_timeout_ms,
            )
        self._profile_add("usb.frame_no_ack.drain_input", profile_stage)

        profile_stage = self._profile_start()
        payload = self._build_frame_payload(command_id, frame)
        self._profile_add("usb.frame_no_ack.payload", profile_stage)
        profile_stage = self._profile_start()
        self._ep_out.write(payload, USB_FRAME_TIMEOUT_MS)
        self._profile_add("usb.frame_no_ack.write", profile_stage)

    def _check_frame_response(self, response: bytes | None) -> None:
        if not response:
            raise RuntimeError("TURZX USB frame upload timed out")
        self._frame_error_count = 0

    def _handle_frame_error(self, exc: Exception) -> None:
        self._frame_error_count += 1
        print(
            f"USB frame upload failed "
            f"({self._frame_error_count}/{MAX_CONSECUTIVE_FRAME_ERRORS}): {exc}",
            flush=True,
        )
    
        try:
            self._clear_endpoint_halt()
            self._reset_and_reconnect()
            self._initialize_device()
        except Exception as reset_exc:
            print(f"USB recovery failed: {reset_exc}", flush=True)
    
        if self._frame_error_count >= MAX_CONSECUTIVE_FRAME_ERRORS:
            raise RuntimeError(
                "TURZX USB display is not accepting frame data. "
                "Unplug/replug the display, then retry with lower --fps "
                "or lower --usb-jpeg-quality."
            ) from exc
