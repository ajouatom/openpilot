from __future__ import annotations

from collections import deque
import ctypes
import os
from pathlib import Path
import queue
import subprocess
import threading
import time
from typing import Any

from cluster_usb_display import TuringUsbDisplay


OPENPILOT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_H264_LIBRARY = OPENPILOT_ROOT / "system" / "loggerd" / "libcluster_h264_encoder_bridge.so"
DEFAULT_H264_HELPER = OPENPILOT_ROOT / "system" / "loggerd" / "cluster_h264_encoder_cli"
DEFAULT_H264_DEVICE = "/dev/v4l/by-path/platform-aa00000.qcom_vidc-video-index1"

NATIVE_INPUT_FORMATS = {"auto": 0, "rgb4": 1, "nv12": 2}
NATIVE_RGB4_LAYOUTS = {"axrgb": 0, "rgba": 1, "bgra": 2}
NativePacketCallback = ctypes.CFUNCTYPE(
    None,
    ctypes.c_void_p,
    ctypes.c_size_t,
    ctypes.c_uint32,
    ctypes.c_uint64,
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_void_p,
)


class H264UsbPipeline:
    def __init__(
        self,
        usb_display: TuringUsbDisplay,
        width: int,
        height: int,
        fps: int,
        bitrate: str,
        gop: int,
        backend: str,
        library_path: str,
        helper_path: str,
        device_path: str,
        input_format: str,
        rgb4_layout: str,
        requested_chunk_size: int,
        wait_for_ack: bool,
        debug: bool,
    ) -> None:
        self.usb_display = usb_display
        self.width = int(width)
        self.height = int(height)
        self.fps = max(1, int(fps))
        self.bitrate = bitrate
        self.gop = max(1, int(gop))
        self.backend_request = backend
        self.backend_name = backend
        self.library_path = library_path
        self.helper_path = helper_path
        self.device_path = device_path
        self.input_format = input_format
        self.rgb4_layout = rgb4_layout
        self.requested_chunk_size = max(0, int(requested_chunk_size))
        self.wait_for_ack = wait_for_ack
        self.debug = debug
        self.chunk_size = 0
        self._chunks_sent = 0
        self._proc: subprocess.Popen[bytes] | None = None
        self._stdout_thread: threading.Thread | None = None
        self._stderr_thread: threading.Thread | None = None
        self._sender_thread: threading.Thread | None = None
        self._native_lib: ctypes.CDLL | None = None
        self._native_handle: int | None = None
        self._native_callback: Any = None
        self._native_frame_index = 0
        self._packet_queue: queue.Queue[Any] | None = None
        self._condition = threading.Condition()
        self._closing = False
        self._stream_started = False
        self._error: BaseException | None = None
        self._samples: list[tuple[str, float]] = []
        self._stderr_tail: deque[str] = deque(maxlen=20)

    def start(self) -> None:
        if self.backend_request in ("auto", "native"):
            try:
                self._start_native()
                return
            except Exception as exc:
                if self.backend_request == "native":
                    raise
                print(f"Warning: native H264 encoder unavailable, falling back to helper: {exc}", flush=True)
                self._close_native()

        self._start_helper()

    def _start_native(self) -> None:
        library = self._resolve_library()
        lib = ctypes.CDLL(library)
        self._configure_native_library(lib)

        bitrate_bps = self._parse_bitrate_bps(self.bitrate)
        handle = lib.cluster_h264_encoder_bridge_create(
            self.width,
            self.height,
            self.fps,
            bitrate_bps,
            self.gop,
            self.device_path.encode("utf-8"),
            NATIVE_INPUT_FORMATS[self.input_format],
            NATIVE_RGB4_LAYOUTS[self.rgb4_layout],
            1 if self.debug else 0,
        )
        if not handle:
            raise RuntimeError("native H264 encoder bridge allocation failed")
        self._native_lib = lib
        self._native_handle = handle
        self._native_callback = NativePacketCallback(self._native_packet_callback)

        if lib.cluster_h264_encoder_bridge_open(handle) != 0:
            raise RuntimeError(self._native_error_text("native H264 encoder open failed"))

        try:
            self.chunk_size = self.usb_display.start_h264_stream(self.requested_chunk_size)
            self._stream_started = True
        except Exception:
            self._close_native()
            raise

        self._packet_queue = queue.Queue(maxsize=64)
        self._sender_thread = threading.Thread(
            target=self._send_queued_packets,
            name="cluster-usb-h264-native-send",
            daemon=True,
        )
        self._sender_thread.start()

        self.backend_name = "native"
        input_name = self._native_input_format_name()
        input_stride = lib.cluster_h264_encoder_bridge_input_stride(handle)
        print(
            "Starting H264 USB native hardware encoder: "
            f"{self.width}x{self.height}@{self.fps} "
            f"bitrate={bitrate_bps} gop={self.gop} "
            f"input={input_name or self.input_format} stride={input_stride} "
            f"rgb4_layout={self.rgb4_layout} device={self.device_path} "
            f"chunk_ack={'on' if self.wait_for_ack else 'off'}",
            flush=True,
        )

    def _start_helper(self) -> None:
        helper = self._resolve_helper_executable()
        command = self._helper_command(helper)
        self.chunk_size = self.usb_display.start_h264_stream(self.requested_chunk_size)
        self._stream_started = True
        self.backend_name = "helper"
        print(
            "Starting H264 USB helper hardware encoder: "
            f"{self.width}x{self.height}@{self.fps} "
            f"bitrate={self.bitrate} gop={self.gop} "
            f"input={self.input_format} rgb4_layout={self.rgb4_layout} "
            f"device={self.device_path} "
            f"chunk_ack={'on' if self.wait_for_ack else 'off'}",
            flush=True,
        )
        if self.debug:
            print(f"H264 helper encoder command: {' '.join(command)}", flush=True)
        try:
            self._proc = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )
        except Exception:
            self.usb_display.stop_h264_stream()
            self._stream_started = False
            raise

        self._stdout_thread = threading.Thread(
            target=self._read_stdout,
            name="cluster-usb-h264-out",
            daemon=True,
        )
        self._stderr_thread = threading.Thread(
            target=self._read_stderr,
            name="cluster-usb-h264-err",
            daemon=True,
        )
        self._stdout_thread.start()
        self._stderr_thread.start()

    def submit_rgba(self, rgba: Any, width: int, height: int) -> None:
        self.check_error()
        if width != self.width or height != self.height:
            raise RuntimeError(
                f"H264 encoder input size changed from {self.width}x{self.height} "
                f"to {width}x{height}"
            )
        if self._closing:
            raise RuntimeError("H264 USB pipeline is closing")

        if self._native_handle is not None:
            self._submit_rgba_native(rgba)
            return

        proc = self._proc
        if proc is None or proc.stdin is None:
            raise RuntimeError("H264 USB pipeline is not started")

        profile_stage = time.perf_counter()
        try:
            self._write_all(proc.stdin.fileno(), rgba, self.width * self.height * 4)
        except BrokenPipeError as exc:
            self._set_error(exc)
            raise RuntimeError(self._error_text("H264 helper encoder pipe closed")) from exc
        except Exception as exc:
            self._set_error(exc)
            raise
        self._add_sample("usb_h264.write_rgba", profile_stage)
        self.check_error()

    def _submit_rgba_native(self, rgba: Any) -> None:
        lib = self._native_lib
        handle = self._native_handle
        if lib is None or handle is None or self._native_callback is None:
            raise RuntimeError("native H264 USB pipeline is not started")

        byte_count = self.width * self.height * 4
        view = memoryview(rgba)
        if view.nbytes < byte_count:
            raise RuntimeError(
                f"H264 encoder RGBA input is {view.nbytes} bytes, expected at least {byte_count}"
            )
        if not view.contiguous:
            raise RuntimeError("H264 encoder RGBA input must be contiguous")

        profile_stage = time.perf_counter()
        try:
            data_ptr = ctypes.addressof(ctypes.c_uint8.from_buffer(view))
        except TypeError as exc:
            raise RuntimeError("H264 encoder RGBA input must expose a writable buffer") from exc

        timestamp_us = self._native_frame_index * 1000000 // self.fps
        result = lib.cluster_h264_encoder_bridge_encode_rgba(
            handle,
            ctypes.c_void_p(data_ptr),
            byte_count,
            timestamp_us,
            self._native_callback,
            None,
        )
        if result != 0:
            raise RuntimeError(self._native_error_text("native H264 encode failed"))
        self._native_frame_index += 1
        self._add_sample("usb_h264.native_encode_rgba", profile_stage)
        self.check_error()

    def profile_samples(self) -> tuple[tuple[str, float], ...]:
        with self._condition:
            samples = tuple(self._samples)
            self._samples.clear()
        return samples

    def check_error(self) -> None:
        with self._condition:
            error = self._error
            closing = self._closing
        if error is not None:
            raise RuntimeError(self._error_text("H264 USB pipeline failed", error)) from error

        proc = self._proc
        if proc is not None and not closing:
            return_code = proc.poll()
            if return_code is not None:
                raise RuntimeError(self._error_text(f"H264 helper encoder exited with code {return_code}"))

    def close(self) -> None:
        with self._condition:
            self._closing = True
            self._condition.notify_all()

        if self._native_handle is not None:
            self._drain_native()
            self._close_native()

        proc = self._proc
        if proc is not None:
            if proc.stdin is not None:
                try:
                    proc.stdin.close()
                except Exception:
                    pass
            try:
                proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                proc.terminate()
                try:
                    proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=2.0)

        if self._stdout_thread is not None:
            self._stdout_thread.join(timeout=3.0)
        if self._stderr_thread is not None:
            self._stderr_thread.join(timeout=1.0)

        packet_queue = self._packet_queue
        if packet_queue is not None:
            try:
                packet_queue.put(None, timeout=1.0)
            except queue.Full:
                pass
        if self._sender_thread is not None:
            self._sender_thread.join(timeout=3.0)

        if self._stream_started:
            try:
                self.usb_display.stop_h264_stream()
            except Exception as exc:
                print(f"Warning: TURZX H264 stop command skipped: {exc}", flush=True)
            self._stream_started = False

    def _resolve_library(self) -> str:
        path = Path(self.library_path)
        candidates = [path]
        if not path.is_absolute():
            candidates.append(OPENPILOT_ROOT / path)
        for candidate in candidates:
            if candidate.exists():
                return str(candidate)
        raise RuntimeError(
            f"native H264 encoder library not found: {self.library_path}. "
            "Build system/loggerd/libcluster_h264_encoder_bridge.so first."
        )

    def _resolve_helper_executable(self) -> str:
        path = Path(self.helper_path)
        candidates = [path]
        if not path.is_absolute():
            candidates.append(OPENPILOT_ROOT / path)
        for candidate in candidates:
            if candidate.exists():
                return str(candidate)
        raise RuntimeError(
            f"H264 hardware encoder helper not found: {self.helper_path}. "
            "Build system/loggerd/cluster_h264_encoder_cli first."
        )

    def _helper_command(self, helper: str) -> list[str]:
        command = [
            helper,
            "--width",
            str(self.width),
            "--height",
            str(self.height),
            "--fps",
            str(self.fps),
            "--bitrate",
            self.bitrate,
            "--gop",
            str(self.gop),
            "--device",
            self.device_path,
            "--input-format",
            self.input_format,
            "--rgb4-layout",
            self.rgb4_layout,
        ]
        if self.debug:
            command.append("--debug")
        return command

    def _configure_native_library(self, lib: ctypes.CDLL) -> None:
        lib.cluster_h264_encoder_bridge_create.argtypes = [
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_char_p,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
        ]
        lib.cluster_h264_encoder_bridge_create.restype = ctypes.c_void_p
        lib.cluster_h264_encoder_bridge_open.argtypes = [ctypes.c_void_p]
        lib.cluster_h264_encoder_bridge_open.restype = ctypes.c_int
        lib.cluster_h264_encoder_bridge_encode_rgba.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_size_t,
            ctypes.c_uint64,
            NativePacketCallback,
            ctypes.c_void_p,
        ]
        lib.cluster_h264_encoder_bridge_encode_rgba.restype = ctypes.c_int
        lib.cluster_h264_encoder_bridge_drain.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            NativePacketCallback,
            ctypes.c_void_p,
        ]
        lib.cluster_h264_encoder_bridge_drain.restype = ctypes.c_int
        lib.cluster_h264_encoder_bridge_close.argtypes = [ctypes.c_void_p]
        lib.cluster_h264_encoder_bridge_close.restype = None
        lib.cluster_h264_encoder_bridge_destroy.argtypes = [ctypes.c_void_p]
        lib.cluster_h264_encoder_bridge_destroy.restype = None
        lib.cluster_h264_encoder_bridge_last_error.argtypes = [ctypes.c_void_p]
        lib.cluster_h264_encoder_bridge_last_error.restype = ctypes.c_char_p
        lib.cluster_h264_encoder_bridge_input_format_name.argtypes = [ctypes.c_void_p]
        lib.cluster_h264_encoder_bridge_input_format_name.restype = ctypes.c_char_p
        lib.cluster_h264_encoder_bridge_input_stride.argtypes = [ctypes.c_void_p]
        lib.cluster_h264_encoder_bridge_input_stride.restype = ctypes.c_size_t

    def _native_packet_callback(
        self,
        data: int,
        size: int,
        _flags: int,
        _timestamp_us: int,
        _codec_config: int,
        _keyframe: int,
        _opaque: int,
    ) -> None:
        if size <= 0 or not data:
            return
        packet_queue = self._packet_queue
        if packet_queue is None:
            return
        try:
            chunk_size = max(1, self.chunk_size)
            base = int(data)
            for offset in range(0, int(size), chunk_size):
                chunk = ctypes.string_at(base + offset, min(chunk_size, int(size) - offset))
                packet_queue.put(chunk, timeout=1.0)
        except queue.Full as exc:
            self._set_error(RuntimeError("native H264 USB sender queue is full"))
        except BaseException as exc:
            self._set_error(exc)

    def _drain_native(self) -> None:
        lib = self._native_lib
        handle = self._native_handle
        if lib is None or handle is None or self._native_callback is None:
            return
        lib.cluster_h264_encoder_bridge_drain(handle, 250, self._native_callback, None)

    def _close_native(self) -> None:
        lib = self._native_lib
        handle = self._native_handle
        if lib is not None and handle is not None:
            lib.cluster_h264_encoder_bridge_destroy(handle)
        self._native_handle = None
        self._native_lib = None
        self._native_callback = None

    def _native_error_text(self, message: str) -> str:
        lib = self._native_lib
        handle = self._native_handle
        if lib is None or handle is None:
            return message
        error = lib.cluster_h264_encoder_bridge_last_error(handle)
        if error:
            return f"{message}: {error.decode('utf-8', errors='replace')}"
        return message

    def _native_input_format_name(self) -> str:
        lib = self._native_lib
        handle = self._native_handle
        if lib is None or handle is None:
            return ""
        value = lib.cluster_h264_encoder_bridge_input_format_name(handle)
        return "" if not value else value.decode("utf-8", errors="replace")

    @staticmethod
    def _parse_bitrate_bps(value: str) -> int:
        text = value.strip()
        multiplier = 1.0
        if text[-1:].lower() == "k":
            multiplier = 1000.0
            text = text[:-1]
        elif text[-1:].lower() == "m":
            multiplier = 1000000.0
            text = text[:-1]
        parsed = int(float(text) * multiplier + 0.5)
        if parsed <= 0:
            raise ValueError(f"invalid H264 bitrate: {value}")
        return parsed

    def _write_all(self, fd: int, data: Any, byte_count: int) -> None:
        view = memoryview(data)
        if view.nbytes < byte_count:
            raise RuntimeError(
                f"H264 encoder RGBA input is {view.nbytes} bytes, expected at least {byte_count}"
            )

        offset = 0
        while offset < byte_count:
            written = os.write(fd, view[offset:byte_count])
            if written <= 0:
                raise BrokenPipeError("H264 helper encoder pipe wrote zero bytes")
            offset += written

    def _read_stdout(self) -> None:
        proc = self._proc
        if proc is None or proc.stdout is None:
            return

        try:
            fd = proc.stdout.fileno()
            chunk_size = max(1, self.chunk_size)
            while True:
                chunk = os.read(fd, chunk_size)
                if not chunk:
                    return
                self._send_h264_chunk(chunk, chunk_size, source="helper")
        except BaseException as exc:
            with self._condition:
                if not self._closing:
                    self._error = exc
                    print(
                        f"H264 USB stdout worker failed after {self._chunks_sent} chunks: "
                        f"{type(exc).__name__}: {exc}",
                        flush=True,
                    )
                self._condition.notify_all()

    def _send_queued_packets(self) -> None:
        packet_queue = self._packet_queue
        if packet_queue is None:
            return
        try:
            chunk_size = max(1, self.chunk_size)
            while True:
                chunk = packet_queue.get()
                if chunk is None:
                    return
                self._send_h264_chunk(chunk, chunk_size, source="native")
        except BaseException as exc:
            with self._condition:
                if not self._closing:
                    self._error = exc
                self._condition.notify_all()

    def _send_h264_chunk(self, chunk: bytes, chunk_size: int, *, source: str) -> None:
        self._chunks_sent += 1
        if self.debug and (self._chunks_sent <= 5 or self._chunks_sent % 30 == 0):
            head = " ".join(f"{byte:02X}" for byte in chunk[:8])
            print(
                f"H264 chunk {self._chunks_sent}: {source} {len(chunk)} bytes "
                f"head={head} ack={'on' if self.wait_for_ack else 'off'}",
                flush=True,
            )
        profile_stage = time.perf_counter()
        self.usb_display.send_h264_chunk(chunk, wait_for_ack=self.wait_for_ack)
        self._add_sample("usb_h264.send_chunk", profile_stage)

    def _read_stderr(self) -> None:
        proc = self._proc
        if proc is None or proc.stderr is None:
            return

        try:
            while True:
                line = proc.stderr.readline()
                if not line:
                    return
                text = line.decode("utf-8", errors="replace").strip()
                if text:
                    with self._condition:
                        self._stderr_tail.append(text)
                    if self.debug:
                        print(f"H264 encoder: {text}", flush=True)
        except Exception:
            return

    def _add_sample(self, name: str, start_time: float) -> None:
        milliseconds = (time.perf_counter() - start_time) * 1000.0
        with self._condition:
            self._samples.append((name, milliseconds))

    def _set_error(self, error: BaseException) -> None:
        with self._condition:
            self._error = error
            self._condition.notify_all()

    def _error_text(self, message: str, error: BaseException | None = None) -> str:
        parts = [message]
        if error is not None:
            parts.append(f"cause: {type(error).__name__}: {error}")
        with self._condition:
            tail = "\n".join(self._stderr_tail)
        if tail:
            parts.append(f"H264 encoder stderr tail:\n{tail}")
        return "\n".join(parts)
