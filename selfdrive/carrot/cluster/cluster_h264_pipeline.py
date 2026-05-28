from __future__ import annotations

from collections import deque
import os
from pathlib import Path
import subprocess
import threading
import time
from typing import Any

from cluster_usb_display import TuringUsbDisplay


OPENPILOT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_H264_HELPER = OPENPILOT_ROOT / "system" / "loggerd" / "cluster_h264_encoder_cli"
DEFAULT_H264_DEVICE = "/dev/v4l/by-path/platform-aa00000.qcom_vidc-video-index1"


class H264UsbPipeline:
    def __init__(
        self,
        usb_display: TuringUsbDisplay,
        width: int,
        height: int,
        fps: int,
        bitrate: str,
        gop: int,
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
        self._condition = threading.Condition()
        self._closing = False
        self._stream_started = False
        self._error: BaseException | None = None
        self._samples: list[tuple[str, float]] = []
        self._stderr_tail: deque[str] = deque(maxlen=20)

    def start(self) -> None:
        self._start_helper()

    def _start_helper(self) -> None:
        helper = self._resolve_helper_executable()
        command = self._helper_command(helper)
        self.chunk_size = self.usb_display.start_h264_stream(self.requested_chunk_size)
        self._stream_started = True
        print(
            "Starting H264 USB hardware encoder: "
            f"{self.width}x{self.height}@{self.fps} "
            f"bitrate={self.bitrate} gop={self.gop} "
            f"input={self.input_format} rgb4_layout={self.rgb4_layout} "
            f"device={self.device_path} "
            f"chunk_ack={'on' if self.wait_for_ack else 'off'}",
            flush=True,
        )
        if self.debug:
            print(f"H264 hardware encoder command: {' '.join(command)}", flush=True)
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

        proc = self._proc
        if proc is None or proc.stdin is None:
            raise RuntimeError("H264 USB pipeline is not started")

        profile_stage = time.perf_counter()
        try:
            proc.stdin.write(rgba)
        except BrokenPipeError as exc:
            self._set_error(exc)
            raise RuntimeError(self._error_text("H264 hardware encoder pipe closed")) from exc
        except Exception as exc:
            self._set_error(exc)
            raise
        self._add_sample("usb_h264.write_rgba", profile_stage)
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
                raise RuntimeError(self._error_text(f"H264 hardware encoder exited with code {return_code}"))

    def close(self) -> None:
        with self._condition:
            self._closing = True
            self._condition.notify_all()

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

        if self._stream_started:
            try:
                self.usb_display.stop_h264_stream()
            except Exception as exc:
                print(f"Warning: TURZX H264 stop command skipped: {exc}", flush=True)
            self._stream_started = False

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
                self._chunks_sent += 1
                if self.debug and (self._chunks_sent <= 5 or self._chunks_sent % 30 == 0):
                    head = " ".join(f"{byte:02X}" for byte in chunk[:8])
                    print(
                        f"H264 chunk {self._chunks_sent}: {len(chunk)} bytes "
                        f"head={head} ack={'on' if self.wait_for_ack else 'off'}",
                        flush=True,
                    )
                profile_stage = time.perf_counter()
                self.usb_display.send_h264_chunk(chunk, wait_for_ack=self.wait_for_ack)
                self._add_sample("usb_h264.send_chunk", profile_stage)
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
