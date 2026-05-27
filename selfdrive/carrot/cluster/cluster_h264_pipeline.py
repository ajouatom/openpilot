from __future__ import annotations

from collections import deque
import os
from pathlib import Path
import shutil
import subprocess
import threading
import time
from typing import Any

from cluster_usb_display import TuringUsbDisplay


class H264UsbPipeline:
    def __init__(
        self,
        usb_display: TuringUsbDisplay,
        width: int,
        height: int,
        fps: int,
        encoder: str,
        bitrate: str,
        gop: int,
        ffmpeg_path: str,
        requested_chunk_size: int,
    ) -> None:
        self.usb_display = usb_display
        self.width = int(width)
        self.height = int(height)
        self.fps = max(1, int(fps))
        self.encoder_request = encoder
        self.encoder_name = encoder
        self.bitrate = bitrate
        self.gop = max(1, int(gop))
        self.ffmpeg_path = ffmpeg_path
        self.requested_chunk_size = max(0, int(requested_chunk_size))
        self.chunk_size = 0
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
        self.encoder_name = self._resolve_encoder()
        command = self._ffmpeg_command(self.encoder_name)
        self.chunk_size = self.usb_display.start_h264_stream(self.requested_chunk_size)
        self._stream_started = True
        print(
            "Starting H264 USB encoder: "
            f"{self.encoder_name} {self.width}x{self.height}@{self.fps} "
            f"bitrate={self.bitrate}",
            flush=True,
        )
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
            raise RuntimeError(self._error_text("ffmpeg H264 encoder pipe closed")) from exc
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
            raise RuntimeError(self._error_text("H264 USB pipeline failed")) from error

        proc = self._proc
        if proc is not None and not closing:
            return_code = proc.poll()
            if return_code is not None:
                raise RuntimeError(self._error_text(f"ffmpeg H264 encoder exited with code {return_code}"))

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

    def _resolve_encoder(self) -> str:
        ffmpeg = self._ffmpeg_executable()
        if self.encoder_request != "auto":
            return self.encoder_request

        encoders = self._available_encoders(ffmpeg)
        for candidate in ("h264_v4l2m2m", "h264_omx", "libx264"):
            if candidate in encoders:
                return candidate
        return "libx264"

    def _ffmpeg_executable(self) -> str:
        path = Path(self.ffmpeg_path)
        if path.exists():
            return str(path)
        found = shutil.which(self.ffmpeg_path)
        if found is None:
            raise RuntimeError(f"ffmpeg executable not found: {self.ffmpeg_path}")
        return found

    def _available_encoders(self, ffmpeg: str) -> set[str]:
        try:
            result = subprocess.run(
                [ffmpeg, "-hide_banner", "-encoders"],
                check=False,
                capture_output=True,
                text=True,
                timeout=5.0,
            )
        except Exception:
            return set()
        names: set[str] = set()
        for line in result.stdout.splitlines():
            fields = line.split()
            if len(fields) >= 2 and fields[0].startswith("V"):
                names.add(fields[1])
        return names

    def _ffmpeg_command(self, encoder: str) -> list[str]:
        ffmpeg = self._ffmpeg_executable()
        command = [
            ffmpeg,
            "-hide_banner",
            "-loglevel",
            "warning",
            "-fflags",
            "nobuffer",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgba",
            "-s:v",
            f"{self.width}x{self.height}",
            "-framerate",
            str(self.fps),
            "-i",
            "pipe:0",
            "-an",
            "-c:v",
            encoder,
            "-b:v",
            self.bitrate,
            "-maxrate",
            self.bitrate,
            "-bufsize",
            self.bitrate,
            "-g",
            str(self.gop),
            "-bf",
            "0",
            "-flags",
            "+low_delay",
        ]
        if encoder == "libx264":
            command.extend(
                [
                    "-preset",
                    "ultrafast",
                    "-tune",
                    "zerolatency",
                    "-profile:v",
                    "baseline",
                    "-pix_fmt",
                    "yuv420p",
                    "-x264-params",
                    f"keyint={self.gop}:min-keyint={self.gop}:scenecut=0:repeat-headers=1",
                ]
            )
        elif encoder == "h264_v4l2m2m":
            command.extend(["-pix_fmt", "nv12"])
        else:
            command.extend(["-pix_fmt", "yuv420p"])

        command.extend(["-flush_packets", "1", "-f", "h264", "pipe:1"])
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
                profile_stage = time.perf_counter()
                self.usb_display.send_h264_chunk(chunk)
                self._add_sample("usb_h264.send_chunk", profile_stage)
        except BaseException as exc:
            with self._condition:
                if not self._closing:
                    self._error = exc
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

    def _error_text(self, message: str) -> str:
        with self._condition:
            tail = "\n".join(self._stderr_tail)
        if tail:
            return f"{message}\nffmpeg stderr tail:\n{tail}"
        return message
