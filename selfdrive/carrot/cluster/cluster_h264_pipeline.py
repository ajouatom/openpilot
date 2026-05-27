from __future__ import annotations

from collections import deque
import os
from pathlib import Path
import shutil
import subprocess
import threading
import time
from typing import Any

from cluster_h264_v4l2 import DEFAULT_V4L2_ENCODER_DEVICE, V4L2H264Encoder
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
        wait_for_ack: bool,
        debug: bool,
        backend: str = "ffmpeg",
        rgb4_order: str = "rgba",
    ) -> None:
        self.usb_display = usb_display
        self.width = int(width)
        self.height = int(height)
        self.fps = max(1, int(fps))
        self.backend_request = backend
        self.backend_name = backend
        self.encoder_request = encoder
        self.encoder_name = encoder
        self.output_muxer_name = "auto"
        self.bitrate = bitrate
        self.gop = max(1, int(gop))
        self.ffmpeg_path = ffmpeg_path
        self.requested_chunk_size = max(0, int(requested_chunk_size))
        self.wait_for_ack = wait_for_ack
        self.debug = debug
        self.rgb4_order = rgb4_order
        self.chunk_size = 0
        self._chunks_sent = 0
        self._proc: subprocess.Popen[bytes] | None = None
        self._v4l2_encoder: V4L2H264Encoder | None = None
        self._stdout_thread: threading.Thread | None = None
        self._stderr_thread: threading.Thread | None = None
        self._condition = threading.Condition()
        self._closing = False
        self._stream_started = False
        self._error: BaseException | None = None
        self._samples: list[tuple[str, float]] = []
        self._stderr_tail: deque[str] = deque(maxlen=20)

    def start(self) -> None:
        if self.backend_request == "auto":
            if Path(DEFAULT_V4L2_ENCODER_DEVICE).exists():
                try:
                    self._start_v4l2()
                    return
                except Exception as exc:
                    print(f"Warning: V4L2 RGB4 H264 backend failed, falling back to ffmpeg: {exc}", flush=True)
                    self._v4l2_encoder = None
                    if self._stream_started:
                        try:
                            self.usb_display.stop_h264_stream()
                        except Exception:
                            pass
                        self._stream_started = False
            self.backend_name = "ffmpeg"
        elif self.backend_request == "v4l2-rgb4":
            self._start_v4l2()
            return

        self._start_ffmpeg()

    def _start_v4l2(self) -> None:
        self.backend_name = "v4l2-rgb4"
        encoder = V4L2H264Encoder(
            self.width,
            self.height,
            self.fps,
            self.bitrate,
            self.gop,
            rgb4_order=self.rgb4_order,
            debug=self.debug,
        )
        profile_stage = time.perf_counter()
        encoder.start()
        self._add_sample("usb_h264.v4l2.start", profile_stage)
        try:
            self.chunk_size = self.usb_display.start_h264_stream(self.requested_chunk_size)
            self._stream_started = True
        except Exception:
            encoder.close()
            raise
        self._v4l2_encoder = encoder
        print(
            "Starting H264 USB encoder: "
            f"v4l2-rgb4 {self.width}x{self.height}@{self.fps} "
            f"bitrate={self.bitrate} gop={self.gop} rgb4_order={self.rgb4_order} "
            f"chunk_ack={'on' if self.wait_for_ack else 'off'}",
            flush=True,
        )

    def _start_ffmpeg(self) -> None:
        self.backend_name = "ffmpeg"
        ffmpeg = self._ffmpeg_executable()
        self.encoder_name = self._resolve_encoder(ffmpeg)
        self.output_muxer_name = self._resolve_output_muxer(ffmpeg)
        command = self._ffmpeg_command(ffmpeg, self.encoder_name, self.output_muxer_name)
        self.chunk_size = self.usb_display.start_h264_stream(self.requested_chunk_size)
        self._stream_started = True
        print(
            "Starting H264 USB encoder: "
            f"{self.encoder_name} {self.width}x{self.height}@{self.fps} "
            f"bitrate={self.bitrate} muxer={self.output_muxer_name} "
            f"chunk_ack={'on' if self.wait_for_ack else 'off'}",
            flush=True,
        )
        if self.debug:
            print(f"H264 ffmpeg command: {' '.join(command)}", flush=True)
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
        if self._v4l2_encoder is not None:
            profile_stage = time.perf_counter()
            packets = self._v4l2_encoder.encode_rgba(rgba, width, height)
            self._add_sample("usb_h264.v4l2.encode_rgba", profile_stage)
            self._add_samples(self._v4l2_encoder.profile_samples())
            for packet in packets:
                self._send_h264_bytes(packet.data, source="v4l2")
            return

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
            raise RuntimeError(self._error_text("H264 USB pipeline failed", error)) from error

        proc = self._proc
        if proc is not None and not closing:
            return_code = proc.poll()
            if return_code is not None:
                raise RuntimeError(self._error_text(f"ffmpeg H264 encoder exited with code {return_code}"))

    def close(self) -> None:
        with self._condition:
            self._closing = True
            self._condition.notify_all()

        if self._v4l2_encoder is not None:
            try:
                self._v4l2_encoder.close()
            finally:
                self._v4l2_encoder = None

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

    def _resolve_encoder(self, ffmpeg: str) -> str:
        if self.encoder_request != "auto":
            return self.encoder_request

        encoders = self._available_encoders(ffmpeg)
        h264_encoders = sorted(name for name in encoders if "264" in name)
        print(f"ffmpeg H264 encoders visible: {', '.join(h264_encoders) or 'none'}", flush=True)
        for candidate in ("h264_v4l2m2m", "h264_omx", "libx264"):
            if candidate in encoders:
                return candidate
        return "libx264"

    def _resolve_output_muxer(self, ffmpeg: str) -> str:
        muxers = self._available_muxers(ffmpeg)
        if self.debug:
            h264_muxers = sorted(name for name in muxers if name in ("h264", "rawvideo"))
            print(f"ffmpeg H264 stdout muxers visible: {', '.join(h264_muxers) or 'none'}", flush=True)
        for candidate in ("h264", "rawvideo"):
            if candidate in muxers:
                return candidate
        raise RuntimeError(
            "ffmpeg does not provide a usable raw H264 stdout muxer "
            "(tried h264 and rawvideo)"
        )

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

    def _available_muxers(self, ffmpeg: str) -> set[str]:
        try:
            result = subprocess.run(
                [ffmpeg, "-hide_banner", "-muxers"],
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
            if len(fields) >= 2 and fields[0].endswith("E"):
                names.add(fields[1])
        return names

    def _ffmpeg_command(self, ffmpeg: str, encoder: str, output_muxer: str) -> list[str]:
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

        command.extend(["-flush_packets", "1", "-f", output_muxer, "pipe:1"])
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
        except Exception:
            return

    def _add_sample(self, name: str, start_time: float) -> None:
        milliseconds = (time.perf_counter() - start_time) * 1000.0
        with self._condition:
            self._samples.append((name, milliseconds))

    def _add_samples(self, samples: tuple[tuple[str, float], ...]) -> None:
        if not samples:
            return
        with self._condition:
            self._samples.extend(samples)

    def _send_h264_bytes(self, data: bytes, *, source: str) -> None:
        chunk_size = max(1, self.chunk_size)
        for offset in range(0, len(data), chunk_size):
            chunk = data[offset : offset + chunk_size]
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
            parts.append(f"ffmpeg stderr tail:\n{tail}")
        return "\n".join(parts)
