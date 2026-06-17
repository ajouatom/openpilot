from __future__ import annotations

from collections.abc import Iterable
import threading
import time

from cluster_usb_display import TuringUsbDisplay


class AsyncJpegUsbPipeline:
    def __init__(self, usb_display: TuringUsbDisplay) -> None:
        self.usb_display = usb_display
        self._condition = threading.Condition()
        self._pending_rgba: tuple[bytes, int, int] | None = None
        self._closing = False
        self._error: BaseException | None = None
        self._samples: list[tuple[str, float]] = []
        self._thread = threading.Thread(target=self._run, name="cluster-usb-jpeg", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def submit_rgba(self, rgba: bytes, width: int, height: int) -> None:
        self.check_error()
        with self._condition:
            self._pending_rgba = (rgba, width, height)
            self._condition.notify()

    def wait_for_capacity(self, timeout: float | None = None) -> bool:
        deadline = None if timeout is None else time.perf_counter() + max(0.0, timeout)
        with self._condition:
            while self._pending_rgba is not None and not self._closing and self._error is None:
                if deadline is None:
                    self._condition.wait()
                    continue
                remaining = deadline - time.perf_counter()
                if remaining <= 0.0:
                    return False
                self._condition.wait(timeout=remaining)
            if self._error is not None:
                raise RuntimeError("asynchronous USB JPEG pipeline failed") from self._error
            return self._pending_rgba is None

    def profile_samples(self) -> tuple[tuple[str, float], ...]:
        with self._condition:
            samples = tuple(self._samples)
            self._samples.clear()
        return samples

    def check_error(self) -> None:
        with self._condition:
            error = self._error
        if error is not None:
            raise RuntimeError("asynchronous USB JPEG pipeline failed") from error

    def close(self) -> None:
        with self._condition:
            self._closing = True
            self._condition.notify()
        self._thread.join(timeout=3.0)

    def _add_sample(self, name: str, start_time: float) -> None:
        milliseconds = (time.perf_counter() - start_time) * 1000.0
        with self._condition:
            self._samples.append((name, milliseconds))

    def _add_samples(self, samples: Iterable[tuple[str, float]]) -> None:
        if not samples:
            return
        with self._condition:
            self._samples.extend(samples)

    def _take_pending(self) -> tuple[bytes, int, int] | None:
        with self._condition:
            while self._pending_rgba is None and not self._closing:
                self._condition.wait(timeout=0.1)
            if self._pending_rgba is None:
                return None
            pending = self._pending_rgba
            self._pending_rgba = None
            self._condition.notify_all()
            return pending

    def _run(self) -> None:
        while True:
            pending = self._take_pending()
            if pending is None:
                return
            rgba, width, height = pending
            try:
                self.usb_display.clear_profile_samples()
                profile_stage = time.perf_counter()
                jpeg = self.usb_display.encode_jpeg(rgba, width, height)
                self._add_sample("usb_async.encode_jpeg", profile_stage)

                profile_stage = time.perf_counter()
                self.usb_display.send_jpeg(jpeg)
                self._add_sample("usb_async.send_jpeg", profile_stage)
                self._add_samples(self.usb_display.profile_samples())
                self.usb_display.clear_profile_samples()
            except BaseException as exc:
                with self._condition:
                    self._error = exc
                    self._condition.notify_all()
                return
