from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re
import time


PROC_STAT_PATH = Path("/proc/stat")
PROC_MEMINFO_PATH = Path("/proc/meminfo")
KGSL_GPU_BUSY_PATH = Path("/sys/class/kgsl/kgsl-3d0/gpubusy")
KGSL_GPU_FREQ_PATHS = (
    Path("/sys/class/kgsl/kgsl-3d0/devfreq/cur_freq"),
    Path("/sys/class/devfreq/soc:qcom,kgsl-3d0/cur_freq"),
)
KGSL_GPU_MAX_FREQ_PATHS = (
    Path("/sys/class/kgsl/kgsl-3d0/devfreq/max_freq"),
    Path("/sys/class/devfreq/soc:qcom,kgsl-3d0/max_freq"),
)
DEVFREQ_PATH = Path("/sys/class/devfreq")
ENCODER_DEVFREQ_HINTS = ("vidc", "venus", "vcodec", "video")
ENCODER_DEVFREQ_PRIORITY_HINTS = (
    "venus_bus_ddr",
    "venus_bus_llcc",
    "arm9_bus_ddr",
    "bus_cnoc",
    "vidc",
    "venus",
    "vcodec",
    "video",
)
EXCLUDED_DEVFREQ_HINTS = ("kgsl", "gpu", "cpu", "cpubw")


@dataclass(frozen=True, slots=True)
class SystemStats:
    memory_total_bytes: int | None = None
    memory_used_bytes: int | None = None
    memory_used_percent: float | None = None
    gpu_used_percent: float | None = None
    gpu_freq_hz: int | None = None
    gpu_max_freq_hz: int | None = None
    encoder_used_percent: float | None = None
    encoder_freq_hz: int | None = None
    encoder_max_freq_hz: int | None = None
    encoder_source: str | None = None
    cpu_core_percents: tuple[float | None, ...] = ()


class SystemStatsSampler:
    def __init__(self, refresh_interval_s: float = 1.0) -> None:
        self.refresh_interval_s = max(0.1, float(refresh_interval_s))
        self._next_sample_time = 0.0
        self._stats = SystemStats()
        self._previous_linux_cpu_times: tuple[tuple[int, int], ...] | None = None
        self._previous_devfreq_times: dict[str, tuple[int, int]] = {}

    def sample(self, now: float | None = None) -> SystemStats:
        if now is None:
            now = time.perf_counter()
        if now < self._next_sample_time:
            return self._stats

        stats = self._sample_linux()
        if stats is not None:
            self._stats = stats
        self._next_sample_time = now + self.refresh_interval_s
        return self._stats

    def _sample_linux(self) -> SystemStats | None:
        if not PROC_STAT_PATH.exists() and not PROC_MEMINFO_PATH.exists():
            return None

        memory_total, memory_used, memory_percent = self._read_linux_memory()
        gpu_percent = self._read_gpu_percent()
        gpu_freq = self._read_first_int(KGSL_GPU_FREQ_PATHS)
        gpu_max_freq = self._read_first_int(KGSL_GPU_MAX_FREQ_PATHS)
        encoder_percent, encoder_freq, encoder_max_freq, encoder_source = self._read_encoder_devfreq()
        cpu_times = self._read_linux_cpu_times()
        if cpu_times is None:
            cpu_percents: tuple[float | None, ...] = ()
        else:
            cpu_percents = self._linux_cpu_percents(cpu_times)
            self._previous_linux_cpu_times = cpu_times

        return SystemStats(
            memory_total_bytes=memory_total,
            memory_used_bytes=memory_used,
            memory_used_percent=memory_percent,
            gpu_used_percent=gpu_percent,
            gpu_freq_hz=gpu_freq,
            gpu_max_freq_hz=gpu_max_freq,
            encoder_used_percent=encoder_percent,
            encoder_freq_hz=encoder_freq,
            encoder_max_freq_hz=encoder_max_freq,
            encoder_source=encoder_source,
            cpu_core_percents=cpu_percents,
        )

    @staticmethod
    def _read_first_int(paths: tuple[Path, ...]) -> int | None:
        for path in paths:
            value = SystemStatsSampler._read_int_file(path)
            if value is not None:
                return value
        return None

    @staticmethod
    def _read_int_file(path: Path) -> int | None:
        try:
            text = path.read_text(encoding="utf-8", errors="ignore").strip()
        except OSError:
            return None
        match = re.search(r"-?\d+", text)
        if match is None:
            return None
        try:
            return int(match.group(0))
        except ValueError:
            return None

    @staticmethod
    def _read_float_file(path: Path) -> float | None:
        try:
            text = path.read_text(encoding="utf-8", errors="ignore").strip()
        except OSError:
            return None
        match = re.search(r"-?\d+(?:\.\d+)?", text)
        if match is None:
            return None
        try:
            return float(match.group(0))
        except ValueError:
            return None

    @staticmethod
    def _read_gpu_percent() -> float | None:
        try:
            parts = KGSL_GPU_BUSY_PATH.read_text(encoding="utf-8", errors="ignore").strip().split()
        except OSError:
            return None
        if len(parts) < 2:
            return None
        try:
            used = int(parts[0])
            total = int(parts[1])
        except ValueError:
            return None
        if total <= 0:
            return None
        return max(0.0, min(100.0, used / total * 100.0))

    def _read_encoder_devfreq(self) -> tuple[float | None, int | None, int | None, str | None]:
        for path in self._encoder_devfreq_paths():
            used_percent = self._read_devfreq_percent(path)
            cur_freq = self._read_int_file(path / "cur_freq")
            max_freq = self._read_int_file(path / "max_freq")
            if max_freq is None or max_freq <= 0:
                max_freq = self._read_available_max_freq(path)
            if used_percent is None:
                used_percent = self._freq_ratio_percent(cur_freq, max_freq)
            if used_percent is not None or cur_freq is not None:
                return used_percent, cur_freq, max_freq, path.name
        return None, None, None, None

    @classmethod
    def _encoder_devfreq_paths(cls) -> tuple[Path, ...]:
        try:
            paths = tuple(DEVFREQ_PATH.iterdir())
        except OSError:
            return ()
        candidates: list[Path] = []
        for path in paths:
            haystack = cls._devfreq_haystack(path)
            if any(hint in haystack for hint in EXCLUDED_DEVFREQ_HINTS):
                continue
            if any(hint in haystack for hint in ENCODER_DEVFREQ_HINTS):
                candidates.append(path)
        return tuple(sorted(candidates, key=cls._devfreq_priority))

    @staticmethod
    def _devfreq_haystack(path: Path) -> str:
        name = path.name.lower()
        try:
            target = str(path.resolve()).lower()
        except OSError:
            target = ""
        return f"{name} {target}"

    @staticmethod
    def _devfreq_priority(path: Path) -> tuple[int, str]:
        haystack = SystemStatsSampler._devfreq_haystack(path)
        for index, hint in enumerate(ENCODER_DEVFREQ_PRIORITY_HINTS):
            if hint in haystack:
                return index, path.name
        return len(ENCODER_DEVFREQ_PRIORITY_HINTS), path.name

    @staticmethod
    def _read_available_max_freq(path: Path) -> int | None:
        try:
            text = (path / "available_frequencies").read_text(encoding="utf-8", errors="ignore")
        except OSError:
            return None
        values: list[int] = []
        for match in re.finditer(r"\d+", text):
            try:
                value = int(match.group(0))
            except ValueError:
                continue
            if value > 0:
                values.append(value)
        return max(values) if values else None

    def _read_devfreq_percent(self, path: Path) -> float | None:
        load = self._read_float_file(path / "load")
        if load is not None:
            return self._normalize_percent(load)

        busy = self._read_int_file(path / "busy_time")
        total = self._read_int_file(path / "total_time")
        if busy is None or total is None or total <= 0:
            return None

        key = str(path)
        previous = self._previous_devfreq_times.get(key)
        self._previous_devfreq_times[key] = (busy, total)
        if previous is not None:
            previous_busy, previous_total = previous
            delta_busy = busy - previous_busy
            delta_total = total - previous_total
            if delta_total > 0:
                return max(0.0, min(100.0, delta_busy / delta_total * 100.0))
        return max(0.0, min(100.0, busy / total * 100.0))

    @staticmethod
    def _normalize_percent(value: float) -> float:
        if value > 100.0 and value <= 1000.0:
            value /= 10.0
        elif value > 1000.0:
            value /= 1000.0
        return max(0.0, min(100.0, value))

    @staticmethod
    def _freq_ratio_percent(cur_freq: int | None, max_freq: int | None) -> float | None:
        if cur_freq is None or max_freq is None or max_freq <= 0:
            return None
        return max(0.0, min(100.0, cur_freq / max_freq * 100.0))

    @staticmethod
    def _read_linux_memory() -> tuple[int | None, int | None, float | None]:
        try:
            values: dict[str, int] = {}
            for line in PROC_MEMINFO_PATH.read_text(encoding="utf-8", errors="ignore").splitlines():
                parts = line.split()
                if len(parts) < 2:
                    continue
                name = parts[0].rstrip(":")
                try:
                    values[name] = int(parts[1]) * 1024
                except ValueError:
                    continue
        except OSError:
            return None, None, None

        total = values.get("MemTotal")
        available = values.get("MemAvailable")
        if available is None:
            free = values.get("MemFree", 0)
            buffers = values.get("Buffers", 0)
            cached = values.get("Cached", 0)
            available = free + buffers + cached
        if total is None or total <= 0:
            return total, None, None

        used = max(0, min(total, total - available))
        return total, used, used / total * 100.0

    @staticmethod
    def _read_linux_cpu_times() -> tuple[tuple[int, int], ...] | None:
        try:
            lines = PROC_STAT_PATH.read_text(encoding="utf-8", errors="ignore").splitlines()
        except OSError:
            return None

        cpu_times: list[tuple[int, int]] = []
        for line in lines:
            parts = line.split()
            if not parts:
                continue
            name = parts[0]
            if not name.startswith("cpu") or not name[3:].isdigit():
                continue
            try:
                fields = [int(value) for value in parts[1:]]
            except ValueError:
                continue
            if len(fields) < 4:
                continue
            idle = fields[3] + (fields[4] if len(fields) > 4 else 0)
            total = sum(fields)
            cpu_times.append((total, idle))
        return tuple(cpu_times)

    def _linux_cpu_percents(self, cpu_times: tuple[tuple[int, int], ...]) -> tuple[float | None, ...]:
        previous = self._previous_linux_cpu_times
        if previous is None or len(previous) != len(cpu_times):
            return tuple(None for _ in cpu_times)

        percents: list[float | None] = []
        for (total, idle), (previous_total, previous_idle) in zip(cpu_times, previous, strict=True):
            delta_total = total - previous_total
            delta_idle = idle - previous_idle
            if delta_total <= 0:
                percents.append(None)
                continue
            busy = max(0, min(delta_total, delta_total - delta_idle))
            percents.append(busy / delta_total * 100.0)
        return tuple(percents)
