from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import time


PROC_PATH = Path("/proc")
PROC_STAT_PATH = Path("/proc/stat")
PROC_MEMINFO_PATH = Path("/proc/meminfo")


@dataclass(frozen=True, slots=True)
class SystemStats:
    memory_total_bytes: int | None = None
    memory_used_bytes: int | None = None
    memory_used_percent: float | None = None
    cpu_core_percents: tuple[float | None, ...] = ()


class SystemStatsSampler:
    def __init__(self, refresh_interval_s: float = 1.0) -> None:
        self.refresh_interval_s = max(0.1, float(refresh_interval_s))
        self._next_sample_time = 0.0
        self._stats = SystemStats()
        self._previous_linux_cpu_times: tuple[tuple[int, int], ...] | None = None

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
            cpu_core_percents=cpu_percents,
        )

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


@dataclass(frozen=True, slots=True)
class _ThreadCpuSample:
    key: tuple[int, int, int]
    cpu_time_ticks: int
    processor: int


class ClusterProcessCoreUsageSampler:
    def __init__(self, refresh_interval_s: float = 1.0) -> None:
        self.refresh_interval_s = max(0.1, float(refresh_interval_s))
        self._next_sample_time = 0.0
        self._last_sample_time: float | None = None
        self._previous_thread_ticks: dict[tuple[int, int, int], int] | None = None
        self._text: str | None = None
        try:
            self._clock_ticks = int(os.sysconf(os.sysconf_names["SC_CLK_TCK"]))
        except (AttributeError, KeyError, OSError, ValueError):
            self._clock_ticks = 100

    def sample_text(self, now: float | None = None) -> str | None:
        if now is None:
            now = time.perf_counter()
        if now < self._next_sample_time:
            return self._text

        samples = self._read_cluster_thread_samples()
        next_ticks = {sample.key: sample.cpu_time_ticks for sample in samples}
        text: str | None = None
        if self._previous_thread_ticks is not None and self._last_sample_time is not None:
            elapsed = max(0.001, now - self._last_sample_time)
            by_core: dict[int, float] = {}
            for sample in samples:
                previous_ticks = self._previous_thread_ticks.get(sample.key)
                if previous_ticks is None or sample.processor < 0:
                    continue
                delta_ticks = max(0, sample.cpu_time_ticks - previous_ticks)
                if delta_ticks == 0:
                    continue
                percent = delta_ticks / max(1, self._clock_ticks) / elapsed * 100.0
                by_core[sample.processor] = by_core.get(sample.processor, 0.0) + percent
            parts = [
                f"{core}({max(1, int(round(percent)))})"
                for core, percent in sorted(by_core.items())
                if percent >= 0.5
            ]
            text = "[" + ",".join(parts) + "]"

        self._previous_thread_ticks = next_ticks
        self._last_sample_time = now
        self._next_sample_time = now + self.refresh_interval_s
        self._text = text
        return self._text

    def _read_cluster_thread_samples(self) -> tuple[_ThreadCpuSample, ...]:
        if not PROC_PATH.exists():
            return ()

        current_pid = os.getpid()
        samples: list[_ThreadCpuSample] = []
        try:
            pid_dirs = tuple(PROC_PATH.iterdir())
        except OSError:
            return ()
        for pid_dir in pid_dirs:
            if not pid_dir.name.isdigit():
                continue
            pid = int(pid_dir.name)
            if pid != current_pid and not self._is_cluster_process(pid_dir):
                continue
            task_dir = pid_dir / "task"
            try:
                thread_dirs = tuple(task_dir.iterdir())
            except OSError:
                continue
            for thread_dir in thread_dirs:
                if not thread_dir.name.isdigit():
                    continue
                sample = self._read_thread_stat(pid, int(thread_dir.name), thread_dir / "stat")
                if sample is not None:
                    samples.append(sample)
        return tuple(samples)

    @staticmethod
    def _is_cluster_process(pid_dir: Path) -> bool:
        text_parts: list[str] = []
        try:
            text_parts.append((pid_dir / "comm").read_text(encoding="utf-8", errors="ignore"))
        except OSError:
            pass
        try:
            cmdline = (pid_dir / "cmdline").read_bytes().replace(b"\0", b" ")
            text_parts.append(cmdline.decode("utf-8", errors="ignore"))
        except OSError:
            pass
        return "cluster" in " ".join(text_parts).lower()

    @staticmethod
    def _read_thread_stat(pid: int, tid: int, stat_path: Path) -> _ThreadCpuSample | None:
        try:
            text = stat_path.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            return None
        close = text.rfind(")")
        if close < 0 or close + 2 >= len(text):
            return None
        fields = text[close + 2 :].split()
        if len(fields) <= 36:
            return None
        try:
            utime = int(fields[11])
            stime = int(fields[12])
            starttime = int(fields[19])
            processor = int(fields[36])
        except ValueError:
            return None
        return _ThreadCpuSample(
            key=(pid, tid, starttime),
            cpu_time_ticks=utime + stime,
            processor=processor,
        )
