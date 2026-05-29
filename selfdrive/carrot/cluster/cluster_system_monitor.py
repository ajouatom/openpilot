from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re
import subprocess
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
DEVFREQ_LOAD_PATHS = (("load",), ("device", "load"))
DEVFREQ_BUSY_PATHS = (("busy_time",), ("busy",), ("device", "busy_time"))
DEVFREQ_TOTAL_PATHS = (("total_time",), ("total",), ("device", "total_time"))
DEVFREQ_CUR_FREQ_PATHS = (("cur_freq",), ("current_freq",), ("target_freq",), ("freq",))
DEVFREQ_MAX_FREQ_PATHS = (("max_freq",), ("max_frequency",), ("peak_freq",))
DEVFREQ_AVAILABLE_FREQ_PATHS = (("available_frequencies",), ("available_freqs",), ("freq_table",))
DEVFREQ_TIME_IN_STATE_PATHS = (("time_in_state",), ("stats", "time_in_state"))
MSM_VIDC_DEBUGFS_PATH = Path("/sys/kernel/debug/msm_vidc")
SUDO_READ_TIMEOUT_S = 0.5


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
    encoder_frame_rate_fps: float | None = None
    encoder_target_fps: float | None = None
    encoder_source: str | None = None
    cpu_core_percents: tuple[float | None, ...] = ()


class SystemStatsSampler:
    def __init__(self, refresh_interval_s: float = 1.0) -> None:
        self.refresh_interval_s = max(0.1, float(refresh_interval_s))
        self._next_sample_time = 0.0
        self._stats = SystemStats()
        self._previous_linux_cpu_times: tuple[tuple[int, int], ...] | None = None
        self._previous_devfreq_times: dict[str, tuple[int, int]] = {}
        self._previous_devfreq_time_in_state: dict[str, dict[int, int]] = {}
        self._previous_vidc_debugfs_counts: dict[str, tuple[int, float]] = {}
        self._vidc_debugfs_info_paths: tuple[Path, ...] | None = None
        self._reported_encoder_unavailable = False

    def sample(self, now: float | None = None) -> SystemStats:
        if now is None:
            now = time.perf_counter()
        if now < self._next_sample_time:
            return self._stats

        stats = self._sample_linux(now)
        if stats is not None:
            self._stats = stats
        self._next_sample_time = now + self.refresh_interval_s
        return self._stats

    def _sample_linux(self, now: float) -> SystemStats | None:
        if not PROC_STAT_PATH.exists() and not PROC_MEMINFO_PATH.exists():
            return None

        memory_total, memory_used, memory_percent = self._read_linux_memory()
        gpu_percent = self._read_gpu_percent()
        gpu_freq = self._read_first_int(KGSL_GPU_FREQ_PATHS)
        gpu_max_freq = self._read_first_int(KGSL_GPU_MAX_FREQ_PATHS)
        (
            encoder_percent,
            encoder_freq,
            encoder_max_freq,
            encoder_rate_fps,
            encoder_target_fps,
            encoder_source,
        ) = self._read_encoder_stats(now)
        if encoder_percent is None and encoder_source is None and not self._reported_encoder_unavailable:
            debug_text = self.encoder_devfreq_debug_text()
            if debug_text:
                print(f"System stats VENC devfreq unavailable: {debug_text}", flush=True)
                self._reported_encoder_unavailable = True
        elif encoder_percent is not None or encoder_source is not None:
            self._reported_encoder_unavailable = False
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
            encoder_frame_rate_fps=encoder_rate_fps,
            encoder_target_fps=encoder_target_fps,
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

    def _read_encoder_stats(
        self,
        now: float,
    ) -> tuple[float | None, int | None, int | None, float | None, float | None, str | None]:
        debugfs_percent, debugfs_rate, debugfs_target, debugfs_source = self._read_encoder_debugfs(now)
        if debugfs_source is not None:
            return debugfs_percent, None, None, debugfs_rate, debugfs_target, debugfs_source

        devfreq_percent, devfreq_freq, devfreq_max_freq, devfreq_source = self._read_encoder_devfreq()
        return devfreq_percent, devfreq_freq, devfreq_max_freq, None, None, devfreq_source

    def _read_encoder_debugfs(self, now: float) -> tuple[float | None, float | None, float | None, str | None]:
        fallback: tuple[float | None, float | None, float | None, str | None] = (None, None, None, None)
        valid_paths = False
        for path in self._vidc_debugfs_instance_paths():
            text = self._read_text_file(path, allow_sudo=True)
            if not text:
                continue
            valid_paths = True
            parsed = self._parse_vidc_instance_info(text)
            if parsed is None:
                continue
            target_fps, completed_count = parsed
            rate_fps = self._vidc_completed_rate(path, completed_count, now)
            used_percent = self._freq_ratio_percent_float(rate_fps, target_fps)
            source = "msm_vidc"
            if used_percent is not None:
                return used_percent, rate_fps, target_fps, source
            if fallback[3] is None:
                fallback = (None, rate_fps, target_fps, source)
        if not valid_paths:
            self._vidc_debugfs_info_paths = None
        return fallback

    def _vidc_completed_rate(self, path: Path, completed_count: int | None, now: float) -> float | None:
        if completed_count is None:
            return None
        key = str(path)
        previous = self._previous_vidc_debugfs_counts.get(key)
        self._previous_vidc_debugfs_counts[key] = (completed_count, now)
        if previous is None:
            return None
        previous_count, previous_time = previous
        elapsed = now - previous_time
        if elapsed <= 0.05 or completed_count < previous_count:
            return None
        return (completed_count - previous_count) / elapsed

    @classmethod
    def _parse_vidc_instance_info(cls, text: str) -> tuple[float | None, int | None] | None:
        if "(Encoder)" not in text:
            return None
        target_fps = cls._first_regex_float(text, r"(?m)^\s*fps:\s*([0-9]+(?:\.[0-9]+)?)\s*$")
        completed_count = cls._first_regex_int(text, r"(?m)^\s*FBD Count:\s*([0-9]+)\s*$")
        if completed_count is None:
            completed_count = cls._first_regex_int(text, r"(?m)^\s*EBD Count:\s*([0-9]+)\s*$")
        if target_fps is None and completed_count is None:
            return None
        return target_fps, completed_count

    @staticmethod
    def _first_regex_int(text: str, pattern: str) -> int | None:
        match = re.search(pattern, text)
        if match is None:
            return None
        try:
            return int(match.group(1))
        except ValueError:
            return None

    @staticmethod
    def _first_regex_float(text: str, pattern: str) -> float | None:
        match = re.search(pattern, text)
        if match is None:
            return None
        try:
            return float(match.group(1))
        except ValueError:
            return None

    def _vidc_debugfs_instance_paths(self) -> tuple[Path, ...]:
        if self._vidc_debugfs_info_paths:
            return self._vidc_debugfs_info_paths

        paths: tuple[Path, ...] = ()
        try:
            paths = tuple(sorted(MSM_VIDC_DEBUGFS_PATH.glob("core*/inst_*/info")))
        except OSError:
            paths = ()

        if not paths:
            paths = self._sudo_find_vidc_debugfs_instance_paths()

        if paths:
            self._vidc_debugfs_info_paths = paths
        return paths

    @staticmethod
    def _sudo_find_vidc_debugfs_instance_paths() -> tuple[Path, ...]:
        try:
            output = subprocess.check_output(
                [
                    "sudo",
                    "-n",
                    "find",
                    str(MSM_VIDC_DEBUGFS_PATH),
                    "-maxdepth",
                    "3",
                    "-type",
                    "f",
                    "-path",
                    "*/inst_*/info",
                ],
                encoding="utf-8",
                errors="ignore",
                timeout=SUDO_READ_TIMEOUT_S,
                stderr=subprocess.DEVNULL,
            )
        except (OSError, subprocess.SubprocessError):
            return ()
        return tuple(Path(line.strip()) for line in output.splitlines() if line.strip())

    def _read_encoder_devfreq(self) -> tuple[float | None, int | None, int | None, str | None]:
        fallback: tuple[float | None, int | None, int | None, str | None] = (None, None, None, None)
        for path in self._encoder_devfreq_paths():
            used_percent = self._read_devfreq_percent(path)
            cur_freq = self._read_first_child_int(path, DEVFREQ_CUR_FREQ_PATHS)
            max_freq = self._read_first_child_int(path, DEVFREQ_MAX_FREQ_PATHS)
            if max_freq is None or max_freq <= 0:
                max_freq = self._read_available_max_freq(path)
            if used_percent is None:
                time_percent, time_max_freq = self._read_time_in_state_percent(path)
                used_percent = time_percent
                if (max_freq is None or max_freq <= 0) and time_max_freq is not None:
                    max_freq = time_max_freq
            if used_percent is None:
                used_percent = self._freq_ratio_percent(cur_freq, max_freq)
            if used_percent is not None:
                return used_percent, cur_freq, max_freq, path.name
            if fallback[3] is None and (cur_freq is not None or max_freq is not None):
                fallback = (None, cur_freq, max_freq, path.name)
        return fallback

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
        text = SystemStatsSampler._read_first_child_text(path, DEVFREQ_AVAILABLE_FREQ_PATHS)
        if not text:
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
        load = self._read_first_child_float(path, DEVFREQ_LOAD_PATHS)
        if load is not None:
            return self._normalize_percent(load)

        busy = self._read_first_child_int(path, DEVFREQ_BUSY_PATHS)
        total = self._read_first_child_int(path, DEVFREQ_TOTAL_PATHS)
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
        return SystemStatsSampler._freq_ratio_percent_float(float(cur_freq), float(max_freq))

    @staticmethod
    def _freq_ratio_percent_float(current: float | None, maximum: float | None) -> float | None:
        if current is None or maximum is None or maximum <= 0:
            return None
        return max(0.0, min(100.0, current / maximum * 100.0))

    def _read_time_in_state_percent(self, path: Path) -> tuple[float | None, int | None]:
        states = self._read_time_in_state(path)
        if not states:
            return None, None

        max_freq = max(states)
        if max_freq <= 0:
            return None, None

        key = str(path)
        previous = self._previous_devfreq_time_in_state.get(key)
        self._previous_devfreq_time_in_state[key] = states
        active_states = states
        if previous is not None:
            deltas = {freq: time_value - previous.get(freq, 0) for freq, time_value in states.items()}
            deltas = {freq: time_value for freq, time_value in deltas.items() if time_value > 0}
            if deltas:
                active_states = deltas

        total_time = sum(active_states.values())
        if total_time <= 0:
            return None, max_freq
        weighted_freq = sum(freq * time_value for freq, time_value in active_states.items())
        return max(0.0, min(100.0, weighted_freq / total_time / max_freq * 100.0)), max_freq

    @staticmethod
    def _read_time_in_state(path: Path) -> dict[int, int]:
        text = SystemStatsSampler._read_first_child_text(path, DEVFREQ_TIME_IN_STATE_PATHS)
        if not text:
            return {}
        states: dict[int, int] = {}
        for line in text.splitlines():
            values = re.findall(r"\d+", line)
            if len(values) < 2:
                continue
            try:
                freq = int(values[0])
                time_value = int(values[1])
            except ValueError:
                continue
            if freq > 0 and time_value >= 0:
                states[freq] = time_value
        return states

    @staticmethod
    def _read_first_child_text(path: Path, child_paths: tuple[tuple[str, ...], ...]) -> str | None:
        for child_path in child_paths:
            text = SystemStatsSampler._read_text_file(path.joinpath(*child_path))
            if text:
                return text
        return None

    @staticmethod
    def _read_text_file(path: Path, allow_sudo: bool = False) -> str | None:
        try:
            return path.read_text(encoding="utf-8", errors="ignore").strip()
        except OSError:
            if not allow_sudo:
                return None
        try:
            return subprocess.check_output(
                ["sudo", "-n", "cat", str(path)],
                encoding="utf-8",
                errors="ignore",
                timeout=SUDO_READ_TIMEOUT_S,
                stderr=subprocess.DEVNULL,
            ).strip()
        except (OSError, subprocess.SubprocessError):
            return None

    @staticmethod
    def _read_first_child_int(path: Path, child_paths: tuple[tuple[str, ...], ...]) -> int | None:
        for child_path in child_paths:
            value = SystemStatsSampler._read_int_file(path.joinpath(*child_path))
            if value is not None:
                return value
        return None

    @staticmethod
    def _read_first_child_float(path: Path, child_paths: tuple[tuple[str, ...], ...]) -> float | None:
        for child_path in child_paths:
            value = SystemStatsSampler._read_float_file(path.joinpath(*child_path))
            if value is not None:
                return value
        return None

    def encoder_devfreq_debug_text(self) -> str:
        parts: list[str] = []
        for path in self._encoder_devfreq_paths():
            cur_freq = self._read_first_child_int(path, DEVFREQ_CUR_FREQ_PATHS)
            max_freq = self._read_first_child_int(path, DEVFREQ_MAX_FREQ_PATHS)
            available_max = self._read_available_max_freq(path)
            load = self._read_first_child_float(path, DEVFREQ_LOAD_PATHS)
            busy = self._read_first_child_int(path, DEVFREQ_BUSY_PATHS)
            total = self._read_first_child_int(path, DEVFREQ_TOTAL_PATHS)
            time_in_state = self._read_time_in_state(path)
            fields = [
                f"cur={self._debug_value(cur_freq)}",
                f"max={self._debug_value(max_freq)}",
                f"avail_max={self._debug_value(available_max)}",
                f"load={self._debug_value(load)}",
                f"busy={self._debug_value(busy)}",
                f"total={self._debug_value(total)}",
                f"tis={'yes' if time_in_state else 'no'}",
            ]
            parts.append(f"{path.name}({','.join(fields)})")
        return "; ".join(parts)

    @staticmethod
    def _debug_value(value: int | float | None) -> str:
        if value is None:
            return "-"
        if isinstance(value, float):
            return f"{value:g}"
        return str(value)

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
