from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
import gc
import os
import time
from typing import Any


GC_POST_FREEZE_THRESHOLDS = (60000, 120, 120)


def _post_freeze_gc_thresholds() -> tuple[int, int, int]:
    text = os.environ.get("CLUSTER_GC_THRESHOLDS", "").strip()
    if not text:
        return GC_POST_FREEZE_THRESHOLDS
    try:
        values = tuple(int(part.strip()) for part in text.replace(":", ",").split(","))
    except ValueError:
        print(f"Warning: invalid CLUSTER_GC_THRESHOLDS={text!r}; using defaults", flush=True)
        return GC_POST_FREEZE_THRESHOLDS
    if len(values) != 3 or any(value <= 0 for value in values):
        print(f"Warning: invalid CLUSTER_GC_THRESHOLDS={text!r}; using defaults", flush=True)
        return GC_POST_FREEZE_THRESHOLDS
    return values


def _read_proc_status() -> dict[str, str]:
    wanted = {
        "VmRSS",
        "Threads",
        "voluntary_ctxt_switches",
        "nonvoluntary_ctxt_switches",
    }
    status: dict[str, str] = {}
    try:
        with open("/proc/self/status", encoding="utf-8") as handle:
            for line in handle:
                key, separator, value = line.partition(":")
                if separator and key in wanted:
                    status[key] = value.strip()
    except OSError:
        pass
    return status


def _read_system_cpu_stat() -> tuple[int, int] | None:
    try:
        with open("/proc/stat", encoding="utf-8") as handle:
            fields = handle.readline().split()
    except OSError:
        return None
    if len(fields) < 5 or fields[0] != "cpu":
        return None
    try:
        values = [int(value) for value in fields[1:]]
    except ValueError:
        return None
    total = sum(values)
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    return total, idle


def _status_kb_to_mb(value: str) -> float | None:
    fields = value.split()
    if not fields:
        return None
    try:
        return int(fields[0]) / 1024.0
    except ValueError:
        return None


@dataclass(slots=True)
class ProfileSampleStats:
    total_ms: float = 0.0
    max_ms: float = 0.0
    last_ms: float = 0.0
    count: int = 0

    @property
    def average_ms(self) -> float:
        return self.total_ms / max(1, self.count)

    def add(self, milliseconds: float) -> None:
        self.total_ms += milliseconds
        if self.count == 0 or milliseconds > self.max_ms:
            self.max_ms = milliseconds
        self.last_ms = milliseconds
        self.count += 1


class ProfileReporter:
    def __init__(self, enabled: bool, interval_s: float) -> None:
        self.enabled = enabled
        self.interval_s = max(0.2, interval_s)
        self.samples: dict[str, ProfileSampleStats] = {}
        self.last_report_time = time.perf_counter()
        self.last_process_time = time.process_time()
        self.last_system_cpu = _read_system_cpu_stat()
        self.report_frames = 0

    def add(self, name: str, milliseconds: float) -> None:
        if not self.enabled:
            return
        stats = self.samples.get(name)
        if stats is None:
            stats = ProfileSampleStats()
            self.samples[name] = stats
        stats.add(milliseconds)

    def add_elapsed(self, name: str, start_time: float) -> None:
        if self.enabled:
            self.add(name, (time.perf_counter() - start_time) * 1000.0)

    def add_samples(self, samples: Iterable[tuple[str, float]]) -> None:
        if not self.enabled:
            return
        for name, milliseconds in samples:
            self.add(name, milliseconds)

    def frame_done(self) -> None:
        if self.enabled:
            self.report_frames += 1

    def maybe_report(self, now: float) -> None:
        if not self.enabled or now - self.last_report_time < self.interval_s:
            return

        elapsed = max(0.001, now - self.last_report_time)
        print(f"PROFILE {self.report_frames} frames / {elapsed:.1f}s", flush=True)
        runtime_summary = self._runtime_summary(elapsed)
        if runtime_summary:
            print(f"  runtime {runtime_summary}", flush=True)
        ordered = sorted(
            self.samples.items(),
            key=lambda item: item[1].average_ms,
            reverse=True,
        )
        for name, stats in ordered:
            if stats.count <= 0:
                continue
            print(
                f"  {name:<42} avg={stats.average_ms:7.2f}ms "
                f"max={stats.max_ms:7.2f}ms last={stats.last_ms:7.2f}ms n={stats.count}",
                flush=True,
            )
        self.samples.clear()
        self.report_frames = 0
        self.last_report_time = now

    def _runtime_summary(self, elapsed: float) -> str:
        parts: list[str] = []

        process_time = time.process_time()
        process_cpu = max(0.0, process_time - self.last_process_time) / elapsed * 100.0
        self.last_process_time = process_time
        parts.append(f"proc_cpu={process_cpu:5.1f}% one_core")

        current_system_cpu = _read_system_cpu_stat()
        if self.last_system_cpu is not None and current_system_cpu is not None:
            last_total, last_idle = self.last_system_cpu
            total, idle = current_system_cpu
            total_delta = total - last_total
            idle_delta = idle - last_idle
            if total_delta > 0:
                system_cpu = max(0.0, min(100.0, (total_delta - idle_delta) / total_delta * 100.0))
                parts.append(f"sys_cpu={system_cpu:5.1f}% all_cores")
        self.last_system_cpu = current_system_cpu

        status = _read_proc_status()
        rss_mb = _status_kb_to_mb(status.get("VmRSS", ""))
        if rss_mb is not None:
            parts.append(f"rss={rss_mb:.1f}MB")
        threads = status.get("Threads")
        if threads is not None:
            parts.append(f"threads={threads}")
        voluntary = status.get("voluntary_ctxt_switches")
        nonvoluntary = status.get("nonvoluntary_ctxt_switches")
        if voluntary is not None and nonvoluntary is not None:
            parts.append(f"ctx={voluntary}/{nonvoluntary}")
        try:
            parts.append(f"load1={os.getloadavg()[0]:.2f}")
        except (AttributeError, OSError):
            pass

        return " ".join(parts)


class GcProfileHook:
    def __init__(self, profile: ProfileReporter) -> None:
        self.profile = profile
        self._starts: dict[int, float] = {}

    def __call__(self, phase: str, info: dict[str, Any]) -> None:
        generation = int(info.get("generation", -1))
        if phase == "start":
            self._starts[generation] = time.perf_counter()
            return
        if phase != "stop":
            return
        start_time = self._starts.pop(generation, None)
        if start_time is not None:
            self.profile.add(f"gc.gen{generation}", (time.perf_counter() - start_time) * 1000.0)


def freeze_gc_after_init(profile: ProfileReporter) -> None:
    freeze = getattr(gc, "freeze", None)
    if freeze is None:
        return

    profile_stage = time.perf_counter()
    gc.collect(2)
    profile.add_elapsed("gc.freeze_init.collect", profile_stage)

    profile_stage = time.perf_counter()
    freeze()
    profile.add_elapsed("gc.freeze_init.freeze", profile_stage)
    gc.set_threshold(*_post_freeze_gc_thresholds())
