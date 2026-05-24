from __future__ import annotations

import gc
import time
from typing import Any


class ProfileReporter:
    def __init__(self, enabled: bool, interval_s: float) -> None:
        self.enabled = enabled
        self.interval_s = max(0.2, interval_s)
        self.samples: dict[str, list[float]] = {}
        self.last_report_time = time.perf_counter()
        self.report_frames = 0

    def add(self, name: str, milliseconds: float) -> None:
        if not self.enabled:
            return
        self.samples.setdefault(name, []).append(milliseconds)

    def add_elapsed(self, name: str, start_time: float) -> None:
        if self.enabled:
            self.add(name, (time.perf_counter() - start_time) * 1000.0)

    def add_samples(self, samples: tuple[tuple[str, float], ...]) -> None:
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
        ordered = sorted(
            self.samples.items(),
            key=lambda item: sum(item[1]) / max(1, len(item[1])),
            reverse=True,
        )
        for name, values in ordered:
            if not values:
                continue
            average = sum(values) / len(values)
            print(
                f"  {name:<42} avg={average:7.2f}ms "
                f"max={max(values):7.2f}ms last={values[-1]:7.2f}ms n={len(values)}",
                flush=True,
            )
        self.samples.clear()
        self.report_frames = 0
        self.last_report_time = now


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
