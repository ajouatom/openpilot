#!/usr/bin/env python3
"""Pre-render P/R screens and compare unchanged driving frames on Linux.

Uses synthetic state, repository fonts and the production raylib renderer.
No vehicle connection, weather request or Windows graphics backend is used.
"""
from __future__ import annotations

import argparse
from contextlib import ExitStack
from dataclasses import replace
import io
import json
from pathlib import Path
import subprocess
import sys
import time
from types import ModuleType, SimpleNamespace
from unittest.mock import patch

import numpy as np
from PIL import Image, ImageDraw

import cluster_renderer
from cluster_models import ClusterAlert, RadarPoint, TpmsInfo, TripReportState
from cluster_system_monitor import SystemStats, SystemStatsSampler
from render_lead_labels import review_state
from openpilot.common import indicator_display
from openpilot.selfdrive.carrot import signal_mode


def baseline_renderer(ref: str) -> ModuleType:
    """Load the two changed rendering modules from a local Git revision.

    Keep __file__ at the real package path so both renderers use identical
    repository fonts and the unchanged upstream Cybertruck asset.
    """
    root = Path(__file__).resolve().parents[4]
    modules = {}
    with patch.dict(sys.modules):
        for name in ("cluster_scene", "cluster_renderer"):
            path = Path(__file__).with_name(f"{name}.py").resolve()
            source = subprocess.check_output(["git", "show", f"{ref}:{path.relative_to(root).as_posix()}"], cwd=root)
            module = ModuleType(name)
            module.__file__ = str(path)
            sys.modules[name] = module
            exec(compile(source, str(path), "exec"), module.__dict__)
            modules[name] = module
    return modules["cluster_renderer"]


def synthetic_state(gear: str, camera_mode: int = 0):
    state = review_state(150.0, camera_mode)
    return replace(
        state, onroad=True, gear_text=gear, car_fingerprint="KIA_EV6",
        speed_kph=0.0 if gear == "P" else 4.0 if gear == "R" else 100.0,
        center_clock_text="12:34", steering_angle_deg=0.0, lfa_active=True,
        fuel_gauge=0.76, energy_gauge_label="battery", tpms=TpmsInfo(36, 37, 36, 37),
        trip_report=TripReportState(duration_s=2400, distance_m=42000, average_speed_kph=63.0, auto_ratio_percent=85),
        radar_points=(RadarPoint("ONCOMING", 45.0, -3.6, "liveTracks", absolute_speed_kph=-30.0),),
    )


def render_review(output: Path, baseline_ref: str | None) -> None:
    if sys.platform != "linux":
        raise RuntimeError("Run this review in the repository's Linux Python environment")
    output.mkdir(parents=True, exist_ok=True)
    baseline = baseline_renderer(baseline_ref) if baseline_ref else None
    fixed_time = time.struct_time((2026, 11, 5, 12, 34, 0, 3, 309, -1))
    clock = SimpleNamespace(monotonic=lambda: 10000.0, perf_counter=time.perf_counter,
                            localtime=lambda *args: fixed_time, strftime=time.strftime)
    comparisons = []
    previews = []

    def capture(renderer, name, state):
        renderer.render_to_png_bytes(state)  # Warm the normal font/texture caches.
        data = renderer.render_to_png_bytes(state)
        (output / f"{name}.png").write_bytes(data)
        with Image.open(io.BytesIO(data)) as frame:
            return np.array(frame.convert("RGBA"))

    def compare(name, current, reference):
        changed = int(np.count_nonzero(np.any(current != reference, axis=2)))
        comparisons.append({"case": name, "changed_pixels": changed})

    with ExitStack() as stack:
        stack.enter_context(patch.object(SystemStatsSampler, "sample", lambda *_: SystemStats()))
        stack.enter_context(patch.object(indicator_display, "current", lambda: None))
        stack.enter_context(patch.object(signal_mode, "signal_mode", lambda: 0))
        stack.enter_context(patch.object(cluster_renderer.EV6_WEATHER, "snapshot", lambda: None))
        stack.enter_context(patch.object(cluster_renderer, "time", clock))
        # Preview selection must not depend on opt-out files on the review host.
        stack.enter_context(patch.object(cluster_renderer, "EV6_PARK_SCREEN_OFF_FLAG", str(output / "absent-park-off")))
        stack.enter_context(patch.object(cluster_renderer, "EV6_REVERSE_SCREEN_OFF_FLAG", str(output / "absent-reverse-off")))
        if baseline:
            stack.enter_context(patch.object(baseline, "time", clock))

        for theme in ("dark", "light"):
            driving = [("D-3d", synthetic_state("D")), ("D-camera", synthetic_state("D", 2)),
                       ("N-3d", synthetic_state("N"))]
            references = {}
            if baseline:
                renderer = baseline.ClusterUiRenderer(language="ko", theme_mode=theme)
                try:
                    renderer.open(hidden=True)
                    for name, state in driving:
                        references[name] = capture(renderer, f"baseline-{theme}-{name}", state)
                finally:
                    renderer.close()

            renderer = cluster_renderer.ClusterUiRenderer(language="ko", theme_mode=theme)
            try:
                renderer.open(hidden=True)
                for name, state in driving:
                    frame = capture(renderer, f"{theme}-{name}-before", state)
                    if baseline:
                        compare(f"{theme}-{name}-before", frame, references[name])
                assert renderer._park_vehicle_model is None, "D/N loaded P/R resources"
                park = synthetic_state("P")
                reverse = synthetic_state("R")
                for name, state in (
                    ("P", park), ("R-straight", reverse),
                    ("R-left", replace(reverse, steering_angle_deg=160.0, left_blindspot=True)),
                    ("R-right", replace(reverse, steering_angle_deg=-160.0, right_blindspot=True)),
                    ("P-ioniq5", replace(park, car_fingerprint="HYUNDAI_IONIQ_5")),
                    ("R-alert", replace(reverse, alert=ClusterAlert("주변을 확인하세요", "합성 경고", size=2, status=1))),
                ):
                    capture(renderer, f"{theme}-{name}", state)
                    previews.append(f"{theme}-{name}")
                assert renderer._park_vehicle_model is not None, "EV6 model failed to load"
                assert renderer._park_model_texture is not None, "P texture failed to render"
                assert renderer._ev6_reverse_sprite, "R sprite failed to render"
                for name, state in driving:
                    frame = capture(renderer, f"{theme}-{name}-after", state)
                    if baseline:
                        compare(f"{theme}-{name}-after", frame, references[name])
            finally:
                renderer.close()
            # Exercise reopening the same object after deleting all GL handles.
            try:
                renderer.open(hidden=True)
                capture(renderer, f"{theme}-P-reopened", synthetic_state("P"))
                assert renderer._park_model_texture is not None
            finally:
                renderer.close()

    sheet = Image.new("RGB", (1920, 4 * 508), "#202020")
    labels = ("dark-P", "dark-R-left", "light-P", "light-R-right")
    draw = ImageDraw.Draw(sheet)
    for row, name in enumerate(labels):
        top = row * 508
        draw.text((12, top + 6), f"SYNTHETIC / Linux / {name}", fill="white")
        with Image.open(output / f"{name}.png") as frame:
            sheet.paste(frame, (0, top + 28))
    sheet.save(output / "gear-screens.png")
    report = {"platform": sys.platform, "raylib_gl_version": int(cluster_renderer.rl.rl_get_version()),
              "baseline": baseline_ref, "previews": previews, "comparisons": comparisons}
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    if any(case["changed_pixels"] for case in comparisons):
        raise AssertionError(f"Driving render differs from baseline; inspect {output / 'report.json'}")
    print(f"Rendered {len(previews)} P/R previews; {len(comparisons)} driving comparisons match exactly: {output}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("build/cluster_gear_screens"))
    parser.add_argument("--baseline-ref", help="local Git ref used for unchanged-driving pixel comparison")
    args = parser.parse_args()
    render_review(args.output, args.baseline_ref)
