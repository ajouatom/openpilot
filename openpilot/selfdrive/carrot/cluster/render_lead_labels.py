#!/usr/bin/env python3
"""Render synthetic lead-label review frames with the production raylib renderer.

Run from the repository root with PYTHONPATH=. and an available Linux GL display.
The camera cases use a plain synthetic image, never a live vehicle connection.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import json
from pathlib import Path

from PIL import Image, ImageDraw

from cluster_config import CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT
from cluster_models import DetectedVehicle, RouteOverlay, SimulatorInput
from cluster_renderer import ClusterUiRenderer
from cluster_simulator import ClusterSimulator


class ReviewRenderer(ClusterUiRenderer):
    def __init__(self):
        super().__init__(language="en", theme_mode="dark")
        self.metric_draws = []

    def _draw_world_label_text(self, text, x, y, size, color, anchor="left"):
        width, height = self._measure_text(text, size, max(1.0, size * 0.02), self._font_for_text(text))
        left = x - width * (0.5 if anchor == "center" else 1.0 if anchor == "right" else 0.0)
        self.metric_draws.append({"text": text, "size": size, "bounds": [left, y - height * 0.5, width, height]})
        super()._draw_world_label_text(text, x, y, size, color, anchor)


def review_state(distance: float, camera_mode: int = 0, *, lead_present: bool = True, stopped: bool = False):
    base = ClusterSimulator().update(SimulatorInput(), 0.05)
    vehicles = (
        DetectedVehicle("L1", distance, 0.0, source="radarState", primary=True, absolute_speed_kph=0.0 if stopped else 90.0),
        DetectedVehicle("L2", distance + 30.0, 0.1, source="radarState", primary=True, absolute_speed_kph=72.0),
        DetectedVehicle("RF", 45.0, 3.6, source="carState", absolute_speed_kph=60.0),
        DetectedVehicle("CUT-IN", 65.0, -3.0, source="radarState", primary=True, cut_in=True, absolute_speed_kph=80.0),
    )
    overlay = None
    if camera_mode in (2, 3):
        camera = Image.new("RGBA", (1928, 1208), (60, 64, 68, 255))
        overlay = RouteOverlay(
            video_rgba=camera.tobytes(), video_width=camera.width, video_height=camera.height,
            video_frame_id=f"synthetic-{camera_mode}", camera_stream="wide" if camera_mode == 3 else "road",
            panel_visible=False,
        )
    return replace(
        base, speed_kph=100.0, cruise_kph=100, speed_limit_kph=100,
        camera_view_mode=camera_mode, camera_device_type="tici", camera_sensor="ar0231",
        detected_vehicles=vehicles if lead_present else vehicles[1:], route_overlay=overlay,
    )


def render_review(output: Path) -> None:
    output.mkdir(parents=True, exist_ok=True)
    renderer = ReviewRenderer()
    report = []

    def capture(name, state):
        # Warm the renderer's normal caches, then capture the actual production draw.
        renderer.render_to_png_bytes(state)
        renderer.metric_draws.clear()
        path = output / f"{name}.png"
        path.write_bytes(renderer.render_to_png_bytes(state))
        report.append({"image": path.name, "metrics": list(renderer.metric_draws)})
        return path

    try:
        renderer.open(hidden=True)
        for theme in ("dark", "light"):
            renderer.set_theme_mode(theme)
            for mode, view in enumerate(("drive", "ego-bottom", "road-camera", "wide-camera")):
                sheet = Image.new("RGB", (renderer.width, 4 * (renderer.height + 28)), "#202020")
                draw = ImageDraw.Draw(sheet)
                for row, distance in enumerate((20.0, 80.0, 150.0, 220.0)):
                    name = f"{theme}-{view}-{int(distance):03d}m"
                    path = capture(name, review_state(distance, mode))
                    top = row * (renderer.height + 28)
                    draw.text((12, top + 6), f"SYNTHETIC / {theme} / {view} / L1 {distance:.0f} m", fill="white")
                    with Image.open(path) as frame:
                        sheet.paste(frame, (0, top + 28))
                sheet.save(output / f"{theme}-{view}-comparison.png")

        renderer.set_theme_mode("dark")
        capture("no-lead-one", review_state(150.0, lead_present=False))
        capture("stopped-lead-one", review_state(150.0, stopped=True))
        renderer.set_display_preferences("en", False)
        capture("imperial", review_state(150.0))
        renderer.set_panel_layout(CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT)
        capture("imperial-driving-right", review_state(150.0))
    finally:
        renderer.close()
    (output / "metrics.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Rendered {len(report)} synthetic frames and 8 comparison sheets to {output}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("build/cluster_lead_labels"))
    args = parser.parse_args()
    render_review(args.output)


if __name__ == "__main__":
    main()
