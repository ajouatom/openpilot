#!/usr/bin/env python3
"""Generate AR projection vectors from openpilot's canonical Python transforms.

This intentionally imports camera.py/orientation.py instead of duplicating the
math in JavaScript. Run from the repository root and copy stdout into
tests/fixtures/ar_projection_golden.json when the canonical contract changes.
"""

import json
from pathlib import Path
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(REPO_ROOT))

from openpilot.common.transformations.camera import DEVICE_CAMERAS, get_view_frame_from_road_frame


SAMPLES = (
  ("ar0231-flat", "ar0231", (0.0, 0.0, 0.0)),
  ("ox03c10-calibrated", "ox03c10", (0.018, -0.041, 0.027)),
  ("os04c10-calibrated", "os04c10", (-0.012, -0.035, -0.022)),
)

ROUTE_FLU_POINTS = (
  (20.0, 0.0, 0.0),
  (35.0, 2.5, 0.0),
  (50.0, -3.0, 1.5),
  (90.0, 5.0, -2.0),
)

FRD_FROM_FLU = np.diag([1.0, -1.0, -1.0])


def clean(value):
  if isinstance(value, np.ndarray):
    return clean(value.tolist())
  if isinstance(value, (list, tuple)):
    return [clean(item) for item in value]
  number = float(value)
  if abs(number) < 5e-13:
    return 0.0
  return round(number, 12)


def main():
  profiles = {}
  cases = []
  for case_id, sensor, rpy in SAMPLES:
    camera = DEVICE_CAMERAS[("tici", sensor)].fcam
    profiles[sensor] = {
      "width": camera.width,
      "height": camera.height,
      "focalLength": camera.focal_length,
    }
    camera_from_route_flu = camera.intrinsics @ get_view_frame_from_road_frame(*rpy, 0.0)[:, :3]
    camera_from_device_frd = camera_from_route_flu @ FRD_FROM_FLU
    points = []
    for point in ROUTE_FLU_POINTS:
      camera_point = camera_from_route_flu @ np.asarray(point)
      points.append({
        "routeFlu": clean(point),
        "pixel": clean((camera_point[0] / camera_point[2], camera_point[1] / camera_point[2])),
        "depth": clean(camera_point[2]),
      })
    cases.append({
      "id": case_id,
      "sensor": sensor,
      "rpyCalib": clean(rpy),
      "cameraFromDeviceFrd": clean(camera_from_device_frd),
      "cameraFromRouteFlu": clean(camera_from_route_flu),
      "points": points,
    })

  payload = {
    "schemaVersion": 1,
    "generator": "tests/generate_ar_projection_golden.py",
    "pythonSources": [
      "openpilot/common/transformations/camera.py",
      "openpilot/common/transformations/orientation.py",
    ],
    "coordinateContract": {
      "device": "FRD",
      "route": "FLU",
      "view": "RDF",
      "frdFromFlu": clean(FRD_FROM_FLU),
    },
    "profiles": profiles,
    "cases": cases,
  }
  print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
  main()
