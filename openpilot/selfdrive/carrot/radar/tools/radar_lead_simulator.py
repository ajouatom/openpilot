#!/usr/bin/env python3
"""Compatibility entry point for physical dPath predictor review."""

from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[5]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.selfdrive.carrot.radar.tools.radar_validation_replay import *  # noqa: F403
from openpilot.selfdrive.carrot.radar.tools.radar_validation_replay import main


if __name__ == "__main__":
  raise SystemExit(main())
