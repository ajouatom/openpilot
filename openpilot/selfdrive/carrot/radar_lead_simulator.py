#!/usr/bin/env python3
"""Compatibility launcher for physical dPath predictor replay."""

from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.selfdrive.carrot.radar.tools.radar_lead_simulator import main


if __name__ == "__main__":
  raise SystemExit(main())
