from __future__ import annotations

import sys
from pathlib import Path


CARROT_DIR = Path(__file__).resolve().parents[1]
if str(CARROT_DIR) not in sys.path:
  sys.path.insert(0, str(CARROT_DIR))

from cluster_replay_usb import build_cluster_args, parse_args


def test_corner_source_is_forwarded_to_cluster_replay():
  args, passthrough = parse_args([
    "route",
    "--corner-source", "stable",
    "--duration", "5",
    "--", "--profile-interval", "1",
  ])

  cluster_args = build_cluster_args(args, passthrough)

  assert cluster_args[cluster_args.index("--route-corner-source") + 1] == "stable"
  assert cluster_args[-3:] == ["--", "--profile-interval", "1"]


def test_trip_report_shortcut_selects_screen_mode_five():
  args, passthrough = parse_args(["route", "--trip-report"])

  cluster_args = build_cluster_args(args, passthrough)

  assert cluster_args[cluster_args.index("--screen-mode") + 1] == "trip-report"
  assert cluster_args[cluster_args.index("--camera-view-mode") + 1] == "2"
