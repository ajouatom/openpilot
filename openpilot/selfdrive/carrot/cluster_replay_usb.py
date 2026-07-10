#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path


CLUSTER_DIR = Path(__file__).resolve().parent / "cluster"
REPO_ROOT = Path(__file__).resolve().parents[3]
REQUIREMENTS_PATH = CLUSTER_DIR / "requirements.txt"
INSTALL_HELP_MODULES = {
    "PIL",
    "capnp",
    "pyray",
    "pygame",
    "usb",
    "zstandard",
}


def print_dependency_help(exc: ModuleNotFoundError) -> None:
    module = exc.name or "unknown"
    print(f"Missing Python module for cluster replay: {module}", file=sys.stderr)
    if module in INSTALL_HELP_MODULES:
        print("", file=sys.stderr)
        print("Install the cluster replay dependencies with:", file=sys.stderr)
        print(f"  {sys.executable} -m pip install -r {REQUIREMENTS_PATH}", file=sys.stderr)
        if module == "pyray":
            print("", file=sys.stderr)
            print("Note: pyray is provided by the 'raylib' package in requirements.txt.", file=sys.stderr)


def build_cluster_args(args: argparse.Namespace, passthrough: list[str]) -> list[str]:
    cluster_args = [
        "--input", "route",
        "--route", str(args.route),
        "--route-log", args.route_log,
        "--route-overlay", args.route_overlay,
        "--output", args.output,
        "--usb-codec", args.usb_codec,
        "--fps", str(args.fps),
        "--route-replay-speed", str(args.speed),
    ]
    if args.duration is not None:
        cluster_args.extend(("--duration", str(args.duration)))
    if args.start_segment is not None:
        cluster_args.extend(("--route-start-segment", str(args.start_segment)))
    if args.max_segments is not None:
        cluster_args.extend(("--route-max-segments", str(args.max_segments)))
    if args.loop:
        cluster_args.append("--route-loop")
    if args.usb_brightness is not None:
        cluster_args.extend(("--usb-brightness", str(args.usb_brightness)))
    if args.profile_render:
        cluster_args.append("--profile-render")
    if args.profile_interval is not None:
        cluster_args.extend(("--profile-interval", str(args.profile_interval)))
    return [*cluster_args, *passthrough]


def parse_args(argv: list[str]) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description="Replay an rlog/route through the carrot cluster renderer and send it to the USB cluster display.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("route", type=Path, help="rlog.zst file, segment directory, or route directory")
    parser.add_argument("--route-log", choices=("rlog", "qlog"), default="rlog", help="Log type to replay")
    parser.add_argument("--output", choices=("usb", "window", "both"), default="both", help="Render target")
    parser.add_argument("--usb-codec", choices=("jpeg", "png", "h264"), default="jpeg", help="USB transport codec")
    parser.add_argument("--fps", type=float, default=20.0, help="Replay/render FPS")
    parser.add_argument("--duration", type=float, default=None, help="Seconds to replay; omit for route end")
    parser.add_argument("--speed", type=float, default=1.0, help="Replay speed multiplier")
    parser.add_argument("--start-segment", type=int, default=None, help="First segment index when a route directory is given")
    parser.add_argument("--max-segments", type=int, default=None, help="Maximum number of route segments to replay")
    parser.add_argument("--loop", action="store_true", help="Loop the replay")
    parser.add_argument("--route-overlay", choices=("off", "compact", "full"), default="compact", help="Replay camera/data overlay on the PC window")
    parser.add_argument("--usb-brightness", type=int, default=None, help="Manual USB display brightness 0-100")
    parser.add_argument("--profile-render", action="store_true", help="Print render/USB timing profile")
    parser.add_argument("--profile-interval", type=float, default=None, help="Seconds between profile reports")
    return parser.parse_known_args(argv)


def main() -> None:
    args, passthrough = parse_args(sys.argv[1:])
    sys.argv = [sys.argv[0], *build_cluster_args(args, passthrough)]

    repo_root_text = str(REPO_ROOT)
    if repo_root_text not in sys.path:
        sys.path.insert(0, repo_root_text)

    try:
        from cluster_run import main as cluster_run_main
    except ModuleNotFoundError as exc:
        print_dependency_help(exc)
        raise SystemExit(2) from exc

    cluster_run_main()


if __name__ == "__main__":
    main()
