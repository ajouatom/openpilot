#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path


CLUSTER_DIR = Path(__file__).resolve().parent / "cluster"
REPO_ROOT = Path(__file__).resolve().parents[3]
REQUIREMENTS_PATH = CLUSTER_DIR / "requirements.txt"
INSTALL_HELP_MODULES = {
    "PIL",
    "aiohttp",
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
        "--route-corner-source", args.corner_source,
        "--route-overlay", args.route_overlay,
        "--route-tools", args.route_tools,
        "--camera-view-mode", str(args.camera_view_mode),
        "--panel-layout", args.panel_layout,
        "--output", args.output,
        "--usb-codec", args.usb_codec,
        "--fps", str(args.fps),
        "--route-replay-speed", str(args.speed),
        "--language", args.language,
        "--metric" if args.is_metric else "--imperial",
    ]
    if args.duration is not None:
        cluster_args.extend(("--duration", str(args.duration)))
    if args.start_time > 0.0:
        cluster_args.extend(("--route-start-time", str(args.start_time)))
    if args.start_segment is not None:
        cluster_args.extend(("--route-start-segment", str(args.start_segment)))
    if args.max_segments is not None:
        cluster_args.extend(("--route-max-segments", str(args.max_segments)))
    if args.loop:
        cluster_args.append("--route-loop")
    if args.pause_on_cutin:
        cluster_args.append("--route-pause-on-cutin")
    if args.usb_brightness is not None:
        cluster_args.extend(("--usb-brightness", str(args.usb_brightness)))
    if args.profile_render:
        cluster_args.append("--profile-render")
    if args.profile_interval is not None:
        cluster_args.extend(("--profile-interval", str(args.profile_interval)))
    if args.navi_overlay:
        cluster_args.extend((
            "--navi-overlay",
            "--navi-host", args.navi_host,
            "--navi-port", str(args.navi_port),
        ))
        if args.navi_advertise_ip is not None:
            cluster_args.extend(("--navi-advertise-ip", args.navi_advertise_ip))
        cluster_args.extend(("--navi-map-theme", args.navi_map_theme))
        if args.navi_no_beacon:
            cluster_args.append("--navi-no-beacon")
    screen_mode = "trip-report" if args.trip_report else args.screen_mode or ("navi" if args.navi_overlay else None)
    if screen_mode is not None:
        cluster_args.extend(("--screen-mode", screen_mode))
    return [*cluster_args, *passthrough]


def parse_args(argv: list[str]) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description="Replay an rlog/route through the carrot cluster renderer and send it to the USB cluster display.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("route", type=Path, help="rlog.zst file, segment directory, or route directory")
    parser.add_argument("--route-log", choices=("rlog", "qlog"), default="rlog", help="Log type to replay")
    parser.add_argument(
        "--corner-source",
        choices=("live", "stable", "raw"),
        default="live",
        help=" ".join((
            "Corner-radar source: live shows device-published liveTracks,",
            "stable reconstructs physical tracks from raw CAN, and raw shows untracked CAN slots",
        )),
    )
    parser.add_argument("--output", choices=("usb", "window", "both"), default="both", help="Render target")
    parser.add_argument("--usb-codec", choices=("jpeg", "png", "h264"), default="jpeg", help="USB transport codec")
    parser.add_argument("--fps", type=float, default=20.0, help="Replay/render FPS")
    parser.add_argument("--duration", type=float, default=None, help="Seconds to replay; omit for route end")
    parser.add_argument("--start-time", type=float, default=0.0, help="Initial playback position in seconds")
    parser.add_argument("--speed", type=float, default=1.0, help="Replay speed multiplier")
    parser.add_argument("--start-segment", type=int, default=None, help="First segment index when a route directory is given")
    parser.add_argument("--max-segments", type=int, default=None, help="Maximum number of route segments to replay")
    parser.add_argument("--loop", action="store_true", help="Loop the replay")
    parser.add_argument(
        "--pause-on-cutin",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Play a Windows alert and pause when radarState.leadsCutIn becomes active",
    )
    parser.add_argument("--route-overlay", choices=("off", "compact", "full"), default="compact", help="Replay camera/data detail level")
    parser.add_argument(
        "--route-tools",
        choices=("separate", "overlay", "off"),
        default="separate",
        help="Place replay debug and seek controls in a separate PC window by default",
    )
    parser.add_argument(
        "--show-recorded-cutins",
        action="store_true",
        help="Compatibility option; recorded radarState cut-ins are always shown",
    )
    parser.add_argument("--front-radar-only", action="store_true", help="Ignore corner radar data and replay as a front-radar-only vehicle")
    parser.add_argument(
        "--cutin-radar-source",
        choices=("corner", "front"),
        default="corner",
        help="Compatibility option; route playback always uses logged radarState cut-ins",
    )
    parser.add_argument(
        "--cutin-sensitivity",
        type=float,
        default=50.0,
        help="Compatibility option; route playback does not recompute cut-ins",
    )
    parser.add_argument("--camera-view-mode", type=int, choices=(0, 1, 2), default=2, help="Cluster camera view mode (default: 2, road camera background)")
    parser.add_argument(
        "--panel-layout",
        choices=("driving-left", "driving-right"),
        default="driving-left",
        help="Place the driving view on the left or right side",
    )
    parser.add_argument(
        "--language",
        choices=("ko", "en"),
        default="ko",
        help="Cluster labels language",
    )
    unit_group = parser.add_mutually_exclusive_group()
    unit_group.add_argument(
        "--metric",
        dest="is_metric",
        action="store_true",
        help="Show km/h, m, and km",
    )
    unit_group.add_argument(
        "--imperial",
        dest="is_metric",
        action="store_false",
        help="Show mph, ft, and mi",
    )
    parser.set_defaults(is_metric=True)
    parser.add_argument("--usb-brightness", type=int, default=None, help="Manual USB display brightness 0-100")
    parser.add_argument("--profile-render", action="store_true", help="Print render/USB timing profile")
    parser.add_argument("--profile-interval", type=float, default=None, help="Seconds between profile reports")
    parser.add_argument(
        "--navi-overlay",
        action="store_true",
        help="Receive live Carrot navigation and merge it with replayed vehicle data",
    )
    parser.add_argument("--navi-host", default="0.0.0.0", help="Carrot navigation receiver bind host")
    parser.add_argument("--navi-port", type=int, default=7714, help="Carrot navigation receiver TCP port")
    parser.add_argument("--navi-advertise-ip", default=None, help="Address advertised to the Android navigation app")
    parser.add_argument("--navi-map-theme", choices=("dark", "auto", "light"), default="dark", help="Theme requested for the smartphone-rendered map")
    parser.add_argument("--navi-no-beacon", action="store_true", help="Disable UDP 7705 navigation discovery")
    screen_group = parser.add_mutually_exclusive_group()
    screen_group.add_argument(
        "--screen-mode",
        default=None,
        help="Cluster screen mode; navi is used by default with --navi-overlay",
    )
    screen_group.add_argument(
        "--trip-report",
        action="store_true",
        help="Show external HUD screen mode 5 with the live driving report",
    )
    return parser.parse_known_args(argv)


def main() -> None:
    args, passthrough = parse_args(sys.argv[1:])
    os.environ["CLUSTER_ROUTE_SHOW_RECORDED_CUTINS"] = "1"
    if args.front_radar_only:
        os.environ["CLUSTER_ROUTE_FRONT_RADAR_ONLY"] = "1"
    else:
        os.environ.pop("CLUSTER_ROUTE_FRONT_RADAR_ONLY", None)
    os.environ["CLUSTER_ROUTE_CUTIN_RADAR_SOURCE"] = args.cutin_radar_source
    os.environ["CLUSTER_ROUTE_CUTIN_SENSITIVITY"] = str(max(0.0, min(100.0, args.cutin_sensitivity)))
    sys.argv = [sys.argv[0], *build_cluster_args(args, passthrough)]

    repo_root_text = str(REPO_ROOT)
    if repo_root_text not in sys.path:
        sys.path.insert(0, repo_root_text)

    try:
        from cluster_run import main as cluster_run_main
    except ModuleNotFoundError as exc:
        print_dependency_help(exc)
        raise SystemExit(2) from exc

    try:
        cluster_run_main()
    except ModuleNotFoundError as exc:
        print_dependency_help(exc)
        raise SystemExit(2) from exc


if __name__ == "__main__":
    main()
