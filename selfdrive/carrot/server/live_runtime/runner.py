from __future__ import annotations

import argparse
import json
import sys
import time


def _summary(snapshot: dict, age_ms: int | None) -> dict:
  runtime = snapshot.get("runtime", {})
  services = snapshot.get("services", {})
  road_camera = services.get("roadCameraState") or {}
  wide_camera = services.get("wideRoadCameraState") or {}
  model = services.get("modelV2") or {}
  car_state = services.get("carState") or {}
  selfdrive_state = services.get("selfdriveState") or {}
  controls_state = services.get("controlsState") or {}
  carrot = services.get("carrotMan") or {}

  return {
    "ts": int(time.time() * 1000),
    "snapshotFresh": runtime.get("snapshotFresh"),
    "ageMs": age_ms,
    "roadReady": road_camera.get("frameId") is not None,
    "wideReady": wide_camera.get("frameId") is not None,
    "roadFrameId": road_camera.get("frameId"),
    "modelReady": model.get("frameId") is not None,
    "modelFrameId": model.get("frameId"),
    "vehicleReady": car_state.get("vEgoCluster") is not None or car_state.get("vEgo") is not None,
    "vEgo": car_state.get("vEgoCluster") if car_state.get("vEgoCluster") is not None else car_state.get("vEgo"),
    "enabled": bool(selfdrive_state.get("enabled") or controls_state.get("enabled")),
    "carrotReady": carrot is not None and carrot != {},
    "activeCoreServices": runtime.get("activeCoreServices"),
    "activeOptionalServices": runtime.get("activeOptionalServices"),
    "missingCoreServices": runtime.get("missingCoreServices", []),
    "missingOptionalServices": runtime.get("missingOptionalServices", []),
  }


def main() -> int:
  parser = argparse.ArgumentParser(description="RealtimeBroker standalone runner")
  parser.add_argument("--repo-flavor", default="c3")
  parser.add_argument("--interval-ms", type=int, default=200)
  parser.add_argument("--iterations", type=int, default=0, help="0 means forever")
  args = parser.parse_args()

  try:
    from .broker import RealtimeBroker
    broker = RealtimeBroker(repo_flavor=args.repo_flavor)
  except RuntimeError as exc:
    print(str(exc), file=sys.stderr)
    print("", file=sys.stderr)
    print("Suggested commands:", file=sys.stderr)
    print("  cd /data/openpilot", file=sys.stderr)
    print("  rm -f prebuilt", file=sys.stderr)
    print("  python system/manager/build.py", file=sys.stderr)
    return 1

  count = 0

  while True:
    snapshot = broker.poll(0)
    print(json.dumps(_summary(snapshot, broker.snapshot_age_ms()), ensure_ascii=False), flush=True)
    count += 1
    if args.iterations > 0 and count >= args.iterations:
      return 0
    time.sleep(max(args.interval_ms, 20) / 1000.0)


if __name__ == "__main__":
  raise SystemExit(main())
