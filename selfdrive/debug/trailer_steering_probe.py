#!/usr/bin/env python3
"""
Capture openpilot lateral-control state and selected Hyundai CAN-FD messages.

Run from the openpilot checkout on the comma device:

  python3 selfdrive/debug/trailer_steering_probe.py --label no_trailer --duration 30
  python3 selfdrive/debug/trailer_steering_probe.py --label trailer_idle --duration 30
  python3 selfdrive/debug/trailer_steering_probe.py --label trailer_drive --duration 60

The script writes:
  <label>_<timestamp>.jsonl        raw CAN/sendcan frames and state snapshots
  <label>_<timestamp>_summary.json address counts, rates, and payload examples
"""

import argparse
import binascii
import json
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path

OPENPILOT_ROOT = Path(__file__).resolve().parents[2]
if str(OPENPILOT_ROOT) not in sys.path:
  sys.path.insert(0, str(OPENPILOT_ROOT))


DEFAULT_ADDRESSES = {
  0x50,   # LKAS
  0xCB,   # LFA_ALT
  0xEA,   # MDPS
  0x110,  # LKAS_ALT
  0x12A,  # LFA
  0x161,  # ADRV cluster/alerts
  0x162,  # CCNC alerts/radar display
  0x1E0,  # LFAHDA_CLUSTER
  0x1EA,  # ADRV lane/radar display
  0x200,  # ADRV status
  0x437,  # trailer connection status
}


def safe_get(obj, name, default=None):
  try:
    return getattr(obj, name)
  except Exception:
    return default


def as_number(value, default=0):
  try:
    return float(value)
  except Exception:
    return default


def car_state_snapshot(sm):
  cc = sm["carControl"]
  cs = sm["carState"]
  co = sm["carOutput"]
  sd = sm["selfdriveState"]

  cc_actuators = safe_get(cc, "actuators")
  co_actuators = safe_get(co, "actuatorsOutput")

  return {
    "type": "state",
    "carControl": {
      "enabled": bool(safe_get(cc, "enabled", False)),
      "latActive": bool(safe_get(cc, "latActive", False)),
      "longActive": bool(safe_get(cc, "longActive", False)),
      "torque": as_number(safe_get(cc_actuators, "torque", 0)),
      "steeringAngleDeg": as_number(safe_get(cc_actuators, "steeringAngleDeg", 0)),
      "torqueOutputCan": as_number(safe_get(cc_actuators, "torqueOutputCan", 0)),
    },
    "carOutput": {
      "torque": as_number(safe_get(co_actuators, "torque", 0)),
      "steeringAngleDeg": as_number(safe_get(co_actuators, "steeringAngleDeg", 0)),
      "torqueOutputCan": as_number(safe_get(co_actuators, "torqueOutputCan", 0)),
    },
    "carState": {
      "vEgo": as_number(safe_get(cs, "vEgo", 0)),
      "latEnabled": bool(safe_get(cs, "latEnabled", False)),
      "trailerConnected": bool(safe_get(cs, "trailerConnected", False)),
      "steeringAngleDeg": as_number(safe_get(cs, "steeringAngleDeg", 0)),
      "steeringTorque": as_number(safe_get(cs, "steeringTorque", 0)),
      "steeringTorqueEps": as_number(safe_get(cs, "steeringTorqueEps", 0)),
      "steeringPressed": bool(safe_get(cs, "steeringPressed", False)),
      "steerFaultTemporary": bool(safe_get(cs, "steerFaultTemporary", False)),
      "steerFaultPermanent": bool(safe_get(cs, "steerFaultPermanent", False)),
      "canValid": bool(safe_get(cs, "canValid", False)),
    },
    "selfdriveState": {
      "enabled": bool(safe_get(sd, "enabled", False)),
      "active": bool(safe_get(sd, "active", False)),
      "state": str(safe_get(sd, "state", "")),
      "alertType": str(safe_get(sd, "alertType", "")),
      "alertText1": str(safe_get(sd, "alertText1", "")),
      "alertText2": str(safe_get(sd, "alertText2", "")),
    },
  }


def parse_addresses(values):
  if not values:
    return set(DEFAULT_ADDRESSES)
  return {int(value, 0) for value in values}


def parse_buses(values):
  if not values:
    return None
  return {int(value, 0) for value in values}


def capture(args):
  import cereal.messaging as messaging

  addresses = parse_addresses(args.address)
  buses = parse_buses(args.bus)

  output_dir = Path(args.output_dir)
  output_dir.mkdir(parents=True, exist_ok=True)
  timestamp = time.strftime("%Y%m%d_%H%M%S")
  stem = f"{args.label}_{timestamp}"
  log_path = output_dir / f"{stem}.jsonl"
  summary_path = output_dir / f"{stem}_summary.json"

  can_sock = messaging.sub_sock("can", addr=args.messaging_address, conflate=False)
  sendcan_sock = messaging.sub_sock("sendcan", addr=args.messaging_address, conflate=False)
  sm = messaging.SubMaster(
    ["carControl", "carState", "carOutput", "selfdriveState"],
    addr=args.messaging_address,
  )

  counts = Counter()
  payload_counts = defaultdict(Counter)
  first_payload = {}
  last_payload = {}
  first_seen = {}
  last_seen = {}
  state_count = 0
  start = time.monotonic()
  last_state = start - args.state_interval
  last_status = start

  metadata = {
    "type": "metadata",
    "label": args.label,
    "createdAt": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    "durationRequested": args.duration,
    "addresses": [f"0x{x:X}" for x in sorted(addresses)],
    "buses": sorted(buses) if buses is not None else "all",
  }

  with log_path.open("w", encoding="utf-8") as log_file:
    log_file.write(json.dumps(metadata, sort_keys=True) + "\n")

    try:
      while True:
        now = time.monotonic()
        elapsed = now - start
        if args.duration > 0 and elapsed >= args.duration:
          break

        sm.update(0)

        for service, sock in (("can", can_sock), ("sendcan", sendcan_sock)):
          for event in messaging.drain_sock(sock):
            frames = safe_get(event, service, [])
            for frame in frames:
              address = int(frame.address)
              bus = int(frame.src)
              if not args.all_addresses and address not in addresses:
                continue
              if buses is not None and bus not in buses:
                continue

              payload = binascii.hexlify(frame.dat).decode("ascii")
              key = (service, bus, address)
              counts[key] += 1
              payload_counts[key][payload] += 1
              first_payload.setdefault(key, payload)
              last_payload[key] = payload
              first_seen.setdefault(key, elapsed)
              last_seen[key] = elapsed

              row = {
                "type": "can",
                "t": round(elapsed, 6),
                "service": service,
                "bus": bus,
                "address": address,
                "addressHex": f"0x{address:X}",
                "data": payload,
              }
              log_file.write(json.dumps(row, sort_keys=True) + "\n")

        if now - last_state >= args.state_interval:
          row = car_state_snapshot(sm)
          row["t"] = round(elapsed, 6)
          log_file.write(json.dumps(row, sort_keys=True) + "\n")
          state_count += 1
          last_state = now

        if now - last_status >= 1.0:
          print(f"{args.label}: {elapsed:5.1f}s, frames={sum(counts.values())}, states={state_count}")
          log_file.flush()
          last_status = now

        time.sleep(0.005)
    except KeyboardInterrupt:
      print("capture stopped by user")

  duration = time.monotonic() - start
  messages = []
  for service, bus, address in sorted(counts):
    key = (service, bus, address)
    examples = [
      {"data": payload, "count": count}
      for payload, count in payload_counts[key].most_common(args.max_examples)
    ]
    messages.append({
      "service": service,
      "bus": bus,
      "address": address,
      "addressHex": f"0x{address:X}",
      "count": counts[key],
      "hz": counts[key] / duration if duration > 0 else 0,
      "first": first_payload[key],
      "last": last_payload[key],
      "firstSeen": first_seen[key],
      "lastSeen": last_seen[key],
      "uniquePayloads": len(payload_counts[key]),
      "examples": examples,
    })

  summary = {
    "label": args.label,
    "duration": duration,
    "log": str(log_path),
    "stateSnapshots": state_count,
    "messages": messages,
  }
  summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")

  print(f"wrote {log_path}")
  print(f"wrote {summary_path}")


def main():
  parser = argparse.ArgumentParser(
    description="Capture Hyundai trailer-mode steering diagnostics",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  )
  parser.add_argument("--label", required=True, help="capture label, e.g. no_trailer or trailer_drive")
  parser.add_argument("--duration", type=float, default=30.0, help="capture duration in seconds; 0 runs until Ctrl-C")
  parser.add_argument("--output-dir", default="/data/trailer_steering_logs")
  parser.add_argument("--messaging-address", default="127.0.0.1")
  parser.add_argument("--address", action="append", help="CAN address to capture; repeatable, accepts hex")
  parser.add_argument("--bus", action="append", help="CAN source bus to capture; repeatable")
  parser.add_argument("--all-addresses", action="store_true", help="record every CAN address")
  parser.add_argument("--state-interval", type=float, default=0.1, help="seconds between control-state snapshots")
  parser.add_argument("--max-examples", type=int, default=20, help="payload examples per address in summary")
  capture(parser.parse_args())


if __name__ == "__main__":
  main()
