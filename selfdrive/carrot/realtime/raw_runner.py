from __future__ import annotations

import argparse
import sys
import time

from .raw_services import DEFAULT_RAW_SERVICES, is_supported_raw_service


def main() -> int:
  parser = argparse.ArgumentParser(description="Raw capnp relay standalone reader")
  parser.add_argument("service", help="service name to subscribe to")
  parser.add_argument("--iterations", type=int, default=0, help="0 means forever")
  args = parser.parse_args()

  service = args.service.strip()
  if not is_supported_raw_service(service):
    print(f"unsupported raw service: {service}", file=sys.stderr)
    print("supported services:", ", ".join(DEFAULT_RAW_SERVICES), file=sys.stderr)
    return 2

  try:
    from cereal import messaging
  except Exception as exc:
    print(f"messaging import failed: {exc}", file=sys.stderr)
    return 1

  try:
    sock = messaging.sub_sock(service, conflate=True)
  except Exception as exc:
    print(f"failed to open socket for {service}: {exc}", file=sys.stderr)
    return 1

  count = 0
  while True:
    payload = sock.receive(non_blocking=False)
    if payload is None:
      continue
    print(f"{int(time.time() * 1000)} {service} {len(payload)}", flush=True)
    count += 1
    if args.iterations > 0 and count >= args.iterations:
      return 0


if __name__ == "__main__":
  raise SystemExit(main())
