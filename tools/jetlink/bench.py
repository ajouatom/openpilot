#!/usr/bin/env python3
"""Parked/synthetic USB round-trip benchmark; never starts vehicle controls."""
import argparse
import json
from pathlib import Path
import time

import numpy as np

from openpilot.selfdrive.modeld.jetlink.link import Client, SPEC


def main():
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument('--frames', type=int, default=1200)
  p.add_argument('--rate', type=float, default=20)
  p.add_argument('--output', type=Path, required=True)
  args = p.parse_args()
  client = Client(timeout=3)
  packed = np.zeros(SPEC.packed_nelem, np.float32)
  warped = np.arange(SPEC.warped_nbytes, dtype=np.uint8).reshape(SPEC.warped_shape)
  durations, server = [], []
  start = time.monotonic()
  try:
    for i in range(args.frames):
      t0 = time.monotonic()
      result = client.infer(warped, packed, i, reset=(i == 0))
      durations.append((time.monotonic() - t0) * 1000)
      server.append(client.timings)
      packed[-SPEC.feat_dim:] = result[SPEC.output_slices['hidden_state']]
      time.sleep(max(0, t0 + 1 / args.rate - time.monotonic()))
  finally:
    client.close()
  a = np.array(durations)
  record = dict(model=SPEC.sha256, frames=len(a), seconds=time.monotonic()-start,
                scope='C4 local IPC + USB round trip + Jetson inference; synthetic prewarped input; no camera warp',
                mean_ms=float(a.mean()), p95_ms=float(np.percentile(a, 95)), p99_ms=float(np.percentile(a, 99)),
                max_ms=float(a.max()), over_50ms=int((a > 50).sum()), server_mean_us=np.mean(server, axis=0).tolist())
  args.output.write_text(json.dumps(record, indent=2))
  np.savez_compressed(args.output.with_suffix('.npz'), durations=durations, server=server)
  print(json.dumps(record))


if __name__ == '__main__':
  main()
