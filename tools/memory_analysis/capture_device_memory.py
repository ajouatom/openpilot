#!/usr/bin/env python3
"""Bounded Linux memory observation; never changes service or control settings.

Run on the device, e.g. python3 capture_device_memory.py --seconds 7200
--output /data/carrot/diagnostics/memory.jsonl. Output is private diagnostic data.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time


def read(path):
  try:
    return Path(path).read_text(errors="replace")
  except OSError:
    return ""


def memory_fields(text):
  result = {}
  for line in text.splitlines():
    key, _, value = line.partition(":")
    fields = value.split()
    if fields and fields[0].isdigit():
      result[key] = result.get(key, 0) + int(fields[0])
  return result


def debug_command(command):
  try:
    result = subprocess.run(["sudo", "-n", *command], capture_output=True, text=True, timeout=5)
    return result.stdout if result.returncode == 0 else ""
  except (OSError, subprocess.TimeoutExpired):
    return ""


def debug_read(path):
  return debug_command(["cat", path])


def gpu_inventory(text):
  result = {}
  for line in text.splitlines()[1:]:
    fields = line.split()
    if len(fields) < 9 or not fields[2].isdigit():
      continue
    entry = result.setdefault(fields[5], {"count": 0, "bytes": 0, "mapped_bytes": 0})
    entry["count"] += 1
    entry["bytes"] += int(fields[2])
    entry["mapped_bytes"] += int(fields[8])
  return result


def detail():
  processes = []
  for path in Path("/proc").iterdir():
    if not path.name.isdigit():
      continue
    status = memory_fields(read(path / "status"))
    if status.get("VmRSS", 0) < 2000:
      continue
    # Keep the executable/process title only, never arbitrary command arguments.
    cmd = read(path / "cmdline")
    name = cmd.split("\0", 1)[0]
    role = next((key for key in ("cluster_autorun", "precompiled_worker", "signal_shadow", "signal_color_shadow", "carrot_server", "carrot_navi") if key in cmd), name)
    process = {"pid": int(path.name), "name": name, "role": role, "status_kib": {key: value for key, value in status.items() if key.startswith("Vm")}, "threads": status.get("Threads")}
    process["start_ticks"] = read(path / "stat").rpartition(")")[2].split()[19:20]
    smaps = memory_fields(read(path / "smaps"))
    process["memory_kib"] = {key: smaps.get(key) for key in ("Pss", "Rss", "Private_Dirty", "Private_Clean", "Anonymous", "Swap")}
    try:
      process["fds"] = len(list((path / "fd").iterdir()))
    except OSError:
      pass
    processes.append(process)
  gpu = {}
  for pid in debug_command(["ls", "/sys/kernel/debug/kgsl/proc"]).split():
    if pid.isdigit():
      gpu[pid] = gpu_inventory(debug_read("/sys/kernel/debug/kgsl/proc/" + pid + "/mem"))
  ion = debug_read("/sys/kernel/debug/ion/heaps/system")
  totals = {key: int(value) for key, value in re.findall(r"^\s*(total orphaned|total|deferred free)\s+(\d+)\s*$", ion, re.M)}
  return {"processes": processes, "gpu": gpu, "ion_bytes": totals}


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--output", type=Path, required=True)
  parser.add_argument("--seconds", type=int, default=7200)
  args = parser.parse_args()
  if not 60 <= args.seconds <= 7200:
    parser.error("seconds must be between 60 and 7200")
  os.nice(15)
  try:
    os.sched_setaffinity(0, set(os.sched_getaffinity(0)) & {0, 1, 2, 3})
  except (AttributeError, OSError):
    pass
  args.output.parent.mkdir(parents=True, exist_ok=True)
  # One observer per directory; no overlap or overwriting previous evidence.
  with (args.output.parent / ".memory-observer.lock").open("a") as lock:
    fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    fd = os.open(args.output, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    stopped = False

    def stop(_signum, _frame):
      nonlocal stopped
      stopped = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    start = time.monotonic()
    last_detail = float("-inf")
    written = 0
    with os.fdopen(fd, "w") as output:
      while not stopped and time.monotonic() - start <= args.seconds:
        now = time.monotonic()
        mem = memory_fields(read("/proc/meminfo"))
        row = {"mono": now, "wall": time.time(), "boot": read("/proc/sys/kernel/random/boot_id").strip(), "mem_kib": mem}
        row["flags"] = {key: read(Path("/data/params/d") / key).strip() for key in ("IsOnroad", "ClusterHud", "ClusterHudConnected", "UsbGpuActive")}
        interval = 15 if mem.get("MemAvailable", 0) < mem.get("MemTotal", 0) * .09 else 60
        if now - last_detail >= interval:
          row["detail"] = detail()
          row["detail_seconds"] = time.monotonic() - now
          last_detail = now
        line = json.dumps(row) + "\n"
        if written + len(line.encode()) > 32 * 1024 * 1024:
          break
        output.write(line)
        output.flush()
        written += len(line.encode())
        # Stop observation itself if headroom becomes critically small.
        if mem.get("MemAvailable", 0) < 128 * 1024:
          break
        time.sleep(max(0, 5 - (time.monotonic() - now)))


if __name__ == "__main__":
  main()
