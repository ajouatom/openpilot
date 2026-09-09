"""Image-local deployment checks; private route configuration stays on the NAS."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import sys
import time
import urllib.error
import urllib.request


def check_payload(payload, config):
  if payload.get("schemaVersion") != 1 or not payload.get("frames"):
    raise ValueError("Empty or unsupported radar replay")
  for rule in config.get("forbidden_cutin", []):
    frames = [f for f in payload["frames"] if f.get("video_time_s") is not None
              and rule["start"] < f["video_time_s"] < rule["end"]]
    if not frames:
      raise ValueError("Regression window has no video-aligned frames")
    if any(d["track_id"] == rule["track_id"] and d["stage"] == "CUT-IN"
           for f in frames for d in f["selection"]["cutin_diagnostics"]):
      raise ValueError("Forbidden cut-in regression")
  data = {k: payload[k] for k in ("frames", "graphs", "sourceVersion")}
  return {"frames": len(payload["frames"]), "sourceVersion": payload["sourceVersion"],
          "replayHash": hashlib.sha256(json.dumps(data, sort_keys=True, separators=(",", ":")).encode()).hexdigest()}


def fetch(url):
  with urllib.request.urlopen(url, timeout=30) as response:
    body = response.read()
    if response.headers.get("Content-Encoding") == "gzip":
      body = gzip.decompress(body)
    return response.status, body


def online(config, commit, expected):
  origin = config["base_url"].rstrip("/")
  deadline = time.monotonic() + 360
  while True:
    try:
      status, body = fetch(origin + "/api/v1/health")
      health = json.loads(body)
      if status == 200 and health.get("ok") and health.get("sourceCommit") == commit:
        break
    except (OSError, ValueError):
      pass
    if time.monotonic() >= deadline:
      raise RuntimeError("Health/source commit check timed out")
    time.sleep(2)
  status, page = fetch(origin + config["route_path"])
  if status != 200 or b"attachRadarReview(video)" not in page:
    raise ValueError("Upload-result page is missing its radar viewer")
  while True:
    status, body = fetch(origin + config["radar_path"])
    if status == 200:
      result = check_payload(json.loads(body), config)
      if result != expected:
        raise ValueError("Served radar data does not match fresh image replay")
      break
    if status != 202 or time.monotonic() >= deadline:
      raise RuntimeError("Radar recalculation failed or timed out")
    time.sleep(3)
  for path in config.get("extra_paths", []):
    if fetch(origin + path)[0] != 200:
      raise ValueError("Additional endpoint check failed")
  return result


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--online", action="store_true")
  args = parser.parse_args()
  request = json.load(sys.stdin)
  config = request["config"]
  commit = Path(__file__).with_name("SOURCE_COMMIT").read_text().strip()
  if args.online:
    result = online(config, commit, request["expected"])
  else:
    from openpilot.selfdrive.carrot.radar.tools import radar_web_export as exporter
    result = check_payload(exporter.export_frames(exporter.replay.load_frames(Path(config["log_path"]))), config)
  print(json.dumps({"sourceCommit": commit, "replay": result}))


if __name__ == "__main__":
  main()
