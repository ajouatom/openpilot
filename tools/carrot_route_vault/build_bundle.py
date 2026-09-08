"""Build a DSM Docker context entirely from committed replay and viewer sources.

Usage: python tools/carrot_route_vault/build_bundle.py OUTPUT --ref HEAD
Uncommitted vehicle/controller edits are deliberately excluded.
"""
import argparse
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[2]
SOURCES = (
  "openpilot/selfdrive/carrot/radar_motion",
  "openpilot/selfdrive/carrot/radar/tools/radar_validation_replay.py",
  "openpilot/selfdrive/carrot/radar/tools/radar_web_export.py",
  "openpilot/selfdrive/carrot/cluster",
  "openpilot/selfdrive/controls/lib/cutin_alert.py",
  "openpilot/selfdrive/controls/lib/cutin_helpers.py",
  "openpilot/cereal",
  "opendbc_repo/opendbc/__init__.py",
  "opendbc_repo/opendbc/car",
  "opendbc_repo/opendbc/dbc",
)


def build(destination: Path, ref: str):
  destination.mkdir(parents=True, exist_ok=False)
  commit = subprocess.check_output(["git", "rev-parse", "--verify", ref + "^{commit}"], cwd=ROOT, text=True).strip()
  files = subprocess.check_output(["git", "ls-tree", "-r", "--name-only", commit, "--", *SOURCES], cwd=ROOT, text=True).splitlines()
  for name in files:
    if Path(name).suffix not in {".py", ".capnp", ".dbc"}:
      continue
    target = destination / name
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(subprocess.check_output(["git", "show", f"{commit}:{name}"], cwd=ROOT))
  for name in ("server.py", "viewer.py", "radar.py", "radar_view.js", "requirements.txt", "Dockerfile.vault", "deploy_probe.py"):
    source = f"tools/carrot_route_vault/{name}"
    (destination / name).write_bytes(subprocess.check_output(["git", "show", f"{commit}:{source}"], cwd=ROOT))
  (destination / "SOURCE_COMMIT").write_text(commit + "\n", encoding="utf-8")
  print(f"Built {destination} from {commit}")


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("destination", type=Path)
  parser.add_argument("--ref", default="HEAD")
  args = parser.parse_args()
  build(args.destination, args.ref)
