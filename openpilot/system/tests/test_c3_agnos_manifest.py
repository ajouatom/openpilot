import json
from pathlib import Path

from openpilot.common.basedir import BASEDIR


TICI_DIR = Path(BASEDIR) / "openpilot/system/hardware/tici"


def _load_manifest(name: str) -> list[dict]:
  return json.loads((TICI_DIR / name).read_text(encoding="utf-8"))


def test_device_manifests_use_carrot_agnos_19_8_bluetooth_images() -> None:
  c3 = {partition["name"]: partition for partition in _load_manifest("agnos-tici.json")}
  c4 = {partition["name"]: partition for partition in _load_manifest("agnos.json")}

  assert c3["boot"]["hash_raw"] == "9d1c81ef890edf349e0919260a850ab5d1f95162fbd4b262cfcd52d92c0ac0b8"
  assert c4["boot"]["hash_raw"] == "dccd7965346b0a87a9f64cb6be257f6bb5d3d0f368c8655085efc1e460527f5a"
  assert c3["boot"] != c4["boot"]
  assert c3["system"] == c4["system"]
  assert c3["system"]["hash_raw"] == "375c5d22335770ac08750660bb5b29a3331c550d6a9875b35f36b88af792ea44"
  assert "agnos-19.8-carrot-bt1" in c3["boot"]["url"]
  assert "agnos-19.8-carrot-bt1" in c4["boot"]["url"]
  assert "agnos-19.8-carrot-bt1" in c3["system"]["url"]
  assert "alt" not in c3["system"]


def test_c3_manifest_preserves_proven_legacy_firmware() -> None:
  c3 = {partition["name"]: partition for partition in _load_manifest("agnos-tici.json")}
  expected_hashes = {
    "xbl": "6710967ca9701f205d7ab19c3a9b0dd2f547e65b3d96048b7c2b03755aafa0f1",
    "xbl_config": "63922cfbfdf4ab87986c4ba8f3a4df5bf28414b3f71a29ec5947336722215535",
    "abl": "32a2174b5f764e95dfc54cf358ba01752943b1b3b90e626149c3da7d5f1830b6",
    "aop": "21370172e590bd4ea907a558bcd6df20dc7a6c7d38b8e62fdde18f4a512ba9e9",
    "devcfg": "d7d7e52963bbedbbf8a7e66847579ca106a0a729ce2cf60f4b8d8ea4b535d620",
  }

  assert {name: c3[name]["hash_raw"] for name in expected_hashes} == expected_hashes


def test_c3_and_clone_select_the_c3_manifest() -> None:
  launcher = (Path(BASEDIR) / "launch_chffrplus.sh").read_text(encoding="utf-8")
  updater = (Path(BASEDIR) / "openpilot/system/updated/updated.py").read_text(encoding="utf-8")

  assert 'if [ "$MODEL" = "c3" ] || [ "$MODEL" = "tici" ]; then' in launcher
  assert 'if model in ("c3", "tici"):' in updater
  assert "agnos-tici.json" in launcher
  assert "agnos-tici.json" in updater


def test_launch_requires_carrot_agnos_19_8_bluetooth() -> None:
  launch_env = (Path(BASEDIR) / "launch_env.sh").read_text(encoding="utf-8")
  assert 'export AGNOS_VERSION="19.8-carrot-bt1"' in launch_env
