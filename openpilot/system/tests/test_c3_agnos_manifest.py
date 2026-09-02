import json
from pathlib import Path

from openpilot.common.basedir import BASEDIR


TICI_DIR = Path(BASEDIR) / "openpilot/system/hardware/tici"


def _load_manifest(name: str) -> list[dict]:
  return json.loads((TICI_DIR / name).read_text(encoding="utf-8"))


def test_c4_manifest_uses_production_agnos_19_6_images() -> None:
  c3 = {partition["name"]: partition for partition in _load_manifest("agnos-tici.json")}
  c4 = {partition["name"]: partition for partition in _load_manifest("agnos.json")}

  # C3 still needs its device-specific carrot kernel. Never use that
  # experimental rollout as the default for a clean C3X/C4 install.
  assert c3["boot"]["hash_raw"] == "7f6a45827e82f0d7ee2bab286daf60fec79c40fd4af2aee967b146e06b5ece70"
  assert c4["boot"]["hash_raw"] == "b30f5eef65ec3878f3aa3dcaf2cc95c09e2c1e661cd3a38e94da37dee76f68bd"
  assert c3["boot"] != c4["boot"]
  assert c3["system"]["hash_raw"] == "b97e492686fa3902b0385f13fa5663c911f0b150d06b722111d386f4ad42a7d7"
  assert c4["system"]["hash_raw"] == "5b6ce7965904a157fd3a134ccfcb854f9ca5c1cc2a26b7cb80a4fa4e1cc4aaa3"
  assert "agnos-19.6.3-carrot" in c3["boot"]["url"]
  assert "agnos-19.6.3-carrot" in c3["system"]["url"]
  assert c4["boot"]["url"].startswith("https://commadist.azureedge.net/agnosupdate/")
  assert c4["system"]["url"].startswith("https://commadist.azureedge.net/agnosupdate/")
  assert "alt" in c4["system"]


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


def test_launch_accepts_production_and_already_deployed_carrot_agnos() -> None:
  launch_env = (Path(BASEDIR) / "launch_env.sh").read_text(encoding="utf-8")
  launcher = (Path(BASEDIR) / "launch_chffrplus.sh").read_text(encoding="utf-8")
  updated = (Path(BASEDIR) / "openpilot/system/updated/updated.py").read_text(encoding="utf-8")

  assert 'export AGNOS_VERSION="19.6"' in launch_env
  assert 'export AGNOS_COMPATIBLE_VERSIONS="19.6 19.6.3-carrot"' in launch_env
  assert 'agnos_version_is_compatible "$CURRENT_AGNOS_VERSION"' in launcher
  assert "cur_version in compatible_versions" in updated
