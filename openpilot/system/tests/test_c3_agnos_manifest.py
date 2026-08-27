import json
from pathlib import Path

from openpilot.common.basedir import BASEDIR


TICI_DIR = Path(BASEDIR) / "openpilot/system/hardware/tici"


def _load_manifest(name: str) -> list[dict]:
  return json.loads((TICI_DIR / name).read_text(encoding="utf-8"))


def test_c3_manifest_uses_carrot_boot_and_official_agnos_19_system() -> None:
  c3 = {partition["name"]: partition for partition in _load_manifest("agnos-tici.json")}
  current = {partition["name"]: partition for partition in _load_manifest("agnos.json")}

  assert c3["boot"] == {
    "name": "boot",
    "url": "https://github.com/ajouatom/agnos-builder/releases/download/carrot-c3-19.6-v1/carrot-c3-agnos-19.6-boot.img.xz",
    "hash": "82c790822b3ca56689a2dee892206494e5fe85d86f0d47d654d0e36e84cb2fbd",
    "hash_raw": "82c790822b3ca56689a2dee892206494e5fe85d86f0d47d654d0e36e84cb2fbd",
    "size": 48343040,
    "sparse": False,
    "full_check": True,
    "has_ab": True,
    "ondevice_hash": "7f54840b8ae437034210065da0648b833ccd191a03569257dc849d1db9c063d8",
  }
  assert c3["boot"] != current["boot"]
  assert c3["system"] == current["system"]


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
