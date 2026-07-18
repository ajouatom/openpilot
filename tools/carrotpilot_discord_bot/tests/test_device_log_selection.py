import os
from pathlib import Path

from carrotbot.device_logs import DeviceLogs


def test_selects_latest_requested_date_across_vehicle_folders(tmp_path: Path) -> None:
  dongle_id = "c12f3d0cc306ec8d"
  niro = tmp_path / f"KIA_NIRO_EV {dongle_id}"
  tesla = tmp_path / f"TESLA_MODEL_Y {dongle_id}"
  niro.mkdir()
  tesla.mkdir()

  niro_log = niro / "onroad-20260718-090000-carrot-wip.txt"
  tesla_log = tesla / "onroad-20260718-105446-carrot-wip.txt"
  niro_log.write_text("old same-day session", encoding="utf-8")
  tesla_log.write_text("latest Tesla session", encoding="utf-8")
  (tesla / "toggles-20260718-105446.json").write_text('{"CarName":"TESLA_MODEL_Y"}', encoding="utf-8")
  os.utime(niro_log, (1_700_000_000, 1_700_000_000))
  os.utime(tesla_log, (1_800_000_000, 1_800_000_000))

  result = DeviceLogs(tmp_path).inspect(dongle_id, "check latest logs", "2026-07-18")

  assert "TESLA_MODEL_Y" in result
  assert tesla_log.name in result
  assert "latest Tesla session" in result

