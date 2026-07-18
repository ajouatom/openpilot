from pathlib import Path

import pytest

from carrotbot.repository import Repository, RepositoryError


def make_repo(tmp_path: Path) -> Repository:
  root = tmp_path / "repo"
  (root / ".git").mkdir(parents=True)
  (root / "openpilot" / "selfdrive").mkdir(parents=True)
  (root / "openpilot" / "selfdrive" / "settings.json").write_text(
    '{"title": "오토크루즈제어"}\n',
    encoding="utf-8",
  )
  return Repository(root, "https://example.invalid/repo.git", "carrot-wip")


def test_search_and_read(tmp_path: Path) -> None:
  repo = make_repo(tmp_path)
  result = repo.search_code("오토크루즈", "openpilot")
  assert "settings.json:1" in result
  assert "오토크루즈제어" in result

  content = repo.read_file("openpilot/selfdrive/settings.json", 1, 10)
  assert content.startswith("1:")


def test_blocks_escape_and_secrets(tmp_path: Path) -> None:
  repo = make_repo(tmp_path)
  outside = tmp_path / "outside.txt"
  outside.write_text("secret", encoding="utf-8")
  (repo.root / ".env").write_text("TOKEN=secret", encoding="utf-8")

  with pytest.raises(RepositoryError):
    repo.read_file("../outside.txt", 1, 2)
  with pytest.raises(RepositoryError):
    repo.read_file(".env", 1, 2)


def test_inspects_latest_device_log_and_redacts_identifiers(tmp_path: Path) -> None:
  repo = make_repo(tmp_path)
  logs = tmp_path / "device-logs"
  device = logs / "KIA_SELTOS 13b353b3ea06dcee"
  device.mkdir(parents=True)
  (device / "toggles-20260718-100000.json").write_text(
    '{"ActivateCruiseAfterBrake":"0","ApiToken":"secret-value"}',
    encoding="utf-8",
  )
  (device / "onroad-20260718-100000-carrot-wip.txt").write_text(
    "INFO started ip=10.0.0.3\nERROR modem imei=867652074257866\nlast line\n",
    encoding="utf-8",
  )
  repo.device_logs.root = logs

  result = repo.call_tool(
    "inspect_device_logs",
    {
      "dongle_id": "13b353b3ea06dcee",
      "question": "모뎀 오류",
      "requested_date": "2026-07-18",
      "session_offset": 0,
    },
  )

  assert "KIA_SELTOS" in result
  assert "ActivateCruiseAfterBrake" in result
  assert "secret-value" not in result
  assert "10.0.0.3" not in result
  assert "867652074257866" not in result
  assert "****dcee" in result
  assert repo.device_logs.is_available()


def test_device_log_requires_exact_dongle_id(tmp_path: Path) -> None:
  repo = make_repo(tmp_path)
  repo.device_logs.root = tmp_path / "missing"
  result = repo.call_tool(
    "inspect_device_logs",
    {
      "dongle_id": "../not-an-id",
      "question": "오류",
      "requested_date": "2026-07-18",
      "session_offset": 0,
    },
  )
  assert "16자리" in result


def test_device_log_never_falls_back_to_another_date(tmp_path: Path) -> None:
  repo = make_repo(tmp_path)
  logs = tmp_path / "device-logs"
  device = logs / "KIA_SELTOS 13b353b3ea06dcee"
  device.mkdir(parents=True)
  (device / "onroad-20260717-235959-carrot-wip.txt").write_text("old error", encoding="utf-8")
  repo.device_logs.root = logs

  result = repo.call_tool(
    "inspect_device_logs",
    {
      "dongle_id": "13b353b3ea06dcee",
      "question": "오늘 오류",
      "requested_date": "2026-07-18",
      "session_offset": 0,
    },
  )

  assert "2026-07-18 날짜의 onroad 로그가 없습니다" in result
  assert "과거 날짜의 로그는 대신 사용하지 않습니다" in result
