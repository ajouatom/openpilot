from __future__ import annotations

from datetime import date, datetime
import json
from pathlib import Path
import re
from typing import Any


DONGLE_ID_RE = re.compile(r"(?<![0-9a-fA-F])([0-9a-fA-F]{16})(?![0-9a-fA-F])")
SENSITIVE_KEY_RE = re.compile(
  r"token|secret|pass(word)?|api.?key|auth|cookie|imei|iccid|ip(address)?|vin|serial|dongle|device.?id|latitude|longitude",
  re.I,
)
IPV4_RE = re.compile(r"(?<!\d)(?:\d{1,3}\.){3}\d{1,3}(?!\d)")
LONG_NUMBER_RE = re.compile(r"(?<!\d)\d{14,22}(?!\d)")
SECRET_ASSIGNMENT_RE = re.compile(
  r"(?i)\b(token|secret|password|api[_-]?key|authorization|cookie|imei|iccid|vin|serial|dongle[_-]?id|device[_-]?id|latitude|longitude)\b(\s*[:=]\s*)([^\s,;]+)"
)
IMPORTANT_LOG_RE = re.compile(
  r"traceback|exception|fatal|error|failed|failure|warning|assert|crash|tombstone|blocked|timeout|unhealthy",
  re.I,
)
MAX_SETTINGS_CHARS = 9000
MAX_LOG_CHARS = 14000


class DeviceLogError(RuntimeError):
  pass


def contains_dongle_id(text: str) -> bool:
  return DONGLE_ID_RE.search(text) is not None


def _redact_text(text: str) -> str:
  text = IPV4_RE.sub("[IP 숨김]", text)
  text = LONG_NUMBER_RE.sub("[장치 식별번호 숨김]", text)
  return SECRET_ASSIGNMENT_RE.sub(lambda match: f"{match.group(1)}{match.group(2)}[숨김]", text)


def _redact_json(value: Any) -> Any:
  if isinstance(value, dict):
    return {
      str(key): "[숨김]" if SENSITIVE_KEY_RE.search(str(key)) else _redact_json(item)
      for key, item in value.items()
    }
  if isinstance(value, list):
    return [_redact_json(item) for item in value]
  if isinstance(value, str):
    return _redact_text(value)
  return value


def _timestamp(path: Path) -> str:
  return datetime.fromtimestamp(path.stat().st_mtime).astimezone().isoformat(timespec="seconds")


class DeviceLogs:
  def __init__(self, root: Path | None):
    self.root = root

  def _require_root(self) -> Path:
    if self.root is None:
      raise DeviceLogError("장치 로그 폴더가 설정되지 않았습니다.")
    root = self.root.resolve()
    if not root.is_dir():
      raise DeviceLogError("장치 로그 폴더를 읽을 수 없습니다. 관리자에게 공유폴더 연결을 확인해 달라고 해주세요.")
    return root

  def is_available(self) -> bool:
    return self.root is not None and self.root.is_dir()

  def _device_folders(self, dongle_id: str) -> list[Path]:
    normalized = dongle_id.strip().lower()
    if not re.fullmatch(r"[0-9a-f]{16}", normalized):
      raise DeviceLogError("동글 ID는 영문 a~f와 숫자로 된 16자리 값이어야 합니다.")
    root = self._require_root()
    matches = [
      folder
      for folder in root.iterdir()
      if folder.is_dir() and folder.name.rsplit(" ", 1)[-1].lower() == normalized
    ]
    if not matches:
      raise DeviceLogError("해당 동글 ID의 업로드 로그를 찾지 못했습니다. 장치가 최근 로그를 전송했는지 확인해 주세요.")
    return matches

  @staticmethod
  def _select(files: list[Path], offset: int) -> Path | None:
    ordered = sorted(files, key=lambda path: path.stat().st_mtime, reverse=True)
    return ordered[offset] if offset < len(ordered) else None

  @staticmethod
  def _settings_text(path: Path | None) -> str:
    if path is None:
      return "설정 파일 없음"
    try:
      value = json.loads(path.read_text(encoding="utf-8", errors="replace"))
      content = json.dumps(_redact_json(value), ensure_ascii=False, indent=2, sort_keys=True)
    except (OSError, json.JSONDecodeError) as exc:
      return f"설정 파일 읽기 실패: {exc}"
    return content[:MAX_SETTINGS_CHARS]

  @staticmethod
  def _log_text(path: Path | None, question: str) -> str:
    if path is None:
      return "tmux/onroad 로그 없음"
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    query_terms = {
      term.casefold()
      for term in re.findall(r"[0-9A-Za-z가-힣_+-]{3,}", question)
      if term.casefold() not in {"오류", "로그", "확인", "분석", "해주세요", "문제", "동글아이디", "동글id"}
    }
    selected_indexes: set[int] = set(range(max(0, len(lines) - 120), len(lines)))
    for index, line in enumerate(lines):
      folded = line.casefold()
      if IMPORTANT_LOG_RE.search(line) or any(term in folded for term in query_terms):
        selected_indexes.update(range(max(0, index - 2), min(len(lines), index + 3)))
    selected = [f"{index + 1}:{_redact_text(lines[index])}" for index in sorted(selected_indexes)]
    text = "\n".join(selected)
    if len(text) > MAX_LOG_CHARS:
      text = text[:7000] + "\n... 로그 일부 생략 ...\n" + text[-7000:]
    return text

  def inspect(self, dongle_id: str, question: str, requested_date: str, session_offset: int = 0) -> str:
    if not 0 <= session_offset <= 4:
      raise DeviceLogError("session_offset은 0~4 범위여야 합니다.")
    try:
      parsed_date = date.fromisoformat(requested_date)
    except ValueError as exc:
      raise DeviceLogError("로그 날짜는 YYYY-MM-DD 형식이어야 합니다.") from exc
    date_token = parsed_date.strftime("%Y%m%d")
    folders = self._device_folders(dongle_id)
    dated_logs = [
      log_file
      for folder in folders
      for log_file in folder.glob(f"onroad-{date_token}-*.txt")
    ]
    log_file = self._select(dated_logs, session_offset)
    if log_file is None:
      raise DeviceLogError(
        f"{requested_date} 날짜의 onroad 로그가 없습니다. 과거 날짜의 로그는 대신 사용하지 않습니다. "
        + "장치가 해당 날짜 로그를 전송했는지 확인해 주세요."
      )

    # A dongle can appear under several historical vehicle folders. Select the
    # requested date's newest session across all folders, then pair settings
    # with the vehicle folder that owns that session.
    folder = log_file.parent

    # 같은 세션 이름의 설정 파일을 우선하고, 없으면 동일 순번의 최신 설정을 사용한다.
    session = log_file.name.removeprefix("onroad-").split("-carrot", 1)[0]
    matching_settings = folder / f"toggles-{session}.json"
    settings_file = matching_settings if matching_settings.is_file() else self._select(
      list(folder.glob(f"toggles-{date_token}-*.json")), session_offset
    )
    vehicle = folder.name.rsplit(" ", 1)[0]
    payload = {
      "vehicle_folder": vehicle,
      "dongle_id": f"****{dongle_id[-4:]}",
      "session_offset": session_offset,
      "requested_date": requested_date,
      "onroad_file": log_file.name,
      "onroad_modified": _timestamp(log_file),
      "settings_file": settings_file.name if settings_file else None,
      "settings_modified": _timestamp(settings_file) if settings_file else None,
    }
    return (
      "장치 로그 메타데이터:\n"
      + json.dumps(payload, ensure_ascii=False, indent=2)
      + "\n\n전송된 설정값(민감정보 마스킹):\n"
      + self._settings_text(settings_file)
      + "\n\ntmux/onroad 주요 로그(민감정보 마스킹, 중요 줄과 끝부분):\n"
      + self._log_text(log_file, question)
    )
