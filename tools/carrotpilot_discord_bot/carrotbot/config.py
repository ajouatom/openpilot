from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path


def _required(name: str) -> str:
  value = os.getenv(name, "").strip()
  if not value or "여기에_직접_입력" in value:
    raise ValueError(f"필수 환경변수 {name} 값이 설정되지 않았습니다.")
  return value


def _int(name: str, default: int, minimum: int = 0) -> int:
  raw = os.getenv(name, str(default)).strip()
  try:
    value = int(raw)
  except ValueError as exc:
    raise ValueError(f"{name}은 정수여야 합니다: {raw}") from exc
  if value < minimum:
    raise ValueError(f"{name}은 {minimum} 이상이어야 합니다.")
  return value


def _bool(name: str, default: bool) -> bool:
  raw = os.getenv(name, str(default)).strip().lower()
  if raw in {"1", "true", "yes", "on"}:
    return True
  if raw in {"0", "false", "no", "off"}:
    return False
  raise ValueError(f"{name}은 true 또는 false여야 합니다: {raw}")


@dataclass(frozen=True)
class Config:
  discord_token: str
  openai_api_key: str
  discord_channel_id: int
  require_mention: bool
  openai_model: str
  daily_question_limit: int
  max_tool_rounds: int
  git_repo_url: str
  git_branch: str
  git_update_minutes: int
  repo_path: Path
  database_path: Path
  device_logs_path: Path | None

  @classmethod
  def from_env(cls) -> Config:
    channel_id = _int("DISCORD_CHANNEL_ID", 0, 1)
    model = os.getenv("OPENAI_MODEL", "gpt-5-mini").strip()
    branch = os.getenv("GIT_BRANCH", "carrot-wip").strip()
    repo_url = os.getenv("GIT_REPO_URL", "https://github.com/ajouatom/openpilot.git").strip()
    if not model or not branch or not repo_url:
      raise ValueError("OPENAI_MODEL, GIT_BRANCH, GIT_REPO_URL은 비어 있을 수 없습니다.")

    return cls(
      discord_token=_required("DISCORD_BOT_TOKEN"),
      openai_api_key=_required("OPENAI_API_KEY"),
      discord_channel_id=channel_id,
      require_mention=_bool("REQUIRE_MENTION", False),
      openai_model=model,
      daily_question_limit=_int("DAILY_QUESTION_LIMIT", 100, 1),
      max_tool_rounds=_int("MAX_TOOL_ROUNDS", 4, 1),
      git_repo_url=repo_url,
      git_branch=branch,
      git_update_minutes=_int("GIT_UPDATE_MINUTES", 60, 5),
      repo_path=Path(os.getenv("REPO_PATH", "/repos/openpilot")),
      database_path=Path(os.getenv("DATABASE_PATH", "/data/carrotpilot_bot.sqlite3")),
      device_logs_path=(
        Path(os.environ["DEVICE_LOGS_PATH"])
        if os.getenv("DEVICE_LOGS_PATH", "").strip()
        else None
      ),
    )
