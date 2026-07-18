from pathlib import Path

from carrotbot.storage import Storage


def test_quota_cache_and_history(tmp_path: Path) -> None:
  storage = Storage(tmp_path / "bot.sqlite3")
  assert storage.consume_quota("user", 2) == (True, 1)
  assert storage.consume_quota("user", 2) == (True, 2)
  assert storage.consume_quota("user", 2) == (False, 2)

  storage.save_answer("channel", "user", "  브레이크 질문  ", "답변", "abc", "model")
  assert storage.cached_answer("브레이크   질문", "abc", "model") == "답변"
  assert storage.cached_answer("브레이크 질문", "def", "model") is None
  assert storage.recent_history("channel") == [("  브레이크 질문  ", "답변")]

  storage.save_answer("image-thread", "user", "오류 이미지 확인", "이미지 답변", "abc", "model", cacheable=False)
  assert storage.cached_answer("오류 이미지 확인", "abc", "model") is None
  assert storage.recent_history("image-thread") == [("오류 이미지 확인", "이미지 답변")]


def test_similar_discord_context_prioritizes_configured_member(tmp_path: Path) -> None:
  storage = Storage(tmp_path / "bot.sqlite3")
  storage.save_discord_message(
    "1", "old-thread", "100", "일반회원", False,
    "크루즈 목표속도는 버튼으로 바꾸세요.", "2026-07-17T01:00:00+00:00",
  )
  storage.save_discord_message(
    "2", "trusted-thread", "200", "CarrotMaster", False,
    "계기판 속도를 쓰려면 SpeedFromPCM 설정을 확인하세요.", "2026-07-16T01:00:00+00:00",
  )

  results = storage.similar_discord_context(
    "계기판 속도를 크루즈 목표속도로 쓰는 방법",
    "current-thread",
    frozenset({"200"}),
  )

  assert results
  assert "[priority member] CarrotMaster" in results[0]
  assert "SpeedFromPCM" in results[0]
  assert storage.discord_message_count() == 2
