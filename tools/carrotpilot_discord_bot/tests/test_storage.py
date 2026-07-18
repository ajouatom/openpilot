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
