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
    "1", "old-thread", "100", "일반회원", "normal-user", "C4", False,
    "크루즈 목표속도는 버튼으로 바꾸세요.", "2026-07-17T01:00:00+00:00",
  )
  storage.save_discord_message(
    "2", "trusted-thread", "200", "CarrotMaster", "carrot-master", "당근, C4", False,
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


def test_batch_discord_message_archive(tmp_path: Path) -> None:
  storage = Storage(tmp_path / "bot.sqlite3")
  storage.save_discord_messages([
    ("1", "general", "100", "회원", "member", "C4", False, "싼타페TM 레이더트랙 설정", "2026-07-18T01:00:00+00:00"),
    ("2", "general", "101", "회원2", "member2", "", False, "EnableRadarTracks를 확인하세요", "2026-07-18T01:01:00+00:00"),
  ])

  results = storage.similar_discord_context("싼타페TM 레이더트랙", "current")

  assert storage.discord_message_count() == 2
  assert results
  assert "싼타페TM 레이더트랙 설정" in results[0]


def test_member_alias_lookup_returns_profile_and_messages(tmp_path: Path) -> None:
  storage = Storage(tmp_path / "bot.sqlite3")
  storage.save_discord_message(
    "10", "thread", "777", "뿌앙꾸앙/싼타페TM21년/C4", "ppuang", "당근, C4", False,
    "제 차량은 싼타페 TM 2021년식입니다.", "2026-07-18T01:00:00+00:00",
  )

  results = storage.discord_member_context("뿌앙꾸앙의 차량은 레이더트랙을 지원하나요?")

  assert results
  assert "뿌앙꾸앙/싼타페TM21년/C4" in results[0]
  assert "싼타페 TM 2021년식" in results[0]
  assert "user_id=****" in results[0]


def test_member_alias_lookup_ignores_punctuation_and_needs_no_saved_message(tmp_path: Path) -> None:
  storage = Storage(tmp_path / "bot.sqlite3")
  storage.save_discord_member(
    "778", "ppuang", "뿌앙_꾸앙/K5 dl3 pe 2023/공짜롱컨", "C4", "2026-07-18T01:00:00+00:00",
  )

  results = storage.discord_member_context("뿌앙꾸앙의 차량은 레이더트랙을 지원하나요?")

  assert results
  assert "뿌앙_꾸앙/K5 dl3 pe 2023/공짜롱컨" in results[0]
  assert "no indexed messages" in results[0]


def test_member_profile_sync_does_not_replace_new_name_with_old_history(tmp_path: Path) -> None:
  storage = Storage(tmp_path / "bot.sqlite3")
  storage.save_discord_member("778", "ppuang", "뿌앙_꾸앙/K5", "C4", "2026-07-18T02:00:00+00:00")
  storage.save_discord_member("778", "ppuang", "예전별명", "C4", "2026-07-17T02:00:00+00:00")

  results = storage.discord_member_context("뿌앙꾸앙 차량")

  assert results
  assert "뿌앙_꾸앙/K5" in results[0]


def test_batch_member_profile_sync(tmp_path: Path) -> None:
  storage = Storage(tmp_path / "bot.sqlite3")
  storage.save_discord_members([
    ("778", "ppuang", "뿌앙_꾸앙/K5", "C4", "2026-07-18T02:00:00+00:00"),
    ("779", "driver", "EV6 Driver", "C3", "2026-07-18T02:00:00+00:00"),
  ])

  assert "뿌앙_꾸앙/K5" in storage.discord_member_context("뿌앙꾸앙 차량")[0]


def test_member_mention_lookup_uses_user_id(tmp_path: Path) -> None:
  storage = Storage(tmp_path / "bot.sqlite3")
  storage.save_discord_message(
    "11", "thread", "888", "별명", "stable-user", "C3", False,
    "아이오닉5를 사용합니다.", "2026-07-18T01:00:00+00:00",
  )

  results = storage.discord_member_context("이 사람 차량은 무엇인가요?", frozenset({"888"}))

  assert results
  assert "아이오닉5" in results[0]
