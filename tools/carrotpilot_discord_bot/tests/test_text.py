from carrotbot.text import make_thread_name, split_discord_message


def test_make_thread_name() -> None:
  name = make_thread_name("  브레이크를  밟으면\n크루즈가 꺼져요?  ", "홍 길동")
  assert name == "질문 · 홍 길동 · 브레이크를 밟으면 크루즈가 꺼져요?"


def test_make_thread_name_is_limited() -> None:
  assert len(make_thread_name("가" * 200, "사용자")) == 95


def test_split_discord_message() -> None:
  text = "첫 줄\n" + ("가" * 2000) + "\n마지막"
  chunks = split_discord_message(text, limit=100)
  assert all(0 < len(chunk) <= 100 for chunk in chunks)
  assert "".join(chunks).replace("\n", "") == text.replace("\n", "")
