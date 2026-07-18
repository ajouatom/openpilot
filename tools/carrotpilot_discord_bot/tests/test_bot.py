import asyncio
from types import SimpleNamespace

from carrotbot.bot import CarrotPilotBot


def test_image_attachments_become_question_inputs() -> None:
  message = SimpleNamespace(
    content="",
    attachments=[
      SimpleNamespace(content_type="image/png", filename="error.png", url="https://cdn.discordapp.com/error.png"),
      SimpleNamespace(content_type="text/plain", filename="notes.txt", url="https://cdn.discordapp.com/notes.txt"),
    ],
  )
  bot = object.__new__(CarrotPilotBot)
  bot.config = SimpleNamespace(require_mention=False)

  assert bot._image_urls(message) == ["https://cdn.discordapp.com/error.png"]
  assert bot._question_from(message) == "첨부된 오류 이미지를 확인해 주세요."


def test_text_attachment_without_message_becomes_a_question() -> None:
  message = SimpleNamespace(
    content="",
    attachments=[
      SimpleNamespace(content_type="text/plain", filename="notes.txt", url="https://cdn.discordapp.com/notes.txt"),
    ],
  )
  bot = object.__new__(CarrotPilotBot)
  bot.config = SimpleNamespace(require_mention=False)

  assert bot._question_from(message) == "첨부된 설정 또는 로그 파일을 확인해 주세요."


def test_reads_and_redacts_json_attachment() -> None:
  class FakeAttachment:
    content_type = "application/json"
    filename = "params.json"
    size = 80

    async def read(self) -> bytes:
      return b'{"CruiseButtonMode": 1, "ApiToken": "secret-value", "ip": "10.0.0.3", "note": "dongle c12f3d0cc306ec8d"}'

  message = SimpleNamespace(content="check this", attachments=[FakeAttachment()])
  bot = object.__new__(CarrotPilotBot)

  texts = asyncio.run(bot._attachment_texts(message))

  assert len(texts) == 1
  assert "CruiseButtonMode" in texts[0]
  assert "secret-value" not in texts[0]
  assert "10.0.0.3" not in texts[0]
  assert "c12f3d0cc306ec8d" not in texts[0]
  assert "****ec8d" in texts[0]


def test_save_discord_member_uses_current_display_name_and_roles() -> None:
  saved: dict[str, str] = {}

  class FakeStorage:
    def save_discord_member(self, user_id, username, display_name, roles, updated_at) -> None:
      saved.update(
        user_id=user_id, username=username, display_name=display_name, roles=roles, updated_at=updated_at,
      )

  member = SimpleNamespace(
    id=778,
    name="ppuang",
    display_name="뿌앙_꾸앙/K5 dl3 pe 2023/공짜롱컨",
    bot=False,
    roles=[SimpleNamespace(name="@everyone"), SimpleNamespace(name="C4")],
  )
  bot = object.__new__(CarrotPilotBot)
  bot.storage = FakeStorage()

  bot._save_discord_member(member, "2026-07-18T12:00:00+00:00")

  assert saved["display_name"] == "뿌앙_꾸앙/K5 dl3 pe 2023/공짜롱컨"
  assert saved["roles"] == "C4"
