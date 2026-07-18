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


def test_non_image_attachment_without_text_is_ignored() -> None:
  message = SimpleNamespace(
    content="",
    attachments=[
      SimpleNamespace(content_type="text/plain", filename="notes.txt", url="https://cdn.discordapp.com/notes.txt"),
    ],
  )
  bot = object.__new__(CarrotPilotBot)
  bot.config = SimpleNamespace(require_mention=False)

  assert bot._question_from(message) is None
