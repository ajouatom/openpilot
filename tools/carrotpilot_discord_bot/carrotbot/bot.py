from __future__ import annotations

import asyncio
import logging

import discord

from carrotbot.agent import SupportAgent
from carrotbot.config import Config
from carrotbot.device_logs import contains_dongle_id, redact_attachment_text
from carrotbot.repository import Repository
from carrotbot.storage import Storage
from carrotbot.text import make_thread_name, split_discord_message
from carrotbot.version import BOT_VERSION


log = logging.getLogger(__name__)

IMAGE_CONTENT_TYPES = {"image/png", "image/jpeg", "image/webp", "image/gif"}
IMAGE_SUFFIXES = (".png", ".jpg", ".jpeg", ".webp", ".gif")
MAX_QUESTION_IMAGES = 4
TEXT_CONTENT_TYPES = {
  "application/json",
  "application/toml",
  "application/x-yaml",
  "text/csv",
  "text/plain",
  "text/tab-separated-values",
  "text/yaml",
}
TEXT_SUFFIXES = (".json", ".txt", ".log", ".yaml", ".yml", ".toml", ".ini", ".cfg", ".csv", ".tsv")
MAX_TEXT_ATTACHMENTS = 4
MAX_ATTACHMENT_BYTES = 128 * 1024
MAX_TOTAL_ATTACHMENT_BYTES = 256 * 1024


class CarrotPilotBot(discord.Client):
  def __init__(self, config: Config):
    intents = discord.Intents.default()
    intents.message_content = True
    super().__init__(intents=intents)
    self.config = config
    self.repository = Repository(
      config.repo_path,
      config.git_repo_url,
      config.git_branch,
      config.device_logs_path,
    )
    self.storage = Storage(config.database_path)
    self.agent = SupportAgent(
      config.openai_api_key,
      config.openai_model,
      self.repository,
      config.max_tool_rounds,
    )
    self._repository_ready = False
    self._updater_task: asyncio.Task[None] | None = None

  async def on_ready(self) -> None:
    log.info(
      "Discord connected as %s | bot_version=%s | channel_id=%s | require_mention=%s",
      self.user,
      BOT_VERSION,
      self.config.discord_channel_id,
      self.config.require_mention,
    )
    if not self._repository_ready:
      await asyncio.to_thread(self.repository.ensure_available)
      self._repository_ready = True
      info = self.repository.repo_info()
      log.info("Repository ready: %s %s", info["branch"], info["commit"])
      log.info(
        "Device logs: path=%s available=%s",
        self.config.device_logs_path,
        self.repository.device_logs.is_available(),
      )
    if self._updater_task is None or self._updater_task.done():
      self._updater_task = asyncio.create_task(self._update_repository_loop())

  async def _update_repository_loop(self) -> None:
    while not self.is_closed():
      await asyncio.sleep(self.config.git_update_minutes * 60)
      try:
        await asyncio.to_thread(self.repository.update)
        log.info("Repository update completed")
      except Exception:
        log.exception("Repository update failed; keeping the current checkout")

  def _allowed_channel(self, channel: discord.abc.Messageable) -> bool:
    if getattr(channel, "id", None) == self.config.discord_channel_id:
      return True
    return isinstance(channel, discord.Thread) and channel.parent_id == self.config.discord_channel_id

  def _question_from(self, message: discord.Message) -> str | None:
    content = message.content.strip()
    # 전용 질문 채널에서는 설정에 따라 멘션을 요구하지만, 생성된 쓰레드의
    # 후속 대화는 멘션 없이 자연스럽게 이어간다.
    if self.config.require_mention and not isinstance(message.channel, discord.Thread):
      if self.user is None or self.user not in message.mentions:
        return None
      content = content.replace(f"<@{self.user.id}>", "").replace(f"<@!{self.user.id}>", "").strip()
    if content:
      return content
    if self._image_urls(message):
      return "첨부된 오류 이미지를 확인해 주세요."
    if self._text_attachment_candidates(message):
      return "첨부된 설정 또는 로그 파일을 확인해 주세요."
    return None

  @staticmethod
  def _image_urls(message: discord.Message) -> list[str]:
    urls: list[str] = []
    for attachment in message.attachments:
      content_type = (attachment.content_type or "").lower()
      filename = attachment.filename.lower()
      if content_type in IMAGE_CONTENT_TYPES or filename.endswith(IMAGE_SUFFIXES):
        urls.append(attachment.url)
        if len(urls) >= MAX_QUESTION_IMAGES:
          break
    return urls

  @staticmethod
  def _text_attachment_candidates(message: discord.Message) -> list[discord.Attachment]:
    attachments: list[discord.Attachment] = []
    for attachment in message.attachments:
      content_type = (attachment.content_type or "").split(";", 1)[0].lower()
      filename = attachment.filename.lower()
      if content_type in TEXT_CONTENT_TYPES or filename.endswith(TEXT_SUFFIXES):
        attachments.append(attachment)
        if len(attachments) >= MAX_TEXT_ATTACHMENTS:
          break
    return attachments

  async def _attachment_texts(self, message: discord.Message) -> list[str]:
    results: list[str] = []
    total_bytes = 0
    for attachment in self._text_attachment_candidates(message):
      filename = attachment.filename.replace("\n", " ").replace("\r", " ")[:160]
      declared_size = int(getattr(attachment, "size", 0) or 0)
      if declared_size > MAX_ATTACHMENT_BYTES:
        results.append(f"Attachment `{filename}` was skipped because it exceeds 128 KB.")
        continue
      try:
        data = await attachment.read()
      except (discord.HTTPException, OSError):
        log.warning("Failed to read Discord attachment filename=%s", filename, exc_info=True)
        results.append(f"Attachment `{filename}` could not be read.")
        continue
      if len(data) > MAX_ATTACHMENT_BYTES:
        results.append(f"Attachment `{filename}` was skipped because it exceeds 128 KB.")
        continue
      if total_bytes + len(data) > MAX_TOTAL_ATTACHMENT_BYTES:
        results.append("Remaining text attachments were skipped because the 256 KB total limit was reached.")
        break
      total_bytes += len(data)
      text = data.decode("utf-8-sig", errors="replace")
      results.append(f"Attachment: {filename}\n{redact_attachment_text(text)}")
    return results

  @staticmethod
  def _member_profile(message: discord.Message) -> dict[str, object]:
    roles = [role.name for role in getattr(message.author, "roles", []) if getattr(role, "name", "") != "@everyone"]
    return {
      "discord_user_id": str(message.author.id),
      "username": message.author.name,
      "display_name": message.author.display_name,
      "roles": roles,
    }

  async def on_message(self, message: discord.Message) -> None:
    if message.author.bot or not self._allowed_channel(message.channel):
      return
    question = self._question_from(message)
    if question is None:
      return
    image_urls = self._image_urls(message)
    attachment_texts = await self._attachment_texts(message)
    if not self._repository_ready:
      await message.reply("저장소를 준비하고 있습니다. 잠시 후 다시 질문해 주세요.", mention_author=False)
      return

    if question in {"!상태", "!status"}:
      info = await asyncio.to_thread(self.repository.repo_info)
      logs_status = "연결됨" if self.repository.device_logs.is_available() else "미연결"
      await message.reply(
        f"정상 동작 중 · bot `{BOT_VERSION}` · `{info['branch']}` / "
        + f"`{info['commit']}` · `{self.config.openai_model}` · 장치 로그 `{logs_status}`",
        mention_author=False,
      )
      return

    allowed, used = self.storage.consume_quota(str(message.author.id), self.config.daily_question_limit)
    if not allowed:
      await message.reply(
        f"오늘 질문 한도({self.config.daily_question_limit}회)를 모두 사용했습니다.",
        mention_author=False,
      )
      return

    target_channel: discord.abc.Messageable = message.channel
    reply_to: discord.Message | None = message
    if not isinstance(message.channel, discord.Thread):
      try:
        target_channel = await message.create_thread(
          name=make_thread_name(question, message.author.display_name),
          auto_archive_duration=1440,
          reason="CarrotPilot 질문 자동 분리",
        )
        reply_to = None
      except discord.HTTPException:
        log.exception("Failed to create a question thread")
        await message.reply(
          "질문 쓰레드를 만들지 못했습니다. 봇에 `공개 스레드 만들기`와\n" + "`스레드에서 메시지 보내기` 권한이 있는지 확인해 주세요.",
          mention_author=False,
        )
        return

    context_id = str(target_channel.id)
    info = await asyncio.to_thread(self.repository.repo_info)
    cache_model = f"{self.config.openai_model}@bot-{BOT_VERSION}"
    history = self.storage.recent_history(context_id)
    uses_device_logs = contains_dongle_id(question) or any(contains_dongle_id(item[0]) for item in history)
    cached = None if image_urls or attachment_texts or uses_device_logs else self.storage.cached_answer(question, info["commit"], cache_model)
    if cached is not None:
      self.storage.save_answer(
        context_id,
        str(message.author.id),
        question,
        cached,
        info["commit"],
        cache_model,
      )
      await self._send_answer(
        target_channel,
        cached + "\n\n_같은 커밋의 저장된 답변을 재사용했습니다._",
        reply_to,
      )
      return

    try:
      async with target_channel.typing():
        answer = await asyncio.to_thread(
          self.agent.answer,
          question,
          str(message.author.id),
          history,
          self._member_profile(message),
          image_urls,
          attachment_texts,
        )
      self.storage.save_answer(
        context_id,
        str(message.author.id),
        question,
        answer,
        info["commit"],
        cache_model,
        cacheable=not image_urls and not attachment_texts and not uses_device_logs,
      )
      await self._send_answer(target_channel, answer, reply_to)
      log.info("Answered user=%s daily_count=%d", message.author.id, used)
    except Exception:
      log.exception("Failed to answer Discord question")
      await target_channel.send(
        "질문을 처리하지 못했습니다. 잠시 후 다시 시도해 주세요. 관리자에게는 Container Manager 로그 확인이 필요합니다.",
        allowed_mentions=discord.AllowedMentions.none(),
      )

  async def _send_answer(
    self,
    channel: discord.abc.Messageable,
    answer: str,
    reply_to: discord.Message | None = None,
  ) -> None:
    chunks = split_discord_message(answer)
    allowed_mentions = discord.AllowedMentions.none()
    if reply_to is None:
      await channel.send(chunks[0], allowed_mentions=allowed_mentions)
    else:
      await reply_to.reply(chunks[0], mention_author=False, allowed_mentions=allowed_mentions)
    for chunk in chunks[1:]:
      await channel.send(chunk, allowed_mentions=allowed_mentions)


def run_bot(config: Config) -> None:
  CarrotPilotBot(config).run(config.discord_token, log_handler=None)
