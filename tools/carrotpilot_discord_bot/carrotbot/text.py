from __future__ import annotations


def make_thread_name(question: str, display_name: str, limit: int = 95) -> str:
  question = " ".join(question.split()).replace("`", "")
  display_name = " ".join(display_name.split()).replace("`", "")
  prefix = f"질문 · {display_name or '사용자'} · "
  name = (prefix + question).strip()
  return name[:limit].rstrip(" .·") or "carrotpilot 질문"


def split_discord_message(text: str, limit: int = 1900) -> list[str]:
  text = text.strip()
  if not text:
    return ["답변 내용이 없습니다."]
  chunks: list[str] = []
  remaining = text
  while len(remaining) > limit:
    split_at = max(remaining.rfind("\n", 0, limit), remaining.rfind(" ", 0, limit))
    if split_at < limit // 2:
      split_at = limit
    chunks.append(remaining[:split_at].rstrip())
    remaining = remaining[split_at:].lstrip()
  if remaining:
    chunks.append(remaining)
  return chunks
