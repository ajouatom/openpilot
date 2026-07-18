from __future__ import annotations

from datetime import UTC, datetime
from pathlib import Path
import re
import sqlite3
import threading


class Storage:
  def __init__(self, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    self._lock = threading.Lock()
    self._db = sqlite3.connect(path, check_same_thread=False)
    self._db.execute("PRAGMA journal_mode=WAL")
    self._db.executescript(
      """
      CREATE TABLE IF NOT EXISTS daily_usage (
        day TEXT NOT NULL,
        user_id TEXT NOT NULL,
        count INTEGER NOT NULL,
        PRIMARY KEY(day, user_id)
      );
      CREATE TABLE IF NOT EXISTS answers (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        created_at TEXT NOT NULL,
        channel_id TEXT NOT NULL,
        user_id TEXT NOT NULL,
        question_key TEXT NOT NULL,
        question TEXT NOT NULL,
        answer TEXT NOT NULL,
        commit_hash TEXT NOT NULL,
        model TEXT NOT NULL
      );
      CREATE INDEX IF NOT EXISTS answers_cache
        ON answers(question_key, commit_hash, model);
      CREATE INDEX IF NOT EXISTS answers_history
        ON answers(channel_id, id DESC);
      CREATE TABLE IF NOT EXISTS discord_messages (
        message_id TEXT PRIMARY KEY,
        created_at TEXT NOT NULL,
        channel_id TEXT NOT NULL,
        author_id TEXT NOT NULL,
        author_name TEXT NOT NULL,
        author_username TEXT NOT NULL,
        author_roles TEXT NOT NULL,
        author_is_bot INTEGER NOT NULL,
        content TEXT NOT NULL
      );
      CREATE INDEX IF NOT EXISTS discord_messages_recent
        ON discord_messages(created_at DESC);
      CREATE TABLE IF NOT EXISTS discord_members (
        user_id TEXT PRIMARY KEY,
        username TEXT NOT NULL,
        display_name TEXT NOT NULL,
        roles TEXT NOT NULL,
        updated_at TEXT NOT NULL
      );
      """
    )
    discord_columns = {str(row[1]) for row in self._db.execute("PRAGMA table_info(discord_messages)").fetchall()}
    if "author_id" not in discord_columns:
      self._db.execute("ALTER TABLE discord_messages ADD COLUMN author_id TEXT NOT NULL DEFAULT ''")
    if "author_username" not in discord_columns:
      self._db.execute("ALTER TABLE discord_messages ADD COLUMN author_username TEXT NOT NULL DEFAULT ''")
    if "author_roles" not in discord_columns:
      self._db.execute("ALTER TABLE discord_messages ADD COLUMN author_roles TEXT NOT NULL DEFAULT ''")
    self._db.commit()

  @staticmethod
  def normalize_question(question: str) -> str:
    return " ".join(question.casefold().split())

  def consume_quota(self, user_id: str, limit: int) -> tuple[bool, int]:
    day = datetime.now(UTC).date().isoformat()
    with self._lock:
      row = self._db.execute("SELECT count FROM daily_usage WHERE day=? AND user_id=?", (day, user_id)).fetchone()
      count = int(row[0]) if row else 0
      if count >= limit:
        return False, count
      count += 1
      self._db.execute(
        "INSERT INTO daily_usage(day, user_id, count) VALUES(?, ?, ?) ON CONFLICT(day, user_id) DO UPDATE SET count=excluded.count",
        (day, user_id, count),
      )
      self._db.commit()
      return True, count

  def cached_answer(self, question: str, commit_hash: str, model: str) -> str | None:
    key = self.normalize_question(question)
    with self._lock:
      row = self._db.execute(
        "SELECT answer FROM answers WHERE question_key=? AND commit_hash=? AND model=? ORDER BY id DESC LIMIT 1",
        (key, commit_hash, model),
      ).fetchone()
    return str(row[0]) if row else None

  def save_answer(
    self,
    channel_id: str,
    user_id: str,
    question: str,
    answer: str,
    commit_hash: str,
    model: str,
    cacheable: bool = True,
  ) -> None:
    with self._lock:
      self._db.execute(
        "INSERT INTO answers(created_at, channel_id, user_id, question_key, question, answer, commit_hash, model) VALUES(?, ?, ?, ?, ?, ?, ?, ?)",
        (
          datetime.now(UTC).isoformat(),
          channel_id,
          user_id,
          self.normalize_question(question) if cacheable else "",
          question,
          answer,
          commit_hash,
          model,
        ),
      )
      self._db.commit()

  def recent_history(self, channel_id: str, limit: int = 3) -> list[tuple[str, str]]:
    with self._lock:
      rows = self._db.execute(
        "SELECT question, answer FROM answers WHERE channel_id=? ORDER BY id DESC LIMIT ?",
        (channel_id, limit),
      ).fetchall()
    return [(str(question), str(answer)) for question, answer in reversed(rows)]

  def save_discord_message(
    self,
    message_id: str,
    channel_id: str,
    author_id: str,
    author_name: str,
    author_username: str,
    author_roles: str,
    author_is_bot: bool,
    content: str,
    created_at: str,
  ) -> None:
    self.save_discord_messages([
      (
        message_id, channel_id, author_id, author_name, author_username, author_roles,
        author_is_bot, content, created_at,
      ),
    ])

  def save_discord_messages(
    self,
    messages: list[tuple[str, str, str, str, str, str, bool, str, str]],
  ) -> None:
    rows = [
      (
        message_id, created_at, channel_id, author_id, author_name[:100], author_username[:100],
        author_roles[:500], int(author_is_bot), content.strip()[:6000],
      )
      for (
        message_id, channel_id, author_id, author_name, author_username, author_roles,
        author_is_bot, content, created_at,
      ) in messages
      if content.strip()
    ]
    if not rows:
      return
    with self._lock:
      self._db.executemany(
        "INSERT OR REPLACE INTO discord_messages(message_id, created_at, channel_id, author_id, author_name, author_username, author_roles, author_is_bot, content) "
        + "VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?)",
        rows,
      )
      self._db.commit()

  def save_discord_member(
    self,
    user_id: str,
    username: str,
    display_name: str,
    roles: str,
    updated_at: str,
  ) -> None:
    with self._lock:
      self._db.execute(
        "INSERT INTO discord_members(user_id, username, display_name, roles, updated_at) "
        + "VALUES(?, ?, ?, ?, ?) ON CONFLICT(user_id) DO UPDATE SET "
        + "username=excluded.username, display_name=excluded.display_name, roles=excluded.roles, "
        + "updated_at=excluded.updated_at WHERE excluded.updated_at >= discord_members.updated_at",
        (user_id, username[:100], display_name[:100], roles[:500], updated_at),
      )
      self._db.commit()

  def save_discord_members(self, members: list[tuple[str, str, str, str, str]]) -> None:
    if not members:
      return
    rows = [
      (user_id, username[:100], display_name[:100], roles[:500], updated_at)
      for user_id, username, display_name, roles, updated_at in members
    ]
    with self._lock:
      self._db.executemany(
        "INSERT INTO discord_members(user_id, username, display_name, roles, updated_at) "
        + "VALUES(?, ?, ?, ?, ?) ON CONFLICT(user_id) DO UPDATE SET "
        + "username=excluded.username, display_name=excluded.display_name, roles=excluded.roles, "
        + "updated_at=excluded.updated_at WHERE excluded.updated_at >= discord_members.updated_at",
        rows,
      )
      self._db.commit()

  @staticmethod
  def _search_terms(text: str) -> set[str]:
    normalized = text.casefold()
    terms = {item for item in re.findall(r"[a-z0-9_+-]{2,}", normalized)}
    for chunk in re.findall(r"[가-힣]{2,}", normalized):
      terms.add(chunk)
      for size in (2, 3):
        terms.update(chunk[index : index + size] for index in range(len(chunk) - size + 1))
    return terms

  def similar_discord_context(
    self,
    question: str,
    exclude_channel_id: str,
    priority_user_ids: frozenset[str] = frozenset(),
    limit: int = 8,
  ) -> list[str]:
    query_terms = self._search_terms(question)
    if not query_terms:
      return []
    with self._lock:
      rows = self._db.execute(
        "SELECT created_at, channel_id, author_id, author_name, content FROM discord_messages "
        + "WHERE author_is_bot=0 AND channel_id<>? ORDER BY created_at DESC LIMIT 2500",
        (exclude_channel_id,),
      ).fetchall()

    ranked: list[tuple[float, str, str, str]] = []
    normalized_question = self.normalize_question(question)
    for created_at, channel_id, author_id, author_name, content in rows:
      message_terms = self._search_terms(str(content))
      shared = query_terms.intersection(message_terms)
      if not shared:
        continue
      score = sum(min(len(term), 8) for term in shared) / max(len(query_terms), 1)
      normalized_content = self.normalize_question(str(content))
      if normalized_question in normalized_content or normalized_content in normalized_question:
        score += 5.0
      priority = str(author_id) in priority_user_ids
      if priority:
        score += 25.0
      label = f"[priority member] {author_name}" if priority else str(author_name)
      ranked.append((score, str(created_at), label, str(content)))

    ranked.sort(key=lambda item: (item[0], item[1]), reverse=True)
    results: list[str] = []
    seen: set[str] = set()
    for _, created_at, author_name, content in ranked:
      key = self.normalize_question(content)
      if key in seen:
        continue
      seen.add(key)
      date_text = created_at[:10]
      results.append(f"[{date_text}] {author_name}: {content[:1200]}")
      if len(results) >= limit:
        break
    return results

  def discord_message_count(self) -> int:
    with self._lock:
      row = self._db.execute("SELECT COUNT(*) FROM discord_messages").fetchone()
    return int(row[0]) if row else 0

  @staticmethod
  def _member_aliases(username: str, display_name: str) -> set[str]:
    def compact(value: str) -> str:
      return re.sub(r"[^0-9a-z가-힣]+", "", value.casefold())

    aliases = {compact(username), compact(display_name)}
    aliases.update(compact(part) for part in re.split(r"[/|·]", display_name))
    return {alias for alias in aliases if len(alias) >= 2}

  def discord_member_context(
    self,
    question: str,
    mentioned_user_ids: frozenset[str] = frozenset(),
    limit_members: int = 2,
  ) -> list[str]:
    normalized_question = re.sub(r"[^0-9a-z가-힣]+", "", question.casefold())
    query_terms = self._search_terms(question)
    with self._lock:
      profile_rows = self._db.execute(
        "SELECT user_id, display_name, username, roles FROM discord_members ORDER BY updated_at DESC"
      ).fetchall()
      message_rows = self._db.execute(
        "SELECT created_at, author_id, author_name, author_username, author_roles, content "
        + "FROM discord_messages WHERE author_is_bot=0 ORDER BY created_at DESC LIMIT 4000"
      ).fetchall()

    matched_ids: list[str] = []
    profiles: dict[str, tuple[str, str, str]] = {}
    messages: dict[str, list[tuple[float, str, str]]] = {}
    for author_id, display_name, username, roles in profile_rows:
      author_id = str(author_id)
      aliases = self._member_aliases(str(username), str(display_name))
      alias_match = any(alias in normalized_question for alias in aliases)
      if author_id not in mentioned_user_ids and not alias_match:
        continue
      profiles[author_id] = (str(display_name), str(username), str(roles))
      matched_ids.append(author_id)

    # Keep compatibility with databases populated before the member directory existed.
    for created_at, author_id, display_name, username, roles, content in message_rows:
      author_id = str(author_id)
      if author_id not in profiles:
        aliases = self._member_aliases(str(username), str(display_name))
        alias_match = any(alias in normalized_question for alias in aliases)
        if author_id not in mentioned_user_ids and not alias_match:
          continue
        profiles[author_id] = (str(display_name), str(username), str(roles))
        matched_ids.append(author_id)
      if author_id not in profiles:
        continue
      shared = query_terms.intersection(self._search_terms(str(content)))
      score = float(sum(min(len(term), 8) for term in shared))
      messages.setdefault(author_id, []).append((score, str(created_at), str(content)))

    results: list[str] = []
    for author_id in matched_ids[:limit_members]:
      display_name, username, roles = profiles[author_id]
      ranked = sorted(messages.get(author_id, []), key=lambda item: (item[0], item[1]), reverse=True)
      excerpts = [f"- [{created_at[:10]}] {content[:900]}" for _, created_at, content in ranked[:5]]
      masked_id = f"****{author_id[-4:]}" if len(author_id) >= 4 else "****"
      results.append(
        f"Member lookup: {display_name} (@{username}), user_id={masked_id}, roles={roles or 'none'}\n"
        + "Messages by this member:\n"
        + ("\n".join(excerpts) if excerpts else "- no indexed messages")
      )
    return results
