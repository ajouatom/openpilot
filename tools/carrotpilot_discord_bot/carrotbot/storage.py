from __future__ import annotations

from datetime import UTC, datetime
from pathlib import Path
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
      """
    )
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
