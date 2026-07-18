import pytest

from carrotbot.config import _id_set


def test_priority_discord_user_ids(monkeypatch: pytest.MonkeyPatch) -> None:
  monkeypatch.setenv("PRIORITY_DISCORD_USER_IDS", "123, 456")
  assert _id_set("PRIORITY_DISCORD_USER_IDS") == frozenset({"123", "456"})


def test_priority_discord_user_ids_reject_names(monkeypatch: pytest.MonkeyPatch) -> None:
  monkeypatch.setenv("PRIORITY_DISCORD_USER_IDS", "CarrotMaster")
  with pytest.raises(ValueError):
    _id_set("PRIORITY_DISCORD_USER_IDS")

