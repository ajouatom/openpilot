from __future__ import annotations

from openpilot.selfdrive.carrot.server.services import youtube_test
from openpilot.selfdrive.carrot.server.terminal_commands.custom_commands import youtube_test as youtube_test_command


class FakeParams:
  def __init__(self, live: int) -> None:
    self.live = live
    self.writes: list[tuple[str, int]] = []

  def get_int(self, name: str) -> int:
    assert name == "CarrotYouTubeLive"
    return self.live

  def put_int(self, name: str, value: int) -> None:
    self.writes.append((name, value))
    self.live = value


def test_quality_specs_select_the_matching_encoder():
  assert youtube_test._quality_spec(0)["cmd"][-1] == "--youtube-low"
  assert youtube_test._quality_spec(1)["cmd"][-1] == "--youtube-medium"
  assert youtube_test._quality_spec(2)["cmd"][-1] == "--youtube"
  assert youtube_test._quality_spec(3)["cmd"][-1] == "--youtube-wide"
  assert youtube_test._quality_spec(99)["cmd"][-1] == "--youtube-low"


def test_restore_live_param_only_restores_a_value_owned_by_the_test(monkeypatch):
  params = FakeParams(live=1)
  monkeypatch.setattr(youtube_test, "_params", lambda: params)

  youtube_test._restore_live_param({"forced_live": True, "previous_live": 0})
  assert params.writes == [("CarrotYouTubeLive", 0)]

  params.writes.clear()
  params.live = 0
  youtube_test._restore_live_param({"forced_live": True, "previous_live": 0})
  assert params.writes == []

  params.live = 1
  youtube_test._restore_live_param({"forced_live": False, "previous_live": 0})
  assert params.writes == []


def test_terminal_command_routes_to_the_service(monkeypatch):
  calls = []
  monkeypatch.setattr(youtube_test, "run_command", lambda args: calls.append(args) or 7)

  assert youtube_test_command.run(["status"]) == 7
  assert calls == [["status"]]
