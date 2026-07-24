from __future__ import annotations

import json

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


def _healthy_status() -> dict:
  return {
    "status": "running",
    "runner_alive": True,
    "children": {
      "camerad": {"alive": True},
      "youtube_medium_encoderd": {"alive": True},
    },
    "youtube": {
      "state": "live",
      "transport_connected": True,
      "frame_matches_target": True,
      "target_fps": 20,
      "stream_source_recent_fps": 19.8,
      "target_video_kbps": 2_000,
      "upload_recent_kbps": 1_900,
      "rtmp_writer_frames_written": 500,
      "rtmp_writer_pending_frames": 1,
      "rtmp_writer_capacity": 30,
      "rtmp_writer_pending_bytes": 1_024,
      "rtmp_writer_capacity_bytes": 4 * 1024 * 1024,
      "last_frame_age_ms": 35,
    },
  }


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


def test_diagnosis_accepts_a_stable_bounded_stream():
  status = _healthy_status()

  diagnosis = youtube_test._diagnose_status(status)

  assert diagnosis["healthy"] is True
  assert diagnosis["verdict"] == "pass"
  assert diagnosis["failures"] == []


def test_diagnosis_reports_source_upload_and_backlog_failures():
  status = {
    "runner_alive": True,
    "children": {"camerad": {"alive": True}},
    "youtube": {
      "state": "live",
      "transport_connected": True,
      "target_fps": 20,
      "stream_source_recent_fps": 8,
      "target_video_kbps": 2_000,
      "upload_recent_kbps": 500,
      "rtmp_writer_frames_written": 1,
      "rtmp_writer_pending_frames": 30,
      "rtmp_writer_capacity": 30,
      "rtmp_writer_pending_bytes": 4 * 1024 * 1024,
      "rtmp_writer_capacity_bytes": 4 * 1024 * 1024,
    },
  }

  diagnosis = youtube_test._diagnose_status(status)

  assert diagnosis["healthy"] is False
  assert any("source rate" in failure for failure in diagnosis["failures"])
  assert any("upload rate" in failure for failure in diagnosis["failures"])
  assert "RTMP writer backlog reached its limit" in diagnosis["failures"]


def test_verify_action_routes_to_the_one_shot_test(monkeypatch):
  monkeypatch.setattr(youtube_test, "verify_test", lambda: 9)

  assert youtube_test.run_command(["verify"]) == 9


def test_one_shot_verification_requires_a_stable_window_and_always_stops(monkeypatch):
  clock = {"value": 100.0}
  status_calls = {"count": 0}
  stopped: list[bool] = []
  reported: list[dict] = []

  def get_status() -> dict:
    status_calls["count"] += 1
    if status_calls["count"] == 1:
      return {"runner_alive": False}
    return _healthy_status()

  monkeypatch.setattr(youtube_test, "get_status", get_status)
  monkeypatch.setattr(youtube_test, "start_test", lambda *, announce_next_step: 0)
  monkeypatch.setattr(youtube_test, "print_status", lambda status=None: reported.append(status) or 0)
  monkeypatch.setattr(youtube_test, "stop_test", lambda: stopped.append(True) or 0)
  monkeypatch.setattr(youtube_test.time, "monotonic", lambda: clock["value"])
  monkeypatch.setattr(youtube_test.time, "sleep", lambda seconds: clock.__setitem__("value", clock["value"] + seconds))

  assert youtube_test.verify_test() == 0
  assert clock["value"] >= 100.0 + youtube_test.VERIFY_STABLE_SECONDS
  assert len(reported) == 1
  assert stopped == [True]


def test_compact_report_excludes_credentials(monkeypatch):
  status = _healthy_status()
  status["youtube"]["stream_key"] = "do-not-share"
  monkeypatch.setattr(youtube_test, "_tail_log", lambda lines: [f"requested-lines={lines}"])

  report = youtube_test._compact_report(status)
  encoded = json.dumps(report)

  assert "do-not-share" not in encoded
  assert report["runner_log_tail"] == ["requested-lines=40"]
