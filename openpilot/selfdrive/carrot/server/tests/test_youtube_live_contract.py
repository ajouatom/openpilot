from __future__ import annotations

import re
from pathlib import Path

from openpilot.selfdrive.carrot.server.services import youtube_live
from openpilot.selfdrive.carrot.server.services import youtube_profiles
from openpilot.selfdrive.carrot.server.services import youtube_test


OPENPILOT_ROOT = Path(__file__).resolve().parents[4]
LOGGERD_HEADER = OPENPILOT_ROOT / "system/loggerd/loggerd.h"
V4L_ENCODER = OPENPILOT_ROOT / "system/loggerd/encoder/v4l_encoder.cc"
PROCESS_CONFIG = OPENPILOT_ROOT / "system/manager/process_config.py"

PROFILE_FUNCTIONS = {
  0: "YouTubeLowEncoderSettings",
  1: "YouTubeMediumEncoderSettings",
  2: "YouTubeEncoderSettings",
  3: "YouTubeWideEncoderSettings",
}

PROFILE_COMMANDS = {
  0: ("youtube_low_encoderd", "--youtube-low"),
  1: ("youtube_medium_encoderd", "--youtube-medium"),
  2: ("youtube_encoderd", "--youtube"),
  3: ("youtube_wide_encoderd", "--youtube-wide"),
}


def _cpp_integer(body: str, field: str) -> int:
  match = re.search(rf"\.{re.escape(field)}\s*=\s*([0-9']+)", body)
  assert match is not None, f"missing {field}"
  return int(match.group(1).replace("'", ""))


def _cpp_profile(header: str, function_name: str) -> dict[str, int | bool]:
  pattern = (
    rf"static EncoderSettings {re.escape(function_name)}\(\)\s*\{{"
    + r".*?return EncoderSettings\{(?P<body>.*?)\n\s*\};"
  )
  match = re.search(
    pattern,
    header,
    flags=re.DOTALL,
  )
  assert match is not None, f"missing {function_name}"
  body = match.group("body")
  return {
    "width": _cpp_integer(body, "frame_width"),
    "height": _cpp_integer(body, "frame_height"),
    "video_kbps": _cpp_integer(body, "bitrate") // 1_000,
    "gop_frames": _cpp_integer(body, "gop_size"),
    "cbr": bool(re.search(r"\.cbr\s*=\s*true", body)),
  }


def test_python_profiles_match_native_encoder_contract():
  header = LOGGERD_HEADER.read_text(encoding="utf-8")
  main_fps_match = re.search(r"constexpr int MAIN_FPS\s*=\s*(\d+)", header)
  assert main_fps_match is not None
  main_fps = int(main_fps_match.group(1))
  assert main_fps == 20

  assert set(youtube_live.QUALITY_TARGETS) == set(PROFILE_FUNCTIONS)
  for quality, function_name in PROFILE_FUNCTIONS.items():
    native = _cpp_profile(header, function_name)
    target = youtube_live.QUALITY_TARGETS[quality]
    assert native["width"] == target["width"]
    assert native["height"] == target["height"]
    assert native["video_kbps"] == target["video_kbps"]
    assert native["gop_frames"] == target["gop_seconds"] * main_fps
    assert native["cbr"] is True


def test_all_profiles_use_dedicated_youtube_source_and_encoder():
  assert set(youtube_live.SOURCE_BY_QUALITY) == set(PROFILE_COMMANDS)
  assert set(youtube_live.SOURCE_BY_QUALITY.values()) == {"youtubeRoadEncodeData"}

  for quality, (process_name, command_flag) in PROFILE_COMMANDS.items():
    spec = youtube_test._quality_spec(quality)
    assert spec["name"] == process_name
    assert spec["cmd"][-1] == command_flag
    assert command_flag in spec["match"]


def test_invalid_quality_falls_back_to_low_profile():
  assert youtube_profiles.youtube_profile(None).quality == 0
  assert youtube_profiles.youtube_profile("invalid").quality == 0
  assert youtube_profiles.youtube_profile(99).quality == 0


def test_manager_keeps_youtube_encoders_separate_from_carrot_vision():
  config = PROCESS_CONFIG.read_text(encoding="utf-8")
  for process_name, command_flag in PROFILE_COMMANDS.values():
    assert f'NativeProcess("{process_name}"' in config
    assert f'["./encoderd", "{command_flag}"]' in config

  assert 'NativeProcess("carrot_vision_encoderd"' in config
  assert '["./encoderd", "--carrot-vision-road"]' in config
  assert youtube_live._PROCESS_MATCHES["stream_encoderd"] == "encoderd\x00--carrot-vision-road"
  assert all(
    "carrot-vision-road" not in youtube_live._PROCESS_MATCHES[process_name]
    for process_name, _command_flag in PROFILE_COMMANDS.values()
  )


def test_resource_observation_does_not_change_cluster_or_vision_state(monkeypatch):
  service = object.__new__(youtube_live.YouTubeLiveService)
  reads: list[str] = []
  values = {
    "ClusterHud": 1,
    "DisableDM": 2,
    youtube_live.YOUTUBE_QUALITY_PARAM: 1,
  }
  processes = {
    "carrot_cluster": {"running": True, "pids": [10]},
    "webrtcd": {"running": False, "pids": []},
    "stream_encoderd": {"running": False, "pids": []},
    "youtube_medium_encoderd": {"running": True, "pids": [20]},
  }

  def read_param(name: str, default: int = 0) -> int:
    reads.append(name)
    return values.get(name, default)

  monkeypatch.setattr(service, "_param_int", read_param)
  monkeypatch.setattr(service, "_process_status", lambda: processes)

  status = service._resource_status()

  assert reads == ["ClusterHud", "DisableDM", youtube_live.YOUTUBE_QUALITY_PARAM]
  assert status["cluster"]["running"] is True
  assert status["cluster"]["active"] is True
  assert status["carrot_vision"]["webrtcd_running"] is False
  assert status["carrot_vision"]["configured"] is True
  assert status["carrot_vision"]["active"] is False
  assert status["youtube_encoder"]["selected"] == "youtube_medium_encoderd"
  assert status["youtube_encoder"]["running"] is True


def test_runtime_resolution_rejects_1280x800_for_1280x720_target():
  service = object.__new__(youtube_live.YouTubeLiveService)
  service._frame_width = 1280
  service._frame_height = 800

  assert service._frame_matches_target({"width": 1280, "height": 720}) is False


def test_native_youtube_crop_keeps_the_declared_output_size():
  source = V4L_ENCODER.read_text(encoding="utf-8")
  youtube_block = source.split("if (youtube_road) {", 1)[1].split('LOGD("in buffer size', 1)[0]

  assert "selection.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;" in youtube_block
  assert "selection.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;" not in youtube_block
  assert "out_height = aspect_height" not in youtube_block
  assert "fmt_out.fmt.pix_mp.height" not in youtube_block
  assert "scaling full frame to exact %dx%d" in youtube_block
  assert "YouTube encoder output contract rejected" in source


def test_status_separates_declared_and_observed_frame_rates(monkeypatch, tmp_path):
  monkeypatch.setattr(
    youtube_live,
    "pyav_capabilities",
    lambda: {"available": True, "flv": True, "h264": True, "aac": True},
  )
  monkeypatch.setattr(
    youtube_live,
    "librtmp_capabilities",
    lambda: {"available": True},
  )
  service = youtube_live.YouTubeLiveService(
    state_path=str(tmp_path / "state.json"),
    secret_path=str(tmp_path / "secret.json"),
  )
  monkeypatch.setattr(service, "_param_enabled", lambda: False)
  monkeypatch.setattr(
    service,
    "_param_int",
    lambda name, default=0: 1 if name == youtube_live.YOUTUBE_QUALITY_PARAM else default,
  )
  monkeypatch.setattr(service, "_resource_status", dict)
  monkeypatch.setattr(service, "_warnings", lambda _resources: [])

  status = service.status()

  assert status["declared_frame_fps"] == 20
  assert status["observed_frame_fps"] == 0.0


def test_carrot_vision_warning_requires_an_active_process(monkeypatch):
  service = object.__new__(youtube_live.YouTubeLiveService)
  service._transport_connected = False
  service._started_mono = 0.0
  monkeypatch.setattr(service, "_param_enabled", lambda: True)
  monkeypatch.setattr(service, "_quality_target", dict)

  warnings = service._warnings({
    "cluster": {"enabled": False, "running": False},
    "carrot_vision": {
      "enabled": True,
      "webrtcd_running": False,
      "stream_encoderd_running": False,
    },
    "youtube_encoder": {"running": True},
  })

  assert not any("Carrot Vision" in warning for warning in warnings)
