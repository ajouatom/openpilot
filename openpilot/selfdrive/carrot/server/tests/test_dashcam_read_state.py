import json

import pytest

from openpilot.selfdrive.carrot.server.features.dashcam import read_state


@pytest.fixture
def state_path(tmp_path, monkeypatch):
  path = tmp_path / "state" / "dashcam_read_state.json"
  monkeypatch.setattr(read_state, "CARROT_DASHCAM_READ_STATE_PATH", str(path))
  return path


def test_recent_segment_survives_reload_and_replaces_the_previous_value(state_path):
  read_state.write_dashcam_recent_segment("route-a--1")
  read_state.write_dashcam_recent_segment("route-b--2")

  assert read_state.read_dashcam_read_state() == {"recentSegment": "route-b--2"}
  payload = json.loads(state_path.read_text(encoding="utf-8"))
  assert payload["recentSegment"] == "route-b--2"
  assert payload["version"] == 1


def test_invalid_or_corrupt_state_never_leaks_into_the_client(state_path):
  state_path.parent.mkdir(parents=True)
  state_path.write_text('{"recentSegment":"../outside"}', encoding="utf-8")

  assert read_state.read_dashcam_read_state() == {"recentSegment": ""}
  with pytest.raises(ValueError):
    read_state.write_dashcam_recent_segment("../outside")
