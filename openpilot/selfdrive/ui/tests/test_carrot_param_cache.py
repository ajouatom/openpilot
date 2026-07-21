import ast
import importlib.util
import sys
import types
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.ui.carrot_param_cache import (
  BorderParamSnapshot,
  RealtimeUiParamSnapshot,
  TimedSnapshotCache,
  read_border_params,
  read_realtime_ui_params,
  read_screen_record,
)


UI_DIR = Path(__file__).parents[1]
MAIN_LAYOUT_PATH = UI_DIR / "layouts" / "main.py"


class FakeParams:
  def __init__(self, values, failures=None):
    self.values = values
    self.failures = dict(failures or {})
    self.calls = []
    self.writes = []

  def _read(self, key):
    self.calls.append(key)
    if self.failures.get(key, 0) > 0:
      self.failures[key] -= 1
      raise RuntimeError(f"failed to read {key}")
    return self.values[key]

  def get(self, key):
    return self._read(key)

  def get_bool(self, key):
    return bool(self._read(key))

  def get_float(self, key):
    return float(self._read(key))

  def get_int(self, key):
    return int(self._read(key))

  def put_bool_nonblocking(self, key, value):
    self.writes.append((key, value))


@pytest.fixture
def main_layout_module(monkeypatch):
  class Widget:
    pass

  class Placeholder:
    pass

  class FakeGuiApp:
    def __init__(self):
      self.recording = False
      self.calls = []

    def is_recording(self):
      return self.recording

    def start_recording(self):
      self.calls.append("start")
      self.recording = True

    def stop_recording(self):
      self.calls.append("stop")
      self.recording = False

    def toggle_recording(self):
      self.calls.append("toggle")
      self.recording = not self.recording

  gui_app = FakeGuiApp()
  fake_ui_state = SimpleNamespace(params=None, started=True, sm=None)
  messaging = types.ModuleType("openpilot.cereal.messaging")
  messaging.PubMaster = Placeholder
  messaging.new_message = lambda *args, **kwargs: SimpleNamespace()
  cereal = types.ModuleType("openpilot.cereal")
  cereal.messaging = messaging

  stubs = {
    "pyray": SimpleNamespace(Rectangle=Placeholder),
    "openpilot.cereal": cereal,
    "openpilot.cereal.messaging": messaging,
    "openpilot.selfdrive.ui.layouts.sidebar": SimpleNamespace(Sidebar=Placeholder, SIDEBAR_WIDTH=0),
    "openpilot.selfdrive.ui.layouts.home": SimpleNamespace(HomeLayout=Placeholder),
    "openpilot.selfdrive.ui.layouts.settings.settings": SimpleNamespace(
      SettingsLayout=Placeholder,
      PanelType=SimpleNamespace(TOGGLES=0, DEVICE=1, FIREHOSE=2),
    ),
    "openpilot.selfdrive.ui.onroad.augmented_road_view": SimpleNamespace(AugmentedRoadView=Placeholder),
    "openpilot.selfdrive.ui.ui_state": SimpleNamespace(
      device=SimpleNamespace(add_interactive_timeout_callback=lambda callback: None),
      ui_state=fake_ui_state,
    ),
    "openpilot.system.ui.lib.application": SimpleNamespace(gui_app=gui_app),
    "openpilot.system.ui.widgets": SimpleNamespace(Widget=Widget),
    "openpilot.selfdrive.ui.layouts.onboarding": SimpleNamespace(OnboardingWindow=Placeholder),
  }
  for name, stub in stubs.items():
    monkeypatch.setitem(sys.modules, name, stub)

  module_name = f"_test_carrot_main_layout_{id(gui_app)}"
  spec = importlib.util.spec_from_file_location(module_name, MAIN_LAYOUT_PATH)
  assert spec is not None and spec.loader is not None
  module = importlib.util.module_from_spec(spec)
  monkeypatch.setitem(sys.modules, module_name, module)
  spec.loader.exec_module(module)
  return module, gui_app, fake_ui_state


def make_main_layout(module, gui_app, ui_state, params):
  ui_state.params = params
  layout = object.__new__(module.MainLayout)
  layout._last_carrot_cmd_idx = 0
  layout._screen_record_param = TimedSnapshotCache(
    gui_app.is_recording(),
    lambda: read_screen_record(params),
  )
  return layout


def carrot_command(index, arg):
  return {
    "carrotMan": SimpleNamespace(
      carrotCmdIndex=index,
      carrotCmd="RECORD",
      carrotArg=arg,
    ),
  }


def test_snapshot_refreshes_at_half_second_boundary():
  values = iter((1, 2))
  calls = []
  cache = TimedSnapshotCache(0, lambda: calls.append(True) or next(values))

  assert cache.refresh(0.0) == 1
  assert cache.next_refresh_time == 0.5
  assert cache.refresh(0.499) == 1
  assert len(calls) == 1

  assert cache.refresh(0.5) == 2
  assert cache.next_refresh_time == 1.0
  assert len(calls) == 2


def test_failed_refresh_keeps_snapshot_and_retries_next_frame():
  calls = 0

  def load():
    nonlocal calls
    calls += 1
    if calls == 1:
      raise RuntimeError("temporary read failure")
    return "complete"

  cache = TimedSnapshotCache("previous", load)

  assert cache.refresh(0.0) == "previous"
  assert cache.next_refresh_time == pytest.approx(0.05)
  assert cache.refresh(0.049) == "previous"
  assert cache.refresh(0.1) == "complete"
  assert cache.next_refresh_time == pytest.approx(0.6)


def test_border_snapshot_is_atomic_when_middle_read_fails():
  values = {
    "CarName": "MX5",
    "HyundaiCameraSCC": 1,
    "NNFFModelName": "nnff",
    "CustomSR": 180,
    "GitBranch": "carrot-ms-ui2",
    "NetworkAddress": "192.168.0.2",
  }
  params = FakeParams(values, failures={"CustomSR": 1})
  params_memory = FakeParams(values)
  previous = BorderParamSnapshot(top_left="previous", custom_sr=12.3)
  cache = TimedSnapshotCache(previous, lambda: read_border_params(params, params_memory))

  assert cache.refresh(0.0) is previous
  assert params.calls == ["CarName", "HyundaiCameraSCC", "NNFFModelName", "CustomSR"]
  assert params_memory.calls == []

  snapshot = cache.refresh(0.1)
  assert snapshot == BorderParamSnapshot(
    top_left="MX5(CAMERA SCC),NNFF",
    top_left_op_long="MX5(CAMERA SCC),NNFF",
    custom_sr=18.0,
    bottom_left="carrot-ms-ui2",
    bottom_right="192.168.0.2",
  )


def test_border_snapshot_prepares_op_long_text_without_changing_order():
  values = {
    "CarName": "MX5",
    "HyundaiCameraSCC": 0,
    "NNFFModelName": "nnff",
    "CustomSR": 180,
    "GitBranch": "carrot-ms-ui2",
    "NetworkAddress": "",
  }

  snapshot = read_border_params(FakeParams(values), FakeParams(values))

  assert snapshot.top_left == "MX5,NNFF"
  assert snapshot.top_left_op_long == "MX5 - OP Long,NNFF"
  assert snapshot.bottom_left == "carrot-ms-ui2"


def test_realtime_ui_snapshot_reads_all_values_once():
  params = FakeParams({"RecordAudio": 1, "IsMetric": 1, "AlwaysOnDM": 0})

  assert read_realtime_ui_params(params) == RealtimeUiParamSnapshot(
    record_audio=True,
    is_metric=True,
    always_on_dm=False,
  )
  assert params.calls == ["RecordAudio", "IsMetric", "AlwaysOnDM"]


def test_pending_store_rejects_stale_screen_record_until_acknowledged():
  params = FakeParams({"ScreenRecord": False})
  cache = TimedSnapshotCache(False, lambda: read_screen_record(params))

  cache.store_pending(True, 3.0)
  assert cache.refresh(3.499) is True
  assert params.calls == []
  assert cache.refresh(3.5) is True
  assert params.calls == ["ScreenRecord"]

  params.values["ScreenRecord"] = True
  assert cache.refresh(4.0) is True
  params.values["ScreenRecord"] = False
  assert cache.refresh(4.5) is False


def test_pending_store_expires_if_acknowledgement_is_never_observed():
  params = FakeParams({"ScreenRecord": False})
  cache = TimedSnapshotCache(False, lambda: read_screen_record(params))

  cache.store_pending(True, 3.0)
  assert cache.refresh(3.5) is True
  assert cache.refresh(4.0) is False


def test_persistent_failures_back_off_to_normal_interval():
  calls = []

  def fail():
    calls.append(True)
    raise RuntimeError("Params unavailable")

  cache = TimedSnapshotCache("previous", fail)
  retry_delays = (0.05, 0.1, 0.2, 0.4, 0.5, 0.5)
  now = 0.0
  for retry_delay in retry_delays:
    assert cache.refresh(now) == "previous"
    deadline = now + retry_delay
    assert cache.next_refresh_time == pytest.approx(deadline)
    now = deadline + 1e-6

  assert len(calls) == len(retry_delays)


def test_delayed_screen_record_write_cannot_reverse_start(main_layout_module, monkeypatch):
  module, gui_app, ui_state = main_layout_module
  params = FakeParams({"ScreenRecord": False})
  layout = make_main_layout(module, gui_app, ui_state, params)
  now = 0.0
  monkeypatch.setattr(module.time, "monotonic", lambda: now)

  assert layout._handle_carrot_record_cmd(carrot_command(1, "START")) is True
  assert params.writes == [("ScreenRecord", True)]

  # The nonblocking write is still stale at the first refresh boundary.
  now = 0.5
  assert layout._handle_carrot_record_cmd(carrot_command(1, "START")) is True
  assert gui_app.is_recording() is True
  assert params.writes == [("ScreenRecord", True)]

  # Observe the acknowledgement, then accept a later external stop request.
  params.values["ScreenRecord"] = True
  now = 1.0
  assert layout._handle_carrot_record_cmd(carrot_command(1, "START")) is True
  params.values["ScreenRecord"] = False
  now = 1.5
  assert layout._handle_carrot_record_cmd(carrot_command(1, "START")) is False


def test_missing_screen_record_ack_is_bounded(main_layout_module, monkeypatch):
  module, gui_app, ui_state = main_layout_module
  params = FakeParams({"ScreenRecord": False})
  layout = make_main_layout(module, gui_app, ui_state, params)
  now = 0.0
  monkeypatch.setattr(module.time, "monotonic", lambda: now)

  assert layout._handle_carrot_record_cmd(carrot_command(1, "START")) is True
  now = 0.5
  assert layout._handle_carrot_record_cmd(carrot_command(1, "START")) is True
  now = 1.0
  assert layout._handle_carrot_record_cmd(carrot_command(1, "START")) is False


@pytest.mark.parametrize(
  ("recording", "started", "arg", "expected"),
  (
    (True, True, "STOP", False),
    (False, True, "TOGGLE", True),
    (True, False, "START", False),
  ),
)
def test_record_commands_and_offroad_clear_sync_pending_value(
  main_layout_module, monkeypatch, recording, started, arg, expected,
):
  module, gui_app, ui_state = main_layout_module
  gui_app.recording = recording
  ui_state.started = started
  params = FakeParams({"ScreenRecord": recording})
  layout = make_main_layout(module, gui_app, ui_state, params)
  monkeypatch.setattr(module.time, "monotonic", lambda: 2.0)

  assert layout._handle_carrot_record_cmd(carrot_command(1, arg)) is expected
  assert gui_app.is_recording() is expected
  assert params.writes == [("ScreenRecord", expected)]
  if not started:
    assert "start" not in gui_app.calls

  # A stale file value at the next boundary must not undo the local command.
  assert layout._screen_record_param.refresh(2.5) is expected


def test_targeted_param_reads_are_isolated_from_per_frame_paths():
  targets = {
    "CarName",
    "HyundaiCameraSCC",
    "NNFFModelName",
    "CustomSR",
    "GitBranch",
    "NetworkAddress",
    "RecordAudio",
    "IsMetric",
    "AlwaysOnDM",
    "ScreenRecord",
  }
  production_files = (
    UI_DIR / "onroad" / "augmented_road_view.py",
    UI_DIR / "ui_state.py",
    UI_DIR / "layouts" / "main.py",
  )
  offenders = []

  for path in production_files:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for call in (node for node in ast.walk(tree) if isinstance(node, ast.Call)):
      if not isinstance(call.func, ast.Attribute) or call.func.attr not in {"get", "get_bool", "get_float", "get_int"}:
        continue
      if not call.args or not isinstance(call.args[0], ast.Constant) or call.args[0].value not in targets:
        continue
      offenders.append((path.name, call.lineno, call.args[0].value))

  assert offenders == []
