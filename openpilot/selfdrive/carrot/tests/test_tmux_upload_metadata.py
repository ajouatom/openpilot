import ast
from datetime import UTC, datetime
from pathlib import Path
from types import SimpleNamespace

from openpilot.selfdrive.carrot.tmux_metadata import capture_tmux_metadata


CARROT_MAN = Path(__file__).resolve().parents[1] / "carrot_man.py"


class FakeParams:
  def __init__(self, values):
    self.values = values

  def get(self, key):
    return self.values.get(key)


class FakeSubMaster:
  def __init__(self, *, segment_num=0, seen=True, valid=True):
    self.seen = {"roadEncodeIdx": seen}
    self.valid = {"roadEncodeIdx": valid}
    self.message = SimpleNamespace(segmentNum=segment_num)

  def __getitem__(self, service):
    assert service == "roadEncodeIdx"
    return self.message


def test_capture_tmux_metadata_uses_current_route_segment_and_utc_time():
  params = FakeParams({
    "CurrentRoute": b"00000042--abcdefghij",
    "DongleId": "61d2c91e1039ab5e",
  })
  sm = FakeSubMaster(segment_num=3)
  now = datetime(2026, 7, 15, 3, 10, 0, 123456, tzinfo=UTC)

  assert capture_tmux_metadata(params, sm, now=now) == {
    "route": "61d2c91e1039ab5e|00000042--abcdefghij",
    "segment": "3",
    "captured_at": "2026-07-15T03:10:00.123456Z",
  }


def test_capture_tmux_metadata_does_not_duplicate_canonical_dongle():
  params = FakeParams({
    "CurrentRoute": "61d2c91e1039ab5e|2026-07-15--03-10-00",
    "DongleId": "61d2c91e1039ab5e",
  })
  result = capture_tmux_metadata(
    params,
    FakeSubMaster(segment_num=9),
    now=datetime(2026, 7, 15, tzinfo=UTC),
  )
  assert result["route"] == "61d2c91e1039ab5e|2026-07-15--03-10-00"
  assert result["segment"] == "9"


def test_capture_tmux_metadata_is_best_effort_when_route_or_encoder_missing():
  params = FakeParams({"CurrentRoute": None, "DongleId": b"dongle"})
  result = capture_tmux_metadata(
    params,
    FakeSubMaster(segment_num=-1, seen=False, valid=False),
    now=datetime(2026, 7, 15, tzinfo=UTC),
  )
  assert result == {
    "route": None,
    "segment": None,
    "captured_at": "2026-07-15T00:00:00Z",
  }


def test_carrot_man_wires_capture_snapshot_into_tmux_payload():
  tree = ast.parse(CARROT_MAN.read_text())
  carrot_class = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "CarrotMan")
  functions = {
    node.name: node
    for node in carrot_class.body
    if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
  }

  init_literals = {
    node.value
    for node in ast.walk(functions["__init__"])
    if isinstance(node, ast.Constant) and isinstance(node.value, str)
  }
  assert "roadEncodeIdx" in init_literals

  make_calls = {
    node.func.id
    for node in ast.walk(functions["make_tmux_data"])
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
  }
  assert "capture_tmux_metadata" in make_calls

  send_calls = {
    node.func.id
    for node in ast.walk(functions["send_tmux_http"])
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
  }
  assert "capture_tmux_metadata" in send_calls
  dict_keys = {
    key.value
    for node in ast.walk(functions["send_tmux_http"])
    if isinstance(node, ast.Dict)
    for key in node.keys
    if isinstance(key, ast.Constant) and isinstance(key.value, str)
  }
  assert {"route", "segment", "captured_at"} <= dict_keys
