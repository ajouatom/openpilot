import ast
from datetime import UTC, datetime
from pathlib import Path

from openpilot.selfdrive.carrot.tmux_metadata import TmuxCaptureStore, capture_tmux_metadata


CARROT_MAN = Path(__file__).resolve().parents[1] / "carrot_man.py"


class FakeParams:
  def __init__(self, values):
    self.values = values

  def get(self, key):
    return self.values.get(key)


def test_capture_tmux_metadata_uses_logger_route_directory_and_utc_time(tmp_path):
  route_id = "00000042--abcdefghij"
  (tmp_path / f"{route_id}--0").mkdir()
  (tmp_path / f"{route_id}--3").mkdir()
  (tmp_path / "00000041--otherroute--9").mkdir()
  params = FakeParams({
    "CurrentRoute": route_id.encode(),
    "DongleId": "61d2c91e1039ab5e",
  })
  now = datetime(2026, 7, 15, 3, 10, 0, 123456, tzinfo=UTC)

  assert capture_tmux_metadata(params, log_root=tmp_path, now=now) == {
    "route": "61d2c91e1039ab5e|00000042--abcdefghij",
    "segment": "3",
    "captured_at": "2026-07-15T03:10:00.123456Z",
  }


def test_capture_tmux_metadata_ignores_non_directory_and_non_numeric_segments(tmp_path):
  route_id = "00000042--abcdefghij"
  (tmp_path / f"{route_id}--5").write_text("not a directory")
  (tmp_path / f"{route_id}--not-a-segment").mkdir()
  params = FakeParams({"CurrentRoute": route_id, "DongleId": "dongle"})

  result = capture_tmux_metadata(
    params,
    log_root=tmp_path,
    now=datetime(2026, 7, 15, tzinfo=UTC),
  )
  assert result["segment"] is None


def test_capture_tmux_metadata_does_not_duplicate_canonical_dongle(tmp_path):
  route_id = "61d2c91e1039ab5e|2026-07-15--03-10-00"
  params = FakeParams({"CurrentRoute": route_id, "DongleId": "61d2c91e1039ab5e"})
  result = capture_tmux_metadata(
    params,
    log_root=tmp_path,
    now=datetime(2026, 7, 15, tzinfo=UTC),
  )
  assert result["route"] == route_id
  assert result["segment"] is None


def test_capture_tmux_metadata_is_best_effort_when_route_or_log_root_missing(tmp_path):
  params = FakeParams({"CurrentRoute": None, "DongleId": b"dongle"})
  result = capture_tmux_metadata(
    params,
    log_root=tmp_path / "missing",
    now=datetime(2026, 7, 15, tzinfo=UTC),
  )
  assert result == {
    "route": None,
    "segment": None,
    "captured_at": "2026-07-15T00:00:00Z",
  }


def test_capture_store_keeps_overlapping_retries_isolated():
  store = TmuxCaptureStore()
  store.put("onroad", "/tmp/onroad.log", {"captured_at": "first", "segment": "1"})
  store.put("exception", "/tmp/exception.log", {"captured_at": "second", "segment": "2"})

  assert store.get("onroad").path == "/tmp/onroad.log"
  assert store.get("onroad").metadata["captured_at"] == "first"
  assert store.get("exception").metadata["captured_at"] == "second"
  assert store.pop("onroad").metadata["segment"] == "1"
  assert store.get("onroad") is None
  assert store.get("exception").metadata["segment"] == "2"


def test_carrot_man_wires_keyed_capture_into_tmux_payload():
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
  assert "roadEncodeIdx" not in init_literals

  make_args = [arg.arg for arg in functions["make_tmux_data"].args.args]
  assert "capture_key" in make_args

  make_attributes = {
    node.attr
    for node in ast.walk(functions["make_tmux_data"])
    if isinstance(node, ast.Attribute)
  }
  send_attributes = {
    node.attr
    for node in ast.walk(functions["send_tmux_http"])
    if isinstance(node, ast.Attribute)
  }
  assert "tmux_captures" in make_attributes
  assert "tmux_captures" in send_attributes

  dict_keys = {
    key.value
    for node in ast.walk(functions["send_tmux_http"])
    if isinstance(node, ast.Dict)
    for key in node.keys
    if isinstance(key, ast.Constant) and isinstance(key.value, str)
  }
  assert {"route", "segment", "captured_at"} <= dict_keys
