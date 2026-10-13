"""표시 전용 점의 좌우 보정은 제어 목록과 분리한 채 한 번만 적용한다."""
from copy import deepcopy
import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from opendbc.car import structs
from openpilot.cereal import log
from openpilot.selfdrive.carrot.radar.lateral import set_radar_track_flip


@pytest.mark.parametrize("control_source", ["frontRadar", "scc", "corner235", "corner180", "corner430"])
def test_display_orientation_preserves_control_membership_and_publication_copy(control_source):
  raw = structs.RadarData(
    hasDisplayData=True,
    points=[{"trackId": 1000, "radarSource": control_source, "dRel": 12., "yRel": 1., "yvRel": .2, "measured": True}],
    displayPoints=[{"trackId": 1001, "radarSource": "frontRadar", "dRel": 25., "yRel": 3.5, "yvRel": -.3, "measured": True}],
  )
  original = raw.to_dict()
  published = raw.as_reader().as_builder()
  set_radar_track_flip(published, True)
  set_radar_track_flip(published, True)
  expected = deepcopy(original)
  expected["radarTrackFlipped"] = True
  for point in expected["points"] + expected["displayPoints"]:
    if point["radarSource"] == "frontRadar":
      point["yRel"] *= -1
      point["yvRel"] *= -1
  assert published.to_dict() == expected
  assert raw.to_dict() == original
  # 실제 wire 왕복 뒤에도 반전 상태를 유지하고 원래 좌표로 되돌릴 수 있다.
  with structs.RadarData.from_bytes(published.to_bytes()) as decoded:
    restored = decoded.as_builder()
  set_radar_track_flip(restored, False)
  assert restored.to_dict() == original


def test_legacy_object_without_display_field_retains_orientation_behavior():
  point = SimpleNamespace(radarSource="frontRadar", yRel=2., yvRel=.1)
  data = SimpleNamespace(points=[point], radarTrackFlipped=False)
  set_radar_track_flip(data, True)
  set_radar_track_flip(data, True)
  assert point.yRel == -2. and point.yvRel == -.1


@pytest.mark.parametrize("capability, supported", [(True, True), (False, False), (None, False), (1, False)])
def test_host_input_error_preserves_display_capability_and_empty_payload(capability, supported):
  # 실제 worker의 오류 발행 함수만 원형 클로저에서 실행한다. 소켓·CAN은 만들지 않는다.
  path = Path(__file__).resolve().parents[4] / "openpilot/selfdrive/carrot/radar/radarcan.py"
  tree = ast.parse(path.read_text(encoding="utf-8"))
  main = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "main")
  publish = next(node for node in main.body if isinstance(node, ast.FunctionDef) and node.name == "publish_error")
  wrapper = ast.parse("def setup():\n  needs_reset = False\n  last_error_publish_ns = 0\n").body[0]
  wrapper.body += [publish, ast.Return(value=ast.Name(id="publish_error", ctx=ast.Load()))]
  sent = []

  def new_message(service):
    message = log.Event.new_message()
    message.init(service)
    return message

  namespace = {"radar": SimpleNamespace(DISPLAY_POINTS_SUPPORTED=capability), "radar_track_flip": True,
               "messaging": SimpleNamespace(new_message=new_message), "cloudlog": SimpleNamespace(error=lambda _: None),
               "pm": SimpleNamespace(send=lambda service, message: sent.append((service, message.to_dict())))}
  exec(compile(ast.fix_missing_locations(ast.Module(body=[wrapper], type_ignores=[])), str(path), "exec"), namespace)
  callback = namespace["setup"]()
  callback("inputTimeout", 1_000_000_000)
  callback("inputTimeout", 1_010_000_000)
  assert len(sent) == 1
  service, message = sent[0]
  assert service == "liveTracks" and message["valid"] is False
  data = message["liveTracks"]
  assert data["hasDisplayData"] is supported and data["radarTrackFlipped"] is True
  assert data["errors"]["canError"] is True
  assert not data.get("points") and not data.get("displayPoints")
