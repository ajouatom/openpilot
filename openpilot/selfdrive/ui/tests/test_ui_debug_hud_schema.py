import ast
import re
from pathlib import Path


OPENPILOT_DIR = Path(__file__).parents[3]
LOG_PATH = OPENPILOT_DIR / "cereal" / "log.capnp"
AUGMENTED_ROAD_VIEW_PATH = Path(__file__).parents[1] / "onroad" / "augmented_road_view.py"


EXPECTED_UI_DEBUG_FIELDS = [
  ("drawTimeMillis", 0, "Float32"),
  ("cameraTimeMillis", 1, "Float32"),
  ("modelTimeMillis", 2, "Float32"),
  ("driverStateTimeMillis", 3, "Float32"),
  ("hudTimeMillis", 4, "Float32"),
  ("alertTimeMillis", 5, "Float32"),
  ("extrasTimeMillis", 6, "Float32"),
  ("plotMode", 7, "UInt8"),
  ("recording", 8, "Bool"),
  ("modelPathTimeMillisDEPRECATED", 9, "Float32"),
  ("modelLaneTimeMillisDEPRECATED", 10, "Float32"),
  ("modelBlindSpotTimeMillisDEPRECATED", 11, "Float32"),
  ("modelRadarTimeMillisDEPRECATED", 12, "Float32"),
  ("modelTimingValidDEPRECATED", 13, "Bool"),
  ("hudHeaderTimeMillisDEPRECATED", 14, "Float32"),
  ("hudSpeedTimeMillisDEPRECATED", 15, "Float32"),
  ("hudStatusTimeMillisDEPRECATED", 16, "Float32"),
  ("hudNavigationTimeMillisDEPRECATED", 17, "Float32"),
  ("hudButtonTimeMillisDEPRECATED", 18, "Float32"),
  ("hudPlotTimeMillisDEPRECATED", 19, "Float32"),
  ("hudAuxTimeMillisDEPRECATED", 20, "Float32"),
  ("hudTimingValidDEPRECATED", 21, "Bool"),
  ("modelBlindSpotStateMaskDEPRECATED", 22, "UInt8"),
  ("modelBlindSpotStateValidDEPRECATED", 23, "Bool"),
]

EXPECTED_UI_DEBUG_PUBLISH_FIELDS = {name for name, ordinal, _ in EXPECTED_UI_DEBUG_FIELDS if ordinal <= 8}


def test_ui_debug_schema_reserves_detailed_profiling_ordinals():
  source = LOG_PATH.read_text(encoding="utf-8")
  body = source.split("struct UIDebug {", 1)[1].split("}", 1)[0]
  fields = [
    (name, int(ordinal), field_type)
    for name, ordinal, field_type in re.findall(
      r"^\s*(\w+)\s+@(\d+)\s*:(\w+)", body, flags=re.MULTILINE,
    )
  ]

  assert fields == EXPECTED_UI_DEBUG_FIELDS
  assert len({ordinal for _, ordinal, _ in fields}) == len(fields)
  assert [ordinal for _, ordinal, _ in fields] == list(range(24))


def _is_attribute(node, *names):
  for name in reversed(names[1:]):
    if not isinstance(node, ast.Attribute) or node.attr != name:
      return False
    node = node.value
  return isinstance(node, ast.Name) and node.id == names[0]


def test_augmented_road_view_publishes_compact_snapshot_after_render():
  tree = ast.parse(AUGMENTED_ROAD_VIEW_PATH.read_text(encoding="utf-8"))
  publish_fields = set()
  publish_lines = []
  render_lines = []
  new_message_lines = []
  send_lines = []

  for node in ast.walk(tree):
    if isinstance(node, ast.Assign) and len(node.targets) == 1:
      target = node.targets[0]
      if (
        isinstance(target, ast.Attribute)
        and isinstance(target.value, ast.Name)
        and target.value.id == "ud"
      ):
        assert target.attr not in publish_fields, f"duplicate publish assignment for {target.attr}"
        publish_fields.add(target.attr)
        publish_lines.append(node.lineno)

    if isinstance(node, ast.Call):
      if (
        isinstance(node.func, ast.Attribute)
        and (
          _is_attribute(node.func, "self", "model_renderer", "render")
          or _is_attribute(node.func, "self", "_hud_renderer", "render")
          or _is_attribute(node.func, "self", "alert_renderer", "render")
          or _is_attribute(node.func, "self", "driver_state_renderer", "render")
        )
      ):
        render_lines.append(node.lineno)
      if isinstance(node.func, ast.Attribute) and _is_attribute(node.func, "self", "_pm", "send"):
        send_lines.append(node.lineno)
      if (
        isinstance(node.func, ast.Attribute)
        and _is_attribute(node.func, "messaging", "new_message")
        and node.args
        and isinstance(node.args[0], ast.Constant)
        and node.args[0].value == "uiDebug"
      ):
        valid_keywords = [kw.value for kw in node.keywords if kw.arg == "valid"]
        assert len(valid_keywords) == 1
        assert isinstance(valid_keywords[0], ast.Constant) and valid_keywords[0].value is True
        new_message_lines.append(node.lineno)

  assert publish_fields == EXPECTED_UI_DEBUG_PUBLISH_FIELDS
  assert len(new_message_lines) == 1
  assert render_lines and send_lines
  assert render_lines[-1] < new_message_lines[0] < min(publish_lines) <= max(publish_lines) < send_lines[-1]
