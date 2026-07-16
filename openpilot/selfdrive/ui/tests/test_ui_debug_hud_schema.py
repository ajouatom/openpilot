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
  ("modelPathTimeMillis", 9, "Float32"),
  ("modelLaneTimeMillis", 10, "Float32"),
  ("modelBlindSpotTimeMillis", 11, "Float32"),
  ("modelRadarTimeMillis", 12, "Float32"),
  ("modelTimingValid", 13, "Bool"),
  ("hudHeaderTimeMillis", 14, "Float32"),
  ("hudSpeedTimeMillis", 15, "Float32"),
  ("hudStatusTimeMillis", 16, "Float32"),
  ("hudNavigationTimeMillis", 17, "Float32"),
  ("hudButtonTimeMillis", 18, "Float32"),
  ("hudPlotTimeMillis", 19, "Float32"),
  ("hudAuxTimeMillis", 20, "Float32"),
  ("hudTimingValid", 21, "Bool"),
  ("modelBlindSpotStateMask", 22, "UInt8"),
  ("modelBlindSpotStateValid", 23, "Bool"),
]

EXPECTED_HUD_PUBLISH_MAPPING = {
  "hudHeaderTimeMillis": "header_time_millis",
  "hudSpeedTimeMillis": "speed_time_millis",
  "hudStatusTimeMillis": "status_time_millis",
  "hudNavigationTimeMillis": "navigation_time_millis",
  "hudButtonTimeMillis": "button_time_millis",
  "hudPlotTimeMillis": "plot_time_millis",
  "hudAuxTimeMillis": "aux_time_millis",
  "hudTimingValid": "valid",
}

EXPECTED_MODEL_PUBLISH_MAPPING = {
  "modelPathTimeMillis": "path_time_millis",
  "modelLaneTimeMillis": "lane_time_millis",
  "modelBlindSpotTimeMillis": "blind_spot_time_millis",
  "modelRadarTimeMillis": "radar_time_millis",
  "modelTimingValid": "valid",
  "modelBlindSpotStateMask": "blind_spot_state_mask",
  "modelBlindSpotStateValid": "blind_spot_state_valid",
}


def test_ui_debug_schema_keeps_legacy_ordinals_and_appends_hud_fields():
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


def test_augmented_road_view_publishes_hud_snapshot_after_render():
  tree = ast.parse(AUGMENTED_ROAD_VIEW_PATH.read_text(encoding="utf-8"))
  mappings = {}
  mapping_lines = []
  render_lines = []
  snapshot_lines = []
  send_lines = []

  for node in ast.walk(tree):
    if isinstance(node, ast.Assign) and len(node.targets) == 1:
      target = node.targets[0]
      value = node.value
      if (
        isinstance(target, ast.Attribute)
        and isinstance(target.value, ast.Name)
        and target.value.id == "ud"
        and isinstance(value, ast.Attribute)
        and isinstance(value.value, ast.Name)
        and value.value.id == "hud_timings"
      ):
        assert target.attr not in mappings, f"duplicate publish assignment for {target.attr}"
        mappings[target.attr] = value.attr
        mapping_lines.append(node.lineno)
      if (
        isinstance(target, ast.Name)
        and target.id == "hud_timings"
        and _is_attribute(value, "self", "_hud_renderer", "render_timings")
      ):
        snapshot_lines.append(node.lineno)

    if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
      if _is_attribute(node.func, "self", "_hud_renderer", "render"):
        render_lines.append(node.lineno)
      if _is_attribute(node.func, "self", "_pm", "send"):
        send_lines.append(node.lineno)

  assert mappings == EXPECTED_HUD_PUBLISH_MAPPING
  assert len(snapshot_lines) == 1
  assert render_lines and send_lines
  assert render_lines[-1] < snapshot_lines[0] < min(mapping_lines) <= max(mapping_lines) < send_lines[-1]


def test_augmented_road_view_publishes_model_snapshot_after_render():
  tree = ast.parse(AUGMENTED_ROAD_VIEW_PATH.read_text(encoding="utf-8"))
  mappings = {}
  mapping_lines = []
  render_lines = []
  snapshot_lines = []
  send_lines = []

  for node in ast.walk(tree):
    if isinstance(node, ast.Assign) and len(node.targets) == 1:
      target = node.targets[0]
      value = node.value
      if (
        isinstance(target, ast.Attribute)
        and isinstance(target.value, ast.Name)
        and target.value.id == "ud"
        and isinstance(value, ast.Attribute)
        and isinstance(value.value, ast.Name)
        and value.value.id == "model_timings"
      ):
        assert target.attr not in mappings, f"duplicate publish assignment for {target.attr}"
        mappings[target.attr] = value.attr
        mapping_lines.append(node.lineno)
      if (
        isinstance(target, ast.Name)
        and target.id == "model_timings"
        and _is_attribute(value, "self", "model_renderer", "render_timings")
      ):
        snapshot_lines.append(node.lineno)

    if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
      if _is_attribute(node.func, "self", "model_renderer", "render"):
        render_lines.append(node.lineno)
      if _is_attribute(node.func, "self", "_pm", "send"):
        send_lines.append(node.lineno)

  assert mappings == EXPECTED_MODEL_PUBLISH_MAPPING
  assert len(snapshot_lines) == 1
  assert render_lines and send_lines
  assert render_lines[-1] < snapshot_lines[0] < min(mapping_lines) <= max(mapping_lines) < send_lines[-1]
