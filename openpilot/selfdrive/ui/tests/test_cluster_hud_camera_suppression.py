import ast
from pathlib import Path


UI_DIR = Path(__file__).parents[1]
MAIN_LAYOUT_PATH = UI_DIR / "layouts" / "main.py"
ROAD_VIEW_PATH = UI_DIR / "onroad" / "augmented_road_view.py"


def _method(path: Path, class_name: str, method_name: str) -> ast.FunctionDef:
  tree = ast.parse(path.read_text(encoding="utf-8"))
  cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == class_name)
  return next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == method_name)


def _is_cluster_connected(node: ast.expr, *, negated: bool) -> bool:
  if negated:
    return (
      isinstance(node, ast.UnaryOp)
      and isinstance(node.op, ast.Not)
      and _is_cluster_connected(node.operand, negated=False)
    )
  return (
    isinstance(node, ast.Attribute)
    and isinstance(node.value, ast.Name)
    and node.value.id == "self"
    and node.attr == "_cluster_hud_connected"
  )


def _calls(node: ast.AST, attr: str) -> bool:
  return any(
    isinstance(child, ast.Call)
    and isinstance(child.func, ast.Attribute)
    and child.func.attr == attr
    for child in ast.walk(node)
  )


def test_main_layout_forwards_external_hud_connection_to_road_view():
  render = _method(MAIN_LAYOUT_PATH, "MainLayout", "_render")
  connected_guards = [
    node for node in ast.walk(render)
    if isinstance(node, ast.If)
    and isinstance(node.test, ast.Attribute)
    and isinstance(node.test.value, ast.Name)
    and node.test.value.id == "ui_state"
    and node.test.attr == "started"
  ]
  assert connected_guards
  guard = connected_guards[0]
  assert _calls(guard, "get_bool")
  assert _calls(guard, "set_cluster_hud_connected")


def test_external_hud_skips_camera_and_model_but_retains_device_hud():
  render = _method(ROAD_VIEW_PATH, "AugmentedRoadView", "_render")
  connected = [
    node for node in ast.walk(render)
    if isinstance(node, ast.If) and _is_cluster_connected(node.test, negated=False)
  ]
  disconnected = [
    node for node in ast.walk(render)
    if isinstance(node, ast.If) and _is_cluster_connected(node.test, negated=True)
  ]

  assert any(_calls(node, "draw_rectangle_rec") and _calls(ast.Module(body=node.orelse), "_render")
             for node in connected)
  assert any(_calls(node, "_switch_stream_if_needed") and _calls(node, "_update_calibration")
             for node in disconnected)
  assert any(_calls(node, "render") for node in disconnected)

  guarded_nodes = {id(child) for guard in connected + disconnected for child in ast.walk(guard)}
  retained_renderers = []
  for node in ast.walk(render):
    if not (
      isinstance(node, ast.Call)
      and isinstance(node.func, ast.Attribute)
      and node.func.attr == "render"
      and isinstance(node.func.value, ast.Attribute)
      and isinstance(node.func.value.value, ast.Name)
      and node.func.value.value.id == "self"
    ):
      continue
    if id(node) not in guarded_nodes:
      retained_renderers.append(node.func.value.attr)

  assert {"_hud_renderer", "alert_renderer", "driver_state_renderer"} <= set(retained_renderers)
