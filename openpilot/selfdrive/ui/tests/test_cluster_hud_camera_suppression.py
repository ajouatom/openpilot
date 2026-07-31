import ast
from pathlib import Path


UI_DIR = Path(__file__).parents[1]
MAIN_LAYOUT_PATH = UI_DIR / "layouts" / "main.py"
ROAD_VIEW_PATH = UI_DIR / "onroad" / "augmented_road_view.py"
MICI_MAIN_LAYOUT_PATH = UI_DIR / "mici" / "layouts" / "main.py"
MICI_ROAD_VIEW_PATH = UI_DIR / "mici" / "onroad" / "augmented_road_view.py"
UI_STATE_PATH = UI_DIR / "ui_state.py"


def _method(path: Path, class_name: str, method_name: str) -> ast.FunctionDef:
  tree = ast.parse(path.read_text(encoding="utf-8"))
  cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == class_name)
  return next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == method_name)


def _is_cluster_camera_suppressed(node: ast.expr, *, negated: bool) -> bool:
  if negated:
    return (
      isinstance(node, ast.UnaryOp)
      and isinstance(node.op, ast.Not)
      and _is_cluster_camera_suppressed(node.operand, negated=False)
    )
  return (
    isinstance(node, ast.Attribute)
    and isinstance(node.value, ast.Name)
    and node.value.id == "self"
    and node.attr == "_suppress_camera_for_cluster"
  )


def _calls(node: ast.AST, attr: str) -> bool:
  return any(
    isinstance(child, ast.Call)
    and isinstance(child.func, ast.Attribute)
    and child.func.attr == attr
    for child in ast.walk(node)
  )


def _call(node: ast.AST, attr: str) -> ast.Call:
  return next(
    child for child in ast.walk(node)
    if isinstance(child, ast.Call)
    and isinstance(child.func, ast.Attribute)
    and child.func.attr == attr
  )


def _is_attr(node: ast.expr, owner: str, attr: str) -> bool:
  return (
    isinstance(node, ast.Attribute)
    and isinstance(node.value, ast.Name)
    and node.value.id == owner
    and node.attr == attr
  )


def _assert_main_layout_forwards_camera_preference(path: Path, class_name: str, method_name: str) -> None:
  render = _method(path, class_name, method_name)
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
  setter = _call(guard, "set_cluster_hud_connected")
  assert len(setter.args) == 2
  assert _is_attr(setter.args[1], "ui_state", "show_camera_with_cluster")


def test_main_layouts_forward_external_hud_connection_and_camera_preference():
  _assert_main_layout_forwards_camera_preference(MAIN_LAYOUT_PATH, "MainLayout", "_render")
  _assert_main_layout_forwards_camera_preference(MICI_MAIN_LAYOUT_PATH, "MiciMainLayout", "_handle_transitions")


def test_ui_state_caches_camera_preference_as_exact_enabled_value():
  update_params = _method(UI_STATE_PATH, "UIState", "update_params")
  assignment = next(
    node for node in ast.walk(update_params)
    if isinstance(node, ast.Assign)
    and any(_is_attr(target, "self", "show_camera_with_cluster") for target in node.targets)
  )
  assert isinstance(assignment.value, ast.Compare)
  assert isinstance(assignment.value.left, ast.Call)
  assert isinstance(assignment.value.left.func, ast.Attribute)
  assert assignment.value.left.func.attr == "get_int"
  assert assignment.value.left.args[0].value == "ShowCameraWithCluster"
  assert isinstance(assignment.value.ops[0], ast.Eq)
  assert assignment.value.comparators[0].value == 1


def test_camera_preference_only_bypasses_connected_render_suppression():
  for path in (ROAD_VIEW_PATH, MICI_ROAD_VIEW_PATH):
    setter = _method(path, "AugmentedRoadView", "set_cluster_hud_connected")
    assignment = next(
      node for node in setter.body
      if isinstance(node, ast.Assign)
      and any(_is_attr(target, "self", "_suppress_camera_for_cluster") for target in node.targets)
    )
    assert isinstance(assignment.value, ast.BoolOp)
    assert isinstance(assignment.value.op, ast.And)
    assert isinstance(assignment.value.values[0], ast.Name)
    assert assignment.value.values[0].id == "connected"
    assert isinstance(assignment.value.values[1], ast.UnaryOp)
    assert isinstance(assignment.value.values[1].op, ast.Not)
    assert isinstance(assignment.value.values[1].operand, ast.Name)
    assert assignment.value.values[1].operand.id == "show_camera"


def test_external_hud_skips_camera_and_model_but_retains_device_hud():
  render = _method(ROAD_VIEW_PATH, "AugmentedRoadView", "_render")
  connected = [
    node for node in ast.walk(render)
    if isinstance(node, ast.If) and _is_cluster_camera_suppressed(node.test, negated=False)
  ]
  disconnected = [
    node for node in ast.walk(render)
    if isinstance(node, ast.If) and _is_cluster_camera_suppressed(node.test, negated=True)
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
