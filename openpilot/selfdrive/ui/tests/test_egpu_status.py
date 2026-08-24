import ast
from pathlib import Path


UI_DIR = Path(__file__).resolve().parents[1]
HUD_PATHS = (
  UI_DIR / "onroad" / "hud_renderer.py",
  UI_DIR / "mici" / "onroad" / "hud_renderer.py",
)


def _method(path: Path, class_name: str, method_name: str) -> ast.FunctionDef:
  tree = ast.parse(path.read_text(encoding="utf-8"))
  cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == class_name)
  return next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == method_name)


def test_both_device_huds_render_active_egpu_badge():
  for path in HUD_PATHS:
    render = _method(path, "HudRenderer", "_render")
    calls = [
      node for node in ast.walk(render)
      if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    ]
    assert any(call.func.attr == "_draw_egpu_badge" for call in calls)

    badge = _method(path, "HudRenderer", "_draw_egpu_badge")
    badge_source = ast.unparse(badge)
    assert "badge_w" in badge_source
    assert "rect.width / 2" not in badge_source
    assert any(
      isinstance(node, ast.Attribute)
      and isinstance(node.value, ast.Name)
      and node.value.id == "ui_state"
      and node.attr == "usbgpu_active"
      for node in ast.walk(badge)
    )


def test_ui_state_reads_modeld_egpu_active_param():
  update_params = _method(UI_DIR / "ui_state.py", "UIState", "update_params")
  assert any(
    isinstance(node, ast.Call)
    and isinstance(node.func, ast.Attribute)
    and node.func.attr == "get_bool"
    and node.args
    and isinstance(node.args[0], ast.Constant)
    and node.args[0].value == "UsbGpuActive"
    for node in ast.walk(update_params)
  )
