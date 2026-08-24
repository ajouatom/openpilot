import ast
from pathlib import Path


MICI_HUD_PATH = Path(__file__).resolve().parents[1] / "mici" / "onroad" / "hud_renderer.py"


def _draw_set_speed_source() -> str:
  tree = ast.parse(MICI_HUD_PATH.read_text(encoding="utf-8"))
  hud = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "HudRenderer")
  method = next(node for node in hud.body if isinstance(node, ast.FunctionDef) and node.name == "_draw_set_speed")
  return ast.unparse(method)


def test_mici_navigation_status_is_shifted_one_character_left() -> None:
  source = _draw_set_speed_source()
  assert "panel_x + panel_w * 0.6 - 26" in source


def test_mici_navigation_status_uses_source_specific_colors() -> None:
  source = _draw_set_speed_source()
  assert "Color(199, 125, 255, 230)" in source
  assert "Color(244, 172, 54, 230)" in source
