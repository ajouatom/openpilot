# Yellow CAN badge on both onroad HUDs (Jetson USB CAN connected): wiring and an import-time guard
import ast
from pathlib import Path

UI_DIR = Path(__file__).resolve().parents[1]
HUD_PATHS = (UI_DIR / "onroad" / "hud_renderer.py", UI_DIR / "mici" / "onroad" / "hud_renderer.py")


def _hud(path):
  tree = ast.parse(path.read_text(encoding="utf-8"))
  return tree, next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "HudRenderer")


def test_both_huds_draw_the_can_badge_from_the_egpu_badge():
  for path in HUD_PATHS:
    _, cls = _hud(path)
    methods = {n.name: n for n in cls.body if isinstance(n, ast.FunctionDef)}
    calls = [n for n in ast.walk(methods["_draw_egpu_badge"])
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == "_draw_can_badge"]
    assert len(calls) == 2, path  # with and without an eGPU badge
    assert "can_link" in ast.unparse(methods["_draw_can_badge"])


def test_no_pyray_union_annotation_evaluated_at_import():
  # pyray structs (rl.Rectangle, rl.Color) are factory functions: `rl.Rectangle | None` in a def raises TypeError
  # when the class body runs, which kills the UI at start. The unit-test stubs use real classes, so check the source.
  for path in HUD_PATHS:
    tree, cls = _hud(path)
    if any(isinstance(n, ast.ImportFrom) and n.module == "__future__" and any(a.name == "annotations" for a in n.names) for n in tree.body):
      continue
    for fn in (n for n in cls.body if isinstance(n, ast.FunctionDef)):
      for arg in fn.args.args + fn.args.kwonlyargs:
        ann = arg.annotation
        assert not (isinstance(ann, ast.BinOp) and isinstance(ann.op, ast.BitOr)), f"{path.name}:{fn.name}({arg.arg})"
