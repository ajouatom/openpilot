import ast
from pathlib import Path

from openpilot.common.basedir import BASEDIR


def _function_args(path: Path, class_name: str, function_name: str) -> list[str]:
  tree = ast.parse(path.read_text(encoding="utf-8"))
  cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == class_name)
  function = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == function_name)
  return [arg.arg for arg in function.args.args]


def test_scroller_supports_mici_setup_interaction_controls() -> None:
  scroller = Path(BASEDIR) / "openpilot/system/ui/widgets/scroller.py"
  args = _function_args(scroller, "_Scroller", "scroll_to")

  assert "block_interrupt" in args
  assert "block_widget_interaction" in args


def test_mici_updater_propagates_startup_errors() -> None:
  updater = Path(BASEDIR) / "openpilot/system/ui/mici_updater.py"
  tree = ast.parse(updater.read_text(encoding="utf-8"))
  main = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "main")
  handlers = [node for node in ast.walk(main) if isinstance(node, ast.ExceptHandler)]

  updater_error_handler = next(
    handler for handler in handlers
    if any(isinstance(node, ast.Constant) and node.value == "Updater error: " for node in ast.walk(handler))
  )
  assert any(isinstance(node, ast.Raise) for node in updater_error_handler.body)
