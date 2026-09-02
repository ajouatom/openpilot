import ast
import runpy
import sys
import types
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


def test_standalone_widgets_tolerate_a_stale_params_registry() -> None:
  widgets = Path(BASEDIR) / "openpilot/system/ui/widgets/__init__.py"
  tree = ast.parse(widgets.read_text(encoding="utf-8"))
  device_import = next(
    node for node in ast.walk(tree)
    if isinstance(node, ast.ImportFrom) and node.module == "openpilot.selfdrive.ui.ui_state"
  )
  params_import = next(
    node for node in ast.walk(tree)
    if isinstance(node, ast.ImportFrom) and node.module == "openpilot.common.params"
  )
  params_try = next(node for node in ast.walk(tree) if isinstance(node, ast.Try) and params_import in node.body)
  import_try = next(node for node in ast.walk(tree) if isinstance(node, ast.Try) and device_import in node.body)
  params_handled_names = {
    handler.type.id
    for handler in params_try.handlers
    if isinstance(handler.type, ast.Name)
  }
  handled_names = {
    exception.id
    for handler in import_try.handlers
    if isinstance(handler.type, ast.Tuple)
    for exception in handler.type.elts
    if isinstance(exception, ast.Name)
  }

  assert "UnknownKeyName" in {alias.name for alias in params_import.names}
  assert "ImportError" in params_handled_names
  assert {"ImportError", "UnknownKeyName"} <= handled_names


def test_standalone_widgets_tolerate_missing_params_binary(monkeypatch) -> None:
  widgets = Path(BASEDIR) / "openpilot/system/ui/widgets/__init__.py"
  pyray = types.ModuleType("pyray")
  application = types.ModuleType("openpilot.system.ui.lib.application")
  application.gui_app = object()
  application.MousePos = object
  application.MAX_TOUCH_SLOTS = 2
  application.MouseEvent = object

  monkeypatch.setitem(sys.modules, "pyray", pyray)
  monkeypatch.setitem(sys.modules, "openpilot.system.ui.lib.application", application)
  monkeypatch.setitem(sys.modules, "openpilot.common.params", None)

  namespace = runpy.run_path(str(widgets))
  assert namespace["device"].awake


def test_tici_updater_only_installs_after_button_confirmation() -> None:
  updater = Path(BASEDIR) / "openpilot/system/ui/tici_updater.py"
  source = updater.read_text(encoding="utf-8")

  assert 'Button("Install", click_callback=self.install_update' in source
  assert 'cmd = [self.updater, "--swap", self.manifest]' in source
  assert "mark_update_confirmed(self.manifest)" in source
  assert "if update_confirmed(manifest_path)" in source
  assert "stderr=subprocess.STDOUT" in source
  assert 'Button("Retry", click_callback=self.install_update' in source


def test_mici_updater_checks_manifest_hosts_without_blocking_explicit_install() -> None:
  updater = Path(BASEDIR) / "openpilot/system/ui/mici_updater.py"
  source = updater.read_text(encoding="utf-8")

  assert "probe_urls=manifest_download_urls(manifest_path)" in source
  assert "self._continue_button.set_visible(True)" in source
  assert "mark_update_confirmed(self.manifest)" in source
  assert "if update_confirmed(manifest_path)" in source
  assert "stderr=subprocess.STDOUT" in source
  assert "self._failed_page.set_reason(reason)" in source


def test_connectivity_monitor_accepts_update_specific_probe_urls() -> None:
  setup = Path(BASEDIR) / "openpilot/system/ui/mici_setup.py"
  source = setup.read_text(encoding="utf-8")

  assert "probe_urls: tuple[str, ...] | None = None" in source
  assert "for url in self._probe_urls" in source
