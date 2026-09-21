"""Exercise updater failure/retry transitions without device graphics or disk writes."""
import ast
from collections import deque
from pathlib import Path
import re
from types import SimpleNamespace

import pytest


def load_updater(name, namespace):
  path = Path(__file__).resolve().parents[1] / f"ui/{name}_updater.py"
  tree = ast.parse(path.read_text(encoding="utf-8"))
  cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "Updater")
  cls.bases = []
  module = ast.Module(body=[ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0), cls], type_ignores=[])
  exec(compile(ast.fix_missing_locations(module), str(path), "exec"), namespace)
  return namespace["Updater"]


@pytest.mark.parametrize("name", ["tici", "mici"])
def test_failed_update_can_be_retried_without_reboot(name, mocker):
  failed = mocker.Mock(stdout=iter(["Network retry 4/5 in 10s: ConnectionError: 0\n", "Download failed. Check Wi-Fi, then tap Retry.\n"]))
  failed.wait.return_value = 1
  successful = mocker.Mock(stdout=iter(["Update complete; rebooting: 100\n"]))
  successful.wait.return_value = 0
  popen = mocker.Mock(side_effect=[failed, successful])
  hardware = mocker.Mock()
  confirmation = mocker.Mock()
  app = mocker.Mock()
  namespace = {"subprocess": SimpleNamespace(Popen=popen, PIPE=-1, STDOUT=-2), "HARDWARE": hardware,
               "re": re, "mark_update_confirmed": confirmation, "gui_app": app, "Screen": SimpleNamespace(PROGRESS=2)}
  cls = load_updater(name, namespace)
  updater = cls.__new__(cls)
  updater.updater = "/agnos.py"
  updater.manifest = "/agnos.json"
  updater.update_thread = None
  updater._last_output = deque(maxlen=12)
  updater._failure_reason = None
  updater.show_reboot_button = False
  updater.failure_detail = ""
  updater._progress_page = mocker.Mock()
  updater._failed_page = mocker.Mock()
  updater.show_event = mocker.Mock()
  pending = []
  updater._progress_page.set_shown_callback.side_effect = pending.append
  app.push_widget.side_effect = lambda _widget: pending.pop()() if pending else None
  app.pop_widgets_to.side_effect = lambda _widget, callback: callback()

  def thread(target, **_kwargs):
    return SimpleNamespace(start=target, is_alive=lambda: False)

  namespace["threading"] = SimpleNamespace(Thread=thread)
  updater.install_update()
  hardware.reboot.assert_not_called()
  if name == "mici":
    updater._nav_stack_tick()
    assert "Check Wi-Fi" in updater._failed_page.set_reason.call_args.args[0]
    updater._retry()
  else:
    assert updater.show_reboot_button
    assert "Check Wi-Fi" in updater.failure_detail
    updater.install_update()

  assert popen.call_count == 2
  assert confirmation.call_count == 2
  hardware.reboot.assert_called_once()
  assert updater.progress_value == 100


def test_mici_failure_can_open_network_setup_without_starting_download(mocker):
  app = mocker.Mock()
  namespace = {"gui_app": app}
  updater = load_updater("mici", namespace).__new__(namespace["Updater"])
  updater._network_setup_page = object()
  updater.install_update = mocker.Mock()
  updater._open_network_setup()
  app.push_widget.assert_not_called()
  app.pop_widgets_to.call_args.args[1]()
  app.push_widget.assert_called_once_with(updater._network_setup_page)
  updater.install_update.assert_not_called()


@pytest.mark.parametrize("name", ["tici", "mici"])
def test_retry_does_not_start_a_second_active_updater(name, mocker):
  confirmation = mocker.Mock()
  namespace = {"mark_update_confirmed": confirmation}
  cls = load_updater(name, namespace)
  updater = cls.__new__(cls)
  updater.update_thread = SimpleNamespace(is_alive=lambda: True)
  updater.install_update()
  confirmation.assert_not_called()
