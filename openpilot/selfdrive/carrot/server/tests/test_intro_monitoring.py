import ast
import runpy
from pathlib import Path
from typing import Any

import pytest

from openpilot.selfdrive.carrot.server.services.params import clamp_numeric
from openpilot.selfdrive.carrot.server.services.settings import get_settings_cached
from openpilot.selfdrive.monitoring.config import configure_monitoring, disabled_mode
from openpilot.selfdrive.monitoring.test_disable_dm_config import FakeParams


INTRO = Path(__file__).resolve().parents[1] / "features/intro"
PRESETS = runpy.run_path(str(INTRO / "presets.py"))


@pytest.mark.parametrize("preset", PRESETS["PRESET_NAMES"])
@pytest.mark.parametrize("old_mode", [1, 2])
def test_preset_restores_monitoring_at_next_boot(preset, old_mode):
  params = FakeParams({"DisableDM": old_mode, "DriverMonitoringMode": 1, "CarrotVisionEnabled": 0})
  configure_monitoring(params, {})
  # Run the real write/clamp path without importing unrelated Linux HTTP features.
  namespace = {"Dict": dict, "Any": Any, "get_settings_cached": get_settings_cached,
               "clamp_numeric": clamp_numeric, "get_preset": PRESETS["get_preset"],
               "set_param_value": lambda name, value, _definition: params.put(name, value)}
  source = ast.parse((INTRO / "routes.py").read_text(encoding="utf-8"))
  body = [node for node in source.body if isinstance(node, ast.FunctionDef) and node.name in ("_clamped", "_apply_preset_sync")]
  exec(compile(ast.Module(body=body, type_ignores=[]), "routes.py", "exec"), namespace)
  result = namespace["_apply_preset_sync"](preset)
  assert result["failed"] == {}
  assert result["applied"]["DisableDM"] == 0
  assert result["applied"]["DriverMonitoringMode"] == 0
  assert disabled_mode(params) == old_mode
  assert params.get_int("CarrotVisionEnabled") == 0
  configure_monitoring(params, {})
  assert disabled_mode(params) == 0
