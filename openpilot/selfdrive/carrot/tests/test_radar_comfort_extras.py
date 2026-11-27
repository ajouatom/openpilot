import ast
from pathlib import Path

import pytest

from openpilot.selfdrive.carrot.radar import radar_lead_comfort_extras_enabled


PLANNER = Path(__file__).resolve().parents[2] / "controls" / "lib" / "longitudinal_planner.py"


@pytest.mark.parametrize(("brand", "classic_238", "expected"), [
  ("hyundai", True, False),
  ("hyundai", False, True),
  ("toyota", True, True),
  ("toyota", False, True),
])
def test_only_k7_classic_radar_follows_like_vision(brand, classic_238, expected):
  assert radar_lead_comfort_extras_enabled(brand, classic_238) is expected


def test_planner_gates_all_radar_only_following_extras():
  tree = ast.parse(PLANNER.read_text(encoding="utf-8"))
  gated = set()
  for node in ast.walk(tree):
    if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
      name, value = node.targets[0].id, node.value
    elif isinstance(node, ast.keyword):
      name, value = node.arg, node.value
    else:
      continue
    if (name in ("lead_accel_response_enabled", "lead_gap_enabled", "preview_enabled")
        and isinstance(value, ast.BoolOp) and isinstance(value.op, ast.And)
        and ast.unparse(value.values[0]) == "self.radar_comfort_extras"):
      gated.add(name)
  assert gated == {"lead_accel_response_enabled", "lead_gap_enabled", "preview_enabled"}
