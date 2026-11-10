import ast
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from opendbc.car import structs
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from opendbc.car.hyundai.hyundaican import suppress_casper_ev_fca11_fault


def load_stock_aeb_boundary():
  path = Path(__file__).parents[1] / "carstate.py"
  tree = ast.parse(path.read_text(encoding="utf8"))
  cls = next(x for x in tree.body if isinstance(x, ast.ClassDef) and x.name == "CarState")
  update = next(x for x in cls.body if isinstance(x, ast.FunctionDef) and x.name == "update")
  node = next(
    x for x in update.body if isinstance(x, ast.If) and "not self.CP.flags & HyundaiFlags.CC_ONLY_CAR" in ast.unparse(x.test) and "aeb_src" in ast.unparse(x)
  )
  wrapper = ast.parse("def check(self, cp_cruise, ret): pass")
  wrapper.body[0].body = [node]
  env = dict(CAR=CAR, HyundaiFlags=HyundaiFlags)
  exec(compile(ast.fix_missing_locations(wrapper), str(path), "exec"), env)
  return env["check"]


@pytest.mark.parametrize("status,fail,expected", [(2, 0, False), (3, 0, True), (2, 3, True), (3, 3, True)])
def test_received_fault_is_reported_before_outgoing_fca_suppression(status, fail, expected):
  received = dict(FCA_Status=status, FCA_Failinfo=fail, CF_VSM_Warn=0, CF_VSM_DecCmdAct=0, FCA_CmdAct=0)
  cp = NS(carFingerprint=CAR.HYUNDAI_CASPER_EV, flags=int(HyundaiFlags.CAMERA_SCC | HyundaiFlags.USE_FCA), openpilotLongitudinalControl=True)
  ret = structs.CarState.new_message()
  load_stock_aeb_boundary()(NS(CP=cp), NS(vl={"FCA11": received, "SCC12": {"TakeOverReq": 0}}), ret)
  assert ret.stockSccFault == expected
  outgoing = suppress_casper_ev_fca11_fault(dict(received))
  assert outgoing["FCA_Failinfo"] == 0
  assert received["FCA_Failinfo"] == fail  # keep RX evidence separate
