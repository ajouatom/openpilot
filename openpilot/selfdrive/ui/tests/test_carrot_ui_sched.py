"""carrot 전용: UI 스케줄러 foundation(상시 SCHED_OTHER/core5) 회귀 테스트.

기존 FIFO51/core5 UI는 radar(FIFO51)와 같은 우선순위로 core5를 점유해
20Hz cadence를 위협했다. foundation 계약:
- UI는 상시 SCHED_OTHER (RT/FIFO 승격 프리미티브 부재, 재도입 금지)
- 시작은 core0 부트스트랩(offroad power-save의 core4~7 offline 대응),
  onroad에서 render loop가 cores={5}로 re-affine (실패 시 다음 프레임 재시도)
- 시작 시 SCHED_OTHER를 명시 적용하고 readback 검증 (false success 금지,
  검증 불가면 fail-stop), gc.disable()은 유지
- core7은 modeld+plannerd+dmonitoringmodeld 전용 — UI affinity 재도입 금지
"""
import ast
import os
import sys
import types
from pathlib import Path

import pytest

import openpilot.selfdrive.ui.carrot_ui_sched as cus
from openpilot.selfdrive.ui.carrot_ui_sched import ensure_ui_sched_other

UI_DIR = Path(cus.__file__).parent


def _ui_py_tree():
  # ui.py는 import 부작용(gui_app 생성, 레이아웃→msgq)이 있어 AST로 고정한다
  return ast.parse((UI_DIR / "ui.py").read_text())


def _call_names(tree):
  names = []
  for n in ast.walk(tree):
    if isinstance(n, ast.Call):
      if isinstance(n.func, ast.Name):
        names.append(n.func.id)
      elif isinstance(n.func, ast.Attribute):
        names.append(n.func.attr)
  return names


def _affinity_shapes(tree):
  """set_core_affinity 호출 인자의 구조를 수집 — 부트스트랩([0] 상수 리스트)과
  re-affine(list(cores) 변수 연결)을 구분해 검증한다."""
  shapes = []
  for n in ast.walk(tree):
    if not (isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
            and n.func.id == "set_core_affinity" and n.args):
      continue
    a = n.args[0]
    if isinstance(a, ast.List) and all(isinstance(e, ast.Constant) for e in a.elts):
      shapes.append(("const_list", tuple(e.value for e in a.elts)))
    elif (isinstance(a, ast.Call) and isinstance(a.func, ast.Name) and a.func.id == "list"
          and len(a.args) == 1 and isinstance(a.args[0], ast.Name)):
      shapes.append(("list_of_var", a.args[0].id))
    else:
      shapes.append(("other", ast.dump(a)))
  return shapes


@pytest.mark.skipif(sys.platform != "linux", reason="sched_* API는 Linux 전용")
class TestEnsureSchedOtherContract:
  """시작 시 SCHED_OTHER 명시 적용/readback — false success 금지, bounded 재시도,
  검증 불가면 fail-stop (RT UI가 core5의 radar를 굶기며 돌면 안 된다)."""

  def _run(self, monkeypatch, *, policies, drop_raises=False):
    logs = {"info": [], "critical": []}
    monkeypatch.setattr(cus, "PC", False)
    monkeypatch.setattr(cus, "cloudlog", types.SimpleNamespace(
      info=logs["info"].append, critical=logs["critical"].append))
    drops = {"n": 0}

    def fake_drop():
      drops["n"] += 1
      if drop_raises:
        raise OSError("sched_setscheduler refused")

    seq = list(policies)
    monkeypatch.setattr(cus, "drop_realtime", fake_drop)
    monkeypatch.setattr(os, "sched_getscheduler",
                        lambda pid: seq.pop(0) if seq else (policies[-1] if policies else os.SCHED_OTHER))
    return logs, drops

  def test_verified_logs_success_once(self, monkeypatch):
    logs, drops = self._run(monkeypatch, policies=[os.SCHED_OTHER])
    ensure_ui_sched_other()
    assert drops["n"] == 1
    assert len(logs["info"]) == 1 and not logs["critical"]

  def test_rt_readback_fails_stop_without_false_success(self, monkeypatch):
    # readback이 끝내 FIFO면 성공 로그 없이 critical + fail-stop, 재시도는 bounded
    logs, drops = self._run(monkeypatch, policies=[os.SCHED_FIFO, os.SCHED_FIFO])
    with pytest.raises(RuntimeError):
      ensure_ui_sched_other()
    assert drops["n"] == 2  # _VERIFY_ATTEMPTS — 무한/매 프레임 폭주 금지
    assert not logs["info"] and len(logs["critical"]) == 1

  def test_transient_rt_recovers_on_retry(self, monkeypatch):
    logs, _ = self._run(monkeypatch, policies=[os.SCHED_FIFO, os.SCHED_OTHER])
    ensure_ui_sched_other()
    assert len(logs["info"]) == 1 and not logs["critical"]

  def test_oserror_bounded_then_fail_stop(self, monkeypatch):
    logs, drops = self._run(monkeypatch, policies=[], drop_raises=True)
    with pytest.raises(RuntimeError):
      ensure_ui_sched_other()
    assert drops["n"] == 2 and not logs["info"]

  def test_pc_skips_all_syscalls(self, monkeypatch):
    monkeypatch.setattr(cus, "PC", True)
    def boom():
      raise AssertionError("PC에서는 syscall 자체가 없어야 한다")
    monkeypatch.setattr(cus, "drop_realtime", boom)
    ensure_ui_sched_other()  # no-op


class TestUiStartupAst:
  """ui.py 시작/재affine 구조 고정 — import 부작용 때문에 AST 검증."""

  def test_cores_var_assigned_literal_five(self):
    # cores 변수에 정확히 {5}가 할당된다 (파일 어딘가의 {5}가 아니라 할당 구조)
    tree = _ui_py_tree()
    assigns = [n for n in ast.walk(tree) if isinstance(n, ast.Assign)
               and any(isinstance(t, ast.Name) and t.id == "cores" for t in n.targets)]
    assert len(assigns) == 1
    val = assigns[0].value
    assert isinstance(val, ast.Set) and len(val.elts) == 1
    assert isinstance(val.elts[0], ast.Constant) and val.elts[0].value == 5

  def test_bootstrap_core0_and_reaffine_linked_to_cores(self):
    shapes = _affinity_shapes(_ui_py_tree())
    assert ("const_list", (0,)) in shapes    # core0 부트스트랩 (상수 리스트)
    assert ("list_of_var", "cores") in shapes  # re-affine은 cores 변수와 직접 연결
    # 부트스트랩과 re-affine 외 다른 affinity 호출 형태는 없어야 한다
    assert all(s in (("const_list", (0,)), ("list_of_var", "cores")) for s in shapes)

  def test_gc_disable_retained(self):
    # 스케줄러 helper 제거 과정에서 gc.disable()까지 사라지면 안 된다 —
    # GC pause가 렌더 프레임 히치를 만든다 (기존 config_realtime_process 소관)
    tree = _ui_py_tree()
    assert any(isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
               and n.func.attr == "disable" and isinstance(n.func.value, ast.Name)
               and n.func.value.id == "gc" for n in ast.walk(tree))

  def test_startup_applies_sched_other_contract(self):
    # 시작에서 SCHED_OTHER 명시 적용/검증 헬퍼가 호출된다
    assert "ensure_ui_sched_other" in _call_names(_ui_py_tree())

  def test_no_promotion_primitives_in_ui_py(self):
    tree = _ui_py_tree()
    names = _call_names(tree)
    assert "config_realtime_process" not in names  # 재도입 금지
    assert "sched_setscheduler" not in names
    attrs = {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
    assert "SCHED_FIFO" not in attrs
    idents = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
    imported = {a.name for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) for a in n.names}
    assert "Priority" not in idents and "Priority" not in imported  # CTRL_HIGH/CTRL_LOW 승격 금지
    assert "CTRL_HIGH" not in attrs and "CTRL_LOW" not in attrs
    # FIFO51의 숫자 직승격(config_realtime_process(0, 51) 시절)도 재도입 금지 —
    # 승격 콜 부재로 이미 보장되지만 상수 51/53이 인자로 쓰이지 않는지도 고정
    call_const_args = {a.value for n in ast.walk(tree) if isinstance(n, ast.Call)
                       for a in n.args if isinstance(a, ast.Constant)}
    assert 51 not in call_const_args and 53 not in call_const_args

  def test_no_core7_ui_affinity(self):
    # core7은 modeld(FIFO54)+plannerd(FIFO51)+dmonitoringmodeld(FIFO5) 전용 — UI 재배치 금지
    tree = _ui_py_tree()
    for node in ast.walk(tree):
      if isinstance(node, (ast.Set, ast.List, ast.Tuple)):
        consts = {e.value for e in node.elts if isinstance(e, ast.Constant)}
        assert 7 not in consts

  def test_reaffine_failure_swallowed_and_retryable(self):
    # affinity 실패(offroad에 core5 offline 등)가 UI를 죽이면 안 된다 —
    # try/except OSError로 삼키고, 렌더 루프 안이라 다음 프레임에 재시도된다
    tree = _ui_py_tree()

    def contains_reaffine(node):
      return any(isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                 and n.func.id == "set_core_affinity" and n.args
                 and isinstance(n.args[0], ast.Call) for n in ast.walk(node))

    guarded = [t for t in ast.walk(tree) if isinstance(t, ast.Try) and contains_reaffine(t)
               and any(h.type is not None and isinstance(h.type, ast.Name)
                       and h.type.id == "OSError" for h in t.handlers)]
    assert guarded, "re-affine은 try/except OSError 안에 있어야 한다"
    loops = [n for n in ast.walk(tree) if isinstance(n, (ast.For, ast.While))]
    assert any(any(t in ast.walk(loop) for t in guarded) for loop in loops)


class TestNoFifoAnywhereInUi:
  """Plot ON/OFF 포함 selfdrive/ui 프로덕션 경로 어디에도 스케줄러를 FIFO로
  올리는/복구하는 코드가 없어야 한다 (UI는 상시 SCHED_OTHER)."""

  def _prod_sources(self):
    for p in sorted((UI_DIR).rglob("*.py")):
      if "tests" in p.parts:
        continue
      yield p, p.read_text()

  def test_no_promotion_primitives_in_ui_tree(self):
    offenders = []
    for p, src in self._prod_sources():
      tree = ast.parse(src)
      names = _call_names(tree)
      attrs = {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
      if ("config_realtime_process" in names or "sched_setscheduler" in names
          or "SCHED_FIFO" in attrs):
        offenders.append(str(p.relative_to(UI_DIR)))
    assert not offenders, f"FIFO 승격/복구 프리미티브 잔존: {offenders}"

  def test_no_legacy_fifo_restore_logs(self):
    # 과거 복구 로그가 남아 있으면 실기기 로그 분석이 구 설계로 오판한다
    legacy = ("restored to SCHED_FIFO 53", "restored to SCHED_FIFO 51")
    for p, src in self._prod_sources():
      for s in legacy:
        assert s not in src, f"{p.name}: 과거 로그 문자열 잔존 — {s}"
