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


def test_both_device_huds_render_connected_egpu_badge_with_active_state():
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
    assert "not ui_state.usbgpu_present and (not ui_state.usbgpu_active)" in badge_source
    assert any(
      isinstance(node, ast.Attribute)
      and isinstance(node.value, ast.Name)
      and node.value.id == "ui_state"
      and node.attr == "usbgpu_present"
      for node in ast.walk(badge)
    )
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


def test_modeld_refreshes_hotplug_state_after_startup():
  source = (UI_DIR.parent / "modeld" / "modeld.py").read_text(encoding="utf-8")

  assert "usbgpu_present_now = usbgpu_present()" in source
  assert 'put_bool_nonblocking("UsbGpuPresent", usbgpu_present_now)' in source
  assert 'put_bool_nonblocking("UsbGpuCompiled", usbgpu_compiled_path() is not None)' in source


def test_modeld_retries_transient_egpu_pcie_startup():
  source = (UI_DIR.parent / "modeld" / "modeld.py").read_text(encoding="utf-8")

  assert "USBGPU_INIT_ATTEMPTS = 6" in source
  assert "usbgpu_pcie_not_ready(exc)" in source
  assert "eGPU PCIe link not ready; retrying" in source


def test_modeld_restarts_instead_of_racing_a_timed_out_egpu_loader():
  source = (UI_DIR.parent / "modeld" / "modeld.py").read_text(encoding="utf-8")

  assert "USBGPU_MODEL_LOAD_TIMEOUT = 40" in source
  assert 'params.put_bool("UsbGpuStartupFailed", True)' in source
  assert 'raise RuntimeError("eGPU model loader did not terminate")' in source


def test_modeld_runs_internal_fallback_on_the_same_frame():
  source = (UI_DIR.parent / "modeld" / "modeld.py").read_text(encoding="utf-8")

  fallback = source[source.index("eGPU model failed, falling back to internal GPU"):]
  assert "model = small_model" in fallback
  assert "model_output = model.run(bufs, transforms, inputs, prepare_only)" in fallback


def test_selfdrived_allows_five_seconds_for_egpu_fallback_to_settle():
  source = (UI_DIR.parent / "selfdrived" / "selfdrived.py").read_text(encoding="utf-8")

  assert "def _big_model_settling" in source
  assert "self.big_model_active and not active" in source
  assert "self.big_model_ready_t + 5.0" in source
  assert "and not big_model_settling" in source


def test_tinygrad_retries_interrupted_usb_event_waits():
  source = (UI_DIR.parents[2] / "tinygrad_repo" / "tinygrad" / "runtime" / "support" / "usb.py").read_text(encoding="utf-8")

  assert "rc != libusb.LIBUSB_ERROR_INTERRUPTED" in source
