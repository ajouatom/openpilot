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


def test_both_device_huds_render_egpu_badge_with_shared_runtime_state():
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
    assert "usbgpu_badge_state" in badge_source
    for attr in ("usbgpu_present", "usbgpu_compiled", "usbgpu_compile_pending", "usbgpu_loading", "usbgpu_active", "usbgpu_startup_failed"):
      assert any(
        isinstance(node, ast.Attribute)
        and isinstance(node.value, ast.Name)
        and node.value.id == "ui_state"
        and node.attr == attr
        for node in ast.walk(badge)
      )
    for state in ("active", "loading", "error", "compile_pending", "not_compiled", "ready"):
      assert repr(state) in badge_source
    assert "eGPU REBOOT" in badge_source


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


def test_ui_state_distinguishes_current_compile_from_previous_fallback():
  update_params = _method(UI_DIR / "ui_state.py", "UIState", "update_params")
  calls = [node for node in ast.walk(update_params) if isinstance(node, ast.Call)]
  assert any(isinstance(node.func, ast.Name) and node.func.id == "active_usbgpu_compiled_path" for node in calls)
  assert any(isinstance(node.func, ast.Name) and node.func.id == "usbgpu_compile_pending" for node in calls)
  assert not any(
    isinstance(node.func, ast.Attribute)
    and node.func.attr == "get_bool"
    and node.args
    and isinstance(node.args[0], ast.Constant)
    and node.args[0].value == "UsbGpuCompiled"
    for node in calls
  )


def test_modeld_refreshes_hotplug_state_after_startup():
  source = (UI_DIR.parent / "modeld" / "modeld.py").read_text(encoding="utf-8")

  assert "usbgpu_present_now = usbgpu_present()" in source
  assert 'put_bool_nonblocking("UsbGpuPresent", usbgpu_present_now)' in source
  assert 'put_bool_nonblocking("UsbGpuCompiled", usbgpu_compiled_path() is not None)' in source


def test_modeld_waits_for_a_remembered_egpu_to_enumerate():
  source = (UI_DIR.parent / "modeld" / "modeld.py").read_text(encoding="utf-8")

  assert "USBGPU_DISCOVERY_GRACE_SECONDS = 5.0" in source
  assert '_hardware_seen = params.get_bool("UsbGpuHardwareSeen")' in source
  assert "if not _present and _compiled and _hardware_seen:" in source
  assert "wait_for_usbgpu_present(USBGPU_DISCOVERY_GRACE_SECONDS, USBGPU_DISCOVERY_POLL_INTERVAL)" in source


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


def test_modeld_keeps_egpu_loading_until_first_output_is_published():
  source = (UI_DIR.parent / "modeld" / "modeld.py").read_text(encoding="utf-8")

  loaded = source.index("usbgpu_model_loaded = model is not None")
  pending = source.index("usbgpu_startup_pending = usbgpu_model_loaded", loaded)
  first_send = source.index("pm.send('modelV2', modelv2_send)", pending)
  clear = source.index('params.put_bool("UsbGpuLoading", False)', first_send)
  assert loaded < pending < first_send < clear
  assert "eGPU first model output published; startup complete" in source[clear:]


def test_modeld_runs_internal_fallback_on_the_same_frame():
  source = (UI_DIR.parent / "modeld" / "modeld.py").read_text(encoding="utf-8")

  fallback = source[source.index("eGPU model failed, falling back to internal GPU"):]
  failed_set = fallback.index('params.put_bool("UsbGpuStartupFailed", True)')
  loading_clear = fallback.index('params.put_bool("UsbGpuLoading", False)')
  pending_clear = fallback.index("usbgpu_startup_pending = False")
  assert "model = small_model" in fallback
  assert "model_output = model.run(bufs, transforms, inputs, prepare_only)" in fallback
  assert failed_set < loading_clear < pending_clear < fallback.index("model = small_model")


def test_modeld_queues_tmux_capture_for_all_egpu_failure_paths():
  source = (UI_DIR.parent / "modeld" / "modeld.py").read_text(encoding="utf-8")
  carrot_man_source = (UI_DIR.parent / "carrot" / "carrot_man.py").read_text(encoding="utf-8")

  assert 'USBGPU_TMUX_ERROR_REASON = "egpu_error"' in source
  assert 'queue_usbgpu_error_tmux(params, "model load failed")' in source
  assert 'queue_usbgpu_error_tmux(params, "model load timed out")' in source
  assert 'queue_usbgpu_error_tmux(params, "runtime model execution failed")' in source
  assert '"egpu_error"' in carrot_man_source.split("CARROT_EXCEPTION_TMUX_REASONS =", 1)[1].splitlines()[0]


def test_selfdrived_allows_five_seconds_for_egpu_fallback_to_settle():
  source = (UI_DIR.parent / "selfdrived" / "selfdrived.py").read_text(encoding="utf-8")

  assert "def _big_model_settling" in source
  assert "self.big_model_active and not active" in source
  assert "self.big_model_ready_t + 5.0" in source
  assert "and not big_model_settling" in source


def test_tinygrad_retries_interrupted_usb_event_waits():
  source = (UI_DIR.parents[2] / "tinygrad_repo" / "tinygrad" / "runtime" / "support" / "usb.py").read_text(encoding="utf-8")

  assert "rc != libusb.LIBUSB_ERROR_INTERRUPTED" in source
