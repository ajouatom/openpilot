import importlib
from pathlib import Path
import sys
import types


CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

from cluster_git_status import GitBranchStatusProvider
import cluster_renderer
from cluster_renderer import ClusterUiRenderer


def test_cluster_autorun_defaults_to_normal_scheduler_and_keeps_affinity(monkeypatch):
  params_module = types.ModuleType("openpilot.common.params")
  params_module.Params = object
  hardware_module = types.ModuleType("openpilot.system.hardware")
  hardware_module.TICI = False
  monkeypatch.setitem(sys.modules, "openpilot.common.params", params_module)
  monkeypatch.setitem(sys.modules, "openpilot.system.hardware", hardware_module)

  module_name = "openpilot.selfdrive.carrot.cluster_autorun"
  sys.modules.pop(module_name, None)
  cluster_autorun = importlib.import_module(module_name)
  affinity_calls = []
  monkeypatch.setattr(cluster_autorun, "_cluster_realtime_cores", lambda: [1, 2, 3, 4])
  monkeypatch.setattr(
    cluster_autorun,
    "_set_current_process_affinity",
    lambda cores: affinity_calls.append(cores) or cores,
  )

  try:
    assert cluster_autorun.AUTORUN_DEFAULT_ENV["CLUSTER_REALTIME"] == "0"
    cluster_autorun._configure_autorun_affinity()
    assert affinity_calls == [[1, 2, 3, 4]]
  finally:
    sys.modules.pop(module_name, None)


def test_cluster_run_explicitly_drops_realtime_and_keeps_affinity(monkeypatch):
  cluster_run = importlib.import_module("openpilot.selfdrive.carrot.cluster_run")
  realtime_module = types.ModuleType("openpilot.common.realtime")
  calls = []
  realtime_module.config_realtime_process = lambda *_args: calls.append("realtime")
  realtime_module.drop_realtime = lambda: calls.append("drop")
  realtime_module.set_core_affinity = lambda cores: calls.append(("affinity", cores))
  monkeypatch.setitem(sys.modules, "openpilot.common.realtime", realtime_module)
  monkeypatch.setenv("CLUSTER_REALTIME", "0")
  monkeypatch.setattr(cluster_run, "_resolved_realtime_cores", lambda: [1, 2, 3, 4])
  monkeypatch.setattr(cluster_run.gc, "enable", lambda: calls.append("gc_enable"))

  cluster_run.configure_cluster_realtime()

  assert calls == ["gc_enable", "drop", ("affinity", [1, 2, 3, 4])]


def test_git_status_remote_disabled_never_starts_git_worker(tmp_path, monkeypatch):
  git_dir = tmp_path / ".git"
  git_dir.mkdir()
  (git_dir / "HEAD").write_text("ref: refs/heads/performance\n", encoding="utf-8")
  provider = GitBranchStatusProvider(tmp_path, remote_enabled=False)

  def fail_git(*_args, **_kwargs):
    raise AssertionError("remote-disabled provider must not run git")

  monkeypatch.setattr(provider, "_git", fail_git)

  assert provider.status().branch == "performance"
  assert provider.status().detail == ""
  assert provider._worker is None


def test_renderer_fonts_use_bilinear_filter_without_mipmaps(tmp_path, monkeypatch):
  font_path = tmp_path / "font.ttf"
  font_path.touch()
  renderer = ClusterUiRenderer()
  loaded_fonts = [
    types.SimpleNamespace(texture=types.SimpleNamespace(id=1)),
    types.SimpleNamespace(texture=types.SimpleNamespace(id=2)),
  ]
  filters = []

  monkeypatch.setattr(renderer, "_font_candidates", lambda: [font_path])
  monkeypatch.setattr(cluster_renderer.rl, "set_trace_log_level", lambda _level: None)
  monkeypatch.setattr(cluster_renderer.rl, "load_font_ex", lambda *_args: loaded_fonts.pop(0))
  monkeypatch.setattr(
    cluster_renderer.rl,
    "gen_texture_mipmaps",
    lambda _texture: (_ for _ in ()).throw(AssertionError("HUD fonts must not generate mipmaps")),
  )
  monkeypatch.setattr(
    cluster_renderer.rl,
    "set_texture_filter",
    lambda texture, filter_mode: filters.append((texture.id, filter_mode)),
  )

  renderer._load_font()
  renderer._load_korean_font()

  assert filters == [
    (1, cluster_renderer.rl.TextureFilter.TEXTURE_FILTER_BILINEAR),
    (2, cluster_renderer.rl.TextureFilter.TEXTURE_FILTER_BILINEAR),
  ]
