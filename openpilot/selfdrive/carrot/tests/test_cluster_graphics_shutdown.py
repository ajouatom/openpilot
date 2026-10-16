"""Interrupt cleanup at the same boundaries used by a managed HUD shutdown."""
import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

CLUSTER = Path(__file__).resolve().parents[1] / 'cluster'


@pytest.mark.parametrize('failure', [RuntimeError, KeyboardInterrupt])
def test_renderer_closes_native_window_when_resource_cleanup_fails(failure):
  tree = ast.parse((CLUSTER / 'cluster_renderer.py').read_text(encoding='utf8'))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'ClusterUiRenderer')
  method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'close')
  calls = []

  def fail():
    raise failure('interrupted resource close')

  renderer = SimpleNamespace(_window_open=True, _close_resources=fail)
  scope = {'rl': SimpleNamespace(close_window=lambda: calls.append('window'))}
  exec(compile(ast.Module(body=[method], type_ignores=[]), 'renderer-close', 'exec'), scope)
  with pytest.raises(failure):
    scope['close'](renderer)
  assert calls == ['window']
  assert not renderer._window_open


@pytest.mark.parametrize('fault', ['gc', 'network', 'pipeline', 'renderer', 'display'])
def test_run_cleanup_always_releases_graphics_before_blocking_display_close(fault):
  tree = ast.parse((CLUSTER / 'main.py').read_text(encoding='utf8'))
  run = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'run_demo')
  final = [n for n in run.body if isinstance(n, ast.Try) and n.finalbody][-1].finalbody
  calls = []

  def close(name):
    calls.append(name)
    if name == fault:
      raise KeyboardInterrupt(name)

  scope = {'previous_gc_thresholds': None, 'restore_gc_after_run': lambda _: close('gc'),
               'usb_display': SimpleNamespace(close=lambda: close('display')),
               '_set_cluster_hud_connected': lambda _: None,
               'scheduler': SimpleNamespace(update=lambda *args, **kwargs: None),
               'signal_installed': False, 'gc_hook': None,
               'network_address_provider': SimpleNamespace(close=lambda: close('network')),
               'cluster_core_usage_sampler': None,
               'renderer': SimpleNamespace(release_nv12_dmabuf_output=lambda: None, close=lambda: close('renderer')),
               'h264_pipeline': None, 'usb_pipeline': SimpleNamespace(close=lambda: close('pipeline')),
               'controller': None, 'route_tools_window': None, 'route_source': None, 'live_source': None, 'navi_source': None}
  with pytest.raises(KeyboardInterrupt):
    exec(compile(ast.Module(body=final, type_ignores=[]), 'run-cleanup', 'exec'), scope)
  assert calls[-2:] == ['renderer', 'display']


def test_context_is_owned_before_font_setup_can_be_interrupted():
  tree = ast.parse((CLUSTER / 'cluster_renderer.py').read_text(encoding='utf8'))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'ClusterUiRenderer')
  method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'open')
  calls = []

  def font():
    assert renderer._window_open
    raise KeyboardInterrupt('font setup')

  renderer = SimpleNamespace(_window_open=False, width=1, height=1, title='test', target_fps=0,
                             _profile_start=lambda: 0, _profile_add=lambda *args: None, _load_font=font)
  rl = SimpleNamespace(set_trace_log_level=lambda _: None, TraceLogLevel=SimpleNamespace(LOG_WARNING=1),
                       init_window=lambda *args: calls.append('opened'), is_window_ready=lambda: True)
  scope = {'rl': rl}
  exec(compile(ast.Module(body=[method], type_ignores=[]), 'renderer-open', 'exec'), scope)
  with pytest.raises(KeyboardInterrupt):
    scope['open'](renderer)
  assert calls == ['opened'] and renderer._window_open
