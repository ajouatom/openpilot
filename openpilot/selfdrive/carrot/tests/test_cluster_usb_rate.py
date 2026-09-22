"""Exercise the real rate policy without creating an OpenGL window."""
import argparse
import ast
import sys
import types
from pathlib import Path

import pytest


def load_policy(monkeypatch, active=False, device=True):
  params = types.ModuleType('openpilot.common.params')
  state = {'active': active}
  params.Params = lambda: types.SimpleNamespace(get_bool=lambda key: state['active'])
  monkeypatch.setitem(sys.modules, 'openpilot.common.params', params)
  path = Path(__file__).resolve().parents[1] / 'cluster/main.py'
  tree = ast.parse(path.read_text(encoding='utf8'))
  names = {'ClusterUsbFpsReader', 'resolved_usb_display_fps', 'resolved_h264_encoder_fps', 'resolve_usb_output_rate'}
  nodes = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef)) and n.name in names]
  scope = {'argparse': argparse, 'TICI': device}
  exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), 'exec'), scope)
  return scope, state


@pytest.mark.parametrize('codec', ['h264', 'jpeg', 'png'])
@pytest.mark.parametrize('active,expected', [(False, 10), (True, 5)])
def test_device_live_rate_overrides_cli_and_old_environment(monkeypatch, codec, active, expected):
  monkeypatch.setenv('CLUSTER_AUTORUN_FPS', '60')
  scope, state = load_policy(monkeypatch, active)
  args = argparse.Namespace(fps=60, fps_from_cli=True, output='usb', input='live', usb_codec=codec,
                            usb_h264_fps=30, usb_display_fps=60)
  fps, source, reader, controller_fps, auto = scope['resolve_usb_output_rate'](args)
  assert fps == controller_fps == expected and auto
  assert scope['resolved_h264_encoder_fps'](fps, 30) == expected
  assert source == 'fixed USB policy'
  state['active'] = not active
  assert reader.read() == (10 if active else 5)


def test_desktop_diagnostic_fps_stays_explicit(monkeypatch):
  scope, _ = load_policy(monkeypatch, device=False)
  args = argparse.Namespace(fps=30, fps_from_cli=True, output='usb', input='route', usb_codec='h264',
                            usb_h264_fps=20, usb_display_fps=0)
  fps, _, reader, controller, auto = scope['resolve_usb_output_rate'](args)
  assert (fps, reader, controller, auto) == (30, None, 0, False)


def test_missing_params_uses_ten_fps(monkeypatch):
  scope, _ = load_policy(monkeypatch)
  reader = scope['ClusterUsbFpsReader']()
  reader._params = None
  assert reader.read() == 10
