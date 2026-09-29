import ast
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.common import reboot


@pytest.mark.parametrize('sound_error', [None, FileNotFoundError(), subprocess.TimeoutExpired('sound', 4),
                                       subprocess.CalledProcessError(1, 'sound')])
def test_chime_finishes_or_fails_before_reboot(monkeypatch, sound_error):
  calls = []
  monkeypatch.setattr(reboot, '_device_only', lambda: None)

  def run(args, **kwargs):
    calls.append((args, kwargs))
    if '--sound-only' in args and sound_error:
      raise sound_error

  monkeypatch.setattr(reboot.subprocess, 'run', run)
  reboot.reboot_device()
  assert len(calls) == 2
  assert calls[0][0] == [sys.executable, str(Path(reboot.__file__).resolve()), '--sound-only']
  assert calls[0][1]['timeout'] == reboot.SOUND_TIMEOUT
  assert calls[1][0] == ['sudo', '-n', 'reboot']
  assert calls[1][1]['check']


def test_reboot_command_failure_is_not_hidden(monkeypatch):
  monkeypatch.setattr(reboot, '_device_only', lambda: None)
  monkeypatch.setattr(reboot, 'play_reboot_sound', lambda: None)

  def failed(*args, **kwargs):
    raise subprocess.CalledProcessError(1, 'reboot')

  monkeypatch.setattr(reboot.subprocess, 'run', failed)
  with pytest.raises(subprocess.CalledProcessError):
    reboot.reboot_device()


def test_desktop_never_plays_or_reboots(monkeypatch):
  monkeypatch.setattr(Path, 'exists', lambda path: False)
  monkeypatch.setattr(reboot, 'play_reboot_sound', lambda: pytest.fail('must not play'))
  monkeypatch.setattr(reboot.subprocess, 'Popen', lambda *a, **kw: pytest.fail('must not spawn'))
  with pytest.raises(RuntimeError, match='disabled'):
    reboot.reboot_device()
  with pytest.raises(RuntimeError, match='disabled'):
    reboot.spawn_reboot()


def test_web_reboot_spawns_detached_child_with_existing_delay(monkeypatch):
  monkeypatch.setattr(reboot, '_device_only', lambda: None)
  calls = []
  monkeypatch.setattr(reboot.subprocess, 'Popen', lambda *args, **kwargs: calls.append((args, kwargs)))
  reboot.spawn_reboot(delay=1)
  assert calls == [(([sys.executable, str(Path(reboot.__file__).resolve()), '--delay', '1'],), {'start_new_session': True})]


@pytest.mark.parametrize('setting,volume', [(None, .5), ('invalid', .5), ('0', 0), ('100', .5), ('200', 1), ('300', 1), ('-10', 0)])
def test_pcm_uses_existing_prompt_and_saved_volume(tmp_path, monkeypatch, setting, volume):
  monkeypatch.setattr(reboot, 'PARAMS_PATH', tmp_path)
  if setting is not None:
    (tmp_path / 'SoundVolumeAdjust').write_text(setting)
  calls = []
  monkeypatch.setitem(sys.modules, 'sounddevice', SimpleNamespace(play=lambda *a, **kw: calls.append((a, kw))))
  assert reboot._sound_volume() == volume
  reboot._play_sound()
  if volume == 0:
    assert not calls
    return
  (samples, rate), kwargs = calls[0]
  assert rate == 48000
  assert kwargs == {'blocking': True}
  assert samples.shape[1] == 1
  assert abs(samples.shape[0] / rate - 1.656) < .001
  assert samples.dtype == np.float32
  assert np.max(np.abs(samples)) <= volume
  assert np.any(samples != 0)
  np.testing.assert_array_equal(samples[:7200], 0)


def test_missing_asset_fails_only_inside_audio_child(monkeypatch, tmp_path):
  monkeypatch.setattr(reboot, 'SOUND_PATH', tmp_path / 'missing.wav')
  monkeypatch.setattr(reboot, '_sound_volume', lambda: .5)
  monkeypatch.setitem(sys.modules, 'sounddevice', SimpleNamespace(play=lambda *a, **kw: pytest.fail('must not play')))
  with pytest.raises(FileNotFoundError):
    reboot._play_sound()


def test_hardware_reboot_calls_common_helper(monkeypatch):
  path = Path(__file__).resolve().parents[1] / 'hardware/tici/hardware.py'
  tree = ast.parse(path.read_text(encoding='utf-8'))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Tici')
  method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'reboot')
  namespace = {}
  exec(compile(ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[])), str(path), 'exec'), namespace)
  calls = []
  monkeypatch.setattr(reboot, 'reboot_device', lambda: calls.append('reboot'))
  namespace['reboot'](None)
  assert calls == ['reboot']
