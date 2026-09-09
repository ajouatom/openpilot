import json
import ast
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.ui.egpu_signal_audio import MODEL_ID, SignalAudio, SignalObservation


def message(now, frame, colors=('red_visible',), **changes):
  return dict(modelId=MODEL_ID, camera='road', state='run', timestampEof=int(now*1e9), frameId=frame,
              detections=[dict(label=c, confidence=.7, x1=.4, y1=.2, x2=.42, y2=.23) for c in colors], **changes)


def update(state, now, frame, colors=('red_visible',), **kwargs):
  return state.update(message(now, frame, colors), now=now, valid=True, transport_age=0., enabled=True, **kwargs)


def test_three_distinct_frames_and_time_required_then_no_repeated_tone():
  state = SignalObservation()
  assert update(state, 10., 1) is None
  assert update(state, 10.16, 1) is None
  assert update(state, 10.16, 2) is None
  assert update(state, 10.32, 3) == 'red_visible'
  for i in range(4, 50):
    assert update(state, 10.32+(i-3)*.1, i) is None


@pytest.mark.parametrize('change', [dict(modelId='yolov8-coco'), dict(camera='wideRoad'), dict(state='paused'),
                                   dict(timestampEof=0), dict(timestampEof=20_000_000_000), dict(frameId=None), dict(frameId=-1)])
def test_wrong_model_camera_state_or_capture_age_never_produces_tone(change):
  state = SignalObservation()
  for i in range(5):
    now = 10+i*.16
    m = message(now, i); m.update(change)
    assert state.update(m, now=now, valid=True, transport_age=0, enabled=True) is None


def test_mixed_colors_missing_and_transport_gap_reset_stability():
  state = SignalObservation()
  assert update(state, 10, 1) is None
  assert update(state, 10.16, 2, ('red_visible', 'green_visible')) is None
  assert update(state, 10.32, 3) is None
  assert update(state, 11, 4) is None
  assert update(state, 11.16, 5) is None
  assert update(state, 11.32, 6) == 'red_visible'


@pytest.mark.parametrize('kwargs', [dict(valid=False), dict(transport_age=.4), dict(transport_age=float('nan')),
                                    dict(enabled=False), dict(blocked=True)])
def test_invalid_or_blocked_observation_has_no_tone(kwargs):
  state = SignalObservation()
  for i in range(5):
    now = 10+i*.16
    args = dict(now=now, valid=True, transport_age=0, enabled=True); args.update(kwargs)
    assert state.update(message(now, i), **args) is None


def test_audio_opt_in_one_shot_and_immediate_alert_priority(tmp_path, monkeypatch):
  monkeypatch.setenv('EGPU_YOLO_DIR', str(tmp_path))
  audio = SignalAudio(48000)
  for i in range(4):
    audio.update(message(10+i*.16, i), now=10+i*.16, valid=True, transport_age=0, blocked=False)
  assert not audio.render(1024, priority=False).any()

  (tmp_path/'signal_observation.json').write_text(json.dumps(dict(enabled=True, model_id=MODEL_ID)))
  for i in range(3):
    audio.update(message(12+i*.16, 10+i), now=12+i*.16, valid=True, transport_age=0, blocked=False)
  assert audio.render(1024, priority=False).any()
  assert not audio.render(1024, priority=True).any()
  assert not audio.render(1024, priority=False).any()
  audio.samples = audio.tones['green_visible']; audio.position = 0
  samples = audio.render(10000, priority=False)
  assert np.max(np.abs(samples)) <= .201
  assert not samples[5760:].any()
  assert not audio.render(1024, priority=False).any()


def test_real_soundd_mixer_preserves_alert_and_discards_observation(tmp_path, monkeypatch):
  # Execute the actual mixer method without Linux msgq/hardware imports.
  tree = ast.parse((Path(__file__).parents[1]/'soundd.py').read_text(encoding='utf-8'))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Soundd')
  method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'get_sound_data')
  namespace = dict(np=np, AudibleAlert=SimpleNamespace(none=0), sound_list={1: ('warning.wav', 1, 1.)})
  exec(compile(ast.Module(body=[method], type_ignores=[]), 'soundd.py', 'exec'), namespace)
  monkeypatch.setenv('EGPU_YOLO_DIR', str(tmp_path))
  audio = SignalAudio(48000); audio.samples = audio.tones['red_visible']
  state = SimpleNamespace(signal_audio=audio, current_alert=1, current_sound_frame=0,
                          loaded_sounds={1: np.full(100, .8, dtype=np.float32)}, current_volume=.5)
  np.testing.assert_allclose(namespace['get_sound_data'](state, 50), .4)
  assert audio.samples is None
  state.current_alert = 0
  audio.samples = audio.tones['red_visible']; audio.position = 0
  np.testing.assert_allclose(namespace['get_sound_data'](state, 50), audio.tones['red_visible'][:50]*.5)
