import json
from pathlib import Path

import pytest

from openpilot.selfdrive.carrot.bluetooth.model import Decoder, CommandWriter, CommandReader, validate_config

OFF = {'double': False, 'long': False}
MAC = '00:11:22:33:44:55'


def frame(decoder, events, now):
  result = []
  for event in events:
    result += decoder.feed(*event, now)
  return result + decoder.feed(0, 0, 0, now)


def test_fast_key_fires_on_press_not_release_repeat_or_long_timer():
  d = Decoder(mapping={'key:30': 'blinkerLeft', 'key:30@long': 'blinkerRight', 'key:30@double': 'laneLeft'}, gestures=OFF)
  assert frame(d, [(1, 30, 1)], 1) == ['key:30']
  assert frame(d, [(1, 30, 2)], 1.8) == []
  assert d.flush(1.8) == []
  assert frame(d, [(1, 30, 0)], 2) == []
  assert frame(d, [(1, 30, 1)], 2.1) == ['key:30']


def test_enabled_double_waits_and_does_not_fire_single_on_press():
  d = Decoder(mapping={'key:30': 'blinkerLeft', 'key:30@double': 'blinkerRight'}, gestures={'double': True, 'long': False})
  for stamp in (1, 1.2):
    assert frame(d, [(1, 30, 1)], stamp) == []
    assert frame(d, [(1, 30, 0)], stamp + 0.05) == ([] if stamp == 1 else ['key:30@double'])
  assert d.flush(2) == []


def test_enabled_long_still_fires_once_after_threshold():
  d = Decoder(mapping={'key:30': 'blinkerLeft', 'key:30@long': 'blinkerRight'}, gestures={'double': False, 'long': True})
  assert frame(d, [(1, 30, 1)], 1) == []
  assert d.flush(1.69) == []
  assert d.flush(1.71) == ['key:30@long']
  assert d.flush(2.5) == []
  assert frame(d, [(1, 30, 0)], 2.6) == []


def test_fast_yiser_swipe_precedes_release_and_never_becomes_second_action():
  d = Decoder('yiser-j6', {'left': 'blinkerLeft'}, gestures=OFF)
  assert frame(d, [(3, 0, 300), (3, 1, 500), (1, 330, 1)], 1) == []
  assert frame(d, [(3, 0, 450)], 1.05) == ['left']
  assert frame(d, [(3, 0, 500)], 1.1) == []
  assert frame(d, [(3, 0, 100)], 1.2) == []
  assert frame(d, [(1, 330, 0)], 1.3) == []


def test_fast_recorded_yiser_seven_buttons_still_have_one_action_each():
  d = Decoder('yiser-j6', gestures=OFF)
  tokens = []
  for stamp, kind, code, value in json.loads(Path(__file__).with_name('yiser_j6.json').read_text()):
    tokens += d.feed(kind, code, value, stamp)
  assert tokens == ['up', 'down', 'left', 'right', 'center', '1', '2']


def test_dropped_frame_and_learning_do_not_fire_on_press():
  for learning in (False, True):
    d = Decoder(learning=learning, gestures=OFF)
    if not learning:
      d.feed(0, 3, 0, 0)
    assert frame(d, [(1, 30, 1)], 1) == []


def test_held_key_at_connect_or_mapping_reload_must_be_released_first():
  d = Decoder(gestures=OFF)
  d.synchronize_pressed({30}, 1)
  assert frame(d, [(1, 30, 1)], 1.1) == []
  assert frame(d, [(1, 30, 2)], 1.2) == []
  assert frame(d, [(1, 30, 0)], 1.3) == []
  assert frame(d, [(1, 30, 1)], 1.4) == ['key:30']


def test_gesture_options_preserve_legacy_mappings_and_can_disable_them():
  device = {'profile': 'generic', 'enabled': True, 'mapping': {'key:30': 'blinkerLeft', 'key:30@long': 'blinkerRight'}}
  saved = validate_config({'devices': {MAC: device}})['devices'][MAC]
  assert saved['gestures'] == {'double': False, 'long': True}
  saved['gestures'] = OFF
  assert validate_config({'devices': {MAC: saved}})['devices'][MAC]['mapping'] == device['mapping']
  saved['gestures'] = {'double': 'false', 'long': False}
  with pytest.raises(ValueError):
    validate_config({'devices': {MAC: saved}})


def test_physical_indicators_have_their_own_journal(tmp_path):
  writer = CommandWriter(tmp_path)
  for action in ('blinkerLeft', 'blinkerRight'):
    writer.send(MAC, action, 11)
  assert not writer.events['lane'] and not writer.events['cruise']
  reader = CommandReader('blinker', tmp_path)
  reader.started = 10
  assert reader.read(now=11.05) == 'blinkerLeft'
  assert reader.read(now=11.08) == 'blinkerRight'
  assert reader.read(now=11.1) is None
