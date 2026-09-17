import asyncio
import json
from pathlib import Path

import pytest

from openpilot.selfdrive.carrot.bluetooth.model import CommandReader, Decoder, DEFAULT_MAPPING, atomic_json, validate_config
from openpilot.selfdrive.carrot.bluetooth.bluez import Bluez


def test_recorded_seven_yiser_buttons():
  decoder = Decoder('yiser-j6')
  tokens = []
  for stamp, kind, code, value in json.loads(Path(__file__).with_name('yiser_j6.json').read_text()):
    tokens.extend(decoder.feed(kind, code, value, stamp))
  assert tokens == ['up', 'down', 'left', 'right', 'center', '1', '2']
  assert [DEFAULT_MAPPING[t] for t in tokens] == ['accelCruise', 'decelCruise', 'laneLeft', 'laneRight', 'paddleDecel', 'gapAdjustCruise', 'none']


def test_key_repeat_release_and_dropped_events():
  decoder = Decoder()
  assert decoder.feed(1, 30, 1, 0) == []
  assert decoder.feed(1, 30, 2, .1) == []
  assert decoder.feed(0, 0, 0, .1) == []
  assert decoder.feed(1, 30, 0, .2) == []
  assert decoder.feed(0, 0, 0, .2) == ['key:30']
  assert decoder.feed(1, 30, 0, .3) == []
  assert decoder.feed(0, 0, 0, .3) == []
  decoder.feed(1, 30, 1, 1)
  decoder.feed(0, 3, 0, 1)
  decoder.feed(1, 30, 0, 1.1)
  assert decoder.feed(0, 0, 0, 1.1) == []


def test_stale_incomplete_gestures_do_not_fire():
  decoder = Decoder('yiser-j6')
  for event in [(3, 0, 300), (3, 1, 500), (1, 330, 1), (0, 0, 0)]:
    decoder.feed(*event, 1)
  decoder.feed(1, 330, 0, 3)
  assert decoder.feed(0, 0, 0, 3) == []
  decoder = Decoder('yiser-j6')
  decoder.feed(1, 330, 0, 4)
  assert decoder.feed(0, 0, 0, 4) == []


def test_commands_reject_replay_startup_stale_and_disallowed(tmp_path):
  reader = CommandReader('cruise', tmp_path)
  reader.started = 10
  for number, (created, now, allowed, expected) in enumerate([
    (9.9, 10, True, None), (10.1, 10.2, True, 'accelCruise'), (10.3, 10.9, True, None),
    (12, 11, True, None), (11.1, 11.2, False, None), (11.3, 11.4, True, 'accelCruise'),
  ]):
    atomic_json(reader.path, {'id': str(number), 'time': created, 'action': 'accelCruise'})
    assert reader.read(allowed=allowed, now=now) == expected
    assert reader.read(now=now + .03) is None


def test_entering_test_mode_cancels_pending_action(tmp_path):
  reader = CommandReader('cruise', tmp_path)
  reader.started = 10
  mac = '66:C0:0C:7B:6E:71'
  atomic_json(reader.path, {'id': 'pending', 'time': 11, 'action': 'accelCruise', 'address': mac})
  atomic_json(tmp_path / 'learn.json', {'address': mac, 'until': 120})
  assert reader.read(now=11.1) is None
  atomic_json(tmp_path / 'learn.json', {})
  assert reader.read(now=11.2) is None


@pytest.mark.parametrize('device', [
  {'profile': 'shell'}, {'profile': 'generic', 'enabled': 'true'},
  {'profile': 'generic', 'mapping': {'key:999': 'accelCruise'}},
  {'profile': 'generic', 'mapping': {'key:30': 'systemctl reboot'}},
])
def test_invalid_mapping(device):
  with pytest.raises(ValueError):
    validate_config({'devices': {'66:C0:0C:7B:6E:71': device}})


def test_pairing_prompt_validation():
  async def run():
    client = Bluez()
    client.prompt = {'id': 'current', 'kind': 'RequestPasskey'}
    client.answer = asyncio.get_running_loop().create_future()
    with pytest.raises(ValueError):
      client.respond('old', '123456')
    with pytest.raises(ValueError):
      client.respond('current', 'not a number')
    client.respond('current', '123456')
    assert await client.answer == '123456'
  asyncio.run(run())
