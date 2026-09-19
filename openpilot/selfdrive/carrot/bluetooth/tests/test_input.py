import asyncio
import json
from pathlib import Path

import pytest

from openpilot.selfdrive.carrot.bluetooth.model import Clicks, CommandReader, CommandWriter, Decoder, DEFAULT_MAPPING, atomic_json, validate_config
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


def press(decoder, code, start, duration=.05):
  decoder.feed(1, code, 1, start)
  decoder.feed(0, 0, 0, start)
  decoder.feed(1, code, 0, start + duration)
  return decoder.feed(0, 0, 0, start + duration)


def test_single_has_no_new_delay_without_double_mapping():
  decoder = Decoder(mapping={'key:30': 'accelCruise', 'key:30@double': 'none'})
  assert press(decoder, 30, 1) == ['key:30']
  assert press(decoder, 30, 1.1) == ['key:30']


def test_double_replaces_both_single_actions_and_single_waits():
  decoder = Decoder(mapping={'key:30': 'accelCruise', 'key:30@double': 'carrotCruise'})
  assert press(decoder, 30, 1) == []
  assert press(decoder, 30, 1.2) == ['key:30@double']
  assert decoder.flush(1.7) == []
  assert press(decoder, 30, 2) == []
  assert decoder.flush(2.39) == []
  assert decoder.flush(2.41) == ['key:30']
  assert decoder.flush(2.42) == []


def test_long_fires_once_on_release_and_never_from_repeat():
  decoder = Decoder(mapping={'key:115': 'accelCruise', 'key:115@long': 'carrotCruise'})
  decoder.feed(1, 115, 1, 1)
  for stamp in (1.3, 1.6, 1.9, 2.1):
    decoder.feed(1, 115, 2, stamp)
    assert decoder.feed(0, 0, 0, stamp) == []
    assert decoder.flush(stamp) == []
  decoder.feed(1, 115, 0, 2.2)
  assert decoder.feed(0, 0, 0, 2.2) == ['key:115@long']
  assert decoder.flush(2.6) == []


def test_touch_long_press_and_firmware_short_pulses_are_distinct():
  decoder = Decoder('yiser-j6', {'center@long': 'carrotCruise'})
  for event in [(3, 0, 300), (3, 1, 500), (1, 330, 1), (0, 0, 0)]:
    decoder.feed(*event, 1)
  decoder.feed(1, 330, 0, 2)
  assert decoder.feed(0, 0, 0, 2) == ['center@long']
  # A shutter which reports only short pulses cannot expose physical hold time.
  assert press(Decoder(mapping={'key:115@long': 'carrotCruise'}), 115, 1, .01) == ['key:115']


def test_learning_detects_unassigned_gestures_and_dropped_frames_cancel_pending():
  decoder = Decoder(learning=True)
  assert press(decoder, 30, 1) == []
  assert press(decoder, 30, 1.15) == ['key:30@double']
  assert press(decoder, 31, 2, .8) == ['key:31@long']
  assert press(decoder, 32, 3) == []
  decoder.feed(0, 3, 0, 3.1)
  assert decoder.flush(3.41) == []


def test_delayed_single_expires_instead_of_firing_after_stall():
  clicks = Clicks({'key:30@double': 'carrotCruise'})
  assert clicks.release('key:30', .05, 1) == []
  assert clicks.flush(2) == []


def test_devices_cannot_complete_each_others_double_click():
  a, b = Decoder(learning=True), Decoder(learning=True)
  assert press(a, 115, 1) == []
  assert press(b, 115, 1.1) == []
  assert a.flush(1.41) == ['key:115']
  assert b.flush(1.51) == ['key:115']


def test_multi_device_queue_keeps_all_commands_in_order_once(tmp_path):
  writer = CommandWriter(tmp_path)
  reader = CommandReader('cruise', tmp_path)
  reader.started = 10
  for mac, action in [('A', 'accelCruise'), ('B', 'decelCruise'), ('C', 'carrotCruise')]:
    writer.send(mac, action, 11)
  assert reader.read(now=11.05) == 'accelCruise'
  assert reader.read(now=11.08) == 'decelCruise'
  assert reader.read(now=11.11) == 'carrotCruise'
  assert reader.read(now=11.14) is None


def test_device_disconnect_and_test_only_cancel_its_queued_commands(tmp_path):
  writer = CommandWriter(tmp_path)
  reader = CommandReader('cruise', tmp_path)
  reader.started = 10
  writer.send('A', 'accelCruise', 11)
  writer.send('B', 'decelCruise', 11)
  writer.prune({'B'}, 11.01)
  assert reader.read(now=11.05) == 'decelCruise'
  writer.send('A', 'accelCruise', 11.1)
  writer.send('B', 'carrotCruise', 11.1)
  atomic_json(tmp_path / 'learn.json', {'address': 'A', 'until': 120})
  assert reader.read(now=11.15) == 'carrotCruise'
  atomic_json(tmp_path / 'learn.json', {})
  assert reader.read(now=11.2) is None


def test_disallowed_drains_all_devices_and_channels_remain_separate(tmp_path):
  writer = CommandWriter(tmp_path)
  cruise = CommandReader('cruise', tmp_path)
  lane = CommandReader('lane', tmp_path)
  cruise.started = lane.started = 10
  writer.send('A', 'accelCruise', 11)
  writer.send('B', 'decelCruise', 11)
  writer.send('B', 'laneLeft', 11)
  assert cruise.read(allowed=False, now=11.05) is None
  assert cruise.read(now=11.08) is None
  assert lane.read(now=11.08) == 'laneLeft'


def test_command_queue_is_bounded_and_expires(tmp_path):
  writer = CommandWriter(tmp_path)
  reader = CommandReader('cruise', tmp_path)
  reader.started = 10
  for i in range(100):
    writer.send(str(i), 'accelCruise', 11)
  assert len(writer.events['cruise']) == 64
  assert reader.read(now=11.5) is None
  writer.prune(set(), 11.5)
  assert json.loads(reader.path.read_text()) == {'events': []}


def test_cancelled_device_events_cannot_return_when_writer_republishes(tmp_path):
  writer = CommandWriter(tmp_path)
  reader = CommandReader('cruise', tmp_path)
  reader.started = 10
  writer.send('A', 'accelCruise', 11)
  atomic_json(tmp_path / 'cancelled.json', {'A': 11.01})
  writer.send('B', 'carrotCruise', 11.02)
  assert reader.read(now=11.05) == 'carrotCruise'
  assert reader.read(now=11.08) is None


def test_extended_mappings_and_device_limit():
  devices = {f'00:00:00:00:00:{i:02X}': {'profile': 'generic', 'enabled': True,
    'mapping': {'key:115': 'accelCruise', 'key:115@double': 'carrotCruise', 'key:115@long': 'paddleDecel'}} for i in range(16)}
  assert len(validate_config({'devices': devices})['devices']) == 16
  devices['00:00:00:00:00:FF'] = devices['00:00:00:00:00:00']
  with pytest.raises(ValueError, match='16'):
    validate_config({'devices': devices})
