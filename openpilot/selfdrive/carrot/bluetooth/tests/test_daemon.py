"""Exercise the actual reader loop with local pipes, never vehicle input or CAN."""
import os
from types import SimpleNamespace

import pytest

pytest.importorskip('fcntl')
from openpilot.selfdrive.carrot.bluetooth import daemon
from openpilot.selfdrive.carrot.bluetooth.model import CommandReader, CommandWriter


@pytest.mark.parametrize('learning', [False, True])
@pytest.mark.parametrize('double', [False, True])
def test_daemon_multiplexes_two_remotes_and_only_suppresses_device_under_test(tmp_path, monkeypatch, learning, double):
  first, second = '00:11:22:33:44:55', '66:77:88:99:AA:BB'
  settings = {'devices': {
    first: {'profile': 'generic', 'enabled': True, 'mapping': {'key:115': 'accelCruise'}},
    second: {'profile': 'generic', 'enabled': True, 'mapping': {'key:115': 'carrotCruise'}},
  }}
  if double:
    for device in settings['devices'].values():
      device['mapping']['key:115@double'] = 'gapAdjustCruise'
  clock = [10.0]
  real_read_json = daemon.read_json
  monkeypatch.setattr(daemon, 'RUNTIME', tmp_path)
  monkeypatch.setattr(daemon, 'config', lambda: settings)
  monkeypatch.setattr(daemon, 'CommandWriter', lambda: CommandWriter(tmp_path))
  monkeypatch.setattr(daemon.time, 'monotonic', lambda: clock[0])
  monkeypatch.setattr(daemon, 'read_json', lambda path, default=None:
    {'address': first, 'until': 120} if learning and path.name == 'learn.json' else real_read_json(path, default))

  class State(dict):
    alive = dict.fromkeys(('carState', 'deviceState', 'selfdriveState'), True)
    valid = {'carState': True}

    def update(self, _):
      clock[0] += .01

  state = State(carState=SimpleNamespace(canValid=True, vEgo=0), deviceState=SimpleNamespace(started=True),
                selfdriveState=SimpleNamespace(enabled=False))
  monkeypatch.setattr(daemon.messaging, 'SubMaster', lambda _: state)
  pipes = [os.pipe(), os.pipe()]
  opened = {f'input-{i}': (mac, pipes[i][0]) for i, mac in enumerate((first, second))}
  monkeypatch.setattr(daemon, 'devices', lambda: {path: (mac, 'remote') for path, (mac, _) in opened.items()})
  monkeypatch.setattr(daemon, 'open_input', lambda path: opened[path][1])
  for _, output in pipes:
    os.write(output, b''.join(daemon.EVENT.pack(10, 1000, *event) for event in
      [(1, 115, 1), (0, 0, 0), (1, 115, 0), (0, 0, 0)]))
    os.close(output)
  calls = [0]

  class Done(Exception):
    pass

  def select(fds, *_):
    calls[0] += 1
    if double and calls[0] == 2:
      clock[0] = 10.36
      return [], [], []
    if calls[0] > 1:
      raise Done
    return fds, [], []

  monkeypatch.setattr(daemon.select, 'select', select)
  with pytest.raises(Done):
    daemon.main()
  reader = CommandReader('cruise', tmp_path)
  reader.started = 9
  stamp = 10.4 if double else 10.1
  actions = [reader.read(now=stamp), reader.read(now=stamp + .03), reader.read(now=stamp + .06)]
  assert actions == (['carrotCruise', None, None] if learning else ['accelCruise', 'carrotCruise', None])
