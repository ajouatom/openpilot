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

  state = State(carState=SimpleNamespace(canValid=True, vEgo=0, brakePressed=False, gasPressed=False,
                                       gearShifter='drive', buttonEvents=[]), deviceState=SimpleNamespace(started=True),
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


@pytest.mark.parametrize('stop', ['release', 'disconnect', 'brake', 'gas', 'gear', 'physical', 'disable', 'can', 'offroad', 'stale'])
def test_held_speed_repeats_and_interruptions_remove_pending_ticks(tmp_path, monkeypatch, stop):
  mac = '00:11:22:33:44:55'
  settings = {'devices': {mac: {'profile': 'generic', 'enabled': True,
                               'mapping': {'key:115@long': 'accelCruiseLong'}}}}
  clock = [10.0]
  sent = []

  class Writer(CommandWriter):
    def send(self, *args, **kwargs):
      sent.append((args[1], kwargs['repeat']))
      super().send(*args, **kwargs)

  writer = Writer(tmp_path)
  monkeypatch.setattr(daemon, 'RUNTIME', tmp_path)
  monkeypatch.setattr(daemon, 'config', lambda: settings)
  monkeypatch.setattr(daemon, 'CommandWriter', lambda: writer)
  monkeypatch.setattr(daemon.time, 'monotonic', lambda: clock[0])
  step = [-1]

  class State(dict):
    alive = dict.fromkeys(('carState', 'deviceState', 'selfdriveState'), True)
    valid = {'carState': True}

    def update(self, _):
      step[0] += 1
      clock[0] = [10, 10.71, 11.22, 11.23, 11.8, 12.4][min(step[0], 5)]
      if step[0] >= 3:
        cs = self['carState']
        cs.brakePressed = stop == 'brake'
        cs.gasPressed = stop == 'gas'
        cs.gearShifter = 'park' if stop == 'gear' else 'drive'
        cs.buttonEvents = [object()] if stop == 'physical' else []
        cs.canValid = stop != 'can'
        self['selfdriveState'].enabled = stop != 'disable'
        self['deviceState'].started = stop != 'offroad'
        self.alive['selfdriveState'] = stop != 'stale'

  state = State(carState=SimpleNamespace(canValid=True, vEgo=0, brakePressed=False, gasPressed=False,
                                       gearShifter='drive', buttonEvents=[]), deviceState=SimpleNamespace(started=True),
                selfdriveState=SimpleNamespace(enabled=True))
  monkeypatch.setattr(daemon.messaging, 'SubMaster', lambda _: state)
  fd, output = os.pipe()
  monkeypatch.setattr(daemon, 'devices', lambda: {} if stop == 'disconnect' and step[0] >= 3 else {'input-0': (mac, 'remote')})
  monkeypatch.setattr(daemon, 'open_input', lambda _: fd)

  class Done(Exception):
    pass

  def select(fds, *_):
    if step[0] >= 5:
      raise Done
    if step[0] == 0 or (step[0] == 3 and stop == 'release'):
      sec = int(clock[0])
      usec = round((clock[0] - sec) * 1e6)
      os.write(output, b''.join(daemon.EVENT.pack(sec, usec, *event) for event in
                               [(1, 115, int(step[0] == 0)), (0, 0, 0)]))
      return fds, [], []
    return [], [], []

  monkeypatch.setattr(daemon.select, 'select', select)
  try:
    with pytest.raises(Done):
      daemon.main()
  finally:
    os.close(output)
  assert sent == [('accelCruiseLong', False), ('accelCruiseLong', True)]
  assert writer.events['cruise'] == []
