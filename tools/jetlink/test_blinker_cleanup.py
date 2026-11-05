"""Source teardown must not consume the model IPC's 150 ms reply deadline."""
import json
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import blink_protocol as protocol


@pytest.fixture
def slow_source():
  # Actual child: EOF revokes its input immediately, but shutdown takes longer
  # than a model reply deadline. No vehicle, cereal, CAN, or USB is involved.
  child = subprocess.Popen([sys.executable, '-u', '-c',
                            "import sys,time; print('ready'); sys.stdin.read(); time.sleep(.4)"],
                           stdin=subprocess.PIPE, stdout=subprocess.PIPE)
  source = object.__new__(protocol.Process)
  source.socket, source.process = child.stdin, child
  try:
    assert child.stdout.readline().strip() == b'ready'
    yield source
  finally:
    child.stdin.close()
    if child.poll() is None:
      child.kill()
    child.wait(timeout=5)
    child.stdout.close()


@pytest.mark.parametrize('cause', ['lease_lost', 'adapter_changed', 'close'])
def test_source_shutdown_does_not_block_bridge_or_reuse_old_commands(monkeypatch, slow_source, cause):
  owner = NS(held=True, name='CANable')
  owner.acquire = lambda ready: setattr(owner, 'held', ready) or ready
  owner.release = lambda: setattr(owner, 'held', False)
  monkeypatch.setattr(protocol, 'Owner', lambda _: owner)
  sent, created = [], []
  state = {'token':'t', 'clock':10., 'available':False, 'idle':True, 'generation':'one'}
  client = NS(last_state={'carrot_blinker':state}, _next_seq=lambda:1,
              _expect=lambda *a:NS(payload=protocol.encode({'ready':True, 'token':'t', 'clock':10.})),
              t=NS(send_json=lambda *a:None, send=lambda *a:sent.append(json.loads(a[2][0]))))
  sender = protocol.Sender(client, {'carrot_host':'jetson', protocol.CAPABILITY:True})
  sender.worker, sender.generation = slow_source, 'one'
  def new_source(*args):
    created.append(args)
    return NS(latest=lambda:None, send=lambda _:True, close=lambda **kw:None, process=NS(poll=lambda:None))
  monkeypatch.setattr(protocol, 'Process', new_source)
  if cause == 'adapter_changed':
    state.update(generation='two', available=True)
  started = time.perf_counter()
  if cause == 'close':
    sender.close()
  else:
    sender.send(client)
  elapsed = time.perf_counter()-started
  assert elapsed < .15, f'source teardown blocked model replies for {elapsed:.3f}s'
  assert slow_source.socket.closed and not owner.held and sender.worker is None
  assert sender.reaper.is_alive() and slow_source.process.poll() is None
  if cause != 'close':
    assert sent == [{'token':'t', 'ok':False, 'events':[]}]

  # Repeated health recovery cannot start another policy while the old source
  # is still being reaped. It also cannot replay its previous command.
  state.update(available=True, clock=11.)
  sender.next_send = 0.
  sender.send(client)
  assert not created and not owner.held
  assert all(not packet['ok'] and not packet['events'] for packet in sent)
  sender.reaper.join(timeout=3)
  assert not sender.reaper.is_alive() and slow_source.process.returncode == 0
  if cause != 'close':
    # A fresh receiver may recover after cleanup; the new source starts empty.
    state['clock'] = 12.
    sender.next_send = 0.
    sender.send(client)
    assert created == [('blink_source.py',)] and owner.held
    assert all(not packet['ok'] and not packet['events'] for packet in sent)
  sender.close()


def test_can_worker_close_still_waits_for_hardware_owner_exit(slow_source):
  slow_source.close()
  assert slow_source.socket.closed and slow_source.process.returncode == 0
