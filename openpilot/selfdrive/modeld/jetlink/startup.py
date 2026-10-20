"""Keep USB provisioning alive while a Jetson checks/updates its boot release."""
import json
from pathlib import Path
import time

CAPABILITY = 'carrot_boot_update_v1'
MESSAGE = 0x4003
RELEASE = Path(__file__).with_name('host_release.json')


class RuntimeHandoff(ConnectionError):
  """The boot gate deliberately handed its transport to the selected runtime."""


def initial_connection_wait(error, *, elapsed, ready_once, preparing):
  # Only an unanswered first handshake is ordinary startup. Engine/model
  # errors, unsafe cleanup and failures after a working session remain errors.
  from jetlink.transport.base import LinkError, LinkTimeout
  no_reader = (isinstance(error, LinkError)
               and str(error).startswith(('gadget write had no reader for ',
                                          'gadget write had no reader before deadline;')))
  return (not ready_once and not preparing and 0 <= elapsed < 120
          and (isinstance(error, (TimeoutError, LinkTimeout)) or no_reader))


def wait_for_boot_update(client, peer, wifi, connected, publish, sleep=time.sleep):
  if peer.get('carrot_host') != 'jetson' or peer.get(CAPABILITY) is not True:
    return
  manifest = json.loads(RELEASE.read_text())
  while connected():
    # The signed public release is independent of NetworkManager profile reads.
    # Network provisioning stays on its private channel and out of status/logs.
    client.t.send_json(MESSAGE, client._next_seq(), manifest)
    if wifi is not None:
      wifi.send(client)
    value = client.state().get('carrot_update', {})
    if not isinstance(value, dict):
      value = {}
    publish('updating', peer=peer, host_update=value)
    if value.get('state') == 'ready':
      break
    sleep(.5)
  # The gate has no model. A fresh connection must greet the real server.
  if not connected():
    raise ConnectionError('Jetson disconnected during boot release check')
  raise RuntimeHandoff('Jetson boot gate ended; reconnecting to runtime')
