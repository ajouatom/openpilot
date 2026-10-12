"""Keep USB provisioning alive while a Jetson checks/updates its boot release."""
from pathlib import Path
import time
from openpilot.common.jetson_model import selected_release

CAPABILITY = 'carrot_boot_update_v1'
MESSAGE = 0x4003
RELEASE = Path(__file__).with_name('host_release.json')


def wait_for_boot_update(client, peer, wifi, connected, publish, sleep=time.sleep):
  if peer.get('carrot_host') != 'jetson' or peer.get(CAPABILITY) is not True:
    return
  manifest = selected_release(RELEASE, required=True)
  if manifest.get('delivery') == 'local' and peer.get('carrot_local_release_v1') is not True:
    raise ValueError('Install the PC-prepared ResAction package on Jetson first')
  while connected():
    # The release selection is independent of NetworkManager profile reads.
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
  raise ConnectionError('Jetson boot gate ended; reconnecting to runtime')
