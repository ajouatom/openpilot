"""Keep USB provisioning alive while a Jetson checks/updates its boot release."""
import json
from pathlib import Path
import time

CAPABILITY = 'carrot_boot_update_v1'
MESSAGE = 0x4003
RELEASE = Path(__file__).with_name('host_release.json')


def wait_for_legacy_update(client, peer, wifi, params, connected, publish, sleep=time.sleep):
  from openpilot.common.jetson_maintenance import PENDING, migrated
  if migrated(peer):
    if params.get_bool(PENDING):
      params.put_bool(PENDING, False)
    return
  manifest = json.loads(RELEASE.read_text()) if params.get_bool(PENDING) else None
  while params.get_bool(PENDING) and connected():
    if wifi is not None:
      wifi.send(client)
    if peer.get('carrot_hud_v1') is True:
      # Older updaters only read the HUD snapshot. Send a tiny, genuine road
      # state + signed pin; no preview worker or camera allocation is needed.
      onroad = params.get_bool('IsOnroad') or not params.get_bool('IsOffroad')
      client.t.send_json(0x4000, client._next_seq(), {
        'version': 1, 'params': {'IsOnroad': 'MQ==' if onroad else 'MA=='},
        'jetson_release': manifest, 'events': {}, 'received': {}, 'mono': {}, 'valid': {}, 'alive': {},
      })
    client.state()
    publish('maintenance', peer=peer)
    sleep(.2)


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
  raise ConnectionError('Jetson boot gate ended; reconnecting to runtime')
