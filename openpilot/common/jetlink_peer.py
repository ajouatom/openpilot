"""Conservative peer classification; HELLO is descriptive, not authentication."""
import re

MAC_DEVICE = re.compile(r'(?:coreml|ane(?:-whole)?)-Apple[ _-]M[1-9][0-9]*(?:[ _-](?:Pro|Max|Ultra))?', re.I)


def is_mac_peer(peer, transport='usb'):
  # iOS uses NCM/TCP, including M-series iPads. Never infer Mac from ORT alone.
  return (transport == 'usb' and isinstance(peer, dict)
          and type(peer.get('protocol')) is int and peer['protocol'] == 2
          and peer.get('carrot_host') in (None, 'mac')
          and peer.get('backend') == 'ort' and isinstance(peer.get('device'), str)
          and MAC_DEVICE.fullmatch(peer['device']) is not None)
