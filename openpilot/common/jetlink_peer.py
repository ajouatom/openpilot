"""Conservative peer classification; HELLO is descriptive, not authentication."""
import re

MAC_DEVICE = re.compile(r'(?:coreml|ane(?:-whole)?)-Apple[ _-]M[1-9][0-9]*(?:[ _-](?:Pro|Max|Ultra))?', re.I)
IOS_DEVICE = re.compile(r'(?:coreml|ane(?:-whole)?)-Apple[ _-][AM][1-9][0-9]*(?:[ _-](?:Pro|Max|Ultra))?', re.I)
ANDROID_DEVICE = re.compile(r'(?:htp(?:-whole)?|gpu|npu)-[A-Za-z0-9_.-]+', re.I)


def is_mac_peer(peer, transport='usb'):
  # iOS uses NCM/TCP, including M-series iPads. Never infer Mac from ORT alone.
  return (transport == 'usb' and isinstance(peer, dict)
          and type(peer.get('protocol')) is int and peer['protocol'] in (2, 3)
          and peer.get('carrot_host') in (None, 'mac')
          and peer.get('backend') == 'ort' and isinstance(peer.get('device'), str)
          and MAC_DEVICE.fullmatch(peer['device']) is not None)


def may_provision(peer, mode='usb'):
  """Recognize supported Apps on the selected path, never from power roles."""
  if mode == 'auto':
    return classify_auto_peer(peer, 'usb') in ('usb', 'android')
  if mode == 'usb':
    return is_mac_peer(peer)
  if not (isinstance(peer, dict) and type(peer.get('protocol')) is int and peer['protocol'] == 3
          and peer.get('carrot_host') is None and isinstance(peer.get('device'), str)):
    return False
  if mode == 'ios':
    return peer.get('backend') == 'ort' and IOS_DEVICE.fullmatch(peer['device']) is not None
  if mode == 'android':
    return peer.get('backend') in ('ort', 'litert') and ANDROID_DEVICE.fullmatch(peer['device']) is not None
  return False


def classify_auto_peer(peer, transport):
  """Classify HELLO only after path selection; iPads on TCP are never Macs.

  This is a descriptive capability allowlist, not device authentication or
  evidence that a passive USB host has claimed the FunctionFS endpoints.
  """
  if transport == 'usb':
    if is_mac_peer(peer):
      return 'usb'
    if may_provision(peer, 'android'):
      return 'android'
  elif transport == 'tcp' and may_provision(peer, 'ios'):
    return 'ios'
  return None
