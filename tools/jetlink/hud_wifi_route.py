"""Choose a real WLAN route, preferring the private AP over an optional uplink."""
import ipaddress
import json
import subprocess

from wifi_direct_netlink import INTERFACE


def wlan_address(value):
  address = ipaddress.ip_address(value)
  return address.version == 4 and address.is_private and not (address.is_loopback or address.is_link_local or address.is_unspecified)


def select_link(network, targets, query=None):
  if query is None:
    def query(target, source):
      return json.loads(subprocess.check_output(['ip', '-j', 'route', 'get', target, 'from', source], timeout=1., text=True))
  for interface, source in sorted(network, key=lambda pair: pair[0] != INTERFACE):
    if not interface.startswith('wl') or not wlan_address(source):
      continue
    for target in targets:
      if not wlan_address(target) or target == source:
        continue
      try:
        route = query(target, source)
        if route and route[0].get('dev') == interface and route[0].get('type', 'unicast') == 'unicast':
          return interface, source, target
      except (OSError, ValueError, TypeError, subprocess.SubprocessError):
        continue
  return None
