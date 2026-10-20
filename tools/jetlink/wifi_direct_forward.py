"""Internet masquerading for the private Jetson AP on AGNOS.

NetworkManager can activate its shared profile while its nftables backend
fails on an older kernel. Own only one narrowly scoped, commented NAT rule.
Never flush tables, change forwarding policy, or modify the USB interfaces.
"""
import ipaddress
import subprocess

COMMENT = 'carrot-jetson-direct-owned-nat'


def run(args):
  return subprocess.run(args, capture_output=True, timeout=5).returncode


class Sharing:
  def __init__(self, command=run):
    self.command = command
    self.backend = None
    self.rule = None

  def call(self, action):
    return self.command([self.backend, '-w', '2', '-t', 'nat', action, 'POSTROUTING', *self.rule])

  def ensure(self, subnet):
    network = ipaddress.ip_network(subnet, strict=True)
    if network.version != 4 or network.prefixlen != 24 or not network.is_private:
      raise ValueError('Expected a private AP /24 subnet')
    rule = ['-s', str(network), '!', '-d', str(network),
            '-m', 'comment', '--comment', COMMENT, '-j', 'MASQUERADE']
    if self.rule is not None and self.rule != rule:
      raise ValueError('AP subnet changed without sharing cleanup')
    if self.backend is None:
      for candidate in ('iptables', 'iptables-legacy'):
        try:
          available = self.command([candidate, '-w', '2', '-t', 'nat', '-S', 'POSTROUTING']) == 0
        except FileNotFoundError:
          available = False
        if available:
          self.backend = candidate
          break
      else:
        raise RuntimeError('No operational AP NAT backend')
    self.rule = rule
    result = self.call('-C')
    if result == 1:
      if self.call('-A') != 0:
        raise RuntimeError('AP NAT installation failed')
    elif result != 0:
      raise RuntimeError('AP NAT inspection failed')
    if self.call('-C') != 0:
      raise RuntimeError('AP NAT verification failed')

  def close(self):
    if self.rule is None:
      return
    result = self.call('-C')
    if result == 0:
      if self.call('-D') != 0:
        raise RuntimeError('AP NAT cleanup failed')
    elif result != 1:
      raise RuntimeError('AP NAT cleanup inspection failed')
    self.rule = self.backend = None
