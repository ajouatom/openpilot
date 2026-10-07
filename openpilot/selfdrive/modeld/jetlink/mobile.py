"""Composite cable transport; manual Android/USB use the existing bulk path.

Daemon API:
  mode = transport_mode()                  # env, /data/jetlink-transport, then auto
  setup_gadget(mode)                       # root helper, BEFORE opening ep0
  owner = FfsTransport(..., gadget=GADGET)  # descriptors and UDC bind, keep alive
  cable = MobileCable(owner).start()       # auto/ios, AFTER bind; network + listener
  transport = cable.accept(timeout=1.)     # TcpTransport, independent of USB padding
  transport.close(); cable.close()         # socket + owned DHCP, not ep0
  owner.close()                            # unbind; retain the NCM function

Never call send/recv on the ep0 owner in iOS mode: that would open bulk endpoints.
Keep the cable alive across accept timeouts; recreate it after a UDC rebind (the
kernel may assign a new netdev). close() deliberately does not close the owner.
USB/android retain the original setup_gadget.sh and do not create a listener.
Android's explicit speed policy belongs to the daemon, not this transport.
HELLO backend/device fields are peer assertions, not authenticated phone identity.
For persistent manual setup, write usb, ios or android plus a newline to
/data/jetlink-transport and reboot offroad. Missing configuration defaults to
auto; existing files remain overrides. Auto uses bounded sequential discovery,
not a claim that USB enumeration identifies a host; see AutoTransport below.
"""
import ipaddress
import logging
import math
import os
from pathlib import Path
import re
import socket
import subprocess
import time

from jetlink.transport.base import LinkTimeout
from jetlink.transport.tcp import TcpTransport

GADGET = Path('/sys/kernel/config/usb_gadget/jetlink')
NCM_FUNCTION = 'ncm.jetlink'
NET_CLASS = Path('/sys/class/net')
UDC_CLASS = Path('/sys/class/udc')
ADDRESS = '192.168.60.1'
NETWORK = ipaddress.IPv4Network('192.168.60.0/24')
PORT = 5599
TOOLS = Path(__file__).resolve().parents[4] / 'tools/jetlink'
MODE_FILE = Path('/data/jetlink-transport')
log = logging.getLogger('carrot.jetlink')


class MobileLinkError(RuntimeError):
  pass


def transport_mode(environ=None, config_path=None):
  """Read env override, optional persistent mode file, then default auto.

  Only a missing file is harmless: unreadable configuration propagates OSError,
  and empty/invalid values fail explicitly. No network or phone inference.
  The iOS root helper must install/verify its exact INPUT isolation rule before
  a normal unprivileged daemon can listen; CAP_NET_RAW is not required.
  """
  environ = os.environ if environ is None else environ
  if 'JETLINK_TRANSPORT' in environ:
    value = environ['JETLINK_TRANSPORT']
  else:
    try:
      value = (MODE_FILE if config_path is None else Path(config_path)).read_text().strip()
    except FileNotFoundError:
      value = 'auto'
  if value not in ('auto', 'usb', 'android', 'ios'):
    raise ValueError(f'Jetlink transport (JETLINK_TRANSPORT or {config_path or MODE_FILE}) must be auto, usb, android or ios, not {value!r}')
  return value


def mode(environ=None, config_path=None):
  """Alias for daemon callers; identical strict configuration policy."""
  return transport_mode(environ, config_path)


def _root(script, *args):
  env = dict(os.environ)
  if script == 'setup_mobile.sh':
    env['JETLINK_TRANSPORT'] = 'ios'
  command = ['bash', str(TOOLS / script), *args]
  if os.geteuid() != 0:
    prefix = ['sudo', '-n']
    if script == 'setup_mobile.sh':
      prefix += ['env', 'JETLINK_TRANSPORT=ios']
    command = [*prefix, *command]
  try:
    subprocess.run(command, env=env, check=True, capture_output=True, text=True, timeout=15)
  except subprocess.CalledProcessError as exc:
    raise MobileLinkError(f'{script} failed: {(exc.stderr or exc.stdout or str(exc)).strip()}') from exc
  except (OSError, subprocess.TimeoutExpired) as exc:
    raise MobileLinkError(f'{script} unavailable: {exc}') from exc


def setup_gadget(mode=None):
  """Run the existing plain setup for USB, or add NCM before ep0/UDC binding."""
  mode = transport_mode() if mode is None else transport_mode({'JETLINK_TRANSPORT': mode})
  if mode in ('auto', 'ios'):
    _root('setup_mobile.sh', 'gadget')
  else:
    if ((GADGET / 'configs' / 'c.1' / NCM_FUNCTION).is_symlink() or
        (GADGET / 'functions' / NCM_FUNCTION).exists()):
      # A retained, detached NCM function requires no further teardown.
      # The helper refuses bound/unowned NCM rather than altering another owner.
      teardown_gadget()
    _root('setup_gadget.sh')
  return mode


def teardown_gadget():
  """Detach helper-owned NCM only for an explicit switch to a plain gadget."""
  _root('setup_mobile.sh', '--teardown')


def _timeout(value):
  if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
    raise ValueError('timeout must be a finite positive number')
  return value


class CableTransport(TcpTransport):
  """TCP framing with cable metadata, but no claim about the peer's identity."""
  tx_align = 0
  rx_align = 0

  def __init__(self, sock, owner, ifname, peer):
    super().__init__(sock)
    self.owner = owner
    self.ifname, self.peer = ifname, peer
    self.tx_align = self.rx_align = 0

  def link_info(self):
    info = {'kind': 'cable', 'transport': 'tcp', 'mode': 'ios', 'interface': self.ifname,
            'local': ADDRESS, 'peer': self.peer[0]}
    udc = getattr(self.owner, 'bound_udc', None)
    if udc:
      try:
        speed = (UDC_CLASS / udc / 'current_speed').read_text().strip()
        if speed in ('super-speed-plus', 'super-speed', 'high-speed', 'full-speed', 'low-speed'):
          info['usb_speed'] = speed
      except OSError:
        pass
    return info


class AutoTransport:
  """Give TCP priority, then authorize one sequential bounded USB probe.

  FunctionFS ENABLE and USB SET_INTERFACE also occur during iOS enumeration.
  A host's local claim/read is not exposed as an ep0 event. The existing FFS
  synchronous write cannot be cancelled without unbinding the entire gadget.
  After the grace period, a probe can interrupt a late phone's enumeration.
  The caller must bound it and rebuild after failure. Automatic wire selection
  tries both versions, then leaves NCM undisturbed for a longer retry window.
  Late Jetson boot/update servers can recover without physical replug. Once a
  TCP dial selects NCM, no further USB probes occur until detach.
  """
  diagnostic = ('Auto pending: TCP gets a 5-second grace period before bounded USB HELLO discovery. ' +
                'Two fresh probes (v3 then v2), then a 30-second NCM grace before retry; failures re-enumerate. No concurrent HELLO.')
  fallback_diagnostic = ('Waiting for NCM TCP before the next USB retry; late USB servers recover without unplugging. ' +
                         'An established NCM selection is never interrupted by USB probing.')
  GRACE = 5.
  RETRY_GRACE = 30.

  def __init__(self, owner, cable, allow_usb=True, grace=None):
    self.owner, self.cable = owner, cable
    self.allow_usb = allow_usb
    self.deadline = time.monotonic() + (self.GRACE if grace is None else grace)

  def accept(self, timeout=1.):
    try:
      return self.cable.accept(timeout=timeout), 'ios'
    except LinkTimeout:
      if self.allow_usb and time.monotonic() >= self.deadline and self.owner._configured():
        self.allow_usb = False
        return self.owner, 'usb'
      raise


class MobileCable:
  """Own the isolated listener/DHCP lifetime while retaining a bare ep0 owner."""
  def __init__(self, owner, gadget=GADGET, net_class=NET_CLASS):
    if transport_mode() not in ('auto', 'ios'):
      raise MobileLinkError('MobileCable requires auto or explicit ios mode via JETLINK_TRANSPORT or /data/jetlink-transport')
    if not getattr(owner, 'bound_udc', None):
      raise MobileLinkError('bind the FunctionFS ep0 owner before starting the iOS network')
    if getattr(owner, 'ep_in', -1) != -1 or getattr(owner, 'ep_out', -1) != -1:
      raise MobileLinkError('iOS must not open FunctionFS bulk endpoints')
    self.owner = owner
    self.gadget, self.net_class = Path(gadget), Path(net_class)
    self.listener = None
    self.ifname = self.ifindex = None
    self._network_started = False

  def _interface(self):
    name = (self.gadget / 'functions' / NCM_FUNCTION / 'ifname').read_text().strip()
    if not re.fullmatch(r'[A-Za-z0-9_.:-]{1,15}', name):
      raise MobileLinkError(f'NCM has no usable interface name: {name!r}')
    index = int((self.net_class / name / 'ifindex').read_text().strip())
    if index <= 0:
      raise MobileLinkError(f'NCM interface {name} has an invalid index')
    return name, index

  def _check_interface(self):
    try:
      current = self._interface()
    except (OSError, ValueError) as exc:
      raise MobileLinkError(f'NCM disappeared; close and restart after binding: {exc}') from exc
    if current != (self.ifname, self.ifindex):
      raise MobileLinkError('NCM changed after a rebind; close and restart the cable listener')

  def start(self, timeout=5.):
    """Wait for the post-bind NCM netdev, configure only it, then listen."""
    timeout = _timeout(timeout)
    if self.listener is not None:
      self._check_interface()
      return self
    deadline = time.monotonic() + timeout
    while True:
      try:
        self.ifname, self.ifindex = self._interface()
        break
      except (OSError, ValueError, MobileLinkError) as exc:
        if time.monotonic() >= deadline:
          raise MobileLinkError(f'no NCM netdev after UDC bind (CONFIG_USB_CONFIGFS_NCM required): {exc}') from exc
        time.sleep(min(.05, max(0., deadline - time.monotonic())))
    try:
      self._network_started = True
      _root('setup_mobile.sh', 'net')
      self._check_interface()
      self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
      # The root helper has installed the exact interface-isolating INPUT rule.
      # SO_BINDTODEVICE is additional protection, not a CAP_NET_RAW requirement
      # on the normal unprivileged comma daemon (notably AGNOS kernel 4.9).
      try:
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, self.ifname.encode() + b'\0')
      except (OSError, AttributeError) as exc:
        log.warning('iOS SO_BINDTODEVICE unavailable (%s); using verified helper-owned INPUT firewall isolation on %s',
                    exc, self.ifname)
      self.listener.bind((ADDRESS, PORT))
      self.listener.listen(1)
    except OSError as exc:
      self.close()
      raise MobileLinkError(f'cannot bind iOS listener at {ADDRESS}:{PORT} on NCM {self.ifname}: {exc}') from exc
    except BaseException:
      self.close()
      raise
    return self

  def accept(self, timeout=1.):
    """Accept only a cable-subnet peer; total timeout also bounds rejected peers."""
    deadline = time.monotonic() + _timeout(timeout)
    if self.listener is None:
      raise MobileLinkError('start the iOS cable listener before accepting')
    while True:
      self._check_interface()
      remaining = deadline - time.monotonic()
      if remaining <= 0:
        raise LinkTimeout('no iOS peer dialed the cable before the accept deadline')
      self.listener.settimeout(remaining)
      try:
        connection, peer = self.listener.accept()
      except TimeoutError as exc:
        raise LinkTimeout('no iOS peer dialed the cable before the accept deadline') from exc
      except OSError as exc:
        raise MobileLinkError(f'iOS cable accept failed: {exc}') from exc
      try:
        self._check_interface()
        address = ipaddress.IPv4Address(peer[0])
        if (address not in NETWORK or address in (NETWORK.network_address, NETWORK.broadcast_address, ipaddress.IPv4Address(ADDRESS))
            or connection.getsockname()[0] != ADDRESS):
          connection.close()
          continue
        return CableTransport(connection, self.owner, self.ifname, peer)
      except BaseException:
        connection.close()
        raise

  def close(self):
    """Close the listener and helper-owned DHCP, leaving ep0/FFS untouched."""
    if self.listener is not None:
      self.listener.close()
      self.listener = None
    if self._network_started:
      _root('setup_mobile.sh', 'net-down')
      self._network_started = False

  def __enter__(self):
    return self.start()

  def __exit__(self, *_):
    self.close()
