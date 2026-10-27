"""Negotiate a display-only WLAN side channel once over the existing USB session."""
import json
import os
from pathlib import Path
import secrets
import socket
import subprocess
import sys

CAPABILITY = 'carrot_hud_wifi_v1'
USB_CAPABILITY = 'carrot_hud_usb_v2'
REQUEST, RESPONSE = 0x4020, 0x4021
PORT = 7716


class Worker:
  def __init__(self, role, key):
    self.socket, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    self.process = None
    try:
      self.process = subprocess.Popen(
        [sys.executable, str(Path(__file__).with_name('hud_wifi_worker.py')), role, str(child.fileno())],
        pass_fds=(child.fileno(),), stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
      self.socket.settimeout(2.)
      self.socket.send(json.dumps({'key': key, 'port': PORT}).encode())
      if self.socket.recv(64) != b'ready':
        raise RuntimeError('HUD Wi-Fi worker did not initialize')
    except BaseException:
      self.close()
      raise
    finally:
      child.close()

  def close(self):
    self.socket.close()
    if self.process is not None:
      try:
        self.process.wait(timeout=1.)
      except subprocess.TimeoutExpired:
        self.process.terminate()
        try:
          self.process.wait(timeout=1.)
        except subprocess.TimeoutExpired:
          self.process.kill()
          self.process.wait(timeout=1.)


class WifiPublisher:
  """No per-frame USB work or WLAN waits in the inference owner."""
  def __init__(self, client):
    self.worker = None
    key = secrets.token_hex(32)
    seq = client._next_seq()
    client.t.send_json(REQUEST, seq, {'key': key})
    result = json.loads(bytes(client._expect(RESPONSE, seq, 3.).payload))
    if result != {'ready': True}:
      raise RuntimeError('HUD Wi-Fi receiver unavailable')
    self.worker = Worker('send', key)

  def packet(self):
    return None

  def media_packet(self):
    return None

  def close(self):
    if self.worker is not None:
      self.worker.close()


def publisher(client, peer, legacy):
  if peer.get('carrot_host') != 'jetson':
    return legacy(navi=bool(peer.get('carrot_navi_v1'))) if peer.get('carrot_hud_v1') else None
  if peer.get(USB_CAPABILITY) is True:
    # Explicit new receiver contract; older Wi-Fi-only hosts must never be
    # sent USB display traffic just because their Wi-Fi connection failed.
    if not all(peer.get(key) is True for key in ('carrot_hud_v1', 'carrot_navi_v1', 'carrot_navi_pump_v1')):
      return None
    return legacy(navi=True, nav_map=True)
  if peer.get(CAPABILITY) is not True:
    return None  # Old Jetson runtimes must be updated; never send display over USB.
  try:
    return WifiPublisher(client)
  except Exception:
    # Do not log the control payload/key or silently put HUD back onto USB.
    import logging
    logging.getLogger(__name__).warning('HUD Wi-Fi setup failed; model link unchanged, reconnect to retry')
    return None


def supported():
  return Path('/etc/nv_tegra_release').is_file() and os.name == 'posix'
