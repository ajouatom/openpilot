"""Small nl80211 adapter; no iw installation or changes to the station interface."""
import errno
from pathlib import Path
import socket
import struct
import uuid

INTERFACE = 'wljetson0'


def profile_id():
  return str(uuid.uuid5(uuid.NAMESPACE_URL, 'carrot-jetson-direct:' + Path('/etc/machine-id').read_text().strip()))


def attributes(data):
  result, offset = {}, 0
  while offset + 4 <= len(data):
    size, kind = struct.unpack_from('HH', data, offset)
    if size < 4 or offset + size > len(data):
      raise ValueError('Invalid netlink attribute')
    result[kind & 0x3fff] = data[offset + 4:offset + size]
    offset += (size + 3) & ~3
  return result


def attribute(kind, data):
  value = struct.pack('HH', len(data) + 4, kind) + data
  return value + b'\0' * (-len(value) % 4)


def integer(data):
  return struct.unpack('I', data)[0]


def concurrent_station_ap(rows):
  # A supported mode by itself does not authorize simultaneous operation.
  for row in rows:
    for raw in attributes(row.get(120, b'')).values():
      combo = attributes(raw)
      limits = [attributes(v) for v in attributes(combo.get(1, b'')).values()]
      station = [i for i, v in enumerate(limits) if 2 in attributes(v.get(2, b'')) and integer(v[1]) >= 1]
      ap = [i for i, v in enumerate(limits) if 3 in attributes(v.get(2, b'')) and integer(v[1]) >= 1]
      if integer(combo.get(2, b'\0' * 4)) >= 2 and any(i != j or integer(limits[i][1]) >= 2 for i in station for j in ap):
        return True
  return False


class Wireless:
  def __init__(self):
    self.socket = socket.socket(socket.AF_NETLINK, socket.SOCK_RAW, 16)
    self.socket.bind((0, 0))
    self.socket.settimeout(3.)
    self.sequence = 0
    try:
      self.family = struct.unpack('H', self.query(16, 3, payload=attribute(2, b'nl80211\0'))[0][1])[0]
    except BaseException:
      self.close()
      raise

  def close(self):
    self.socket.close()

  def query(self, family, command, flags=1, payload=b''):
    self.sequence += 1
    body = struct.pack('BBH', command, 1, 0) + payload
    self.socket.send(struct.pack('IHHII', len(body) + 16, family, flags, self.sequence, 0) + body)
    result = []
    while True:
      data, offset = self.socket.recv(262144), 0
      while offset + 16 <= len(data):
        size, kind, response_flags, sequence, _ = struct.unpack_from('IHHII', data, offset)
        if size < 16 or offset + size > len(data):
          raise ValueError('Invalid netlink response')
        part = data[offset + 16:offset + size]
        offset += (size + 3) & ~3
        if sequence != self.sequence:
          continue
        if kind == 3:
          return result
        if kind == 2:
          error, = struct.unpack_from('i', part)
          if error:
            raise OSError(-error, errno.errorcode.get(-error, 'netlink'))
          return result
        result.append(attributes(part[4:]))
        if not response_flags & 2 and not flags & 4:
          return result

  def interfaces(self):
    return [{'name': r[4].rstrip(b'\0').decode(), 'index': integer(r[3]), 'type': integer(r[5]), 'phy': integer(r[1])}
            for r in self.query(self.family, 5, 0x301)]

  def power_save(self, index):
    return integer(self.query(self.family, 62, payload=attribute(3, struct.pack('I', index)))[0][93])

  def set_power_save(self, index, enabled):
    if enabled not in (0, 1):
      raise ValueError('Invalid power-save state')
    self.query(self.family, 61, 5, attribute(3, struct.pack('I', index)) + attribute(93, struct.pack('I', enabled)))

  def create(self):
    rows = self.interfaces()
    station = next(r for r in rows if r['name'] == 'wlan0' and r['type'] == 2)
    phys = [r for r in self.query(self.family, 1, 0x301) if integer(r[1]) == station['phy']]
    if not concurrent_station_ap(phys):
      raise ValueError('Concurrent station/AP unavailable')
    if any(r['name'] == INTERFACE for r in rows):
      raise ValueError('Reserved interface already exists')
    payload = attribute(1, struct.pack('I', station['phy'])) + attribute(4, INTERFACE.encode() + b'\0') + attribute(5, struct.pack('I', 3))
    self.query(self.family, 7, 5, payload)
    # NM/firmware may already have changed this newly created vdev to managed
    # while registering it. Ownership is the successful NEW plus this new
    # name/index, not a transient mode. NM's explicit AP activation sets mode.
    return next(r['index'] for r in self.interfaces() if r['name'] == INTERFACE)

  def delete(self, index):
    # Never remove an interface that has been replaced or reused by another owner.
    if any(r['name'] == INTERFACE and r['index'] == index for r in self.interfaces()):
      self.query(self.family, 8, 5, attribute(3, struct.pack('I', index)))
