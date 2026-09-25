"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

USB host side of the link: libusb bulk transfers to a FunctionFS gadget.

On a comma+Jetson pair this is the Jetson end. A host needs no kernel driver,
libusb going through usbfs, which is what makes a stripped L4T rootfs workable.
openpilot drives the panda and chestnut through usb1 already.
"""
from __future__ import annotations

import logging
from pathlib import Path

from jetlink import protocol as P
from jetlink.transport.base import LinkError, StreamTransport

# pid.codes test allocation. Get a real PID before distributing this.
JETLINK_VID = 0x1209
JETLINK_PID = 0x0001

# Endpoint addresses are discovered, never assumed: FunctionFS renumbers them at
# bind, so a declared 0x01/0x82 can appear as 0x01/0x81. A hardcoded address
# that does not exist fails with LIBUSB_ERROR_IO and looks like a bad cable.
USB_ENDPOINT_DIR_IN = 0x80
USB_TRANSFER_TYPE_BULK = 0x02
MAX_PACKET = 1024   # SuperSpeed bulk
READ_CHUNK = 256 * MAX_PACKET
DEFAULT_TIMEOUT_MS = 2000

log = logging.getLogger('jetlink.usb')


class UsbBulkTransport(StreamTransport):
  packet_size = MAX_PACKET
  read_chunk = READ_CHUNK
  rx_align = P.GADGET_TX_ALIGN
  # A bulk IN read is posted for whole packets, so the buffer needs a packet of
  # headroom: without it a message that resizes the buffer can end with room for
  # zero packets and stall the read for good.
  read_slack = MAX_PACKET

  def __init__(self, handle, context=None, timeout_ms: int = DEFAULT_TIMEOUT_MS,
               interface: int = 0, ep_in: int = 0x81, ep_out: int = 0x01):
    super().__init__(rx_size=2 << 20)
    self.handle = handle
    self.context = context
    self.timeout_ms = timeout_ms
    self.interface = interface
    self.ep_in = ep_in
    self.ep_out = ep_out
    self._zero_copy_reads = True
    # libusb has no vectored bulk write, so messages are gathered here. Reused
    # so the steady state does not allocate half a megabyte per frame.
    self._tx = bytearray(1 << 20)

  @classmethod
  def open(cls, vid: int = JETLINK_VID, pid: int = JETLINK_PID,
           timeout_ms: int = DEFAULT_TIMEOUT_MS, interface: int = 0) -> UsbBulkTransport:
    import usb1
    context = usb1.USBContext()
    context.open()
    handle = None
    try:
      device = next((d for d in context.getDeviceIterator(skip_on_error=True)
                     if (d.getVendorID(), d.getProductID()) == (vid, pid)), None)
      if device is None:
        raise LinkError(f"no jetlink gadget at {vid:04x}:{pid:04x}")
      ep_in, ep_out = _find_bulk_endpoints(device, interface)
      # An enumerated gadget whose owning process has exited fails open with
      # EIO. Everything here must surface as LinkError, or it escapes the
      # server's accept loop and kills the process instead of retrying.
      handle = device.open()
      handle.claimInterface(interface)
    except LinkError:
      _close_quietly(handle, context)
      raise
    except Exception as e:
      _close_quietly(handle, context)
      raise LinkError(f"could not open {vid:04x}:{pid:04x}: {e}") from e
    return cls(handle, context, timeout_ms, interface, ep_in, ep_out)

  @staticmethod
  def present(vid: int = JETLINK_VID, pid: int = JETLINK_PID) -> bool:
    """Cheap presence check that does not open the device.

    sysfs on Linux, a handful of small reads. Elsewhere libusb enumerates,
    which on macOS and Windows needs no driver for a device nobody has
    claimed; the server polls this every two seconds and that is fine.
    """
    sysfs = Path('/sys/bus/usb/devices')
    if sysfs.is_dir():
      for d in sysfs.glob('*'):
        try:
          if (int((d / 'idVendor').read_text(), 16) == vid
              and int((d / 'idProduct').read_text(), 16) == pid):
            return True
        except (OSError, ValueError):
          pass
      return False
    try:
      import usb1
      with usb1.USBContext() as context:
        return any((d.getVendorID(), d.getProductID()) == (vid, pid)
                   for d in context.getDeviceIterator(skip_on_error=True))
    except Exception as e:
      log.warning("cannot enumerate USB devices: %s", e)
      return False

  def _ms(self, timeout: float | None) -> int:
    return self.timeout_ms if timeout is None else max(1, int(timeout * 1000))

  def _write(self, bufs: list[memoryview]) -> int:
    import usb1
    total = sum(b.nbytes for b in bufs)
    if len(self._tx) < total:
      self._tx = bytearray(max(total, len(self._tx) * 2))
    off = 0
    for b in bufs:
      self._tx[off:off + b.nbytes] = b
      off += b.nbytes
    try:
      # One transfer, not several: multiple writes would let the host scheduler
      # interleave and show up as jitter.
      return self.handle.bulkWrite(self.ep_out, memoryview(self._tx)[:total],
                                   timeout=self._ms(self._write_timeout()))
    except usb1.USBErrorTimeout as e:
      # Report what actually went out so the caller resends only the remainder;
      # claiming zero would duplicate bytes the device already has.
      return getattr(e, 'transferred', 0)
    except usb1.USBError as e:
      raise LinkError(f"usb bulk write failed: {e}") from e

  def _read_into(self, dest: memoryview, timeout: float | None) -> int:
    import usb1
    # _clamp_read rounds to whole packets: a bulk IN whose buffer is not a
    # packet multiple can overflow when the device delivers a full final packet.
    n = self._clamp_read(dest)
    if n == 0:
      return 0
    if self._zero_copy_reads:
      try:
        # bulkRead allocates and copies: ~768 KB allocated and 918 KB memcpy a
        # frame. create_binary_buffer over `dest` writes straight into it.
        buf, _ = usb1.create_binary_buffer(dest[:n])
        return self.handle._bulkTransfer(self.ep_in, buf, n, self._ms(timeout))
      except usb1.USBErrorTimeout as e:
        # libusb attaches whatever did arrive to the exception. Dropping it
        # would desync the stream, far worse than a late frame.
        return getattr(e, 'transferred', 0)
      except usb1.USBError as e:
        raise LinkError(f"usb bulk read failed: {e}") from e
      except (AttributeError, TypeError) as e:
        # A python-libusb1 without the private transfer helper. Fall back for
        # good rather than paying the exception on every read.
        self._zero_copy_reads = False
        log.warning("jetlink: no zero-copy bulk read (%s), using bulkRead", e)

    try:
      data = self.handle.bulkRead(self.ep_in, n, timeout=self._ms(timeout))
    except usb1.USBErrorTimeout as e:
      data = getattr(e, 'received', b'')
    except usb1.USBError as e:
      raise LinkError(f"usb bulk read failed: {e}") from e
    if not data:
      return 0  # zero-length packet: a transfer terminator, not an error
    dest[:len(data)] = data
    return len(data)

  def close(self) -> None:
    for fn in (lambda: self.handle.releaseInterface(self.interface), self.handle.close,
               (self.context.close if self.context is not None else None)):
      if fn is None:
        continue
      try:
        fn()
      except Exception:
        pass


def _close_quietly(handle, context) -> None:
  for closer in (getattr(handle, 'close', None), getattr(context, 'close', None)):
    if closer is None:
      continue
    try:
      closer()
    except Exception:
      pass


def _find_bulk_endpoints(device, interface: int) -> tuple[int, int]:
  """(IN, OUT) bulk endpoint addresses for `interface`, from its descriptors."""
  for cfg in device.iterConfigurations():
    for iface in cfg:
      for setting in iface:
        if setting.getNumber() != interface:
          continue
        ep_in = ep_out = None
        for ep in setting:
          if ep.getAttributes() & 0x03 != USB_TRANSFER_TYPE_BULK:
            continue
          if ep.getAddress() & USB_ENDPOINT_DIR_IN:
            ep_in = ep.getAddress()
          else:
            ep_out = ep.getAddress()
        if ep_in is not None and ep_out is not None:
          return ep_in, ep_out
  raise LinkError(f"interface {interface} has no bulk IN/OUT endpoint pair")
