"""Bounded host read-ahead so USB display input overlaps synchronous GPU inference."""
import queue
import threading

from jetlink.transport.base import LinkError, LinkTimeout, Message
from jetlink import protocol as P, request_chunks


class ReadAheadTransport:
  def __init__(self, transport):
    self.transport = transport
    self.queue = queue.Queue(maxsize=2)
    self.requests = request_chunks.Receiver()
    self.last_received_seq = 0
    self.stopped = threading.Event()
    from jetlink.transport.usbbulk import UsbBulkTransport
    self.bulk_input = None
    if isinstance(transport, UsbBulkTransport):
      from host_usb import BulkInput
      self.bulk_input = BulkInput(transport, self.stopped)
      self.raw_read = transport._read_into
      transport._read_into = self.bulk_input.read
    self.thread = threading.Thread(target=self._read, name='jetlink-usb-read', daemon=True)
    self.thread.start()

  @property
  def protocol_version(self):
    return self.transport.protocol_version

  def _put(self, item):
    while not self.stopped.is_set():
      try:
        self.queue.put(item, timeout=.05)
        return
      except queue.Full:
        pass

  def _read(self):
    try:
      while not self.stopped.is_set():
        try:
          msg = self.transport.recv(timeout=None if self.bulk_input is not None else .1)
        except LinkTimeout:
          continue
        if self.bulk_input is not None:
          self.bulk_input.note_message(msg)
        # Keep the original two *logical* messages available while inference
        # runs. Counting small envelopes here would block the writer mid-HUD.
        if msg.msg_type == P.Msg.HELLO_REQ:
          self.requests.clear()
          self.last_received_seq = 0
        elif msg.msg_type == request_chunks.MESSAGE:
          if self.protocol_version != 2:
            raise LinkError('chunked requests require protocol 2')
          msg = self.requests.accept(msg, self.last_received_seq)
          if msg is None:
            continue
        elif self.requests.key is not None:
          if msg.seq <= self.last_received_seq:
            continue
          raise LinkError('interrupted chunked request')
        self.last_received_seq = max(self.last_received_seq, msg.seq)
        # The upstream receive view is valid only until its next recv. Own
        # queued data before reading again while the GPU uses the current input.
        self._put(Message(msg.msg_type, msg.seq, msg.flags, memoryview(bytes(msg.payload))))
    except Exception as exc:
      self._put(LinkError(f'host USB reader failed: {exc}'))
    finally:
      self.requests.clear()

  def recv(self, timeout=None):
    try:
      item = self.queue.get(timeout=.2 if timeout is None else timeout)
    except queue.Empty as exc:
      raise LinkTimeout('host read-ahead queue idle') from exc
    if isinstance(item, Exception):
      raise item
    return item

  def send(self, *args, **kwargs):
    return self.transport.send(*args, **kwargs)

  def close(self):
    self.stopped.set()
    self.thread.join(timeout=1)
    if self.thread.is_alive():
      raise LinkError('host reader did not stop before transport teardown')
    if self.bulk_input is not None:
      self.bulk_input.close()
      # Upstream drains an already-desynced stream after session.close().
      # Restore its raw I/O only after the async reader has relinquished it.
      self.transport._read_into = self.raw_read
      self.bulk_input = None
