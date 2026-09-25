"""Bounded host read-ahead so USB display input overlaps synchronous GPU inference."""
import queue
import threading

from jetlink.transport.base import LinkError, LinkTimeout, Message


class ReadAheadTransport:
  def __init__(self, transport):
    self.transport = transport
    self.queue = queue.Queue(maxsize=2)
    self.stopped = threading.Event()
    self.thread = threading.Thread(target=self._read, name='jetlink-usb-read', daemon=True)
    self.thread.start()

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
          msg = self.transport.recv(timeout=.1)
        except LinkTimeout:
          continue
        # The upstream receive view is valid only until its next recv. Own
        # queued data before reading again while the GPU uses the current input.
        self._put(Message(msg.msg_type, msg.seq, msg.flags, memoryview(bytes(msg.payload))))
    except Exception as exc:
      self._put(LinkError(f'host USB reader failed: {exc}'))

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
