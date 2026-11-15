"""Coordinate native Kick output with external adapters and model lamp isolation."""
from opendbc.car.hyundai.values import HyundaiSafetyFlags
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.stalk_trial import crc
from openpilot.selfdrive.carrot.bluetooth.model import atomic_json, read_json, RUNTIME
from tools.jetlink.blink_owner import Owner  # noqa: TID251 -- shared physical output lease


class KickOutput:
  def __init__(self, cp, params):
    self.configured = (cp.brand == 'hyundai' and not cp.passive and
                       any(cfg.safetyParam & HyundaiSafetyFlags.CANFD_BLINKER_KICK for cfg in cp.safetyConfigs))
    self.params = params
    self.owner = Owner('CANFD Kick')
    self.bus = CanBus(cp).ECAN if self.configured else None
    self.next_update = self.last_journal = 0.
    self.direction = 0
    self.driver_stamp = self.driver_pressed_at = -1.
    self.driver_direction = None
    self.driver_counts = [0, 0]
    self.source_counts = None

  def observe_driver(self, frames, cs, now):
    if not self.configured:
      return
    # USB/BLE can synthesize 0x3C1. Trust this independent lever message only
    # after Kick has acquired the shared lease and drained their commands.
    exclusive = (self.owner.held and 0 <= now - self.owner.renewed < .2 and
                 self.params.get_bool('CanfdBlinkerKick') and self.params.get_bool('IsOnroad') and cs.canValid)
    source_counts = (getattr(cs, 'leftBlinkerStalkCount', 0), getattr(cs, 'rightBlinkerStalkCount', 0))
    # Preserve ordinary lever events while Kick is off or awaiting ownership.
    # Apply deltas so changing the source cannot manufacture a lever press.
    if self.source_counts is None:
      self.driver_counts = list(source_counts)
    elif not exclusive:
      for index in range(2):
        self.driver_counts[index] = (self.driver_counts[index] + source_counts[index] - self.source_counts[index]) % 256
    self.source_counts = source_counts
    if not exclusive or not 0 <= now - self.driver_stamp < .35:
      self.driver_direction = None
    if exclusive:
      for stamp, packets in frames:
        for address, data, bus in packets:
          if address != 0x3C1 or bus != self.bus:
            continue
          if len(data) != 8 or data[0] != crc(data[1:]) or not 0 <= now - stamp / 1e9 < .35:
            self.driver_direction, self.driver_stamp = None, -1.
            continue
          direction = int(bool(data[3] & 0x40)) | (int(bool(data[4] & 1)) << 1)
          # Establish neutral after startup, loss or an invalid frame before
          # accepting a new press. Count even a press/release in one CAN batch.
          if self.driver_direction is not None:
            if direction in (1, 2) and not self.driver_direction & direction:
              index = direction - 1
              self.driver_counts[index] = (self.driver_counts[index] + 1) % 256
              self.driver_pressed_at = now
            self.driver_direction = direction
          elif direction == 0:
            self.driver_direction = 0
          self.driver_stamp = stamp / 1e9
    cs.turnSignalDriverValid = exclusive and self.driver_direction is not None and 0 <= now - self.driver_stamp < .35
    cs.turnSignalDriverDirection = self.driver_direction if cs.turnSignalDriverValid else 0
    cs.leftBlinkerStalkCount, cs.rightBlinkerStalkCount = self.driver_counts

  def update(self, valid, now):
    enabled = self.configured and valid and self.params.get_bool('IsOnroad') and self.params.get_bool('CanfdBlinkerKick')
    if not enabled or now >= self.next_update:
      self.owner.acquire(enabled)
      self.next_update = now + .05
    driver_active = self.driver_direction not in (None, 0) or 0 <= now - self.driver_pressed_at < .2
    return enabled and self.owner.held and not driver_active

  def isolate(self, frames, model, now):
    if not self.configured:
      return frames
    directions = {data[16] >> 5 for addr, data, _ in frames if addr == 0x16A and len(data) == 32}
    direction = 1 if 3 in directions else 2 if 4 in directions else 0
    if not direction:
      return frames
    if self.owner.held:
      try:
        if direction != self.direction or now - self.last_journal >= .1:
          previous = read_json(RUNTIME/'boundary-indicator.json', {})
          previous = previous if isinstance(previous, dict) else {}
          mask = previous.get('mask', 0)
          if type(mask) is not int or mask not in (1, 2, 3) or not 0 <= now - previous.get('time', -100.) < 20:
            mask = 0
          atomic_json(RUNTIME/'boundary-indicator.json', {'time':now, 'direction':direction, 'mask':mask | direction})
          self.direction, self.last_journal = direction, now
        if model is not None and model.meta.indicatorIsolationMask & direction:
          return frames
      except (OSError, TypeError, ValueError):
        pass
    # Wait for modeld to acknowledge ownership before physical lamp feedback can arrive.
    return [frame for frame in frames if frame[0] not in (0x165, 0x16A)]
