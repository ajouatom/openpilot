"""Coordinate native Kick output with external adapters and model lamp isolation."""
from opendbc.car.hyundai.values import HyundaiSafetyFlags
from openpilot.selfdrive.carrot.bluetooth.model import atomic_json, read_json, RUNTIME
from tools.jetlink.blink_owner import Owner  # noqa: TID251 -- shared physical output lease


class KickOutput:
  def __init__(self, cp, params):
    self.configured = (cp.brand == 'hyundai' and not cp.passive and
                       any(cfg.safetyParam & HyundaiSafetyFlags.CANFD_BLINKER_KICK for cfg in cp.safetyConfigs))
    self.params = params
    self.owner = Owner('CANFD Kick')
    self.next_update = self.last_journal = 0.
    self.direction = 0

  def update(self, valid, now):
    enabled = self.configured and valid and self.params.get_bool('IsOnroad') and self.params.get_bool('CanfdBlinkerKick')
    if not enabled or now >= self.next_update:
      self.owner.acquire(enabled)
      self.next_update = now + .05
    return enabled and self.owner.held

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
