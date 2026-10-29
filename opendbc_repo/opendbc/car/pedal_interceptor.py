"""Comma Pedal protocol; vehicle calibration and control targets are supplied by the caller."""
from dataclasses import dataclass
from collections.abc import Callable

PEDAL_INTERCEPTOR_DBC = "pedal_interceptor"
PEDAL_INTERCEPTOR_COMMAND_ADDR = 0x200
PEDAL_INTERCEPTOR_SENSOR_ADDR = 0x201
PEDAL_INTERCEPTOR_BUS = 0
PEDAL_INTERCEPTOR_RX_FREQUENCY = 50
PEDAL_INTERCEPTOR_TX_INTERVAL = 4
PEDAL_INTERCEPTOR_MAX_RX_AGE_NS = 200_000_000


@dataclass(frozen=True)
class PedalInterceptorParams:
  target: Callable[[float, float, float], float]
  dbc_name: str
  driver_thresholds: tuple[int, int]
  rate_up: float
  rate_down: float
  safety_profile: int
  cruise_ready: tuple[str, str]
  cruise_enabled_bit: tuple[int, int]
  cruise_speed: tuple[str, str]
  min_enable_speed: float


@dataclass
class PedalInterceptorState:
  valid: bool = False
  state: int = 5
  gas_pressed: bool = False

  def update(self, parser, params: PedalInterceptorParams):
    timestamp = parser.ts_nanos["GAS_SENSOR"]["STATE"]
    payload = parser.dat.get(PEDAL_INTERCEPTOR_SENSOR_ADDR, b"")
    age = parser._last_update_nanos - timestamp
    self.valid = bool(parser.can_valid and timestamp > 0 and len(payload) == 6 and
                      0 <= age <= PEDAL_INTERCEPTOR_MAX_RX_AGE_NS)
    self.state = int(parser.vl["GAS_SENSOR"]["STATE"]) if self.valid else 5
    self.gas_pressed = self.valid and any(
      int.from_bytes(payload[i * 2:i * 2 + 2], "big") > threshold
      for i, threshold in enumerate(params.driver_thresholds)
    )

  @property
  def faulted(self) -> bool:
    return not self.valid or self.state != 0


def create_pedal_interceptor_command(packer, amount: float, counter: int):
  enabled = amount > 0.001
  values = {"ENABLE": enabled, "COUNTER_PEDAL": counter & 0xF}
  if enabled:
    values["GAS_COMMAND"] = values["GAS_COMMAND2"] = amount * 255.0
  # Leave disabled tracks unset so CANPacker emits raw zero, as in StarPilot.
  # Physical zero would instead encode the DBC offsets into nonzero raw tracks.
  return packer.make_can_msg("GAS_COMMAND", PEDAL_INTERCEPTOR_BUS, values)


class PedalInterceptor:
  def __init__(self, packer, params: PedalInterceptorParams):
    self.packer = packer
    self.params = params
    self.gas_last = 0.0

  def update(self, frame: int, active: bool, accel: float, v_ego: float, set_speed: float):
    if frame % PEDAL_INTERCEPTOR_TX_INTERVAL:
      return None
    if active:
      target = self.params.target(accel, v_ego, set_speed)
      if target <= 0.001:
        self.gas_last = 0.0
      else:
        next_gas = max(self.gas_last - self.params.rate_down, min(target, self.gas_last + self.params.rate_up))
        self.gas_last = min(next_gas, target)
    else:
      self.gas_last = 0.0
    return create_pedal_interceptor_command(self.packer, self.gas_last,
                                          frame // PEDAL_INTERCEPTOR_TX_INTERVAL)
