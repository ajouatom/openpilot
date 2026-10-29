"""Ray EV calibration from StarPilot 98f84431; never a default for other vehicles."""
import numpy as np

from opendbc.car.pedal_interceptor import PedalInterceptorParams


RAY_PEDAL_COMMAND_CAP = 0.70
RAY_PEDAL_CAP_SPEED_BP = [0.0, 12.0, 20.0, 25.0]
RAY_PEDAL_CAP_V = [0.55, 0.55, 0.65, RAY_PEDAL_COMMAND_CAP]
RAY_PEDAL_OFFSET_SPEED_BP = [0.0, 2.0, 4.0, 8.0, 12.0, 16.0, 20.0, 25.0]
RAY_PEDAL_OFFSET_V = [0.08, 0.13, 0.20, 0.32, 0.42, 0.51, 0.565, 0.60]
RAY_PEDAL_ACCEL_GAIN = 0.22
RAY_PEDAL_DECEL_GAIN = 0.55
RAY_PEDAL_RATE_UP = 0.02
RAY_PEDAL_RATE_DOWN = 0.06
RAY_PEDAL_OVERSPEED_CUTOFF = 0.5
RAY_PEDAL_TAPER_BELOW_TARGET = 0.75


def get_ray_pedal_target(accel: float, v_ego: float, set_speed: float) -> float:
  if not all(np.isfinite(x) for x in (accel, v_ego, set_speed)) or set_speed < 1.0:
    return 0.0

  speed_error = set_speed - v_ego
  if speed_error <= -RAY_PEDAL_OVERSPEED_CUTOFF:
    return 0.0

  pedal_offset = float(np.interp(v_ego, RAY_PEDAL_OFFSET_SPEED_BP, RAY_PEDAL_OFFSET_V))
  pedal_cap = float(np.interp(v_ego, RAY_PEDAL_CAP_SPEED_BP, RAY_PEDAL_CAP_V))
  pedal_gain = RAY_PEDAL_DECEL_GAIN if accel < 0.0 else RAY_PEDAL_ACCEL_GAIN
  target = float(np.clip(pedal_offset + accel * pedal_gain, 0.0, pedal_cap))
  if speed_error < 0.0:
    target *= float(np.clip(0.65 * (1.0 + speed_error / RAY_PEDAL_OVERSPEED_CUTOFF), 0.0, 1.0))
  elif speed_error < RAY_PEDAL_TAPER_BELOW_TARGET:
    target *= 0.65 + 0.35 * speed_error / RAY_PEDAL_TAPER_BELOW_TARGET
  return target


RAY_PEDAL_INTERCEPTOR_PARAMS = PedalInterceptorParams(
  target=get_ray_pedal_target,
  command_factors=(0.672, 0.332), command_offsets=(-177.408, -165.004),
  driver_thresholds=(272, 513), rate_up=RAY_PEDAL_RATE_UP, rate_down=RAY_PEDAL_RATE_DOWN,
  safety_profile=16384,  # HyundaiSafetyFlags.PEDAL_INTERCEPTOR_RAY
  cruise_available=("LABEL11", "CC_React"), cruise_enabled_bit=(0x592, 35),
  cruise_speed=("E_EMS11", "Cruise_Limit_Target"), min_enable_speed=-1.0,
)
