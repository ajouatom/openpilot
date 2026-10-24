"""Display-only warning from fresh BSD flags, independent of turn signals."""
import math

PULSE_HZ = 2.0
PULSE_DUTY = .6
DIM_ALPHA = 96
GRADIENT_ALPHA = 64


def edge_alpha(now, started):
  """Immediate bright onset, then 2 Hz; the warning never disappears entirely."""
  if not all(type(v) in (int, float) and math.isfinite(v) for v in (now, started)) or now < started:
    return 255
  return 255 if ((now - started) * PULSE_HZ) % 1 < PULSE_DUTY else DIM_ALPHA


def warning_sides(state, snapshot, now):
  if snapshot is None:
    return False, False
  try:
    received, record = snapshot
    sent = record['sent']
    car_received = record['received']['carState']
    ages = (now - received, sent - car_received)
    if (not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                for v in (now, received, sent, car_received))
        or not all(0 <= age <= .3 for age in ages) or sum(ages) > .3
        or record['valid'].get('carState') is not True
        or record['alive'].get('carState') is not True):
      return False, False
    # carState already retains OEM BSD and ORs fresh optional ONNX results.
    # Neither a turn signal nor an available ONNX service gates this display.
    return state.left_blindspot is True, state.right_blindspot is True
  except (KeyError, TypeError, ValueError, AttributeError, OverflowError):
    return False, False
