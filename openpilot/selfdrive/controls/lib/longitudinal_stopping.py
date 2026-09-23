"""Earlier stop intent for Hyundai CAN FD openpilot longitudinal control."""
import numpy as np

from opendbc.car.hyundai.stopping import ENTRY_SPEED, STOP_SPEED


STOP_PREVIEW_TIME = 1.0


def should_prepare_stop(speeds, accels, t_idxs, *, v_ego, action_t, v_stop):
  """Anticipate an approaching stop without changing the acceleration target.

  Require a decelerating low-speed trajectory that remains near zero at two
  future samples. A launch, steady crawl, or distant high-speed stop must not
  acquire an early stop request. This is an experimental planning rule, not an
  ECU acceptance threshold.
  """
  if len(speeds) != len(t_idxs) or len(accels) != len(t_idxs) or len(t_idxs) < 2:
    return False
  if not (np.isfinite([v_ego, action_t, v_stop]).all() and np.isfinite(speeds).all()
          and np.isfinite(accels).all() and np.isfinite(t_idxs).all()):
    return False
  if not 0.0 <= v_ego <= ENTRY_SPEED or action_t < 0.0 or v_stop <= 0.0 or accels[0] >= 0.0:
    return False
  if np.any(np.diff(t_idxs) <= 0.0):
    return False

  stop_t = min(action_t + STOP_PREVIEW_TIME, t_idxs[-1] - 1.0)
  if stop_t <= action_t:
    return False
  # Do not anticipate a stop beyond an intervening planned acceleration.
  sample_times = [action_t, *[t for t in t_idxs if action_t < t < stop_t + 1.0], stop_t + 1.0]
  planned_speeds = np.interp(sample_times, t_idxs, speeds)
  if planned_speeds[0] > speeds[0] or np.any(np.diff(planned_speeds) > 1e-3):
    return False

  threshold = min(v_stop, STOP_SPEED)
  return bool(np.interp(stop_t, t_idxs, speeds) < threshold and planned_speeds[-1] < threshold)
