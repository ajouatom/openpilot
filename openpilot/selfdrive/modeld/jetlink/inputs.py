"""Populate the selected Jetlink model's float32 payload for one camera frame."""
import numpy as np


def fill_inputs(views, inputs, prev_desire):
  desire = inputs['desire_pulse'].copy()
  desire[0] = 0
  views['desire'][:] = np.where(desire - prev_desire > .99, desire, 0)
  prev_desire[:] = desire
  for name in ('traffic_convention', 'action_t'):
    views[name][:] = inputs[name]
  # MapInput omits expired/missing route inputs. Clear every frame, including
  # the first frame after a join, so a previous turn/map never persists.
  for name in ('nav_map', 'navigation'):
    if name not in views:
      continue
    views[name].fill(0)
    if name in inputs:
      value = np.asarray(inputs[name])
      accepted = ((256, 256), (1, 1, 256, 256)) if name == 'nav_map' else ((47,), (1, 47))
      if value.shape not in accepted or not np.isfinite(value).all():
        raise ValueError(f'Invalid Jetlink {name} input')
      if name == 'nav_map' and (np.any(value < 0) or np.any(value > 255)):
        raise ValueError('Jetlink map pixels must remain in 0..255')
      # RL nav takes float32 pixels in 0..255; its graph divides by 255.
      views[name][...] = value
