"""Validate remote driving graphs before allocating IPC or parsing their outputs."""
import json
import math
from pathlib import Path
import re

from jetlink.spec import ModelSpec

DEFAULT_SPEC = ModelSpec.from_dict(json.loads(Path(__file__).with_name('cinque_v2.json').read_text()))

# Widths consumed by Carrot's parser, message publisher and action calculation.
OUTPUT_WIDTHS = {
  'lane_lines': (528,), 'lane_lines_prob': (8,), 'road_edges': (264,),
  'meta': (55,), 'desire_pred': (32,), 'pose': (12,),
  'wide_from_device_euler': (6,), 'road_transform': (12,),
  'plan': (990, 4955), 'lead': (144, 102), 'lead_prob': (3,),
  'desire_state': (8,),
}


def slice_range(spec, at):
  if (at.step not in (None, 1) or any(end is not None and type(end) is not int for end in (at.start, at.stop)) or
      any(end is not None and not -spec.output_nelem <= end <= spec.output_nelem for end in (at.start, at.stop))):
    raise ValueError('Invalid Jetlink output slice bounds')
  start, stop, _ = at.indices(spec.output_nelem)
  if start >= stop:
    raise ValueError('Empty Jetlink output slice')
  return start, stop


def validate_model_spec(spec):
  if not isinstance(spec.sha256, str) or re.fullmatch('[0-9a-f]{64}', spec.sha256) is None:
    raise ValueError('Invalid Jetlink model SHA256')
  if type(spec.nbytes) is not int or not 0 < spec.nbytes <= 4 << 30:
    raise ValueError('Invalid Jetlink model size')
  if type(spec.frame_skip) is not int or spec.frame_skip != 4:
    raise ValueError('Unsupported Jetlink model context stride')
  for shapes in (spec.input_shapes, spec.output_shapes):
    for shape in shapes.values():
      if not shape or any(type(n) is not int or n <= 0 for n in shape) or math.prod(shape) > 32 << 20:
        raise ValueError('Invalid or oversized Jetlink tensor shape')
  if spec.input_shapes.get('traffic_convention') != (1, 2) or spec.input_shapes.get('action_t') != (1, 2):
    raise ValueError('Unsupported Jetlink scalar inputs')
  if spec.stateful:
    if spec.input_shapes.get('new_img') != (2, 6, 128, 256) or math.prod(spec.input_shapes.get('desire', ())) != 8:
      raise ValueError('Unsupported Jetlink stateful camera/desire contract')
    scalar_names = {'new_img', 'desire', 'traffic_convention', 'action_t'}
    states = set(spec.input_shapes) - scalar_names
    if not states or any(not name.startswith('state_') or
                         spec.output_shapes.get('next_' + name) != spec.input_shapes[name] for name in states):
      raise ValueError('Unsupported Jetlink graph state contract')
    if set(spec.output_shapes) != {'outputs'} | {'next_' + name for name in states}:
      raise ValueError('Unsupported Jetlink state outputs')
  else:
    required = {'img', 'big_img', 'desire_pulse', 'features_buffer', 'traffic_convention', 'action_t'}
    if set(spec.input_shapes) != required or set(spec.output_shapes) != {'outputs'}:
      raise ValueError('Unsupported Jetlink queued graph inputs/outputs')
    if spec.input_shapes['img'] != (1, 12, 128, 256) or spec.input_shapes['big_img'] != spec.input_shapes['img']:
      raise ValueError('Unsupported Jetlink camera contract')
    dp, fb = spec.input_shapes['desire_pulse'], spec.input_shapes['features_buffer']
    if len(dp) != 3 or dp[0] != 1 or dp[2] != 8 or len(fb) < 3 or fb[0] != 1:
      raise ValueError('Unsupported Jetlink recurrent inputs')
    hidden = spec.output_slices.get('hidden_state')
    if hidden is None or slice_range(spec, hidden)[1] - slice_range(spec, hidden)[0] != spec.feat_dim:
      raise ValueError('Jetlink hidden state does not match recurrent inputs')
  if len(spec.output_shapes['outputs']) != 2 or spec.output_shapes['outputs'][0] != 1 or spec.output_nelem > 65536:
    raise ValueError('Unsupported Jetlink driving output shape')
  ranges = sorted(slice_range(spec, at) for at in spec.output_slices.values())
  if any(left[1] > right[0] for left, right in zip(ranges, ranges[1:], strict=False)):
    raise ValueError('Overlapping Jetlink output slices')
  for name, widths in OUTPUT_WIDTHS.items():
    at = spec.output_slices.get(name)
    if at is None or slice_range(spec, at)[1] - slice_range(spec, at)[0] not in widths:
      raise ValueError(f'Unsupported Jetlink driving output {name}')
  action = spec.output_slices.get('action')
  if action is not None and slice_range(spec, action)[1] - slice_range(spec, action)[0] not in (2, 4):
    raise ValueError('Unsupported Jetlink action output')
  if spec.sha256 == DEFAULT_SPEC.sha256 and spec.to_dict() != DEFAULT_SPEC.to_dict():
    raise ValueError('Jetlink model identity or input/output contract mismatch')
  return spec


def model_name(spec):
  return 'Cinque v2' if spec.sha256 == DEFAULT_SPEC.sha256 else f'App model {spec.sha256[:12]}'


def parse_outputs(parser, spec, result):
  outputs = {name: result[None, at] for name, at in spec.output_slices.items()}
  # Older queued exports use five plan hypotheses; publishing still consumes
  # the selected 33 x 15 plan, as it does for the direct-plan exports.
  if outputs['plan'].shape[1] == 4955:
    parser.parse_mdn('plan', outputs, in_N=5, out_N=1, out_shape=(33, 15))
    plan, stds = outputs.pop('plan'), outputs.pop('plan_stds')
    outputs = parser.parse_vision_outputs(outputs)
    parser.parse_categorical_crossentropy('desire_state', outputs, out_shape=(8,))
    outputs.update(plan=plan, plan_stds=stds)
    return outputs
  return parser.parse_outputs(outputs)
