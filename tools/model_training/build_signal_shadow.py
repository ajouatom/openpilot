#!/usr/bin/env python3
"""Build an offline comparison graph, preserving every original control output.

Requires onnx/numpy on the preparation PC. Neither model is committed by this tool.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import onnx
from onnx import helper, numpy_helper


def build(base_path, candidate_path, output_path):
  base = onnx.load(base_path)
  candidate = onnx.load(candidate_path)
  prefix = 'policy/policy_model.temporal_hydra.'
  weight_name = prefix + 'final_layer.plan.weight'
  bias_name = prefix + 'final_layer.plan.bias'
  scale_name = prefix + 'scale_layer.plan.scale'
  left = {i.name: numpy_helper.to_array(i) for i in base.graph.initializer}
  right = {i.name: numpy_helper.to_array(i) for i in candidate.graph.initializer}
  assert left.keys() == right.keys(), 'initializer names differ'
  rows = np.array([15 * t + c for t in range(33) for c in (0, 3, 6)])
  keep = np.setdiff1d(np.arange(990), rows)
  assert left[weight_name].shape == (990, 256)
  for name in left:
    if name in (weight_name, bias_name):
      assert np.array_equal(left[name][keep], right[name][keep]), 'candidate changed non-longitudinal output rows'
    else:
      assert np.array_equal(left[name], right[name]), f'candidate changed shared initializer: {name}'
  for attribute in ('node', 'input', 'output'):
    assert [n.SerializeToString() for n in getattr(base.graph, attribute)] == [n.SerializeToString() for n in getattr(candidate.graph, attribute)]
  assert len(base.graph.output) == 1
  output = base.graph.output[0]
  assert [d.dim_value for d in output.type.tensor_type.shape.dim] == [1, 2576]
  output_name = output.name
  assert not any(output_name in n.input for n in base.graph.node)
  producer = next(n for n in base.graph.node if output_name in n.output)
  for i, name in enumerate(producer.output):
    if name == output_name:
      producer.output[i] = 'signal_shadow/base_outputs'
  final = next(n for n in base.graph.node if weight_name in n.input)
  assert final.op_type == 'Gemm'
  feature_name = final.input[0]
  for name, value in [('weight', right[weight_name][rows]), ('bias', right[bias_name][rows]), ('scale', right[scale_name][rows])]:
    assert np.isfinite(value).all()
    base.graph.initializer.append(numpy_helper.from_array(value.copy(), 'signal_shadow/' + name))
  base.graph.node.extend([
    helper.make_node('Gemm', [feature_name, 'signal_shadow/weight', 'signal_shadow/bias'], ['signal_shadow/raw'],
                     name='signal_shadow/linear', alpha=1.0, beta=1.0, transA=0, transB=1),
    helper.make_node('Mul', ['signal_shadow/raw', 'signal_shadow/scale'], ['signal_shadow/longitudinal'], name='signal_shadow/scaled'),
    helper.make_node('Concat', ['signal_shadow/base_outputs', 'signal_shadow/longitudinal'], [output_name],
                     name='signal_shadow/output', axis=1),
  ])
  output.type.tensor_type.shape.dim[1].dim_value = 2675
  # Cut at the original camera encoder's exposed hidden state. This artifact is
  # NEVER compiled into the controlling QCOM model; a separate CPU worker uses it.
  for name, size in [('current_hidden_flat', 512), ('policy/mul_1', 990), ('signal_shadow/longitudinal', 99)]:
    base.graph.value_info.append(helper.make_tensor_value_info(name, onnx.TensorProto.FLOAT16, [1, size]))
  base = onnx.utils.Extractor(base).extract_model(
    ['current_hidden_flat', 'features_buffer', 'desire_pulse', 'traffic_convention', 'action_t'],
    ['policy/mul_1', 'signal_shadow/longitudinal'])
  metadata = {'version': 2, 'mode': 'comparison_only', 'base_size': 2576, 'shadow_size': 99,
              'base_sha256': hashlib.sha256(Path(base_path).read_bytes()).hexdigest(),
              'candidate_sha256': hashlib.sha256(Path(candidate_path).read_bytes()).hexdigest()}
  properties = {p.key: p.value for p in base.metadata_props}
  properties['signal_shadow'] = json.dumps(metadata, sort_keys=True)
  helper.set_model_props(base, properties)
  onnx.checker.check_model(base, full_check=True)
  onnx.save(base, output_path)
  Path(str(output_path) + '.json').write_text(json.dumps(metadata, indent=2))
  return metadata


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--base', required=True)
  parser.add_argument('--candidate', required=True)
  parser.add_argument('--output', required=True)
  args = parser.parse_args()
  print(json.dumps(build(args.base, args.candidate, args.output), indent=2))
