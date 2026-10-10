"""Differentiable original ONNX vision tail and temporal policy for offline trials.

Starts at the original 512x4x8 spatial map, before final vision convolution,
squeeze/excitation and pooling. No newly invented runtime architecture.
"""
import numpy as np
import onnx
from onnx import numpy_helper, helper
import torch
from torch import nn
from torch.nn import functional as F


class OriginalWeights(nn.Module):
  def __init__(self, arrays, names):
    super().__init__()
    self.names = {name: f'w{i}' for i, name in enumerate(sorted(names))}
    self.static = {k: arrays[k] for k in names}
    for name, key in self.names.items():
      value = arrays[name].copy()
      if value.dtype.kind == 'f':
        self.register_parameter(key, nn.Parameter(torch.from_numpy(value.astype(np.float32)), requires_grad=False))
      else:
        self.register_buffer(key, torch.from_numpy(value))

  def w(self, name):
    return getattr(self, self.names[name])

  def export_values(self):
    return {name: self.w(name).detach().cpu().numpy().astype(self.static[name].dtype) for name in self.names}


class VisionTail(OriginalWeights):
  def __init__(self, graph, arrays):
    start = next(i for i, n in enumerate(graph.node) if n.output[0] == 'vision/conv2d_57')
    stop = next(i for i, n in enumerate(graph.node) if n.output[0] == 'vision/outputs')
    nodes = list(graph.node[start:stop + 1])
    names = {k for n in nodes for k in n.input if k in arrays}
    super().__init__(arrays, names)
    self.nodes = [(n.op_type, list(n.input), n.output[0], {a.name: helper.get_attribute_value(a) for a in n.attribute}) for n in nodes]

  def forward(self, spatial):
    env = {name: self.w(name) for name in self.names}
    env['vision/add_11'] = spatial
    for op, inputs, output, attr in self.nodes:
      x = [env[name] for name in inputs]
      if op == 'Conv':
        pads = attr.get('pads', [0, 0, 0, 0])
        assert pads[:2] == pads[2:]
        y = F.conv2d(x[0], x[1], x[2], stride=tuple(attr.get('strides', [1, 1])),
                     padding=tuple(pads[:2]), dilation=tuple(attr.get('dilations', [1, 1])), groups=attr.get('group', 1))
      elif op == 'Gemm':
        assert attr.get('transB', 0) == 1 and attr.get('transA', 0) == 0
        assert attr.get('alpha', 1) == 1 and attr.get('beta', 1) == 1
        y = F.linear(*x)
      elif op == 'Relu':
        y = F.relu(x[0])
      elif op == 'Gelu':
        y = F.gelu(x[0], approximate=attr.get('approximate', b'none').decode())
      elif op == 'Sigmoid':
        y = torch.sigmoid(x[0])
      elif op == 'Add':
        y = x[0] + x[1]
      elif op == 'Mul':
        y = x[0] * x[1]
      elif op == 'Div':
        y = x[0] / x[1]
      elif op in ('ReduceMean', 'ReduceL2'):
        axes = tuple(int(i) for i in self.static[inputs[1]])
        y = x[0].mean(axes, keepdim=bool(attr.get('keepdims', 1))) if op == 'ReduceMean' else torch.linalg.vector_norm(x[0], dim=axes, keepdim=bool(attr.get('keepdims', 1)))
      elif op == 'Reshape':
        shape = self.static[inputs[1]].tolist()
        shape[0] = spatial.shape[0]
        y = x[0].reshape(shape)
      elif op == 'Clip':
        y = x[0].clamp_min(float(self.static[inputs[1]]))
      elif op == 'Expand':
        shape = self.static[inputs[1]].tolist()
        shape[0] = spatial.shape[0]
        y = x[0].expand(shape)
      elif op == 'Concat':
        y = torch.cat(x, dim=attr['axis'])
      else:
        raise NotImplementedError((op, output))
      env[output] = y
    return env['vision/outputs']


class TemporalPolicy(OriginalWeights):
  def __init__(self, graph, arrays):
    names = {k for n in graph.node if n.name.startswith('policy/') for k in n.input if k in arrays}
    super().__init__(arrays, names)
    self.attrs = {n.name: {a.name: helper.get_attribute_value(a) for a in n.attribute} for n in graph.node}
    assert np.array_equal(arrays['policy/val_6'].ravel(), np.arange(-9, 0))
    assert int(arrays['policy/val_65']) == 8
    assert tuple(arrays['policy/val_36'][1:]) == (9, 3, 8, 64)
    self.p = 'policy/policy_model.'

  def linear(self, x, name):
    return F.linear(x, self.w(self.p + name + '.weight'), self.w(self.p + name + '.bias'))

  def transposed(self, x, value, bias):
    return x @ self.w('policy/' + value) + self.w(self.p + bias + '.bias')

  def residual(self, x, prefix):
    x = F.relu(x + self.linear(F.relu(self.linear(x, prefix + '.block_a.0')), prefix + '.block_a.3'))
    return F.relu(x + self.linear(F.relu(self.linear(x, prefix + '.block_b.1')), prefix + '.block_b.4'))

  def forward(self, hidden, desire, traffic):
    # Exactly the final9 of the25 original history slots: -32,-28,...,-4,0 frames.
    x = self.transposed(F.silu(self.transposed(hidden, 'val_12', 'temporal_summarizer._feats_encode.0')), 'val_15', 'temporal_summarizer._feats_encode.2')
    d = self.linear(F.silu(self.linear(desire.flatten(1), 'temporal_summarizer._desire_encode.0')), 'temporal_summarizer._desire_encode.2')
    c = self.linear(F.silu(self.linear(traffic, 'temporal_summarizer._traffic_encode.0')), 'temporal_summarizer._traffic_encode.2')
    x = x + self.w('policy/unsqueeze_2') + d[:, None] + c[:, None]
    prefix = 'temporal_summarizer.transformer.0.'
    norm = lambda y, part: F.layer_norm(y, (512,), self.w(self.p + prefix + part + '.layer_norm.weight'), self.w(self.p + prefix + part + '.layer_norm.bias'), 1e-5)
    qkv = self.transposed(norm(x, 'attn'), 'val_28', prefix + 'attn.c_attn')
    q, k, v = qkv.reshape(-1, 9, 3, 8, 64).permute(2, 0, 3, 1, 4).unbind(0)
    attention = (q @ k.transpose(-2, -1)) * self.w('policy/scalar_tensor_default')
    attention = attention.masked_fill(self.w('policy/bitwise_not'), float('-inf')).softmax(-1)
    y = (attention @ v).transpose(1, 2).reshape(-1, 9, 512)
    x = x + self.transposed(y, 'val_57', prefix + 'attn.c_proj')
    y = self.transposed(norm(x, 'mlp'), 'val_61', prefix + 'mlp.c_fc')
    y = F.gelu(y, approximate=self.attrs['policy/node_gelu'].get('approximate', b'none').decode())
    x = x + self.transposed(y, 'val_63', prefix + 'mlp.c_proj')
    h = self.residual(x[:, 8], 'temporal_hydra.resblock')
    results = []
    for head in ('plan', 'desire_state'):
      a = F.relu(self.linear(h, 'temporal_hydra.in_layer.' + head))
      a = F.relu(a + self.linear(F.relu(self.linear(a, 'temporal_hydra.res_layer.' + head + '.0')), 'temporal_hydra.res_layer.' + head + '.2'))
      result = self.linear(a, 'temporal_hydra.final_layer.' + head)
      if head == 'plan':
        result = result * self.w(self.p + 'temporal_hydra.scale_layer.plan.scale')
      results.append(result)
    results.append(self.w('policy/pad').expand(hidden.shape[0], -1))
    return torch.cat(results, dim=1)


class VisionTrial(nn.Module):
  def __init__(self, source):
    super().__init__()
    model = onnx.load(source)
    arrays = {x.name: numpy_helper.to_array(x).copy() for x in model.graph.initializer}
    self.vision = VisionTail(model.graph, arrays)
    self.policy = TemporalPolicy(model.graph, arrays)
    self.color = nn.Linear(512, 2)

  def configure(self, temporal=False):
    for name in self.vision.names:
      self.vision.w(name).requires_grad_(name.startswith('vision/vision_model.vision._en.final_conv.') or name.startswith('vision/vision_model.vision._en.head.fc.'))
    for name in self.policy.names:
      w = self.policy.w(name)
      if not w.is_floating_point():
        continue
      train = temporal and (name in {'policy/val_12', 'policy/val_15', 'policy/val_28', 'policy/val_57', 'policy/val_61', 'policy/val_63'} or '.temporal_summarizer.' in name)
      w.requires_grad_(train)

  def forward(self, maps, desire, traffic):
    batch, history = maps.shape[:2]
    assert history == 9
    vision = self.vision(maps.flatten(0, 1)).reshape(batch, history, 1576)
    hidden = vision[:, :, 1064:1576]
    policy = self.policy(hidden, desire, traffic)
    return torch.cat([vision[:, -1], policy], dim=1), self.color(hidden[:, -1]), hidden

  def export(self, source, destination):
    model = onnx.load(source)
    values = {**self.vision.export_values(), **self.policy.export_values()}
    changed = []
    for initializer in model.graph.initializer:
      name = initializer.name
      if name in values:
        if not np.array_equal(values[name], numpy_helper.to_array(initializer)):
          assert np.isfinite(values[name]).all(), name
          changed.append(name)
        initializer.CopyFrom(numpy_helper.from_array(values[name], name))
    onnx.checker.check_model(model)
    onnx.save(model, destination)
    return changed
