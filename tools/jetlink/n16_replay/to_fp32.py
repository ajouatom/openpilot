#!/usr/bin/env python3
"""N16 fp16 graph -> fp32 graph for fast, multithreaded ONNX Runtime CPU runs.
usage: to_fp32.py IN.onnx OUT.onnx"""
import sys, numpy as np, onnx
from onnx import numpy_helper, TensorProto

m = onnx.load(sys.argv[1])
g = m.graph
F16, F32 = TensorProto.FLOAT16, TensorProto.FLOAT

def fix_tensor(t):
  if t.data_type == F16:
    a = numpy_helper.to_array(t).astype(np.float32)
    t.CopyFrom(numpy_helper.from_array(a, t.name))
    return 1
  return 0

def fix_type(vi):
  tt = vi.type.tensor_type
  if tt.elem_type == F16:
    tt.elem_type = F32; return 1
  return 0

counts = dict(init=0, const=0, cast=0, io=0, attr=0)
for t in g.initializer: counts['init'] += fix_tensor(t)
for n in g.node:
  for at in n.attribute:
    if at.type == onnx.AttributeProto.TENSOR: counts['const'] += fix_tensor(at.t)
    if n.op_type == 'Cast' and at.name == 'to' and at.i == F16: at.i = F32; counts['cast'] += 1
    if n.op_type in ('ConstantOfShape',) and at.type == onnx.AttributeProto.TENSOR: pass
    if at.name == 'dtype' and at.type == onnx.AttributeProto.INT and at.i == F16: at.i = F32; counts['attr'] += 1
for vi in list(g.input) + list(g.output) + list(g.value_info): counts['io'] += fix_type(vi)
print(counts)
onnx.save(m, sys.argv[2])
print('saved', sys.argv[2])
