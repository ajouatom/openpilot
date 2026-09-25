"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

A driving model's graph metadata: input/output names, shapes and dtypes, plus
openpilot's output_slices and model_checkpoint metadata_props.

Adapter over whichever parser the host has, neither a hard dependency.
tinygrad's OnnxPBParser is preferred and is what openpilot's
get_model_metadata.py uses: it walks the protobuf without materialising 766 MB
of weights, and a comma has it already. The onnx package is the Jetson's, which
needs it for onnx_patch anyway.
"""
from __future__ import annotations

import codecs
import pickle
from dataclasses import dataclass, field

# ONNX TensorProto.DataType -> numpy dtype name
ELEM_TYPE = {
  1: 'float32', 2: 'uint8', 3: 'int8', 4: 'uint16', 5: 'int16', 6: 'int32',
  7: 'int64', 9: 'bool', 10: 'float16', 11: 'float64', 12: 'uint32', 13: 'uint64',
}


@dataclass
class OnnxMeta:
  inputs: dict[str, tuple[int, ...]] = field(default_factory=dict)
  outputs: dict[str, tuple[int, ...]] = field(default_factory=dict)
  input_types: dict[str, str] = field(default_factory=dict)
  output_types: dict[str, str] = field(default_factory=dict)
  props: dict[str, str] = field(default_factory=dict)

  @property
  def output_slices(self) -> dict[str, slice]:
    """openpilot stashes a base64 pickle of the output slice map in metadata_props."""
    raw = self.props.get('output_slices')
    if raw is None:
      raise KeyError("output_slices not in model metadata_props")
    return pickle.loads(codecs.decode(raw.encode(), 'base64'))

  @property
  def model_checkpoint(self) -> str | None:
    return self.props.get('model_checkpoint')


def _parse_tinygrad(path: str) -> OnnxMeta:
  """Same approach as openpilot's get_model_metadata.py, without importing it.

  ModelProto is narrowed to two fields, so the 766 MB of initializers are
  skipped rather than built.
  """
  from tinygrad.nn.onnx import OnnxPBParser

  class _MetaParser(OnnxPBParser):
    def _parse_ModelProto(self) -> dict:
      obj: dict = {"graph": {"input": [], "output": []}, "metadata_props": []}
      for fid, wire_type in self._parse_message(self.reader.len):
        if fid == 7:
          obj["graph"] = self._parse_GraphProto()
        elif fid == 14:
          obj["metadata_props"].append(self._parse_StringStringEntryProto())
        else:
          self.reader.skip_field(wire_type)
      return obj

  model = _MetaParser(path).parse()
  meta = OnnxMeta()
  for key, dest_shape, dest_type in (('input', meta.inputs, meta.input_types),
                                     ('output', meta.outputs, meta.output_types)):
    for vi in model['graph'][key]:
      t = vi['parsed_type']
      dest_shape[vi['name']] = tuple(int(d) if isinstance(d, int) else 0 for d in t.shape)
      dest_type[vi['name']] = str(getattr(t, 'dtype', ''))
  for prop in model['metadata_props']:
    meta.props[prop['key']] = prop['value']
  return meta


def _parse_onnx(path: str) -> OnnxMeta:
  import onnx

  model = onnx.load(path, load_external_data=False)
  meta = OnnxMeta()
  for vis, dest_shape, dest_type in ((model.graph.input, meta.inputs, meta.input_types),
                                     (model.graph.output, meta.outputs, meta.output_types)):
    for vi in vis:
      tt = vi.type.tensor_type
      dest_shape[vi.name] = tuple(d.dim_value for d in tt.shape.dim)
      dest_type[vi.name] = ELEM_TYPE.get(tt.elem_type, f'unknown({tt.elem_type})')
  for p in model.metadata_props:
    meta.props[p.key] = p.value
  return meta


def parse_file(path: str) -> OnnxMeta:
  errors = []
  for name, fn in (('tinygrad', _parse_tinygrad), ('onnx', _parse_onnx)):
    try:
      return fn(path)
    except Exception as e:
      # not just ImportError: a parser that chokes on a newer layout falls
      # through to the other one rather than aborting
      errors.append(f'{name}: {type(e).__name__}: {e}')
  raise RuntimeError(
    "could not read model metadata; need tinygrad or the onnx package. Tried:\n  "
    + "\n  ".join(errors))


def describe(meta: OnnxMeta) -> str:
  lines = ['inputs:']
  lines += [f'  {k:<20} {str(v):<24} {meta.input_types.get(k, "")}' for k, v in meta.inputs.items()]
  lines.append('outputs:')
  lines += [f'  {k:<20} {str(v):<24} {meta.output_types.get(k, "")}' for k, v in meta.outputs.items()]
  lines.append(f'metadata_props: {sorted(meta.props)}')
  return '\n'.join(lines)


def _main() -> None:
  import sys
  m = parse_file(sys.argv[1])
  print(describe(m))
  print('checkpoint:', m.model_checkpoint)
  for k, v in m.output_slices.items():
    print(f'  slice {k:<24} {v}')


if __name__ == '__main__':
  _main()
