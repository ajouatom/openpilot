"""Validated pinned-host feedback buffers for serial TensorRT execution."""


def feedback_sources(inputs, outputs, pairs):
  sources = {}
  for name, output in pairs.items():
    target, source = inputs[name], outputs[output]
    if target.shape != source.shape or target.dtype != source.dtype:
      raise ValueError(f'State feedback contract mismatch: {name}')
    sources[name] = source
  return sources
