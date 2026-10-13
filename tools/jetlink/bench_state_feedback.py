#!/usr/bin/env python3
"""Offline A/B/A of pinned-host versus GPU state copies. Never opens USB.

Not imported by the inference server. Requires an existing plan, checks all
outputs byte-for-byte, keeps all CPU finiteness checks, and refuses to run
while the installed inference service is active. Does not change model/plan,
services, clocks, persistent configuration or a release symlink.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

ROOT = next((p for p in Path(__file__).resolve().parents if (p / 'third_party/jetlink/jetlink/spec.py').is_file()), None)
if ROOT is not None:
  sys.path.insert(0, str(ROOT / 'third_party/jetlink'))
from jetlink.queues import StatefulQueues
from jetlink.spec import ModelSpec


def require_stopped_service(run=subprocess.run):
  result = run(['systemctl', 'show', 'carrot-jetlink.service', '-p', 'LoadState', '-p', 'ActiveState'],
               check=True, capture_output=True, text=True, timeout=5)
  fields = dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)
  if fields.get('LoadState') != 'loaded' or fields.get('ActiveState') not in ('inactive', 'failed'):
    raise RuntimeError('Run only with the installed Jetlink inference service stopped')


def enqueue_candidate(engine, cuda, device_feedback):
  """Keep source/destination device allocations distinct; preserve stream order."""
  for name, binding in engine.inputs.items():
    source = engine._state_host_sources.get(name, binding)
    if device_feedback and name in engine._state_host_sources:
      cuda.check(cuda._rt.cudaMemcpyAsync(binding.device_ptr, source.device_ptr, binding.nbytes,
                                         cuda._rt.cudaMemcpyKind.cudaMemcpyDeviceToDevice, engine.stream))
    else:
      cuda.memcpy_h2d_async(binding.device_ptr, source.host_ptr, binding.nbytes, engine.stream)
  if not engine.context.execute_async_v3(engine.stream):
    raise RuntimeError('execute_async_v3 failed')
  # Deliberately retain every D2H tensor and all StatefulQueues checks. Reducing
  # D2H needs a separate validated GPU nonfinite check; this experiment does not.
  for binding in engine.outputs.values():
    cuda.memcpy_d2h_async(binding.host_ptr, binding.device_ptr, binding.nbytes, engine.stream)


def reset_device_feedback(engine, cuda):
  """After CPU queue reset, zero the device *sources* used by the next graph."""
  for source in engine._state_host_sources.values():
    if np.any(source.host):
      raise ValueError('CPU state must be reset before resetting device feedback')
  for source in engine._state_host_sources.values():
    cuda.memcpy_h2d_async(source.device_ptr, source.host_ptr, source.nbytes, engine.stream)
  cuda.stream_sync(engine.stream)


def digest_outputs(values):
  return {name: hashlib.sha256(memoryview(value).cast('B')).hexdigest() for name, value in values.items()}


def digest_file(path):
  # Jetson's Python 3.10 predates hashlib.file_digest.
  digest = hashlib.sha256()
  with path.open('rb') as stream:
    while chunk := stream.read(1 << 20):
      digest.update(chunk)
  return digest.hexdigest()


def summarize(rows):
  a = np.asarray(rows)
  return {name: {'mean_ms': float(a[:, i].mean()), 'p99_ms': float(np.percentile(a[:, i], 99)),
                 'max_ms': float(a[:, i].max())}
          for i, name in enumerate(('input', 'engine_including_copies', 'feedback', 'total'))}


def compare(spec, engine_path, frames, rate):
  from jetlink.server.backends.trt.engine import TrtEngine
  from jetlink.server.backends.trt import cudart
  from jetlink.server.session import _check_shapes

  class CandidateEngine(TrtEngine):
    def _enqueue(self):
      enqueue_candidate(self, cudart, True)

  reports, reference = {}, []
  base_image = np.arange(spec.warped_nbytes, dtype=np.uint8).reshape(spec.warped_shape)
  packed = np.zeros(spec.packed_nelem, np.float32)
  offset = 0
  for name, size in zip(spec.packed_shapes, spec.packed_sizes, strict=True):
    packed[offset:offset + size] = np.arange(size, dtype=np.uint8) if name == 'nav_map' else .125
    offset += size

  for label, engine_type in (('host_before', TrtEngine), ('device_copy', CandidateEngine), ('host_after', TrtEngine)):
    require_stopped_service()
    engine = engine_type(str(engine_path))
    try:
      _check_shapes(engine, spec)
      engine.configure_state_feedback(spec.state_pairs)
      dest = {n: engine.host_input(n) for n in engine.inputs}
      queues = StatefulQueues(spec, dest)

      def reset(queues=queues, engine=engine, device_feedback=label == 'device_copy'):
        queues.reset()
        if device_feedback:
          reset_device_feedback(engine, cudart)

      queues.step_into(base_image, packed, dest)
      reset()
      warm_result = engine.warm()
      if engine.graph_exec is None:
        raise RuntimeError('CUDA graph not captured; cannot compare with deployed graph path')
      reset()
      rows = []
      for i in range(frames):
        if i % 25 == 0:
          require_stopped_service()
          reset()
        image = base_image + np.uint8(i % 256)
        t0 = time.perf_counter()
        queues.step_into(image, packed, dest)
        t1 = time.perf_counter()
        values = engine.run()
        t2 = time.perf_counter()
        queues.feedback(values)
        t3 = time.perf_counter()
        actual = digest_outputs(values)
        if label == 'host_before':
          reference.append(actual)
        elif actual != reference[i]:
          raise AssertionError(f'All-output bit parity failed: {label} frame {i}')
        if i >= 10:
          rows.append([(t1-t0)*1000, (t2-t1)*1000, (t3-t2)*1000, (t3-t0)*1000])
        time.sleep(max(0, t0 + 1 / rate - time.perf_counter()))

      # Reject nonfinite driving output AND each state output. Recovery must
      # match the original zero-history first frame, including device state.
      for name in spec.output_shapes:
        reset()
        queues.step_into(base_image, packed, dest)
        values = engine.run()
        values[name].flat[0] = np.nan
        try:
          queues.feedback(values)
        except FloatingPointError:
          pass
        else:
          raise AssertionError(f'Nonfinite output accepted: {name}')
        if any(np.any(a) for a in queues.state.values()):
          raise AssertionError('Failed output did not clear CPU history')
        reset()
        queues.step_into(base_image, packed, dest)
        values = engine.run()
        queues.feedback(values)
        if digest_outputs(values) != reference[0]:
          raise AssertionError(f'Device history survived reset: {name}')
      reports[label] = {'timings': summarize(rows), 'warm': warm_result,
                        'state_bytes': sum(dest[n].nbytes for n in spec.state_pairs),
                        'all_outputs_exact': True, 'nonfinite_reset_recovery': True}
      print(label, json.dumps(reports[label]), flush=True)
    finally:
      engine.close()
  return {'scope': 'isolated synthetic host inference, no camera/USB/vehicle output',
          'model_sha256': spec.sha256, 'frames_per_variant': frames, 'variants': reports}


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--engine', type=Path, required=True)
  parser.add_argument('--spec', type=Path, default=(ROOT or Path('/opt/carrot-jetlink/current')) /
                      'openpilot/selfdrive/modeld/jetlink/jetson_model.json')
  parser.add_argument('--frames', type=int, default=150)
  parser.add_argument('--rate', type=float, default=20)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  if not 50 <= args.frames <= 1200 or not 1 <= args.rate <= 40:
    parser.error('Use 50..1200 frames and a rate of 1..40 Hz')
  if args.output.exists() or not args.engine.is_file():
    parser.error('Output must be new and the existing engine must be a file')
  spec = ModelSpec.from_dict(json.loads(args.spec.read_text(encoding='utf-8-sig')))
  if not spec.stateful:
    parser.error('This comparison requires a stateful model')
  require_stopped_service()
  engine_sha256 = digest_file(args.engine)
  result = compare(spec, args.engine, args.frames, args.rate)
  result['engine_sha256'] = engine_sha256
  with args.output.open('x', encoding='utf-8') as out:
    json.dump(result, out, indent=2)


if __name__ == '__main__':
  main()
